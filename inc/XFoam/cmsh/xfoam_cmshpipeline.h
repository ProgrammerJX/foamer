#ifndef XFoam_CMshPipeline_H_
#define XFoam_CMshPipeline_H_

// cmsh 一站式 facade。把以下几个阶段串到一次 run() 里，外部调用者只需
//   * 提供 XFoam_BrepBase（已 buildFeatures）
//   * 提供 XFoam_CMshPolyMeshGen 接收最终网格（不落盘；可选事后再 writeToDir）
// 即可拿到一张可用 polyMesh。整条管线 in-memory，没有 OpenFOAM 那种中间
// 文件往返。
//
// 阶段顺序：
//   1) XFoam_CMshOctreeCreator     →  XFoam_CMshOctree
//   2) XFoam_CMshCartesianExtractor → polyMesh hex 框架（含 perFacePatches /
//                                     locationInMesh BFS）
//   3) XFoam_CMshSurfaceMapper（可选）  → boundary point → 最近 brep 表面
//   4) XFoam_CMshSurfaceEdgeExtractor（可选） → corner / edge 吸附
//   5) XFoam_CMshMeshOptimizer（可选）  → boundary Laplacian + reproject +
//                                         quality rollback
//
// 与单独 new 各模块直接调的等价：facade 仅做参数转发 + 顺序串接 + 计时统计；
// 没有任何隐藏行为。

#include "XFoam/cmsh/xfoam_cmshedgeinserter.h"
#include "XFoam/cmsh/xfoam_cmshfeaturepinner.h"
#include "XFoam/cmsh/xfoam_cmshmeshoptimizer.h"
#include "XFoam/cmsh/xfoam_cmshmeshuntangler.h"
#include "XFoam/cmsh/xfoam_cmshobjrefine.h"
#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/cmsh/xfoam_cmshrepatcher.h"
#include "XFoam/cmsh/xfoam_cmshsurfaceedgeextractor.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <memory>
#include <string>
#include <vector>

class XFoam_BoundBox;
class XFoam_BrepBase;

/*---------------------------------------------------------------------------*\
                    Class XFoam_CMshPipeline Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshPipeline
{
public:
	struct Params
	{
		// ---- octree ----
		XFoam_Scalar maxCellSize             = 1.0;
		int          surfLevel               = 4;
		int          maxLevel                = 14;
		bool         inflateRoot             = true;
		XFoam_Scalar rootInflate             = static_cast<XFoam_Scalar>(0.05);
		bool         fitFeatures             = false;
		XFoam_Scalar fitFeaturesSafety       = static_cast<XFoam_Scalar>(0.5);
		int          fitFeaturesMaxLevelBump = 2;

		/// per-TopoDS_Face fitFeatures：按每张 face 自己的 bbox 反推加密 level，
		/// 只把小 TpFace 加密，避免全局 leaf 爆炸。fitFeaturesMaxLevelBump 同
		/// 时充当 cap。开此项时通常关掉 fitFeatures（避免双重 bump）。
		bool         perFaceFitFeatures       = false;
		XFoam_Scalar perFaceFitFeaturesSafety = static_cast<XFoam_Scalar>(0.5);

		/// 真 · per-leaf 局部加密：只对靠近 TpEdge / TpVertex 的 leaf 加密，
		/// flat 远离 feature 的 leaf 保持 surfLevel。perFace 只能给整张 face
		/// 一个 level；这里能在单张大 face 的尖角附近 bump，平坦处不动。
		bool         localFeatureRefine       = false;
		XFoam_Scalar localFeatureSafety       = static_cast<XFoam_Scalar>(0.5);
		XFoam_Scalar localFeatureSearchMul    = static_cast<XFoam_Scalar>(2.0);

		/// coverAllFaces：自适应加密循环，对标 cfMesh::automaticRefinement 思路：
		/// extract → 看哪些 TpFace 没被覆盖 → 对每个未覆盖 face 的 bbox 局部
		/// refineRegion 到当前 leaf level+1 → 再 extract，直到全覆盖或顶到
		/// maxLevel / 达到 coverAllFacesMaxRounds。突破 perFaceFitFeaturesBump
		/// 限制（只针对实际未覆盖的 sliver 面），不会全局炸 leaf。
		bool         coverAllFaces            = false;
		int          coverAllFacesMaxRounds   = 3;

		// ---- extractor ----
		bool         keepInside              = true;
		bool         keepData                = true;
		bool         keepOutside             = false;
		bool         perFacePatches          = false;
		bool         useLocationInMesh       = false;
		XFoam_Vector3D locationInMesh        = XFoam_Vector3D(0, 0, 0);
		std::string  defaultPatchName        = "walls";
		std::string  defaultPatchType        = "wall";

		/// perFacePatches 下，是否输出未命中 sub-patch 的空 patch（防 TpFace 丢）
		/// 默认 true：与 snappyHexMesh 行为一致，保所有 TopoDS_Face id 在
		/// boundary 列表里 referenceable，避免下游 BC 配置丢面。
		bool         fillAllSubPatches       = true;

		/// perFacePatches 下，是否在 mapper（+ edgeSnap）之后用 post-mapped
		/// centroid 重写 pm.patches（重新算 closestSubPatchId），把 extract 阶段
		/// 因 face 还在整数 grid 上而错分的 TpFace 归属修正过来。
		bool         repatchAfterMap         = true;

		// ---- mapper ----
		bool         enableMapper            = true;
		int          mapIter                 = 1;
		XFoam_Scalar mapRelax                = 1.0;
		XFoam_Scalar mapMaxDist              = 1e30;

		// ---- edge snap ----
		bool         enableEdgeSnap          = true;
		XFoam_Scalar featureSearchRadius     = 0;     ///< 0 → 自动 = cellSize/2
		bool         snapCorners             = true;
		bool         snapEdges               = true;

		// ---- feature pinner (B1: TpVertex 钉死) ----
		/// 启用后在 mapper + edge-snap 之后、optimizer 之前跑一遍 pinner：
		/// 把每个 TpVertex 钉到最近的 mesh boundary 点。被钉死的点会通过
		/// setFixedPoints 传给后续 optimizer 锁住。
		bool         enableFeaturePinner     = false;
		XFoam_Scalar pinRadius               = 0;   ///< 0 = 自动 = 2*cellSize

		// ---- patchRefine：name -> level，覆盖到 perFaceFitFeatures 之上 ----
		/// 每条规则会展开成 brep.subPatchIdsByName(name) 列表内所有 sub-patch
		/// 的 perFaceLevel 取 max。常用 VBrep STL solid name / MBrep 自动
		/// face_<id> 名。
		std::vector<std::pair<std::string, int>> patchRefine;

		// ---- untangler (修复翻转 boundary face) ----
		/// optimizer 之后跑：检测翻转/退化 boundary face，对受影响 boundary
		/// point 试若干候选位置，取使最差质量最大的那个。pinned + inserted
		/// 点保持不动。
		bool         enableUntangler         = false;
		int          untanglerIter           = 3;
		XFoam_Scalar untanglerTangleDot      = static_cast<XFoam_Scalar>(0.0);

		// ---- edge inserter (B2: TpEdge densify) ----
		/// 启用后扫所有 boundary face edge，若两端 mesh 点属于不同 sub-patch，
		/// 在 edge 中点投影到 TpEdge / surface，把新 point 插进所有相关 face
		/// 顶点列表里。face 由 N 边变 N+1 边（可能多次 +1）。新点也加入
		/// optimizer fixedPoints。
		bool         enableEdgeInsert        = false;
		XFoam_Scalar edgeInsertRadius        = 0;   ///< 0 = 自动 = cellSize
		bool         edgeInsertRequireFeature = true; ///< false → 落表面也可

		// ---- optimizer ----
		bool         enableOptimizer         = false;
		int          optIter                 = 3;
		XFoam_Scalar optRelax                = static_cast<XFoam_Scalar>(0.5);
		bool         optReproject            = true;
		bool         optSnapFeatures         = false; ///< 通常跟 enableEdgeSnap 一致
		bool         optQuality              = false;
		XFoam_Scalar optMinFaceNormalDot     = static_cast<XFoam_Scalar>(0.5);
		XFoam_Scalar optMinFaceAreaRatio     = static_cast<XFoam_Scalar>(0.1);

		bool         verbose                 = true;
	};

	struct Stats
	{
		XFoam_Label nLeaves         = 0;
		XFoam_Label nCells          = 0;
		XFoam_Label nPoints         = 0;
		XFoam_Label nFaces          = 0;
		XFoam_Label nInternalFaces  = 0;
		XFoam_Label nPatches        = 0;
		XFoam_Label mapperMoved     = 0;
		XFoam_Label nCoveredSubs    = 0;  ///< extractor 完成时被命中的 sub-patch
		XFoam_Label nMissingSubs    = 0;  ///< 仍未被命中（geometry 丢）
		int         coverRounds     = 0;  ///< coverAllFaces 实际跑了几轮
		XFoam_CMshSurfaceEdgeExtractor::Stats edgeStats;
		XFoam_CMshFeaturePinner::Stats        pinStats;
		XFoam_CMshEdgeInserter::Stats         edgeInsertStats;
		XFoam_CMshMeshOptimizer::Stats        optimizerStats;
		XFoam_CMshMeshUntangler::Stats        untanglerStats;
		XFoam_CMshRepatcher::Stats            repatchStats;
		double msOctree         = 0;
		double msExtract        = 0;
		double msMapper         = 0;
		double msEdge           = 0;
		double msPin            = 0;
		double msEdgeInsert     = 0;
		double msOptimizer      = 0;
		double msUntangler      = 0;
		double msRepatch        = 0;
		double msCoverLoop      = 0;
	};

	explicit XFoam_CMshPipeline(const Params& p);

	XFoam_CMshPipeline& addBoxRefine(
		const XFoam_BoundBox& box, int level, const std::string& name = "box");
	XFoam_CMshPipeline& addSphereRefine(
		const XFoam_Vector3D& centre, XFoam_Scalar radius, int level,
		const std::string& name = "sphere");
	XFoam_CMshPipeline& addConeRefine(
		const XFoam_Vector3D& a, const XFoam_Vector3D& b,
		XFoam_Scalar radiusA, XFoam_Scalar radiusB, int level,
		const std::string& name = "cone");
	XFoam_CMshPipeline& addObjectRefine(std::unique_ptr<XFoam_CMshObjRefine> obj);

	/// 主入口。brep 需已 buildFeatures()；pm 输出。
	Stats run(XFoam_BrepBase& brep, XFoam_CMshPolyMeshGen& pm);

private:
	Params                                              p_;
	std::vector<std::unique_ptr<XFoam_CMshObjRefine>>   objects_;
};

#endif // XFoam_CMshPipeline_H_
