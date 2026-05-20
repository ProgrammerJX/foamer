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

#include "XFoam/cmsh/xfoam_cmshmeshoptimizer.h"
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
		XFoam_CMshSurfaceEdgeExtractor::Stats edgeStats;
		XFoam_CMshMeshOptimizer::Stats        optimizerStats;
		XFoam_CMshRepatcher::Stats            repatchStats;
		double msOctree     = 0;
		double msExtract    = 0;
		double msMapper     = 0;
		double msEdge       = 0;
		double msOptimizer  = 0;
		double msRepatch    = 0;
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
