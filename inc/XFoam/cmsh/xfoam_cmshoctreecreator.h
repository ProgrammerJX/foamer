#ifndef XFoam_CMshOctreeCreator_H_
#define XFoam_CMshOctreeCreator_H_

// 对标 cfMesh: meshLibrary/utilities/octrees/meshOctree/meshOctreeCreator/
//              meshOctreeCreator.{H,C} 系列文件
//
// 在 myfoam 里负责把一个 BrepBase + refinement controls 转换成一棵
// XFoam_CMshOctree。原版 cfMesh 有 objectRefinements / patchRefinements /
// boundaryLayerCells 等多种 refinement source；MVP 只先支持：
//
//   * 表面加密（addSurfaceRefine）：与该 brep 相交的 leaf 加密到 level
//   * 区域加密（addRegionRefine）：与该 region overlaps 的 leaf 加密到 level
//   * 整体 baseLevel（params.maxCellSize → log2(rootSpan / maxCellSize)）
//
// 后续 phase 2.x 再补 objectRefinement (box/sphere/cone)、edge refinement、
// patch-name based refinement 等。

#include "XFoam/cmsh/xfoam_cmshobjrefine.h"
#include "XFoam/cmsh/xfoam_cmshoctree.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <memory>
#include <vector>

class XFoam_BrepBase;

/*---------------------------------------------------------------------------*\
                   Class XFoam_CMshOctreeCreator Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshOctreeCreator
{
public:
	struct SurfaceRefine
	{
		const XFoam_BrepBase* brep  = nullptr;
		int                   level = 0;
	};

	struct RegionRefine
	{
		XFoam_BoundBox box;
		int            level = 0;
	};

	struct Params
	{
		/// 全局最大 cell 边长。OctreeCreator 据此推 baseLevel：
		///   baseLevel = ceil(log2(maxRootSpan / maxCellSize))，再 clamp 到
		///   [0, maxLevel]。
		XFoam_Scalar maxCellSize = 1.0;

		/// hard cap on level；防止小 feature 反推出来的 surface refine level
		/// 跑飞（cfMesh 默认 20，这里 14 已绰绰有余 → 2^14 = 16384 per axis）。
		int maxLevel = 14;

		/// 是否把 root bbox 略放大一圈；surface 贴在 root 边缘时 leaf box 求交
		/// 容易漏触发。
		bool inflateRoot = true;

		/// inflateRoot 时按 root 对角线比例放大。0.05 = 5%。
		XFoam_Scalar rootInflate = static_cast<XFoam_Scalar>(0.05);

		/// 打开 fitFeatures：build() 会在 SurfaceRefine 之前根据
		/// brep.minFeatureLength() 自动提 sr.level（但不超过 sr.level +
		/// fitFeaturesMaxLevelBump，再 clamp 到 Params.maxLevel）。
		/// 目标 cellSize = minFeatureLength * fitFeaturesSafety。
		bool         fitFeatures             = false;
		XFoam_Scalar fitFeaturesSafety       = static_cast<XFoam_Scalar>(0.5);
		int          fitFeaturesMaxLevelBump = 2;

		/// fitFeatures 升级版：按每张 TopoDS_Face 自己的 bbox 反推 level
		/// （而非一刀切提全局 surfLevel）。对小 TpFace 多加密，对大 TpFace
		/// 保持原 level，避免全局 leaf 数爆炸。具体规则：
		///   bbox_min_side = min(subPatchBounds(s).span_x|y|z)
		///   wanted        = bbox_min_side * perFaceFitFeaturesSafety
		///   needed        = ceil(log2(maxRootSpan / wanted))
		///   targetLevel   = clamp(needed, surfLevel,
		///                          surfLevel + fitFeaturesMaxLevelBump, maxLevel)
		/// 然后调 refineToSurfacePerFace。需要 brep.subPatchBounds() 支持。
		bool         perFaceFitFeatures      = false;
		XFoam_Scalar perFaceFitFeaturesSafety = static_cast<XFoam_Scalar>(0.5);
	};

	/// rootBox 通常 = primary brep.bounds()；OctreeCreator 视 inflateRoot 决定
	/// 是否再放大。
	XFoam_CMshOctreeCreator(const XFoam_BoundBox& rootBox, const Params& p);

	XFoam_CMshOctreeCreator& addSurfaceRefine(const XFoam_BrepBase& s, int level);
	XFoam_CMshOctreeCreator& addRegionRefine(const XFoam_BoundBox& region, int level);

	/// 加入一个 objectRefinement（box/sphere/cone 等）。Creator 接管所有权。
	/// 与 RegionRefine 的差别：RegionRefine 只能 box；ObjectRefine 可以是任意
	/// XFoam_CMshObjRefine 子类，box 同样可用 XFoam_CMshBoxRefine 表达。
	XFoam_CMshOctreeCreator& addObjectRefine(std::unique_ptr<XFoam_CMshObjRefine> obj);

	/// surfs[0] 必须存在；它是 octree 的"主"几何 → 用来做 inside/outside 分类。
	/// 流程：
	///   1) inflate root bbox
	///   2) new Octree(primary, rootBox)
	///   3) refineUniform(baseLevel)
	///   4) 对每个 SurfaceRefine 调 refineToSurface
	///   5) 对每个 RegionRefine 调 refineRegion
	///   6) classifyLeaves
	/// 返回的 AutoPtr 由 caller 接管。
	XFoam_AutoPtr<XFoam_CMshOctree> build() const;

private:
	XFoam_BoundBox             rootBox_;
	Params                     p_;
	std::vector<SurfaceRefine>                         surfs_;
	std::vector<RegionRefine>                          regions_;
	std::vector<std::unique_ptr<XFoam_CMshObjRefine>>  objects_;
};

#endif // XFoam_CMshOctreeCreator_H_
