#ifndef XFoam_CMshSurfaceEdgeExtractor_H_
#define XFoam_CMshSurfaceEdgeExtractor_H_

// 对标 cfMesh: meshLibrary/utilities/surfaceTools/meshSurfaceEdgeExtractor/
//              meshSurfaceEdgeExtractor.{H,C}
//
// 在 SurfaceMapper 把 boundary point 投到 surface 之后做：
//   * 对每个 boundary point，在 searchRadius 内查 BrepBase.closestFeature()
//   * FeatureKind::Vertex → 是 corner 候选；snapCorners 打开则吸过去
//   * FeatureKind::Edge   → 是 sharp edge 候选；snapEdges 打开则投到该 edge
//   * 选择跨 breps 中距离最近的那次 snap
//
// 适合在 Mapper 之后单独跑一遍，让 boundary 在尖角 / 棱线处贴合更利索；
// 也对应 snappy 的 "feature snap" 阶段（隐式特征捕捉）。
//
// 限制（MVP）：
//   * 没有 cfMesh 的 "vertex-info" 分类，所有 boundary point 都当 feature
//     候选；落在平面区域的点 closestFeature 会因 searchRadius 不够而 None，
//     自然不被动；尖角附近会被吸到 corner，符合直觉。
//   * 没有迭代 + smoothing；一遍过。

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <vector>

/*---------------------------------------------------------------------------*\
              Class XFoam_CMshSurfaceEdgeExtractor Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshSurfaceEdgeExtractor
{
public:
	struct Params
	{
		/// closestFeature 查询半径；≤ 0 视为自动 = 0.5 * 启发式 cellSize
		/// （由 caller 通过 setCellSizeHint() 提供）。
		XFoam_Scalar searchRadius   = 0;

		/// snap 距离上限，避免极端情况下把点拖得太远
		XFoam_Scalar maxSnapDist    = 1e30;

		bool snapCorners = true;     ///< FeatureKind::Vertex
		bool snapEdges   = true;     ///< FeatureKind::Edge
		bool verbose     = false;

		/// 启发式 cell size，仅用于 searchRadius<=0 时自动估算
		XFoam_Scalar cellSizeHint   = 0;
	};

	struct Stats
	{
		XFoam_Label nCornerSnap = 0;
		XFoam_Label nEdgeSnap   = 0;
		XFoam_Scalar maxSnapDist = 0;
	};

	XFoam_CMshSurfaceEdgeExtractor(
		XFoam_CMshPolyMeshGen& pm,
		const std::vector<const XFoam_BrepBase*>& breps,
		const std::vector<int>& bndPointIds,
		const Params& p);

	/// 一遍 snap，原地改 pm.points。
	Stats snap();

private:
	XFoam_CMshPolyMeshGen&                    pm_;
	std::vector<const XFoam_BrepBase*>        breps_;
	std::vector<int>                          bndPoints_;
	Params                                    p_;
};

#endif // XFoam_CMshSurfaceEdgeExtractor_H_
