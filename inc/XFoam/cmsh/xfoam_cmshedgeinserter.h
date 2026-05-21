#ifndef XFoam_CMshEdgeInserter_H_
#define XFoam_CMshEdgeInserter_H_

// 对标 cfMesh: meshLibrary/utilities/surfaceTools/meshSurfaceEdgeExtractor/
//             meshSurfaceEdgeCreateEdgeVertices.C
//
// 思想：boundary face 的每条 edge，若两端 mesh point 属于不同 sub-patch
// （closestSubPatchId 不同），说明这条 edge 跨过了一条 TpEdge（CAD 面间
// 的特征棱）。在 edge 中点投影到该 TpEdge（brep.closestFeature with
// FeatureKind::Edge / closestPointAndNormal 兜底），再把这个新 point 插
// 到所有引用该 mesh edge 的 face 顶点列表里——face 由 N 边变 N+1 边。
//
// 与 cfMesh 的差异：
//   * cfMesh 用 meshOctree.findNearestPointToEdge(edgePoints, patches) 拿
//     octree 上的最近 surface 点；我们走 brep（VBrep BVH / MBrep OCCT），
//     精度更好。
//   * cfMesh 用 pointRegions（mesh point → patchId）做端点 patch 查询；
//     我们直接 brep.closestSubPatchId 在 mesh point 上现查（一次 KDTree
//     query）。
//   * 不分 internal / boundary edge 单独存：直接 (faceIdx, slot) → edgeKey
//     去重；同一对 (minVid, maxVid) 只处理一次。
//
// 不动 cell / owner / neighbour / patches —— 只改 points[] 和 faces[].verts
// （face 多 1 个 vert）。配在 mapper + edge-snap + (optional) pinner 之后
// 比较合理，使端点已落到表面。

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <vector>

class XFoam_BrepBase;

class XFoam_API XFoam_CMshEdgeInserter
{
public:
	struct Params
	{
		/// 投影 / feature 搜索半径；≤0 自动 = cellSizeHint。
		XFoam_Scalar searchRadius  = 0;
		XFoam_Scalar cellSizeHint  = 0;

		/// 是否要求 closestFeature(p, R) 返回 Edge 才插入；否则 (投到表面)
		/// 也插入。默认 true（更接近 cfMesh：插的就是 TpEdge 点）。
		bool         requireEdgeFeature = true;

		bool         verbose       = false;
	};

	struct Stats
	{
		XFoam_Label nBoundaryEdges  = 0; ///< 总 boundary edge 数
		XFoam_Label nCrossPatch     = 0; ///< 端点 patchId 不同的 edge 数
		XFoam_Label nInserted       = 0; ///< 实际插入 new point 的 edge 数
		XFoam_Label nProjFail       = 0; ///< feature 投影失败（跳过）的 edge 数
		XFoam_Label nFacesGrown     = 0; ///< 因 insert 而 size++ 的 face 数
		XFoam_Label nNewPoints      = 0; ///< 新增 mesh point 数
		XFoam_Scalar maxProjDist    = 0; ///< 插入点 vs 原 edge 中点的最大距离
	};

	XFoam_CMshEdgeInserter(
		XFoam_CMshPolyMeshGen& pm,
		const XFoam_BrepBase&  brep,
		const Params&          p);

	Stats insert();

	/// 这次 insert() 新增的 mesh point 下标（pm.points 末尾连续段）。
	/// 后续 optimizer 可视情况一并 setFixedPoints（这些点已经落到 TpEdge
	/// 上，再被 Laplacian 拉走会丢特征）。
	const std::vector<int>& newPoints() const { return newPts_; }

private:
	XFoam_CMshPolyMeshGen& pm_;
	const XFoam_BrepBase&  brep_;
	Params                 p_;
	std::vector<int>       newPts_;
};

#endif // XFoam_CMshEdgeInserter_H_
