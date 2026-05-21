#ifndef XFoam_CMshFeaturePinner_H_
#define XFoam_CMshFeaturePinner_H_

// 对应 cfMesh 视角的 "force-snap to feature point/edge"：
//   * meshSurfaceEdgeExtractor 的 createBoundaryFaces 思路是把 patch 边界
//     上的 mesh edge 重新打到 corner / TpEdge 上；这里只做点端 pinning（B1）。
//   * 与原版 mesher 不同的是：我们走虚拓扑层（XFoam_BrepBase.featureVertex...）
//     而非 triSurfacePartitioner.corners；这样对 MBrep（OCCT 参数化）
//     得到 TopoDS_Vertex 的精确解析坐标，没有离散误差。
//
// 行为：
//   1) 遍历 brep.nFeatureVertices() 拿每个 TpVertex 的精确位置 v_i
//   2) 在 mesh boundary 点子集里找最近 mesh 点 p_j
//   3) 若 |v_i - p_j| ≤ pinRadius，把 p_j 整体钉到 v_i（覆盖之前的 mapper /
//      Laplacian / project 结果）。一个 mesh 点至多被一个 TpVertex 钉死；
//      若多个 TpVertex 抢同一个 mesh 点，最近的赢，其它跳过 + 记到 Stats。
//   4) Optional：当最近 mesh 点距离 > pinRadius，可以 insert 一个新 mesh
//      vertex（劈分最近 face）。MVP 不做插入，只 pin。
//
// 不动 face / owner / neighbour / patches，只改 points[]。配在 mapper 和
// edge-snap 之后、optimizer 之前最合理（pinning 是硬约束，不应被后续 smoothing
// 抹掉；optimizer 应能 detect 这些点并锁定）。

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <vector>

class XFoam_BrepBase;
class XFoam_CMshSurfaceEngine;

class XFoam_API XFoam_CMshFeaturePinner
{
public:
	struct Params
	{
		/// 最远 pin 距离；> 该距离的 TpVertex 视为 "mesh 太粗，没法 pin"，
		/// 不动；记到 Stats.nOutOfRange。默认 0 = 自动 = 2 × cellSizeHint。
		XFoam_Scalar pinRadius   = 0;
		XFoam_Scalar cellSizeHint = 0;

		bool         verbose     = false;
	};

	struct Stats
	{
		XFoam_Label nTpVerts         = 0; ///< brep.nFeatureVertices()
		XFoam_Label nPinned          = 0; ///< 成功钉住的数量
		XFoam_Label nOutOfRange      = 0; ///< 距离 > pinRadius 的数量
		XFoam_Label nConflictSkipped = 0; ///< 同 mesh 点被多个 TpVertex 抢，输家
		XFoam_Scalar maxPinShift     = 0; ///< 实际钉死前后位移的最大值
	};

	/// pm 必须已经 extract（boundary points 已建立）；brep 应已 buildFeatures()
	/// 让 nFeatureVertices() > 0（VBrep 必须，MBrep 自动）。
	XFoam_CMshFeaturePinner(
		XFoam_CMshPolyMeshGen& pm,
		const XFoam_BrepBase&  brep,
		const Params&          p);

	/// 复用外部 SurfaceEngine：跳过对 pm.faces 的 boundary point 扫描。
	XFoam_CMshFeaturePinner(
		XFoam_CMshPolyMeshGen&         pm,
		const XFoam_BrepBase&          brep,
		const Params&                  p,
		const XFoam_CMshSurfaceEngine& se);

	/// 跑一遍 pin pass，返回 stats。
	Stats pin();

	/// 被 pin 的 mesh point 下标集合（pin() 之后才有效）。后续 optimizer
	/// 应跳过这些点（hard constraint）。
	const std::vector<int>& pinnedPoints() const { return pinnedPoints_; }

private:
	XFoam_CMshPolyMeshGen& pm_;
	const XFoam_BrepBase&  brep_;
	Params                 p_;
	std::vector<int>       pinnedPoints_;
	std::vector<int>       bndPointsCache_; ///< 来自 SE 或 lazy 计算
	bool                   haveBndCache_ = false;
};

#endif // XFoam_CMshFeaturePinner_H_
