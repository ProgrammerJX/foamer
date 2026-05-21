#ifndef XFoam_CMshMeshUntangler_H_
#define XFoam_CMshMeshUntangler_H_

// 对应 cfMesh: meshLibrary/utilities/smoothers/geometry/meshOptimizer/
//   tetMeshOptimisation/advancedSmoothers/meshUntangler/
//
// 简化版（MVP）：cfMesh 的 untangler 走"vertex 周围所有 face 的 half-space
// 交（feasible region）"找 centroid 保证无翻转；我们 MVP 走更直接的方案：
//
//   1) 用 SurfaceEngine 算每个 boundary face 的 (centre, normal, area)
//   2) 算每个 boundary face 的"邻居平均法向" navg = avg of faceFaces normals
//   3) 标 tangled face：area <= eps OR (normal · navg < tangleDot)
//   4) tangled face 的所有顶点 → tangled point 集合
//   5) 每个 tangled point 试若干候选位置，选 maximise 该点 incident face
//      最差质量的那个；若全都不优于当前，保留原位置
//      候选位置：
//        a) 该 point 的 nbr 平均（Laplacian target）
//        b) (a) 再投回 brep 表面（如果 brep != 空）
//        c) incident face centroid 平均
//        d) "good" nbr 平均（排除位于 tangled face 上的 nbr）
//   6) 多轮迭代，每轮 SurfaceEngine 不重建（只更新 face geom + 局部重算）
//
// 与 cfMesh 差异：
//   * 不构造 partTetMeshSimplex / 不做 half-space 交 → 不能保证 100% 修
//     好（cfMesh 是 provable 的）；但 MVP 工程上能修绝大多数 Laplacian /
//     project / pin 后造成的 sliver / mild inversion；剩余的会在 stats
//     的 nStillTangled 报告，留给用户检视。
//   * 只针对 boundary face；体内 cell 翻转留给 phase 3b（quality-aware
//     volume optimizer）。
//
// 推荐用法：mapper / edge-snap / pinner / edge-insert / Laplacian
// 全部跑完后，最后跑一遍 untangler 兜底；同时用 setFixedPoints 保护
// pinned + inserted 点。

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <vector>

class XFoam_BrepBase;
class XFoam_CMshSurfaceEngine;

class XFoam_API XFoam_CMshMeshUntangler
{
public:
	struct Params
	{
		int          nIterations    = 3;
		/// face area <= eps * cellSizeHint^2 视为退化
		XFoam_Scalar areaEps        = static_cast<XFoam_Scalar>(1e-8);
		/// face normal · neighbour-average normal 小于该值视为翻转
		XFoam_Scalar tangleDot      = static_cast<XFoam_Scalar>(0.0);
		/// 重投影到 brep 表面（如果 brep != 空）；候选 (b) 用
		bool         reproject      = true;
		XFoam_Scalar cellSizeHint   = 0;

		bool         verbose        = false;
	};

	struct Stats
	{
		XFoam_Label nFacesInitial   = 0;
		XFoam_Label nFacesTangled0  = 0; ///< 第 0 轮 tangled face 数
		XFoam_Label nFacesTangledN  = 0; ///< 最后一轮剩余 tangled face 数
		XFoam_Label nPointsTouched  = 0;
		XFoam_Label nPointsImproved = 0;
		XFoam_Scalar maxMove        = 0;
	};

	/// 单独构造（自己 build SurfaceEngine 副本）。便于直接调用测试。
	XFoam_CMshMeshUntangler(
		XFoam_CMshPolyMeshGen& pm,
		const XFoam_BrepBase*  brep,
		const Params&          p);

	/// 复用 pipeline 的 SurfaceEngine。se 寿命需覆盖 untangle() 调用。
	XFoam_CMshMeshUntangler(
		XFoam_CMshPolyMeshGen&         pm,
		const XFoam_BrepBase*          brep,
		const Params&                  p,
		const XFoam_CMshSurfaceEngine& se);

	/// 设固定点（pin + insert 产物），untangler 不动它们。
	void setFixedPoints(std::vector<int> ids);

	Stats untangle();

private:
	XFoam_CMshPolyMeshGen&                   pm_;
	const XFoam_BrepBase*                    brep_ = nullptr;
	Params                                   p_;
	const XFoam_CMshSurfaceEngine*           seExt_ = nullptr;
	std::vector<char>                        isFixed_;
};

#endif // XFoam_CMshMeshUntangler_H_
