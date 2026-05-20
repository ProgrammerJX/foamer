#ifndef XFoam_CMshMeshOptimizer_H_
#define XFoam_CMshMeshOptimizer_H_

// 对标 cfMesh: meshLibrary/utilities/smoothers/geometry/meshOptimizer/
//              meshOptimizer.{H,C}（boundary smoothing 子集 + cfMesh
//              surfaceOptimizer 的 vertex smoother 思路）
//
// MVP 做的事：
//   1) 扫所有 boundary face，建 boundary-point → boundary-point 邻接（同面共点）
//   2) 多轮：对每个 boundary point 取 (1-r) * self + r * mean(neighbours)
//   3) 每轮后 re-project 到最近 brep 表面（防止滑离表面）
//   4) 可选每轮做一次 feature snap（吸回 corner / edge，防止 Laplacian 抹平
//      尖角）
//
// 不做的事（MVP）：
//   * 内部点 / cell-center Laplacian
//   * 质量门限触发回滚（cfMesh 真正的 optimizer 会算 skewness / volume ratio
//     回滚坏 vertex 移动）
//   * 锥形 / 退化 cell 修复

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <vector>

class XFoam_BrepBase;

/*---------------------------------------------------------------------------*\
                  Class XFoam_CMshMeshOptimizer Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshMeshOptimizer
{
public:
	struct Params
	{
		int          nIterations = 3;     ///< Laplacian + reproject 的轮数
		XFoam_Scalar relaxFactor = 0.5;   ///< 每轮位移系数，0=不动 / 1=完全 Laplacian
		bool         reproject   = true;  ///< 每轮 Laplacian 后再 re-project 到表面
		bool         snapFeatures = false; ///< 每轮再吸一次 feature edge / corner
		XFoam_Scalar featureSearchRadius = 0; ///< feature snap 半径；≤0 → 用 cellSizeHint*0.5
		XFoam_Scalar cellSizeHint = 0;   ///< 用于自动 featureSearchRadius
		bool         verbose     = false;
	};

	struct Stats
	{
		XFoam_Label   nMoved      = 0;
		XFoam_Scalar  avgMove     = 0;
		XFoam_Scalar  maxMove     = 0;
	};

	XFoam_CMshMeshOptimizer(
		XFoam_CMshPolyMeshGen& pm,
		const XFoam_BrepBase& brep,
		const Params& p);

	/// 原地修改 pm.points；返回最末轮 stats。
	Stats optimize();

	const std::vector<int>& boundaryPointIds() const { return bndPoints_; }

private:
	XFoam_CMshPolyMeshGen&        pm_;
	const XFoam_BrepBase&         brep_;
	Params                        p_;
	std::vector<int>              bndPoints_;
	std::vector<std::vector<int>> nbrs_;   ///< 与 bndPoints_ 一一对应；存的是 globalVID

	void buildBoundaryAdjacency();
	void reprojectOne(int vid);
};

#endif // XFoam_CMshMeshOptimizer_H_
