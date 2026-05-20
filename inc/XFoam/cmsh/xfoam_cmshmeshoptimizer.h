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
// 支持的事：
//   * qualityCheck：每轮所有 move 应用完后扫每个 boundary point 的 incident
//     face，若任一 face 的法向翻转（new·old < minFaceNormalDot）或面积过分
//     缩水（new/old < minFaceAreaRatio），rollback 该 point 到本轮起始位置；
//     统计 nRollback。
//
// 不做的事（MVP）：
//   * 内部点 / cell-center Laplacian
//   * 锥形 / 退化 cell 修复（rollback 是被动防御，不修复已坏的 cell）

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

		/// quality check + rollback
		bool         qualityCheck       = false;
		XFoam_Scalar minFaceNormalDot   = static_cast<XFoam_Scalar>(0.5);  ///< face 法向翻转门限
		XFoam_Scalar minFaceAreaRatio   = static_cast<XFoam_Scalar>(0.1);  ///< face 面积缩水门限
	};

	struct Stats
	{
		XFoam_Label   nMoved      = 0;
		XFoam_Scalar  avgMove     = 0;
		XFoam_Scalar  maxMove     = 0;
		XFoam_Label   nRollback   = 0;  ///< 因 quality 触发回滚的 boundary point 数
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
	std::vector<std::vector<int>> nbrs_;          ///< 与 bndPoints_ 一一对应；存的是 globalVID
	std::vector<std::vector<int>> incidentFaces_; ///< 与 bndPoints_ 一一对应；存的是 boundary faceIdx

	void buildBoundaryAdjacency();
	void reprojectOne(int vid);
	void faceNormalAndArea(int faceIdx,
	                       XFoam_Vector3D& outNormal,
	                       XFoam_Scalar&   outArea) const;
};

#endif // XFoam_CMshMeshOptimizer_H_
