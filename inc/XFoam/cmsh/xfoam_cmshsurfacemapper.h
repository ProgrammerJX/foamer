#ifndef XFoam_CMshSurfaceMapper_H_
#define XFoam_CMshSurfaceMapper_H_

// 对标 cfMesh: meshLibrary/utilities/surfaceTools/meshSurfaceMapper/
//              meshSurfaceMapper.{H,C}
//
// 把 XFoam_CMshPolyMeshGen 的"boundary point"投到 BrepBase 表面：
//   1) 扫 boundary faces，收 unique boundary vertex ids
//   2) 对每个 boundary point，遍历所有 BrepBase 找最近一个，
//      调用 closestPointAndNormal() 取投影点
//   3) points[i] = (1 - relax) * old + relax * projected
//   4) 多轮迭代（cfMesh 默认 1 轮就够；snap 阶段会循环调用以做 smoothing）
//
// 不做的事（MVP）：
//   * laplace smoothing / 调和投影（cfMesh 真正的 mapper 还会带 smoothing）
//   * featureEdge / featureVertex 专用投影（→ phase c）
//   * patch 拆分（默认还是 1 个 walls patch；将来按 closestSubPatchId 分）

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <vector>

/*---------------------------------------------------------------------------*\
                  Class XFoam_CMshSurfaceMapper Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshSurfaceMapper
{
public:
	struct Params
	{
		int          nIterations = 1;     ///< 投影迭代次数
		XFoam_Scalar relaxFactor = 1.0;   ///< 每轮位移系数 (1 = 一次到位)
		XFoam_Scalar maxDist     = 1e30;  ///< 若到表面距离 > 此，跳过此点
		bool         verbose     = false; ///< 打印每轮 moved / max-dist 统计
	};

	XFoam_CMshSurfaceMapper(
		XFoam_CMshPolyMeshGen& pm,
		const std::vector<const XFoam_BrepBase*>& breps,
		const Params& p);

	/// 主入口：原地修改 pm.points。返回被实际移动的点数。
	XFoam_Label mapToSurface();

	/// 仅识别 boundary 点，不投。useful for c (edge extractor) 后续做 mask。
	const std::vector<int>& boundaryPointIds() const { return bndPoints_; }

private:
	XFoam_CMshPolyMeshGen&                    pm_;
	std::vector<const XFoam_BrepBase*>        breps_;
	Params                                    p_;
	std::vector<int>                          bndPoints_;

	void collectBoundaryPoints();
};

#endif // XFoam_CMshSurfaceMapper_H_
