#ifndef XFoam_RefinementParameters_H_
#define XFoam_RefinementParameters_H_

// 对标 OpenFOAM-13: src/mesh/snappyHexMesh/refinementParameters/refinementParameters.H
// 仅承载 snappyHexMeshDict.castellatedMeshControls 段的"标量/向量字段"，几何细化层级与
// surface/feature 相关项见 xfoam_refinementsurfaces.h；driver 见 xfoam_snappyhexmesh.h。
//
// 部分移植：
//   - locationInMesh（单点）+ locationsInMesh（多点，OF-13）：driver 用 per-surface inside/outside
//     bitmask 来定 region 归属，cell 若与任一 location 同 bitmask 则保留。
// 未移植：
//   - cellZones / faceZones（OF-13 引入，需要 polyTopoChange 风格的拓扑变更才能落地）
//   - allowFreeStandingZoneFaces 之外的细化策略
//   - meshRegion / planarAngle / 全部并行控制
// 仅承载字段、提供 readDict()，不持有任何算法。

#include "XFoam/utilities/xfoam_common.h"

#include <vector>

class XFoam_Dictionary;

/*---------------------------------------------------------------------------*\
                    Class XFoam_RefinementParameters Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_RefinementParameters
{
public:
	// castellatedMeshControls 的核心数字旋钮，缺省值取自 OF 默认。
	// 这些字段直接对应 OF 同名 dict key，便于将来抄实现时一对一映射。
	XFoam_Label maxLocalCells = 1000000;
	XFoam_Label maxGlobalCells = 2000000;
	XFoam_Label minRefinementCells = 0;
	XFoam_Label nCellsBetweenLevels = 1;
	XFoam_Scalar resolveFeatureAngle = 30;
	XFoam_Scalar maxLoadUnbalance = 0;
	bool allowFreeStandingZoneFaces = true;
	// 多 region 锚点列表：dict key `locationsInMesh ((x y z) (x y z) ...)`；
	// 兼容 OF-13 之前的 `locationInMesh (x y z)`，readDict 会把它视为 size==1 的 list。
	// 至少 1 点才能开始 snappyHexMesh；hasLocationInMesh() 即 !locationsInMesh.empty()。
	std::vector<XFoam_Vector3D> locationsInMesh;

	XFoam_RefinementParameters() = default;
	explicit XFoam_RefinementParameters(const XFoam_Dictionary& castellatedDict);

	/// 从 castellatedMeshControls 子字典读入字段；缺省字段保持构造默认值。
	/// 命名规范: foam_code.md
	/// 移植规范: foam_code.md
	/// \return true 若至少识别到一个已知 key；false 若 dict 完全为空（仍按缺省值使用）。
	bool readDict(const XFoam_Dictionary& castellatedDict);

	/// 是否至少配置了一个 location（对应旧版 hasLocationInMesh）。
	bool hasLocationInMesh() const { return !locationsInMesh.empty(); }

	/// 首个 location；用于旧 sample / 测试中“当成单点”看的兼容路径。
	/// 在 hasLocationInMesh()==false 时返回零向量，不应被调用。
	const XFoam_Vector3D& locationInMesh() const
	{
		static const XFoam_Vector3D zero(0, 0, 0);
		return locationsInMesh.empty() ? zero : locationsInMesh.front();
	}
};

#endif
