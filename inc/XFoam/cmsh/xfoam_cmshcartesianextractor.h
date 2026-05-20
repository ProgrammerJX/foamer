#ifndef XFoam_CMshCartesianExtractor_H_
#define XFoam_CMshCartesianExtractor_H_

// 对标 cfMesh: meshLibrary/cartesianMesh/cartesianMeshExtractor/
//              cartesianMeshExtractor.{H,C}
//
// 把 XFoam_CMshOctree 的 leaves 直接抽成 XFoam_CMshPolyMeshGen（即 polyMesh
// 5 件套）。语义对齐 OpenFOAM polyMesh 规范：
//   * 仅保留"in-mesh" leaves（默认 type ∈ {Inside, Data}；若给 locationInMesh，
//     则保留与该 location 同侧的连通群）。
//   * 每个保留 leaf → 1 个 hex cell；6 张 face；level-mix 边界自动按 2:1
//     balance 切成多个 sub-quad（一面变 4 张），coarse 侧 cell 变多面体（≥ 9 张面）。
//   * Internal vs boundary：面只被 1 个 cell 引用 → boundary（暂全归一个 patch）；
//     2 个 cell 引用 → internal。
//
// 不做的事（MVP 阶段）：
//   * locationInMesh（默认就是 type==Inside；不做 BFS 连通群剔除）
//   * cell zones / face zones / multi-region
//
// 支持的事：
//   * perFacePatches：boundary face 按 brep.closestSubPatchId(faceCenter) 拆
//     patch；每个 sub-patch 单独成一个 polyMesh boundary patch。multi-brep
//     时 patch 名格式 "<brepName>_<subPatchName>"。

#include "XFoam/cmsh/xfoam_cmshoctree.h"
#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <string>
#include <vector>

/*---------------------------------------------------------------------------*\
                Class XFoam_CMshCartesianExtractor Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshCartesianExtractor
{
public:
	struct Params
	{
		/// 是否保留 Inside 类 leaf（在 surface 内部的）
		bool keepInside = true;

		/// 是否保留 Data 类 leaf（与 surface 相交的，"贴皮"层）
		bool keepData   = true;

		/// 是否保留 Outside（一般不要；调试 / 包壳法时可开）
		bool keepOutside = false;

		/// 给所有 boundary face 用的 patch name / type。
		std::string defaultPatchName = "walls";
		std::string defaultPatchType = "wall";

		/// 是否在 extract() 内部自动调 octree.balance21()。MVP 默认 true 以保证
		/// face splitting 假设成立（任意相邻 leaf level 差 ≤ 1）。
		bool autoBalance = true;

		/// per-surface / per-subPatch boundary patch 拆分。
		///   * false（默认）：所有 boundary face 都进 defaultPatchName patch
		///   * true：boundary face 按 brep.closestSubPatchId(faceCenter) 分桶；
		///     需要同时通过 ctor 传入 breps 列表
		bool perFacePatches = false;

		/// patch type 总缺省。已知 patch 名（"walls", "inlet", "outlet", ...）
		/// 可考虑后续做 name → type 映射；MVP 一律用 defaultPatchType。
	};

	/// 构造：oct 是必需的 octree；breps 仅在 perFacePatches=true 时有效，用来
	/// 查 closestSubPatchId。breps[0] 通常 = octree 主 brep；多 brep 时 patch
	/// 名格式 "<brepName_or_idx>_<subPatchName>"。
	XFoam_CMshCartesianExtractor(
		XFoam_CMshOctree& oct,
		const Params& p,
		std::vector<const XFoam_BrepBase*> breps = {},
		std::vector<std::string> brepNames = {});

	/// 主入口：clear out → 走完 vertex/face dedup → 填好 out 的 5 张表。
	/// 返回 false 若 out 是空网格（in-mesh leaf=0）。
	bool extract(XFoam_CMshPolyMeshGen& out);

private:
	XFoam_CMshOctree&                    oct_;
	Params                               p_;
	std::vector<const XFoam_BrepBase*>   breps_;
	std::vector<std::string>             brepNames_;
};

#endif // XFoam_CMshCartesianExtractor_H_
