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
//   * cell zones / face zones / multi-region
//
// 支持的事：
//   * perFacePatches：boundary face 按 brep.closestSubPatchId(faceCenter) 拆
//     patch；每个 sub-patch 单独成一个 polyMesh boundary patch。
//   * locationInMesh：给定一个 3D 点，extract 只保留该点所在 face-连通群（用
//     octree.faceNeighbour BFS 走 Inside+Data 的 in-mesh leaf）。可干掉"外
//     壳"伪 Inside 群（如多层 surface 之间的间隙腔体）。

#include "XFoam/cmsh/xfoam_cmshoctree.h"
#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <string>

class XFoam_BrepBase;

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

		/// per-subPatch boundary patch 拆分。
		///   * false（默认）：所有 boundary face 都进 defaultPatchName patch
		///   * true：boundary face 按 brep.closestSubPatchId(faceCenter) 分桶；
		///     需要同时通过 ctor 传入 brep
		bool perFacePatches = false;

		/// 是否启用 locationInMesh BFS 连通群剔除
		bool useLocationInMesh = false;
		XFoam_Vector3D locationInMesh = XFoam_Vector3D(0, 0, 0);

		/// perFacePatches 下，是否为 brep 中未被任何 boundary face 命中的
		/// sub-patch 也输出一个 nFaces=0 的空 patch（防止小 TpFace 在网格太
		/// 粗时"消失"；与 snappyHexMesh 行为一致）。
		bool fillAllSubPatches = false;

		/// patch type 总缺省。已知 patch 名（"walls", "inlet", "outlet", ...）
		/// 可考虑后续做 name → type 映射；MVP 一律用 defaultPatchType。
	};

	/// 上次 extract() 的 sub-patch 覆盖情况。perFacePatches=true + brep!=nullptr
	/// 时填充；其它情况字段全 0/空。
	/// 用途：pipeline 的 coverAllFaces 自适应加密循环读 missingSubIds，对每
	/// 个未覆盖 TpFace 局部 refineRegion，再 extract 一次。
	struct Stats
	{
		XFoam_Label               nSubPatches    = 0;  ///< brep.nSubPatches()
		XFoam_Label               nCoveredSubs   = 0;  ///< 至少有 1 face 落到的 sub-patch 数
		std::vector<XFoam_Label>  missingSubIds;       ///< 未命中的 sub-patch id 列表
	};

	/// 构造：oct 是必需的 octree；brep 仅在 perFacePatches=true 时用，用来
	/// 查 closestSubPatchId。通常传 octree 主 brep。
	XFoam_CMshCartesianExtractor(
		XFoam_CMshOctree& oct,
		const Params& p,
		const XFoam_BrepBase* brep = nullptr);

	/// 主入口：clear out → 走完 vertex/face dedup → 填好 out 的 5 张表。
	/// 返回 false 若 out 是空网格（in-mesh leaf=0）。
	bool extract(XFoam_CMshPolyMeshGen& out);

	const Stats& stats() const { return stats_; }

private:
	XFoam_CMshOctree&             oct_;
	Params                        p_;
	const XFoam_BrepBase*         brep_ = nullptr;
	Stats                         stats_;
};

#endif // XFoam_CMshCartesianExtractor_H_
