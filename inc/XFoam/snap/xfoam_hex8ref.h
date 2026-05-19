#ifndef XFoam_Hex8Ref_H_
#define XFoam_Hex8Ref_H_

// 八叉树细化（Octree refinement）：在一张结构化 (Nx, Ny, Nz) 的 base hex 网格上，
// 为每个 base-cell 内部独立维护一棵 octree；叶子 (Leaf) 用
//   (ai, aj, ak, level, si, sj, sk)
// 唯一标识，其中 (si, sj, sk) ∈ [0, 2^level)。
//
// 对标 OpenFOAM-13 src/dynamicMesh/polyTopoChange/polyTopoChange/hexRef8。
// 关键差异（受 myfoam 极简定位影响）：
//   - 不持有 polyMesh，本类只管 *拓扑*（cell-cell adjacency + level）
//   - 不维护父/子指针，只存当前活跃 leaf 数组 + 懒重建的 leafMap
//   - 不实现 cellLevel/pointLevel 持久化；只暴露 Leaf.level
//   - 2:1 平衡只考虑 face-邻居（不考虑 edge / point 邻居），跟 hexRef8 的
//     consistentRefinement 选 nBufferLayers=1 等价
//   - 物理坐标 (world coords) 与 STL 判定全在外部用 predicate / 在调用方算出 (u,v,w)
//
// 用法（snappyHexMesh 调用方）：
//   XFoam_Hex8Ref oct(Nx, Ny, Nz, LEVEL_CAP);
//   oct.initBaseLeaves();
//   oct.refineByPredicate(targetLevel, [&](const Leaf& l) { /* l 与 STL 相交？ */ });
//   oct.balance21();
//   oct.cullByPredicate([&](const Leaf& l) { /* l 在 STL 外侧？ */ });
//   oct.assignCellIds();
//   for (auto& l : oct.leaves()) { l.corner[oc] = addPoint( worldOf(l, oc) ); }
//   for (auto& l : oct.leaves()) { ... 用 oct.resolveFaceNeighbor 决定 face 发法 ... }

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

/*---------------------------------------------------------------------------*\
                       Class XFoam_Hex8Ref Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_Hex8Ref
{
public:
	/// 八叉树叶子；对外是 plain struct，方便调用方直接读 / 写 corner / kept。
	/// 内部不维护父子指针 — 树形结构通过 leafMap 反查。
	struct Leaf
	{
		/// 所属 base-cell 在 (Nx, Ny, Nz) 中的 3D 索引。
		XFoam_Label ai;
		XFoam_Label aj;
		XFoam_Label ak;

		/// 细化级别：0 = 未细化的 base-cell；每深一级 (si,sj,sk) 数轴尺寸 ×2。
		XFoam_Label level;

		/// 在自身 level 下的 base-cell 内子位置；各 ∈ [0, 2^level)。
		XFoam_Label si;
		XFoam_Label sj;
		XFoam_Label sk;

		/// cull 阶段标记：true = 保留到最终网格。
		bool kept;

		/// 在 kept leaves 中的全局 cell id；未编号或不 kept 时为 -1。
		XFoam_Label cellId;

		/// 8 个角点在调用方全局点表里的 id；调用方在 generateCornerPoints 等价阶段写入。
		/// 角点顺序严格按 OpenFOAM hex 约定（leafCornerParam 对外暴露同一约定）。
		XFoam_Label corner[8];

		Leaf()
			: ai(0), aj(0), ak(0), level(0), si(0), sj(0), sk(0),
			  kept(true), cellId(-1)
		{
			for (int i = 0; i < 8; ++i) corner[i] = -1;
		}
	};

	/// face d 的邻居解析结果。
	enum class FaceNbrKind
	{
		OutOfGrid, ///< d 方向走出背景 (Nx,Ny,Nz) 网格 → 外边界 wall
		Same,      ///< 邻居恰为 same-level，单 quad 拼接
		Coarser,   ///< 邻居在更粗 level，单 quad；粗侧由对侧负责切 split-face
		Finer,     ///< 邻居在 self.level + 1，至少 1 个 fine leaf 存在；本侧需切 split-face
		None       ///< 邻居 base-cell 存在但 leafMap 找不到任何 leaf（拓扑错误，不该出现）
	};

	/// resolveFaceNeighbor 返回；fineLeafIdx 仅 Finer 模式下有效。
	struct FaceNbr
	{
		FaceNbrKind kind;
		XFoam_Label leafIdx;          ///< Same / Coarser 时邻居 leaf 在 leaves() 里的下标；否则 -1
		XFoam_Label fineLeafIdx[4];   ///< Finer 时 2×2 fine leaf 下标，未找到的为 -1

		FaceNbr() : kind(FaceNbrKind::OutOfGrid), leafIdx(-1)
		{
			for (int i = 0; i < 4; ++i) fineLeafIdx[i] = -1;
		}
	};

	/// 重构 phase 1 / phase 3 的谓词类型；接收 leaf 自身，返回 true 表示需要 subdivide / cull。
	typedef std::function<bool(const Leaf&)> LeafPredicate;

	/// 构造：仅记录背景网格分辨率与 level 上限（默认 4，便于把 (si,sj,sk) 塞进 4-bit 字段）。
	XFoam_Hex8Ref(XFoam_Label nxIn, XFoam_Label nyIn, XFoam_Label nzIn, XFoam_Label levelCapIn = 4);

	/// Phase 0：每个 base-cell 放 1 个 level=0 的 leaf；调用方应在 refineByPredicate 之前调一次。
	void initBaseLeaves();

	/// Phase 1：迭代细化。每一轮快照当前 leaves，对所有 level < targetLevel 且 pred(leaf) == true
	/// 的 leaf 调 subdivide()；直到一整轮没有新的 subdivide 或迭代上限。
	void refineByPredicate(XFoam_Label targetLevel, const LeafPredicate& pred);

	/// Phase 2：2:1 face-balance。对每个 leaf 查 6 个 face 邻居，若任一邻居 level > self.level + 1
	/// 就 subdivide 自己；迭代直至稳定。OpenFOAM 的 hexRef8::consistentRefinement 同义。
	void balance21();

	/// Phase 1.5：把"高 level 区域"沿 face 邻居方向膨胀 nLayers 圈。
	/// 每一轮：找出所有"有更粗 face 邻居"的 leaf，把这些粗邻居 subdivide 一级（把它们升到与
	/// 当前细侧同级）。等价 OpenFOAM 的 meshRefinement::extendMarkedCells + nBufferLayers > 1。
	///
	/// nLayers == 0 → no-op；nLayers == 1 即"在 balance21 之前先膨胀 1 圈"，恰好让 dict 里
	/// nCellsBetweenLevels = 2 落地（balance21 自己保证 1 cell intermediate，再 +1 cell buffer）。
	///
	/// 通常调用顺序：refineByPredicate → extendHighLevel(N-1) → balance21
	void extendHighLevel(int nLayers);

	/// Phase 3：按 predicate 标记 kept。pred(leaf) == true 表示该 leaf 应被切除（kept = false）。
	void cullByPredicate(const LeafPredicate& pred);

	/// Phase 4：为 kept leaves 依序分配 cellId（从 0 开始）；返回 nKept。
	XFoam_Label assignCellIds();

	// ----- 静态参数空间几何辅助 -----
	// 所有结果都是 base-cell A 内的归一化 (u, v, w) ∈ [0,1]^3；调用方需用 paramToWorld 等
	// 把它换算成物理坐标。这样 Hex8Ref 自身完全不知道背景 hex 的 8 个角点。

	/// leaf 的 8 个角点（OpenFOAM hex 顶点约定）在归一化 base-cell 坐标 [0,1]^3 中的位置。
	/// 约定：
	///   oc=0 (0,0,0) | oc=1 (1,0,0) | oc=2 (1,1,0) | oc=3 (0,1,0)
	///   oc=4 (0,0,1) | oc=5 (1,0,1) | oc=6 (1,1,1) | oc=7 (0,1,1)
	/// 注意 oc=2 是 (1,1,0)、oc=3 是 (0,1,0) — 这是 OF 的 hex 约定；bit-位编码会把 v2/v3、v6/v7
	/// 颠倒，再去 kHexFace[d] 取面会得到对角四边形（曾在 octree 早期实现里踩过这个坑）。
	static void leafCornerParam(
		const Leaf& l, int oc,
		XFoam_Scalar& u, XFoam_Scalar& v, XFoam_Scalar& w);

	/// leaf 的几何中心在归一化 base-cell 坐标 [0,1]^3 中的位置（用于 inside/outside 判定）。
	static void leafCentroidParam(
		const Leaf& l,
		XFoam_Scalar& u, XFoam_Scalar& v, XFoam_Scalar& w);

	/// 在 leaf l 的 face d 上，按 (rr, cc) ∈ {0..2}^2 在 level+1 分辨率下取 9 个 face 点之一。
	/// 4 个角点 (rr/cc ∈ {0,2})、4 个 edge-mid (恰有一个为 1)、1 个 face-center (rr=cc=1)。
	/// 用来在粗侧 leaf 切 split-face：调用方把 9 点全部 addPoint 后切成 4 个 sub-quad。
	static void faceSteinerParam(
		const Leaf& l, int d, int rr, int cc,
		XFoam_Scalar& u, XFoam_Scalar& v, XFoam_Scalar& w);

	// ----- 邻居查询 -----

	/// 查 leaf l 的 face d 邻居。Lazy 重建 leafMap（如果上次调过 subdivide / refine / balance）。
	/// 多次连续调用 O(1) 一次哈希查询；不要在每个 face 上重复 build。
	FaceNbr resolveFaceNeighbor(const Leaf& l, int d) const;

	// ----- 访问器 / 统计 -----

	XFoam_Label nx() const noexcept { return nx_; }
	XFoam_Label ny() const noexcept { return ny_; }
	XFoam_Label nz() const noexcept { return nz_; }
	XFoam_Label levelCap() const noexcept { return levelCap_; }

	XFoam_Label numLeaves() const noexcept
	{
		return static_cast<XFoam_Label>(leaves_.size());
	}
	const std::vector<Leaf>& leaves() const noexcept { return leaves_; }
	std::vector<Leaf>& leaves() noexcept { return leaves_; }

	XFoam_Label maxLevelReached() const;

	/// 把 [0..7] 各 level 的 leaf 计数填到 out[0..7]（>=8 的 level 被忽略，但本类 levelCap<=4）。
	void perLevelCounts(XFoam_Label out[8]) const;

private:
	XFoam_Label nx_;
	XFoam_Label ny_;
	XFoam_Label nz_;
	XFoam_Label levelCap_;

	std::vector<Leaf> leaves_;

	// 懒重建：subdivide / refine / balance / cull 都会标 dirty，下一次 resolveFaceNeighbor
	// 会触发一次 rebuildLeafMap()。assignCellIds 不动拓扑，不脏。
	mutable std::unordered_map<uint64_t, XFoam_Label> leafMap_;
	mutable bool leafMapDirty_;

	void rebuildLeafMap() const;
	void markLeafMapDirty() { leafMapDirty_ = true; }

	// 原地把 leaves_[idx] 拆成 8 个孩子：第 1 个就地替换，剩 7 个 push_back。父 leaf 的 corner /
	// cellId / kept 不继承（孩子重置）。levelCap 限制下静默 no-op。
	void subdivide(XFoam_Label idx);

	// 在 leaf 自身 level 下沿方向 d 走 1 格；返回邻居所在 base-cell + base-cell 内的 (ns_i, ns_j, ns_k)。
	// 跨 base-cell 会自动 wrap。若邻居 base-cell 越界返回 false。
	bool stepNeighborAtSameLevel(
		const Leaf& l, int d,
		XFoam_Label& aiOut, XFoam_Label& ajOut, XFoam_Label& akOut,
		XFoam_Label& nsiOut, XFoam_Label& nsjOut, XFoam_Label& nskOut) const;

	bool inGrid(XFoam_Label i, XFoam_Label j, XFoam_Label k) const noexcept
	{
		return i >= 0 && i < nx_ && j >= 0 && j < ny_ && k >= 0 && k < nz_;
	}

	// (ai, aj, ak, L, si, sj, sk) → uint64 key。布局假定 levelCap <= 4 →
	// si/sj/sk < 16；ai/aj/ak 各 16 bit。
	static uint64_t encodeLeafKey(
		XFoam_Label ai, XFoam_Label aj, XFoam_Label ak,
		XFoam_Label level,
		XFoam_Label si, XFoam_Label sj, XFoam_Label sk);
};

#endif
