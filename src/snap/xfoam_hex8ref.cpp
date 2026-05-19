#include "XFoam/snap/xfoam_hex8ref.h"

#include <algorithm>
#include <cassert>

namespace
{
// OpenFOAM hex 顶点约定下，oc ∈ [0..7] 对应的 (u, v, w) ∈ {0,1}^3。
// 与 leafCornerParam 的 doxygen 注释一致。曾经踩坑：用 bit 位 ((oc&1)/(oc&2)/(oc&4))
// 编码会把 v2/v3、v6/v7 颠倒，再走 kHexFace[d] 时取到对角顶点。
const int kOfU[8] = {0, 1, 1, 0, 0, 1, 1, 0};
const int kOfV[8] = {0, 0, 1, 1, 0, 0, 1, 1};
const int kOfW[8] = {0, 0, 0, 0, 1, 1, 1, 1};

// face 方向上 (axD, axA, axB)：axD 是 face 法向所在 axis（0=x, 1=y, 2=z），
// axA / axB 是面内两个自由 axis。与 snappyhexmesh.cpp 的 kHexFace 表一致；
// 这里复制一份是为了让 Hex8Ref 不依赖 snappyhexmesh.cpp 的私有 anon 表。
const int kFaceAxes[6][3] = {
	{2, 0, 1}, // d=0: -z; axD=z, axA=x, axB=y
	{2, 0, 1}, // d=1: +z
	{1, 0, 2}, // d=2: -y; axD=y, axA=x, axB=z
	{1, 0, 2}, // d=3: +y
	{0, 1, 2}, // d=4: -x; axD=x, axA=y, axB=z
	{0, 1, 2}  // d=5: +x
};
} // namespace

XFoam_Hex8Ref::XFoam_Hex8Ref(
	XFoam_Label nxIn, XFoam_Label nyIn, XFoam_Label nzIn, XFoam_Label levelCapIn)
	: nx_(nxIn), ny_(nyIn), nz_(nzIn), levelCap_(levelCapIn),
	  leaves_(), leafMap_(), leafMapDirty_(true)
{}

uint64_t XFoam_Hex8Ref::encodeLeafKey(
	XFoam_Label ai, XFoam_Label aj, XFoam_Label ak,
	XFoam_Label level,
	XFoam_Label si, XFoam_Label sj, XFoam_Label sk)
{
	// 布局：ai[10] | aj[10] | ak[10] | level[4] | si[10] | sj[10] | sk[10] = 64 bit。
	// Debug 校验各字段未越界；Release 走 mask 截断（仍可能拓扑错乱 → 必须靠 assert 拦住）。
	constexpr uint64_t kBaseMask  = 0x3FF; // 10 bit
	constexpr uint64_t kLevelMask = 0xF;   // 4 bit
	constexpr uint64_t kSubMask   = 0x3FF; // 10 bit
	assert(ai    >= 0 && static_cast<uint64_t>(ai)    <= kBaseMask);
	assert(aj    >= 0 && static_cast<uint64_t>(aj)    <= kBaseMask);
	assert(ak    >= 0 && static_cast<uint64_t>(ak)    <= kBaseMask);
	assert(level >= 0 && static_cast<uint64_t>(level) <= kLevelMask);
	assert(si    >= 0 && static_cast<uint64_t>(si)    <= kSubMask);
	assert(sj    >= 0 && static_cast<uint64_t>(sj)    <= kSubMask);
	assert(sk    >= 0 && static_cast<uint64_t>(sk)    <= kSubMask);

	uint64_t k = static_cast<uint64_t>(ai) & kBaseMask;
	k = (k << 10) | (static_cast<uint64_t>(aj)    & kBaseMask);
	k = (k << 10) | (static_cast<uint64_t>(ak)    & kBaseMask);
	k = (k <<  4) | (static_cast<uint64_t>(level) & kLevelMask);
	k = (k << 10) | (static_cast<uint64_t>(si)    & kSubMask);
	k = (k << 10) | (static_cast<uint64_t>(sj)    & kSubMask);
	k = (k << 10) | (static_cast<uint64_t>(sk)    & kSubMask);
	return k;
}

void XFoam_Hex8Ref::rebuildLeafMap() const
{
	leafMap_.clear();
	leafMap_.reserve(leaves_.size() * 2);
	for (size_t i = 0; i < leaves_.size(); ++i)
	{
		const Leaf& l = leaves_[i];
		leafMap_.emplace(
			encodeLeafKey(l.ai, l.aj, l.ak, l.level, l.si, l.sj, l.sk),
			static_cast<XFoam_Label>(i));
	}
	leafMapDirty_ = false;
}

void XFoam_Hex8Ref::subdivide(XFoam_Label idx)
{
	const Leaf parent = leaves_[idx];
	if (parent.level >= levelCap_) return;

	// 8 个孩子按 (oc=0..7, kOfU/V/W) 排列，但其实 oct 索引不重要 — 只是 push 顺序。
	// 第一个孩子就地替换父 leaf，避免 erase 引起的 vector shift。
	Leaf c0;
	c0.ai = parent.ai; c0.aj = parent.aj; c0.ak = parent.ak;
	c0.level = parent.level + 1;
	c0.si = 2 * parent.si;
	c0.sj = 2 * parent.sj;
	c0.sk = 2 * parent.sk;
	c0.kept = true;
	c0.cellId = -1;
	for (int o = 0; o < 8; ++o) c0.corner[o] = -1;
	leaves_[idx] = c0;

	for (int oc = 1; oc < 8; ++oc)
	{
		Leaf c;
		c.ai = parent.ai; c.aj = parent.aj; c.ak = parent.ak;
		c.level = parent.level + 1;
		c.si = 2 * parent.si + (oc & 1);
		c.sj = 2 * parent.sj + ((oc >> 1) & 1);
		c.sk = 2 * parent.sk + ((oc >> 2) & 1);
		c.kept = true;
		c.cellId = -1;
		for (int o = 0; o < 8; ++o) c.corner[o] = -1;
		leaves_.push_back(c);
	}
	markLeafMapDirty();
}

void XFoam_Hex8Ref::initBaseLeaves()
{
	leaves_.clear();
	leaves_.reserve(static_cast<size_t>(nx_) * ny_ * nz_);
	for (XFoam_Label k = 0; k < nz_; ++k)
	{
		for (XFoam_Label j = 0; j < ny_; ++j)
		{
			for (XFoam_Label i = 0; i < nx_; ++i)
			{
				Leaf l;
				l.ai = i; l.aj = j; l.ak = k;
				leaves_.push_back(l);
			}
		}
	}
	markLeafMapDirty();
}

void XFoam_Hex8Ref::refineByPredicate(XFoam_Label targetLevel, const LeafPredicate& pred)
{
	// 每轮快照当前 leaves_.size()，避免边遍历边 push_back 时迭代到新孩子。
	// 收敛上限：targetLevel + 4 — 第一次扫已经能把所有 L=0 的 leaf 推到 L=1，
	// 之后每轮至多再深一级。+4 给迭代不规则的 STL 一点 buffer。
	XFoam_Label iter = 0;
	bool anySubdivided = true;
	while (anySubdivided && iter < targetLevel + 4)
	{
		anySubdivided = false;
		const size_t prevSize = leaves_.size();
		for (size_t i = 0; i < prevSize; ++i)
		{
			if (leaves_[i].level >= targetLevel) continue;
			if (pred(leaves_[i]))
			{
				subdivide(static_cast<XFoam_Label>(i));
				anySubdivided = true;
			}
		}
		++iter;
	}
}

void XFoam_Hex8Ref::balance21()
{
	// 反复 pass：每轮重建 leafMap_，遍历当前所有 leaves，查 6 个 face 上是否存在比自己
	// 更细 (level > self.level + 1) 的邻居 leaf；找到就 subdivide 自己。直到一整轮无变化。
	XFoam_Label iter = 0;
	bool anySubdivided = true;
	while (anySubdivided && iter < (levelCap_ + 2))
	{
		anySubdivided = false;
		rebuildLeafMap();
		const size_t prevSize = leaves_.size();
		for (size_t i = 0; i < prevSize; ++i)
		{
			const Leaf& l = leaves_[i];
			if (l.level >= levelCap_) continue;

			XFoam_Label maxNbr = -1;
			for (int d = 0; d < 6; ++d)
			{
				XFoam_Label aiN, ajN, akN, nsi, nsj, nsk;
				if (!stepNeighborAtSameLevel(l, d, aiN, ajN, akN, nsi, nsj, nsk)) continue;

				// 从最细 level 倒着试；只要找到 1 个 level > l.level + 1 的邻居就够。
				for (XFoam_Label lc = l.level + 2; lc <= levelCap_; ++lc)
				{
					const XFoam_Label factor = static_cast<XFoam_Label>(1) << (lc - l.level);
					const int axD = kFaceAxes[d][0];
					const int axA = kFaceAxes[d][1];
					const int axB = kFaceAxes[d][2];
					const XFoam_Label offD = (d % 2 == 0) ? (factor - 1) : 0;
					const XFoam_Label nsArr[3] = {nsi, nsj, nsk};
					bool found = false;
					for (XFoam_Label a = 0; a < factor && !found; ++a)
					{
						for (XFoam_Label b = 0; b < factor && !found; ++b)
						{
							XFoam_Label fp[3];
							fp[axD] = nsArr[axD] * factor + offD;
							fp[axA] = nsArr[axA] * factor + a;
							fp[axB] = nsArr[axB] * factor + b;
							const uint64_t key = encodeLeafKey(aiN, ajN, akN, lc, fp[0], fp[1], fp[2]);
							if (leafMap_.find(key) != leafMap_.end())
							{
								found = true;
								if (lc > maxNbr) maxNbr = lc;
							}
						}
					}
					if (found) break;
				}
			}

			if (maxNbr > l.level + 1)
			{
				subdivide(static_cast<XFoam_Label>(i));
				anySubdivided = true;
			}
		}
		++iter;
	}
}

void XFoam_Hex8Ref::extendHighLevel(int nLayers)
{
	// 每一轮（pass）：
	//   1) 重建 leafMap_
	//   2) 找所有"自身 level > 0 且至少有一个 Coarser face 邻居"的 leaf — 这是当前"细侧前沿"
	//   3) 把它们的所有 Coarser face 邻居 leaf 索引收到 toSubdivide
	//   4) 去重，按降序 subdivide（subdivide 是 in-place + push_back，旧 idx<X 不动）
	//
	// 单轮把"前沿"外扩 1 圈；nLayers 轮就外扩 nLayers 圈。每轮新增的 leaves 仍是粗侧（比刚
	// 升的细侧低 1 级），下一轮自然成为新的"前沿粗邻居"。
	for (int iter = 0; iter < nLayers; ++iter)
	{
		rebuildLeafMap();
		std::vector<XFoam_Label> toSubdivide;
		toSubdivide.reserve(leaves_.size() / 4);

		for (size_t i = 0; i < leaves_.size(); ++i)
		{
			const Leaf& l = leaves_[i];
			if (l.level == 0) continue;
			for (int d = 0; d < 6; ++d)
			{
				const FaceNbr nbr = resolveFaceNeighbor(l, d);
				if (nbr.kind == FaceNbrKind::Coarser && nbr.leafIdx >= 0)
				{
					// 只升 level == self.level - 1 的邻居；更粗的 (差 ≥ 2) 留给 balance21 处理，
					// 避免一次性把"远端粗 cell"无意义地拉细。
					if (leaves_[nbr.leafIdx].level == l.level - 1)
					{
						toSubdivide.push_back(nbr.leafIdx);
					}
				}
			}
		}

		if (toSubdivide.empty()) break;

		std::sort(toSubdivide.begin(), toSubdivide.end());
		toSubdivide.erase(std::unique(toSubdivide.begin(), toSubdivide.end()), toSubdivide.end());
		// 降序处理：subdivide(idx) 不动 < idx 的项，遇 push_back 也只追加在尾；不会失效。
		std::sort(toSubdivide.rbegin(), toSubdivide.rend());
		for (XFoam_Label idx : toSubdivide)
		{
			if (leaves_[idx].level < levelCap_)
			{
				subdivide(idx);
			}
		}
	}
}

void XFoam_Hex8Ref::cullByPredicate(const LeafPredicate& pred)
{
	for (size_t i = 0; i < leaves_.size(); ++i)
	{
		leaves_[i].kept = !pred(leaves_[i]);
	}
	// kept 翻转不动拓扑；不脏化 leafMap_。
}

XFoam_Label XFoam_Hex8Ref::assignCellIds()
{
	XFoam_Label next = 0;
	for (size_t i = 0; i < leaves_.size(); ++i)
	{
		if (leaves_[i].kept) leaves_[i].cellId = next++;
		else                 leaves_[i].cellId = -1;
	}
	return next;
}

void XFoam_Hex8Ref::leafCornerParam(
	const Leaf& l, int oc,
	XFoam_Scalar& u, XFoam_Scalar& v, XFoam_Scalar& w)
{
	const XFoam_Label n = static_cast<XFoam_Label>(1) << l.level;
	u = static_cast<XFoam_Scalar>(l.si + kOfU[oc]) / n;
	v = static_cast<XFoam_Scalar>(l.sj + kOfV[oc]) / n;
	w = static_cast<XFoam_Scalar>(l.sk + kOfW[oc]) / n;
}

void XFoam_Hex8Ref::leafCentroidParam(
	const Leaf& l,
	XFoam_Scalar& u, XFoam_Scalar& v, XFoam_Scalar& w)
{
	const XFoam_Label n = static_cast<XFoam_Label>(1) << l.level;
	u = (static_cast<XFoam_Scalar>(l.si) + static_cast<XFoam_Scalar>(0.5)) / n;
	v = (static_cast<XFoam_Scalar>(l.sj) + static_cast<XFoam_Scalar>(0.5)) / n;
	w = (static_cast<XFoam_Scalar>(l.sk) + static_cast<XFoam_Scalar>(0.5)) / n;
}

void XFoam_Hex8Ref::faceSteinerParam(
	const Leaf& l, int d, int rr, int cc,
	XFoam_Scalar& u, XFoam_Scalar& v, XFoam_Scalar& w)
{
	const int axD = kFaceAxes[d][0];
	const int axA = kFaceAxes[d][1];
	const int axB = kFaceAxes[d][2];
	// d=负方向 (-z/-y/-x)：face 落在 L 层的 sX 处 → L+1 层 2*sX
	// d=正方向 (+z/+y/+x)：face 落在 L 层的 sX+1 处 → L+1 层 2*sX+2
	const int offD = (d % 2 == 0) ? 0 : 2;
	const XFoam_Label sArr[3] = {l.si, l.sj, l.sk};
	XFoam_Label fp[3];
	fp[axD] = 2 * sArr[axD] + offD;
	fp[axA] = 2 * sArr[axA] + cc;
	fp[axB] = 2 * sArr[axB] + rr;
	const XFoam_Label nF = static_cast<XFoam_Label>(1) << (l.level + 1);
	u = static_cast<XFoam_Scalar>(fp[0]) / nF;
	v = static_cast<XFoam_Scalar>(fp[1]) / nF;
	w = static_cast<XFoam_Scalar>(fp[2]) / nF;
}

bool XFoam_Hex8Ref::stepNeighborAtSameLevel(
	const Leaf& l, int d,
	XFoam_Label& aiOut, XFoam_Label& ajOut, XFoam_Label& akOut,
	XFoam_Label& nsiOut, XFoam_Label& nsjOut, XFoam_Label& nskOut) const
{
	aiOut = l.ai; ajOut = l.aj; akOut = l.ak;
	nsiOut = l.si; nsjOut = l.sj; nskOut = l.sk;
	const XFoam_Label n = static_cast<XFoam_Label>(1) << l.level;
	switch (d)
	{
	case 0: --nskOut; break;
	case 1: ++nskOut; break;
	case 2: --nsjOut; break;
	case 3: ++nsjOut; break;
	case 4: --nsiOut; break;
	case 5: ++nsiOut; break;
	default: return false;
	}
	if (nsiOut < 0)       { --aiOut; nsiOut = n - 1; }
	else if (nsiOut >= n) { ++aiOut; nsiOut = 0; }
	if (nsjOut < 0)       { --ajOut; nsjOut = n - 1; }
	else if (nsjOut >= n) { ++ajOut; nsjOut = 0; }
	if (nskOut < 0)       { --akOut; nskOut = n - 1; }
	else if (nskOut >= n) { ++akOut; nskOut = 0; }
	return inGrid(aiOut, ajOut, akOut);
}

XFoam_Hex8Ref::FaceNbr XFoam_Hex8Ref::resolveFaceNeighbor(const Leaf& l, int d) const
{
	FaceNbr r;
	if (leafMapDirty_) rebuildLeafMap();

	XFoam_Label aiN, ajN, akN, nsi, nsj, nsk;
	if (!stepNeighborAtSameLevel(l, d, aiN, ajN, akN, nsi, nsj, nsk))
	{
		r.kind = FaceNbrKind::OutOfGrid;
		return r;
	}

	// same-level
	{
		auto it = leafMap_.find(encodeLeafKey(aiN, ajN, akN, l.level, nsi, nsj, nsk));
		if (it != leafMap_.end())
		{
			r.kind = FaceNbrKind::Same;
			r.leafIdx = it->second;
			return r;
		}
	}

	// coarser：向上 (L-1, L-2, ...) 找包含 (nsi, nsj, nsk) 的 ancestor
	for (XFoam_Label lq = l.level - 1; lq >= 0; --lq)
	{
		const XFoam_Label shift = l.level - lq;
		auto it = leafMap_.find(
			encodeLeafKey(aiN, ajN, akN, lq, nsi >> shift, nsj >> shift, nsk >> shift));
		if (it != leafMap_.end())
		{
			r.kind = FaceNbrKind::Coarser;
			r.leafIdx = it->second;
			return r;
		}
	}

	// finer at L+1（2:1 balance 后保证最多差 1 级）
	const int axD = kFaceAxes[d][0];
	const int axA = kFaceAxes[d][1];
	const int axB = kFaceAxes[d][2];
	// finer 邻居在我这一侧的 face 接缝：从 L+1 视角看，邻居的 axD 子位置紧贴本侧
	//   d 负方向 → 邻居 axD 子位置 = 2*nsArr[axD] + 1
	//   d 正方向 → 邻居 axD 子位置 = 2*nsArr[axD] + 0
	const XFoam_Label offD = (d % 2 == 0) ? 1 : 0;
	const XFoam_Label nsArr[3] = {nsi, nsj, nsk};
	bool anyFine = false;
	for (XFoam_Label rr = 0; rr < 2; ++rr)
	{
		for (XFoam_Label cc = 0; cc < 2; ++cc)
		{
			XFoam_Label fp[3];
			fp[axD] = 2 * nsArr[axD] + offD;
			fp[axA] = 2 * nsArr[axA] + cc;
			fp[axB] = 2 * nsArr[axB] + rr;
			auto it = leafMap_.find(encodeLeafKey(aiN, ajN, akN, l.level + 1, fp[0], fp[1], fp[2]));
			if (it != leafMap_.end())
			{
				r.fineLeafIdx[rr * 2 + cc] = it->second;
				anyFine = true;
			}
		}
	}
	r.kind = anyFine ? FaceNbrKind::Finer : FaceNbrKind::None;
	return r;
}

XFoam_Label XFoam_Hex8Ref::maxLevelReached() const
{
	XFoam_Label m = 0;
	for (size_t i = 0; i < leaves_.size(); ++i)
	{
		if (leaves_[i].level > m) m = leaves_[i].level;
	}
	return m;
}

void XFoam_Hex8Ref::perLevelCounts(XFoam_Label out[kMaxLevelBuckets]) const
{
	for (int i = 0; i < kMaxLevelBuckets; ++i) out[i] = 0;
	for (size_t i = 0; i < leaves_.size(); ++i)
	{
		const XFoam_Label lv = leaves_[i].level;
		if (lv >= 0 && lv < kMaxLevelBuckets) ++out[lv];
	}
}
