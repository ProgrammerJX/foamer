#include "XFoam/cmsh/xfoam_cmshoctree.h"

#include "XFoam/cmsh/xfoam_cmshobjrefine.h"

#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <cmath>

// =============================================================================
// XFoam_CMshOctreeCubeCoords
// =============================================================================

void XFoam_CMshOctreeCubeCoords::cubeBox(
	const XFoam_BoundBox& root,
	XFoam_Vector3D&       outMin,
	XFoam_Vector3D&       outMax) const
{
	const XFoam_Vector3D s = root.span();
	const XFoam_Label    n = static_cast<XFoam_Label>(1) << level; // 2^level
	const XFoam_Scalar   dx = s.x() / static_cast<XFoam_Scalar>(n);
	const XFoam_Scalar   dy = s.y() / static_cast<XFoam_Scalar>(n);
	const XFoam_Scalar   dz = s.z() / static_cast<XFoam_Scalar>(n);
	outMin = XFoam_Vector3D(root.min().x() + posX * dx,
	                        root.min().y() + posY * dy,
	                        root.min().z() + posZ * dz);
	outMax = XFoam_Vector3D(outMin.x() + dx,
	                        outMin.y() + dy,
	                        outMin.z() + dz);
}

XFoam_Vector3D XFoam_CMshOctreeCubeCoords::centre(const XFoam_BoundBox& root) const
{
	XFoam_Vector3D mn, mx;
	cubeBox(root, mn, mx);
	return XFoam_Vector3D(static_cast<XFoam_Scalar>(0.5) * (mn.x() + mx.x()),
	                      static_cast<XFoam_Scalar>(0.5) * (mn.y() + mx.y()),
	                      static_cast<XFoam_Scalar>(0.5) * (mn.z() + mx.z()));
}

XFoam_Scalar XFoam_CMshOctreeCubeCoords::size(const XFoam_BoundBox& root) const
{
	const XFoam_Label n = static_cast<XFoam_Label>(1) << level;
	return root.span().x() / static_cast<XFoam_Scalar>(n);
}

XFoam_CMshOctreeCubeCoords XFoam_CMshOctreeCubeCoords::childCoords(int childIdx) const
{
	const int bx = childIdx & 0x1;
	const int by = (childIdx >> 1) & 0x1;
	const int bz = (childIdx >> 2) & 0x1;
	return XFoam_CMshOctreeCubeCoords(
		(posX << 1) + bx,
		(posY << 1) + by,
		(posZ << 1) + bz,
		static_cast<std::uint8_t>(level + 1));
}

// =============================================================================
// XFoam_CMshOctreeCube
// =============================================================================

XFoam_CMshOctreeCube::~XFoam_CMshOctreeCube()
{
	for (int i = 0; i < 8; ++i)
	{
		delete children[i];
		children[i] = nullptr;
	}
}

void XFoam_CMshOctreeCube::subdivide()
{
	if (!isLeaf()) return;
	const XFoam_CMshOctreeCubeCoords cc(*this);
	for (int i = 0; i < 8; ++i)
	{
		children[i] = new XFoam_CMshOctreeCube(cc.childCoords(i));
	}
	leafIdx = -1;
}

// =============================================================================
// XFoam_CMshOctree
// =============================================================================

XFoam_CMshOctree::XFoam_CMshOctree(
	const XFoam_BrepBase& surface,
	const XFoam_BoundBox& rootBox)
	: surface_(surface)
	, rootBox_(rootBox)
{
	root_ = new XFoam_CMshOctreeCube(XFoam_CMshOctreeCubeCoords(0, 0, 0, 0));
	rebuildLeaves();
}

XFoam_CMshOctree::~XFoam_CMshOctree()
{
	delete root_;
	root_ = nullptr;
}

void XFoam_CMshOctree::collectLeavesRecursive(XFoam_CMshOctreeCube* c)
{
	if (!c) return;
	if (c->isLeaf())
	{
		c->leafIdx = static_cast<XFoam_Label>(leaves_.size());
		leaves_.push_back(c);
		return;
	}
	c->leafIdx = -1;
	for (int i = 0; i < 8; ++i)
	{
		collectLeavesRecursive(c->children[i]);
	}
}

void XFoam_CMshOctree::rebuildLeaves()
{
	leaves_.clear();
	collectLeavesRecursive(root_);
	rebuildIndex();
}

std::uint64_t XFoam_CMshOctree::packCoord(
	std::uint8_t level, XFoam_Label x, XFoam_Label y, XFoam_Label z)
{
	// level ≤ 14 → 4 bit；x/y/z 各 20 bit (上限 2^20 = 1048576，远超 maxLevel 实际值)
	return (static_cast<std::uint64_t>(level) & 0xF)
	     | ((static_cast<std::uint64_t>(x) & 0xFFFFF) << 4)
	     | ((static_cast<std::uint64_t>(y) & 0xFFFFF) << 24)
	     | ((static_cast<std::uint64_t>(z) & 0xFFFFF) << 44);
}

void XFoam_CMshOctree::rebuildIndex()
{
	leafByCoords_.clear();
	leafByCoords_.reserve(leaves_.size() * 2);
	for (const auto* leaf : leaves_)
	{
		leafByCoords_.emplace(packCoord(leaf->level, leaf->posX, leaf->posY, leaf->posZ), leaf);
	}
}

const XFoam_CMshOctreeCube* XFoam_CMshOctree::findLeafByCoords(
	std::uint8_t level, XFoam_Label x, XFoam_Label y, XFoam_Label z) const
{
	auto it = leafByCoords_.find(packCoord(level, x, y, z));
	return it == leafByCoords_.end() ? nullptr : it->second;
}

namespace
{
// face dir → (axis, sign)：0:-x 1:+x 2:-y 3:+y 4:-z 5:+z
constexpr int kFaceAxis[6] = {0, 0, 1, 1, 2, 2};
constexpr int kFaceSign[6] = {-1, +1, -1, +1, -1, +1};
// face d 的两条切向轴
constexpr int kFaceTang0[6] = {1, 1, 0, 0, 0, 0};
constexpr int kFaceTang1[6] = {2, 2, 2, 2, 1, 1};
} // namespace

XFoam_CMshOctree::FaceNbrResult
XFoam_CMshOctree::faceNeighbour(const XFoam_CMshOctreeCube& self, int d) const
{
	FaceNbrResult r;
	const int axis = kFaceAxis[d];
	const int sign = kFaceSign[d];
	const XFoam_Label pos[3] = {self.posX, self.posY, self.posZ};

	// 同 level 邻居坐标：把 axis 维 +/- 1
	XFoam_Label same[3] = {pos[0], pos[1], pos[2]};
	same[axis] = pos[axis] + sign;
	const XFoam_Label n = static_cast<XFoam_Label>(1) << self.level;
	if (same[axis] < 0 || same[axis] >= n)
	{
		// 落在 root 外
		return r;
	}

	if (const auto* lf = findLeafByCoords(self.level, same[0], same[1], same[2]))
	{
		r.kind = FaceNbrKind::Same;
		r.same = lf;
		return r;
	}

	// 粗一级邻居：(level-1, same >> 1)
	if (self.level > 0)
	{
		const std::uint8_t lc = self.level - 1;
		if (const auto* lf = findLeafByCoords(lc, same[0] >> 1, same[1] >> 1, same[2] >> 1))
		{
			r.kind = FaceNbrKind::Coarser;
			r.same = lf;
			return r;
		}
	}

	// 细一级邻居：level+1，axis 维取 self 这一侧 (sign<0 ? 2*pos[axis]-1 : 2*pos[axis]+2)
	const std::uint8_t lf = static_cast<std::uint8_t>(self.level + 1);
	XFoam_Label base[3] = {2 * pos[0], 2 * pos[1], 2 * pos[2]};
	const XFoam_Label nf = static_cast<XFoam_Label>(1) << lf;
	// 邻居在 +sign 侧、紧贴接缝：sign>0 → axis 维 = base[axis] + 2 - 1; sign<0 → axis 维 = base[axis] - 1
	const XFoam_Label faxis = (sign > 0) ? (base[axis] + 2) : (base[axis] - 1);
	if (faxis < 0 || faxis >= nf)
	{
		return r;
	}
	const int t0 = kFaceTang0[d];
	const int t1 = kFaceTang1[d];
	bool anyFine = false;
	for (int b1 = 0; b1 < 2; ++b1)
	{
		for (int b0 = 0; b0 < 2; ++b0)
		{
			XFoam_Label fp[3] = {base[0], base[1], base[2]};
			fp[axis] = faxis;
			fp[t0]   = base[t0] + b0;
			fp[t1]   = base[t1] + b1;
			if (const auto* lfp = findLeafByCoords(lf, fp[0], fp[1], fp[2]))
			{
				r.finer[b1 * 2 + b0] = lfp;
				anyFine = true;
			}
		}
	}
	if (anyFine) r.kind = FaceNbrKind::Finer;
	return r;
}

void XFoam_CMshOctree::refineByObject(const XFoam_CMshObjRefine& obj, int targetLevel)
{
	const int target = (targetLevel < 0) ? obj.level : targetLevel;
	bool progressed = true;
	int safety = 64;
	while (progressed && safety-- > 0)
	{
		progressed = false;
		std::vector<XFoam_CMshOctreeCube*> snap = leaves_;
		for (auto* leaf : snap)
		{
			if (!leaf->isLeaf()) continue;
			if (leaf->level >= target) continue;
			XFoam_Vector3D mn, mx;
			leaf->cubeBox(rootBox_, mn, mx);
			if (!obj.boxIntersects(XFoam_BoundBox(mn, mx))) continue;
			leaf->subdivide();
			progressed = true;
		}
		if (progressed) rebuildLeaves();
	}
}

void XFoam_CMshOctree::balance21()
{
	// 2:1 balance：保证任意两个 face 相邻 leaf 的 level 差 ≤ 1。
	//
	// 算法：从最细的 leaf 开始，看 6 个 face-neighbor 方向；若邻居方向上有 leaf
	// 比 self 粗 ≥ 2 级（用 findLeafByCoords 在 (self.level - 2) 找到 ancestor
	// leaf），就把那个 coarse leaf 细分一次。反复扫直到稳定。每次细分必把不平衡
	// 度降 1，故 safety 足够。
	bool progressed = true;
	int safety = 64;
	while (progressed && safety-- > 0)
	{
		progressed = false;
		std::vector<XFoam_CMshOctreeCube*> snap = leaves_;
		// 先按 level 从大到小排，先处理最细的
		std::sort(snap.begin(), snap.end(),
			[](const XFoam_CMshOctreeCube* a, const XFoam_CMshOctreeCube* b) {
				return a->level > b->level;
			});
		for (auto* leaf : snap)
		{
			if (!leaf->isLeaf()) continue;
			for (int d = 0; d < 6; ++d)
			{
				const int axis = kFaceAxis[d];
				const int sign = kFaceSign[d];
				XFoam_Label pos[3] = {leaf->posX, leaf->posY, leaf->posZ};
				pos[axis] += sign;
				const XFoam_Label n = static_cast<XFoam_Label>(1) << leaf->level;
				if (pos[axis] < 0 || pos[axis] >= n) continue;

				// 同层若已有 leaf 不用细化；同层若是空 cube 说明邻居更粗
				if (findLeafByCoords(leaf->level, pos[0], pos[1], pos[2])) continue;

				// 找祖先 leaf；若它的 level ≤ self.level - 2，就 subdivide 它
				for (int dl = 1; dl <= leaf->level; ++dl)
				{
					const std::uint8_t lq = static_cast<std::uint8_t>(leaf->level - dl);
					const XFoam_Label cx = pos[0] >> dl;
					const XFoam_Label cy = pos[1] >> dl;
					const XFoam_Label cz = pos[2] >> dl;
					if (auto* lf = const_cast<XFoam_CMshOctreeCube*>(
						findLeafByCoords(lq, cx, cy, cz)))
					{
						if (dl >= 2)
						{
							lf->subdivide();
							progressed = true;
						}
						break;
					}
				}
			}
		}
		if (progressed) rebuildLeaves();
	}
}

void XFoam_CMshOctree::refineUniform(int baseLevel)
{
	if (baseLevel <= 0) return;
	// 每一轮：拷 leaves 快照，所有 level < baseLevel 的 leaf 一次性 subdivide；
	// rebuild 后下一轮继续，直到没有任何 leaf 还低于 baseLevel。
	bool progressed = true;
	while (progressed)
	{
		progressed = false;
		std::vector<XFoam_CMshOctreeCube*> snap = leaves_;
		for (auto* leaf : snap)
		{
			if (static_cast<int>(leaf->level) >= baseLevel) continue;
			leaf->subdivide();
			progressed = true;
		}
		if (progressed) rebuildLeaves();
	}
}

void XFoam_CMshOctree::refineToSurface(int targetLevel)
{
	if (targetLevel <= 0) return;
	bool progressed = true;
	while (progressed)
	{
		progressed = false;
		std::vector<XFoam_CMshOctreeCube*> snap = leaves_;
		for (auto* leaf : snap)
		{
			if (static_cast<int>(leaf->level) >= targetLevel) continue;
			XFoam_Vector3D mn, mx;
			leaf->cubeBox(rootBox_, mn, mx);
			XFoam_BoundBox bb(mn, mx);
			if (!surface_.boxIntersects(bb)) continue;
			leaf->subdivide();
			progressed = true;
		}
		if (progressed) rebuildLeaves();
	}
}

void XFoam_CMshOctree::refineRegion(const XFoam_BoundBox& region, int targetLevel)
{
	if (targetLevel <= 0) return;
	bool progressed = true;
	while (progressed)
	{
		progressed = false;
		std::vector<XFoam_CMshOctreeCube*> snap = leaves_;
		for (auto* leaf : snap)
		{
			if (static_cast<int>(leaf->level) >= targetLevel) continue;
			XFoam_Vector3D mn, mx;
			leaf->cubeBox(rootBox_, mn, mx);
			XFoam_BoundBox bb(mn, mx);
			if (!region.overlaps(bb)) continue;
			leaf->subdivide();
			progressed = true;
		}
		if (progressed) rebuildLeaves();
	}
}

void XFoam_CMshOctree::classifyLeaves()
{
	for (auto* leaf : leaves_)
	{
		XFoam_Vector3D mn, mx;
		leaf->cubeBox(rootBox_, mn, mx);
		XFoam_BoundBox bb(mn, mx);
		if (surface_.boxIntersects(bb))
		{
			leaf->type = XFoam_CMshCubeType::Data;
		}
		else
		{
			const XFoam_Vector3D c(static_cast<XFoam_Scalar>(0.5) * (mn.x() + mx.x()),
			                        static_cast<XFoam_Scalar>(0.5) * (mn.y() + mx.y()),
			                        static_cast<XFoam_Scalar>(0.5) * (mn.z() + mx.z()));
			leaf->type = surface_.contains(c)
				? XFoam_CMshCubeType::Inside
				: XFoam_CMshCubeType::Outside;
		}
	}
}

void XFoam_CMshOctree::countLeavesByLevel(std::vector<XFoam_Label>& out) const
{
	out.clear();
	for (const auto* leaf : leaves_)
	{
		const std::size_t lv = leaf->level;
		if (out.size() <= lv) out.resize(lv + 1, 0);
		++out[lv];
	}
}

void XFoam_CMshOctree::countLeavesByType(
	XFoam_Label& nUnknown,
	XFoam_Label& nOutside,
	XFoam_Label& nData,
	XFoam_Label& nInside) const
{
	nUnknown = nOutside = nData = nInside = 0;
	for (const auto* leaf : leaves_)
	{
		switch (leaf->type)
		{
			case XFoam_CMshCubeType::Unknown: ++nUnknown; break;
			case XFoam_CMshCubeType::Outside: ++nOutside; break;
			case XFoam_CMshCubeType::Data:    ++nData;    break;
			case XFoam_CMshCubeType::Inside:  ++nInside;  break;
		}
	}
}

void XFoam_CMshOctree::forEachLeaf(
	const std::function<void(const XFoam_CMshOctreeCube&)>& fn) const
{
	for (const auto* leaf : leaves_)
	{
		fn(*leaf);
	}
}
