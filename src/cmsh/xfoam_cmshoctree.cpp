#include "XFoam/cmsh/xfoam_cmshoctree.h"

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
