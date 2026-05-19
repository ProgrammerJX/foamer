#include "XFoam/snap/xfoam_trisurface.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace
{
// 简化但稳的"点到三角形最近点"，对照 Christer Ericson, Real-Time Collision Detection, §5.1.5。
// 返回 closest 点（位于 triangle 上），dist2 为 |p-closest|^2。
XFoam_Vector3D closestPointOnTriangle(
	const XFoam_Vector3D& p,
	const XFoam_Vector3D& a,
	const XFoam_Vector3D& b,
	const XFoam_Vector3D& c,
	XFoam_Scalar& dist2)
{
	const XFoam_Vector3D ab = b - a;
	const XFoam_Vector3D ac = c - a;
	const XFoam_Vector3D ap = p - a;
	const XFoam_Scalar d1 = ab & ap;
	const XFoam_Scalar d2 = ac & ap;
	if (d1 <= 0 && d2 <= 0)
	{
		const XFoam_Vector3D q = a;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Vector3D bp = p - b;
	const XFoam_Scalar d3 = ab & bp;
	const XFoam_Scalar d4 = ac & bp;
	if (d3 >= 0 && d4 <= d3)
	{
		const XFoam_Vector3D q = b;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Scalar vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0)
	{
		const XFoam_Scalar v = d1 / (d1 - d3);
		const XFoam_Vector3D q = a + ab * v;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Vector3D cp = p - c;
	const XFoam_Scalar d5 = ab & cp;
	const XFoam_Scalar d6 = ac & cp;
	if (d6 >= 0 && d5 <= d6)
	{
		const XFoam_Vector3D q = c;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Scalar vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0)
	{
		const XFoam_Scalar w = d2 / (d2 - d6);
		const XFoam_Vector3D q = a + ac * w;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Scalar va = d3 * d6 - d5 * d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
	{
		const XFoam_Scalar w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		const XFoam_Vector3D q = b + (c - b) * w;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Scalar denom = static_cast<XFoam_Scalar>(1) / (va + vb + vc);
	const XFoam_Scalar v = vb * denom;
	const XFoam_Scalar w = vc * denom;
	const XFoam_Vector3D q = a + ab * v + ac * w;
	dist2 = (p - q).magSqr();
	return q;
}

// Möller-Trumbore 三角形 vs 射线（origin, dir 任意），dir 沿 +X，dir = (1,0,0)。
// 命中且 t > 0 → 真命中（用于内外射线计数）。
bool rayHitsTrianglePlusX(
	const XFoam_Vector3D& orig,
	const XFoam_Vector3D& a,
	const XFoam_Vector3D& b,
	const XFoam_Vector3D& c)
{
	// dir = (1, 0, 0)
	const XFoam_Vector3D edge1 = b - a;
	const XFoam_Vector3D edge2 = c - a;
	// pvec = dir × edge2 = (0, -e2.z, e2.y)
	const XFoam_Vector3D pvec(
		0,
		-edge2.z(),
		edge2.y());
	const XFoam_Scalar det = edge1 & pvec;
	const XFoam_Scalar eps = static_cast<XFoam_Scalar>(1e-12);
	if (det > -eps && det < eps)
	{
		return false; // 平行
	}
	const XFoam_Scalar invDet = static_cast<XFoam_Scalar>(1) / det;
	const XFoam_Vector3D tvec = orig - a;
	const XFoam_Scalar u = (tvec & pvec) * invDet;
	if (u < 0 || u > 1) return false;
	const XFoam_Vector3D qvec = tvec ^ edge1;
	// dir & qvec = qvec.x
	const XFoam_Scalar v = qvec.x() * invDet;
	if (v < 0 || u + v > 1) return false;
	const XFoam_Scalar t = (edge2 & qvec) * invDet;
	return t > eps;
}

void readBinaryStream(std::istream& is, void* buf, size_t n)
{
	is.read(static_cast<char*>(buf), static_cast<std::streamsize>(n));
}
} // namespace

void XFoam_TriSurface::addTriangle(
	const XFoam_Vector3D& a,
	const XFoam_Vector3D& b,
	const XFoam_Vector3D& c,
	const XFoam_Vector3D* explicitNormal)
{
	Triangle t;
	t.v0 = a;
	t.v1 = b;
	t.v2 = c;
	if (explicitNormal)
	{
		t.normal = XFoam_normalised(*explicitNormal);
	}
	else
	{
		t.normal = XFoam_normalised((b - a) ^ (c - a));
	}
	t.bbox = XFoam_BoundBox(a, a);
	t.bbox.min().x() = std::min({a.x(), b.x(), c.x()});
	t.bbox.min().y() = std::min({a.y(), b.y(), c.y()});
	t.bbox.min().z() = std::min({a.z(), b.z(), c.z()});
	t.bbox.max().x() = std::max({a.x(), b.x(), c.x()});
	t.bbox.max().y() = std::max({a.y(), b.y(), c.y()});
	t.bbox.max().z() = std::max({a.z(), b.z(), c.z()});
	tris_.push_back(t);
}

void XFoam_TriSurface::rebuildBounds()
{
	if (tris_.empty())
	{
		bounds_ = XFoam_BoundBox();
		return;
	}
	XFoam_Vector3D mn = tris_.front().bbox.min();
	XFoam_Vector3D mx = tris_.front().bbox.max();
	for (size_t i = 1; i < tris_.size(); ++i)
	{
		const XFoam_BoundBox& bb = tris_[i].bbox;
		mn.x() = std::min(mn.x(), bb.min().x());
		mn.y() = std::min(mn.y(), bb.min().y());
		mn.z() = std::min(mn.z(), bb.min().z());
		mx.x() = std::max(mx.x(), bb.max().x());
		mx.y() = std::max(mx.y(), bb.max().y());
		mx.z() = std::max(mx.z(), bb.max().z());
	}
	bounds_ = XFoam_BoundBox(mn, mx);
}

XFoam_Scalar XFoam_TriSurface::bboxMinDistSqr(
	const XFoam_Vector3D& p, const XFoam_BoundBox& bb)
{
	// 三轴独立：超出 [min, max] 区间的部分各自贡献平方距离；区间内时该轴贡献 0。
	const XFoam_Scalar dx = std::max<XFoam_Scalar>(
		0, std::max<XFoam_Scalar>(bb.min().x() - p.x(), p.x() - bb.max().x()));
	const XFoam_Scalar dy = std::max<XFoam_Scalar>(
		0, std::max<XFoam_Scalar>(bb.min().y() - p.y(), p.y() - bb.max().y()));
	const XFoam_Scalar dz = std::max<XFoam_Scalar>(
		0, std::max<XFoam_Scalar>(bb.min().z() - p.z(), p.z() - bb.max().z()));
	return dx * dx + dy * dy + dz * dz;
}

void XFoam_TriSurface::buildBvh()
{
	bvhNodes_.clear();
	bvhOrder_.clear();
	if (tris_.empty())
	{
		return;
	}
	bvhOrder_.resize(tris_.size());
	for (size_t i = 0; i < tris_.size(); ++i)
	{
		bvhOrder_[i] = static_cast<XFoam_Label>(i);
	}
	// 二叉树 N 个 leaf 最多 2N - 1 节点；leaf ≤ ceil(N / kBvhLeafLimit)，所以总节点 ≤ 2 * N。
	// 预 reserve 避免 push_back 反复 realloc（不会导致 int 索引失效，但拷贝 BvhNode 成本高）。
	bvhNodes_.reserve(2 * tris_.size());
	buildBvhRecursive(0, static_cast<int>(tris_.size()));
}

int XFoam_TriSurface::buildBvhRecursive(int lo, int hi)
{
	// 先占位（值待填）；递归时 nodeIdx 即可在 bvhNodes_ 中找到本节点；递归内部可能 push_back 但
	// 我们只在末尾写本节点字段，所以 emplace 时不要持有 &node 引用。
	bvhNodes_.emplace_back();
	const int nodeIdx = static_cast<int>(bvhNodes_.size() - 1);

	// 当前节点的 bbox = 范围内所有 tri bbox 合并。
	XFoam_BoundBox bb = tris_[bvhOrder_[lo]].bbox;
	for (int i = lo + 1; i < hi; ++i)
	{
		const XFoam_BoundBox& tb = tris_[bvhOrder_[i]].bbox;
		bb.min().x() = std::min(bb.min().x(), tb.min().x());
		bb.min().y() = std::min(bb.min().y(), tb.min().y());
		bb.min().z() = std::min(bb.min().z(), tb.min().z());
		bb.max().x() = std::max(bb.max().x(), tb.max().x());
		bb.max().y() = std::max(bb.max().y(), tb.max().y());
		bb.max().z() = std::max(bb.max().z(), tb.max().z());
	}

	const int count = hi - lo;
	if (count <= kBvhLeafLimit)
	{
		bvhNodes_[nodeIdx].bbox = bb;
		bvhNodes_[nodeIdx].firstTri = lo;
		bvhNodes_[nodeIdx].triCount = count;
		bvhNodes_[nodeIdx].leftIdx = -1;
		bvhNodes_[nodeIdx].rightIdx = -1;
		return nodeIdx;
	}

	// 选 extent 最大的轴；按 tri bbox 中心在该轴的排序，从中位数切。
	const XFoam_Scalar ex = bb.max().x() - bb.min().x();
	const XFoam_Scalar ey = bb.max().y() - bb.min().y();
	const XFoam_Scalar ez = bb.max().z() - bb.min().z();
	int axis = 0;
	if (ey > ex) axis = 1;
	if (axis == 0 ? (ez > ex) : (ez > ey)) axis = 2;

	auto centroidOnAxis = [&](XFoam_Label triIdx) -> XFoam_Scalar {
		const XFoam_BoundBox& tb = tris_[triIdx].bbox;
		switch (axis)
		{
		case 0: return static_cast<XFoam_Scalar>(0.5) * (tb.min().x() + tb.max().x());
		case 1: return static_cast<XFoam_Scalar>(0.5) * (tb.min().y() + tb.max().y());
		default: return static_cast<XFoam_Scalar>(0.5) * (tb.min().z() + tb.max().z());
		}
	};

	std::sort(bvhOrder_.begin() + lo, bvhOrder_.begin() + hi,
		[&](XFoam_Label a, XFoam_Label b) {
			return centroidOnAxis(a) < centroidOnAxis(b);
		});

	const int mid = (lo + hi) / 2;
	const int leftIdx = buildBvhRecursive(lo, mid);
	const int rightIdx = buildBvhRecursive(mid, hi);

	// 递归过程可能 realloc 过 bvhNodes_；nodeIdx 仍然有效。
	bvhNodes_[nodeIdx].bbox = bb;
	bvhNodes_[nodeIdx].firstTri = -1;
	bvhNodes_[nodeIdx].triCount = 0;
	bvhNodes_[nodeIdx].leftIdx = leftIdx;
	bvhNodes_[nodeIdx].rightIdx = rightIdx;
	return nodeIdx;
}

void XFoam_TriSurface::bvhClosestPoint(
	const XFoam_Vector3D& p,
	XFoam_Scalar& bestD2,
	XFoam_Vector3D& bestQ,
	XFoam_Label& bestTri,
	int nodeIdx) const
{
	const BvhNode& n = bvhNodes_[nodeIdx];
	if (bboxMinDistSqr(p, n.bbox) >= bestD2) return; // 整个 subtree 不可能更近 → 剪枝

	if (n.triCount > 0)
	{
		for (int i = 0; i < n.triCount; ++i)
		{
			const XFoam_Label triIdx = bvhOrder_[n.firstTri + i];
			const Triangle& t = tris_[triIdx];
			XFoam_Scalar d2 = 0;
			const XFoam_Vector3D q = closestPointOnTriangle(p, t.v0, t.v1, t.v2, d2);
			if (d2 < bestD2)
			{
				bestD2 = d2;
				bestQ = q;
				bestTri = triIdx;
			}
		}
		return;
	}

	// 先递更近一侧，能更早把 bestD2 拉小，提升对侧剪枝效率。
	const XFoam_Scalar lD2 = bboxMinDistSqr(p, bvhNodes_[n.leftIdx].bbox);
	const XFoam_Scalar rD2 = bboxMinDistSqr(p, bvhNodes_[n.rightIdx].bbox);
	int firstChild, secondChild;
	XFoam_Scalar firstD2, secondD2;
	if (lD2 <= rD2)
	{
		firstChild  = n.leftIdx;  firstD2  = lD2;
		secondChild = n.rightIdx; secondD2 = rD2;
	}
	else
	{
		firstChild  = n.rightIdx; firstD2  = rD2;
		secondChild = n.leftIdx;  secondD2 = lD2;
	}
	if (firstD2  < bestD2) bvhClosestPoint(p, bestD2, bestQ, bestTri, firstChild);
	if (secondD2 < bestD2) bvhClosestPoint(p, bestD2, bestQ, bestTri, secondChild);
}

int XFoam_TriSurface::bvhRayCountPlusX(const XFoam_Vector3D& p, int nodeIdx) const
{
	const BvhNode& n = bvhNodes_[nodeIdx];
	// 射线沿 +X：节点 bbox 整体在 p 左侧 → 无命中；y/z 不在节点 bbox 内 → 同样无命中。
	if (n.bbox.max().x() < p.x()) return 0;
	if (p.y() < n.bbox.min().y() || p.y() > n.bbox.max().y()) return 0;
	if (p.z() < n.bbox.min().z() || p.z() > n.bbox.max().z()) return 0;

	if (n.triCount > 0)
	{
		int count = 0;
		for (int i = 0; i < n.triCount; ++i)
		{
			const XFoam_Label triIdx = bvhOrder_[n.firstTri + i];
			const Triangle& t = tris_[triIdx];
			if (p.y() < t.bbox.min().y() || p.y() > t.bbox.max().y()) continue;
			if (p.z() < t.bbox.min().z() || p.z() > t.bbox.max().z()) continue;
			if (t.bbox.max().x() < p.x()) continue;
			if (rayHitsTrianglePlusX(p, t.v0, t.v1, t.v2)) ++count;
		}
		return count;
	}

	return bvhRayCountPlusX(p, n.leftIdx) + bvhRayCountPlusX(p, n.rightIdx);
}

bool XFoam_TriSurface::bvhBoxIntersects(const XFoam_BoundBox& q, int nodeIdx) const
{
	const BvhNode& n = bvhNodes_[nodeIdx];
	if (!q.overlaps(n.bbox)) return false;
	if (n.triCount > 0)
	{
		for (int i = 0; i < n.triCount; ++i)
		{
			const XFoam_Label triIdx = bvhOrder_[n.firstTri + i];
			if (q.overlaps(tris_[triIdx].bbox)) return true;
		}
		return false;
	}
	if (bvhBoxIntersects(q, n.leftIdx)) return true;
	return bvhBoxIntersects(q, n.rightIdx);
}

bool XFoam_TriSurface::read(const std::string& path)
{
	// 头 5 字节是不是 "solid"。binary STL 也可能以 "solid" 开头，所以二次嗅探：
	// 文件总大小 = 84 + 50 * N（N 来自字节 80..84 的 uint32）→ binary。
	std::ifstream f(path, std::ios::binary);
	if (!f) return false;
	char head[5] = {0};
	f.read(head, 5);
	if (f.gcount() < 5)
	{
		return false;
	}
	const bool startsWithSolid =
		(head[0] == 's' || head[0] == 'S') &&
		(head[1] == 'o' || head[1] == 'O') &&
		(head[2] == 'l' || head[2] == 'L') &&
		(head[3] == 'i' || head[3] == 'I') &&
		(head[4] == 'd' || head[4] == 'D');
	f.seekg(0, std::ios::end);
	const std::streamoff fileSize = f.tellg();
	f.close();
	if (startsWithSolid)
	{
		// 大小一致 → 仍可能 binary。先按 ascii 试，失败再 binary。
		if (readAsciiSTL(path) && !tris_.empty())
		{
			return true;
		}
	}
	if (fileSize >= 84)
	{
		return readBinarySTL(path);
	}
	return false;
}

bool XFoam_TriSurface::readAsciiSTL(const std::string& path)
{
	tris_.clear();
	std::ifstream f(path);
	if (!f) return false;
	std::string tok;
	XFoam_Vector3D n(0, 0, 0);
	XFoam_Vector3D v[3];
	int vi = 0;
	bool haveN = false;
	while (f >> tok)
	{
		// 大小写敏感的 STL 关键字，OF 与 ASCII STL 标准都按小写写。
		if (tok == "facet")
		{
			std::string maybeNormal;
			if (!(f >> maybeNormal)) break;
			if (maybeNormal == "normal")
			{
				f >> n.x() >> n.y() >> n.z();
				haveN = true;
			}
			continue;
		}
		if (tok == "vertex")
		{
			if (vi >= 3) { vi = 0; haveN = false; }
			f >> v[vi].x() >> v[vi].y() >> v[vi].z();
			++vi;
			if (vi == 3)
			{
				addTriangle(v[0], v[1], v[2], haveN ? &n : nullptr);
				vi = 0;
				haveN = false;
			}
		}
		// 其他关键字（outer/loop/endloop/endfacet/endsolid/solid）直接忽略
	}
	rebuildBounds();
	buildBvh();
	return !tris_.empty();
}

bool XFoam_TriSurface::readBinarySTL(const std::string& path)
{
	tris_.clear();
	std::ifstream f(path, std::ios::binary);
	if (!f) return false;
	char header[80];
	readBinaryStream(f, header, 80);
	uint32_t nTri = 0;
	readBinaryStream(f, &nTri, sizeof(nTri));
	if (!f) return false;
	for (uint32_t i = 0; i < nTri; ++i)
	{
		float vals[12];
		readBinaryStream(f, vals, sizeof(vals));
		uint16_t attr = 0;
		readBinaryStream(f, &attr, sizeof(attr));
		if (!f) break;
		const XFoam_Vector3D n(vals[0], vals[1], vals[2]);
		const XFoam_Vector3D a(vals[3], vals[4], vals[5]);
		const XFoam_Vector3D b(vals[6], vals[7], vals[8]);
		const XFoam_Vector3D c(vals[9], vals[10], vals[11]);
		addTriangle(a, b, c, &n);
	}
	rebuildBounds();
	buildBvh();
	return !tris_.empty();
}

XFoam_Scalar XFoam_TriSurface::distance(const XFoam_Vector3D& p) const
{
	if (tris_.empty()) return std::numeric_limits<XFoam_Scalar>::infinity();
	XFoam_Scalar bestD2 = std::numeric_limits<XFoam_Scalar>::infinity();
	XFoam_Vector3D bestQ;
	XFoam_Label bestTri = -1;
	if (!bvhNodes_.empty())
	{
		bvhClosestPoint(p, bestD2, bestQ, bestTri, 0);
	}
	else
	{
		for (size_t i = 0; i < tris_.size(); ++i)
		{
			XFoam_Scalar d2 = 0;
			(void)closestPointOnTriangle(p, tris_[i].v0, tris_[i].v1, tris_[i].v2, d2);
			if (d2 < bestD2) bestD2 = d2;
		}
	}
	return std::sqrt(bestD2);
}

void XFoam_TriSurface::closestPointAndNormal(
	const XFoam_Vector3D& p,
	XFoam_Vector3D& outClosest,
	XFoam_Vector3D& outNormal) const
{
	if (tris_.empty())
	{
		outClosest = p;
		outNormal = XFoam_Vector3D(0, 0, 0);
		return;
	}
	XFoam_Scalar bestD2 = std::numeric_limits<XFoam_Scalar>::infinity();
	XFoam_Vector3D bestQ = tris_[0].v0;
	XFoam_Label bestTri = 0;
	if (!bvhNodes_.empty())
	{
		bvhClosestPoint(p, bestD2, bestQ, bestTri, 0);
	}
	else
	{
		for (size_t i = 0; i < tris_.size(); ++i)
		{
			XFoam_Scalar d2 = 0;
			const XFoam_Vector3D q = closestPointOnTriangle(p, tris_[i].v0, tris_[i].v1, tris_[i].v2, d2);
			if (d2 < bestD2)
			{
				bestD2 = d2;
				bestTri = static_cast<XFoam_Label>(i);
				bestQ = q;
			}
		}
	}
	outClosest = bestQ;
	outNormal = tris_[bestTri].normal;
}

int XFoam_TriSurface::rayCountPlusX(const XFoam_Vector3D& p) const
{
	if (!bvhNodes_.empty())
	{
		return bvhRayCountPlusX(p, 0);
	}
	int count = 0;
	for (size_t i = 0; i < tris_.size(); ++i)
	{
		const Triangle& t = tris_[i];
		if (p.y() < t.bbox.min().y() || p.y() > t.bbox.max().y()) continue;
		if (p.z() < t.bbox.min().z() || p.z() > t.bbox.max().z()) continue;
		if (p.x() > t.bbox.max().x()) continue;
		if (rayHitsTrianglePlusX(p, t.v0, t.v1, t.v2))
		{
			++count;
		}
	}
	return count;
}

bool XFoam_TriSurface::contains(const XFoam_Vector3D& p) const
{
	// 把射线起点在 y/z 上相对抖动一个 bbox-相关的小量，避开 +X 射线恰好穿过
	// 三角顶点/棱时奇偶计数不一致的退化（典型如球心 (0,0,0) 对 (R,0,0) 处顶点）。
	const XFoam_Scalar sp = static_cast<XFoam_Scalar>(bounds_.mag());
	const XFoam_Scalar eps = std::max(static_cast<XFoam_Scalar>(1e-9), sp * static_cast<XFoam_Scalar>(1e-7));
	const XFoam_Vector3D pp(
		p.x(),
		p.y() + eps * static_cast<XFoam_Scalar>(0.7314159),
		p.z() + eps * static_cast<XFoam_Scalar>(0.4142135));
	return (rayCountPlusX(pp) & 1) != 0;
}

bool XFoam_TriSurface::boxIntersects(const XFoam_BoundBox& box) const
{
	// 粗筛：query bbox 与三角面 bbox overlap 即认为相交。BVH 短路退出，从根开始递归。
	if (tris_.empty()) return false;
	if (!bvhNodes_.empty()) return bvhBoxIntersects(box, 0);
	for (size_t i = 0; i < tris_.size(); ++i)
	{
		if (box.overlaps(tris_[i].bbox))
		{
			return true;
		}
	}
	return false;
}

// ------- Snap #7 feature extraction & query -------
//
// 流程：
//   1) Vertex 去重：按量化坐标 hash（精度 = bbox.span() / 1e9 量级），每个 Triangle 的
//      v0/v1/v2 映射到唯一 vertex id。
//   2) Edge → tri 邻接表：每个三角形 3 条 edge（按 sorted (vid, vid)），收集所有引用它的
//      tri 下标。
//   3) Feature 判定：
//        - 边界 edge（只 1 个 tri 引用）→ feature
//        - 非流形 edge（≥3 tri 引用）→ feature
//        - 流形 edge（恰 2 tri）→ 若 normal · normal < cos(thresh) → feature
//   4) Feature vertex：累计每个 vid 入射的 feature edge 数，≥ 3 即为 feature vertex。
//
// 复杂度：O(N_tri)；查询时 closestFeature 线性扫描 feature 数组。Feature 数远小于
// 三角面数，cylinder1 1620 tri → ~30 feature edge。
namespace
{
struct PtKeyTri
{
	int64_t x, y, z;
	bool operator==(const PtKeyTri& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct PtKeyTriHash
{
	size_t operator()(const PtKeyTri& k) const noexcept
	{
		// 三轴 64-bit 混合：移位异或，常数取自 splitmix64 spread。
		uint64_t h = static_cast<uint64_t>(k.x) * 0x9E3779B97F4A7C15ULL;
		h ^= static_cast<uint64_t>(k.y) + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
		h ^= static_cast<uint64_t>(k.z) + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
		return static_cast<size_t>(h);
	}
};
struct EdgeKeyTri
{
	int a, b; // a < b
	bool operator==(const EdgeKeyTri& o) const { return a == o.a && b == o.b; }
};
struct EdgeKeyTriHash
{
	size_t operator()(const EdgeKeyTri& k) const noexcept
	{
		return (static_cast<size_t>(k.a) * 0x9E3779B97F4A7C15ULL) ^ static_cast<size_t>(k.b);
	}
};

inline XFoam_Scalar pointSegmentDistSqr(
	const XFoam_Vector3D& p,
	const XFoam_Vector3D& a,
	const XFoam_Vector3D& b,
	XFoam_Vector3D& outClosest)
{
	const XFoam_Vector3D ab = b - a;
	const XFoam_Scalar abLen2 = ab.x() * ab.x() + ab.y() * ab.y() + ab.z() * ab.z();
	if (abLen2 <= 0)
	{
		outClosest = a;
		const XFoam_Vector3D d = p - a;
		return d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
	}
	const XFoam_Vector3D ap = p - a;
	XFoam_Scalar t = (ap.x() * ab.x() + ap.y() * ab.y() + ap.z() * ab.z()) / abLen2;
	if (t < 0) t = 0;
	if (t > 1) t = 1;
	outClosest = a + ab * t;
	const XFoam_Vector3D d = p - outClosest;
	return d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
}
} // namespace

void XFoam_TriSurface::buildFeatures(XFoam_Scalar angleDegThresh)
{
	featureEdges_.clear();
	featureVerts_.clear();
	if (tris_.empty()) return;

	// 1) Vertex 去重：量化精度按 bbox span 取。bbox 跨 1.0 时 eps ≈ 1e-9。
	const XFoam_Vector3D span(
		std::max<XFoam_Scalar>(bounds_.max().x() - bounds_.min().x(), 1),
		std::max<XFoam_Scalar>(bounds_.max().y() - bounds_.min().y(), 1),
		std::max<XFoam_Scalar>(bounds_.max().z() - bounds_.min().z(), 1));
	const XFoam_Scalar maxSpan = std::max(std::max(span.x(), span.y()), span.z());
	const XFoam_Scalar eps = maxSpan * static_cast<XFoam_Scalar>(1e-9);
	const XFoam_Scalar invEps = static_cast<XFoam_Scalar>(1) / eps;
	auto keyOf = [&](const XFoam_Vector3D& p) -> PtKeyTri {
		return {
			static_cast<int64_t>(std::llround(p.x() * invEps)),
			static_cast<int64_t>(std::llround(p.y() * invEps)),
			static_cast<int64_t>(std::llround(p.z() * invEps))
		};
	};

	std::unordered_map<PtKeyTri, int, PtKeyTriHash> vertId;
	vertId.reserve(tris_.size() * 3);
	std::vector<XFoam_Vector3D> uniqueVerts;
	uniqueVerts.reserve(tris_.size() / 2);
	auto addVert = [&](const XFoam_Vector3D& p) -> int {
		const PtKeyTri k = keyOf(p);
		auto it = vertId.find(k);
		if (it != vertId.end()) return it->second;
		const int id = static_cast<int>(uniqueVerts.size());
		uniqueVerts.push_back(p);
		vertId.emplace(k, id);
		return id;
	};

	std::vector<std::array<int, 3>> triVids(tris_.size());
	for (size_t ti = 0; ti < tris_.size(); ++ti)
	{
		triVids[ti][0] = addVert(tris_[ti].v0);
		triVids[ti][1] = addVert(tris_[ti].v1);
		triVids[ti][2] = addVert(tris_[ti].v2);
	}

	// 2) Edge → tri 邻接表。每条 edge 用 sorted (vid, vid) 键。
	std::unordered_map<EdgeKeyTri, std::vector<int>, EdgeKeyTriHash> edgeTris;
	edgeTris.reserve(tris_.size() * 3);
	for (size_t ti = 0; ti < tris_.size(); ++ti)
	{
		for (int e = 0; e < 3; ++e)
		{
			int a = triVids[ti][e];
			int b = triVids[ti][(e + 1) % 3];
			if (a > b) std::swap(a, b);
			edgeTris[{a, b}].push_back(static_cast<int>(ti));
		}
	}

	// 3) Feature 判定：开边 / 非流形 / 折角。
	// MSVC 未默认定义 M_PI；直接用 3.14159... / 180 常量。
	const XFoam_Scalar cosThresh = std::cos(
		angleDegThresh * static_cast<XFoam_Scalar>(3.14159265358979323846 / 180.0));
	std::unordered_map<int, int> vertFeatDegree;
	for (const auto& kv : edgeTris)
	{
		const EdgeKeyTri& key = kv.first;
		const std::vector<int>& ts = kv.second;
		bool isFeature = false;
		if (ts.size() == 1) isFeature = true;
		else if (ts.size() >= 3) isFeature = true;
		else
		{
			const XFoam_Vector3D& n1 = tris_[ts[0]].normal;
			const XFoam_Vector3D& n2 = tris_[ts[1]].normal;
			const XFoam_Scalar dot = n1.x() * n2.x() + n1.y() * n2.y() + n1.z() * n2.z();
			// dot 小于 cosThresh 即夹角大于 thresh：cos 单调下降。法向已是单位向量，无需 normalize。
			if (dot < cosThresh) isFeature = true;
		}
		if (isFeature)
		{
			FeatureEdge fe;
			fe.p1 = uniqueVerts[key.a];
			fe.p2 = uniqueVerts[key.b];
			featureEdges_.push_back(fe);
			++vertFeatDegree[key.a];
			++vertFeatDegree[key.b];
		}
	}

	// 4) Feature vertex：≥3 条入射 feature edge 视为尖角。
	for (const auto& kv : vertFeatDegree)
	{
		if (kv.second >= 3)
		{
			FeatureVertex fv;
			fv.p = uniqueVerts[kv.first];
			featureVerts_.push_back(fv);
		}
	}
}

XFoam_TriSurface::FeatureKind XFoam_TriSurface::closestFeature(
	const XFoam_Vector3D& p,
	XFoam_Scalar searchRadius,
	XFoam_Vector3D& outClosest) const
{
	if (featureEdges_.empty() && featureVerts_.empty()) return FeatureKind::None;
	const XFoam_Scalar r2 = searchRadius * searchRadius;
	XFoam_Scalar bestD2 = r2;
	FeatureKind bestKind = FeatureKind::None;
	XFoam_Vector3D bestQ;

	// 先扫 feature vertex：尖角优先（更典型的 snap 目标）。
	for (size_t i = 0; i < featureVerts_.size(); ++i)
	{
		const XFoam_Vector3D& fp = featureVerts_[i].p;
		const XFoam_Vector3D d = fp - p;
		const XFoam_Scalar d2 = d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
		if (d2 < bestD2)
		{
			bestD2 = d2;
			bestKind = FeatureKind::Vertex;
			bestQ = fp;
		}
	}
	// 再扫 feature edge。若已经在 vertex 半径内，可能仍被 edge 进一步逼近 → 都查。
	for (size_t i = 0; i < featureEdges_.size(); ++i)
	{
		XFoam_Vector3D q;
		const XFoam_Scalar d2 = pointSegmentDistSqr(p, featureEdges_[i].p1, featureEdges_[i].p2, q);
		if (d2 < bestD2)
		{
			bestD2 = d2;
			bestKind = FeatureKind::Edge;
			bestQ = q;
		}
	}
	if (bestKind != FeatureKind::None) outClosest = bestQ;
	return bestKind;
}
