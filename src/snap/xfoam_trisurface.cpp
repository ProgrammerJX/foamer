#include "XFoam/snap/xfoam_trisurface.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

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
	return !tris_.empty();
}

XFoam_Scalar XFoam_TriSurface::distance(const XFoam_Vector3D& p) const
{
	if (tris_.empty()) return std::numeric_limits<XFoam_Scalar>::infinity();
	XFoam_Scalar bestD2 = std::numeric_limits<XFoam_Scalar>::infinity();
	for (size_t i = 0; i < tris_.size(); ++i)
	{
		XFoam_Scalar d2 = 0;
		(void)closestPointOnTriangle(p, tris_[i].v0, tris_[i].v1, tris_[i].v2, d2);
		if (d2 < bestD2) bestD2 = d2;
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
	size_t bestI = 0;
	XFoam_Vector3D bestQ = tris_[0].v0;
	for (size_t i = 0; i < tris_.size(); ++i)
	{
		XFoam_Scalar d2 = 0;
		const XFoam_Vector3D q = closestPointOnTriangle(p, tris_[i].v0, tris_[i].v1, tris_[i].v2, d2);
		if (d2 < bestD2)
		{
			bestD2 = d2;
			bestI = i;
			bestQ = q;
		}
	}
	outClosest = bestQ;
	outNormal = tris_[bestI].normal;
}

int XFoam_TriSurface::rayCountPlusX(const XFoam_Vector3D& p) const
{
	int count = 0;
	for (size_t i = 0; i < tris_.size(); ++i)
	{
		const Triangle& t = tris_[i];
		// 粗筛：射线沿 +X，y/z 必须在 tri bbox 内。
		if (p.y() < t.bbox.min().y() || p.y() > t.bbox.max().y()) continue;
		if (p.z() < t.bbox.min().z() || p.z() > t.bbox.max().z()) continue;
		if (p.x() > t.bbox.max().x()) continue; // 三角形全在 p 左侧
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
	// 粗筛：bbox 与三角面 bbox overlap 即认为相交。够细化阶段用。
	for (size_t i = 0; i < tris_.size(); ++i)
	{
		if (box.overlaps(tris_[i].bbox))
		{
			return true;
		}
	}
	return false;
}
