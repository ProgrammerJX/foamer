#include "XFoam/cmsh/xfoam_cmshsurfaceengine.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

namespace
{
inline std::uint64_t edgeKey(int a, int b)
{
	if (a > b) std::swap(a, b);
	return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32)
	     | static_cast<std::uint32_t>(b);
}
} // namespace

XFoam_CMshSurfaceEngine::XFoam_CMshSurfaceEngine(const XFoam_CMshPolyMeshGen& pm)
	: pm_(pm)
{
	buildFaceMeta();
	buildBndPoints();
	buildFaceGeometry();
	buildPointAdjacency();
	buildEdges();
}

void XFoam_CMshSurfaceEngine::buildFaceMeta()
{
	const int nInt = pm_.nInternalFaces();
	const int nF   = pm_.nFaces();
	const int nBnd = nF - nInt;
	bndFaceIds_.resize(static_cast<std::size_t>(nBnd));
	facePatch_.assign(static_cast<std::size_t>(nBnd), -1);
	faceOwner_.assign(static_cast<std::size_t>(nBnd), -1);
	for (int i = 0; i < nBnd; ++i)
	{
		bndFaceIds_[static_cast<std::size_t>(i)] = nInt + i;
		if (nInt + i < static_cast<int>(pm_.owner.size()))
			faceOwner_[static_cast<std::size_t>(i)] = pm_.owner[static_cast<std::size_t>(nInt + i)];
	}
	// patch id：扫 pm.patches，凡 [startFace, startFace+nFaces) 内的归属之
	for (std::size_t pi = 0; pi < pm_.patches.size(); ++pi)
	{
		const auto& P = pm_.patches[pi];
		for (int k = 0; k < P.nFaces; ++k)
		{
			const int fi = P.startFace + k;
			const int idx = fi - nInt;
			if (idx >= 0 && idx < nBnd) facePatch_[static_cast<std::size_t>(idx)] = static_cast<int>(pi);
		}
	}
}

void XFoam_CMshSurfaceEngine::buildBndPoints()
{
	bp_.assign(pm_.points.size(), -1);
	bndPoints_.clear();
	bndPoints_.reserve(pm_.points.size() / 4);
	for (int fi : bndFaceIds_)
	{
		for (int v : pm_.faces[static_cast<std::size_t>(fi)].verts)
		{
			if (v < 0 || v >= static_cast<int>(bp_.size())) continue;
			if (bp_[static_cast<std::size_t>(v)] != -1) continue;
			bp_[static_cast<std::size_t>(v)] = static_cast<int>(bndPoints_.size());
			bndPoints_.push_back(v);
		}
	}
	// 排序后修正 bp_（保证 bndPoints_ 升序）
	std::sort(bndPoints_.begin(), bndPoints_.end());
	for (std::size_t i = 0; i < bndPoints_.size(); ++i)
		bp_[static_cast<std::size_t>(bndPoints_[i])] = static_cast<int>(i);
}

void XFoam_CMshSurfaceEngine::buildFaceGeometry()
{
	const std::size_t n = bndFaceIds_.size();
	faceCentres_.assign(n, XFoam_Vector3D(0, 0, 0));
	faceNormals_.assign(n, XFoam_Vector3D(0, 0, 0));
	faceAreas_.assign(n, 0);
	for (std::size_t i = 0; i < n; ++i)
	{
		const auto& f = pm_.faces[static_cast<std::size_t>(bndFaceIds_[i])];
		const std::size_t nv = f.verts.size();
		if (nv < 3) continue;
		// centroid（顶点平均，多边形足够，cfMesh 也这么做）
		XFoam_Vector3D c(0, 0, 0);
		for (int v : f.verts) c += pm_.points[static_cast<std::size_t>(v)];
		c *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(nv));
		faceCentres_[i] = c;
		// Newell normal + area
		XFoam_Vector3D N(0, 0, 0);
		for (std::size_t k = 0; k < nv; ++k)
		{
			const auto& a = pm_.points[static_cast<std::size_t>(f.verts[k])];
			const auto& b = pm_.points[static_cast<std::size_t>(f.verts[(k + 1) % nv])];
			N += XFoam_Vector3D(
				(a.y() - b.y()) * (a.z() + b.z()),
				(a.z() - b.z()) * (a.x() + b.x()),
				(a.x() - b.x()) * (a.y() + b.y()));
		}
		const XFoam_Scalar mag = N.mag();
		faceAreas_[i] = static_cast<XFoam_Scalar>(0.5) * mag;
		if (mag > 0) faceNormals_[i] = N * (XFoam_Scalar(1) / mag);
	}
}

void XFoam_CMshSurfaceEngine::buildPointAdjacency()
{
	const std::size_t nBnd = bndPoints_.size();
	pointFaces_.assign(nBnd, {});
	std::vector<std::unordered_set<int>> ppTmp(nBnd);
	std::vector<std::unordered_set<int>> ptchTmp(nBnd);

	for (std::size_t bfi = 0; bfi < bndFaceIds_.size(); ++bfi)
	{
		const int fi = bndFaceIds_[bfi];
		const auto& f = pm_.faces[static_cast<std::size_t>(fi)];
		const int pa = facePatch_[bfi];
		for (std::size_t k = 0; k < f.verts.size(); ++k)
		{
			const int v = f.verts[k];
			if (v < 0 || v >= static_cast<int>(bp_.size())) continue;
			const int li = bp_[static_cast<std::size_t>(v)];
			if (li < 0) continue;
			pointFaces_[static_cast<std::size_t>(li)].push_back(static_cast<int>(bfi));
			if (pa >= 0) ptchTmp[static_cast<std::size_t>(li)].insert(pa);
			// neighbours: 同面 prev + next vert
			const std::size_t n = f.verts.size();
			const int vNext = f.verts[(k + 1) % n];
			const int vPrev = f.verts[(k + n - 1) % n];
			if (vNext >= 0)
			{
				const int ln = bp_[static_cast<std::size_t>(vNext)];
				if (ln >= 0) ppTmp[static_cast<std::size_t>(li)].insert(ln);
			}
			if (vPrev >= 0)
			{
				const int lp = bp_[static_cast<std::size_t>(vPrev)];
				if (lp >= 0) ppTmp[static_cast<std::size_t>(li)].insert(lp);
			}
		}
	}
	pointPoints_.assign(nBnd, {});
	pointPatches_.assign(nBnd, {});
	for (std::size_t li = 0; li < nBnd; ++li)
	{
		pointPoints_[li].assign(ppTmp[li].begin(), ppTmp[li].end());
		pointPatches_[li].assign(ptchTmp[li].begin(), ptchTmp[li].end());
	}

	// point normal = sum(faceNormal) over incident bndFaces，再单位化
	pointNormals_.assign(nBnd, XFoam_Vector3D(0, 0, 0));
	for (std::size_t li = 0; li < nBnd; ++li)
	{
		XFoam_Vector3D n(0, 0, 0);
		for (int bfi : pointFaces_[li]) n += faceNormals_[static_cast<std::size_t>(bfi)];
		const XFoam_Scalar mg = n.mag();
		if (mg > 0) pointNormals_[li] = n * (XFoam_Scalar(1) / mg);
	}
}

void XFoam_CMshSurfaceEngine::buildEdges()
{
	const std::size_t nBnd = bndPoints_.size();
	std::unordered_map<std::uint64_t, int> keyToEdge;
	keyToEdge.reserve(nBnd * 3);

	faceEdges_.assign(bndFaceIds_.size(), {});
	for (std::size_t bfi = 0; bfi < bndFaceIds_.size(); ++bfi)
	{
		const int fi = bndFaceIds_[bfi];
		const auto& f = pm_.faces[static_cast<std::size_t>(fi)];
		const std::size_t n = f.verts.size();
		auto& fe = faceEdges_[bfi];
		fe.resize(n, -1);
		for (std::size_t k = 0; k < n; ++k)
		{
			const int a = f.verts[k];
			const int b = f.verts[(k + 1) % n];
			if (a < 0 || b < 0 || a == b) continue;
			const auto key = edgeKey(a, b);
			auto it = keyToEdge.find(key);
			int eid;
			if (it == keyToEdge.end())
			{
				eid = static_cast<int>(edges_.size());
				edges_.emplace_back(std::min(a, b), std::max(a, b));
				keyToEdge.emplace(key, eid);
			}
			else
			{
				eid = it->second;
			}
			fe[k] = eid;
		}
	}

	const std::size_t nE = edges_.size();
	edgeFaces_.assign(nE, {});
	edgePatches_.assign(nE, {});
	std::vector<std::unordered_set<int>> epTmp(nE);
	for (std::size_t bfi = 0; bfi < bndFaceIds_.size(); ++bfi)
	{
		const int pa = facePatch_[bfi];
		for (int eid : faceEdges_[bfi])
		{
			if (eid < 0) continue;
			edgeFaces_[static_cast<std::size_t>(eid)].push_back(static_cast<int>(bfi));
			if (pa >= 0) epTmp[static_cast<std::size_t>(eid)].insert(pa);
		}
	}
	for (std::size_t e = 0; e < nE; ++e)
		edgePatches_[e].assign(epTmp[e].begin(), epTmp[e].end());

	// bpEdges_：bnd-local point id → edge ids
	bpEdges_.assign(nBnd, {});
	for (std::size_t e = 0; e < nE; ++e)
	{
		const auto& E = edges_[e];
		const int la = bp_[static_cast<std::size_t>(E.v0)];
		const int lb = bp_[static_cast<std::size_t>(E.v1)];
		if (la >= 0) bpEdges_[static_cast<std::size_t>(la)].push_back(static_cast<int>(e));
		if (lb >= 0) bpEdges_[static_cast<std::size_t>(lb)].push_back(static_cast<int>(e));
	}
}
