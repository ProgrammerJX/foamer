#include "XFoam/cmsh/xfoam_cmshedgeinserter.h"

#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
inline std::uint64_t edgeKey(int a, int b)
{
	if (a > b) std::swap(a, b);
	return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32)
	     | static_cast<std::uint32_t>(b);
}
} // namespace

XFoam_CMshEdgeInserter::XFoam_CMshEdgeInserter(
	XFoam_CMshPolyMeshGen& pm,
	const XFoam_BrepBase&  brep,
	const Params&          p)
	: pm_(pm)
	, brep_(brep)
	, p_(p)
{}

XFoam_CMshEdgeInserter::Stats XFoam_CMshEdgeInserter::insert()
{
	Stats st;
	newPts_.clear();

	if (brep_.empty() || pm_.points.empty() || pm_.faces.empty()) return st;

	XFoam_Scalar R = p_.searchRadius;
	if (R <= 0)
	{
		R = (p_.cellSizeHint > 0)
			? p_.cellSizeHint
			: static_cast<XFoam_Scalar>(1.0);
	}

	const XFoam_Label nInt = pm_.nInternalFaces();
	const XFoam_Label nF   = pm_.nFaces();

	// 1) 收 boundary point + 给每个 boundary point 查一次 subPatchId（cache）
	std::unordered_map<int, XFoam_Label> ptSub;
	ptSub.reserve(pm_.points.size() / 4);
	for (XFoam_Label fi = nInt; fi < nF; ++fi)
	{
		for (int v : pm_.faces[static_cast<std::size_t>(fi)].verts)
		{
			if (v < 0) continue;
			if (ptSub.find(v) != ptSub.end()) continue;
			const XFoam_Vector3D& p = pm_.points[static_cast<std::size_t>(v)];
			ptSub.emplace(v, brep_.closestSubPatchId(p));
		}
	}

	// 2) 扫所有 boundary edge：(faceIdx, slot) -> edgeKey
	// 同 key 多 face 共享，新点共用。
	struct EdgeRec {
		int  faceA = -1, slotA = -1; ///< 第一个引用 face / slot
		bool seen  = false;
		int  newVid = -1;            ///< 已插入的新 mesh point；-1 表示未插
	};
	std::unordered_map<std::uint64_t, EdgeRec> edges;
	edges.reserve(pm_.points.size());

	for (XFoam_Label fi = nInt; fi < nF; ++fi)
	{
		const auto& verts = pm_.faces[static_cast<std::size_t>(fi)].verts;
		const std::size_t n = verts.size();
		if (n < 3) continue;
		for (std::size_t k = 0; k < n; ++k)
		{
			const int a = verts[k];
			const int b = verts[(k + 1) % n];
			if (a < 0 || b < 0 || a == b) continue;
			const auto key = edgeKey(a, b);
			auto it = edges.find(key);
			if (it == edges.end())
			{
				EdgeRec r; r.faceA = static_cast<int>(fi); r.slotA = static_cast<int>(k); r.seen = false;
				edges.emplace(key, r);
			}
		}
	}
	st.nBoundaryEdges = static_cast<XFoam_Label>(edges.size());

	// 3) 对每条 boundary edge：端点跨 patch → 投影 → 插点
	for (auto& kv : edges)
	{
		const std::uint64_t key = kv.first;
		EdgeRec& rec = kv.second;
		const int a = static_cast<int>(key >> 32);
		const int b = static_cast<int>(key & 0xFFFFFFFFu);
		const auto itA = ptSub.find(a);
		const auto itB = ptSub.find(b);
		if (itA == ptSub.end() || itB == ptSub.end()) continue;
		if (itA->second == itB->second) continue; // 同 patch，跳过
		++st.nCrossPatch;

		const XFoam_Vector3D& pa = pm_.points[static_cast<std::size_t>(a)];
		const XFoam_Vector3D& pb = pm_.points[static_cast<std::size_t>(b)];
		const XFoam_Vector3D mid = (pa + pb) * static_cast<XFoam_Scalar>(0.5);

		XFoam_Vector3D q, t;
		const auto kind = brep_.closestFeature(mid, R, q, t);
		bool good = false;
		if (kind == XFoam_BrepBase::FeatureKind::Edge
		    || kind == XFoam_BrepBase::FeatureKind::Vertex)
		{
			good = true;
		}
		else if (!p_.requireEdgeFeature)
		{
			// 退化到表面投影
			XFoam_Vector3D nrm;
			brep_.closestPointAndNormal(mid, q, nrm);
			if (std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()))
				good = true;
		}
		if (!good) { ++st.nProjFail; continue; }

		const XFoam_Scalar dist = (q - mid).mag();
		if (dist > st.maxProjDist) st.maxProjDist = dist;

		const int newVid = static_cast<int>(pm_.points.size());
		pm_.points.push_back(q);
		newPts_.push_back(newVid);
		rec.newVid = newVid;
		++st.nInserted;
		++st.nNewPoints;
	}

	if (st.nInserted == 0)
	{
		if (p_.verbose)
		{
			std::cout << "  edgeInserter: boundaryEdges=" << st.nBoundaryEdges
			          << "  crossPatch=" << st.nCrossPatch
			          << "  inserted=0 (nothing to do)\n";
		}
		return st;
	}

	// 4) 应用：扫所有 face，把每条带 newVid 的 edge 切开
	for (XFoam_Label fi = 0; fi < nF; ++fi)
	{
		auto& verts = pm_.faces[static_cast<std::size_t>(fi)].verts;
		const std::size_t n = verts.size();
		if (n < 3) continue;
		std::vector<int> nv;
		nv.reserve(n + 4);
		bool changed = false;
		for (std::size_t k = 0; k < n; ++k)
		{
			const int a = verts[k];
			const int b = verts[(k + 1) % n];
			nv.push_back(a);
			if (a < 0 || b < 0 || a == b) continue;
			const auto it = edges.find(edgeKey(a, b));
			if (it == edges.end()) continue;
			if (it->second.newVid < 0) continue;
			nv.push_back(it->second.newVid);
			changed = true;
		}
		if (changed)
		{
			verts = std::move(nv);
			++st.nFacesGrown;
		}
	}

	if (p_.verbose)
	{
		std::cout << "  edgeInserter: boundaryEdges=" << st.nBoundaryEdges
		          << "  crossPatch=" << st.nCrossPatch
		          << "  inserted=" << st.nInserted
		          << "  projFail=" << st.nProjFail
		          << "  facesGrown=" << st.nFacesGrown
		          << "  newPoints=" << st.nNewPoints
		          << "  maxProjDist=" << st.maxProjDist
		          << "  R=" << R << "\n";
	}

	return st;
}
