#include "XFoam/cmsh/xfoam_cmshedgeinserter.h"

#include "XFoam/cmsh/xfoam_cmshsurfaceengine.h"
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

XFoam_CMshEdgeInserter::XFoam_CMshEdgeInserter(
	XFoam_CMshPolyMeshGen&         pm,
	const XFoam_BrepBase&          brep,
	const Params&                  p,
	const XFoam_CMshSurfaceEngine& se)
	: pm_(pm)
	, brep_(brep)
	, p_(p)
	, se_(&se)
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

	// edgeKey → 已插入的 newVid (-1 = 跨 patch 但未成功插) 或 不存在 = 同
	// patch / 同 sub-patch。 用于第二步面切割时 O(1) 查询。
	std::unordered_map<std::uint64_t, int> keyToNew;

	if (se_)
	{
		// ===== 快路径：复用 SurfaceEngine 的 edges + edgePatches =====
		// pm.patches 与 brep sub-patch 一一对应（perFacePatches=true 时）。
		const auto& engEdges    = se_->edges();
		const auto& edgePatches = se_->edgePatches();
		st.nBoundaryEdges = static_cast<XFoam_Label>(engEdges.size());
		keyToNew.reserve(engEdges.size() / 4);

		for (std::size_t e = 0; e < engEdges.size(); ++e)
		{
			if (edgePatches[e].size() < 2) continue; // 同 patch → 同一 TpFace
			++st.nCrossPatch;
			const int a = engEdges[e].v0;
			const int b = engEdges[e].v1;
			const XFoam_Vector3D& pa = pm_.points[static_cast<std::size_t>(a)];
			const XFoam_Vector3D& pb = pm_.points[static_cast<std::size_t>(b)];
			const XFoam_Vector3D mid = (pa + pb) * static_cast<XFoam_Scalar>(0.5);

			XFoam_Vector3D q, t;
			const auto kind = brep_.closestFeature(mid, R, q, t);
			bool good = (kind == XFoam_BrepBase::FeatureKind::Edge
			          || kind == XFoam_BrepBase::FeatureKind::Vertex);
			if (!good && !p_.requireEdgeFeature)
			{
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
			keyToNew.emplace(edgeKey(a, b), newVid);
			++st.nInserted;
			++st.nNewPoints;
		}
	}
	else
	{
		// ===== 旧路径：自建 boundary point → subPatchId + edge dedup =====
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
		std::unordered_map<std::uint64_t, int> seen; // dedup
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
				if (!seen.emplace(key, 0).second) continue;
			}
		}
		st.nBoundaryEdges = static_cast<XFoam_Label>(seen.size());
		for (auto& kv : seen)
		{
			const std::uint64_t key = kv.first;
			const int a = static_cast<int>(key >> 32);
			const int b = static_cast<int>(key & 0xFFFFFFFFu);
			const auto itA = ptSub.find(a);
			const auto itB = ptSub.find(b);
			if (itA == ptSub.end() || itB == ptSub.end()) continue;
			if (itA->second == itB->second) continue;
			++st.nCrossPatch;

			const XFoam_Vector3D& pa = pm_.points[static_cast<std::size_t>(a)];
			const XFoam_Vector3D& pb = pm_.points[static_cast<std::size_t>(b)];
			const XFoam_Vector3D mid = (pa + pb) * static_cast<XFoam_Scalar>(0.5);

			XFoam_Vector3D q, t;
			const auto kind = brep_.closestFeature(mid, R, q, t);
			bool good = (kind == XFoam_BrepBase::FeatureKind::Edge
			          || kind == XFoam_BrepBase::FeatureKind::Vertex);
			if (!good && !p_.requireEdgeFeature)
			{
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
			keyToNew.emplace(key, newVid);
			++st.nInserted;
			++st.nNewPoints;
		}
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

	// 4) 应用：扫所有 face（含 internal），把每条命中 keyToNew 的 edge 切开。
	//    必须扫所有 face：internal face 也共享 boundary edge 时若不切割会破环
	//    cell conformity。
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
			const auto it = keyToNew.find(edgeKey(a, b));
			if (it == keyToNew.end()) continue;
			nv.push_back(it->second);
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
