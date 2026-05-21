#include "XFoam/cmsh/xfoam_cmshfeaturepinner.h"

#include "XFoam/cmsh/xfoam_cmshsurfaceengine.h"
#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>

XFoam_CMshFeaturePinner::XFoam_CMshFeaturePinner(
	XFoam_CMshPolyMeshGen& pm,
	const XFoam_BrepBase&  brep,
	const Params&          p)
	: pm_(pm)
	, brep_(brep)
	, p_(p)
{}

XFoam_CMshFeaturePinner::XFoam_CMshFeaturePinner(
	XFoam_CMshPolyMeshGen&         pm,
	const XFoam_BrepBase&          brep,
	const Params&                  p,
	const XFoam_CMshSurfaceEngine& se)
	: pm_(pm)
	, brep_(brep)
	, p_(p)
	, bndPointsCache_(se.bndPoints())
	, haveBndCache_(true)
{}

XFoam_CMshFeaturePinner::Stats XFoam_CMshFeaturePinner::pin()
{
	Stats st;
	pinnedPoints_.clear();

	const XFoam_Label nTp = brep_.nFeatureVertices();
	st.nTpVerts = nTp;
	if (nTp == 0 || pm_.points.empty()) return st;

	// 计算 pinRadius：默认 = 2 * cellSizeHint，若 hint 也 0 则 root.diag * 1e-3
	XFoam_Scalar pinR = p_.pinRadius;
	if (pinR <= 0)
	{
		if (p_.cellSizeHint > 0) pinR = p_.cellSizeHint * static_cast<XFoam_Scalar>(2.0);
		else
		{
			// 走 mesh bbox 估个尺度
			XFoam_Vector3D mn = pm_.points[0], mx = pm_.points[0];
			for (const auto& q : pm_.points)
			{
				if (q.x() < mn.x()) mn.x() = q.x();
				if (q.y() < mn.y()) mn.y() = q.y();
				if (q.z() < mn.z()) mn.z() = q.z();
				if (q.x() > mx.x()) mx.x() = q.x();
				if (q.y() > mx.y()) mx.y() = q.y();
				if (q.z() > mx.z()) mx.z() = q.z();
			}
			const XFoam_Vector3D d = mx - mn;
			pinR = d.mag() * static_cast<XFoam_Scalar>(1e-3);
		}
	}

	std::vector<int> bndPts;
	if (haveBndCache_)
	{
		bndPts = bndPointsCache_;
	}
	else
	{
		// fallback: 扫 pm.faces 自己建（旧路径）
		std::vector<char> isBnd(pm_.points.size(), 0);
		const XFoam_Label nInt = pm_.nInternalFaces();
		const XFoam_Label nF   = pm_.nFaces();
		for (XFoam_Label fi = nInt; fi < nF; ++fi)
		{
			for (int v : pm_.faces[static_cast<std::size_t>(fi)].verts)
			{
				if (v >= 0 && v < static_cast<int>(isBnd.size())) isBnd[static_cast<std::size_t>(v)] = 1;
			}
		}
		bndPts.reserve(pm_.points.size() / 4);
		for (std::size_t i = 0; i < isBnd.size(); ++i)
		{
			if (isBnd[i]) bndPts.push_back(static_cast<int>(i));
		}
	}
	if (bndPts.empty())
	{
		if (p_.verbose) std::cout << "  pinner: no boundary points\n";
		return st;
	}

	// 每个 mesh 点最多被一个 TpVertex 占用：用 owner[meshPt] → 最近 TpVertex id
	// + bestDist。多个 TpVertex 抢同一个 mesh 点时，最近的赢，其它 ++nConflictSkipped。
	struct PinClaim { XFoam_Label tpId = -1; XFoam_Scalar d2 = 0; };
	std::unordered_map<int, PinClaim> owner;
	owner.reserve(static_cast<std::size_t>(nTp));

	const XFoam_Scalar pinR2 = pinR * pinR;

	for (XFoam_Label tpId = 0; tpId < nTp; ++tpId)
	{
		const XFoam_Vector3D v = brep_.featureVertexPosition(tpId);
		XFoam_Scalar bestD2 = std::numeric_limits<XFoam_Scalar>::infinity();
		int bestPt = -1;
		for (int pid : bndPts)
		{
			const XFoam_Vector3D dp = pm_.points[static_cast<std::size_t>(pid)] - v;
			const XFoam_Scalar d2 = dp.magSqr();
			if (d2 < bestD2) { bestD2 = d2; bestPt = pid; }
		}
		if (bestPt < 0 || bestD2 > pinR2) { ++st.nOutOfRange; continue; }

		auto it = owner.find(bestPt);
		if (it == owner.end())
		{
			owner.emplace(bestPt, PinClaim{tpId, bestD2});
		}
		else if (bestD2 < it->second.d2)
		{
			++st.nConflictSkipped;
			it->second = PinClaim{tpId, bestD2};
		}
		else
		{
			++st.nConflictSkipped;
		}
	}

	// 把 owner 里的赢家应用到 pm.points
	pinnedPoints_.reserve(owner.size());
	for (const auto& kv : owner)
	{
		const int meshPt = kv.first;
		const XFoam_Vector3D v = brep_.featureVertexPosition(kv.second.tpId);
		const XFoam_Vector3D shift = v - pm_.points[static_cast<std::size_t>(meshPt)];
		const XFoam_Scalar shiftMag = shift.mag();
		if (shiftMag > st.maxPinShift) st.maxPinShift = shiftMag;
		pm_.points[static_cast<std::size_t>(meshPt)] = v;
		pinnedPoints_.push_back(meshPt);
		++st.nPinned;
	}
	std::sort(pinnedPoints_.begin(), pinnedPoints_.end());

	if (p_.verbose)
	{
		std::cout << "  pinner: TpVerts=" << st.nTpVerts
		          << "  pinned=" << st.nPinned
		          << "  oor=" << st.nOutOfRange
		          << "  conflict=" << st.nConflictSkipped
		          << "  pinR=" << pinR
		          << "  maxShift=" << st.maxPinShift << "\n";
	}
	return st;
}
