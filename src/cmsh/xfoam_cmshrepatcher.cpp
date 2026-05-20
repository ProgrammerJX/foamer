#include "XFoam/cmsh/xfoam_cmshrepatcher.h"

#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <iostream>
#include <unordered_set>
#include <vector>

XFoam_CMshRepatcher::XFoam_CMshRepatcher(
	XFoam_CMshPolyMeshGen& pm, const XFoam_BrepBase& brep, const Params& p)
	: pm_(pm), brep_(brep), p_(p)
{}

XFoam_CMshRepatcher::Stats XFoam_CMshRepatcher::repatch()
{
	Stats st;
	st.nPatchesBefore = static_cast<XFoam_Label>(pm_.patches.size());
	if (brep_.empty()) return st;

	const int nInt   = pm_.nInternalFaces();
	const int nAll   = pm_.nFaces();
	const int nBnd   = nAll - nInt;
	st.nBoundaryFaces = static_cast<XFoam_Label>(nBnd);
	if (nBnd <= 0)
	{
		pm_.patches.clear();
		return st;
	}

	// 1) 对每张 boundary face 算 post-mapped centroid → newSub
	std::vector<int> newSub(static_cast<std::size_t>(nBnd), -1);
	std::vector<int> oldSub(static_cast<std::size_t>(nBnd), -1);

	// 把当前 pm.patches[] 的归属反查回每个 boundary face（用于统计 reassigned）
	for (const auto& P : pm_.patches)
	{
		for (int i = 0; i < P.nFaces; ++i)
		{
			const int fi = P.startFace + i;
			if (fi >= nInt && fi < nAll)
			{
				oldSub[static_cast<std::size_t>(fi - nInt)] = -2; // 占位，后面再 map
			}
		}
	}
	// 给 oldSub 真正塞 sub-id：按 patch 的名字 → "sub<id>" 解析；name 不 match
	// 则记 -1（表示之前归属未知）。
	{
		int faceIdx = 0;
		for (const auto& P : pm_.patches)
		{
			int sub = -1;
			// 尝试反解：先匹配 brep 的 subPatchName(id) == P.name
			const XFoam_Label nSub = brep_.nSubPatches();
			for (XFoam_Label s = 0; s < nSub; ++s)
			{
				if (static_cast<const std::string&>(brep_.subPatchName(s)) == P.name)
				{
					sub = static_cast<int>(s);
					break;
				}
			}
			for (int i = 0; i < P.nFaces; ++i)
			{
				const int fi = P.startFace + i;
				if (fi >= nInt && fi < nAll)
				{
					oldSub[static_cast<std::size_t>(fi - nInt)] = sub;
				}
			}
			(void) faceIdx;
		}
	}

	for (int bi = 0; bi < nBnd; ++bi)
	{
		const int fi = nInt + bi;
		const auto& verts = pm_.faces[static_cast<std::size_t>(fi)].verts;
		if (verts.empty()) continue;
		XFoam_Vector3D c(0, 0, 0);
		for (int v : verts) c += pm_.points[static_cast<std::size_t>(v)];
		c *= (XFoam_Scalar(1) / static_cast<XFoam_Scalar>(verts.size()));
		const XFoam_Label s = brep_.closestSubPatchId(c);
		newSub[static_cast<std::size_t>(bi)] = static_cast<int>(s);
		if (oldSub[static_cast<std::size_t>(bi)] != static_cast<int>(s))
		{
			++st.nReassigned;
		}
	}

	// 2) 按 (newSub, owner) 排序得到 reorderedBnd（保持 sub 内 owner 升序）
	std::vector<int> order(static_cast<std::size_t>(nBnd));
	for (int i = 0; i < nBnd; ++i) order[static_cast<std::size_t>(i)] = i;
	std::sort(order.begin(), order.end(),
		[&](int a, int b) {
			if (newSub[static_cast<std::size_t>(a)] != newSub[static_cast<std::size_t>(b)])
				return newSub[static_cast<std::size_t>(a)] < newSub[static_cast<std::size_t>(b)];
			return pm_.owner[static_cast<std::size_t>(nInt + a)] < pm_.owner[static_cast<std::size_t>(nInt + b)];
		});

	// 3) 重写 boundary face 段 + owner
	std::vector<XFoam_CMshPolyMeshGen::Face> newBnd(static_cast<std::size_t>(nBnd));
	std::vector<int> newOwner(static_cast<std::size_t>(nBnd));
	for (int i = 0; i < nBnd; ++i)
	{
		const int src = order[static_cast<std::size_t>(i)];
		newBnd[static_cast<std::size_t>(i)]   = std::move(pm_.faces[static_cast<std::size_t>(nInt + src)]);
		newOwner[static_cast<std::size_t>(i)] = pm_.owner[static_cast<std::size_t>(nInt + src)];
	}
	for (int i = 0; i < nBnd; ++i)
	{
		pm_.faces[static_cast<std::size_t>(nInt + i)] = std::move(newBnd[static_cast<std::size_t>(i)]);
		pm_.owner[static_cast<std::size_t>(nInt + i)] = newOwner[static_cast<std::size_t>(i)];
	}

	// 4) 重写 patches[]
	pm_.patches.clear();
	std::unordered_set<int> hitSub;
	int i = 0;
	while (i < nBnd)
	{
		int j = i;
		const int curSub = newSub[static_cast<std::size_t>(order[static_cast<std::size_t>(i)])];
		while (j < nBnd && newSub[static_cast<std::size_t>(order[static_cast<std::size_t>(j)])] == curSub) ++j;

		XFoam_CMshPolyMeshGen::Patch P;
		std::string name;
		if (curSub >= 0)
		{
			const XFoam_String sn = brep_.subPatchName(static_cast<XFoam_Label>(curSub));
			name = static_cast<const std::string&>(sn);
			hitSub.insert(curSub);
		}
		if (name.empty()) name = "sub" + std::to_string(curSub < 0 ? 0 : curSub);
		P.name      = name;
		P.type      = p_.defaultPatchType;
		P.startFace = nInt + i;
		P.nFaces    = j - i;
		pm_.patches.push_back(std::move(P));
		i = j;
	}

	if (p_.fillAllSubPatches)
	{
		const XFoam_Label nSub = brep_.nSubPatches();
		const int tailStart = nInt + nBnd;
		for (XFoam_Label s = 0; s < nSub; ++s)
		{
			if (hitSub.find(static_cast<int>(s)) != hitSub.end()) continue;
			XFoam_CMshPolyMeshGen::Patch P;
			const XFoam_String sn = brep_.subPatchName(s);
			std::string name = static_cast<const std::string&>(sn);
			if (name.empty()) name = "sub" + std::to_string(s);
			P.name      = name;
			P.type      = p_.defaultPatchType;
			P.startFace = tailStart;
			P.nFaces    = 0;
			pm_.patches.push_back(std::move(P));
			++st.nEmptyAdded;
		}
	}

	st.nPatchesAfter = static_cast<XFoam_Label>(pm_.patches.size());

	if (p_.verbose)
	{
		std::cout << "  repatcher: nBnd=" << st.nBoundaryFaces
		          << "  reassigned=" << st.nReassigned
		          << "  patches " << st.nPatchesBefore << " -> " << st.nPatchesAfter
		          << "  (empty added " << st.nEmptyAdded << ")\n";
	}
	return st;
}
