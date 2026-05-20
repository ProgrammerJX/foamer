#include "XFoam/cmsh/xfoam_cmshcartesianextractor.h"

#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
// 8 corner offsets for a cube; node order matches OpenFOAM hex 0..7
//   0: (-,-,-)  1: (+,-,-)  2: (+,+,-)  3: (-,+,-)
//   4: (-,-,+)  5: (+,-,+)  6: (+,+,+)  7: (-,+,+)
constexpr int kCornerOff[8][3] = {
	{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
	{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}
};

// 6 hex faces; node order is CCW when viewed from outside-of-cell:
//   d=0 (-x): 0,4,7,3       d=1 (+x): 1,2,6,5
//   d=2 (-y): 0,1,5,4       d=3 (+y): 3,7,6,2
//   d=4 (-z): 0,3,2,1       d=5 (+z): 4,5,6,7
constexpr int kFaceCorners[6][4] = {
	{0, 4, 7, 3}, {1, 2, 6, 5},
	{0, 1, 5, 4}, {3, 7, 6, 2},
	{0, 3, 2, 1}, {4, 5, 6, 7}
};

// face d 的"轴向" (0=x,1=y,2=z) 和 sign
constexpr int kAxis[6] = {0, 0, 1, 1, 2, 2};
constexpr int kSign[6] = {-1, 1, -1, 1, -1, 1};

// 整数 grid 上一个点的 key：64-bit hash 用
struct VertKey
{
	std::int64_t x, y, z;
	bool operator==(const VertKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct VertKeyHash
{
	std::size_t operator()(const VertKey& k) const noexcept
	{
		// 简单 splitmix-ish
		std::size_t h = static_cast<std::size_t>(k.x) * 0x9E3779B97F4A7C15ULL;
		h ^= static_cast<std::size_t>(k.y) + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
		h ^= static_cast<std::size_t>(k.z) + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
		return h;
	}
};

// 面去重 key：4 个 vertex id 排序后的 tuple
struct FaceKey
{
	std::array<int, 4> v;
	bool operator==(const FaceKey& o) const { return v == o.v; }
};
struct FaceKeyHash
{
	std::size_t operator()(const FaceKey& k) const noexcept
	{
		std::size_t h = 1469598103934665603ULL;
		for (int i = 0; i < 4; ++i)
		{
			h ^= static_cast<std::size_t>(k.v[i]);
			h *= 1099511628211ULL;
		}
		return h;
	}
};
} // namespace

XFoam_CMshCartesianExtractor::XFoam_CMshCartesianExtractor(
	XFoam_CMshOctree& oct,
	const Params& p,
	const XFoam_BrepBase* brep)
	: oct_(oct)
	, p_(p)
	, brep_(brep)
{}

bool XFoam_CMshCartesianExtractor::extract(XFoam_CMshPolyMeshGen& out)
{
	out = XFoam_CMshPolyMeshGen();

	// 1) 2:1 balance（让 face neighbour 至多 1 级差）
	if (p_.autoBalance) oct_.balance21();

	// 2) 选哪些 leaf 进网格
	std::vector<const XFoam_CMshOctreeCube*> cells;
	cells.reserve(static_cast<std::size_t>(oct_.nLeaves()));
	std::vector<int> leafToCell(static_cast<std::size_t>(oct_.nLeaves()), -1);
	std::vector<bool> keepFlag(static_cast<std::size_t>(oct_.nLeaves()), false);
	oct_.forEachLeaf([&](const XFoam_CMshOctreeCube& c) {
		bool keep = false;
		switch (c.type)
		{
			case XFoam_CMshCubeType::Inside:  keep = p_.keepInside; break;
			case XFoam_CMshCubeType::Data:    keep = p_.keepData; break;
			case XFoam_CMshCubeType::Outside: keep = p_.keepOutside; break;
			default: break;
		}
		if (keep) keepFlag[static_cast<std::size_t>(c.leafIdx)] = true;
	});

	// 2b) locationInMesh BFS：把 keepFlag 收紧到指定点所在 face-连通群
	if (p_.useLocationInMesh)
	{
		const auto* seed = oct_.findLeafContaining(p_.locationInMesh);
		if (!seed)
		{
			std::cerr << "cmsh extractor: locationInMesh "
			          << "(" << p_.locationInMesh.x() << ", "
			          << p_.locationInMesh.y() << ", "
			          << p_.locationInMesh.z() << ") 不在 root bbox 内，忽略该过滤。\n";
		}
		else if (!keepFlag[static_cast<std::size_t>(seed->leafIdx)])
		{
			std::cerr << "cmsh extractor: locationInMesh 落在被过滤掉的 leaf "
			          << "(type=" << static_cast<int>(seed->type)
			          << ", level=" << static_cast<int>(seed->level)
			          << ")，BFS 无法启动，忽略该过滤。\n";
		}
		else
		{
			std::vector<bool> reached(static_cast<std::size_t>(oct_.nLeaves()), false);
			std::queue<const XFoam_CMshOctreeCube*> bfs;
			bfs.push(seed);
			reached[static_cast<std::size_t>(seed->leafIdx)] = true;
			while (!bfs.empty())
			{
				const auto* cur = bfs.front();
				bfs.pop();
				for (int d = 0; d < 6; ++d)
				{
					const auto nbr = oct_.faceNeighbour(*cur, d);
					auto consume = [&](const XFoam_CMshOctreeCube* lf) {
						if (!lf) return;
						const std::size_t idx = static_cast<std::size_t>(lf->leafIdx);
						if (!keepFlag[idx] || reached[idx]) return;
						reached[idx] = true;
						bfs.push(lf);
					};
					switch (nbr.kind)
					{
						case XFoam_CMshOctree::FaceNbrKind::Same:
						case XFoam_CMshOctree::FaceNbrKind::Coarser:
							consume(nbr.same);
							break;
						case XFoam_CMshOctree::FaceNbrKind::Finer:
							for (int k = 0; k < 4; ++k) consume(nbr.finer[k]);
							break;
						default: break;
					}
				}
			}
			// 用 reached 替换 keepFlag
			XFoam_Label nDropped = 0;
			for (std::size_t i = 0; i < keepFlag.size(); ++i)
			{
				if (keepFlag[i] && !reached[i]) ++nDropped;
				keepFlag[i] = keepFlag[i] && reached[i];
			}
			std::cout << "cmsh extractor: locationInMesh BFS kept group, dropped "
			          << nDropped << " disconnected leaves.\n";
		}
	}

	oct_.forEachLeaf([&](const XFoam_CMshOctreeCube& c) {
		if (keepFlag[static_cast<std::size_t>(c.leafIdx)])
		{
			leafToCell[static_cast<std::size_t>(c.leafIdx)] = static_cast<int>(cells.size());
			cells.push_back(&c);
		}
	});
	if (cells.empty()) return false;
	out.nCells = static_cast<int>(cells.size());

	// 3) 推 maxLevel + 整数 grid step
	std::uint8_t maxLevel = 0;
	for (const auto* c : cells)
	{
		if (c->level > maxLevel) maxLevel = c->level;
	}

	const XFoam_BoundBox& root = oct_.rootBox();
	const XFoam_Vector3D  span = root.span();
	const XFoam_Label     denom = static_cast<XFoam_Label>(1) << maxLevel;
	const XFoam_Scalar    dxMin = span.x() / static_cast<XFoam_Scalar>(denom);
	const XFoam_Scalar    dyMin = span.y() / static_cast<XFoam_Scalar>(denom);
	const XFoam_Scalar    dzMin = span.z() / static_cast<XFoam_Scalar>(denom);

	// 4) vertex dedup（整数 grid coords + hash）
	std::unordered_map<VertKey, int, VertKeyHash> vmap;
	vmap.reserve(cells.size() * 8);

	auto registerVertex = [&](XFoam_Label ix, XFoam_Label iy, XFoam_Label iz) -> int
	{
		const VertKey k{ix, iy, iz};
		auto it = vmap.find(k);
		if (it != vmap.end()) return it->second;
		const int idx = static_cast<int>(out.points.size());
		out.points.push_back(XFoam_Vector3D(
			root.min().x() + ix * dxMin,
			root.min().y() + iy * dyMin,
			root.min().z() + iz * dzMin));
		vmap.emplace(k, idx);
		return idx;
	};

	// 把 (cube, corner=0..7) 转成 grid coords
	auto cubeCornerGrid = [&](const XFoam_CMshOctreeCube& c, int corner,
	                          XFoam_Label& ix, XFoam_Label& iy, XFoam_Label& iz)
	{
		const XFoam_Label step = static_cast<XFoam_Label>(1) << (maxLevel - c.level);
		const XFoam_Label x0 = c.posX * step;
		const XFoam_Label y0 = c.posY * step;
		const XFoam_Label z0 = c.posZ * step;
		ix = x0 + kCornerOff[corner][0] * step;
		iy = y0 + kCornerOff[corner][1] * step;
		iz = z0 + kCornerOff[corner][2] * step;
	};

	// 把 sub-quad（4 个 finer cell 中的一个 sub-face）转 grid coords：
	//   coarse cell 的 face d 被 4 切；finer index = (b1*2 + b0)，每个 sub-quad
	//   的 4 corner 在 coarse cell 的 face 平面上。
	// 简化做法：直接取 finer cell 在该 face 上的 4 个 corner，因为 dedup 之后
	// finer 的 corner 就是 sub-quad 的 4 个角。
	auto fineCellSubQuad = [&](const XFoam_CMshOctreeCube& fineCell, int faceD,
	                           int outVerts[4])
	{
		// 在 fineCell 的 faceD 方向上，4 个 corner = kFaceCorners[faceD]
		// 但 faceD 在 fine cell 是反方向：粗 cell 的 +x face = 细 cell 的 -x face。
		// face d 的 opposite d^1 (toggle low bit)
		const int oppD = faceD ^ 1;
		for (int i = 0; i < 4; ++i)
		{
			const int corner = kFaceCorners[oppD][i];
			XFoam_Label ix, iy, iz;
			cubeCornerGrid(fineCell, corner, ix, iy, iz);
			outVerts[i] = registerVertex(ix, iy, iz);
		}
	};

	// 5) 收集 cell 的 face：用 FaceKey dedup
	//    每个 face 第一次出现 → 加入 raw faces; owner=当前 cellIdx, neighbour=-1
	//    第二次出现 → 标 neighbour=当前 cellIdx（实际值后面纠正方向 / 翻转）
	struct RawFace
	{
		std::array<int, 4> verts; // 原始 CCW 序（owner→neighbour 方向）
		int owner     = -1;
		int neighbour = -1;
		int patchKey  = 0;        // boundary only：(brepIdx << 16) | subPatchId+1；0 = default
	};
	std::vector<RawFace> raw;
	raw.reserve(cells.size() * 3);
	std::unordered_map<FaceKey, int, FaceKeyHash> faceMap;
	faceMap.reserve(cells.size() * 3);

	auto addFace = [&](int ownerCell, const int v[4])
	{
		FaceKey k;
		k.v = {v[0], v[1], v[2], v[3]};
		std::sort(k.v.begin(), k.v.end());
		auto it = faceMap.find(k);
		if (it == faceMap.end())
		{
			RawFace rf;
			rf.verts = {v[0], v[1], v[2], v[3]};
			rf.owner = ownerCell;
			rf.neighbour = -1;
			const int fi = static_cast<int>(raw.size());
			raw.push_back(rf);
			faceMap.emplace(k, fi);
		}
		else
		{
			RawFace& rf = raw[it->second];
			if (rf.neighbour != -1)
			{
				std::cerr << "cmsh extractor: face would have 3 owners ("
				          << rf.owner << "," << rf.neighbour << "," << ownerCell << ")\n";
				return;
			}
			// 保证 owner < neighbour；若 ownerCell 更小，交换并反向 verts 让法向
			// 从 owner→neighbour（cfMesh 约定）。
			if (ownerCell < rf.owner)
			{
				rf.neighbour = rf.owner;
				rf.owner = ownerCell;
				std::reverse(rf.verts.begin(), rf.verts.end());
			}
			else
			{
				rf.neighbour = ownerCell;
			}
		}
	};

	for (std::size_t ci = 0; ci < cells.size(); ++ci)
	{
		const XFoam_CMshOctreeCube& cell = *cells[ci];
		const int cellIdx = static_cast<int>(ci);
		for (int d = 0; d < 6; ++d)
		{
			auto nbr = oct_.faceNeighbour(cell, d);

			if (nbr.kind == XFoam_CMshOctree::FaceNbrKind::Finer)
			{
				// 4 sub-quads，每个 sub-quad owner=cellIdx, neighbour=finer leaf cell
				for (int k = 0; k < 4; ++k)
				{
					if (!nbr.finer[k]) continue;
					const int fineCellIdx = leafToCell[static_cast<std::size_t>(nbr.finer[k]->leafIdx)];
					if (fineCellIdx < 0)
					{
						// fine 邻居被 keepXxx 过滤掉了 → 这块作为 boundary
						int v[4];
						fineCellSubQuad(*nbr.finer[k], d, v);
						addFace(cellIdx, v);
						continue;
					}
					// 双方都在网格里：emit 由 owner-side（cellIdx）。fine 那边
					// 会在它自己的 d^1 face 上看见 Coarser 邻居 → skip emit。
					int v[4];
					fineCellSubQuad(*nbr.finer[k], d, v);
					addFace(cellIdx, v);
				}
			}
			else if (nbr.kind == XFoam_CMshOctree::FaceNbrKind::Coarser)
			{
				// coarse 在网格里 → 它已经/将会 emit 这块的 sub-quads；fine 跳过
				const int coarseCellIdx = leafToCell[static_cast<std::size_t>(nbr.same->leafIdx)];
				if (coarseCellIdx < 0)
				{
					// coarse 邻居被过滤掉 → 这块算 boundary，fine 自己 emit
					int v[4];
					for (int i = 0; i < 4; ++i)
					{
						const int corner = kFaceCorners[d][i];
						XFoam_Label ix, iy, iz;
						cubeCornerGrid(cell, corner, ix, iy, iz);
						v[i] = registerVertex(ix, iy, iz);
					}
					addFace(cellIdx, v);
				}
				// else: skip
			}
			else if (nbr.kind == XFoam_CMshOctree::FaceNbrKind::Same)
			{
				const int otherIdx = leafToCell[static_cast<std::size_t>(nbr.same->leafIdx)];
				int v[4];
				for (int i = 0; i < 4; ++i)
				{
					const int corner = kFaceCorners[d][i];
					XFoam_Label ix, iy, iz;
					cubeCornerGrid(cell, corner, ix, iy, iz);
					v[i] = registerVertex(ix, iy, iz);
				}
				if (otherIdx < 0)
				{
					// 邻居被过滤掉 → boundary
					addFace(cellIdx, v);
				}
				else
				{
					addFace(cellIdx, v);
				}
			}
			else // None
			{
				int v[4];
				for (int i = 0; i < 4; ++i)
				{
					const int corner = kFaceCorners[d][i];
					XFoam_Label ix, iy, iz;
					cubeCornerGrid(cell, corner, ix, iy, iz);
					v[i] = registerVertex(ix, iy, iz);
				}
				addFace(cellIdx, v);
			}
		}
	}

	// 6) 把 raw faces 重新排：先所有 internal（按 (owner, neighbour) 升序），
	//    再所有 boundary（按 patchKey 升序 → owner 升序）。
	std::vector<int> internalIdx, boundaryIdx;
	internalIdx.reserve(raw.size());
	boundaryIdx.reserve(raw.size());
	for (std::size_t i = 0; i < raw.size(); ++i)
	{
		if (raw[i].neighbour >= 0) internalIdx.push_back(static_cast<int>(i));
		else                       boundaryIdx.push_back(static_cast<int>(i));
	}

	// 6a) 给 boundary face 打 patchKey（perFacePatches 才需要）
	if (p_.perFacePatches && brep_ && !brep_->empty())
	{
		for (int rfi : boundaryIdx)
		{
			RawFace& rf = raw[static_cast<std::size_t>(rfi)];
			XFoam_Vector3D c(0, 0, 0);
			for (int vi : rf.verts) c += out.points[static_cast<std::size_t>(vi)];
			c *= static_cast<XFoam_Scalar>(0.25);
			const XFoam_Label sub = brep_->closestSubPatchId(c);
			rf.patchKey = static_cast<int>(sub + 1);
		}
	}

	std::sort(internalIdx.begin(), internalIdx.end(),
		[&](int a, int b) {
			if (raw[a].owner != raw[b].owner) return raw[a].owner < raw[b].owner;
			return raw[a].neighbour < raw[b].neighbour;
		});
	std::sort(boundaryIdx.begin(), boundaryIdx.end(),
		[&](int a, int b) {
			if (raw[a].patchKey != raw[b].patchKey) return raw[a].patchKey < raw[b].patchKey;
			return raw[a].owner < raw[b].owner;
		});

	out.faces.reserve(raw.size());
	out.owner.reserve(raw.size());
	out.neighbour.reserve(internalIdx.size());

	for (int rfi : internalIdx)
	{
		XFoam_CMshPolyMeshGen::Face f;
		f.verts.assign(raw[rfi].verts.begin(), raw[rfi].verts.end());
		out.faces.push_back(std::move(f));
		out.owner.push_back(raw[rfi].owner);
		out.neighbour.push_back(raw[rfi].neighbour);
	}
	const int startBoundary = static_cast<int>(out.faces.size());
	for (int rfi : boundaryIdx)
	{
		XFoam_CMshPolyMeshGen::Face f;
		f.verts.assign(raw[rfi].verts.begin(), raw[rfi].verts.end());
		out.faces.push_back(std::move(f));
		out.owner.push_back(raw[rfi].owner);
	}

	// 6b) 按 patchKey 拼出 patches
	if (p_.perFacePatches && brep_ && !brep_->empty())
	{
		int i = 0;
		const int nb = static_cast<int>(boundaryIdx.size());
		while (i < nb)
		{
			int j = i;
			const int curKey = raw[boundaryIdx[static_cast<std::size_t>(i)]].patchKey;
			while (j < nb && raw[boundaryIdx[static_cast<std::size_t>(j)]].patchKey == curKey) ++j;

			const int subId = curKey - 1; // patchKey = sub + 1，0 留给 "未知"
			XFoam_CMshPolyMeshGen::Patch P;
			std::string subName;
			if (subId >= 0)
			{
				const XFoam_String sn = brep_->subPatchName(static_cast<XFoam_Label>(subId));
				subName = static_cast<const std::string&>(sn);
			}
			if (subName.empty()) subName = "sub" + std::to_string(subId < 0 ? 0 : subId);
			P.name      = subName;
			P.type      = p_.defaultPatchType;
			P.startFace = startBoundary + i;
			P.nFaces    = j - i;
			out.patches.push_back(std::move(P));
			i = j;
		}
	}
	else
	{
		XFoam_CMshPolyMeshGen::Patch P;
		P.name      = p_.defaultPatchName;
		P.type      = p_.defaultPatchType;
		P.startFace = startBoundary;
		P.nFaces    = static_cast<int>(out.faces.size()) - startBoundary;
		out.patches.push_back(std::move(P));
	}

	return true;
}
