#include "XFoam/snap/xfoam_snappyhexmesh.h"

#include "XFoam/block/xfoam_blockmesh.h"
#include "XFoam/mesh/xfoam_polymesh.h"
#include "XFoam/mesh/xfoam_polypatch.h"
#include "XFoam/mesh/xfoam_shape.h"
#include "XFoam/utilities/xfoam_dictionary.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace
{
// 单 hex 块 8 角点（OF 习惯：i-fastest，j 其次，k 最慢）按 (i,j,k) ∈ {0,1}^3 的三线性插值。
// 顺序与 XFoam_CellModel::hex 一致：
//   v0=(0,0,0) v1=(1,0,0) v2=(1,1,0) v3=(0,1,0)
//   v4=(0,0,1) v5=(1,0,1) v6=(1,1,1) v7=(0,1,1)
XFoam_Vector3D trilinearHex(
	const XFoam_Vector3D corner[8],
	XFoam_Scalar u, XFoam_Scalar v, XFoam_Scalar w)
{
	const XFoam_Scalar ui = 1 - u, vi = 1 - v, wi = 1 - w;
	XFoam_Vector3D p(0, 0, 0);
	// (1-u)(1-v)(1-w) v0  + u(1-v)(1-w) v1 + u v (1-w) v2 + (1-u) v (1-w) v3
	p += corner[0] * (ui * vi * wi);
	p += corner[1] * (u  * vi * wi);
	p += corner[2] * (u  * v  * wi);
	p += corner[3] * (ui * v  * wi);
	p += corner[4] * (ui * vi * w);
	p += corner[5] * (u  * vi * w);
	p += corner[6] * (u  * v  * w);
	p += corner[7] * (ui * v  * w);
	return p;
}

inline XFoam_Label pointIdx(XFoam_Label i, XFoam_Label j, XFoam_Label k, XFoam_Label Nx, XFoam_Label Ny)
{
	return i + j * (Nx + 1) + k * (Nx + 1) * (Ny + 1);
}

inline XFoam_Label cellIdx(XFoam_Label i, XFoam_Label j, XFoam_Label k, XFoam_Label Nx, XFoam_Label Ny)
{
	return i + j * Nx + k * Nx * Ny;
}

// OF 标准 hex 6 个面，按 cellModel hex 中 facei → 4 顶点（在 0..7 局部编号下的）次序。
// face 0: z-min (0 3 2 1)   face 1: z-max (4 5 6 7)
// face 2: y-min (0 1 5 4)   face 3: y-max (3 7 6 2)
// face 4: x-min (0 4 7 3)   face 5: x-max (1 2 6 5)
const int kHexFace[6][4] = {
	{0, 3, 2, 1},
	{4, 5, 6, 7},
	{0, 1, 5, 4},
	{3, 7, 6, 2},
	{0, 4, 7, 3},
	{1, 2, 6, 5}
};

// 给定本格 (i,j,k) 与 facei，返回其邻格 (di, dj, dk) 偏移；越界即外部。
void neighbourOffset(int facei, int& di, int& dj, int& dk)
{
	di = dj = dk = 0;
	switch (facei)
	{
	case 0: dk = -1; break;
	case 1: dk = +1; break;
	case 2: dj = -1; break;
	case 3: dj = +1; break;
	case 4: di = -1; break;
	case 5: di = +1; break;
	}
}

// 单 hex 是否触碰块边界 face：返回 facei (0..5) 是否为 i/j/k = 0 或 N-1 的面。
bool isBlockBoundaryFace(int facei, XFoam_Label i, XFoam_Label j, XFoam_Label k,
                         XFoam_Label Nx, XFoam_Label Ny, XFoam_Label Nz)
{
	switch (facei)
	{
	case 0: return k == 0;
	case 1: return k == Nz - 1;
	case 2: return j == 0;
	case 3: return j == Ny - 1;
	case 4: return i == 0;
	case 5: return i == Nx - 1;
	}
	return false;
}
} // namespace

// ============================================================================
// XFoam_SnappyHexMesh ctor: dict 解析
// ============================================================================

namespace
{
// XFoam 的 dict 解析器对 `key { ... }` 形式会把内容存为 primitive stream（而不是
// sub-dictionary）。这里提供一个"按需把 stream 解析成 dict"的小工具，让上层
// 代码可以一致地按 sub-dict 取用。返回一个 owned XFoam_Dictionary（外面用智能指针接）。
//
// 不修改 XFoam_Dictionary 主体是为了把改动尽量收敛在 snap 模块内。
std::unique_ptr<XFoam_Dictionary> streamToDict(XFoam_ITstream& s)
{
	std::unique_ptr<XFoam_Dictionary> out;
	s.rewind();
	XFoam_Token t;
	s >> t;
	if (!t.good() || !(t == XFoam_Token::BEGIN_BLOCK))
	{
		return out;
	}
	XFoam_TokenList inner;
	int depth = 1;
	while (s >> t, t.good())
	{
		if (t == XFoam_Token::BEGIN_BLOCK || t == XFoam_Token::BEGIN_LIST)
		{
			++depth;
			inner.append(t);
		}
		else if (t == XFoam_Token::END_BLOCK || t == XFoam_Token::END_LIST)
		{
			--depth;
			if (depth == 0) break;
			inner.append(t);
		}
		else
		{
			inner.append(t);
		}
	}
	out.reset(new XFoam_Dictionary());
	XFoam_ITstream is(XFoam_KeyType("__snappy_inner__"), inner);
	is.rewind();
	if (!out->read(is))
	{
		out.reset();
	}
	return out;
}

// 取 dict 的子段，无论它被存成真正的 sub-dict 还是被存成 `{...}` 形式的 stream。
// owned 的生命期跟 DictView 一致；dict 指针在 DictView 销毁前都有效。
struct DictView
{
	const XFoam_Dictionary* dict = nullptr;
	std::unique_ptr<XFoam_Dictionary> owned;
};

DictView asDict(const XFoam_Dictionary& parent, const XFoam_Word& key)
{
	DictView v;
	const XFoam_Entry* e = parent.lookupEntryPtr(key, false, true);
	if (!e) return v;
	if (e->isDict())
	{
		v.dict = &e->dict();
		return v;
	}
	if (e->isStream())
	{
		v.owned = streamToDict(e->stream());
		v.dict = v.owned.get();
		return v;
	}
	return v;
}
} // namespace

XFoam_SnappyHexMesh::XFoam_SnappyHexMesh(const XFoam_Dictionary& dict)
{
	{
		DictView v = asDict(dict, XFoam_Word("castellatedMeshControls"));
		if (v.dict) refine_.readDict(*v.dict);
	}
	{
		DictView v = asDict(dict, XFoam_Word("snapControls"));
		if (v.dict) snap_.readDict(*v.dict);
	}
	{
		DictView v = asDict(dict, XFoam_Word("addLayersControls"));
		if (v.dict) layer_.readDict(*v.dict);
	}
	readPhaseFlags(dict);
	readRefinementSurfacesFirst(dict);
	readGeometryFirst(dict);
}

void XFoam_SnappyHexMesh::readPhaseFlags(const XFoam_Dictionary& dict)
{
	(void)dict.readIfPresent(XFoam_Word("castellatedMesh"), phases_.castellatedMesh);
	(void)dict.readIfPresent(XFoam_Word("snap"), phases_.snap);
	(void)dict.readIfPresent(XFoam_Word("addLayers"), phases_.addLayers);
}

void XFoam_SnappyHexMesh::readRefinementSurfacesFirst(const XFoam_Dictionary& dict)
{
	// castellatedMeshControls.refinementSurfaces.<surfaceName> { level (a b); ... }
	// 仅取第一个，level 取 (min max) 的 max 作为全局 L。
	DictView cv = asDict(dict, XFoam_Word("castellatedMeshControls"));
	if (!cv.dict) return;
	DictView sv = asDict(*cv.dict, XFoam_Word("refinementSurfaces"));
	if (!sv.dict) return;
	const XFoam_WordList keys = sv.dict->toc();
	if (keys.empty()) return;
	firstSurfaceName_ = keys[0];
	DictView pv = asDict(*sv.dict, firstSurfaceName_);
	const XFoam_Dictionary* sd = pv.dict;
	if (!sd) return;
	// level 是个 2-int list（min max）。这里用 readType 取第二个数；解析失败回退 0。
	const XFoam_Entry* le = sd->lookupEntryPtr(XFoam_Word("level"), false, false);
	if (!le) return;
	XFoam_ITstream& is = le->stream();
	is.rewind();
	XFoam_Label lvMin = 0, lvMax = 0;
	XFoam_Token t;
	is >> t;
	if (t.isPunctuation() && t.pToken() == XFoam_Token::BEGIN_LIST)
	{
		int32_t mn = 0, mx = 0;
		is >> mn >> mx;
		lvMin = static_cast<XFoam_Label>(mn);
		lvMax = static_cast<XFoam_Label>(mx);
	}
	(void)lvMin;
	globalLevel_ = std::max<XFoam_Label>(0, lvMax);
}

void XFoam_SnappyHexMesh::readGeometryFirst(const XFoam_Dictionary& dict)
{
	// geometry { <name>.stl { type triSurfaceMesh; name <name>; } }
	// 也兼容旧式：geometry { <name>.stl { type triSurfaceMesh; file "<name>.stl"; } }
	DictView gv = asDict(dict, XFoam_Word("geometry"));
	const XFoam_Dictionary* geo = gv.dict;
	if (!geo) return;
	const XFoam_WordList keys = geo->toc();
	if (keys.empty()) return;
	const XFoam_Word k0 = keys[0];
	DictView pv = asDict(*geo, k0);
	const XFoam_Dictionary* gd = pv.dict;
	XFoam_Word fileName(static_cast<const XFoam_String&>(k0));
	if (gd)
	{
		XFoam_FileName tmp;
		if (gd->readIfPresent(XFoam_Word("file"), tmp) && !tmp.empty())
		{
			fileName = XFoam_Word(static_cast<const XFoam_String&>(tmp));
		}
	}
	firstSurfaceFile_ = XFoam_FileName(static_cast<const XFoam_String&>(fileName));
}

// ============================================================================
// run(): refine + cull + snap
// ============================================================================

bool XFoam_SnappyHexMesh::run(
	const XFoam_BlockMesh& bg,
	const XFoam_TriSurface& stl,
	XFoam_AutoPtr<XFoam_PolyMesh>& outPolyMesh,
	Stats& stats) const
{
	stats = Stats();
	stats.refinementLevel = globalLevel_;
	stats.stlPatchName = firstSurfaceName_.empty() ? XFoam_Word("snappy") : firstSurfaceName_;

	// 限制：单 hex 块。多 block 拼接尚未支持（calcMergeInfo 是恒等映射，跨块合点还没补）。
	if (bg.size() == 0 || !bg.set(0))
	{
		std::cerr << "snappy: background BlockMesh has no usable block 0.\n";
		return false;
	}
	if (bg.size() > 1)
	{
		std::cerr << "snappy: WARNING: multi-block bg unsupported; only block 0 will be refined.\n";
	}
	if (stl.empty())
	{
		std::cerr << "snappy: STL is empty.\n";
		return false;
	}
	if (!refine_.hasLocationInMesh)
	{
		std::cerr << "snappy: castellatedMeshControls.locationInMesh required.\n";
		return false;
	}

	// 1) 收 block 0 的 8 个角点 + 原 cell 计数
	const XFoam_BlockDescriptor& bd = bg[0];
	const XFoam_CellShape& shape0 = bd.blockShape();
	const XFoam_UList<XFoam_Vector3D>& blockVerts = bd.vertices();
	XFoam_Vector3D corner[8];
	for (int i = 0; i < 8; ++i)
	{
		corner[i] = blockVerts[shape0[i]] * bg.scaleFactor();
	}

	const XFoam_Label nx0 = bd.density().x();
	const XFoam_Label ny0 = bd.density().y();
	const XFoam_Label nz0 = bd.density().z();
	const XFoam_Label refFactor = 1 << static_cast<int>(globalLevel_); // 2^L
	const XFoam_Label Nx = nx0 * refFactor;
	const XFoam_Label Ny = ny0 * refFactor;
	const XFoam_Label Nz = nz0 * refFactor;
	stats.nBgCells = nx0 * ny0 * nz0;
	stats.nRefinedCells = Nx * Ny * Nz;

	// 2) 生成细化结构网点 + cellShapes
	const XFoam_Label nPts = (Nx + 1) * (Ny + 1) * (Nz + 1);
	XFoam_PointField points(nPts);
	for (XFoam_Label k = 0; k <= Nz; ++k)
	{
		const XFoam_Scalar w = static_cast<XFoam_Scalar>(k) / static_cast<XFoam_Scalar>(Nz);
		for (XFoam_Label j = 0; j <= Ny; ++j)
		{
			const XFoam_Scalar v = static_cast<XFoam_Scalar>(j) / static_cast<XFoam_Scalar>(Ny);
			for (XFoam_Label i = 0; i <= Nx; ++i)
			{
				const XFoam_Scalar u = static_cast<XFoam_Scalar>(i) / static_cast<XFoam_Scalar>(Nx);
				points[pointIdx(i, j, k, Nx, Ny)] = trilinearHex(corner, u, v, w);
			}
		}
	}

	XFoam_CellShapeList allCells(Nx * Ny * Nz);
	for (XFoam_Label k = 0; k < Nz; ++k)
	{
		for (XFoam_Label j = 0; j < Ny; ++j)
		{
			for (XFoam_Label i = 0; i < Nx; ++i)
			{
				XFoam_LabelList lbl(8);
				lbl[0] = pointIdx(i,     j,     k,     Nx, Ny);
				lbl[1] = pointIdx(i + 1, j,     k,     Nx, Ny);
				lbl[2] = pointIdx(i + 1, j + 1, k,     Nx, Ny);
				lbl[3] = pointIdx(i,     j + 1, k,     Nx, Ny);
				lbl[4] = pointIdx(i,     j,     k + 1, Nx, Ny);
				lbl[5] = pointIdx(i + 1, j,     k + 1, Nx, Ny);
				lbl[6] = pointIdx(i + 1, j + 1, k + 1, Nx, Ny);
				lbl[7] = pointIdx(i,     j + 1, k + 1, Nx, Ny);
				allCells[cellIdx(i, j, k, Nx, Ny)] = XFoam_CellShape(XFoam_CellModel::hex(), lbl, false);
			}
		}
	}

	// 3) STL 内外分类：先确定 locationInMesh 的类别，每 cell 中心用同法分类。
	const bool locInside = stl.contains(refine_.locationInMesh);
	std::vector<unsigned char> keep(static_cast<size_t>(Nx * Ny * Nz), 0);
	for (XFoam_Label k = 0; k < Nz; ++k)
	{
		for (XFoam_Label j = 0; j < Ny; ++j)
		{
			for (XFoam_Label i = 0; i < Nx; ++i)
			{
				const XFoam_Vector3D centroid = trilinearHex(
					corner,
					(static_cast<XFoam_Scalar>(i) + 0.5) / Nx,
					(static_cast<XFoam_Scalar>(j) + 0.5) / Ny,
					(static_cast<XFoam_Scalar>(k) + 0.5) / Nz);
				const bool inside = stl.contains(centroid);
				if (inside == locInside)
				{
					keep[cellIdx(i, j, k, Nx, Ny)] = 1;
				}
			}
		}
	}

	// 4) 构造保留 cells 列表 + 边界面分两类：原 block 壁、STL 新暴露面
	XFoam_CellShapeList keptCells;
	keptCells.setSize(0);
	{
		XFoam_Label cnt = 0;
		for (XFoam_Label c = 0; c < Nx * Ny * Nz; ++c)
		{
			if (keep[c]) ++cnt;
		}
		keptCells.setSize(cnt);
	}

	XFoam_FaceList originalWallFaces;
	XFoam_FaceList stlFaces;

	XFoam_Label putIdx = 0;
	for (XFoam_Label k = 0; k < Nz; ++k)
	{
		for (XFoam_Label j = 0; j < Ny; ++j)
		{
			for (XFoam_Label i = 0; i < Nx; ++i)
			{
				const XFoam_Label c = cellIdx(i, j, k, Nx, Ny);
				if (!keep[c]) continue;

				const XFoam_CellShape& cs = allCells[c];
				keptCells[putIdx++] = cs;

				for (int fi = 0; fi < 6; ++fi)
				{
					int di = 0, dj = 0, dk = 0;
					neighbourOffset(fi, di, dj, dk);
					const XFoam_Label ni = i + di;
					const XFoam_Label nj = j + dj;
					const XFoam_Label nk = k + dk;
					bool nOutside = (ni < 0 || ni >= Nx || nj < 0 || nj >= Ny || nk < 0 || nk >= Nz);
					bool nRemoved = !nOutside && !keep[cellIdx(ni, nj, nk, Nx, Ny)];

					// 邻居存在且也是 kept → internal face，让 PolyMesh 自己 dedupe。
					if (!nOutside && !nRemoved) continue;

					// 这是 kept cell 的一面边界面：
					XFoam_Face quad(4);
					for (int k4 = 0; k4 < 4; ++k4)
					{
						quad[k4] = cs[kHexFace[fi][k4]];
					}
					if (isBlockBoundaryFace(fi, i, j, k, Nx, Ny, Nz))
					{
						originalWallFaces.append(quad);
					}
					else
					{
						stlFaces.append(quad);
					}
				}
			}
		}
	}
	stats.nKeptCells = keptCells.size();

	// 5) snap STL patch points 到 STL 最近点
	if (phases_.snap && !stlFaces.empty())
	{
		std::vector<unsigned char> snapped(static_cast<size_t>(nPts), 0);
		for (XFoam_Label fi = 0; fi < stlFaces.size(); ++fi)
		{
			const XFoam_Face& f = stlFaces[fi];
			for (XFoam_Label k4 = 0; k4 < f.size(); ++k4)
			{
				const XFoam_Label pi = f[k4];
				if (snapped[pi]) continue;
				snapped[pi] = 1;
				XFoam_Vector3D closest, normal;
				stl.closestPointAndNormal(points[pi], closest, normal);
				const XFoam_Scalar d = (closest - points[pi]).mag();
				if (d > stats.maxSnapDistance) stats.maxSnapDistance = d;
				points[pi] = closest;
				++stats.nSnappedPoints;
			}
		}
	}

	// 6) 装配 polyMesh
	XFoam_FaceListList patches;
	XFoam_WordList patchNames;
	XFoam_WordList patchTypes;
	{
		XFoam_Label nPatch = 0;
		if (!originalWallFaces.empty()) ++nPatch;
		if (!stlFaces.empty()) ++nPatch;
		patches.setSize(nPatch);
		patchNames.setSize(nPatch);
		patchTypes.setSize(nPatch);
		XFoam_Label pi = 0;
		if (!originalWallFaces.empty())
		{
			patches[pi] = originalWallFaces;
			const XFoam_WordList origNames = bg.patchNames();
			const XFoam_WordList& origTypes = bg.patchTypes();
			patchNames[pi] = origNames.empty() ? XFoam_Word("walls") : origNames[0];
			patchTypes[pi] = origTypes.empty() ? XFoam_Word("patch") : origTypes[0];
			++pi;
		}
		if (!stlFaces.empty())
		{
			patches[pi] = stlFaces;
			patchNames[pi] = stats.stlPatchName;
			patchTypes[pi] = XFoam_Word("wall");
			++pi;
		}
	}
	stats.outPatchTypes = patchTypes;

	const XFoam_PtrListDictionary<XFoam_Dictionary> emptyDicts;
	outPolyMesh.reset(new XFoam_PolyMesh(
		XFoam_move(points),
		keptCells,
		patches,
		XFoam_move(patchNames),
		emptyDicts,
		XFoam_Word("defaultFaces"),
		XFoam_Word(XFoam_PolyPatch::typeName)));

	return true;
}
