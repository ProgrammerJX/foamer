#include "XFoam/snap/xfoam_snappyhexmesh.h"

#include "XFoam/block/xfoam_blockmesh.h"
#include "XFoam/utilities/xfoam_dictionary.h"

#include <algorithm>
#include <array>
#include <boost/filesystem.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unordered_map>
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

// 6 个 face 方向上的相邻 base-cell 偏移：(di, dj, dk)。
const int kFaceDir[6][3] = {
	{ 0,  0, -1}, // 0: z-min
	{ 0,  0, +1}, // 1: z-max
	{ 0, -1,  0}, // 2: y-min
	{ 0, +1,  0}, // 3: y-max
	{-1,  0,  0}, // 4: x-min
	{+1,  0,  0}  // 5: x-max
};

// 点 dedup 用的坐标键。整数化坐标 = round(p / eps)，eps ~= bbox 对角 * 1e-9。
struct PointKey
{
	int64_t kx;
	int64_t ky;
	int64_t kz;
	bool operator==(const PointKey& o) const noexcept
	{
		return kx == o.kx && ky == o.ky && kz == o.kz;
	}
};

struct PointKeyHash
{
	size_t operator()(const PointKey& k) const noexcept
	{
		const uint64_t a = static_cast<uint64_t>(k.kx);
		const uint64_t b = static_cast<uint64_t>(k.ky);
		const uint64_t c = static_cast<uint64_t>(k.kz);
		uint64_t h = a * 0x9E3779B97F4A7C15ull + b;
		h = h * 0x9E3779B97F4A7C15ull + c;
		h ^= (h >> 33);
		return static_cast<size_t>(h);
	}
};

// face dedup 用的有序顶点元组键。最多 4 顶点（quad），多边形我们这里只产生 quad
// （split face 也是 4 个独立 quad），所以 4 个 int 足够。
struct FaceKey
{
	int v[4]; // 升序
	bool operator==(const FaceKey& o) const noexcept
	{
		return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2] && v[3] == o.v[3];
	}
};

struct FaceKeyHash
{
	size_t operator()(const FaceKey& k) const noexcept
	{
		uint64_t h = static_cast<uint32_t>(k.v[0]);
		for (int i = 1; i < 4; ++i)
		{
			h = h * 0x9E3779B97F4A7C15ull + static_cast<uint32_t>(k.v[i]);
		}
		h ^= (h >> 33);
		return static_cast<size_t>(h);
	}
};

inline FaceKey makeFaceKey(int a, int b, int c, int d)
{
	int t[4] = {a, b, c, d};
	std::sort(t, t + 4);
	FaceKey k;
	k.v[0] = t[0]; k.v[1] = t[1]; k.v[2] = t[2]; k.v[3] = t[3];
	return k;
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
// run(): adaptive refine + cull + snap → 直接写 polyMesh 目录
// ============================================================================

namespace
{

// 写一个 OF 风格 FoamFile 头。
void writeFoamFileHeader(std::ofstream& os, const char* clazz, const char* location, const char* object)
{
	os << "/*--------------------------------*- C++ -*----------------------------------*\\\n"
	   << "| =========                 |                                                 |\n"
	   << "| \\\\      /  F ield         | XFoam (myfoam snappyHexMesh utility)            |\n"
	   << "|  \\\\    /   O peration     |                                                 |\n"
	   << "|   \\\\  /    A nd           |                                                 |\n"
	   << "|    \\\\/     M anipulation  |                                                 |\n"
	   << "\\*---------------------------------------------------------------------------*/\n"
	   << "FoamFile\n{\n"
	   << "    version     2.0;\n"
	   << "    format      ascii;\n"
	   << "    class       " << clazz << ";\n"
	   << "    location    \"" << location << "\";\n"
	   << "    object      " << object << ";\n"
	   << "}\n"
	   << "// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n\n";
}

void writeFoamTail(std::ofstream& os)
{
	os << "\n// ************************************************************************* //\n";
}

bool writePolyMeshFiles(
	const std::string& dir,
	const std::vector<XFoam_Vector3D>& pts,
	const std::vector<std::vector<int>>& faces,
	const std::vector<int>& owner,
	const std::vector<int>& neighbour,
	XFoam_Label nInternal,
	const std::vector<std::string>& patchNames,
	const std::vector<std::string>& patchTypes,
	const std::vector<XFoam_Label>& patchStart,
	const std::vector<XFoam_Label>& patchSize)
{
	namespace fs = boost::filesystem;
	fs::create_directories(dir);

	// points
	{
		std::ofstream f(dir + "/points");
		if (!f) return false;
		writeFoamFileHeader(f, "vectorField", "constant/polyMesh", "points");
		f << pts.size() << "\n(\n";
		f << std::setprecision(15);
		for (size_t i = 0; i < pts.size(); ++i)
		{
			f << "(" << pts[i].x() << " " << pts[i].y() << " " << pts[i].z() << ")\n";
		}
		f << ")\n";
		writeFoamTail(f);
	}

	// faces
	{
		std::ofstream f(dir + "/faces");
		if (!f) return false;
		writeFoamFileHeader(f, "faceList", "constant/polyMesh", "faces");
		f << faces.size() << "\n(\n";
		for (size_t i = 0; i < faces.size(); ++i)
		{
			const auto& fc = faces[i];
			f << fc.size() << "(";
			for (size_t k = 0; k < fc.size(); ++k)
			{
				if (k) f << ' ';
				f << fc[k];
			}
			f << ")\n";
		}
		f << ")\n";
		writeFoamTail(f);
	}

	// owner
	{
		std::ofstream f(dir + "/owner");
		if (!f) return false;
		writeFoamFileHeader(f, "labelList", "constant/polyMesh", "owner");
		f << owner.size() << "\n(\n";
		for (size_t i = 0; i < owner.size(); ++i) f << owner[i] << "\n";
		f << ")\n";
		writeFoamTail(f);
	}

	// neighbour (only internal faces)
	{
		std::ofstream f(dir + "/neighbour");
		if (!f) return false;
		writeFoamFileHeader(f, "labelList", "constant/polyMesh", "neighbour");
		f << nInternal << "\n(\n";
		for (XFoam_Label i = 0; i < nInternal; ++i) f << neighbour[i] << "\n";
		f << ")\n";
		writeFoamTail(f);
	}

	// boundary
	{
		std::ofstream f(dir + "/boundary");
		if (!f) return false;
		writeFoamFileHeader(f, "polyBoundaryMesh", "constant/polyMesh", "boundary");
		f << patchNames.size() << "\n(\n";
		for (size_t p = 0; p < patchNames.size(); ++p)
		{
			f << "    " << patchNames[p] << "\n"
			  << "    {\n"
			  << "        type            " << (patchTypes[p].empty() ? "patch" : patchTypes[p]) << ";\n"
			  << "        nFaces          " << patchSize[p] << ";\n"
			  << "        startFace       " << patchStart[p] << ";\n"
			  << "    }\n";
		}
		f << ")\n";
		writeFoamTail(f);
	}
	return true;
}

// 双线性插值在 base-cell A 内 (u, v, w) ∈ [0,1]^3 处的物理点。
// 这里走的是把 base-cell A 在背景 hex 中的 (u', v', w') 参数空间换算后再三线性 hex 求位置。
inline XFoam_Vector3D paramToWorld(
	const XFoam_Vector3D blockCorner[8],
	int Nx, int Ny, int Nz,
	int ai, int aj, int ak,
	XFoam_Scalar u, XFoam_Scalar v, XFoam_Scalar w)
{
	const XFoam_Scalar U = (static_cast<XFoam_Scalar>(ai) + u) / Nx;
	const XFoam_Scalar V = (static_cast<XFoam_Scalar>(aj) + v) / Ny;
	const XFoam_Scalar W = (static_cast<XFoam_Scalar>(ak) + w) / Nz;
	const XFoam_Scalar ui = 1 - U, vi = 1 - V, wi = 1 - W;
	XFoam_Vector3D p(0, 0, 0);
	p += blockCorner[0] * (ui * vi * wi);
	p += blockCorner[1] * (U  * vi * wi);
	p += blockCorner[2] * (U  * V  * wi);
	p += blockCorner[3] * (ui * V  * wi);
	p += blockCorner[4] * (ui * vi * W);
	p += blockCorner[5] * (U  * vi * W);
	p += blockCorner[6] * (U  * V  * W);
	p += blockCorner[7] * (ui * V  * W);
	return p;
}

} // anon namespace

bool XFoam_SnappyHexMesh::run(
	const XFoam_BlockMesh& bg,
	const XFoam_TriSurface& stl,
	const XFoam_FileName& outPolyMeshDir,
	Stats& stats) const
{
	stats = Stats();
	stats.refinementLevel = globalLevel_;
	stats.stlPatchName = firstSurfaceName_.empty() ? XFoam_Word("snappy") : firstSurfaceName_;

	// ----- 0. 前置校验 -----
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

	// ----- 1. block-0 物理 8 顶点 + 基网格分辨率 -----
	const XFoam_BlockDescriptor& bd = bg[0];
	const XFoam_CellShape& shape0 = bd.blockShape();
	const XFoam_UList<XFoam_Vector3D>& bv = bd.vertices();
	XFoam_Vector3D blockCorner[8];
	XFoam_Vector3D bbMin = bv[shape0[0]] * bg.scaleFactor();
	XFoam_Vector3D bbMax = bbMin;
	for (int i = 0; i < 8; ++i)
	{
		blockCorner[i] = bv[shape0[i]] * bg.scaleFactor();
		bbMin.x() = std::min(bbMin.x(), blockCorner[i].x());
		bbMin.y() = std::min(bbMin.y(), blockCorner[i].y());
		bbMin.z() = std::min(bbMin.z(), blockCorner[i].z());
		bbMax.x() = std::max(bbMax.x(), blockCorner[i].x());
		bbMax.y() = std::max(bbMax.y(), blockCorner[i].y());
		bbMax.z() = std::max(bbMax.z(), blockCorner[i].z());
	}
	const int Nx = static_cast<int>(bd.density().x());
	const int Ny = static_cast<int>(bd.density().y());
	const int Nz = static_cast<int>(bd.density().z());
	stats.nBgCells = static_cast<XFoam_Label>(Nx) * Ny * Nz;

	// 点 dedup 容差：以背景 bbox 对角为尺度，1e-9 相对量级。
	const XFoam_Scalar bbDiag = (bbMax - bbMin).mag();
	const XFoam_Scalar eps = std::max<XFoam_Scalar>(1e-12, bbDiag * static_cast<XFoam_Scalar>(1e-9));
	const double invEps = 1.0 / static_cast<double>(eps);
	const int LEVEL_CAP = 4;
	const int targetLevel = std::min<int>(static_cast<int>(globalLevel_), LEVEL_CAP);

	auto baseIdx = [&](int i, int j, int k) -> int { return i + j * Nx + k * Nx * Ny; };
	auto inGrid  = [&](int i, int j, int k) -> bool {
		return i >= 0 && i < Nx && j >= 0 && j < Ny && k >= 0 && k < Nz;
	};

	// ----- 2. 每个 base-cell 的 level 初值（基于 STL bbox 相交粗筛）-----
	std::vector<int> level(static_cast<size_t>(Nx * Ny * Nz), 0);
	for (int k = 0; k < Nz; ++k)
	{
		for (int j = 0; j < Ny; ++j)
		{
			for (int i = 0; i < Nx; ++i)
			{
				// 用 8 个角点的 axis-aligned bbox 做粗筛。base-cell 不是严格轴对齐时这是包络。
				XFoam_Vector3D cmin = paramToWorld(blockCorner, Nx, Ny, Nz, i, j, k, 0, 0, 0);
				XFoam_Vector3D cmax = cmin;
				for (int oc = 0; oc < 8; ++oc)
				{
					const XFoam_Scalar u = (oc & 1) ? 1.0 : 0.0;
					const XFoam_Scalar v = (oc & 2) ? 1.0 : 0.0;
					const XFoam_Scalar w = (oc & 4) ? 1.0 : 0.0;
					const XFoam_Vector3D p = paramToWorld(blockCorner, Nx, Ny, Nz, i, j, k, u, v, w);
					cmin.x() = std::min(cmin.x(), p.x()); cmax.x() = std::max(cmax.x(), p.x());
					cmin.y() = std::min(cmin.y(), p.y()); cmax.y() = std::max(cmax.y(), p.y());
					cmin.z() = std::min(cmin.z(), p.z()); cmax.z() = std::max(cmax.z(), p.z());
				}
				const XFoam_BoundBox cbb(cmin, cmax);
				if (stl.boxIntersects(cbb))
				{
					level[baseIdx(i, j, k)] = targetLevel;
				}
			}
		}
	}

	// ----- 2b. nCellsBetweenLevels 缓冲扩张 -----
	// 每一轮：cell 的 level 至少为 (邻居 level - 1)。重复 nCellsBetweenLevels 次。
	const int nBuffer = std::max<int>(1, static_cast<int>(refine_.nCellsBetweenLevels));
	for (int it = 0; it < nBuffer * targetLevel; ++it)
	{
		std::vector<int> nxt = level;
		bool changed = false;
		for (int k = 0; k < Nz; ++k)
		{
			for (int j = 0; j < Ny; ++j)
			{
				for (int i = 0; i < Nx; ++i)
				{
					int target = nxt[baseIdx(i, j, k)];
					for (int d = 0; d < 6; ++d)
					{
						const int ni = i + kFaceDir[d][0];
						const int nj = j + kFaceDir[d][1];
						const int nk = k + kFaceDir[d][2];
						if (!inGrid(ni, nj, nk)) continue;
						target = std::max(target, level[baseIdx(ni, nj, nk)] - 1);
					}
					if (target != nxt[baseIdx(i, j, k)])
					{
						nxt[baseIdx(i, j, k)] = target;
						changed = true;
					}
				}
			}
		}
		level.swap(nxt);
		if (!changed) break;
	}

	for (int i = 0; i < Nx * Ny * Nz; ++i)
	{
		const int L = std::min(LEVEL_CAP, std::max(0, level[i]));
		level[i] = L;
		stats.maxAdaptiveLevel = std::max<XFoam_Label>(stats.maxAdaptiveLevel, L);
		if (L < 8) ++stats.perLevelCells[L];
	}

	// ----- 3. 全局点表 + 每个 base-cell 的 sub-cell 顶点表 -----
	std::vector<XFoam_Vector3D> pts;
	pts.reserve(static_cast<size_t>(Nx * Ny * Nz) * 8);
	std::unordered_map<PointKey, int, PointKeyHash> ptIdx;
	ptIdx.reserve(static_cast<size_t>(Nx * Ny * Nz) * 8);

	auto addPoint = [&](const XFoam_Vector3D& p) -> int {
		PointKey k{
			static_cast<int64_t>(std::llround(p.x() * invEps)),
			static_cast<int64_t>(std::llround(p.y() * invEps)),
			static_cast<int64_t>(std::llround(p.z() * invEps))
		};
		auto it = ptIdx.find(k);
		if (it != ptIdx.end()) return it->second;
		const int id = static_cast<int>(pts.size());
		pts.push_back(p);
		ptIdx.emplace(k, id);
		return id;
	};

	// 把每个 base-cell A 内部的 (n+1)^3 个点（n = 2^L）一次性生成、加到全局点表里，
	// 并记录在 cellPts[ai][aj][ak] -> 3D 数组。考虑到内存，按 base-cell 临时构造。
	struct BaseCellPts
	{
		int n = 0;
		std::vector<int> ids; // size (n+1)^3
		inline int& at(int i, int j, int k) { return ids[i + j * (n + 1) + k * (n + 1) * (n + 1)]; }
		inline int  at(int i, int j, int k) const { return ids[i + j * (n + 1) + k * (n + 1) * (n + 1)]; }
	};
	std::vector<BaseCellPts> basePts(static_cast<size_t>(Nx * Ny * Nz));

	for (int k = 0; k < Nz; ++k)
	{
		for (int j = 0; j < Ny; ++j)
		{
			for (int i = 0; i < Nx; ++i)
			{
				const int idxA = baseIdx(i, j, k);
				const int L = level[idxA];
				const int n = 1 << L;
				BaseCellPts& bp = basePts[idxA];
				bp.n = n;
				bp.ids.assign(static_cast<size_t>((n + 1) * (n + 1) * (n + 1)), -1);
				for (int kk = 0; kk <= n; ++kk)
				{
					const XFoam_Scalar w = static_cast<XFoam_Scalar>(kk) / n;
					for (int jj = 0; jj <= n; ++jj)
					{
						const XFoam_Scalar v = static_cast<XFoam_Scalar>(jj) / n;
						for (int ii = 0; ii <= n; ++ii)
						{
							const XFoam_Scalar u = static_cast<XFoam_Scalar>(ii) / n;
							const XFoam_Vector3D p = paramToWorld(blockCorner, Nx, Ny, Nz, i, j, k, u, v, w);
							bp.at(ii, jj, kk) = addPoint(p);
						}
					}
				}
			}
		}
	}

	// ----- 4. 生成 sub-cells；按 STL 中心点判定 keep -----
	struct SubCell
	{
		int corner[8];
		int ai, aj, ak;
		int si, sj, sk;
		int level;
		bool kept;
		int  globalId = -1;
		int  faceEffLevel[6] = {0, 0, 0, 0, 0, 0};
	};
	std::vector<SubCell> subCells;
	subCells.reserve(static_cast<size_t>(Nx * Ny * Nz) * 4);

	// 每个 base-cell 在 subCells 数组中的起点 + sub-cell stride（n^3）。
	std::vector<int> baseSubStart(static_cast<size_t>(Nx * Ny * Nz), -1);

	const bool locInside = stl.contains(refine_.locationInMesh);

	for (int k = 0; k < Nz; ++k)
	{
		for (int j = 0; j < Ny; ++j)
		{
			for (int i = 0; i < Nx; ++i)
			{
				const int idxA = baseIdx(i, j, k);
				const int L = level[idxA];
				const int n = 1 << L;
				baseSubStart[idxA] = static_cast<int>(subCells.size());
				const BaseCellPts& bp = basePts[idxA];
				for (int sk = 0; sk < n; ++sk)
				{
					for (int sj = 0; sj < n; ++sj)
					{
						for (int si = 0; si < n; ++si)
						{
							SubCell sc;
							sc.ai = i; sc.aj = j; sc.ak = k;
							sc.si = si; sc.sj = sj; sc.sk = sk;
							sc.level = L;
							sc.corner[0] = bp.at(si,     sj,     sk);
							sc.corner[1] = bp.at(si + 1, sj,     sk);
							sc.corner[2] = bp.at(si + 1, sj + 1, sk);
							sc.corner[3] = bp.at(si,     sj + 1, sk);
							sc.corner[4] = bp.at(si,     sj,     sk + 1);
							sc.corner[5] = bp.at(si + 1, sj,     sk + 1);
							sc.corner[6] = bp.at(si + 1, sj + 1, sk + 1);
							sc.corner[7] = bp.at(si,     sj + 1, sk + 1);
							const XFoam_Scalar uc = (static_cast<XFoam_Scalar>(si) + 0.5) / n;
							const XFoam_Scalar vc = (static_cast<XFoam_Scalar>(sj) + 0.5) / n;
							const XFoam_Scalar wc = (static_cast<XFoam_Scalar>(sk) + 0.5) / n;
							const XFoam_Vector3D centroid =
								paramToWorld(blockCorner, Nx, Ny, Nz, i, j, k, uc, vc, wc);
							sc.kept = (stl.contains(centroid) == locInside);
							subCells.push_back(sc);
						}
					}
				}
			}
		}
	}
	stats.nRefinedCells = static_cast<XFoam_Label>(subCells.size());

	// 每个 base-cell 的 6 面 effective level：max(L_A, L_neighbour)（不在网格内的取 L_A）。
	for (int k = 0; k < Nz; ++k)
	{
		for (int j = 0; j < Ny; ++j)
		{
			for (int i = 0; i < Nx; ++i)
			{
				const int idxA = baseIdx(i, j, k);
				const int L = level[idxA];
				const int n = 1 << L;
				int eff[6];
				for (int d = 0; d < 6; ++d)
				{
					const int ni = i + kFaceDir[d][0];
					const int nj = j + kFaceDir[d][1];
					const int nk = k + kFaceDir[d][2];
					eff[d] = inGrid(ni, nj, nk) ? std::max(L, level[baseIdx(ni, nj, nk)]) : L;
				}
				const int start = baseSubStart[idxA];
				for (int sk = 0; sk < n; ++sk)
				{
					for (int sj = 0; sj < n; ++sj)
					{
						for (int si = 0; si < n; ++si)
						{
							const int local = si + sj * n + sk * n * n;
							SubCell& sc = subCells[static_cast<size_t>(start + local)];
							// 仅在 sub-cell 真正落在 base-cell 该 face 上时，才把 effLevel 传过去。
							sc.faceEffLevel[0] = (sk == 0)     ? eff[0] : L;
							sc.faceEffLevel[1] = (sk == n - 1) ? eff[1] : L;
							sc.faceEffLevel[2] = (sj == 0)     ? eff[2] : L;
							sc.faceEffLevel[3] = (sj == n - 1) ? eff[3] : L;
							sc.faceEffLevel[4] = (si == 0)     ? eff[4] : L;
							sc.faceEffLevel[5] = (si == n - 1) ? eff[5] : L;
						}
					}
				}
			}
		}
	}

	// ----- 5. 给 kept sub-cells 编 globalId -----
	XFoam_Label nextCellId = 0;
	for (size_t s = 0; s < subCells.size(); ++s)
	{
		if (subCells[s].kept) subCells[s].globalId = static_cast<int>(nextCellId++);
	}
	stats.nKeptCells = nextCellId;

	// ----- 6. 产生所有面（按 dedup）+ 决定 owner/neighbour/patch -----
	// face_map: sorted-4-tuple → 在 finalFaces 数组里的索引
	struct FInfo
	{
		std::vector<int> verts;       // 该 face 的真正多边形（保 ccw owner-out 时再翻；这里只存）
		int owner = -1;
		int neighbour = -1;
		int patch = -1;               // -1 表示尚未归类
		bool fromGridBoundary = false; // 仅当 boundary 时有意义
	};
	std::vector<FInfo> finalFaces;
	std::unordered_map<FaceKey, int, FaceKeyHash> faceIdx;
	finalFaces.reserve(subCells.size() * 6);
	faceIdx.reserve(subCells.size() * 6);

	// 把一个 quad（4 顶点）加入 finalFaces 或合并到已有面。
	// quadOwnerIsLow: true 表示该 quad 的 cell 在 sorted-key 里作为"较小"的一侧来源；
	//                 这里我们不区分，靠 owner/neighbour 处理逻辑统一。
	// onGridBoundary: 当 cell 在背景 grid 边上、且没有有效邻居 sub-cell 时为 true（被 STL 切的不算）。
	auto emitQuad = [&](int v0, int v1, int v2, int v3, int cellId, bool onGridBoundary) {
		const FaceKey key = makeFaceKey(v0, v1, v2, v3);
		auto it = faceIdx.find(key);
		if (it == faceIdx.end())
		{
			FInfo f;
			f.verts = { v0, v1, v2, v3 };
			f.owner = cellId;
			f.neighbour = -1;
			f.fromGridBoundary = onGridBoundary;
			const int id = static_cast<int>(finalFaces.size());
			finalFaces.push_back(std::move(f));
			faceIdx.emplace(key, id);
		}
		else
		{
			FInfo& f = finalFaces[it->second];
			// 第二次遇到同一 face：变 internal。owner = min, neighbour = max。
			if (f.neighbour != -1)
			{
				// 同一 face 出现 3 次？拓扑异常。直接报错。
				std::cerr << "snappy: WARNING: face appearing 3+ times in dedup map (cells=" << f.owner
				          << "," << f.neighbour << "," << cellId << "). Mesh may be malformed.\n";
			}
			if (cellId < f.owner)
			{
				f.neighbour = f.owner;
				f.owner = cellId;
				std::reverse(f.verts.begin(), f.verts.end()); // 翻面让 normal 指向新 neighbour
			}
			else
			{
				f.neighbour = cellId;
			}
			f.fromGridBoundary = false;
		}
	};

	// 对一个 base-cell 的某个 face（方向 d）上的某个 sub-cell：
	// 该 sub-cell 在 base-cell A 的 face d 上的"局部 (s_a, s_b)"，
	// face 的 4 个本地（sub-cell）顶点序号在 kHexFace[d] 里。
	// 当 effLevel[d] > sc.level 时，要把该 quad 切成 2x2 sub-quads，
	// 用 base-cell A 在 face d 上更细一格的 4 个边中点 + 中心点 Steiner 点。
	// 这些 Steiner 点的坐标和邻居 base-cell B 在该 face 上产生的点是同一物理点，
	// 进 addPoint 时会 dedup 上。
	auto cornerOfBase = [&](int ai, int aj, int ak, int xi, int yj, int zk) -> int {
		const BaseCellPts& bp = basePts[baseIdx(ai, aj, ak)];
		return bp.at(xi, yj, zk);
	};

	// 把 base-cell A 在 face d 上 (sub_a, sub_b) 子格的 4 个粗角点位置（base-cell A 局部 ii/jj/kk）
	// 投到全局点 id。当 emit 普通 quad 时直接用 sc.corner[kHexFace[d][...]] 即可；split 模式下
	// 需要把同样的 4 角点 + 4 中点 + 中心点取出来，使用 base-cell A 的更细分辨率（2*n_A）来抽点。
	// 但 base-cell A 的点表只到 n_A 分辨率，没有中点。因此 split 时我们必须用邻居 base-cell B 的点表
	// （B 的分辨率更细，自带这些 Steiner 点）。
	//
	// helper: 在邻居 base-cell B 的对面（即 d 的反方向）上拿 (xb, yb, zb) 处的点 id。
	// B 在某个 face 上的局部坐标范围 0..n_B；对于 face d，我们需要的子格 ii/jj 在 B 内的下标:
	//   - 如果 d 是 -x: A 看的 (sj, sk) 等于 B 的 (sj_B, sk_B)，但 B 的 si_B 取 n_B-1（紧贴 A）
	//   - 等等。把 lookup 写成函数。
	auto neighborFacePoint = [&](int ai, int aj, int ak, int dirA,
	                             int faceI, int faceJ, int nA, int LB) -> int {
		const int ni = ai + kFaceDir[dirA][0];
		const int nj = aj + kFaceDir[dirA][1];
		const int nk = ak + kFaceDir[dirA][2];
		(void)nA;
		const int nB = 1 << LB;
		// A 上 face d 的 (faceI, faceJ) 局部坐标范围 0..nA*step；step = nB/nA。
		// 在 B 内的对应点：
		// 注：A 的 face d 与 B 的 face (d XOR 1) 在物理上是同一面。点的 (i,j,k) 在 B 里：
		const int ni_x = (dirA == 5 ? 0 : (dirA == 4 ? nB : -1));
		const int nj_y = (dirA == 3 ? 0 : (dirA == 2 ? nB : -1));
		const int nk_z = (dirA == 1 ? 0 : (dirA == 0 ? nB : -1));
		int xi = ni_x, yj = nj_y, zk = nk_z;
		// 把 face 本地 (faceI, faceJ) 填进 B 的非约束两个轴上。
		// face d == 0 or 1: 受约束的是 k (zk)，face 局部 (i, j) 对应 (xi, yj)。
		// face d == 2 or 3: 受约束的是 j (yj)，face 局部 (i, j) 对应 (xi, zk)。
		// face d == 4 or 5: 受约束的是 i (xi)，face 局部 (i, j) 对应 (yj, zk)。
		switch (dirA)
		{
		case 0: case 1: xi = faceI; yj = faceJ; break;
		case 2: case 3: xi = faceI; zk = faceJ; break;
		case 4: case 5: yj = faceI; zk = faceJ; break;
		}
		const BaseCellPts& bp = basePts[baseIdx(ni, nj, nk)];
		return bp.at(xi, yj, zk);
	};

	// 主循环：对每个 sub-cell 的 6 个 face，按 effLevel 决定 emit 一个 quad 还是 4 个 sub-quads。
	for (size_t s = 0; s < subCells.size(); ++s)
	{
		const SubCell& sc = subCells[s];
		if (!sc.kept) continue;
		const int idxA = baseIdx(sc.ai, sc.aj, sc.ak);
		const int LA = level[idxA];
		const int nA = 1 << LA;

		bool isPoly = false;

		for (int d = 0; d < 6; ++d)
		{
			// 取本地 4 顶点（sub-cell 局部 0..7 排列下）
			const int* fv = kHexFace[d];
			const int gv[4] = { sc.corner[fv[0]], sc.corner[fv[1]], sc.corner[fv[2]], sc.corner[fv[3]] };

			// 该 face 是 sub-cell 之间还是 base-cell 之间？
			//   - 如果该 face 不是 sub-cell 在 base-cell A 的 face-d 上的边界，则一定是 sub-cell 之间，
			//     直接 emit 一个 quad（dedup 会把双方面合上）。
			bool subOnBaseFace = false;
			switch (d)
			{
			case 0: subOnBaseFace = (sc.sk == 0); break;
			case 1: subOnBaseFace = (sc.sk == nA - 1); break;
			case 2: subOnBaseFace = (sc.sj == 0); break;
			case 3: subOnBaseFace = (sc.sj == nA - 1); break;
			case 4: subOnBaseFace = (sc.si == 0); break;
			case 5: subOnBaseFace = (sc.si == nA - 1); break;
			}

			if (!subOnBaseFace)
			{
				emitQuad(gv[0], gv[1], gv[2], gv[3], sc.globalId, false);
				continue;
			}

			// 在 base-cell A 的 face-d 上：决定该 sub-face 的 effective level
			const int LB = sc.faceEffLevel[d]; // = max(LA, L_neighbour_in_dir_d) 或 LA（无邻居）
			const int ni = sc.ai + kFaceDir[d][0];
			const int nj = sc.aj + kFaceDir[d][1];
			const int nk = sc.ak + kFaceDir[d][2];
			const bool hasNbr = inGrid(ni, nj, nk);

			if (LB == LA)
			{
				// 简单：发一个 quad。dedup 会与邻居（同 level 或没邻居/无 kept 邻居）会合。
				const bool onGridBnd = !hasNbr; // base-cell 不在 grid 边上但邻居 base-cell 也是 LA → 不是 grid 边
				emitQuad(gv[0], gv[1], gv[2], gv[3], sc.globalId, onGridBnd);
				continue;
			}

			// LB > LA：切 face。当前仅支持 diff = 1（即 LB = LA + 1）。
			const int diff = LB - LA;
			if (diff != 1)
			{
				// 退化处理：发一个 quad，告警；mesh 可能挂面。
				std::cerr << "snappy: WARNING: face level diff=" << diff
				          << " > 1 unsupported; emitting un-split quad.\n";
				emitQuad(gv[0], gv[1], gv[2], gv[3], sc.globalId, false);
				continue;
			}

			// 计算 A 上该 sub-face 在 face-d 上的局部 (faceI0, faceJ0) ~ (faceI0+1, faceJ0+1)
			// 单位是 A 的 sub-cell 步长 1；映射到 B 的细分坐标 (step = 2) 范围 (2*faceI0..2*faceI0+2)。
			int faceI0 = 0, faceJ0 = 0;
			switch (d)
			{
			case 0: case 1: faceI0 = sc.si; faceJ0 = sc.sj; break;
			case 2: case 3: faceI0 = sc.si; faceJ0 = sc.sk; break;
			case 4: case 5: faceI0 = sc.sj; faceJ0 = sc.sk; break;
			}
			// 在 B 内（细 face）该 sub-face 覆盖 (2*faceI0..2*faceI0+2) × (2*faceJ0..2*faceJ0+2)。
			// 拿 9 个 face 上 Steiner / corner 点 ID（从 B 的点表，dedup 后等于 A face 同位置）。
			// p[r][c] r,c ∈ {0,1,2}
			int p[3][3];
			for (int rr = 0; rr < 3; ++rr)
			{
				for (int cc = 0; cc < 3; ++cc)
				{
					const int fi = 2 * faceI0 + cc;
					const int fj = 2 * faceJ0 + rr;
					p[rr][cc] = neighborFacePoint(sc.ai, sc.aj, sc.ak, d, fi, fj, nA, LB);
				}
			}
			// 4 个 sub-quad，CCW 与原 quad 一致（按 (faceI, faceJ) 的 (cc, rr) 顺序）。
			// 原 quad 的本地顶点次序 fv[0..3] 是某一组逆时针。我们这里按 (cc, rr) 的 4 个 cell:
			//   [0..1][0..1], [1..2][0..1], [1..2][1..2], [0..1][1..2]
			// 每个 cell 4 顶点取 (p[r][c], p[r][c+1], p[r+1][c+1], p[r+1][c])。
			for (int rr = 0; rr < 2; ++rr)
			{
				for (int cc = 0; cc < 2; ++cc)
				{
					emitQuad(p[rr][cc], p[rr][cc + 1], p[rr + 1][cc + 1], p[rr + 1][cc],
					         sc.globalId, false);
				}
			}
			isPoly = true;
			++stats.nSplitFaces;
		}
		if (isPoly) ++stats.nPolyhedralCells;
	}

	// ----- 7. 把 faces 切成 internal / boundary, 给 boundary 分 patch -----
	// boundary patch: faces 都是单 cell 引用 (neighbour == -1)
	//   - fromGridBoundary == true → 背景 walls 顶面 → 原 BlockMesh.patchNames()[0]
	//   - 否则（dedup 只见 1 次但邻居 base-cell 存在，意味着邻居 sub-cell 被 STL 切掉）→ stl patch
	std::vector<int> internalIdx; internalIdx.reserve(finalFaces.size());
	std::vector<int> wallsIdx, stlIdx;
	for (int fi = 0; fi < static_cast<int>(finalFaces.size()); ++fi)
	{
		const FInfo& f = finalFaces[fi];
		if (f.neighbour != -1) internalIdx.push_back(fi);
		else if (f.fromGridBoundary) wallsIdx.push_back(fi);
		else stlIdx.push_back(fi);
	}

	// internal faces 按 (owner, neighbour) 升序排
	std::sort(internalIdx.begin(), internalIdx.end(), [&](int a, int b) {
		const FInfo& fa = finalFaces[a];
		const FInfo& fb = finalFaces[b];
		if (fa.owner != fb.owner) return fa.owner < fb.owner;
		return fa.neighbour < fb.neighbour;
	});
	// boundary faces 按 owner 升序，方便定位
	auto byOwner = [&](int a, int b) { return finalFaces[a].owner < finalFaces[b].owner; };
	std::sort(wallsIdx.begin(), wallsIdx.end(), byOwner);
	std::sort(stlIdx.begin(),   stlIdx.end(),   byOwner);

	// ----- 8. snap：把 STL patch 上独立点投到 STL 最近点 -----
	if (phases_.snap && !stlIdx.empty())
	{
		std::vector<unsigned char> snapped(pts.size(), 0);
		for (int fi : stlIdx)
		{
			const FInfo& f = finalFaces[fi];
			for (int v : f.verts)
			{
				if (snapped[v]) continue;
				snapped[v] = 1;
				XFoam_Vector3D closest, normal;
				stl.closestPointAndNormal(pts[v], closest, normal);
				const XFoam_Scalar d = (closest - pts[v]).mag();
				if (d > stats.maxSnapDistance) stats.maxSnapDistance = d;
				pts[v] = closest;
				++stats.nSnappedPoints;
			}
		}
	}

	// ----- 9. 装配 polyMesh 三大列表 + patch 表 -----
	std::vector<std::vector<int>> facesOut;
	std::vector<int> ownerOut, nbrOut;
	facesOut.reserve(finalFaces.size());
	ownerOut.reserve(finalFaces.size());
	nbrOut.reserve(internalIdx.size());

	for (int fi : internalIdx)
	{
		facesOut.push_back(finalFaces[fi].verts);
		ownerOut.push_back(finalFaces[fi].owner);
		nbrOut.push_back(finalFaces[fi].neighbour);
	}
	std::vector<std::string> patchNamesOut, patchTypesOut;
	std::vector<XFoam_Label> patchStartOut, patchSizeOut;
	auto addPatch = [&](const std::vector<int>& idxs, const std::string& name, const std::string& type) {
		if (idxs.empty()) return;
		patchNamesOut.push_back(name);
		patchTypesOut.push_back(type);
		patchStartOut.push_back(static_cast<XFoam_Label>(facesOut.size()));
		patchSizeOut.push_back(static_cast<XFoam_Label>(idxs.size()));
		for (int fi : idxs)
		{
			facesOut.push_back(finalFaces[fi].verts);
			ownerOut.push_back(finalFaces[fi].owner);
		}
	};
	const XFoam_WordList origNames = bg.patchNames();
	const XFoam_WordList& origTypes = bg.patchTypes();
	const std::string wallName = origNames.empty() ? std::string("walls")
		: std::string(static_cast<const std::string&>(static_cast<const XFoam_String&>(origNames[0])));
	const std::string wallType = origTypes.empty() ? std::string("wall")
		: std::string(static_cast<const std::string&>(static_cast<const XFoam_String&>(origTypes[0])));
	addPatch(wallsIdx, wallName, wallType);
	addPatch(stlIdx,
		std::string(static_cast<const std::string&>(static_cast<const XFoam_String&>(stats.stlPatchName))),
		std::string("wall"));

	stats.outPatchNames.setSize(static_cast<XFoam_Label>(patchNamesOut.size()));
	stats.outPatchTypes.setSize(static_cast<XFoam_Label>(patchTypesOut.size()));
	for (size_t p = 0; p < patchNamesOut.size(); ++p)
	{
		stats.outPatchNames[static_cast<XFoam_Label>(p)] = XFoam_Word(patchNamesOut[p]);
		stats.outPatchTypes[static_cast<XFoam_Label>(p)] = XFoam_Word(patchTypesOut[p]);
	}
	stats.nPoints = static_cast<XFoam_Label>(pts.size());
	stats.nFaces  = static_cast<XFoam_Label>(facesOut.size());
	stats.nInternalFaces = static_cast<XFoam_Label>(internalIdx.size());
	stats.nBoundaryFaces = stats.nFaces - stats.nInternalFaces;

	// ----- 10. 写文件 -----
	const std::string outDirStr = static_cast<const std::string&>(static_cast<const XFoam_String&>(outPolyMeshDir));
	if (!writePolyMeshFiles(outDirStr, pts, facesOut, ownerOut, nbrOut,
		stats.nInternalFaces, patchNamesOut, patchTypesOut, patchStartOut, patchSizeOut))
	{
		std::cerr << "snappy: failed to write polyMesh dir " << outDirStr << "\n";
		return false;
	}
	return true;
}
