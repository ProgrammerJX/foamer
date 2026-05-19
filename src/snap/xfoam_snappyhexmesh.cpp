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

	// ----- 2. Octree 数据结构 -----
	// 每个 base-cell 内部一棵八叉树；leaf 用 (ai, aj, ak, level, si, sj, sk) 唯一标识。
	// 不维护父/子指针 — 只存当前活跃 leaf 列表 + 一个 (encoded key → leaf index) 哈希表。
	// subdivide 直接把 leaf 替换为 8 个孩子（in-place 第一个 + 追加 7 个）。
	struct Leaf
	{
		int ai, aj, ak;
		int level;
		int si, sj, sk;
		bool kept = true;
		int  cellId = -1;
		int  corner[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
	};
	std::vector<Leaf> leaves;
	leaves.reserve(static_cast<size_t>(Nx) * Ny * Nz);

	for (int k = 0; k < Nz; ++k)
	{
		for (int j = 0; j < Ny; ++j)
		{
			for (int i = 0; i < Nx; ++i)
			{
				Leaf l;
				l.ai = i; l.aj = j; l.ak = k;
				l.level = 0; l.si = l.sj = l.sk = 0;
				leaves.push_back(l);
			}
		}
	}

	// 把 (level, si, sj, sk) 在 base-cell 内的 8 个 corner / axis-aligned bbox 拿出来。
	// 角点排列严格按 OpenFOAM hex 约定（与 kHexFace 表一致）：
	//   v0=(0,0,0) v1=(1,0,0) v2=(1,1,0) v3=(0,1,0)
	//   v4=(0,0,1) v5=(1,0,1) v6=(1,1,1) v7=(0,1,1)
	// 注意 v2 是 (1,1,0)、v3 是 (0,1,0) — 用 bit 位编码 (oc&1, oc&2, oc&4) 会把 v2/v3、v6/v7 颠倒，
	// 走 kHexFace[d] 取面时取到对角顶点 → 面变成扭曲四边形 → dedup / 物理含义全乱。
	static const int kOfU[8] = {0, 1, 1, 0, 0, 1, 1, 0};
	static const int kOfV[8] = {0, 0, 1, 1, 0, 0, 1, 1};
	static const int kOfW[8] = {0, 0, 0, 0, 1, 1, 1, 1};
	auto leafCornersWorld = [&](const Leaf& l, XFoam_Vector3D out[8]) {
		const int n = 1 << l.level;
		for (int oc = 0; oc < 8; ++oc)
		{
			const XFoam_Scalar u = static_cast<XFoam_Scalar>(l.si + kOfU[oc]) / n;
			const XFoam_Scalar v = static_cast<XFoam_Scalar>(l.sj + kOfV[oc]) / n;
			const XFoam_Scalar w = static_cast<XFoam_Scalar>(l.sk + kOfW[oc]) / n;
			out[oc] = paramToWorld(blockCorner, Nx, Ny, Nz, l.ai, l.aj, l.ak, u, v, w);
		}
	};
	auto leafBBox = [&](const Leaf& l) -> XFoam_BoundBox {
		XFoam_Vector3D c[8]; leafCornersWorld(l, c);
		XFoam_Vector3D mn = c[0], mx = c[0];
		for (int i = 1; i < 8; ++i)
		{
			mn.x() = std::min(mn.x(), c[i].x()); mx.x() = std::max(mx.x(), c[i].x());
			mn.y() = std::min(mn.y(), c[i].y()); mx.y() = std::max(mx.y(), c[i].y());
			mn.z() = std::min(mn.z(), c[i].z()); mx.z() = std::max(mx.z(), c[i].z());
		}
		return XFoam_BoundBox(mn, mx);
	};
	auto leafCentroid = [&](const Leaf& l) -> XFoam_Vector3D {
		const int n = 1 << l.level;
		return paramToWorld(blockCorner, Nx, Ny, Nz, l.ai, l.aj, l.ak,
			(static_cast<XFoam_Scalar>(l.si) + 0.5) / n,
			(static_cast<XFoam_Scalar>(l.sj) + 0.5) / n,
			(static_cast<XFoam_Scalar>(l.sk) + 0.5) / n);
	};

	// Subdivide：把 leaves[idx] 一拆 8（in-place 第一个 + push_back 7 个）。
	auto subdivide = [&](int idx) {
		const Leaf parent = leaves[idx];
		if (parent.level >= LEVEL_CAP) return;
		Leaf c0 = parent;
		c0.level = parent.level + 1;
		c0.si = 2 * parent.si;
		c0.sj = 2 * parent.sj;
		c0.sk = 2 * parent.sk;
		leaves[idx] = c0;
		for (int oc = 1; oc < 8; ++oc)
		{
			Leaf c = parent;
			c.level = parent.level + 1;
			c.si = 2 * parent.si + (oc & 1);
			c.sj = 2 * parent.sj + ((oc >> 1) & 1);
			c.sk = 2 * parent.sk + ((oc >> 2) & 1);
			leaves.push_back(c);
		}
	};

	// (ai, aj, ak, L, si, sj, sk) → 唯一 uint64 key（LEVEL_CAP <= 4 时 si/sj/sk < 16）。
	auto makeLeafKey = [](int ai, int aj, int ak, int L, int si, int sj, int sk) -> uint64_t {
		uint64_t k = static_cast<uint32_t>(ai);
		k = (k << 16) | static_cast<uint32_t>(aj);
		k = (k << 16) | static_cast<uint32_t>(ak);
		k = (k << 4)  | static_cast<uint32_t>(L);
		k = (k << 4)  | static_cast<uint32_t>(si);
		k = (k << 4)  | static_cast<uint32_t>(sj);
		k = (k << 4)  | static_cast<uint32_t>(sk);
		return k;
	};
	auto buildLeafMap = [&]() -> std::unordered_map<uint64_t, int> {
		std::unordered_map<uint64_t, int> m;
		m.reserve(leaves.size() * 2);
		for (size_t i = 0; i < leaves.size(); ++i)
		{
			const Leaf& l = leaves[i];
			m.emplace(makeLeafKey(l.ai, l.aj, l.ak, l.level, l.si, l.sj, l.sk), static_cast<int>(i));
		}
		return m;
	};

	// 直接根据 d/2 选 face 上的固定轴 + 两个自由轴。
	// 与 kHexFace 一致：d=0/1 → axis k (z); d=2/3 → axis j (y); d=4/5 → axis i (x)
	// (axA, axB) 是 face 上的 (cc, rr) 自由轴对应的 axis (0=x, 1=y, 2=z)。
	const int kFaceAxes[6][3] = {
		{2, 0, 1}, // d=0: -z; (axD=z, axA=x, axB=y)
		{2, 0, 1}, // d=1: +z
		{1, 0, 2}, // d=2: -y; (axD=y, axA=x, axB=z)
		{1, 0, 2}, // d=3: +y
		{0, 1, 2}, // d=4: -x; (axD=x, axA=y, axB=z)
		{0, 1, 2}  // d=5: +x
	};

	// 把 leaf 朝方向 d 走 1 格，返回邻居 base-cell + 该 base-cell 内 (ns_i, ns_j, ns_k) at level L.
	// 若邻居在另一个 base-cell 里，会自动 wrap (ns_X 从 -1 或 n 折回 n-1 或 0)。
	// 失败 (out of grid) → 返回 false。
	auto stepNeighborAtSameLevel = [&](const Leaf& l, int d,
	                                   int& ai_n, int& aj_n, int& ak_n,
	                                   int& ns_i, int& ns_j, int& ns_k) -> bool {
		ai_n = l.ai; aj_n = l.aj; ak_n = l.ak;
		ns_i = l.si; ns_j = l.sj; ns_k = l.sk;
		const int n = 1 << l.level;
		switch (d)
		{
		case 0: --ns_k; break;
		case 1: ++ns_k; break;
		case 2: --ns_j; break;
		case 3: ++ns_j; break;
		case 4: --ns_i; break;
		case 5: ++ns_i; break;
		}
		if (ns_i < 0)  { --ai_n; ns_i = n - 1; }
		else if (ns_i >= n) { ++ai_n; ns_i = 0; }
		if (ns_j < 0)  { --aj_n; ns_j = n - 1; }
		else if (ns_j >= n) { ++aj_n; ns_j = 0; }
		if (ns_k < 0)  { --ak_n; ns_k = n - 1; }
		else if (ns_k >= n) { ++ak_n; ns_k = 0; }
		return inGrid(ai_n, aj_n, ak_n);
	};

	// ----- 3. Phase 1: 按 STL bbox 加密 -----
	// 每轮：遍历当前所有 leaves，bbox 与 STL 相交且 level < target 的就 subdivide 成 8 子节点。
	// 子节点在下一轮再判定（直到不再变化）。
	{
		int iter = 0;
		bool any_subdivided = true;
		while (any_subdivided && iter < targetLevel + 4)
		{
			any_subdivided = false;
			const size_t prevSize = leaves.size();
			for (size_t i = 0; i < prevSize; ++i)
			{
				if (leaves[i].level >= targetLevel) continue;
				if (stl.boxIntersects(leafBBox(leaves[i])))
				{
					subdivide(static_cast<int>(i));
					any_subdivided = true;
				}
			}
			++iter;
		}
	}

	// ----- 4. Phase 2: 2:1 balance -----
	// 对每个 leaf 查 6 个 face 方向：如果任何邻居 level > self.level + 1，subdivide 自己。
	// 反复直到无变化（每轮重建 leafMap）。
	{
		int iter = 0;
		bool any_subdivided = true;
		while (any_subdivided && iter < (LEVEL_CAP + 2))
		{
			any_subdivided = false;
			auto leafMap = buildLeafMap();
			const size_t prevSize = leaves.size();
			for (size_t i = 0; i < prevSize; ++i)
			{
				const Leaf& l = leaves[i];
				if (l.level >= LEVEL_CAP) continue;
				int maxNbr = -1;
				for (int d = 0; d < 6; ++d)
				{
					int ai_n, aj_n, ak_n, ns_i, ns_j, ns_k;
					if (!stepNeighborAtSameLevel(l, d, ai_n, aj_n, ak_n, ns_i, ns_j, ns_k)) continue;
					// 在 leafMap 里找比 self 更细的邻居 (L+2 起)；只要找到一个，就说明 balance 被破坏。
					for (int Lc = l.level + 2; Lc <= LEVEL_CAP; ++Lc)
					{
						const int factor = 1 << (Lc - l.level);
						const int axD = kFaceAxes[d][0];
						const int axA = kFaceAxes[d][1];
						const int axB = kFaceAxes[d][2];
						const int offD = (d % 2 == 0) ? (factor - 1) : 0;
						const int ns_arr[3] = {ns_i, ns_j, ns_k};
						bool found = false;
						for (int a = 0; a < factor && !found; ++a)
						{
							for (int b = 0; b < factor && !found; ++b)
							{
								int fp[3];
								fp[axD] = ns_arr[axD] * factor + offD;
								fp[axA] = ns_arr[axA] * factor + a;
								fp[axB] = ns_arr[axB] * factor + b;
								auto it = leafMap.find(makeLeafKey(ai_n, aj_n, ak_n, Lc, fp[0], fp[1], fp[2]));
								if (it != leafMap.end()) { found = true; if (Lc > maxNbr) maxNbr = Lc; }
							}
						}
						if (found) break; // 找到一个最大 level 就够了
					}
				}
				if (maxNbr > l.level + 1)
				{
					subdivide(static_cast<int>(i));
					any_subdivided = true;
				}
			}
			++iter;
		}
	}

	// ----- 5. 统计 per-level cell 数 -----
	for (const Leaf& l : leaves)
	{
		stats.maxAdaptiveLevel = std::max<XFoam_Label>(stats.maxAdaptiveLevel, l.level);
		if (l.level < 8) ++stats.perLevelCells[l.level];
	}
	stats.nRefinedCells = static_cast<XFoam_Label>(leaves.size());

	// ----- 6. STL 切除：用 leaf 中心做 inside/outside 判定 -----
	const bool locInside = stl.contains(refine_.locationInMesh);
	for (Leaf& l : leaves)
	{
		l.kept = (stl.contains(leafCentroid(l)) == locInside);
	}

	// ----- 7. kept leaves 编 globalId -----
	XFoam_Label nextCellId = 0;
	for (Leaf& l : leaves)
	{
		if (l.kept) l.cellId = static_cast<int>(nextCellId++);
	}
	stats.nKeptCells = nextCellId;

	// ----- 8. 全局点表 + addPoint -----
	std::vector<XFoam_Vector3D> pts;
	pts.reserve(leaves.size() * 8);
	std::unordered_map<PointKey, int, PointKeyHash> ptIdx;
	ptIdx.reserve(leaves.size() * 8);
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

	// ----- 9. 给所有 leaf（kept 或不 kept）生成 8 个 corner 点 -----
	// 不 kept 的也要做：粗侧 split-face 的 Steiner 点会被细侧（可能 kept 也可能不 kept）的
	// corner 点 dedup 上；如果不生成，dedup 链不全。
	for (Leaf& l : leaves)
	{
		XFoam_Vector3D c[8]; leafCornersWorld(l, c);
		for (int oc = 0; oc < 8; ++oc) l.corner[oc] = addPoint(c[oc]);
	}

	// ----- 10. Face emit -----
	struct FInfo
	{
		std::vector<int> verts;        // polygon (CCW with normal pointing owner→neighbour)
		int  owner = -1;
		int  neighbour = -1;
		bool fromGridBoundary = false; // 仅 boundary 时区分 walls vs stl patch
	};
	std::vector<FInfo> finalFaces;
	std::unordered_map<FaceKey, int, FaceKeyHash> faceIdx;
	finalFaces.reserve(leaves.size() * 6);
	faceIdx.reserve(leaves.size() * 6);

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
			if (f.neighbour != -1)
			{
				std::cerr << "snappy: WARNING: face appearing 3+ times in dedup (cells="
				          << f.owner << "," << f.neighbour << "," << cellId << ")\n";
			}
			if (cellId < f.owner)
			{
				f.neighbour = f.owner;
				f.owner = cellId;
				std::reverse(f.verts.begin(), f.verts.end());
			}
			else
			{
				f.neighbour = cellId;
			}
			f.fromGridBoundary = false;
		}
	};

	// 在 leaf l 的 face d 上，按 (rr, cc) ∈ {0..2}^2 取 L+1 分辨率的 face 9 点之一。
	// 用 paramToWorld + addPoint 落点；如果细侧 leaf 的 corner 已经生成，dedup 直接对上。
	auto faceFinePoint = [&](const Leaf& l, int d, int rr, int cc) -> int {
		const int axD = kFaceAxes[d][0];
		const int axA = kFaceAxes[d][1];
		const int axB = kFaceAxes[d][2];
		// d 为负方向 (-z, -y, -x): face 在 L 层 sX 处，对应 L+1 层 2*sX。
		// d 为正方向 (+z, +y, +x): face 在 L 层 sX+1 处，对应 L+1 层 2*sX+2。
		const int offD = (d % 2 == 0) ? 0 : 2;
		const int s_arr[3] = {l.si, l.sj, l.sk};
		int fp[3];
		fp[axD] = 2 * s_arr[axD] + offD;
		fp[axA] = 2 * s_arr[axA] + cc;
		fp[axB] = 2 * s_arr[axB] + rr;
		const int nF = 1 << (l.level + 1);
		const XFoam_Vector3D p = paramToWorld(blockCorner, Nx, Ny, Nz, l.ai, l.aj, l.ak,
			static_cast<XFoam_Scalar>(fp[0]) / nF,
			static_cast<XFoam_Scalar>(fp[1]) / nF,
			static_cast<XFoam_Scalar>(fp[2]) / nF);
		return addPoint(p);
	};

	auto leafMap = buildLeafMap();

	// 在邻居 base-cell 内查 same-level / coarser / finer 邻居。
	// 返回值：
	//   - same: leafIdx >= 0
	//   - coarser: leafIdx >= 0 + coarserLevel < self.level
	//   - finer: leafIdx 4 个一组（rr*2+cc），可能为 -1
	// fineIdx 仅在 finer 模式有意义。
	enum class NbrKind { OutOfGrid, Same, Coarser, Finer, None };

	auto resolveFaceNeighbor = [&](const Leaf& l, int d,
	                               NbrKind& kind, int& leafIdx, int fineIdx[4]) {
		fineIdx[0] = fineIdx[1] = fineIdx[2] = fineIdx[3] = -1;
		leafIdx = -1;
		int ai_n, aj_n, ak_n, ns_i, ns_j, ns_k;
		if (!stepNeighborAtSameLevel(l, d, ai_n, aj_n, ak_n, ns_i, ns_j, ns_k))
		{
			kind = NbrKind::OutOfGrid;
			return;
		}
		// Same-level
		auto it_s = leafMap.find(makeLeafKey(ai_n, aj_n, ak_n, l.level, ns_i, ns_j, ns_k));
		if (it_s != leafMap.end()) { kind = NbrKind::Same; leafIdx = it_s->second; return; }
		// Coarser (走到根)
		for (int Lq = l.level - 1; Lq >= 0; --Lq)
		{
			const int shift = l.level - Lq;
			auto it_c = leafMap.find(makeLeafKey(ai_n, aj_n, ak_n, Lq,
			                                     ns_i >> shift, ns_j >> shift, ns_k >> shift));
			if (it_c != leafMap.end()) { kind = NbrKind::Coarser; leafIdx = it_c->second; return; }
		}
		// Finer at L+1（2:1 balance 后保证最多差 1）
		const int axD = kFaceAxes[d][0];
		const int axA = kFaceAxes[d][1];
		const int axB = kFaceAxes[d][2];
		const int offD = (d % 2 == 0) ? 1 : 0;
		const int ns_arr[3] = {ns_i, ns_j, ns_k};
		bool anyFine = false;
		for (int rr = 0; rr < 2; ++rr)
		{
			for (int cc = 0; cc < 2; ++cc)
			{
				int fp[3];
				fp[axD] = 2 * ns_arr[axD] + offD;
				fp[axA] = 2 * ns_arr[axA] + cc;
				fp[axB] = 2 * ns_arr[axB] + rr;
				auto it_f = leafMap.find(makeLeafKey(ai_n, aj_n, ak_n, l.level + 1, fp[0], fp[1], fp[2]));
				if (it_f != leafMap.end()) { fineIdx[rr * 2 + cc] = it_f->second; anyFine = true; }
			}
		}
		kind = anyFine ? NbrKind::Finer : NbrKind::None;
	};

	for (size_t s = 0; s < leaves.size(); ++s)
	{
		const Leaf& l = leaves[s];
		if (!l.kept) continue;
		bool isPoly = false;

		for (int d = 0; d < 6; ++d)
		{
			const int* fv = kHexFace[d];
			const int gv[4] = { l.corner[fv[0]], l.corner[fv[1]], l.corner[fv[2]], l.corner[fv[3]] };

			NbrKind kind;
			int nbrIdx;
			int fineIdx[4];
			resolveFaceNeighbor(l, d, kind, nbrIdx, fineIdx);

			if (kind == NbrKind::OutOfGrid)
			{
				// background grid 边 → walls patch
				emitQuad(gv[0], gv[1], gv[2], gv[3], l.cellId, true);
				continue;
			}
			if (kind == NbrKind::Same || kind == NbrKind::Coarser)
			{
				// 同 level 或粗一级邻居：发一个 quad。粗侧自己会发 split sub-quad 与我对齐。
				// 若邻居不 kept，本面 dedup 失败 → 留作 boundary (stl patch)。
				emitQuad(gv[0], gv[1], gv[2], gv[3], l.cellId, false);
				continue;
			}
			if (kind == NbrKind::Finer)
			{
				// 我是粗侧：把这面切成 2x2 = 4 个 sub-quad
				int pgrid[3][3];
				for (int rr = 0; rr < 3; ++rr)
				{
					for (int cc = 0; cc < 3; ++cc)
					{
						pgrid[rr][cc] = faceFinePoint(l, d, rr, cc);
					}
				}
				for (int rr = 0; rr < 2; ++rr)
				{
					for (int cc = 0; cc < 2; ++cc)
					{
						emitQuad(pgrid[rr][cc], pgrid[rr][cc + 1],
						         pgrid[rr + 1][cc + 1], pgrid[rr + 1][cc],
						         l.cellId, false);
						++stats.nSplitFaces;
						(void)fineIdx; // 细侧是否 kept 不影响 emit 决策 — 它们自己发或者不发，dedup 会处理
					}
				}
				isPoly = true;
				continue;
			}
			// NbrKind::None：邻居 base-cell 存在但 leafMap 里找不到 leaf — 不该发生
			std::cerr << "snappy: WARNING: leaf (" << l.ai << "," << l.aj << "," << l.ak << ", L=" << l.level
			          << ") face d=" << d << " has no neighbor leaf in any level. Emitting as boundary.\n";
			emitQuad(gv[0], gv[1], gv[2], gv[3], l.cellId, false);
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
