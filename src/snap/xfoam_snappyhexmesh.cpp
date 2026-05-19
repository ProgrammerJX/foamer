#include "XFoam/snap/xfoam_snappyhexmesh.h"

#include "XFoam/block/xfoam_blockmesh.h"
#include "XFoam/snap/xfoam_hex8ref.h"
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

// face dedup 用的有序顶点元组键。多边形最多 8 顶点（hex 6 边 + 边切了产生的中点；任何
// 单面最多 4 条边 + 4 个边中点 = 8）。
struct FaceKey
{
	std::vector<int> v; // 升序
	bool operator==(const FaceKey& o) const noexcept { return v == o.v; }
};

struct FaceKeyHash
{
	size_t operator()(const FaceKey& k) const noexcept
	{
		uint64_t h = 0;
		for (int x : k.v)
		{
			h = h * 0x9E3779B97F4A7C15ull + static_cast<uint32_t>(x);
		}
		h ^= (h >> 33);
		return static_cast<size_t>(h);
	}
};

inline FaceKey makeFaceKey(const std::vector<int>& verts)
{
	FaceKey k;
	k.v = verts;
	std::sort(k.v.begin(), k.v.end());
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
	readRefinementSurfaces(dict);
	readGeometry(dict);
}

XFoam_Label XFoam_SnappyHexMesh::globalRefinementLevel() const noexcept
{
	XFoam_Label g = 0;
	for (size_t i = 0; i < surfaces_.size(); ++i)
	{
		if (surfaces_[i].maxLevel > g) g = surfaces_[i].maxLevel;
	}
	return g;
}

namespace
{
static const XFoam_Word emptyWordRef_;
static const XFoam_FileName emptyFileNameRef_;
} // namespace

const XFoam_Word& XFoam_SnappyHexMesh::firstSurfaceName() const noexcept
{
	return surfaces_.empty() ? emptyWordRef_ : surfaces_[0].name;
}

const XFoam_FileName& XFoam_SnappyHexMesh::firstSurfaceFile() const noexcept
{
	return surfaces_.empty() ? emptyFileNameRef_ : surfaces_[0].file;
}

void XFoam_SnappyHexMesh::readPhaseFlags(const XFoam_Dictionary& dict)
{
	(void)dict.readIfPresent(XFoam_Word("castellatedMesh"), phases_.castellatedMesh);
	(void)dict.readIfPresent(XFoam_Word("snap"), phases_.snap);
	(void)dict.readIfPresent(XFoam_Word("addLayers"), phases_.addLayers);
}

void XFoam_SnappyHexMesh::readRefinementSurfaces(const XFoam_Dictionary& dict)
{
	// castellatedMeshControls.refinementSurfaces.<surfaceName> { level (min max); ... }
	// 每个 entry 入一个 SurfaceSpec；level 缺失回退 (0 0)。geometry 文件名留待 readGeometry 填。
	surfaces_.clear();
	DictView cv = asDict(dict, XFoam_Word("castellatedMeshControls"));
	if (!cv.dict) return;
	DictView sv = asDict(*cv.dict, XFoam_Word("refinementSurfaces"));
	if (!sv.dict) return;
	const XFoam_WordList keys = sv.dict->toc();
	for (XFoam_Label i = 0; i < keys.size(); ++i)
	{
		SurfaceSpec spec;
		spec.name = keys[i];
		DictView pv = asDict(*sv.dict, spec.name);
		const XFoam_Dictionary* sd = pv.dict;
		if (sd)
		{
			const XFoam_Entry* le = sd->lookupEntryPtr(XFoam_Word("level"), false, false);
			if (le)
			{
				XFoam_ITstream& is = le->stream();
				is.rewind();
				XFoam_Token t;
				is >> t;
				if (t.isPunctuation() && t.pToken() == XFoam_Token::BEGIN_LIST)
				{
					int32_t mn = 0, mx = 0;
					is >> mn >> mx;
					spec.minLevel = static_cast<XFoam_Label>(mn);
					spec.maxLevel = static_cast<XFoam_Label>(mx);
				}
			}
		}
		surfaces_.push_back(spec);
	}
}

void XFoam_SnappyHexMesh::readGeometry(const XFoam_Dictionary& dict)
{
	// geometry { <fileKey> { type triSurfaceMesh; name <surfaceName>; } }
	// 也兼容旧式：geometry { <fileKey> { type triSurfaceMesh; file "<...>"; } }
	// 通过 name 子项与 refinementSurfaces 的 key 对齐 — 若 entry 无 name 子项，回退
	// 把 fileKey 自身（去掉扩展名）当作 surface name。
	DictView gv = asDict(dict, XFoam_Word("geometry"));
	const XFoam_Dictionary* geo = gv.dict;
	if (!geo) return;
	const XFoam_WordList keys = geo->toc();
	for (XFoam_Label gi = 0; gi < keys.size(); ++gi)
	{
		const XFoam_Word fileKey = keys[gi];
		DictView pv = asDict(*geo, fileKey);
		const XFoam_Dictionary* gd = pv.dict;
		XFoam_Word fileName(static_cast<const XFoam_String&>(fileKey));
		XFoam_Word surfaceName;
		if (gd)
		{
			(void)gd->readIfPresent(XFoam_Word("name"), surfaceName);
			XFoam_FileName tmpFile;
			if (gd->readIfPresent(XFoam_Word("file"), tmpFile) && !tmpFile.empty())
			{
				fileName = XFoam_Word(static_cast<const XFoam_String&>(tmpFile));
			}
		}
		// 找匹配的 SurfaceSpec —— 优先用 entry 的 name 子项；若缺，则用 fileKey 去尾缀对齐。
		XFoam_Word matchName = surfaceName.empty() ? fileKey : surfaceName;
		for (size_t si = 0; si < surfaces_.size(); ++si)
		{
			if (surfaces_[si].name == matchName)
			{
				surfaces_[si].file = XFoam_FileName(static_cast<const XFoam_String&>(fileName));
				break;
			}
		}
	}
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
	std::vector<const XFoam_TriSurface*> stls(1, &stl);
	return run(bg, stls, outPolyMeshDir, stats);
}

bool XFoam_SnappyHexMesh::run(
	const XFoam_BlockMesh& bg,
	const std::vector<const XFoam_TriSurface*>& stls,
	const XFoam_FileName& outPolyMeshDir,
	Stats& stats) const
{
	const XFoam_Label nSurf = static_cast<XFoam_Label>(stls.size());
	const XFoam_Label nSpec = static_cast<XFoam_Label>(surfaces_.size());

	stats = Stats();
	stats.refinementLevel = globalRefinementLevel();
	stats.stlPatchName = (nSpec > 0)
		? surfaces_[0].name
		: XFoam_Word("snappy");

	// ----- 0. 前置校验 -----
	if (bg.size() == 0)
	{
		std::cerr << "snappy: background BlockMesh has no blocks.\n";
		return false;
	}
	for (XFoam_Label bi = 0; bi < bg.size(); ++bi)
	{
		if (!bg.set(bi))
		{
			std::cerr << "snappy: background BlockMesh block " << bi << " not constructible.\n";
			return false;
		}
	}
	if (nSpec == 0)
	{
		std::cerr << "snappy: no refinementSurfaces in dict.\n";
		return false;
	}
	if (nSurf != nSpec)
	{
		std::cerr << "snappy: ERROR: stls.size()=" << nSurf
		          << " != surfaces().size()=" << nSpec
		          << " (调用方需按 surfaces() 顺序传入).\n";
		return false;
	}
	bool anyNonEmpty = false;
	for (XFoam_Label si = 0; si < nSurf; ++si)
	{
		if (stls[si] && !stls[si]->empty()) { anyNonEmpty = true; break; }
	}
	if (!anyNonEmpty)
	{
		std::cerr << "snappy: all STLs are empty or null.\n";
		return false;
	}
	if (!refine_.hasLocationInMesh)
	{
		std::cerr << "snappy: castellatedMeshControls.locationInMesh required.\n";
		return false;
	}

	using Leaf = XFoam_Hex8Ref::Leaf;

	// ----- 1. 收集所有 block 几何 + 全局 bbox -----
	struct BlockData
	{
		XFoam_Vector3D corner[8];
		int Nx = 0, Ny = 0, Nz = 0;
		XFoam_Label cellIdOffset = 0;
	};
	const XFoam_Label nBlocks = bg.size();
	std::vector<BlockData> blocks(static_cast<size_t>(nBlocks));
	XFoam_Vector3D bbMin(0, 0, 0), bbMax(0, 0, 0);
	bool bbInit = false;
	XFoam_Label totalBgCells = 0;
	for (XFoam_Label bi = 0; bi < nBlocks; ++bi)
	{
		const XFoam_BlockDescriptor& bd = bg[bi];
		const XFoam_CellShape& shp = bd.blockShape();
		const XFoam_UList<XFoam_Vector3D>& bv = bd.vertices();
		BlockData& B = blocks[static_cast<size_t>(bi)];
		for (int i = 0; i < 8; ++i)
		{
			B.corner[i] = bv[shp[i]] * bg.scaleFactor();
			if (!bbInit)
			{
				bbMin = bbMax = B.corner[i];
				bbInit = true;
			}
			else
			{
				bbMin.x() = std::min(bbMin.x(), B.corner[i].x());
				bbMin.y() = std::min(bbMin.y(), B.corner[i].y());
				bbMin.z() = std::min(bbMin.z(), B.corner[i].z());
				bbMax.x() = std::max(bbMax.x(), B.corner[i].x());
				bbMax.y() = std::max(bbMax.y(), B.corner[i].y());
				bbMax.z() = std::max(bbMax.z(), B.corner[i].z());
			}
		}
		B.Nx = static_cast<int>(bd.density().x());
		B.Ny = static_cast<int>(bd.density().y());
		B.Nz = static_cast<int>(bd.density().z());
		totalBgCells += static_cast<XFoam_Label>(B.Nx) * B.Ny * B.Nz;
	}
	stats.nBgCells = totalBgCells;

	// 点 dedup 容差：以全局 bbox 对角为尺度，1e-9 相对量级。
	const XFoam_Scalar bbDiag = (bbMax - bbMin).mag();
	const XFoam_Scalar eps = std::max<XFoam_Scalar>(1e-12, bbDiag * static_cast<XFoam_Scalar>(1e-9));
	const double invEps = 1.0 / static_cast<double>(eps);
	// LEVEL_CAP 受 XFoam_Hex8Ref::kMaxEncodedLevel 约束（leaf-key 编码 si/sj/sk 各 10 bit → cap ≤ 10）。
	// 默认 8 已足以让 base-cell=0.25 的 needle/cylinder 测例（cell ~0.001）很平滑；上 9/10 再调更高分辨率。
	const int LEVEL_CAP = std::min<int>(8, XFoam_Hex8Ref::kMaxEncodedLevel);
	const int targetLevel = std::min<int>(static_cast<int>(globalRefinementLevel()), LEVEL_CAP);

	// 通用 surface maxLevel 取上限。
	std::vector<XFoam_Label> surfMaxLevel(nSurf, 0);
	for (XFoam_Label si = 0; si < nSurf; ++si)
	{
		surfMaxLevel[si] = std::min<XFoam_Label>(surfaces_[si].maxLevel,
		                                         static_cast<XFoam_Label>(LEVEL_CAP));
	}

	// ----- 2. 每个 block 独立建 Hex8Ref + 各 phase -----
	// 跨 block 共享面在最终 emit 阶段靠 pts dedup + face dedup 自动合成 internal。
	std::vector<XFoam_Hex8Ref> octs;
	octs.reserve(static_cast<size_t>(nBlocks));
	for (XFoam_Label bi = 0; bi < nBlocks; ++bi)
	{
		octs.emplace_back(blocks[static_cast<size_t>(bi)].Nx,
		                  blocks[static_cast<size_t>(bi)].Ny,
		                  blocks[static_cast<size_t>(bi)].Nz,
		                  LEVEL_CAP);
		octs.back().initBaseLeaves();
	}

	// 通用辅助：根据 leaf 所属的 block 计算 world 坐标。
	auto leafCornersWorldB = [&](const BlockData& B, const Leaf& l, XFoam_Vector3D out[8]) {
		for (int oc = 0; oc < 8; ++oc)
		{
			XFoam_Scalar u, v, w;
			XFoam_Hex8Ref::leafCornerParam(l, oc, u, v, w);
			out[oc] = paramToWorld(B.corner, B.Nx, B.Ny, B.Nz, l.ai, l.aj, l.ak, u, v, w);
		}
	};
	auto leafBBoxB = [&](const BlockData& B, const Leaf& l) -> XFoam_BoundBox {
		XFoam_Vector3D c[8]; leafCornersWorldB(B, l, c);
		XFoam_Vector3D mn = c[0], mx = c[0];
		for (int i = 1; i < 8; ++i)
		{
			mn.x() = std::min(mn.x(), c[i].x()); mx.x() = std::max(mx.x(), c[i].x());
			mn.y() = std::min(mn.y(), c[i].y()); mx.y() = std::max(mx.y(), c[i].y());
			mn.z() = std::min(mn.z(), c[i].z()); mx.z() = std::max(mx.z(), c[i].z());
		}
		return XFoam_BoundBox(mn, mx);
	};
	auto leafCentroidWorldB = [&](const BlockData& B, const Leaf& l) -> XFoam_Vector3D {
		XFoam_Scalar u, v, w;
		XFoam_Hex8Ref::leafCentroidParam(l, u, v, w);
		return paramToWorld(B.corner, B.Nx, B.Ny, B.Nz, l.ai, l.aj, l.ak, u, v, w);
	};

	auto insideAny = [&](const XFoam_Vector3D& p) -> bool {
		for (XFoam_Label si = 0; si < nSurf; ++si)
		{
			if (stls[si] && !stls[si]->empty() && stls[si]->contains(p)) return true;
		}
		return false;
	};
	const bool locInside = insideAny(refine_.locationInMesh);

	const int nBufferLayers = std::max(0, static_cast<int>(refine_.nCellsBetweenLevels) - 1);

	// ----- 2.x 每 block 走完 refine → buffer → balance → cull → assignCellIds -----
	stats.nRefinedCells = 0;
	stats.nKeptCells = 0;
	stats.maxAdaptiveLevel = 0;
	for (int L = 0; L < XFoam_Hex8Ref::kMaxLevelBuckets; ++L) stats.perLevelCells[L] = 0;
	for (XFoam_Label bi = 0; bi < nBlocks; ++bi)
	{
		BlockData& B = blocks[static_cast<size_t>(bi)];
		XFoam_Hex8Ref& oct = octs[static_cast<size_t>(bi)];

		oct.refineByPredicate(
			static_cast<XFoam_Label>(targetLevel),
			[&](const Leaf& l) -> bool {
				const XFoam_BoundBox bb = leafBBoxB(B, l);
				for (XFoam_Label si = 0; si < nSurf; ++si)
				{
					if (!stls[si] || stls[si]->empty()) continue;
					if (l.level >= surfMaxLevel[si]) continue;
					if (stls[si]->boxIntersects(bb)) return true;
				}
				return false;
			});
		if (nBufferLayers > 0) oct.extendHighLevel(nBufferLayers);
		oct.balance21();

		XFoam_Label pl[XFoam_Hex8Ref::kMaxLevelBuckets] = {0};
		oct.perLevelCounts(pl);
		for (int L = 0; L < XFoam_Hex8Ref::kMaxLevelBuckets; ++L) stats.perLevelCells[L] += pl[L];
		stats.maxAdaptiveLevel = std::max(stats.maxAdaptiveLevel, oct.maxLevelReached());
		stats.nRefinedCells += oct.numLeaves();

		oct.cullByPredicate(
			[&](const Leaf& l) -> bool {
				return insideAny(leafCentroidWorldB(B, l)) != locInside;
			});

		const XFoam_Label kept = oct.assignCellIds();
		B.cellIdOffset = stats.nKeptCells;
		stats.nKeptCells += kept;
		// 把 cellId 平移到全局编号
		std::vector<Leaf>& lvs = oct.leaves();
		for (size_t i = 0; i < lvs.size(); ++i)
		{
			if (lvs[i].cellId >= 0) lvs[i].cellId += B.cellIdOffset;
		}
	}

	// ----- 8. 全局点表 + addPoint （跨 block 共用，自动 dedup 共享面顶点） -----
	const XFoam_Label totalLeaves = [&]() {
		XFoam_Label t = 0;
		for (XFoam_Label bi = 0; bi < nBlocks; ++bi) t += octs[static_cast<size_t>(bi)].numLeaves();
		return t;
	}();
	std::vector<XFoam_Vector3D> pts;
	pts.reserve(static_cast<size_t>(totalLeaves) * 8);
	std::unordered_map<PointKey, int, PointKeyHash> ptIdx;
	ptIdx.reserve(static_cast<size_t>(totalLeaves) * 8);
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

	// ----- 9. 给所有 block 的所有 leaf 生成 8 个 corner 点 -----
	// 跨 block 的共享面顶点位置一致 → addPoint 自然 dedup。
	for (XFoam_Label bi = 0; bi < nBlocks; ++bi)
	{
		BlockData& B = blocks[static_cast<size_t>(bi)];
		std::vector<Leaf>& lvs = octs[static_cast<size_t>(bi)].leaves();
		for (Leaf& l : lvs)
		{
			XFoam_Vector3D c[8]; leafCornersWorldB(B, l, c);
			for (int oc = 0; oc < 8; ++oc) l.corner[oc] = addPoint(c[oc]);
		}
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
	finalFaces.reserve(static_cast<size_t>(totalLeaves) * 6);
	faceIdx.reserve(static_cast<size_t>(totalLeaves) * 6);

	// emit 任意 N 边形（N >= 3）；用排序顶点集做 dedup。第二次遇到时按 OF 约定
	// 设小 cellId 为 owner，并反转 verts 让法向从 owner → neighbour。
	auto emitFace = [&](const std::vector<int>& verts, int cellId, bool onGridBoundary) {
		const FaceKey key = makeFaceKey(verts);
		auto it = faceIdx.find(key);
		if (it == faceIdx.end())
		{
			FInfo f;
			f.verts = verts;
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
	// 几何位置由 Hex8Ref 给出 (u, v, w)，本侧再 paramToWorld + addPoint 走 dedup。
	auto faceFinePointB = [&](const BlockData& B, const Leaf& l, int d, int rr, int cc) -> int {
		XFoam_Scalar u, v, w;
		XFoam_Hex8Ref::faceSteinerParam(l, d, rr, cc, u, v, w);
		const XFoam_Vector3D p = paramToWorld(B.corner, B.Nx, B.Ny, B.Nz, l.ai, l.aj, l.ak, u, v, w);
		return addPoint(p);
	};

	// 查全局点表里是否已有 pts[vA] 与 pts[vB] 的中点（用同 invEps 容差量化哈希）。
	// 若有 → 返回那个点 id；否则 -1。
	// 这是 OpenFOAM hexRef8 "在 face 边上插入更高 pointLevel 的中点" 的等价实现：
	// 只要别处某个 split face 把这条边对应的中点 addPoint 过，本面 emit 时就插入它。
	auto midPointId = [&](int vA, int vB) -> int {
		const XFoam_Vector3D mid = (pts[vA] + pts[vB]) * 0.5;
		PointKey k{
			static_cast<int64_t>(std::llround(mid.x() * invEps)),
			static_cast<int64_t>(std::llround(mid.y() * invEps)),
			static_cast<int64_t>(std::llround(mid.z() * invEps))
		};
		auto it = ptIdx.find(k);
		return (it != ptIdx.end()) ? it->second : -1;
	};

	// ----- 10a. 预 pass：把所有 split face 的 9 个 Steiner 点先 addPoint 进 ptIdx -----
	// 这样后面任何 cell emit 自己的"非 split 面"时，midPointId 都能查到边中点。
	// 不在这里 emit 任何 face；只 populate 点表。
	for (XFoam_Label bi = 0; bi < nBlocks; ++bi)
	{
		const BlockData& B = blocks[static_cast<size_t>(bi)];
		const XFoam_Hex8Ref& oct = octs[static_cast<size_t>(bi)];
		const std::vector<Leaf>& lvs = oct.leaves();
		for (size_t s = 0; s < lvs.size(); ++s)
		{
			const Leaf& l = lvs[s];
			if (!l.kept) continue;
			for (int d = 0; d < 6; ++d)
			{
				const XFoam_Hex8Ref::FaceNbr nbr = oct.resolveFaceNeighbor(l, d);
				if (nbr.kind != XFoam_Hex8Ref::FaceNbrKind::Finer) continue;
				for (int rr = 0; rr < 3; ++rr)
				{
					for (int cc = 0; cc < 3; ++cc)
					{
						(void)faceFinePointB(B, l, d, rr, cc);
					}
				}
			}
		}
	}

	// 切面 sub-quad 自然 winding (rr,cc)→(rr,cc+1)→(rr+1,cc+1)→(rr+1,cc) 等价
	//   +axA × +axB；对 d=0/3/4 与 outward 法向相反，需翻转，否则负体积。
	static const bool kFlipSub[6] = {true, false, false, true, true, false};

	// ----- 10b. 正式 emit （所有 block） -----
	// 跨 block 的共享面：A 的边界 leaf 在某 d 方向是 OutOfGrid（block 内意义），但 B 同位置
	// 的 leaf 会 emit 对应的同顶点集 face；emitFace dedup 自动把 fromGridBoundary 清掉，face
	// 进入 internal 集合。
	for (XFoam_Label bi = 0; bi < nBlocks; ++bi)
	{
		const BlockData& B = blocks[static_cast<size_t>(bi)];
		const XFoam_Hex8Ref& oct = octs[static_cast<size_t>(bi)];
		const std::vector<Leaf>& lvs = oct.leaves();
		for (size_t s = 0; s < lvs.size(); ++s)
		{
			const Leaf& l = lvs[s];
			if (!l.kept) continue;
			bool isPoly = false;

			for (int d = 0; d < 6; ++d)
			{
				const XFoam_Hex8Ref::FaceNbr nbr = oct.resolveFaceNeighbor(l, d);

				if (nbr.kind == XFoam_Hex8Ref::FaceNbrKind::Finer)
				{
					int pgrid[3][3];
					for (int rr = 0; rr < 3; ++rr)
					{
						for (int cc = 0; cc < 3; ++cc)
						{
							pgrid[rr][cc] = faceFinePointB(B, l, d, rr, cc);
						}
					}
					const bool flipSub = kFlipSub[d];
					for (int rr = 0; rr < 2; ++rr)
					{
						for (int cc = 0; cc < 2; ++cc)
						{
							int q[4];
							if (flipSub)
							{
								q[0] = pgrid[rr][cc];
								q[1] = pgrid[rr + 1][cc];
								q[2] = pgrid[rr + 1][cc + 1];
								q[3] = pgrid[rr][cc + 1];
							}
							else
							{
								q[0] = pgrid[rr][cc];
								q[1] = pgrid[rr][cc + 1];
								q[2] = pgrid[rr + 1][cc + 1];
								q[3] = pgrid[rr + 1][cc];
							}
							std::vector<int> sq;
							sq.reserve(8);
							for (int i = 0; i < 4; ++i)
							{
								const int vA = q[i];
								const int vB = q[(i + 1) % 4];
								sq.push_back(vA);
								const int mid = midPointId(vA, vB);
								if (mid >= 0 && mid != vA && mid != vB)
								{
									sq.push_back(mid);
								}
							}
							emitFace(sq, l.cellId, false);
							++stats.nSplitFaces;
						}
					}
					isPoly = true;
					continue;
				}

				const int* fv = kHexFace[d];
				std::vector<int> verts;
				verts.reserve(8);
				for (int i = 0; i < 4; ++i)
				{
					const int vA = l.corner[fv[i]];
					const int vB = l.corner[fv[(i + 1) % 4]];
					verts.push_back(vA);
					const int mid = midPointId(vA, vB);
					if (mid >= 0 && mid != vA && mid != vB)
					{
						verts.push_back(mid);
						isPoly = true;
					}
				}

				const bool onGridBoundary = (nbr.kind == XFoam_Hex8Ref::FaceNbrKind::OutOfGrid);
				if (nbr.kind == XFoam_Hex8Ref::FaceNbrKind::None)
				{
					std::cerr << "snappy: WARNING: leaf (" << l.ai << "," << l.aj << "," << l.ak
					          << ", L=" << l.level << ") face d=" << d
					          << " has no neighbor leaf in any level. Emitting as boundary.\n";
				}
				emitFace(verts, l.cellId, onGridBoundary);
			}
			if (isPoly) ++stats.nPolyhedralCells;
		}
	}

	// ----- 7. 把 faces 切成 internal / boundary, 给 boundary 分 patch -----
	// boundary patch: faces 都是单 cell 引用 (neighbour == -1)
	//   - fromGridBoundary == true → 背景 walls 顶面 → 原 BlockMesh.patchNames()[0]
	//   - 否则 (dedup 只见 1 次但邻居 base-cell 存在，意味着邻居 sub-cell 被 STL 切掉)
	//     → 归属到该 face centroid 最近的 STL 对应的 patch（多 surface 时）。
	std::vector<int> internalIdx; internalIdx.reserve(finalFaces.size());
	std::vector<int> wallsIdx;
	// stlIdxBySurf[si] 收集第 si 个 surface 上的 boundary face 序号
	std::vector<std::vector<int>> stlIdxBySurf(static_cast<size_t>(nSurf));
	auto faceCentroid = [&](const FInfo& f) -> XFoam_Vector3D {
		XFoam_Vector3D c(0, 0, 0);
		for (int v : f.verts) c = c + pts[v];
		const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / static_cast<XFoam_Scalar>(f.verts.size());
		return c * inv;
	};
	for (int fi = 0; fi < static_cast<int>(finalFaces.size()); ++fi)
	{
		const FInfo& f = finalFaces[fi];
		if (f.neighbour != -1) { internalIdx.push_back(fi); continue; }
		if (f.fromGridBoundary) { wallsIdx.push_back(fi); continue; }
		// 找最近 STL：以 face centroid 到各 STL closestPoint 的距离作判据
		const XFoam_Vector3D fc = faceCentroid(f);
		XFoam_Label bestSi = -1;
		XFoam_Scalar bestD2 = std::numeric_limits<XFoam_Scalar>::max();
		for (XFoam_Label si = 0; si < nSurf; ++si)
		{
			if (!stls[si] || stls[si]->empty()) continue;
			XFoam_Vector3D cp, nm;
			stls[si]->closestPointAndNormal(fc, cp, nm);
			const XFoam_Vector3D d = cp - fc;
			const XFoam_Scalar d2 = d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
			if (d2 < bestD2) { bestD2 = d2; bestSi = si; }
		}
		if (bestSi < 0) bestSi = 0; // 全空时落第 0 个 (前置已校验非全空)
		stlIdxBySurf[static_cast<size_t>(bestSi)].push_back(fi);
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
	for (auto& v : stlIdxBySurf) std::sort(v.begin(), v.end(), byOwner);

	// ----- 8. snap：每个 surface 上的 boundary 点投到该 surface STL 最近点 -----
	// 同时按 Snap #5 motionSmoother 把 patch 点的位移向内传播 nSmoothInternal 圈，
	// 让最贴 STL 的内层 hex cell 不会被孤零零地拉成尖角 polyhedron。
	if (phases_.snap)
	{
		std::vector<unsigned char> snapped(pts.size(), 0);
		// 保存 snap 前 pts，便于 motionSmoother 计算位移 = pts_snapped - pts_orig。
		// 仅在需要 internal smoothing 时才占用这份拷贝。
		const XFoam_Label nInternal = snap_.nSmoothInternal;
		std::vector<XFoam_Vector3D> origPts;
		if (nInternal > 0) origPts = pts;

		for (XFoam_Label si = 0; si < nSurf; ++si)
		{
			if (!stls[si] || stls[si]->empty()) continue;
			for (int fi : stlIdxBySurf[static_cast<size_t>(si)])
			{
				const FInfo& f = finalFaces[fi];
				for (int v : f.verts)
				{
					if (snapped[v]) continue;
					snapped[v] = 1;
					XFoam_Vector3D closest, normal;
					stls[si]->closestPointAndNormal(pts[v], closest, normal);
					const XFoam_Scalar d = (closest - pts[v]).mag();
					if (d > stats.maxSnapDistance) stats.maxSnapDistance = d;
					pts[v] = closest;
					++stats.nSnappedPoints;
				}
			}
		}

		// Snap #5 motionSmoother：把 snapped 点的位移按 Jacobi/Laplacian 向内传播 N 圈。
		// 算法（对标 OpenFOAM-13 Foam::motionSmoother）：
		//   1) 由 face edges 构建 point→point 邻接（去重；任意 N 边形按 i-(i+1)% n 邻接）。
		//   2) BFS 从 snapped 点出发，给所有 ≤ N 圈内的点打 ringId；ringId > N 视为远点。
		//   3) 初始化 disp：snapped 点为 (pts_snapped - pts_orig)；其余为 0。
		//   4) Jacobi 迭代 N 轮：每个 non-snapped 且 ringId ≤ N 的点位移 = 邻居位移均值。
		//      只统计 ringId ≤ N 的邻居，避免远点的零位移把传播稀释。
		//   5) 把 disp 加回 origPts：pts = origPts + disp。snapped 点的 disp 保持，pts 不变。
		if (nInternal > 0 && stats.nSnappedPoints > 0)
		{
			const size_t nPts = pts.size();

			// (1) 邻接表。先 push edge 双向，再排序+unique 去重；用 vector 比 set 省 3-4x 内存。
			std::vector<std::vector<int>> ptNbrs(nPts);
			for (const FInfo& f : finalFaces)
			{
				const int n = static_cast<int>(f.verts.size());
				for (int i = 0; i < n; ++i)
				{
					const int v1 = f.verts[i];
					const int v2 = f.verts[(i + 1) % n];
					ptNbrs[static_cast<size_t>(v1)].push_back(v2);
					ptNbrs[static_cast<size_t>(v2)].push_back(v1);
				}
			}
			for (auto& vec : ptNbrs)
			{
				std::sort(vec.begin(), vec.end());
				vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
			}

			// (2) BFS 给 ringId 打标；超出 N 圈的保持 INT_MAX。
			const int N = static_cast<int>(nInternal);
			std::vector<int> ringId(nPts, std::numeric_limits<int>::max());
			{
				std::vector<int> frontier;
				frontier.reserve(stats.nSnappedPoints);
				for (size_t v = 0; v < nPts; ++v)
				{
					if (snapped[v]) { ringId[v] = 0; frontier.push_back(static_cast<int>(v)); }
				}
				for (int r = 0; r < N && !frontier.empty(); ++r)
				{
					std::vector<int> next;
					next.reserve(frontier.size() * 2);
					for (int v : frontier)
					{
						for (int u : ptNbrs[static_cast<size_t>(v)])
						{
							if (ringId[u] > r + 1)
							{
								ringId[u] = r + 1;
								next.push_back(u);
							}
						}
					}
					frontier.swap(next);
				}
			}

			// (3) 初始化 disp。
			std::vector<XFoam_Vector3D> disp(nPts, XFoam_Vector3D(0, 0, 0));
			for (size_t v = 0; v < nPts; ++v)
			{
				if (snapped[v]) disp[v] = pts[v] - origPts[v];
			}

			// (4) Jacobi 迭代 N 轮（Gauss-Seidel 也可，但用 swap 双 buffer 更并行友好）。
			std::vector<XFoam_Vector3D> dispNew(disp);
			for (int iter = 0; iter < N; ++iter)
			{
				for (size_t v = 0; v < nPts; ++v)
				{
					if (snapped[v]) { dispNew[v] = disp[v]; continue; }
					if (ringId[v] > N) { dispNew[v] = disp[v]; continue; }

					XFoam_Vector3D sum(0, 0, 0);
					int cnt = 0;
					for (int u : ptNbrs[v])
					{
						if (ringId[u] > N) continue; // 远点（disp ≡ 0）不参与平均，否则会拖弱传播
						sum.x() += disp[u].x();
						sum.y() += disp[u].y();
						sum.z() += disp[u].z();
						++cnt;
					}
					if (cnt > 0)
					{
						const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / static_cast<XFoam_Scalar>(cnt);
						dispNew[v] = sum * inv;
					}
				}
				std::swap(disp, dispNew);
			}

			// (5) 应用：snapped 点位移已应用在 pts 上（snap 时已 pts[v]=closest），跳过；
			//   其它 ringId ≤ N 的点用 disp 覆盖 pts；远点 / 非传播点保持原状。
			for (size_t v = 0; v < nPts; ++v)
			{
				if (snapped[v]) continue;
				if (ringId[v] > N) continue;
				const XFoam_Scalar mv = disp[v].mag();
				if (mv > 0)
				{
					++stats.nSmoothedInternalPoints;
					if (mv > stats.maxInternalSmoothMove) stats.maxInternalSmoothMove = mv;
				}
				pts[v] = origPts[v] + disp[v];
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
	for (XFoam_Label si = 0; si < nSurf; ++si)
	{
		const std::string pname = std::string(
			static_cast<const std::string&>(static_cast<const XFoam_String&>(surfaces_[si].name)));
		addPatch(stlIdxBySurf[static_cast<size_t>(si)], pname, std::string("wall"));
	}

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
