#include "XFoam/snap/xfoam_snappyhexmesh.h"
#include "XFoam/snap/xfoam_pointconstraint.h"

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
	const XFoam_BrepBase&  surf,
	const XFoam_FileName&  outPolyMeshDir,
	Stats&                 stats) const
{
	std::vector<const XFoam_BrepBase*> surfs(1, &surf);
	return run(bg, surfs, outPolyMeshDir, stats);
}

bool XFoam_SnappyHexMesh::run(
	const XFoam_BlockMesh&                       bg,
	const std::vector<const XFoam_BrepBase*>&    stls,
	const XFoam_FileName&                        outPolyMeshDir,
	Stats&                                       stats) const
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
	if (!refine_.hasLocationInMesh())
	{
		std::cerr << "snappy: castellatedMeshControls.locationsInMesh (or legacy locationInMesh) required.\n";
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
	// 多 region 选择 (locationsInMesh)：每个 location 编一个 per-surface inside/outside
	// bitmask（第 si 位 = stls[si]->contains(p)）。cell 中心如果与任一 location 同 bitmask
	// 就保留 —— 这把 "outside all surfaces / inside surface-A only / inside surface-B only" 等
	// 互不相交的 region 切开，单 location + 单 surface 退化成旧 insideAny 比较。
	// 64 位封顶（refinementSurfaces 上限自然落在这个数量级以内）。
	using RegionMask = uint64_t;
	if (nSurf > 64)
	{
		std::cerr << "snappy: refinementSurfaces.size()=" << nSurf
		          << " > 64 不被 region-mask 支持。\n";
		return false;
	}
	auto regionMask = [&](const XFoam_Vector3D& p) -> RegionMask {
		RegionMask m = 0;
		for (XFoam_Label si = 0; si < nSurf; ++si)
		{
			if (stls[si] && !stls[si]->empty() && stls[si]->contains(p))
			{
				m |= (RegionMask(1) << si);
			}
		}
		return m;
	};
	std::vector<RegionMask> locMasks;
	locMasks.reserve(refine_.locationsInMesh.size());
	for (size_t li = 0; li < refine_.locationsInMesh.size(); ++li)
	{
		locMasks.push_back(regionMask(refine_.locationsInMesh[li]));
	}
	// 兼容旧 stats / sample 路径下"insideAny(loc)"概念：取首 location 的旧布尔。
	(void)insideAny;  // 仅供调试/未来 patch 名生成；当前驱动改走 bitmask。

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
				const RegionMask cm = regionMask(leafCentroidWorldB(B, l));
				for (size_t li = 0; li < locMasks.size(); ++li)
				{
					if (locMasks[li] == cm) return false;
				}
				return true;
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
	// 同时为 Snap #7 维护 ptCellSize_：每个 point 所属 leaf 的最小轴长（多个 leaf 共享时取
	// 最细那个），后续 feature snap 用 0.5 * ptCellSize 作为搜索半径。
	std::vector<XFoam_Scalar> ptCellSize;
	for (XFoam_Label bi = 0; bi < nBlocks; ++bi)
	{
		BlockData& B = blocks[static_cast<size_t>(bi)];
		std::vector<Leaf>& lvs = octs[static_cast<size_t>(bi)].leaves();
		for (Leaf& l : lvs)
		{
			XFoam_Vector3D c[8]; leafCornersWorldB(B, l, c);
			// 最小轴长：(c1-c0).x, (c3-c0).y, (c4-c0).z 的最小绝对值
			const XFoam_Scalar dx = std::abs(c[1].x() - c[0].x());
			const XFoam_Scalar dy = std::abs(c[3].y() - c[0].y());
			const XFoam_Scalar dz = std::abs(c[4].z() - c[0].z());
			const XFoam_Scalar leafMin = std::min(dx, std::min(dy, dz));
			for (int oc = 0; oc < 8; ++oc)
			{
				const int pid = addPoint(c[oc]);
				l.corner[oc] = pid;
				if (pid >= static_cast<int>(ptCellSize.size()))
				{
					ptCellSize.resize(static_cast<size_t>(pid) + 1, std::numeric_limits<XFoam_Scalar>::max());
				}
				if (leafMin < ptCellSize[static_cast<size_t>(pid)])
				{
					ptCellSize[static_cast<size_t>(pid)] = leafMin;
				}
			}
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

	// ----- 8. snap：四步 -----
	//   8a) STL 投影 snapped 边界点
	//   8b) Snap #5 motionSmoother：patch 位移向内传播 nSmoothInternal 圈
	//   8c) Snap #6 validate-and-relax：找到负体积/退化 cell，对其顶点做 50% 回退，
	//        最多 nRelaxIter 轮
	//   8d) 统计 max snap 距离更新（如果 relax 改了位置）
	if (phases_.snap)
	{
		std::vector<unsigned char> snapped(pts.size(), 0);
		// Snap #9 pointConstraint：每个 boundary 点记录剩余 DOF。free=interior，
		// plane=普通 surface snap，line=feature edge snap，fixed=feature vertex snap。
		// 多 surface 同一点会被多次 combine（取约束更严的那个），与 OF 的 pointConstraint
		// 累计行为一致。
		std::vector<XFoam_PointConstraint> ptCons(pts.size());
		// 始终保存 snap 前 pts：motionSmoother 与 validate-and-relax 都要用它做位移基准 /
		// 回退到 grid 上。这份拷贝在 L=6 cylinder1 case 大约 2 MB（250k pts × 8B floats × 3）。
		const XFoam_Label nInternal = snap_.nSmoothInternal;
		const XFoam_Label nRelaxIt  = std::max<XFoam_Label>(0, snap_.nRelaxIter);
		std::vector<XFoam_Vector3D> origPts = pts;

		// Snap #7 feature snap：如果 dict 打开了 implicitFeatureSnap 且 STL 上抽过 feature，
		// 则每个 boundary pt 先做 surface snap 拿 qSurf，再在 snap_.tolerance * ptCellSize
		// 半径内查最近 feature（OF 习惯 tolerance=2.0 → 半径为 2 倍局部 cell size，足够覆盖
		// L+1 邻居）。两种合并规则（closestFeature 内部已用 radius 做了过滤）：
		//   * feature Vertex 在半径内 → snap 到 vertex（尖角第一优先）
		//   * feature Edge   在半径内 → snap 到 edge 投影点（cylinder cap 棱由此变锋利）
		//   * 都不在半径内 → 用 qSurf
		const bool doFeatureSnap = snap_.implicitFeatureSnap;
		const XFoam_Scalar featureRadiusFactor = std::max<XFoam_Scalar>(snap_.tolerance, 1);
		for (XFoam_Label si = 0; si < nSurf; ++si)
		{
			if (!stls[si] || stls[si]->empty()) continue;
			const bool hasFeatures = doFeatureSnap
				&& (stls[si]->nFeatureEdges() > 0 || stls[si]->nFeatureVertices() > 0);
			for (int fi : stlIdxBySurf[static_cast<size_t>(si)])
			{
				const FInfo& f = finalFaces[fi];
				for (int v : f.verts)
				{
					if (snapped[v]) continue;
					snapped[v] = 1;
					XFoam_Vector3D closest, normal;
					stls[si]->closestPointAndNormal(pts[v], closest, normal);

					// 默认 plane 约束（normal 已被 STL 归一化）。feature 命中会被升级。
					XFoam_PointConstraint pc = XFoam_PointConstraint::plane(normal);

					if (hasFeatures && v < static_cast<int>(ptCellSize.size()))
					{
						const XFoam_Scalar radius = featureRadiusFactor * ptCellSize[v];
						XFoam_Vector3D fq;
						XFoam_Vector3D ft;
						const auto kind = stls[si]->closestFeature(pts[v], radius, fq, ft);
						if (kind == XFoam_BrepBase::FeatureKind::Vertex)
						{
							closest = fq;
							++stats.nFeatureVertexSnaps;
							pc = XFoam_PointConstraint::fixed();
						}
						else if (kind == XFoam_BrepBase::FeatureKind::Edge)
						{
							closest = fq;
							++stats.nFeatureEdgeSnaps;
							// 用 closestFeature 回填的 unit edge 切向构造 line 约束。
							// 退化 (|t|==0) 时落回 plane 防御，避免 line(0) 失效。
							if (ft.mag() > 0)
							{
								pc = XFoam_PointConstraint::line(ft);
							}
							else
							{
								pc = XFoam_PointConstraint::plane(normal);
							}
						}
					}

					const XFoam_Scalar d = (closest - pts[v]).mag();
					if (d > stats.maxSnapDistance) stats.maxSnapDistance = d;
					pts[v] = closest;
					++stats.nSnappedPoints;

					ptCons[static_cast<size_t>(v)].combine(pc);
				}
			}
		}

		// 把所有 snapped 点的最终 ptCons 按 DOF 分桶计数，给到 stats（便于 sample/test
		// 校验：plane + line + fixed == 总 snapped 点数）。
		for (size_t v = 0; v < pts.size(); ++v)
		{
			if (!snapped[v]) continue;
			switch (ptCons[v].nConstraints())
			{
				case 1: ++stats.nPlaneConstrained; break;
				case 2: ++stats.nLineConstrained;  break;
				case 3: ++stats.nFixedConstrained; break;
				default: break;
			}
		}

		// ----- 8a/b 完成（投影只在上面）。下面是 motionSmoother + validate-and-relax 公用的
		// point→point 邻接表与 cell→faces 反向表。先建好，后两阶段都用。
		const size_t nPts = pts.size();
		const XFoam_Label nCells = stats.nKeptCells;

		// (邻接) 由 face edges 构建 point→point 邻接（去重；任意 N 边形按 i-(i+1)% n 邻接）。
		// 用 vector + sort/unique 比 set 省 3~4x 内存（L=6 250k pts × ~12 nbr 仍较紧凑）。
		std::vector<std::vector<int>> ptNbrs;
		auto buildPtNbrs = [&]() {
			if (!ptNbrs.empty()) return;
			ptNbrs.assign(nPts, {});
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
		};

		// (cell→faces) snap 阶段还没装配 facesOut；这里直接用 finalFaces.owner/.neighbour
		// 反向收集每个 cell 的 face id（容量上限 = #cells，但实际平均 6）。
		std::vector<std::vector<int>> cellFaces;
		auto buildCellFaces = [&]() {
			if (!cellFaces.empty()) return;
			cellFaces.assign(static_cast<size_t>(nCells), {});
			for (int fi = 0; fi < static_cast<int>(finalFaces.size()); ++fi)
			{
				const FInfo& f = finalFaces[fi];
				if (f.owner >= 0 && f.owner < nCells)
					cellFaces[static_cast<size_t>(f.owner)].push_back(fi);
				if (f.neighbour >= 0 && f.neighbour < nCells)
					cellFaces[static_cast<size_t>(f.neighbour)].push_back(fi);
			}
		};

		// (cell 体积) 用散度定理：V = (1/3) Σ_f sign · (centroid_f · area_vec_f)。
		// 面被 fan triangulation 切成 n-2 个三角形（从 v0 出发），各贡献：
		//   area_vec_tri = 0.5 · (v1 - v0) × (v2 - v0)
		//   centroid_tri = (v0 + v1 + v2) / 3
		// 面 area_vec = Σ triangles area_vec_tri，centroid = area-weighted average centroid_tri。
		auto cellVolume = [&](int cellId) -> XFoam_Scalar {
			XFoam_Scalar vol = 0;
			for (int fi : cellFaces[static_cast<size_t>(cellId)])
			{
				const FInfo& f = finalFaces[fi];
				const int n = static_cast<int>(f.verts.size());
				if (n < 3) continue;
				const XFoam_Vector3D v0 = pts[f.verts[0]];
				XFoam_Vector3D af(0, 0, 0);
				XFoam_Vector3D cf(0, 0, 0);
				XFoam_Scalar totalArea = 0;
				for (int i = 1; i + 1 < n; ++i)
				{
					const XFoam_Vector3D v1 = pts[f.verts[i]];
					const XFoam_Vector3D v2 = pts[f.verts[i + 1]];
					const XFoam_Vector3D e1 = v1 - v0;
					const XFoam_Vector3D e2 = v2 - v0;
					const XFoam_Vector3D triN(
						static_cast<XFoam_Scalar>(0.5) * (e1.y() * e2.z() - e1.z() * e2.y()),
						static_cast<XFoam_Scalar>(0.5) * (e1.z() * e2.x() - e1.x() * e2.z()),
						static_cast<XFoam_Scalar>(0.5) * (e1.x() * e2.y() - e1.y() * e2.x()));
					const XFoam_Scalar triA = triN.mag();
					const XFoam_Vector3D triC = (v0 + v1 + v2) * (static_cast<XFoam_Scalar>(1.0 / 3.0));
					af = af + triN;
					cf = cf + triC * triA;
					totalArea += triA;
				}
				if (totalArea <= 0) continue;
				cf = cf * (static_cast<XFoam_Scalar>(1) / totalArea);
				const XFoam_Scalar sign = (f.owner == cellId)
					? static_cast<XFoam_Scalar>(1)
					: static_cast<XFoam_Scalar>(-1);
				vol += sign * (cf.x() * af.x() + cf.y() * af.y() + cf.z() * af.z());
			}
			return vol * (static_cast<XFoam_Scalar>(1.0 / 3.0));
		};

		// ----- 8b. Snap #5 motionSmoother（Laplacian 向内传播 N 圈） -----
		if (nInternal > 0 && stats.nSnappedPoints > 0)
		{
			buildPtNbrs();
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

			std::vector<XFoam_Vector3D> disp(nPts, XFoam_Vector3D(0, 0, 0));
			for (size_t v = 0; v < nPts; ++v)
			{
				if (snapped[v]) disp[v] = pts[v] - origPts[v];
			}
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

		// ----- 8c. Snap #6 validate-and-relax -----
		// 算法（对标 OpenFOAM-13 snappySnapDriver::scaleMesh + checkMesh）：
		//   1) 遍历 cells 计算体积；阈值 := pos & not too small。
		//   2) 把 bad cell 的所有顶点（snapped 或被 smoother 移动过的）按 50% 回退到 origPts；
		//      OF 的做法是缩放 displacement 而不是直接覆盖 pts，与此等价（因为 disp = pts - orig）。
		//   3) 再算一次体积；如果还有 bad cell，再回退一次，最多 nRelaxIter 轮。
		//   4) 失败 cell 数与最坏体积均写进 stats，调用方可决定是否报警 / 重 mesh。
		if (nRelaxIt > 0 && nCells > 0 && stats.nSnappedPoints > 0)
		{
			buildCellFaces();
			constexpr XFoam_Scalar kVolEps = static_cast<XFoam_Scalar>(1e-30);
			constexpr XFoam_Scalar kRelaxAlpha = static_cast<XFoam_Scalar>(0.5);

			auto findBadCells = [&](std::vector<int>& out, XFoam_Scalar& outMinVol) {
				out.clear();
				outMinVol = std::numeric_limits<XFoam_Scalar>::max();
				for (int c = 0; c < nCells; ++c)
				{
					const XFoam_Scalar V = cellVolume(c);
					if (V < outMinVol) outMinVol = V;
					if (V <= kVolEps) out.push_back(c);
				}
			};

			std::vector<int> badCells;
			XFoam_Scalar minVol = 0;
			findBadCells(badCells, minVol);
			stats.nBadCellsInitial = static_cast<XFoam_Label>(badCells.size());
			stats.minCellVolumeInitial = minVol;

			std::vector<unsigned char> needRelax(nPts, 0);
			XFoam_Label relaxUsed = 0;
			while (!badCells.empty() && relaxUsed < nRelaxIt)
			{
				std::fill(needRelax.begin(), needRelax.end(), 0);
				for (int c : badCells)
				{
					for (int fi : cellFaces[static_cast<size_t>(c)])
					{
						for (int v : finalFaces[fi].verts) needRelax[v] = 1;
					}
				}
				// 把每个被标记的点拉回 origPts 一半；snapped 点也照样收缩，避免局部高位移
				// 把相邻 cell 拽成负体。
				// Snap #9 pointConstraint：对 boundary snapped 点，relax delta 必须先经过
				// constrainDisplacement 投影到允许 DOF（plane: 沿 surface 切向；line: 沿
				// edge 切向；fixed: 0）。这避免 relax 把点从 feature 上拽下来，但仍允许
				// 它在 surface 内滑动来打开负体积 cell。
				for (size_t v = 0; v < nPts; ++v)
				{
					if (!needRelax[v]) continue;
					const XFoam_Vector3D toOrig = origPts[v] - pts[v];
					XFoam_Vector3D step(
						toOrig.x() * kRelaxAlpha,
						toOrig.y() * kRelaxAlpha,
						toOrig.z() * kRelaxAlpha);
					if (snapped[v])
					{
						step = ptCons[v].constrainDisplacement(step);
					}
					pts[v] = pts[v] + step;
				}
				++relaxUsed;
				findBadCells(badCells, minVol);
			}
			stats.nRelaxIterationsUsed = relaxUsed;
			stats.nBadCellsFinal = static_cast<XFoam_Label>(badCells.size());
			stats.minCellVolumeFinal = minVol;
		}

		// ----- 8d. 重新统计 max snap distance（relax 后 snapped pts 可能不再是 closest） -----
		if (nRelaxIt > 0 && stats.nRelaxIterationsUsed > 0)
		{
			XFoam_Scalar maxMove = 0;
			for (size_t v = 0; v < nPts; ++v)
			{
				if (!snapped[v]) continue;
				const XFoam_Scalar mv = (pts[v] - origPts[v]).mag();
				if (mv > maxMove) maxMove = mv;
			}
			stats.maxSnapDistance = maxMove;
		}
	}

	// ----- 8.5. addLayers：按 layers { patch { nSurfaceLayers N; } } 在指定 patch 上扩展 prism 层 -----
	// 与 OpenFOAM-13 src/mesh/snappyHexMesh/snappyLayerDriver 的核心思路一致，但大幅简化：
	//
	// 几何模型（关键决定）：boundary point 真的被"拉回 mesh 内部"，让原 cell 让出 totalThickness
	// 的薄壳给 prism 层填充。具体：
	//   * 每个 patch 唯一点 P：保存 P_stl = pts[P]；在 pts 末尾追加 N 个新点 ringV[k] (k=0..N-1)，
	//     位置 = P_stl - n_p · cumT[k]，其中 cumT[0]=0、cumT[k]=Σ_{j<k} t_j、cumT[N]=totalThickness。
	//   * 然后把 pts[P] 平移到 P_stl - n_p · cumT[N]（最深 ring N 位置）。
	//   * 这样 P 共享给 owner cell C 的 ≥1 张非 patch 面在 polyMesh 里仍只是「顶点坐标改了」，
	//     C 的体积按散度定理重新求即可；不需要重写 C 的 face 列表。
	//
	// 拓扑构造：
	//   * 原 boundary face F 用 P，所以现在 F 位于 ring N。F.neighbour = layerCell_{N-1}（最里
	//     一层）。F.fromGridBoundary = false，进 internalIdx。
	//   * 每层 k=0..N-1 加一个 prism cell layerCell_k：
	//       - bottom face F_k 用 ringV[*][k]：owner = layerCell_k；k=0 时 neighbour=-1（新 patch
	//         boundary），k>0 时 neighbour = layerCell_{k-1}。
	//       - top face：k=N-1 时 = 原 F；k<N-1 时 = F_{k+1}（即下一层的 bottom）。
	//   * side quad：沿 F 每条 edge (v_i, v_j) 各加 N 张 quad，第 k 张连接 ringV[v_*][k] 与
	//     ringV[v_*][k+1]（k=N-1 时用 P 替代 ringV[v_*][N]）；patch 内共享 edge → 同 quad 被两
	//     个 layer cell 共享（FaceKey dedup）；patch 边缘 edge → quad 落入 walls patch。
	//
	// 已移植：
	//   * relativeSizes=true 时，每个 vertex 的 firstLayerThickness 缩放为 layer_.firstLayerThickness
	//     × sqrt(avg incident patch face area)（局部 cell-size 代理），层厚跟着曲率/网格密度变化。
	//   * layer cell 体积扫描：新生成的 prism cell 若 V≤0 计入 stats.nLayerCellsNegative，
	//     最小体积写 stats.minLayerCellVolume，调用方可据此告警 / 关层。
	// 未移植项（与 OF 显著差异，留作后续 TODO）：
	//   * medial axis / nGrow / nBufferCellsNoExtrude / nSmoothNormals / nSmoothThickness；
	//   * 凹凸自检 / layer collapse / thickness-to-cell-size 限制（重叠时会出负体积，目前仅在
	//     stats.nLayerCellsNegative 里报警，不回退 / 不 collapse）；
	//   * 多 patch 共享同一个 point 时合并 normal 的策略（这里每 patch 独立处理，跨 patch 不
	//     共享 layer vertex；共享点会被各自 patch 平移，可能产生几何冲突）。
	if (phases_.addLayers && !layer_.perPatchLayers.empty() && layer_.firstLayerThickness > 0)
	{
		using LayerEdgeKey = std::pair<int, int>;
		struct LayerEdgeKeyHash
		{
			size_t operator()(const LayerEdgeKey& k) const noexcept
			{
				const size_t h1 = std::hash<int>()(k.first);
				const size_t h2 = std::hash<int>()(k.second);
				return h1 * 0x9E3779B97F4A7C15ULL ^ h2;
			}
		};

		// 把 perPatchLayers 的 dict key 解到 stlIdxBySurf 的下标。仅匹配 STL surface patch；
		// walls / 其它 default patch 暂不支持 prism 扩张。
		std::vector<std::pair<XFoam_Label, XFoam_Label>> patchLayerJobs;
		for (XFoam_Label si = 0; si < nSurf; ++si)
		{
			const XFoam_Word& sname = surfaces_[si].name;
			auto it = layer_.perPatchLayers.find(sname);
			if (it == layer_.perPatchLayers.end()) continue;
			const XFoam_Label nL = it();
			if (nL <= 0) continue;
			if (stlIdxBySurf[static_cast<size_t>(si)].empty()) continue;
			patchLayerJobs.emplace_back(si, nL);
		}

		// 全部新增 layer cell 的下标范围（用于最后做体积扫描）。
		const int layerCellStartId = static_cast<int>(stats.nKeptCells);
		// 取消有效 layer job 时不做任何事 → 走原路径。
		for (size_t job = 0; job < patchLayerJobs.size(); ++job)
		{
			const XFoam_Label si = patchLayerJobs[job].first;
			const XFoam_Label nL = patchLayerJobs[job].second;
			std::vector<int>& patchFaces = stlIdxBySurf[static_cast<size_t>(si)];

			// (a) 几何累计层厚向量。relativeSizes=true 时层厚要按 per-vertex local cell size
			// 缩放，所以我们不算全局 cumT，而是只算「相对量」 cumRel[k] = Σ_{j<k} r^j，最终
			// 每个 vertex 用 cumT_v[k] = scale_v · firstLayerThickness · cumRel[k]。
			// 全局 abs 模式 (relativeSizes=false) 取 scale_v ≡ 1 → 与旧行为完全一致。
			std::vector<XFoam_Scalar> cumRel(static_cast<size_t>(nL + 1), 0);
			for (XFoam_Label k = 0; k < nL; ++k)
			{
				const XFoam_Scalar rk = std::pow(
					static_cast<XFoam_Scalar>(layer_.expansionRatio), static_cast<XFoam_Scalar>(k));
				cumRel[static_cast<size_t>(k + 1)] = cumRel[static_cast<size_t>(k)] + rk;
			}

			// (b) 收集 patch 上每个 unique vertex 的 area-weighted outward 法向，
			// 同时累计 (面积总和, 入射 face 数)。relativeSizes 用的 local cell-size 取
			// sqrt(avg incident face area) — 比 face 边长更稳。
			auto faceAreaVec = [&](const FInfo& f) -> XFoam_Vector3D {
				XFoam_Vector3D a(0, 0, 0);
				const int n = static_cast<int>(f.verts.size());
				if (n < 3) return a;
				const XFoam_Vector3D v0 = pts[f.verts[0]];
				for (int i = 1; i + 1 < n; ++i)
				{
					const XFoam_Vector3D e1 = pts[f.verts[i]] - v0;
					const XFoam_Vector3D e2 = pts[f.verts[i + 1]] - v0;
					a.x() += static_cast<XFoam_Scalar>(0.5) * (e1.y() * e2.z() - e1.z() * e2.y());
					a.y() += static_cast<XFoam_Scalar>(0.5) * (e1.z() * e2.x() - e1.x() * e2.z());
					a.z() += static_cast<XFoam_Scalar>(0.5) * (e1.x() * e2.y() - e1.y() * e2.x());
				}
				return a;
			};
			std::unordered_map<int, XFoam_Vector3D> pNormSum;
			std::unordered_map<int, XFoam_Scalar> pAreaSum;
			std::unordered_map<int, int> pIncCount;
			pNormSum.reserve(patchFaces.size() * 4);
			pAreaSum.reserve(patchFaces.size() * 4);
			pIncCount.reserve(patchFaces.size() * 4);
			for (int fi : patchFaces)
			{
				const XFoam_Vector3D a = faceAreaVec(finalFaces[fi]);
				const XFoam_Scalar area = a.mag();
				for (int v : finalFaces[fi].verts)
				{
					auto itp = pNormSum.find(v);
					if (itp == pNormSum.end()) pNormSum.emplace(v, a);
					else { itp->second.x() += a.x(); itp->second.y() += a.y(); itp->second.z() += a.z(); }
					auto ita = pAreaSum.find(v);
					if (ita == pAreaSum.end()) pAreaSum.emplace(v, area);
					else ita->second += area;
					auto itc = pIncCount.find(v);
					if (itc == pIncCount.end()) pIncCount.emplace(v, 1);
					else ++itc->second;
				}
			}
			// (c) ringV[v][k] = ring k 上的 vertex id（k=0..nL-1 为新点；k=nL 用原 v）。
			std::unordered_map<int, std::vector<int>> ringV;
			ringV.reserve(pNormSum.size());
			for (auto& kv : pNormSum)
			{
				const int curV = kv.first;
				const XFoam_Vector3D rawN = kv.second;
				const XFoam_Scalar mag = rawN.mag();
				if (mag <= 0)
				{
					// 退化点：normal 为 0；把 ringV 全设成 curV，使生成的 prism cell 退化为 0
					// 体积（不会导致 polyMesh 写入失败，但会被 validate-and-relax 标 bad）。
					ringV.emplace(curV, std::vector<int>(static_cast<size_t>(nL), curV));
					continue;
				}
				const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / mag;
				const XFoam_Vector3D n(rawN.x() * inv, rawN.y() * inv, rawN.z() * inv);

				// per-vertex 首层厚度。relativeSizes=true 时 scale_v = sqrt(avg incident face
				// area)；false 时 scale_v = 1。
				XFoam_Scalar t0_v = layer_.firstLayerThickness;
				if (layer_.relativeSizes)
				{
					const auto ita = pAreaSum.find(curV);
					const auto itc = pIncCount.find(curV);
					if (ita != pAreaSum.end() && itc != pIncCount.end() && itc->second > 0)
					{
						const XFoam_Scalar avgA = ita->second / static_cast<XFoam_Scalar>(itc->second);
						const XFoam_Scalar L = std::sqrt(std::max<XFoam_Scalar>(avgA, 0));
						t0_v = layer_.firstLayerThickness * L;
					}
				}

				const XFoam_Vector3D Pstl = pts[curV];
				std::vector<int> rings(static_cast<size_t>(nL));
				for (XFoam_Label k = 0; k < nL; ++k)
				{
					const XFoam_Scalar shift = t0_v * cumRel[static_cast<size_t>(k)];
					const XFoam_Vector3D pNew(
						Pstl.x() - n.x() * shift,
						Pstl.y() - n.y() * shift,
						Pstl.z() - n.z() * shift);
					rings[static_cast<size_t>(k)] = static_cast<int>(pts.size());
					pts.push_back(pNew);
					++stats.nLayerPointsAdded;
				}
				// 把原 vertex 平移到 ring N（最深）。
				const XFoam_Scalar totalShift = t0_v * cumRel[static_cast<size_t>(nL)];
				pts[curV] = XFoam_Vector3D(
					Pstl.x() - n.x() * totalShift,
					Pstl.y() - n.y() * totalShift,
					Pstl.z() - n.z() * totalShift);
				ringV.emplace(curV, std::move(rings));
			}
			auto ringVid = [&](int v, XFoam_Label k) -> int {
				if (k == nL) return v;
				auto it = ringV.find(v);
				if (it == ringV.end()) return v; // 不在 patch 上（理论上不会发生）
				return it->second[static_cast<size_t>(k)];
			};

			// (d) 拓扑构造：原 F 在 ring N，再为每层加 bottom face + side quads。
			std::vector<int> newOutermostFaces; // ring 0 face → 替换 patchFaces
			newOutermostFaces.reserve(patchFaces.size());

			// per-edge / per-layer 共享：sideQuad[(min(vi,vj),max(vi,vj),k)] = faceId
			// 这里把 edge key 与 ring k 嵌成 (a,b) where a = min,vj 压成 pair<int,int>，k 单独
			// 用一个 vector 数组按层划分（避免三元 key 哈希）。
			std::vector<std::unordered_map<LayerEdgeKey, int, LayerEdgeKeyHash>>
				sideKeyPerLayer(static_cast<size_t>(nL));

			for (int origFi : patchFaces)
			{
				FInfo& origF = finalFaces[origFi];
				const int upperCell = origF.owner;
				const int nV = static_cast<int>(origF.verts.size());

				// 当前迭代：layerCell_k = nKeptCells + 全局 layer cell 偏移；记下首个，循环 ++ 用
				const int baseCellId = static_cast<int>(stats.nKeptCells) + stats.nLayerCellsAdded;
				const int innermostLayerCell = baseCellId + (nL - 1);
				stats.nLayerCellsAdded += nL;

				// 原 F 由 boundary → internal：owner 上层 cell, neighbour innermostLayerCell。
				origF.neighbour = innermostLayerCell;
				origF.fromGridBoundary = false;
				internalIdx.push_back(origFi);

				// 逐层加 bottom face（ring k）+ 链 cell 关系。
				int prevBottomFi = -1; // F_{k+1} from previous iter (used as top for layerCell_k)
				for (XFoam_Label kk = nL - 1; kk >= 0; --kk)
				{
					const int layerCell = baseCellId + static_cast<int>(kk);
					// bottom face F_kk 顶点 = ringVid(orig vert, kk)
					FInfo bot;
					bot.verts.reserve(nV);
					for (int vid : origF.verts) bot.verts.push_back(ringVid(vid, kk));
					bot.owner = layerCell;
					bot.neighbour = (kk == 0) ? -1 : (baseCellId + static_cast<int>(kk - 1));
					bot.fromGridBoundary = false;
					const int botFi = static_cast<int>(finalFaces.size());
					finalFaces.push_back(std::move(bot));
					++stats.nLayerFacesAdded;
					if (kk > 0)
					{
						internalIdx.push_back(botFi);
					}
					else
					{
						newOutermostFaces.push_back(botFi); // 进 stlIdxBySurf[si]
					}
					// 调试时方便：标记前一个 bottom（不真正使用）
					prevBottomFi = botFi;
					(void)prevBottomFi;
				}

				// side quads：每条 edge (v_i, v_j) × 每层 k=0..nL-1。
				for (int i = 0; i < nV; ++i)
				{
					const int v_i = origF.verts[i];
					const int v_j = origF.verts[(i + 1) % nV];
					const LayerEdgeKey ek = (v_i < v_j)
						? std::make_pair(v_i, v_j)
						: std::make_pair(v_j, v_i);

					for (XFoam_Label kk = 0; kk < nL; ++kk)
					{
						auto& sm = sideKeyPerLayer[static_cast<size_t>(kk)];
						auto sit = sm.find(ek);
						const int layerCell = baseCellId + static_cast<int>(kk);
						const int vi_outer = ringVid(v_i, kk);     // ring k (closer to STL outside)
						const int vj_outer = ringVid(v_j, kk);
						const int vi_inner = ringVid(v_i, kk + 1); // ring k+1 (deeper inward)
						const int vj_inner = ringVid(v_j, kk + 1);

						if (sit == sm.end())
						{
							FInfo side;
							// CCW winding viewed from "outside" of layerCell (outward of patch sideways):
							// {vi_outer, vj_outer, vj_inner, vi_inner}
							side.verts = {vi_outer, vj_outer, vj_inner, vi_inner};
							side.owner = layerCell;
							side.neighbour = -1;
							side.fromGridBoundary = true; // patch 边缘 quad 暂归 walls
							const int sFi = static_cast<int>(finalFaces.size());
							finalFaces.push_back(std::move(side));
							sm.emplace(ek, sFi);
							++stats.nLayerFacesAdded;
						}
						else
						{
							FInfo& side = finalFaces[sit->second];
							side.neighbour = layerCell;
							side.fromGridBoundary = false;
							internalIdx.push_back(sit->second);
						}
					}
				}
			}

			// 把 patch 边缘 side quad（最后没找到 partner）补进 walls。
			for (XFoam_Label kk = 0; kk < nL; ++kk)
			{
				for (auto& kv : sideKeyPerLayer[static_cast<size_t>(kk)])
				{
					const int sFi = kv.second;
					if (finalFaces[sFi].neighbour == -1 && finalFaces[sFi].fromGridBoundary)
					{
						wallsIdx.push_back(sFi);
					}
				}
			}

			// 用新的 ring-0 bottom face 替换原 patch face list。
			patchFaces = std::move(newOutermostFaces);
			++stats.nLayerPatches;
		}

		// 全部 layer patch 处理完后再把 layer cell 数加到 nKeptCells。
		stats.nKeptCells += stats.nLayerCellsAdded;

		// (e) layer cell quality 扫描：反向收集每个新 layer cell 的 face 列表 → 散度定理
		// 求体积。≤0 计入 stats.nLayerCellsNegative；同时记录最小体积。这是 OF 的
		// snappyLayerDriver::checkAndRevertLayers 的极简版（不会 revert，仅报警）。
		if (stats.nLayerCellsAdded > 0)
		{
			const int layerCellEndId = static_cast<int>(stats.nKeptCells);
			std::vector<std::vector<int>> lcFaces(static_cast<size_t>(stats.nLayerCellsAdded));
			for (int fi = 0; fi < static_cast<int>(finalFaces.size()); ++fi)
			{
				const FInfo& f = finalFaces[fi];
				if (f.owner >= layerCellStartId && f.owner < layerCellEndId)
				{
					lcFaces[static_cast<size_t>(f.owner - layerCellStartId)].push_back(fi);
				}
				if (f.neighbour >= layerCellStartId && f.neighbour < layerCellEndId)
				{
					lcFaces[static_cast<size_t>(f.neighbour - layerCellStartId)].push_back(fi);
				}
			}
			XFoam_Scalar minVol = std::numeric_limits<XFoam_Scalar>::max();
			XFoam_Label nNeg = 0;
			for (int c = 0; c < stats.nLayerCellsAdded; ++c)
			{
				const int globalC = layerCellStartId + c;
				XFoam_Scalar vol = 0;
				for (int fi : lcFaces[static_cast<size_t>(c)])
				{
					const FInfo& f = finalFaces[fi];
					const int n = static_cast<int>(f.verts.size());
					if (n < 3) continue;
					const XFoam_Vector3D v0 = pts[f.verts[0]];
					XFoam_Vector3D af(0, 0, 0);
					XFoam_Vector3D cf(0, 0, 0);
					XFoam_Scalar totalArea = 0;
					for (int i = 1; i + 1 < n; ++i)
					{
						const XFoam_Vector3D v1 = pts[f.verts[i]];
						const XFoam_Vector3D v2 = pts[f.verts[i + 1]];
						const XFoam_Vector3D e1 = v1 - v0;
						const XFoam_Vector3D e2 = v2 - v0;
						const XFoam_Vector3D triN(
							static_cast<XFoam_Scalar>(0.5) * (e1.y() * e2.z() - e1.z() * e2.y()),
							static_cast<XFoam_Scalar>(0.5) * (e1.z() * e2.x() - e1.x() * e2.z()),
							static_cast<XFoam_Scalar>(0.5) * (e1.x() * e2.y() - e1.y() * e2.x()));
						const XFoam_Scalar triA = triN.mag();
						const XFoam_Vector3D triC = (v0 + v1 + v2) * (static_cast<XFoam_Scalar>(1.0 / 3.0));
						af = af + triN;
						cf = cf + triC * triA;
						totalArea += triA;
					}
					if (totalArea <= 0) continue;
					cf = cf * (static_cast<XFoam_Scalar>(1) / totalArea);
					const XFoam_Scalar sign = (f.owner == globalC)
						? static_cast<XFoam_Scalar>(1)
						: static_cast<XFoam_Scalar>(-1);
					vol += sign * (cf.x() * af.x() + cf.y() * af.y() + cf.z() * af.z());
				}
				vol *= static_cast<XFoam_Scalar>(1.0 / 3.0);
				if (vol < minVol) minVol = vol;
				if (vol <= 0) ++nNeg;
			}
			stats.nLayerCellsNegative = nNeg;
			stats.minLayerCellVolume = (minVol == std::numeric_limits<XFoam_Scalar>::max())
				? 0 : minVol;
		}

		// addLayers 改动 wallsIdx / stlIdxBySurf / internalIdx 的 owner 序，统一重排。
		std::sort(wallsIdx.begin(), wallsIdx.end(),
			[&](int a, int b) { return finalFaces[a].owner < finalFaces[b].owner; });
		for (auto& v : stlIdxBySurf)
		{
			std::sort(v.begin(), v.end(),
				[&](int a, int b) { return finalFaces[a].owner < finalFaces[b].owner; });
		}
		std::sort(internalIdx.begin(), internalIdx.end(), [&](int a, int b) {
			const FInfo& fa = finalFaces[a];
			const FInfo& fb = finalFaces[b];
			if (fa.owner != fb.owner) return fa.owner < fb.owner;
			return fa.neighbour < fb.neighbour;
		});
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
