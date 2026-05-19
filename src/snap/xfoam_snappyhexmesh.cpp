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
	if (bg.size() == 0 || !bg.set(0))
	{
		std::cerr << "snappy: background BlockMesh has no usable block 0.\n";
		return false;
	}
	if (bg.size() > 1)
	{
		std::cerr << "snappy: WARNING: multi-block bg unsupported; only block 0 will be refined.\n";
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
	const int targetLevel = std::min<int>(static_cast<int>(globalRefinementLevel()), LEVEL_CAP);

	// ----- 2. 八叉树细化（Octree refinement）-----
	// 全部拓扑（subdivide / 2:1 balance / 邻居查询）都收在 XFoam_Hex8Ref 里。
	// 这里只负责把 *world coords* 上的判断（leaf bbox 与 STL 相交、leaf 中心在 STL 哪一侧）
	// 通过 paramToWorld 翻译给它。
	XFoam_Hex8Ref oct(Nx, Ny, Nz, LEVEL_CAP);
	oct.initBaseLeaves();

	using Leaf = XFoam_Hex8Ref::Leaf;

	auto leafCornersWorld = [&](const Leaf& l, XFoam_Vector3D out[8]) {
		for (int oc = 0; oc < 8; ++oc)
		{
			XFoam_Scalar u, v, w;
			XFoam_Hex8Ref::leafCornerParam(l, oc, u, v, w);
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
	auto leafCentroidWorld = [&](const Leaf& l) -> XFoam_Vector3D {
		XFoam_Scalar u, v, w;
		XFoam_Hex8Ref::leafCentroidParam(l, u, v, w);
		return paramToWorld(blockCorner, Nx, Ny, Nz, l.ai, l.aj, l.ak, u, v, w);
	};

	// ----- 3. Phase 1: 按 STL bbox 加密 -----
	// 多 surface predicate：任一 STL 与 leaf bbox 相交、且 leaf.level 还没到该 surface 的
	// maxLevel 时 → 需要 subdivide。每个 surface 独立保各自的 maxLevel（"per-surface cap"），
	// 全局 refineByPredicate cap 取所有 maxLevel 的 max。
	std::vector<XFoam_Label> surfMaxLevel(nSurf, 0);
	for (XFoam_Label si = 0; si < nSurf; ++si)
	{
		surfMaxLevel[si] = std::min<XFoam_Label>(surfaces_[si].maxLevel,
		                                         static_cast<XFoam_Label>(LEVEL_CAP));
	}
	oct.refineByPredicate(
		static_cast<XFoam_Label>(targetLevel),
		[&](const Leaf& l) -> bool {
			const XFoam_BoundBox bb = leafBBox(l);
			for (XFoam_Label si = 0; si < nSurf; ++si)
			{
				if (!stls[si] || stls[si]->empty()) continue;
				if (l.level >= surfMaxLevel[si]) continue;
				if (stls[si]->boxIntersects(bb)) return true;
			}
			return false;
		});

	// ----- 4. Phase 1.5: nCellsBetweenLevels buffer 膨胀 -----
	// OF 语义：nCellsBetweenLevels=N → 相邻不同 level 间至少 N 个中间 level cell。
	// balance21() 自带 1 个中间 cell；外扩 (N-1) 圈把高 level 区域沿 face 邻居向粗侧膨胀。
	const int nBufferLayers = std::max(0, static_cast<int>(refine_.nCellsBetweenLevels) - 1);
	if (nBufferLayers > 0)
	{
		oct.extendHighLevel(nBufferLayers);
	}

	// ----- 4'. Phase 2: 2:1 face-balance -----
	oct.balance21();

	// ----- 5. 统计 per-level cell 数 -----
	oct.perLevelCounts(stats.perLevelCells);
	stats.maxAdaptiveLevel = oct.maxLevelReached();
	stats.nRefinedCells = oct.numLeaves();

	// ----- 6. STL 切除：用 leaf 中心做 inside/outside 判定 -----
	// pred=true 表示需要被切除；"切除" 的语义 = 与 locationInMesh 不同侧。
	// "inside" 对多 surface 取 OR：任何一个 STL contains 即视为 inside
	// (适合 STL 是封闭 obstacle 的并集；不适合 region 隔离需求，未来 #8 处理)。
	auto insideAny = [&](const XFoam_Vector3D& p) -> bool {
		for (XFoam_Label si = 0; si < nSurf; ++si)
		{
			if (stls[si] && !stls[si]->empty() && stls[si]->contains(p)) return true;
		}
		return false;
	};
	const bool locInside = insideAny(refine_.locationInMesh);
	oct.cullByPredicate(
		[&](const Leaf& l) -> bool {
			return insideAny(leafCentroidWorld(l)) != locInside;
		});

	// ----- 7. kept leaves 编 globalId -----
	stats.nKeptCells = oct.assignCellIds();

	std::vector<Leaf>& leaves = oct.leaves();

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
	auto faceFinePoint = [&](const Leaf& l, int d, int rr, int cc) -> int {
		XFoam_Scalar u, v, w;
		XFoam_Hex8Ref::faceSteinerParam(l, d, rr, cc, u, v, w);
		const XFoam_Vector3D p = paramToWorld(blockCorner, Nx, Ny, Nz, l.ai, l.aj, l.ak, u, v, w);
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
	for (size_t s = 0; s < leaves.size(); ++s)
	{
		const Leaf& l = leaves[s];
		if (!l.kept) continue;
		for (int d = 0; d < 6; ++d)
		{
			const XFoam_Hex8Ref::FaceNbr nbr = oct.resolveFaceNeighbor(l, d);
			if (nbr.kind != XFoam_Hex8Ref::FaceNbrKind::Finer) continue;
			for (int rr = 0; rr < 3; ++rr)
			{
				for (int cc = 0; cc < 3; ++cc)
				{
					(void)faceFinePoint(l, d, rr, cc);
				}
			}
		}
	}

	// 切面 sub-quad 自然 winding (rr,cc)→(rr,cc+1)→(rr+1,cc+1)→(rr+1,cc) 等价
	//   +axA × +axB；对 d=0/3/4 与 outward 法向相反，需翻转，否则负体积。
	static const bool kFlipSub[6] = {true, false, false, true, true, false};

	// ----- 10b. 正式 emit -----
	for (size_t s = 0; s < leaves.size(); ++s)
	{
		const Leaf& l = leaves[s];
		if (!l.kept) continue;
		bool isPoly = false;

		for (int d = 0; d < 6; ++d)
		{
			const XFoam_Hex8Ref::FaceNbr nbr = oct.resolveFaceNeighbor(l, d);

			if (nbr.kind == XFoam_Hex8Ref::FaceNbrKind::Finer)
			{
				// 我是粗侧：切 2x2 sub-quad。
				int pgrid[3][3];
				for (int rr = 0; rr < 3; ++rr)
				{
					for (int cc = 0; cc < 3; ++cc)
					{
						pgrid[rr][cc] = faceFinePoint(l, d, rr, cc);
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
						// sub-quad 的 4 条边可能也有更深层 (L+2 等) 的中点；同样要
						// 与对侧 fine cell 的 polygon emit 对齐，否则 dedup 失败。
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

			// 非 split face：可能仍是 polygon — 若 hex 角点连边的中点被别的 cell split
			// 时 addPoint 过，则插入该中点。最多 4 个边 → 最多多插 4 个点 → polygon 最大 8 边。
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
	if (phases_.snap)
	{
		std::vector<unsigned char> snapped(pts.size(), 0);
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
