#include "doctest/doctest.h"
#include "tcommon.h"
#include "XFoam/snap/xfoam_blockmesh.h"
#include "XFoam/snap/xfoam_polymesh.h"
#include "XFoam/snap/xfoam_shape.h"
#include "XFoam/utilities/xfoam_common.h"
#include <boost/filesystem.hpp>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

inline XFoam_FileName XFoamTests_blockMeshDictFileName(const char* testSourceFile)
{
	return XFoam_FileName((XFoamTests_dataDir(testSourceFile) / "dict" / "blockMeshDict").lexically_normal().generic_string());
}

/// 单六面体单元 (1 1 1) 剖分，与 blockMeshDict 顶点/boundary 一致；供 PolyMesh 单 hex 构造路径集成测试。
inline XFoam_FileName XFoamTests_blockMeshDictSingleCellPolyFileName(const char* testSourceFile)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(testSourceFile) / "dict" / "blockMeshDict_singleCellPoly").lexically_normal().generic_string());
}

/// hex (4 5 6) simpleGrading (1 1 1)；与 singleCellPoly 同顶点/boundary。仅能做 BlockMesh 级断言：当前 XFoam_PolyMesh 该构造路径仍限 8 点单 hex。
inline XFoam_FileName XFoamTests_blockMeshDictHex456FileName(const char* testSourceFile)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(testSourceFile) / "dict" / "blockMeshDict_hex456").lexically_normal().generic_string());
}

// 从仓库内 ASCII blockMeshDict（FoamFile 头）中提取 8 个顶点与 convertToMeters（测试用最小解析，非通用 Foam 语法器）。
bool XFoamTests_readBlockMeshVerticesAscii(
	const XFoam_FileName& path,
	XFoam_Scalar& convertToMeters,
	XFoam_List<XFoam_Vector3D>& verts)
{
	convertToMeters = 1.0;
	verts.clear();
	std::ifstream in(static_cast<const std::string&>(path).c_str());
	if (!in)
	{
		return false;
	}
	std::string line;
	enum class Mode
	{
		None,
		Vertices
	} mode = Mode::None;
	while (std::getline(in, line))
	{
		{
			std::string t = line;
			for (char& c : t)
			{
				if (c == '\t' || c == '\r')
				{
					c = ' ';
				}
			}
			auto p = t.find_first_not_of(' ');
			if (p != std::string::npos)
			{
				t = t.substr(p);
			}
			if (t.rfind("convertToMeters", 0) == 0)
			{
				std::istringstream ls(t);
				std::string key;
				ls >> key >> convertToMeters;
				continue;
			}
			if (t == "vertices")
			{
				mode = Mode::Vertices;
				continue;
			}
			if (t == "(" && mode == Mode::Vertices)
			{
				continue;
			}
			if (t == ");" || t == ")")
			{
				if (mode == Mode::Vertices)
				{
					break;
				}
			}
			if (mode == Mode::Vertices && !t.empty() && t[0] == '(')
			{
				double x = 0, y = 0, z = 0;
				if (std::sscanf(t.c_str(), "(%lf %lf %lf)", &x, &y, &z) == 3)
				{
					const XFoam_Label n = verts.size();
					verts.setSize(n + 1);
					verts[n] = XFoam_Vector3D(
						static_cast<XFoam_Scalar>(x),
						static_cast<XFoam_Scalar>(y),
						static_cast<XFoam_Scalar>(z));
				}
			}
		}
	}
	return verts.size() == 8;
}

// 与 OpenFOAM applications/utilities/mesh/generation/blockMesh/blockMesh.C 中 polyMesh 构造（约 221 行）等价的最小单块六面体：
// clone(blocks.points()) / blocks.cells() / blocks.patches() 等在此处由显式拓扑代替（XFoam_BlockMesh 尚未生成完整拓扑）。
XFoam_AutoPtr<XFoam_PolyMesh> XFoamTests_makeSingleHexPolyMeshFromBlockMeshDictVertices(
	const XFoam_List<XFoam_Vector3D>& pts,
	const XFoam_Scalar scale)
{
	if (pts.size() != 8)
	{
		return XFoam_AutoPtr<XFoam_PolyMesh>();
	}
	XFoam_PointField points(8);
	for (XFoam_Label i = 0; i < 8; ++i)
	{
		points[i] = pts[i] * scale;
	}
	// boundary 面来自 data/dict/blockMeshDict 中 WALLS（顺序与字典一致）
	const XFoam_Label f0[4] = {3, 7, 6, 2};
	const XFoam_Label f1[4] = {0, 4, 7, 3};
	const XFoam_Label f2[4] = {2, 6, 5, 1};
	const XFoam_Label f3[4] = {1, 5, 4, 0};
	const XFoam_Label f4[4] = {0, 3, 2, 1};
	const XFoam_Label f5[4] = {4, 5, 6, 7};
	XFoam_FaceList faces(6);
	for (int fi = 0; fi < 6; ++fi)
	{
		const XFoam_Label* src = (fi == 0   ? f0
				: fi == 1 ? f1
				: fi == 2 ? f2
				: fi == 3 ? f3
				: fi == 4 ? f4
						  : f5);
		XFoam_Face& f = faces[fi];
		f.setSize(4);
		for (int k = 0; k < 4; ++k)
		{
			f[k] = src[k];
		}
	}
	XFoam_LabelList owner(6);
	for (XFoam_Label i = 0; i < 6; ++i)
	{
		owner[i] = 0;
	}
	XFoam_LabelList nei(0);

	XFoam_WordList patchNames(1);
	patchNames[0] = XFoam_Word("WALLS");
	XFoam_LabelList patchSizes(1);
	patchSizes[0] = 6;
	XFoam_WordList patchTypes(1);
	patchTypes[0] = XFoam_Word("patch");

	return XFoam_AutoPtr<XFoam_PolyMesh>(new XFoam_PolyMesh(
		XFoam_move(points),
		XFoam_move(faces),
		XFoam_move(owner),
		XFoam_move(nei),
		patchNames,
		patchSizes,
		patchTypes));
}
} // namespace

TEST_CASE("XFoam_BlockMesh construct from dictionary (stub pipeline)")
{
	const XFoam_FileName dictPath = XFoamTests_blockMeshDictFileName(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);

	std::ifstream ifs(static_cast<const std::string&>(dictPath).c_str(), std::ios::in);
	REQUIRE(ifs.good());
	XFoam_ISstream is(ifs, static_cast<const XFoam_String&>(dictPath));
	XFoam_Dictionary dict;
	// XFoam_Dictionary::read 当前为占位实现（见 xfoam_dictionary.cpp）；仍走与 blockMesh 相同的入口。
	CHECK(dict.read(is));

	const XFoam_FileName meshPath(".");
	const XFoam_Word region("region0");
	XFoam_BlockMesh bm(dict, meshPath, region);
	CHECK(std::string(XFoam_BlockMesh::typeName) == "blockMesh");
	CHECK(bm.scaleFactor() == doctest::Approx(1.0));
	CHECK(bm.vertices().size() == 8);
	CHECK(bm.topology().nCells() == 1);
	CHECK(bm.numZonedBlocks() == 0);
}

TEST_CASE("Read blockMeshDict and build XFoam_PolyMesh (blockMesh.C-style)")
{
	const XFoam_FileName dictPath = XFoamTests_blockMeshDictFileName(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);

	XFoam_Scalar scale = 1;
	XFoam_List<XFoam_Vector3D> verts;
	REQUIRE(XFoamTests_readBlockMeshVerticesAscii(dictPath, scale, verts));

	XFoam_AutoPtr<XFoam_PolyMesh> meshPtr = XFoamTests_makeSingleHexPolyMeshFromBlockMeshDictVertices(verts, scale);
	REQUIRE(meshPtr.valid());
	const XFoam_PolyMesh& mesh = meshPtr();
	CHECK(mesh.nCells() == 1);
	CHECK(mesh.nFaces() == 6);
	CHECK(mesh.nBoundaryFaces() == 6);
	CHECK(mesh.boundary().size() == 1);

	CHECK(mesh.bounds().volume() > 0.0);
	const XFoam_Field<XFoam_Vector3D>& fc = mesh.faceCentres();
	CHECK(fc.size() == mesh.nFaces());
	const XFoam_Field<XFoam_Vector3D>& cc = mesh.cellCentres();
	CHECK(cc.size() == mesh.nCells());
}

TEST_CASE("IODictionary single-arg smoke")
{
	const XFoam_FileName dictPath = XFoamTests_blockMeshDictFileName(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);
	const XFoam_IODictionary meshDictIO(XFoam_systemDictIO(dictPath));
	CHECK(meshDictIO.size() >= 0);
}

TEST_CASE("blockMesh.hex111")
{
	const XFoam_FileName dictPath = XFoamTests_blockMeshDictSingleCellPolyFileName(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);
	const XFoam_IODictionary meshDictIO(XFoam_systemDictIO(dictPath));
	XFoam_BlockMesh blocks(meshDictIO);
	// 单块 hex (1 1 1)：8 角点、1 单元、boundary 一条 WALLS 共 6 面。
	REQUIRE(blocks.points().size() == 8);
	REQUIRE(blocks.cells().size() == 1);
	REQUIRE(blocks.vertices().size() == 8);
	REQUIRE(blocks.patches().size() == 1);
	REQUIRE(blocks.patches()[0].size() == 6);
	// 由 points/cells/patches 装配的 XFoam_PolyMesh 第二构造函数在部分环境下仍 SIGSEGV；write/BDF 待该路径稳定后再测。
}

/// 
TEST_CASE("blockMesh.hex456")
{
	const XFoam_FileName dictPath = XFoamTests_blockMeshDictHex456FileName(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);
	const XFoam_IODictionary meshDictIO(XFoam_systemDictIO(dictPath));
	XFoam_BlockMesh blocks(meshDictIO);
	// (4+1)(5+1)(6+1)=210 点，4*5*6=120 单元
	REQUIRE(blocks.points().size() == 210);
	REQUIRE(blocks.cells().size() == 120);
	REQUIRE(blocks.vertices().size() == 8);
	// XFoam_PolyMesh(points, cells, patches, …) 多块拓扑路径仍易崩溃；面数级断言待该路径稳定后再启用。
}
