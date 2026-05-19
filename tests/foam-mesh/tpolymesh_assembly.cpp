// Direct test of XFoam_PolyMesh "polyMeshFromShapeMesh" ctor (points + cellShapes
// + boundaryFaces + patch metadata).
//
// Inputs are hand-built in OF mesh-point space so the test stays independent of
// the BlockMesh -> mesh-point relabelling, which has a separate, known gap.
//
// Historical context: this ctor used to SIGSEGV because
// XFoam_PolyMesh::cellShapePointCells() built a DynamicList with
// `XFoam_DynamicList(nPoints)`. The ctor only resizes the backing storage and
// keeps the UList view at size 0 / data ptr nullptr, so `pc[curPoint]` indexed a
// null pointer. Fixed in xfoam_polymesh.cpp by adding `pc.setSize(nPoints)`.
//
// hex111 case below is the smallest reproducer of the original crash path. The
// numerical assertions verify topology counts match a single hex.
#include "doctest/doctest.h"
#include "tcommon.h"
#include "XFoam/block/xfoam_blockmesh.h"
#include "XFoam/mesh/xfoam_polymesh.h"
#include "XFoam/mesh/xfoam_shape.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_dictionary.h"

namespace
{

XFoam_PointField unitCubePoints()
{
	XFoam_PointField pts(8);
	pts[0] = XFoam_Vector3D(0, 0, 0);
	pts[1] = XFoam_Vector3D(1, 0, 0);
	pts[2] = XFoam_Vector3D(1, 1, 0);
	pts[3] = XFoam_Vector3D(0, 1, 0);
	pts[4] = XFoam_Vector3D(0, 0, 1);
	pts[5] = XFoam_Vector3D(1, 0, 1);
	pts[6] = XFoam_Vector3D(1, 1, 1);
	pts[7] = XFoam_Vector3D(0, 1, 1);
	return pts;
}

XFoam_CellShapeList singleHexShape()
{
	XFoam_LabelList verts(8);
	for (XFoam_Label i = 0; i < 8; ++i) verts[i] = i;
	XFoam_CellShapeList out(1);
	out[0] = XFoam_CellShape(XFoam_CellModel::hex(), verts, false);
	return out;
}

XFoam_FaceListList singleWallsPatchOnHex()
{
	// 6 OF-canonical hex faces, each a 4-vertex quad in mesh-point space.
	const XFoam_Label fv[6][4] = {
		{0, 3, 7, 4},
		{1, 2, 6, 5},
		{0, 1, 5, 4},
		{3, 2, 6, 7},
		{0, 1, 2, 3},
		{4, 5, 6, 7},
	};
	XFoam_FaceList walls(6);
	for (XFoam_Label f = 0; f < 6; ++f)
	{
		XFoam_Face quad(4);
		for (XFoam_Label k = 0; k < 4; ++k) quad[k] = fv[f][k];
		walls[f] = quad;
	}
	XFoam_FaceListList patches(1);
	patches[0] = walls;
	return patches;
}

XFoam_AutoPtr<XFoam_PolyMesh> assembleHex(
	XFoam_PointField points,
	const XFoam_CellShapeList& shapes,
	const XFoam_FaceListList& patches,
	const XFoam_Word& wallName)
{
	XFoam_WordList patchNames(1);
	patchNames[0] = wallName;
	const XFoam_PtrListDictionary<XFoam_Dictionary> patchDicts;
	// XFoam_PolyPatch::New 当前只支持 typeName ("polyPatch")；OF 中的 "patch" 别名未注册。
	return XFoam_AutoPtr<XFoam_PolyMesh>(new XFoam_PolyMesh(
		XFoam_move(points),
		shapes,
		patches,
		XFoam_move(patchNames),
		patchDicts,
		XFoam_Word("defaultFaces"),
		XFoam_Word(XFoam_PolyPatch::typeName)));
}

} // namespace

TEST_CASE("XFoam_PolyMesh ctor B: single hex, all 6 faces in one patch (no SIGSEGV)")
{
	XFoam_PointField pts = unitCubePoints();
	XFoam_CellShapeList shapes = singleHexShape();
	XFoam_FaceListList patches = singleWallsPatchOnHex();

	XFoam_AutoPtr<XFoam_PolyMesh> meshPtr = assembleHex(pts, shapes, patches, XFoam_Word("WALLS"));
	REQUIRE(meshPtr.valid());
	const XFoam_PolyMesh& mesh = meshPtr();

	CHECK(mesh.nPoints() == 8);
	CHECK(mesh.nCells() == 1);
	CHECK(mesh.nFaces() == 6);
	CHECK(mesh.nInternalFaces() == 0);
	CHECK(mesh.nBoundaryFaces() == 6);
	REQUIRE(mesh.boundary().size() >= 1);
}

TEST_CASE("XFoam_PolyMesh ctor B: single hex, no boundary -> defaultFaces collects all 6")
{
	XFoam_PointField pts = unitCubePoints();
	XFoam_CellShapeList shapes = singleHexShape();
	const XFoam_FaceListList emptyPatches;

	XFoam_WordList patchNames;
	const XFoam_PtrListDictionary<XFoam_Dictionary> patchDicts;

	XFoam_AutoPtr<XFoam_PolyMesh> meshPtr(new XFoam_PolyMesh(
		XFoam_move(pts),
		shapes,
		emptyPatches,
		XFoam_move(patchNames),
		patchDicts,
		XFoam_Word("defaultFaces"),
		XFoam_Word(XFoam_PolyPatch::typeName)));
	REQUIRE(meshPtr.valid());
	const XFoam_PolyMesh& mesh = meshPtr();

	CHECK(mesh.nCells() == 1);
	CHECK(mesh.nFaces() == 6);
	CHECK(mesh.nBoundaryFaces() == 6);
	REQUIRE(mesh.boundary().size() == 1);
}

namespace
{

inline XFoam_FileName XFoamTests_blockMeshDict_e2e_singleCell(const char* testSourceFile)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(testSourceFile) / "dict" / "blockMeshDict_singleCellPoly")
			.lexically_normal()
			.generic_string());
}

inline XFoam_FileName XFoamTests_blockMeshDict_e2e_hex456(const char* testSourceFile)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(testSourceFile) / "dict" / "blockMeshDict_hex456")
			.lexically_normal()
			.generic_string());
}

XFoam_AutoPtr<XFoam_PolyMesh> assemblePolyMeshFromBlockMesh(const XFoam_BlockMesh& blocks)
{
	XFoam_PointField points(blocks.points());
	XFoam_CellShapeList cellShapes(blocks.cells());
	XFoam_FaceListList patches(blocks.patches());
	XFoam_WordList patchNames(blocks.patchNames());
	const XFoam_PtrListDictionary<XFoam_Dictionary> patchDicts;

	return XFoam_AutoPtr<XFoam_PolyMesh>(new XFoam_PolyMesh(
		XFoam_move(points),
		cellShapes,
		patches,
		XFoam_move(patchNames),
		patchDicts,
		XFoam_Word("defaultFaces"),
		XFoam_Word(XFoam_PolyPatch::typeName)));
}

} // namespace

TEST_CASE("blockMeshDict -> PolyMesh end-to-end: single hex (1 1 1)")
{
	const XFoam_FileName dictPath = XFoamTests_blockMeshDict_e2e_singleCell(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);

	const XFoam_IODictionary meshDictIO(XFoam_systemDictIO(dictPath));
	XFoam_BlockMesh blocks(meshDictIO);
	REQUIRE(blocks.cells().size() == 1);
	REQUIRE(blocks.points().size() == 8);
	REQUIRE(blocks.patches().size() == 1);
	REQUIRE(blocks.patches()[0].size() == 6);

	XFoam_AutoPtr<XFoam_PolyMesh> meshPtr = assemblePolyMeshFromBlockMesh(blocks);
	REQUIRE(meshPtr.valid());
	const XFoam_PolyMesh& mesh = meshPtr();

	CHECK(mesh.nPoints() == 8);
	CHECK(mesh.nCells() == 1);
	CHECK(mesh.nInternalFaces() == 0);
	CHECK(mesh.nBoundaryFaces() == 6);
	CHECK(mesh.nFaces() == 6);
	REQUIRE(mesh.boundary().size() >= 1);
}

TEST_CASE("blockMeshDict -> PolyMesh end-to-end: single hex block (4 5 6)")
{
	const XFoam_FileName dictPath = XFoamTests_blockMeshDict_e2e_hex456(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);

	const XFoam_IODictionary meshDictIO(XFoam_systemDictIO(dictPath));
	XFoam_BlockMesh blocks(meshDictIO);
	REQUIRE(blocks.cells().size() == 120);
	REQUIRE(blocks.points().size() == 210);
	REQUIRE(blocks.patches().size() == 1);
	// 单 hex 块 (4,5,6)：6 个块面分别细分为 5*6 + 5*6 + 4*6 + 4*6 + 4*5 + 4*5 = 148 个子四边形。
	REQUIRE(blocks.patches()[0].size() == 148);

	XFoam_AutoPtr<XFoam_PolyMesh> meshPtr = assemblePolyMeshFromBlockMesh(blocks);
	REQUIRE(meshPtr.valid());
	const XFoam_PolyMesh& mesh = meshPtr();

	// 4*5*6 = 120 cells；内面 = (4-1)*5*6 + 4*(5-1)*6 + 4*5*(6-1) = 90 + 96 + 100 = 286；
	// 边界面 = 2*(5*6 + 4*6 + 4*5) = 148；总面 = 286 + 148 = 434。
	CHECK(mesh.nPoints() == 210);
	CHECK(mesh.nCells() == 120);
	CHECK(mesh.nInternalFaces() == 286);
	CHECK(mesh.nBoundaryFaces() == 148);
	CHECK(mesh.nFaces() == 434);
}
