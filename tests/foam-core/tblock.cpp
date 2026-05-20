#include "doctest/doctest.h"
#include "XFoam/snap/xfoam_block.h"
#include "XFoam/snap/xfoam_blockface.h"
#include "XFoam/utilities/xfoam_common.h"

TEST_CASE("XFoam_BlockDescriptor hex metrics and facePoints")
{
	XFoam_List<XFoam_Vector3D> verts(8);
	verts[0] = XFoam_Vector3D(0, 0, 0);
	verts[1] = XFoam_Vector3D(1, 0, 0);
	verts[2] = XFoam_Vector3D(1, 1, 0);
	verts[3] = XFoam_Vector3D(0, 1, 0);
	verts[4] = XFoam_Vector3D(0, 0, 1);
	verts[5] = XFoam_Vector3D(1, 0, 1);
	verts[6] = XFoam_Vector3D(1, 1, 1);
	verts[7] = XFoam_Vector3D(0, 1, 1);

	XFoam_FixedList<XFoam_Label, 8> lab({0, 1, 2, 3, 4, 5, 6, 7});
	const XFoam_CellShape hex("hex", lab);

	XFoam_BlockEdgeList edges;
	XFoam_BlockFaceList faces;

	XFoam_List<XFoam_GradingDescriptors> expand(12);
	const XFoam_Vector<XFoam_Label> density(2, 2, 2);

	const XFoam_BlockDescriptor bd(hex, verts, edges, faces, density, expand, XFoam_String());

	CHECK(bd.nPoints() == 27);
	CHECK(bd.nCells() == 8);
	CHECK(bd.pointLabel(2, 2, 2) == 26);
	CHECK(bd.blockPoint(6).z() == doctest::Approx(1.0));

	XFoam_List<XFoam_Vector3D> pts(27);
	for (XFoam_Label k = 0; k <= 2; ++k)
	{
		for (XFoam_Label j = 0; j <= 2; ++j)
		{
			for (XFoam_Label i = 0; i <= 2; ++i)
			{
				pts[bd.pointLabel(i, j, k)] = XFoam_Vector3D(
					0.5 * static_cast<double>(i),
					0.5 * static_cast<double>(j),
					0.5 * static_cast<double>(k));
			}
		}
	}
	const auto fp = bd.facePoints(pts);
	CHECK(fp[0].size() == 9);
	CHECK(fp[0][0].x() == doctest::Approx(0.0));
	CHECK(fp[5][8].z() == doctest::Approx(1.0));
}

TEST_CASE("XFoam_BlockDescriptor curved face match")
{
	XFoam_List<XFoam_Vector3D> verts(8);
	for (int i = 0; i < 8; ++i)
	{
		verts[i] = XFoam_Vector3D(0, 0, static_cast<double>(i));
	}
	XFoam_FixedList<XFoam_Label, 8> lab({0, 1, 2, 3, 4, 5, 6, 7});
	const XFoam_CellShape hex("hex", lab);
	XFoam_BlockEdgeList edges;
	XFoam_BlockFaceList faces;
	// bottom face of hex: verts 0,1,2,3
	faces.append(XFoam_AutoPtr<XFoam_BlockFace>(
		new XFoam_blockFaces::XFoam_PlaneFace(XFoam_Face({0, 1, 2, 3}))));

	XFoam_List<XFoam_GradingDescriptors> expand(12);
	const XFoam_Vector<XFoam_Label> density(1, 1, 1);
	const XFoam_BlockDescriptor bd(hex, verts, edges, faces, density, expand, XFoam_String());
	CHECK(bd.nCurvedFaces() == 1);
	CHECK(bd.curvedFaces()[4] == 0);
}

TEST_CASE("XFoam_CellModel hex volume and centre")
{
	const XFoam_CellModel& m = XFoam_CellModel::hex();
	CHECK(m.nPoints() == 8);
	CHECK(m.nFaces() == 6);
	CHECK(m.nEdges() == 12);

	XFoam_LabelList lab(8);
	for (XFoam_Label i = 0; i < 8; ++i)
	{
		lab[i] = i;
	}
	XFoam_List<XFoam_Vector3D> pts(8);
	pts[0] = XFoam_Vector3D(0, 0, 0);
	pts[1] = XFoam_Vector3D(1, 0, 0);
	pts[2] = XFoam_Vector3D(1, 1, 0);
	pts[3] = XFoam_Vector3D(0, 1, 0);
	pts[4] = XFoam_Vector3D(0, 0, 1);
	pts[5] = XFoam_Vector3D(1, 0, 1);
	pts[6] = XFoam_Vector3D(1, 1, 1);
	pts[7] = XFoam_Vector3D(0, 1, 1);

	const XFoam_Scalar vol = m.mag(lab, pts);
	CHECK(std::abs(vol - 1.0) < 1e-6);
	const XFoam_Vector3D c = m.centre(lab, pts);
	CHECK(c.x() == doctest::Approx(0.5));
	CHECK(c.y() == doctest::Approx(0.5));
	CHECK(c.z() == doctest::Approx(0.5));

	CHECK(&m == &XFoam_CellModel::hex());
	CHECK(m == XFoam_CellModel::hex());
}

TEST_CASE("XFoam_CellShape matches OpenFOAM cellShape API")
{
	XFoam_FixedList<XFoam_Label, 8> lab({0, 1, 2, 3, 4, 5, 6, 7});
	XFoam_CellShape cs("hex", lab);
	CHECK(&cs.model() == &XFoam_CellModel::hex());
	CHECK(cs.nPoints() == 8);
	CHECK(cs.nFaces() == 6);
	CHECK(cs.nEdges() == 12);
	CHECK(cs.faces().size() == 6);
	CHECK(cs.edges().size() == 12);
	XFoam_LabelList lab2(8);
	for (XFoam_Label i = 0; i < 8; ++i)
	{
		lab2[i] = i;
	}
	const XFoam_CellShape cs2(XFoam_CellModel::hex(), lab2);
	CHECK(cs2 == cs);
}

// 对标 OpenFOAM case：system/blockMeshDict 中 blocks 单 hex + simpleGrading（例：cylinder4）。
TEST_CASE("blockMeshDict blocks: hex (200 20 20) simpleGrading (1 1 1)")
{
	XFoam_PointField vertices(8);
	for (XFoam_Label i = 0; i < 8; ++i)
	{
		vertices[i] = XFoam_Vector3D(0, 0, 0);
	}
	XFoam_BlockEdgeList edges;
	XFoam_BlockFaceList faces;
	XFoam_Dictionary meshDict;

	// 与 cylinder4 system/blockMeshDict 中单条一致：hex 顶点 + (nX nY nZ) + simpleGrading (ex ex ex)
	const char* blockLine = "hex (0 1 2 3 4 5 6 7) (200 20 20) simpleGrading (1 1 1)";

	XFoam_IStringStream iss(blockLine);
	XFoam_Block b(meshDict, 0, vertices, edges, faces, iss());
	CHECK(b.density().x() == 200);
	CHECK(b.density().y() == 20);
	CHECK(b.density().z() == 20);
	CHECK(b.nCells() == 200 * 20 * 20);
	CHECK(b.nPoints() == 201 * 21 * 21);
	CHECK(&b.blockShape().model() == &XFoam_CellModel::hex());
	for (XFoam_Label vi = 0; vi < 8; ++vi)
	{
		CHECK(b.blockShape()[vi] == vi);
	}
}
