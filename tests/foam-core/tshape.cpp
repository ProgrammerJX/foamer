#include "doctest/doctest.h"
#include "XFoam/snap/xfoam_shape.h"

TEST_CASE("XFoam_Face area compare reverseFace edges")
{
	XFoam_Face tri({0, 1, 2});
	XFoam_Face triRev({0, 2, 1});
	CHECK(XFoam_Face::compare(tri, tri) == 1);
	CHECK(XFoam_Face::compare(tri, triRev) == -1);
	CHECK(tri == triRev);
	CHECK(tri.mag(XFoam_List<XFoam_Vector3D>{
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(1, 0, 0),
		XFoam_Vector3D(0, 1, 0),
	}) == doctest::Approx(0.5));

	const XFoam_Face rev = tri.reverseFace();
	CHECK(rev[0] == 0);
	CHECK(rev[1] == 2);
	CHECK(rev[2] == 1);

	const XFoam_List<XFoam_Edge> ee = tri.edges();
	CHECK(ee.size() == 3);
	CHECK(ee[0].start() == 0);
	CHECK(ee[0].end() == 1);
	CHECK(tri.edgeDirection(ee[0]) == 1);

	XFoam_Face q({0, 0, 1, 2});
	CHECK(q.collapse() == 3);
}

TEST_CASE("XFoam_Edge compare centre mag connected")
{
	const XFoam_Edge ab(0, 1);
	const XFoam_Edge ba(1, 0);
	const XFoam_Edge cd(2, 3);
	CHECK(XFoam_Edge::compare(ab, ab) == 1);
	CHECK(XFoam_Edge::compare(ab, ba) == -1);
	CHECK(XFoam_Edge::compare(ab, cd) == 0);
	CHECK(ab == ba);
	CHECK(ab != cd);
	CHECK(ab.connected(ba));
	CHECK(ab.commonVertex(ba) == 0);
	CHECK(ab.otherVertex(1) == 0);

	XFoam_List<XFoam_Vector3D> pts;
	pts.append(XFoam_Vector3D(0, 0, 0));
	pts.append(XFoam_Vector3D(3, 0, 0));
	CHECK(ab.mag(pts) == doctest::Approx(3.0));
	CHECK(ab.centre(pts).x() == doctest::Approx(1.5));
	CHECK(ab.vec(pts).x() == doctest::Approx(3.0));
	const XFoam_LinePointRef seg = ab.line(pts);
	CHECK(seg.mag() == doctest::Approx(3.0));
}

TEST_CASE("XFoam_Cell labels edges mag centre bb opposingFace operator==")
{
	// 单位立方体顶点与 XFoam_CellModel::hex 面顺序一致
	XFoam_List<XFoam_Vector3D> pts(8);
	pts[0] = XFoam_Vector3D(0, 0, 0);
	pts[1] = XFoam_Vector3D(1, 0, 0);
	pts[2] = XFoam_Vector3D(1, 1, 0);
	pts[3] = XFoam_Vector3D(0, 1, 0);
	pts[4] = XFoam_Vector3D(0, 0, 1);
	pts[5] = XFoam_Vector3D(1, 0, 1);
	pts[6] = XFoam_Vector3D(1, 1, 1);
	pts[7] = XFoam_Vector3D(0, 1, 1);

	XFoam_List<XFoam_Face> fs(6);
	fs[0] = XFoam_Face({0, 3, 7, 4});
	fs[1] = XFoam_Face({1, 2, 6, 5});
	fs[2] = XFoam_Face({0, 1, 5, 4});
	fs[3] = XFoam_Face({3, 2, 6, 7});
	fs[4] = XFoam_Face({0, 1, 2, 3});
	fs[5] = XFoam_Face({4, 5, 6, 7});

	const XFoam_Cell cell({0, 1, 2, 3, 4, 5});
	CHECK(cell.nFaces() == 6);
	CHECK(cell.labels(fs).size() == 8);
	CHECK(cell.edges(fs).size() == 12);
	CHECK(cell.mag(pts, fs) == doctest::Approx(1.0));
	CHECK(cell.centre(pts, fs).x() == doctest::Approx(0.5));
	CHECK(cell.centre(pts, fs).y() == doctest::Approx(0.5));
	CHECK(cell.centre(pts, fs).z() == doctest::Approx(0.5));

	const XFoam_BoundBox bx = cell.bb(pts, fs);
	CHECK(bx.min().x() == doctest::Approx(0.0));
	CHECK(bx.max().z() == doctest::Approx(1.0));

	CHECK(XFoam_Cell::opposingFaceLabel(cell, 4, fs) == 5);
	const XFoam_OppositeFace opp = cell.opposingFace(4, fs);
	CHECK(opp.found());
	CHECK(opp.oppositeIndex() == 5);
	CHECK(opp.masterIndex() == 4);

	XFoam_Cell perm({5, 4, 3, 2, 1, 0});
	CHECK(perm == cell);
	CHECK_FALSE(perm == XFoam_Cell({0, 1, 2, 3, 4}));
}
