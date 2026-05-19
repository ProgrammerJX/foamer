#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"

TEST_CASE("XFoam_BoundBox from corners and metrics")
{
	XFoam_List<XFoam_Vector3D> pts(3);
	pts[0] = XFoam_Vector3D(0, 0, 0);
	pts[1] = XFoam_Vector3D(2, 0, 0);
	pts[2] = XFoam_Vector3D(1, 3, 1);
	const XFoam_BoundBox bb(pts);
	CHECK(bb.min().x() == doctest::Approx(0));
	CHECK(bb.max().y() == doctest::Approx(3));
	CHECK(bb.volume() == doctest::Approx(6.0));
	CHECK(bb.midpoint().x() == doctest::Approx(1));
	CHECK(bb.span().z() == doctest::Approx(1));
	const XFoam_Scalar d = bb.mag();
	CHECK(d == doctest::Approx(std::sqrt(4.0 + 9.0 + 1.0)));
}

TEST_CASE("XFoam_BoundBox contains overlaps nearest faces")
{
	const XFoam_BoundBox bb(XFoam_Vector3D(0, 0, 0), XFoam_Vector3D(1, 1, 1));
	CHECK(bb.contains(XFoam_Vector3D(0.5, 0.5, 0.5)));
	CHECK_FALSE(bb.containsInside(XFoam_Vector3D(0, 0.5, 0.5)));
	const XFoam_BoundBox inner(XFoam_Vector3D(0.2, 0.2, 0.2), XFoam_Vector3D(0.8, 0.8, 0.8));
	CHECK(bb.contains(inner));
	const XFoam_BoundBox ovl(XFoam_Vector3D(0.5, 0.5, 0.5), XFoam_Vector3D(2, 2, 2));
	CHECK(bb.overlaps(ovl));
	CHECK(bb.overlaps(XFoam_Vector3D(0.5, 0.5, 0.5), 0.5 * 0.5));
	const auto n = bb.nearest(XFoam_Vector3D(2, 0.5, 0.5));
	CHECK(n.x() == doctest::Approx(1));
	CHECK(n.y() == doctest::Approx(0.5));
	const auto f = XFoam_BoundBox::faces();
	CHECK(f.size() == 6);
	CHECK(f[0][2] == 2);
}

TEST_CASE("XFoam_BoundBox stream roundtrip")
{
	const XFoam_BoundBox a(XFoam_Vector3D(1, 2, 3), XFoam_Vector3D(4, 5, 6));
	XFoam_OStringStream os;
	os << a;
	XFoam_IStringStream is(os.str());
	XFoam_BoundBox b;
	is >> b;
	CHECK(b == a);
}

TEST_CASE("XFoam_BoundBox subset indices")
{
	XFoam_List<XFoam_Vector3D> pts(4);
	for (int i = 0; i < 4; ++i)
	{
		pts[i] = XFoam_Vector3D(0, 0, static_cast<double>(i));
	}
	XFoam_LabelList ix(2);
	ix[0] = 1;
	ix[1] = 3;
	const XFoam_BoundBox bb(pts, ix);
	CHECK(bb.min().z() == doctest::Approx(1));
	CHECK(bb.max().z() == doctest::Approx(3));
	XFoam_FixedList<XFoam_Label, 3> tri({0, 1, 2});
	CHECK(XFoam_BoundBox(pts, tri).max().z() == doctest::Approx(2));
}
