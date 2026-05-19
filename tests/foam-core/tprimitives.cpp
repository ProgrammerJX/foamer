#include "doctest/doctest.h"
#include "XFoam/primitive/xfoam_line.h"
#include "XFoam/primitive/xfoam_triangle.h"
#include "XFoam/primitive/xfoam_plane.h"
#include "XFoam/primitive/xfoam_tetrahedron.h"
#include "XFoam/primitive/xfoam_pyramid.h"
#include "XFoam/utilities/xfoam_common.h"

TEST_CASE("XFoam_LinePoints basic and nearestDist point")
{
	const XFoam_Vector3D a(0, 0, 0);
	const XFoam_Vector3D b(1, 0, 0);
	const XFoam_LinePoints ln(a, b);

	CHECK(ln.mag() == doctest::Approx(1.0));
	CHECK(ln.vec().x() == doctest::Approx(1.0));
	CHECK(ln.centre().x() == doctest::Approx(0.5));

	const XFoam_PointHit<XFoam_Vector3D> hMid = ln.nearestDist(XFoam_Vector3D(0.5, 1.0, 0.0));
	CHECK(hMid.hit());
	CHECK(hMid.rawPoint().x() == doctest::Approx(0.5));
	CHECK(hMid.rawPoint().y() == doctest::Approx(0.0));

	const XFoam_PointHit<XFoam_Vector3D> hLeft = ln.nearestDist(XFoam_Vector3D(-1.0, 0.0, 0.0));
	CHECK_FALSE(hLeft.hit());
	CHECK(hLeft.rawPoint().x() == doctest::Approx(0.0));
}

TEST_CASE("XFoam_LinePointRef and line-line nearestDist")
{
	//异面：*this 在 z=0 沿 x；edge 在 x=0、y=1 沿 z。最近端点 (0,0,0) 与 (0,1,0)，距离 1。
	XFoam_Vector3D a0(0, 0, 0);
	XFoam_Vector3D b0(1, 0, 0);
	XFoam_Vector3D a1(0, 1, 0);
	XFoam_Vector3D b1(0, 1, 1);

	const XFoam_LinePoints seg0(a0, b0);
	const XFoam_LinePointRef seg1(a1, b1);

	XFoam_Vector3D p0;
	XFoam_Vector3D p1;
	const XFoam_Scalar d = seg0.nearestDist(seg1, p0, p1);
	CHECK(d == doctest::Approx(1.0));
	CHECK(p0.x() == doctest::Approx(0.0));
	CHECK(p0.y() == doctest::Approx(0.0));
	CHECK(p1.x() == doctest::Approx(0.0));
	CHECK(p1.y() == doctest::Approx(1.0));
}

TEST_CASE("XFoam_TrianglePoints area centre barycentric")
{
	const XFoam_Vector3D a(0, 0, 0);
	const XFoam_Vector3D b(1, 0, 0);
	const XFoam_Vector3D c(0, 1, 0);
	const XFoam_TrianglePoints tri(a, b, c);

	CHECK(tri.mag() == doctest::Approx(0.5));
	CHECK(tri.normal().z() == doctest::Approx(1.0));

	const XFoam_Vector3D ctr = tri.centre();
	CHECK(ctr.x() == doctest::Approx(1.0 / 3.0));
	CHECK(ctr.y() == doctest::Approx(1.0 / 3.0));

	XFoam_Barycentric2D bar;
	tri.pointToBarycentric(ctr, bar);
	CHECK(bar[0] + bar[1] + bar[2] == doctest::Approx(1.0));
}

TEST_CASE("XFoam_TrianglePoints ray intersection")
{
	const XFoam_TrianglePoints tri(
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(1, 0, 0),
		XFoam_Vector3D(0, 1, 0));

	const XFoam_PointHit<XFoam_Vector3D> hit = tri.intersection(
		XFoam_Vector3D(0.25, 0.25, 1.0),
		XFoam_Vector3D(0, 0, -1),
		XFoam_Intersection::algorithm::fullRay,
		0.0);

	CHECK(hit.hit());
	CHECK(hit.rawPoint().z() == doctest::Approx(0.0));
	CHECK(hit.distance() == doctest::Approx(1.0));
}

TEST_CASE("XFoam_TrianglePoints nearestPoint")
{
	const XFoam_TrianglePoints tri(
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(1, 0, 0),
		XFoam_Vector3D(0, 1, 0));

	const XFoam_PointHit<XFoam_Vector3D> n = tri.nearestPoint(XFoam_Vector3D(-0.5, -0.5, 0.0));
	CHECK_FALSE(n.hit());
	CHECK(n.rawPoint().x() == doctest::Approx(0.0));
	CHECK(n.rawPoint().y() == doctest::Approx(0.0));
}

TEST_CASE("XFoam_Plane xy distance nearest signedDistance")
{
	const XFoam_Plane pl(
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(1, 0, 0),
		XFoam_Vector3D(0, 1, 0));

	CHECK(pl.valid());
	CHECK(pl.normal().z() == doctest::Approx(1.0));

	const XFoam_Vector3D q(3.0, 4.0, 5.0);
	CHECK(pl.signedDistance(q) == doctest::Approx(5.0));
	CHECK(pl.distance(q) == doctest::Approx(5.0));

	const XFoam_Vector3D n = pl.nearestPoint(q);
	CHECK(n.z() == doctest::Approx(0.0));
	CHECK(n.x() == doctest::Approx(3.0));
}

TEST_CASE("XFoam_Plane lineIntersect and normalIntersect")
{
	const XFoam_Plane pl(
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(0, 0, 1));

	const XFoam_LinePoints vertical(
		XFoam_Vector3D(1, 1, -2),
		XFoam_Vector3D(1, 1, 2));

	const XFoam_Scalar t = pl.lineIntersect(vertical);
	CHECK(t == doctest::Approx(0.5));

	const XFoam_Scalar t2 = pl.normalIntersect(XFoam_Vector3D(0, 0, -1), XFoam_Vector3D(0, 0, 1));
	CHECK(t2 == doctest::Approx(1.0));
}

TEST_CASE("XFoam_Plane mirror and side")
{
	const XFoam_Plane pl(
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(0, 0, 1));

	const XFoam_Vector3D p(1, 2, 3);
	const XFoam_Vector3D m = pl.mirror(p);
	CHECK(m.z() == doctest::Approx(-3.0));
	CHECK(m.x() == doctest::Approx(1.0));

	CHECK(pl.sideOfPlane(XFoam_Vector3D(0, 0, 1)) == XFoam_Plane::NORMAL);
	CHECK(pl.sideOfPlane(XFoam_Vector3D(0, 0, -1)) == XFoam_Plane::FLIP);
}

TEST_CASE("XFoam_Plane planeIntersect two planes")
{
	const XFoam_Plane xy(
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(1, 0, 0),
		XFoam_Vector3D(0, 1, 0));
	const XFoam_Plane xz(
		XFoam_Vector3D(0, 0, 0),
		XFoam_Vector3D(1, 0, 0),
		XFoam_Vector3D(0, 0, 1));

	const XFoam_Plane::ray r = xy.planeIntersect(xz);
	const XFoam_Vector3D d = r.dir();
	const XFoam_Scalar md = static_cast<XFoam_Scalar>(std::sqrt(static_cast<double>(d.magSqr())));
	CHECK(md > 1e-9);
	// 交线平行于 x轴：dir 与 (1,0,0) 平行
	const XFoam_Vector3D ux(1, 0, 0);
	const XFoam_Scalar crossMag = (d / md ^ ux).mag();
	CHECK(crossMag == doctest::Approx(0.0).epsilon(1e-8));
}

TEST_CASE("XFoam_Plane planePlaneIntersect three planes")
{
	const XFoam_Plane px(1, 0, 0, 0); // x = 0
	const XFoam_Plane py(0, 1, 0, 0); // y = 0
	const XFoam_Plane pz(0, 0, 1, 0); // z = 0

	const XFoam_Vector3D o = px.planePlaneIntersect(py, pz);
	CHECK(o.x() == doctest::Approx(0.0));
	CHECK(o.y() == doctest::Approx(0.0));
	CHECK(o.z() == doctest::Approx(0.0));
}

TEST_CASE("XFoam_TensorD identity multiply and inverse")
{
	const XFoam_TensorD I = XFoam_TensorD::I();
	const XFoam_TensorD A(1, 2, 3, 0, 1, 4, 5, 6, 0);
	const XFoam_TensorD P = A & I;
	CHECK(P == A);
	const XFoam_TensorD Ai = XFoam_inv(A);
	const XFoam_TensorD R = Ai & A;
	CHECK(R.xx() == doctest::Approx(1.0));
	CHECK(R.yy() == doctest::Approx(1.0));
	CHECK(R.zz() == doctest::Approx(1.0));
	CHECK(R.xy() == doctest::Approx(0.0).epsilon(1e-12));
}

TEST_CASE("XFoam_PyramidQuadLabels centre height mag")
	{
		// 单位正方形 z=0 为底，顶点 (0.5,0.5,1)：体积 = (1/3)*底面积*高 = 1/3
		const XFoam_List<XFoam_Vector3D> pts{
			XFoam_Vector3D(0, 0, 0),
			XFoam_Vector3D(1, 0, 0),
			XFoam_Vector3D(1, 1, 0),
			XFoam_Vector3D(0, 1, 0),
		};

		const XFoam_PyramidQuadBase base(XFoam_FixedList<XFoam_Label, 4>({0, 1, 2, 3}));
		const XFoam_Vector3D apex(0.5, 0.5, 1.0);
		const XFoam_PyramidQuadLabels pyr(base, apex);

		const XFoam_Vector3D ctr = pyr.centre(pts);
		CHECK(ctr.x() == doctest::Approx(0.5));
		CHECK(ctr.y() == doctest::Approx(0.5));
		CHECK(ctr.z() == doctest::Approx(0.25));

		const XFoam_Vector3D h = pyr.height(pts);
		CHECK(h.z() == doctest::Approx(1.0));

		CHECK(pyr.mag(pts) == doctest::Approx(1.0 / 3.0));
	}

	TEST_CASE("XFoam_Tetrahedron volume centre inside barycentric")
{
	const XFoam_Vector3D a(0, 0, 0);
	const XFoam_Vector3D b(1, 0, 0);
	const XFoam_Vector3D c(0, 1, 0);
	const XFoam_Vector3D d(0, 0, 1);
	const XFoam_TetrahedronPoints tet(a, b, c, d);

	CHECK(tet.mag() == doctest::Approx(1.0 / 6.0));
	CHECK(tet.centre().x() == doctest::Approx(0.25));
	CHECK(tet.centre().y() == doctest::Approx(0.25));
	CHECK(tet.centre().z() == doctest::Approx(0.25));

	CHECK(tet.inside(XFoam_Vector3D(0.1, 0.1, 0.1)));
	CHECK_FALSE(tet.inside(XFoam_Vector3D(0.8, 0.8, 0.8)));

	XFoam_Barycentric br(0.25, 0.25, 0.25, 0.25);
	CHECK(tet.barycentricToPoint(br).x() == doctest::Approx(0.25));

	XFoam_Barycentric back;
	tet.pointToBarycentric(tet.centre(), back);
	CHECK(back[0] + back[1] + back[2] + back[3] == doctest::Approx(1.0).epsilon(1e-9));

	const XFoam_BoundBox bb = tet.bounds();
	CHECK(bb.min().x() == doctest::Approx(0.0));
	CHECK(bb.max().z() == doctest::Approx(1.0));

	XFoam_RandomGenerator rnd(12345);
	const XFoam_Vector3D rp = tet.randomPoint(rnd);
	CHECK(tet.inside(rp));
}
