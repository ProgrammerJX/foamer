#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"

static_assert(
	std::is_same<XFoam_Field<XFoam_Scalar>::cmptType, XFoam_Scalar>::value,
	"scalar Field cmptType");
static_assert(
	std::is_same<XFoam_Field<XFoam_Vector3D>::cmptType, XFoam_Vector3D>::value,
	"vector Field cmptType");

TEST_CASE("XFoam_Field and XFoam_SubField basics")
{
	XFoam_Field<XFoam_Scalar> a(3, 2.0);
	CHECK(a.size() == 3);
	CHECK(a[1] == doctest::Approx(2.0));

	XFoam_Field<XFoam_Scalar> b(a);
	CHECK(b[0] == doctest::Approx(2.0));

	XFoam_SubField<XFoam_Scalar> sub(a, 2, 1);
	CHECK(sub.size() == 2);
	CHECK(sub[0] == doctest::Approx(2.0));
	sub[0] = 5.0;
	CHECK(a[1] == doctest::Approx(5.0));
}

TEST_CASE("XFoam_Field clone yields tmp")
{
	XFoam_Field<XFoam_Scalar> a(2, 3.0);
	XFoam_Tmp<XFoam_Field<XFoam_Scalar>> t = a.clone();
	CHECK(t().size() == 2);
	CHECK(t()[0] == doctest::Approx(3.0));
}

TEST_CASE("XFoam_Field reductions and unary ops")
{
	XFoam_Field<XFoam_Scalar> f({1.0, -2.0, 3.0});
	CHECK(XFoam_sum(f) == doctest::Approx(2.0));
	CHECK(XFoam_max(f) == doctest::Approx(3.0));
	CHECK(XFoam_min(f) == doctest::Approx(-2.0));
	CHECK(XFoam_average(f) == doctest::Approx(2.0 / 3.0));

	CHECK(XFoam_sumMag(f) == doctest::Approx(6.0));
	CHECK(XFoam_maxMag(f) == doctest::Approx(3.0));

	const XFoam_Field<XFoam_Scalar> n = -f;
	CHECK(n[0] == doctest::Approx(-1.0));
	CHECK(XFoam_neg(f)[1] == doctest::Approx(2.0));

	CHECK(XFoam_mag(f)[0] == doctest::Approx(1.0));
	CHECK(XFoam_mag(f)[1] == doctest::Approx(2.0));
	CHECK(XFoam_magSqr(f)[2] == doctest::Approx(9.0));

	const XFoam_Field<XFoam_Scalar> g({4.0, 9.0});
	CHECK(XFoam_sqrt(g)[0] == doctest::Approx(2.0));
	CHECK(XFoam_sqrt(g)[1] == doctest::Approx(3.0));
	CHECK(XFoam_pow(g, XFoam_Scalar(0.5))[0] == doctest::Approx(2.0));

	const XFoam_Field<XFoam_Scalar> h({2.0, -3.0});
	CHECK(XFoam_pow3(h)[0] == doctest::Approx(8.0));
	CHECK(XFoam_pow4(h)[1] == doctest::Approx(81.0));
}

TEST_CASE("XFoam_Field empty reductions throw")
{
	XFoam_Field<XFoam_Scalar> empty;
	CHECK_THROWS_AS(XFoam_max(empty), const XFoam_Error&);
	CHECK_THROWS_AS(XFoam_min(empty), const XFoam_Error&);
	CHECK_THROWS_AS(XFoam_average(empty), const XFoam_Error&);
	CHECK_THROWS_AS(XFoam_maxMag(empty), const XFoam_Error&);
}

TEST_CASE("XFoam_Field binary and elementwise ops")
{
	XFoam_Field<XFoam_Scalar> a({1.0, 2.0, 6.0});
	XFoam_Field<XFoam_Scalar> b({1.0, 4.0, 2.0});

	const XFoam_Field<XFoam_Scalar> s = XFoam_add(a, b);
	CHECK(s[0] == doctest::Approx(2.0));
	CHECK(s[1] == doctest::Approx(6.0));

	CHECK(XFoam_subtract(a, b)[2] == doctest::Approx(4.0));
	CHECK(XFoam_multiply(a, b)[1] == doctest::Approx(8.0));
	CHECK(XFoam_divide(a, b)[2] == doctest::Approx(3.0));

	CHECK((a + b)[0] == doctest::Approx(2.0));
	CHECK((a - b)[1] == doctest::Approx(-2.0));
	CHECK((a * b)[2] == doctest::Approx(12.0));
	CHECK((a / b)[0] == doctest::Approx(1.0));

	CHECK((a + XFoam_Scalar(1.0))[0] == doctest::Approx(2.0));
	CHECK((XFoam_Scalar(10.0) + a)[1] == doctest::Approx(12.0));
	CHECK((a - XFoam_Scalar(1.0))[0] == doctest::Approx(0.0));
	CHECK((XFoam_Scalar(3.0) - a)[1] == doctest::Approx(1.0));
	CHECK((a * XFoam_Scalar(2.0))[2] == doctest::Approx(12.0));
	CHECK((XFoam_Scalar(2.0) * a)[2] == doctest::Approx(12.0));
	CHECK((a / XFoam_Scalar(2.0))[1] == doctest::Approx(1.0));
	CHECK((XFoam_Scalar(12.0) / a)[2] == doctest::Approx(2.0));

	CHECK(XFoam_min(a, b)[1] == doctest::Approx(2.0));
	CHECK(XFoam_max(a, b)[1] == doctest::Approx(4.0));

	XFoam_Field<XFoam_Scalar> c(2, 1.0);
	XFoam_Field<XFoam_Scalar> d(3, 2.0);
	CHECK_THROWS_AS(XFoam_add(c, d), const XFoam_Error&);
}

TEST_CASE("XFoam_Field vector reductions and mag")
{
	XFoam_Field<XFoam_Vector3D> u(2);
	u[0] = XFoam_Vector3D(3, 0, 4);
	u[1] = XFoam_Vector3D(0, 1, 0);

	CHECK(XFoam_sum(u)[0] == doctest::Approx(3.0));
	CHECK(XFoam_sum(u)[1] == doctest::Approx(1.0));
	CHECK(XFoam_sumMag(u) == doctest::Approx(6.0));
	CHECK(XFoam_maxMag(u) == doctest::Approx(5.0));
	CHECK(XFoam_mag(u)[0] == doctest::Approx(5.0));
	CHECK(XFoam_magSqr(u)[0] == doctest::Approx(25.0));
}

TEST_CASE("XFoam_Field vector ops")
{
	XFoam_Field<XFoam_Vector3D> u(2);
	u[0] = XFoam_Vector3D(1, 0, 0);
	u[1] = XFoam_Vector3D(0, 1, 0);
	XFoam_Field<XFoam_Vector3D> v(2);
	v[0] = XFoam_Vector3D(0, 1, 0);
	v[1] = XFoam_Vector3D(1, 0, 0);

	const XFoam_Field<XFoam_Vector3D> c = u ^ v;
	CHECK(c[0].z() == doctest::Approx(1.0));
	CHECK((u & v)[0] == doctest::Approx(0.0));
	CHECK(XFoam_sumProd(u, v) == doctest::Approx(0.0));

	const XFoam_Field<XFoam_Vector3D> h = u * v;
	CHECK(h[0].x() == doctest::Approx(0.0));
	CHECK(h[1].y() == doctest::Approx(0.0));

	const XFoam_Field<XFoam_Vector3D> w = XFoam_sqr(u);
	CHECK(w[0].x() == doctest::Approx(1.0));
	CHECK(w[1].y() == doctest::Approx(1.0));

	XFoam_Field<XFoam_Scalar> x;
	XFoam_component(x, u, 0);
	CHECK(x[0] == doctest::Approx(1.0));
	CHECK(x[1] == doctest::Approx(0.0));

	XFoam_Field<XFoam_Vector3D> y(1, XFoam_Vector3D(0, 0, 0));
	XFoam_Field<XFoam_Scalar> comp(1, 7.0);
	XFoam_replaceComponent(y, comp, 1);
	CHECK(y[0].y() == doctest::Approx(7.0));
}
