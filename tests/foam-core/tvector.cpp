#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"

TEST_CASE("XFoam_Vector3D basics")
{
	const XFoam_Vector3D a(1, 2, 3);
	const XFoam_Vector3D b(4, 5, 6);
	CHECK((a & b) == doctest::Approx(32.0));
	const XFoam_Vector3D c = a + b;
	CHECK(c.x() == doctest::Approx(5.0));
	CHECK(c.y() == doctest::Approx(7.0));
	CHECK(c.z() == doctest::Approx(9.0));
	CHECK(a.magSqr() == doctest::Approx(14.0));
}

TEST_CASE("XFoam_Vector dot cross mag")
{
	const XFoam_Vector<double> a(1.0, 0.0, 0.0);
	const XFoam_Vector<double> b(0.0, 1.0, 0.0);
	CHECK((a & b) == doctest::Approx(0.0));
	const auto c = a ^ b;
	CHECK(c.x() == doctest::Approx(0.0));
	CHECK(c.y() == doctest::Approx(0.0));
	CHECK(c.z() == doctest::Approx(1.0));
	CHECK(a.mag() == doctest::Approx(1.0));
	const XFoam_Vector<double> z(XFoam_Zero_v);
	CHECK(z.magSqr() == doctest::Approx(0.0));
}
