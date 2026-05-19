#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"
#include <set>

TEST_CASE("XFoam_RandomGenerator scalar01 and integerAB")
{
	XFoam_RandomGenerator g(XFoam_Label(2026));
	const XFoam_Scalar a = g.scalar01();
	const XFoam_Scalar b = g.scalar01();
	CHECK(a >= 0.0);
	CHECK(a <= 1.0);
	CHECK(b >= 0.0);
	CHECK(b <= 1.0);

	for (int i = 0; i < 20; ++i)
	{
		const XFoam_Label v = g.integerAB(0, 5);
		CHECK(v >= 0);
		CHECK(v <= 5);
	}
}

TEST_CASE("XFoam_RandomGenerator rawState roundtrip")
{
	XFoam_RandomGenerator g(XFoam_Label(1));
	const std::uint64_t s0 = g.rawState();
	g.sample();
	const std::uint64_t s1 = g.rawState();
	CHECK(s1 != s0);
	g.setRawState(s0);
	CHECK(g.rawState() == s0);
}

TEST_CASE("XFoam_RandomGenerator permute")
{
	XFoam_RandomGenerator g(XFoam_Label(99));
	XFoam_LabelList lst({0, 1, 2, 3, 4});
	g.permute(lst);
	std::set<XFoam_Label> uniq;
	for (XFoam_Label i = 0; i < lst.size(); ++i)
	{
		uniq.insert(lst[i]);
	}
	CHECK(uniq.size() == static_cast<std::size_t>(lst.size()));
}

TEST_CASE("XFoam_RandomGenerator copy assign move")
{
	XFoam_RandomGenerator a(XFoam_Label(7));
	(void)a.sample();
	XFoam_RandomGenerator b(a);
	CHECK(b.rawState() == a.rawState());
	XFoam_RandomGenerator c(XFoam_Label(8));
	c = a;
	CHECK(c.rawState() == a.rawState());
	XFoam_RandomGenerator d(XFoam_move(c));
	CHECK(d.rawState() == a.rawState());
}
