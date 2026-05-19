// #include "doctest/doctest.h"

// #include "XFoam/utilities/xfoam_randomgenerator.h"

// #include <algorithm>
// #include <sstream>
// #include <string>
// #include <vector>

// TEST_CASE("XFoam_RandomGenerator: same seed same sequence")
// {
// 	XFoam_RandomGenerator a(42);
// 	XFoam_RandomGenerator b(42);
// 	for (int i = 0; i < 64; ++i)
// 	{
// 		REQUIRE(a.scalar01() == b.scalar01());
// 	}
// }

// TEST_CASE("XFoam_RandomGenerator: label 0 matches default construction")
// {
// 	XFoam_RandomGenerator d;
// 	XFoam_RandomGenerator z(0);
// 	REQUIRE(d.rawState() == z.rawState());
// }

// TEST_CASE("XFoam_RandomGenerator: scalar01 in [0,1)")
// {
// 	XFoam_RandomGenerator r(31415);
// 	for (int i = 0; i < 500; ++i)
// 	{
// 		const XFoam_Scalar u = r.scalar01();
// 		REQUIRE(u >= 0.0);
// 		REQUIRE(u < 1.0);
// 	}
// }

// TEST_CASE("XFoam_RandomGenerator: scalarAB within bounds")
// {
// 	XFoam_RandomGenerator r(271828);
// 	const XFoam_Scalar lo = -1.5;
// 	const XFoam_Scalar hi = 3.25;
// 	for (int i = 0; i < 300; ++i)
// 	{
// 		const XFoam_Scalar x = r.scalarAB(lo, hi);
// 		REQUIRE(x >= lo - 1.0e-12);
// 		REQUIRE(x <= hi + 1.0e-12);
// 	}
// }

// TEST_CASE("XFoam_RandomGenerator: integerAB inclusive range")
// {
// 	XFoam_RandomGenerator r(12345);
// 	for (int i = 0; i < 400; ++i)
// 	{
// 		const XFoam_Label k = r.integerAB(2, 7);
// 		REQUIRE(k >= 2);
// 		REQUIRE(k <= 7);
// 	}
// 	const XFoam_Label single = r.integerAB(5, 5);
// 	REQUIRE(single == 5);
// }

// TEST_CASE("XFoam_RandomGenerator: stream state round-trip")
// {
// 	XFoam_RandomGenerator g(999);
// 	(void)g.scalar01();
// 	(void)g.scalar01();

// 	std::stringstream ss;
// 	XFoam_OStream& os = static_cast<XFoam_OStream&>(ss);
// 	os << g;
// 	REQUIRE(ss.good());
// 	ss.seekg(0);

// 	XFoam_RandomGenerator h(1);
// 	XFoam_IStream& is = static_cast<XFoam_IStream&>(ss);
// 	is >> h;
// 	REQUIRE(is.good());

// 	const XFoam_Scalar xg = g.scalar01();
// 	const XFoam_Scalar xh = h.scalar01();
// 	REQUIRE(xg == xh);
// }

// TEST_CASE("XFoam_RandomGenerator: permute preserves multiset")
// {
// 	std::vector<int> v{1, 2, 2, 3, 5, 8};
// 	auto sorted = v;
// 	std::sort(sorted.begin(), sorted.end());

// 	XFoam_RandomGenerator r(161803);
// 	r.permute(v);
// 	std::sort(v.begin(), v.end());

// 	REQUIRE(v.size() == sorted.size());
// 	REQUIRE(v == sorted);
// }

// TEST_CASE("XFoam_RandomGenerator: word seed constructs")
// {
// 	XFoam_RandomGenerator::Seed s(std::string("caseDir"));
// 	XFoam_RandomGenerator r(s);
// 	REQUIRE(r.scalar01() >= 0.0);
// 	REQUIRE(r.scalar01() < 1.0);
// }

// TEST_CASE("XFoam_RandomGenerator: copy shares sequence from same state")
// {
// 	XFoam_RandomGenerator a(77);
// 	(void)a.scalar01();
// 	XFoam_RandomGenerator b(a);
// 	REQUIRE(a.scalar01() == b.scalar01());
// }

// TEST_CASE("XFoam_RandomGenerator: generator() runs")
// {
// 	XFoam_RandomGenerator r = XFoam_RandomGenerator::generator();
// 	REQUIRE(r.scalar01() >= 0.0);
// 	REQUIRE(r.scalar01() < 1.0);
// }
