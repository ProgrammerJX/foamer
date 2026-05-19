#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"

TEST_CASE("XFoam_hashBytes XFoam_hashWords")
{
	const char buf[] = {1, 2, 3, 4};
	const unsigned h0 = XFoam_hashBytes(buf, sizeof(buf), 0u);
	const unsigned h1 = XFoam_hashBytes(buf, sizeof(buf), 1u);
	CHECK(h0 != h1);
	XFoam_UInt32 w[] = {1u, 2u, 3u};
	const unsigned hw = XFoam_hashWords(w, 3, 0u);
	CHECK(hw != 0u);
	unsigned a = 3u;
	unsigned b = 5u;
	(void)XFoam_hashWordsDual(w, 3, a, b);
	CHECK(a != 0u);
}

TEST_CASE("XFoam_HashTable")
{
	XFoam_HashTable<int> t(8);
	CHECK(t.empty());
	CHECK_FALSE(t.found("x"));
	CHECK(t.insert("a", 1));
	CHECK_FALSE(t.insert("a", 99));
	CHECK(t.found("a"));
	CHECK(t["a"] == 1);
	t.set("a", 2);
	CHECK(t["a"] == 2);
	CHECK_FALSE(t.found("z"));
	CHECK(t.size() == 1);
	XFoam_HashTable<int> u;
	u.insert("m", 3);
	t.set(u);
	CHECK(t.size() == 2);
	CHECK(t.found("m"));
	const auto st = t.sortedToc();
	REQUIRE(st.size() == 2u);
	CHECK(st[0] == "a");
	CHECK(st[1] == "m");
	CHECK_FALSE(t.erase("z"));
	CHECK_FALSE(t.found("z"));
	auto it = t.find("a");
	REQUIRE(it != t.end());
	CHECK(it.key() == "a");
	CHECK(*it == 2);
	CHECK(t.erase(it));
	CHECK_FALSE(t.found("a"));
}

TEST_CASE("XFoam_Map XFoam_Map")
{
	XFoam_Map<double> m(16);
	CHECK(m.insert(1, 3.14));
	CHECK(m.found(1));
	CHECK(m[1] == 3.14);
	XFoam_Map<int> n{{2, 5}, {3, 7}};
	CHECK(n.size() == 2);
	CHECK(n[2] == 5);
	CHECK(n[3] == 7);
}

TEST_CASE("XFoam_HashSet wordHashSet labelHashSet")
{
	XFoam_WordHashSet ws{"a", "b"};
	CHECK(ws.size() == 2);
	CHECK(ws["a"]);
	CHECK_FALSE(ws["z"]);
	CHECK(ws.insert("c"));
	CHECK_FALSE(ws.insert("a"));
	ws -= XFoam_HashSet<XFoam_String>{"b"};
	CHECK_FALSE(ws["b"]);
	XFoam_HashSet<XFoam_String> u{"a", "c", "d"};
	const auto x = ws ^ u;
	CHECK(x.size() == 1);
	CHECK(x["d"]);
	CHECK_FALSE(x["a"]);
	XFoam_LabelHashSet ls{1, 2, 3};
	ls |= XFoam_LabelHashSet{3, 4};
	CHECK(ls.size() == 4);
	const auto both = ls & XFoam_LabelHashSet{2, 3, 99};
	CHECK(both.size() == 2);
	XFoam_HashTable<int, XFoam_Label> ht;
	ht.set(7, 0);
	ht.set(8, 1);
	XFoam_LabelHashSet fromHt(ht);
	CHECK(fromHt.size() == 2);
	CHECK(fromHt[7]);
}
