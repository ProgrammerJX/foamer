#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"

namespace
{
struct Counted
{
	static int alive;
	Counted() { ++alive; }
	Counted(const Counted&) { ++alive; }
	Counted& operator=(const Counted&) { return *this; }
	~Counted() { --alive; }
};
int Counted::alive = 0;

struct Widget
{
	int v;
	explicit Widget(int x = 0)
		: v(x)
	{
	}

	XFoam_AutoPtr<Widget> clone() const
	{
		return XFoam_AutoPtr<Widget>(new Widget(v + 100));
	}
};
} // namespace

TEST_CASE("XFoam_AutoPtr default and valid")
{
	XFoam_AutoPtr<int> a;
	CHECK(a.empty());
	CHECK_FALSE(a.valid());

	XFoam_AutoPtr<int> b(new int(7));
	CHECK(b.valid());
	CHECK_FALSE(b.empty());
	CHECK(*b == 7);
	CHECK(b() == 7);
}

TEST_CASE("XFoam_AutoPtr ptr release and manual delete")
{
	int* raw = new int(42);
	XFoam_AutoPtr<int> a(raw);
	int* p = a.ptr();
	CHECK(p == raw);
	CHECK(a.empty());
	delete p;
}

TEST_CASE("XFoam_AutoPtr reset and clear")
{
	XFoam_AutoPtr<int> a(new int(1));
	a.reset(new int(2));
	CHECK(*a == 2);
	a.clear();
	CHECK(a.empty());
}

TEST_CASE("XFoam_AutoPtr operator-> and const access")
{
	struct Node
	{
		int x = 11;
	};
	XFoam_AutoPtr<Node> n(new Node());
	CHECK(n->x == 11);
	const XFoam_AutoPtr<Node>& cn = n;
	CHECK(cn->x == 11);
	CHECK((*cn).x == 11);
}

TEST_CASE("XFoam_AutoPtr copy constructor transfers ownership")
{
	XFoam_AutoPtr<int> a(new int(5));
	XFoam_AutoPtr<int> b(a);
	CHECK(b.valid());
	CHECK(*b == 5);
	CHECK(a.empty());
}

TEST_CASE("XFoam_AutoPtr assignment from raw pointer and from autoPtr")
{
	XFoam_AutoPtr<int> a;
	a = new int(8);
	CHECK(*a == 8);

	XFoam_AutoPtr<int> b(new int(9));
	XFoam_AutoPtr<int> c;
	c = b;
	CHECK(c.valid());
	CHECK(*c == 9);
	CHECK(b.empty());
}

TEST_CASE("XFoam_AutoPtr move constructor and move assignment")
{
	XFoam_AutoPtr<int> a(new int(20));
	XFoam_AutoPtr<int> b(XFoam_move(a));
	CHECK(b.valid());
	CHECK(*b == 20);
	CHECK(a.empty());

	XFoam_AutoPtr<int> c(new int(30));
	b = XFoam_move(c);
	CHECK(b.valid());
	CHECK(*b == 30);
	CHECK(c.empty());
}

TEST_CASE("XFoam_AutoPtr const copy with reuse true")
{
	XFoam_AutoPtr<int> a(new int(100));
	const XFoam_AutoPtr<int> ca(a); // a emptied
	XFoam_AutoPtr<int> b(ca, true);
	CHECK(b.valid());
	CHECK(*b == 100);
	CHECK(ca.empty());
}

TEST_CASE("XFoam_AutoPtr const copy with reuse false uses clone")
{
	const XFoam_AutoPtr<Widget> a(new Widget(1));
	XFoam_AutoPtr<Widget> b(a, false);
	CHECK(b.valid());
	CHECK(a.valid());
	CHECK(a->v == 1);
	CHECK(b->v == 101);
}

TEST_CASE("XFoam_AutoPtr destructor deletes object")
{
	CHECK(Counted::alive == 0);
	{
		XFoam_AutoPtr<Counted> p(new Counted());
		CHECK(Counted::alive == 1);
	}
	CHECK(Counted::alive == 0);
}
