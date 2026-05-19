#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"

TEST_CASE("XFoam_Circulator and XFoam_ConstCirculator")
{
	XFoam_LabelList lst({10, 20, 30});
	XFoam_Circulator<XFoam_LabelList> c(lst);
	CHECK(c.size() == 3);
	CHECK(*c == 10);
	CHECK(c.next() == 20);
	CHECK(c.prev() == 30);
	++c;
	CHECK(*c == 20);
	--c;
	CHECK(*c == 10);

	int sum = 0;
	if (c.size())
	{
		do
		{
			sum += c();
		} while (c.circulate(XFoam_CirculatorBase::direction::clockwise));
	}
	CHECK(sum == 10 + 20 + 30);

	const XFoam_LabelList& clst = lst;
	XFoam_ConstCirculator<XFoam_LabelList> cc(clst);
	CHECK(cc.size() == 3);
	CHECK(*cc == 10);
	int csum = 0;
	if (cc.size())
	{
		do
		{
			csum += cc();
		} while (cc.circulate(XFoam_CirculatorBase::direction::clockwise));
	}
	CHECK(csum == 60);
}
