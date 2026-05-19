#include "doctest/doctest.h"
#include "XFoam/utilities/xfoam_common.h"

TEST_CASE("XFoam_Time default paths")
{
	const XFoam_Time t;
	CHECK_NOTHROW((void)t.rootPath());
	CHECK_NOTHROW((void)t.caseName());
	CHECK_NOTHROW((void)t.globalCaseName());
	CHECK_NOTHROW((void)t.name());
	CHECK_NOTHROW((void)t.constant());
	CHECK_NOTHROW((void)t.system());
}
