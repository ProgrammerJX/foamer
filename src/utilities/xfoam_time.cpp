#include "XFoam/utilities/xfoam_time.h"

XFoam_API XFoam_Time::XFoam_Time()
	: XFoam_ObjectRegistry(static_cast<const XFoam_Time&>(*this), 128)
	, rootPath_()
	, caseName_(".")
	, globalCaseName_(".")
	, timeName_("0")
	, constant_("constant")
	, system_("system")
{
}
