#include "XFoam/utilities/xfoam_fileoperation.h"
#include "XFoam/utilities/xfoam_error.h"
#include "XFoam/utilities/xfoam_ioobject.h"

bool XFoam_FileHandler::readHeader(
	XFoam_RegIOobject& obj,
	const XFoam_FileName& fName,
	const XFoam_Word& typeName) const
{
	(void)typeName;
	if (fName.empty())
	{
		return false;
	}
	try
	{
		XFoam_IFstream is(fName);
		if (!is.good())
		{
			return false;
		}
		return static_cast<XFoam_IOobject&>(obj).readHeader(is);
	}
	catch (const XFoam_Error&)
	{
		return false;
	}
	catch (const std::exception&)
	{
		return false;
	}
}

bool XFoam_FileHandler::read(
	XFoam_RegIOobject& obj,
	const bool masterOnly,
	const XFoam_IOstream::streamFormat defaultFormat,
	const XFoam_Word& typeNameArg) const
{
	(void)masterOnly;
	(void)defaultFormat;
	(void)typeNameArg;
	const XFoam_FileName fName = obj.filePath();
	if (fName.empty())
	{
		return false;
	}
	try
	{
		XFoam_IFstream is(fName);
		if (!is.good())
		{
			return false;
		}
		XFoam_IOobject& ioo = static_cast<XFoam_IOobject&>(obj);
		if (!ioo.readHeader(is))
		{
			return false;
		}
		return obj.readData(is);
	}
	catch (const XFoam_Error&)
	{
		return false;
	}
	catch (const std::exception&)
	{
		return false;
	}
}
