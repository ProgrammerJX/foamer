#include "XFoam/snap/xfoam_blockvert.h"
#include <cctype>
#include <string>

namespace
{
void skipWs(XFoam_IStream& is)
{
	while (is.good())
	{
		const int ch = is.peek();
		if (ch == std::char_traits<char>::eof())
		{
			break;
		}
		if (!std::isspace(static_cast<unsigned char>(ch)))
		{
			break;
		}
		is.get();
	}
}
} // namespace

XFoam_BlockVert::XFoam_BlockVert() = default;

XFoam_BlockVert::~XFoam_BlockVert() = default;

XFoam_AutoPtr<XFoam_BlockVert> XFoam_BlockVert::clone() const
{
	return XFoam_AutoPtr<XFoam_BlockVert>();
}

XFoam_AutoPtr<XFoam_BlockVert> XFoam_BlockVert::New(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	const XFoam_SearchableSurfaceList& geometry,
	XFoam_IStream& is)
{
	(void)dict;
	(void)geometry;
	skipWs(is);
	const int c = is.peek();
	if (c == std::char_traits<char>::eof())
	{
		XFoam_FatalErrorInFunction << "Empty stream while reading blockVertex" << XFoam_abort(XFoam_FatalError);
		return XFoam_AutoPtr<XFoam_BlockVert>();
	}
	if (static_cast<char>(c) == '(')
	{
		return XFoam_AutoPtr<XFoam_BlockVert>(new XFoam_PointVert(dict, index, geometry, is));
	}

	XFoam_Word vertexTypeW;
	if (!(is >> vertexTypeW))
	{
		XFoam_FatalErrorInFunction << "Failed to read blockVertex type" << XFoam_abort(XFoam_FatalError);
		return XFoam_AutoPtr<XFoam_BlockVert>();
	}
	const XFoam_String vertexType(static_cast<const XFoam_String&>(vertexTypeW));
	if (vertexType == "point")
	{
		return XFoam_AutoPtr<XFoam_BlockVert>(new XFoam_PointVert(dict, index, geometry, is));
	}
	XFoam_FatalErrorInFunction
		<< "Unknown blockVertex type " << vertexType << "\nValid blockVertex types are: point\n"
		<< XFoam_abort(XFoam_FatalError);
	return XFoam_AutoPtr<XFoam_BlockVert>();
}

XFoam_Label XFoam_BlockVert::read(XFoam_IStream& is, const XFoam_Dictionary& dict)
{
	(void)dict;
	XFoam_Label v = -1;
	is >> v;
	return v;
}

void XFoam_BlockVert::write(
	XFoam_OStream& os,
	const XFoam_Label val,
	const XFoam_Dictionary& d)
{
	(void)d;
	os << val;
}

XFoam_PointVert::XFoam_PointVert(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	const XFoam_SearchableSurfaceList& geometry,
	XFoam_IStream& is)
	: XFoam_BlockVert()
	, vertex_()
{
	(void)dict;
	(void)index;
	(void)geometry;
	is >> vertex_;
}

XFoam_PointVert::~XFoam_PointVert() = default;

XFoam_PointVert::operator XFoam_Vector3D() const
{
	return vertex_;
}
