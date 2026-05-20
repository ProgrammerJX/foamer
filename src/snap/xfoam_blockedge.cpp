#include "XFoam/snap/xfoam_blockedge.h"
#include "XFoam/snap/xfoam_blockvert.h"
#include <string>

XFoam_BlockEdge::XFoam_BlockEdge(
	const XFoam_UList<XFoam_Vector3D>& pts,
	const XFoam_Label start,
	const XFoam_Label end)
	: points_(pts)
	, start_(start)
	, end_(end)
{}

XFoam_BlockEdge::XFoam_BlockEdge(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	const XFoam_UList<XFoam_Vector3D>& pts,
	XFoam_IStream& is)
	: points_(pts)
	, start_(XFoam_BlockVert::read(is, dict))
	, end_(XFoam_BlockVert::read(is, dict))
{
	(void)index;
}

XFoam_BlockEdge::~XFoam_BlockEdge() = default;

XFoam_Tmp<XFoam_Field<XFoam_Vector3D>> XFoam_BlockEdge::position(
	const XFoam_ScalarList& lambdas) const
{
	auto* out = new XFoam_Field<XFoam_Vector3D>(lambdas.size());
	for (XFoam_Label i = 0; i < lambdas.size(); ++i)
	{
		(*out)[i] = position(lambdas[i]);
	}
	return XFoam_Tmp<XFoam_Field<XFoam_Vector3D>>(out);
}

XFoam_List<XFoam_Vector3D> XFoam_BlockEdge::appendEndPoints(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_Label start,
	const XFoam_Label end,
	const XFoam_UList<XFoam_Vector3D>& otherKnots)
{
	XFoam_List<XFoam_Vector3D> allKnots(otherKnots.size() + 2);
	allKnots[0] = points[start];
	allKnots[allKnots.size() - 1] = points[end];
	for (XFoam_Label knotI = 0; knotI < otherKnots.size(); ++knotI)
	{
		allKnots[knotI + 1] = otherKnots[knotI];
	}
	return allKnots;
}

void XFoam_BlockEdge::write(XFoam_OStream& os, const XFoam_Dictionary& d) const
{
	(void)d;
	os << start_ << '\t' << end_ << '\n';
}

XFoam_AutoPtr<XFoam_BlockEdge> XFoam_BlockEdge::clone() const
{
	return XFoam_AutoPtr<XFoam_BlockEdge>();
}

XFoam_AutoPtr<XFoam_BlockEdge> XFoam_BlockEdge::New(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	const XFoam_SearchableSurfaceList& geometry,
	const XFoam_UList<XFoam_Vector3D>& points,
	XFoam_IStream& is)
{
	(void)geometry;
	XFoam_String edgeType;
	is >> edgeType;
	if (edgeType == "line")
	{
		return XFoam_AutoPtr<XFoam_BlockEdge>(
			new XFoam_LineEdge(dict, index, geometry, points, is));
	}
	XFoam_FatalErrorInFunction
		<< "Unknown blockEdge type " << edgeType << "\nValid blockEdge types are: line\n"
		<< XFoam_abort(XFoam_FatalError);
	return XFoam_AutoPtr<XFoam_BlockEdge>();
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BlockEdge& p)
{
	os << p.start() << '\t' << p.end() << '\n';
	return os;
}

XFoam_LineEdge::XFoam_LineEdge(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_Label start,
	const XFoam_Label end)
	: XFoam_BlockEdge(points, start, end)
{}

XFoam_LineEdge::XFoam_LineEdge(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	const XFoam_SearchableSurfaceList& geometry,
	const XFoam_UList<XFoam_Vector3D>& points,
	XFoam_IStream& is)
	: XFoam_BlockEdge(dict, index, points, is)
{
	(void)geometry;
}

XFoam_LineEdge::~XFoam_LineEdge() = default;

XFoam_Vector3D XFoam_LineEdge::position(const XFoam_Scalar lambda) const
{
	if (lambda < -XFoam_small || lambda > 1.0 + XFoam_small)
	{
		XFoam_FatalErrorInFunction
			<< "Parameter out of range, lambda = " << lambda << XFoam_abort(XFoam_FatalError);
	}
	return points_[start()] + lambda * (points_[end()] - points_[start()]);
}

XFoam_Scalar XFoam_LineEdge::length() const
{
	return static_cast<XFoam_Scalar>((points_[end()] - points_[start()]).mag());
}
