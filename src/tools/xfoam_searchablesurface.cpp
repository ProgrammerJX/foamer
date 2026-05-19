#include "XFoam/tools/xfoam_searchablesurface.h"

void XFoam_TrivialSearchableSurface::findNearest(
	const XFoam_Field<XFoam_Vector3D>& points,
	const XFoam_Field<XFoam_Scalar>&,
	XFoam_List<XFoam_PointIndexHit>& hits) const
{
	hits.setSize(points.size());
	for (XFoam_Label i = 0; i < points.size(); ++i)
	{
		hits[i] = XFoam_PointIndexHit(true, points[i], -1);
	}
}
