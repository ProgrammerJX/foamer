#include "XFoam/primitive/xfoam_intersection.h"

XFoam_Scalar XFoam_Intersection::planarTol_ = static_cast<XFoam_Scalar>(0.2);

const XFoam_HashTable<int, XFoam_String> XFoam_Intersection::directionNames_({
	{XFoam_String("vector"), static_cast<int>(XFoam_Intersection::direction::vector)},
	{XFoam_String("contactSphere"), static_cast<int>(XFoam_Intersection::direction::contactSphere)},
});

const XFoam_HashTable<int, XFoam_String> XFoam_Intersection::algorithmNames_({
	{XFoam_String("fullRay"), static_cast<int>(XFoam_Intersection::algorithm::fullRay)},
	{XFoam_String("halfRay"), static_cast<int>(XFoam_Intersection::algorithm::halfRay)},
	{XFoam_String("visible"), static_cast<int>(XFoam_Intersection::algorithm::visible)},
});

XFoam_Scalar XFoam_Intersection::setPlanarTol(const XFoam_Scalar t)
{
	if (t < -XFoam_vSmall)
	{
		XFoam_FatalErrorInFunction
			<< "XFoam_Intersection::setPlanarTol(const scalar) : tolerance must be >= 0"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_Scalar oldTol = planarTol_;
	planarTol_ = t;
	return oldTol;
}
