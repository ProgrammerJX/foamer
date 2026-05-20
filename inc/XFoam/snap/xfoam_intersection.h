#ifndef XFoam_Intersection_H_
#define XFoam_Intersection_H_

// 对标 OpenFOAM meshes/primitiveShapes/triangle/intersection.H / intersection.C。
// 由 src/primitive/xfoam_intersection.cpp 提供静态成员定义。

#include "XFoam/utilities/xfoam_common.h"

class XFoam_API XFoam_Intersection
{
	static XFoam_Scalar planarTol_;

public:
	enum class direction
	{
		vector,
		contactSphere
	};

	enum class algorithm
	{
		fullRay,
		halfRay,
		visible
	};

	static const XFoam_HashTable<int, XFoam_String> directionNames_;
	static const XFoam_HashTable<int, XFoam_String> algorithmNames_;

	static XFoam_Scalar planarTol() { return planarTol_; }

	static XFoam_Scalar setPlanarTol(const XFoam_Scalar t);
};

#endif
