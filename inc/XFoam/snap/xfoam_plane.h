#ifndef XFoam_Plane_H_
#define XFoam_Plane_H_

// 平面对齐 OpenFOAM meshes/primitiveShapes/plane/plane.H + plane.C 的主要接口。
// 未移植：dictionary 构造、writeDict（XFoam 无 Foam::dictionary）。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/snap/xfoam_line.h"

class XFoam_API XFoam_Plane
{
public:
	enum side
	{
		NORMAL,
		FLIP
	};
	class ray
	{
		XFoam_Vector3D refPoint_;
		XFoam_Vector3D dir_;

	public:
		ray(const XFoam_Vector3D& refPoint, const XFoam_Vector3D& dir)
			: refPoint_(refPoint)
			, dir_(dir)
		{}

		const XFoam_Vector3D& refPoint() const { return refPoint_; }
		const XFoam_Vector3D& dir() const { return dir_; }
	};

private:
	XFoam_Vector3D normal_;
	XFoam_Vector3D point_;

	void calcPntAndVec(XFoam_Scalar C1, XFoam_Scalar C2, XFoam_Scalar C3, XFoam_Scalar C4);
	void calcPntAndVec(
		const XFoam_Vector3D& point1,
		const XFoam_Vector3D& point2,
		const XFoam_Vector3D& point3);

public:
	explicit XFoam_Plane(const XFoam_Vector3D& normalVector);
	XFoam_Plane(const XFoam_Vector3D& basePoint, const XFoam_Vector3D& normalVector);
	XFoam_Plane(
		const XFoam_Vector3D& point1,
		const XFoam_Vector3D& point2,
		const XFoam_Vector3D& point3);
	explicit XFoam_Plane(XFoam_Scalar a, XFoam_Scalar b, XFoam_Scalar c, XFoam_Scalar d);
	bool valid() const;
	const XFoam_Vector3D& normal() const { return normal_; }
	const XFoam_Vector3D& refPoint() const { return point_; }
	XFoam_FixedList<XFoam_Scalar, 4> planeCoeffs() const;
	XFoam_Vector3D aPoint() const;
	XFoam_Vector3D nearestPoint(const XFoam_Vector3D& p) const;
	XFoam_Scalar distance(const XFoam_Vector3D& p) const;
	XFoam_Scalar signedDistance(const XFoam_Vector3D& p) const;
	XFoam_Scalar normalIntersect(const XFoam_Vector3D& pnt0, const XFoam_Vector3D& dir) const;
	XFoam_Scalar normalIntersect(const ray& r) const { return normalIntersect(r.refPoint(), r.dir()); }
	XFoam_Scalar lineIntersect(const XFoam_LinePoints& l) const;
	XFoam_Scalar lineIntersect(const XFoam_LinePointRef& l) const;
	ray planeIntersect(const XFoam_Plane& plane2) const;
	XFoam_Vector3D planePlaneIntersect(const XFoam_Plane& plane2, const XFoam_Plane& plane3) const;
	side sideOfPlane(const XFoam_Vector3D& p) const;
	XFoam_Vector3D mirror(const XFoam_Vector3D& p) const;
};

inline bool operator==(const XFoam_Plane& a, const XFoam_Plane& b)
{
	return a.normal() == b.normal() && a.refPoint() == b.refPoint();
}

inline bool operator!=(const XFoam_Plane& a, const XFoam_Plane& b)
{
	return !(a == b);
}

#endif
