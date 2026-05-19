#include "XFoam/primitive/xfoam_plane.h"
#include "XFoam/primitive/xfoam_line.h"

// * * * * * * * * * * * * * Private helpers (plane.C 依赖 Foam::mag/normalised/tensor；此处等价实现) * * * //

static inline XFoam_Vector3D XFoam_plane_invalidPoint()
{
	return XFoam_Vector3D(XFoam_vGreat, XFoam_vGreat, XFoam_vGreat);
}

static inline XFoam_Scalar XFoam_plane_comp(const XFoam_Vector3D& v, int i)
{
	return v[static_cast<XFoam_Size>(i)];
}

// * * * * * * * * * * * * * Private Member Functions * * * * * * * * * * * //

void XFoam_Plane::calcPntAndVec(
	const XFoam_Scalar a,
	const XFoam_Scalar b,
	const XFoam_Scalar c,
	const XFoam_Scalar d)
{
	normal_ = XFoam_Vector3D(a, b, c);
	const XFoam_Scalar magNormal = static_cast<XFoam_Scalar>(normal_.mag());

	if (magNormal > 0)
	{
		normal_ /= magNormal;
	}
	else
	{
		normal_ = XFoam_Vector3D(XFoam_Zero_v);
	}

	if (magNormal > XFoam_mag(d) * XFoam_vSmall)
	{
		point_ = (-d / magNormal) * normal_;
	}
	else
	{
		point_ = XFoam_plane_invalidPoint();
	}
}

void XFoam_Plane::calcPntAndVec(
	const XFoam_Vector3D& point1,
	const XFoam_Vector3D& point2,
	const XFoam_Vector3D& point3)
{
	normal_ = XFoam_normalised((point1 - point2) ^ (point2 - point3));
	point_ = (point1 + point2 + point3) * (1.0 / 3.0);
}

// * * * * * * * * * * * * * * * * Constructors * * * * * * * * * * * * * * //

XFoam_Plane::XFoam_Plane(const XFoam_Vector3D& normalVector)
	: normal_(XFoam_normalised(normalVector))
	, point_(XFoam_Zero_v)
{}

XFoam_Plane::XFoam_Plane(const XFoam_Vector3D& basePoint, const XFoam_Vector3D& normalVector)
	: normal_(XFoam_normalised(normalVector))
	, point_(basePoint)
{}

XFoam_Plane::XFoam_Plane(
	const XFoam_Scalar a,
	const XFoam_Scalar b,
	const XFoam_Scalar c,
	const XFoam_Scalar d)
{
	calcPntAndVec(a, b, c, d);
}

XFoam_Plane::XFoam_Plane(
	const XFoam_Vector3D& point1,
	const XFoam_Vector3D& point2,
	const XFoam_Vector3D& point3)
{
	calcPntAndVec(point1, point2, point3);
}

// * * * * * * * * * * * * * * * Member Functions * * * * * * * * * * * * * //

bool XFoam_Plane::valid() const
{
	return static_cast<double>(normal_.mag()) > static_cast<double>(XFoam_rootSmall)
		&& refPoint() != XFoam_plane_invalidPoint();
}

XFoam_FixedList<XFoam_Scalar, 4> XFoam_Plane::planeCoeffs() const
{
	XFoam_FixedList<XFoam_Scalar, 4> C;

	const XFoam_Scalar magX = XFoam_mag(normal_.x());
	const XFoam_Scalar magY = XFoam_mag(normal_.y());
	const XFoam_Scalar magZ = XFoam_mag(normal_.z());

	if (magX > magY)
	{
		if (magX > magZ)
		{
			C[0] = 1;
			C[1] = normal_.y() / normal_.x();
			C[2] = normal_.z() / normal_.x();
		}
		else
		{
			C[0] = normal_.x() / normal_.z();
			C[1] = normal_.y() / normal_.z();
			C[2] = 1;
		}
	}
	else
	{
		if (magY > magZ)
		{
			C[0] = normal_.x() / normal_.y();
			C[1] = 1;
			C[2] = normal_.z() / normal_.y();
		}
		else
		{
			C[0] = normal_.x() / normal_.z();
			C[1] = normal_.y() / normal_.z();
			C[2] = 1;
		}
	}

	C[3] = -C[0] * point_.x() - C[1] * point_.y() - C[2] * point_.z();

	return C;
}

XFoam_Vector3D XFoam_Plane::aPoint() const
{
	const XFoam_Vector3D& refPt = refPoint();
	XFoam_FixedList<XFoam_Scalar, 4> pc = planeCoeffs();
	const XFoam_Scalar perturbX = refPt.x() + static_cast<XFoam_Scalar>(1e-3);
	const XFoam_Scalar perturbY = refPt.y() + static_cast<XFoam_Scalar>(1e-3);
	const XFoam_Scalar perturbZ = refPt.z() + static_cast<XFoam_Scalar>(1e-3);

	if (XFoam_mag(pc[2]) < XFoam_small)
	{
		if (XFoam_mag(pc[1]) < XFoam_small)
		{
			const XFoam_Scalar x =
				-1.0
				* (pc[3] + pc[1] * perturbY + pc[2] * perturbZ) / pc[0];

			return XFoam_Vector3D(x, perturbY, perturbZ);
		}

		const XFoam_Scalar y =
			-1.0 * (pc[3] + pc[0] * perturbX + pc[2] * perturbZ) / pc[1];

		return XFoam_Vector3D(perturbX, y, perturbZ);
	}

	const XFoam_Scalar z =
		-1.0 * (pc[3] + pc[0] * perturbX + pc[1] * perturbY) / pc[2];

	return XFoam_Vector3D(perturbX, perturbY, z);
}

XFoam_Vector3D XFoam_Plane::nearestPoint(const XFoam_Vector3D& p) const
{
	return p - normal_ * signedDistance(p);
}

XFoam_Scalar XFoam_Plane::distance(const XFoam_Vector3D& p) const
{
	return XFoam_mag(signedDistance(p));
}

XFoam_Scalar XFoam_Plane::signedDistance(const XFoam_Vector3D& p) const
{
	return (p - point_) & normal_;
}

XFoam_Scalar XFoam_Plane::normalIntersect(
	const XFoam_Vector3D& pnt0,
	const XFoam_Vector3D& dir) const
{
	const XFoam_Scalar num = (point_ - pnt0) & normal_;
	const XFoam_Scalar den = dir & normal_;

	return XFoam_mag(den) > XFoam_mag(num) * XFoam_vSmall ? num / den : XFoam_vGreat;
}

XFoam_Scalar XFoam_Plane::lineIntersect(const XFoam_LinePoints& l) const
{
	return normalIntersect(l.start(), l.vec());
}

XFoam_Scalar XFoam_Plane::lineIntersect(const XFoam_LinePointRef& l) const
{
	return normalIntersect(l.start(), l.vec());
}

XFoam_Plane::ray XFoam_Plane::planeIntersect(const XFoam_Plane& plane2) const
{
	const XFoam_Vector3D& n1 = normal();
	const XFoam_Vector3D& n2 = plane2.normal();
	const XFoam_Vector3D& p1 = refPoint();
	const XFoam_Vector3D& p2 = plane2.refPoint();
	XFoam_Scalar n1p1 = n1 & p1;
	XFoam_Scalar n2p2 = n2 & p2;
	XFoam_Vector3D dir = n1 ^ n2;
	XFoam_Scalar magX = XFoam_mag(dir.x());
	XFoam_Scalar magY = XFoam_mag(dir.y());
	XFoam_Scalar magZ = XFoam_mag(dir.z());
	int iZero = 0;
	int i1 = 1;
	int i2 = 2;

	if (magX > magY)
	{
		if (magX > magZ)
		{
			iZero = 0;
			i1 = 1;
			i2 = 2;
		}
		else
		{
			iZero = 2;
			i1 = 0;
			i2 = 1;
		}
	}
	else
	{
		if (magY > magZ)
		{
			iZero = 1;
			i1 = 2;
			i2 = 0;
		}
		else
		{
			iZero = 2;
			i1 = 0;
			i2 = 1;
		}
	}
	XFoam_Vector3D pt(XFoam_Zero_v);
	pt[static_cast<XFoam_Size>(iZero)] = 0;
    pt[i1] = (n2[i2]*n1p1 - n1[i2]*n2p2) / (n1[i1]*n2[i2] - n2[i1]*n1[i2]);
    pt[i2] = (n2[i1]*n1p1 - n1[i1]*n2p2) / (n1[i2]*n2[i1] - n1[i1]*n2[i2]);

	return ray(pt, dir);
}

XFoam_Vector3D XFoam_Plane::planePlaneIntersect(
	const XFoam_Plane& plane2,
	const XFoam_Plane& plane3) const
{
	XFoam_FixedList<XFoam_Scalar, 4> coeffs1(planeCoeffs());
	XFoam_FixedList<XFoam_Scalar, 4> coeffs2(plane2.planeCoeffs());
	XFoam_FixedList<XFoam_Scalar, 4> coeffs3(plane3.planeCoeffs());

	const XFoam_TensorD A(
		coeffs1[0],
		coeffs1[1],
		coeffs1[2],
		coeffs2[0],
		coeffs2[1],
		coeffs2[2],
		coeffs3[0],
		coeffs3[1],
		coeffs3[2]);

	const XFoam_Scalar d = XFoam_det(A);
	if (std::fabs(static_cast<double>(d)) < static_cast<double>(XFoam_sqr(XFoam_rootSmall)))
	{
		return XFoam_Vector3D(XFoam_Zero_v);
	}

	const XFoam_Vector3D rhs(-coeffs1[3], -coeffs2[3], -coeffs3[3]);
	return XFoam_inv(A, d) & rhs;
}

XFoam_Plane::side XFoam_Plane::sideOfPlane(const XFoam_Vector3D& p) const
{
	const XFoam_Scalar angle((p - point_) & normal_);

	return (angle < 0 ? FLIP : NORMAL);
}

XFoam_Vector3D XFoam_Plane::mirror(const XFoam_Vector3D& p) const
{
	const XFoam_Vector3D mirroredPtDir = p - nearestPoint(p);

	if ((normal() & mirroredPtDir) > 0)
	{
		return p - 2.0 * distance(p) * normal();
	}

	return p + 2.0 * distance(p) * normal();
}
