#ifndef XFoam_pointHit_H_
#define XFoam_pointHit_H_

// 对齐 OpenFOAM meshes/primitiveShapes/objectHit/pointHit.H：描述点与几何对象求交/最近点的结果。

#include "XFoam/utilities/xfoam_common.h"

template<class PointType>
class XFoam_PointHit
{
	PointType point_;
	XFoam_Scalar distance_;
	bool hit_;
	bool eligibleMiss_;

public:
	typedef PointType point_type;

	XFoam_PointHit()
		: point_{}
		, distance_(XFoam_great)
		, hit_(false)
		, eligibleMiss_(false)
	{}

	explicit XFoam_PointHit(const point_type& p)
		: point_(p)
		, distance_(XFoam_great)
		, hit_(false)
		, eligibleMiss_(false)
	{}

	XFoam_PointHit(bool hit, const point_type& p, XFoam_Scalar dist, bool eligibleMiss = false)
		: point_(p)
		, distance_(dist)
		, hit_(hit)
		, eligibleMiss_(eligibleMiss)
	{}

	bool hit() const noexcept { return hit_; }

	bool eligibleMiss() const noexcept { return eligibleMiss_; }

	const point_type& point() const noexcept { return point_; }

	XFoam_Scalar distance() const noexcept { return distance_; }

	const point_type& hitPoint() const
	{
		if (!hit_)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_PointHit::hitPoint: requested a hit point, but it was not hit"));
		}
		return point_;
	}

	const point_type& missPoint() const
	{
		if (hit_)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_PointHit::missPoint: requested a miss point, but it was hit"));
		}
		return point_;
	}

	const point_type& rawPoint() const noexcept { return point_; }

	void setHit() noexcept
	{
		hit_ = true;
		eligibleMiss_ = false;
	}

	void setMiss(bool eligible) noexcept
	{
		hit_ = false;
		eligibleMiss_ = eligible;
	}

	void setPoint(const point_type& p) { point_ = p; }

	void setDistance(XFoam_Scalar d) noexcept { distance_ = d; }

	void hitPoint(const point_type& p)
	{
		point_ = p;
		hit_ = true;
		eligibleMiss_ = false;
	}

	bool operator<(const XFoam_PointHit<PointType>& rhs) const noexcept
	{
		return distance_ < rhs.distance_;
	}
};

template<class PointType>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_PointHit<PointType>& pHit)
{
	return os << pHit.hit() << ' ' << pHit.point() << ' ' << pHit.distance() << ' '
		<< pHit.eligibleMiss();
}

typedef XFoam_PointHit<XFoam_Vector3D> XFoam_pointHit;

#endif
