#ifndef XFoam_Triangle_H_
#define XFoam_Triangle_H_

// 三角形图元，对齐 OpenFOAM Foam::triangle（template<class Point, class PointRef>）。
// 未移植：inertia、randomPoint 等。

#include "XFoam/primitive/xfoam_intersection.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/primitive/xfoam_barycentric.h"
#include "XFoam/primitive/xfoam_pointhit.h"

template<class Point, class PointRef>
class XFoam_Triangle
{
	PointRef a_;
	PointRef b_;
	PointRef c_;

public:
	enum ProxType
	{
		proxNone = 0,
		proxPoint = 1,
		proxEdge = 2
	};

	XFoam_Triangle() = default;

	XFoam_Triangle(const Point& a, const Point& b, const Point& c)
		: a_(a)
		, b_(b)
		, c_(c)
	{}

	XFoam_Triangle(const XFoam_UList<Point>& points, const XFoam_FixedList<XFoam_Label, 3>& indices)
		: a_(points[indices[0]])
		, b_(points[indices[1]])
		, c_(points[indices[2]])
	{}

	const Point& a() const { return a_; }
	const Point& b() const { return b_; }
	const Point& c() const { return c_; }

	Point centre() const { return (a_ + b_ + c_) * (1.0 / 3.0); }

	Point area() const { return 0.5 * ((b_ - a_) ^ (c_ - a_)); }

	XFoam_Scalar mag() const { return static_cast<XFoam_Scalar>(area().mag()); }

	Point normal() const
	{
		const Point ar = area();
		const XFoam_Scalar m = static_cast<XFoam_Scalar>(ar.mag());
		return m > 0 ? ar / m : Point(XFoam_Zero_v);
	}

	XFoam_Tuple2<Point, XFoam_Scalar> circumCircle() const
	{
		const XFoam_Scalar d1 = (c_ - a_) & (b_ - a_);
		const XFoam_Scalar d2 = -(c_ - b_) & (b_ - a_);
		const XFoam_Scalar d3 = (c_ - a_) & (c_ - b_);

		const XFoam_Scalar c1 = d2 * d3;
		const XFoam_Scalar c2 = d3 * d1;
		const XFoam_Scalar c3 = d1 * d2;

		const XFoam_Scalar denom = c1 + c2 + c3;

		if (std::fabs(static_cast<double>(denom)) < static_cast<double>(XFoam_rootSmall))
		{
			static const XFoam_Scalar sqrt3 = static_cast<XFoam_Scalar>(std::sqrt(3.0));
			return XFoam_Tuple2<Point, XFoam_Scalar>(
				Point(XFoam_great, XFoam_great, XFoam_great), sqrt3 * XFoam_great);
		}

		const Point centre =
			((c2 + c3) * a_ + (c3 + c1) * b_ + (c1 + c2) * c_) / (2 * denom);

		const XFoam_Scalar inside = XFoam_max(
			static_cast<XFoam_Scalar>(0.0),
			(d1 + d2) * (d2 + d3) * (d3 + d1) / (4 * denom));
		const XFoam_Scalar radius = static_cast<XFoam_Scalar>(std::sqrt(static_cast<double>(inside)));

		return XFoam_Tuple2<Point, XFoam_Scalar>(centre, radius);
	}

	XFoam_Scalar quality() const
	{
		const XFoam_Scalar r = circumCircle().second();
		static const XFoam_Scalar sqrt3 = static_cast<XFoam_Scalar>(std::sqrt(3.0));
		return mag() / (0.75 * sqrt3 * XFoam_sqr(XFoam_min(r, XFoam_great)) + XFoam_rootSmall);
	}

	template<class PR2>
	XFoam_Scalar sweptVol(const XFoam_Triangle<Point, PR2>& t) const
	{
		return (1.0 / 12.0)
			* (((t.a() - a_) & ((b_ - a_) ^ (c_ - a_))) + ((t.b() - b_) & ((c_ - b_) ^ (t.a() - b_)))
				+ ((c_ - t.c()) & ((t.b() - t.c()) ^ (t.a() - t.c()))) + ((t.a() - a_) & ((b_ - a_) ^ (c_ - a_)))
				+ ((b_ - t.b()) & ((t.a() - t.b()) ^ (t.c() - t.b())))
				+ ((c_ - t.c()) & ((b_ - t.c()) ^ (t.a() - t.c()))));
	}

	Point barycentricToPoint(const XFoam_Barycentric2D& bary) const
	{
		return bary[0] * a_ + bary[1] * b_ + bary[2] * c_;
	}

	XFoam_Barycentric2D pointToBarycentric(const Point& pt) const
	{
		XFoam_Barycentric2D bary;
		pointToBarycentric(pt, bary);
		return bary;
	}

	XFoam_Scalar pointToBarycentric(const Point& pt, XFoam_Barycentric2D& bary) const
	{
		const Point v0 = b_ - a_;
		const Point v1 = c_ - a_;
		const Point v2 = pt - a_;

		const XFoam_Scalar d00 = v0 & v0;
		const XFoam_Scalar d01 = v0 & v1;
		const XFoam_Scalar d11 = v1 & v1;
		const XFoam_Scalar d20 = v2 & v0;
		const XFoam_Scalar d21 = v2 & v1;

		const XFoam_Scalar denom = d00 * d11 - d01 * d01;

		if (std::fabs(static_cast<double>(denom)) < static_cast<double>(XFoam_small))
		{
			bary = XFoam_Barycentric2D(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
			return denom;
		}

		bary[1] = (d11 * d20 - d01 * d21) / denom;
		bary[2] = (d00 * d21 - d01 * d20) / denom;
		bary[0] = 1.0 - bary[1] - bary[2];

		return denom;
	}

	XFoam_PointHit<Point> intersection(
		const Point& orig,
		const Point& dir,
		const XFoam_Intersection::algorithm alg,
		const XFoam_Scalar tol = 0.0) const
	{
		const Point edge1 = b_ - a_;
		const Point edge2 = c_ - a_;

		const Point pVec = dir ^ edge2;

		const XFoam_Scalar det = edge1 & pVec;

		XFoam_PointHit<Point> hitResult(false, Point(XFoam_Zero_v), XFoam_great, false);

		if (alg == XFoam_Intersection::algorithm::visible)
		{
			if (det < XFoam_rootSmall)
			{
				return hitResult;
			}
		}
		else if (
			alg == XFoam_Intersection::algorithm::halfRay
			|| alg == XFoam_Intersection::algorithm::fullRay)
		{
			if (det > -XFoam_rootSmall && det < XFoam_rootSmall)
			{
				return hitResult;
			}
		}

		const XFoam_Scalar inv_det = 1.0 / det;

		const Point tVec = orig - a_;

		const XFoam_Scalar u = (tVec & pVec) * inv_det;

		if (u < -tol || u > 1.0 + tol)
		{
			return hitResult;
		}

		const Point qVec = tVec ^ edge1;

		const XFoam_Scalar v = (dir & qVec) * inv_det;

		if (v < -tol || u + v > 1.0 + tol)
		{
			return hitResult;
		}

		const XFoam_Scalar t = (edge2 & qVec) * inv_det;

		if (alg == XFoam_Intersection::algorithm::halfRay && t < -tol)
		{
			return hitResult;
		}

		hitResult.setHit();
		hitResult.setPoint(a_ + edge1 * u + edge2 * v);
		hitResult.setDistance(t);

		return hitResult;
	}

	XFoam_PointHit<Point> ray(
		const Point& p,
		const Point& q,
		const XFoam_Intersection::algorithm alg = XFoam_Intersection::algorithm::fullRay,
		const XFoam_Intersection::direction dir = XFoam_Intersection::direction::vector) const
	{
		const Point E0 = b_ - a_;
		const Point E1 = c_ - a_;

		XFoam_PointHit<Point> inter(p);

		Point n(0.5 * (E0 ^ E1));
		const XFoam_Scalar magArea = static_cast<XFoam_Scalar>(n.mag());

		if (magArea < XFoam_small)
		{
			inter.setMiss(false);
			inter.setPoint(a_);
			inter.setDistance(static_cast<XFoam_Scalar>((a_ - p).mag()));
			return inter;
		}

		XFoam_Scalar qmag = static_cast<XFoam_Scalar>(q.mag());
		if (qmag < XFoam_small)
		{
			inter.setMiss(false);
			inter.setPoint(a_);
			inter.setDistance(static_cast<XFoam_Scalar>((a_ - p).mag()));
			return inter;
		}

		Point q1 = q / qmag;

		if (dir == XFoam_Intersection::direction::contactSphere)
		{
			n = n / magArea;
			return ray(p, q1 - n, alg, XFoam_Intersection::direction::vector);
		}

		Point pInter;
		bool hit = false;
		{
			XFoam_PointHit<Point> fastInter = intersection(p, q1, XFoam_Intersection::algorithm::fullRay);
			hit = fastInter.hit();

			if (hit)
			{
				pInter = fastInter.rawPoint();
			}
			else
			{
				const Point v = a_ - p;
				pInter = p + q1 * (q1 & v);
			}
		}

		const XFoam_Scalar dist = q1 & (pInter - p);

		const XFoam_Scalar planarPointTol =
			XFoam_min(
				XFoam_min(static_cast<XFoam_Scalar>((E0).mag()), static_cast<XFoam_Scalar>((E1).mag())),
				static_cast<XFoam_Scalar>((c_ - b_).mag()))
			* XFoam_Intersection::planarTol();

		const XFoam_Scalar ndota = q1 & area();
		bool eligible = (alg == XFoam_Intersection::algorithm::fullRay)
			|| (alg == XFoam_Intersection::algorithm::halfRay && dist > -planarPointTol)
			|| (alg == XFoam_Intersection::algorithm::visible && ndota < -XFoam_small);

		if (hit && eligible)
		{
			inter.setHit();
			inter.setPoint(pInter);
			inter.setDistance(dist);
		}
		else
		{
			inter.setMiss(eligible);
			inter.setPoint(nearestPoint(p).rawPoint());
			inter.setDistance(static_cast<XFoam_Scalar>((pInter - p).mag()));
		}

		return inter;
	}

	XFoam_PointHit<Point> nearestPointClassify(
		const Point& p,
		XFoam_Label& nearType,
		XFoam_Label& nearLabel) const
	{
		const Point ab = b_ - a_;
		const Point ac = c_ - a_;
		const Point ap = p - a_;

		XFoam_Scalar d1 = ab & ap;
		XFoam_Scalar d2 = ac & ap;

		if (d1 <= 0.0 && d2 <= 0.0)
		{
			nearType = proxPoint;
			nearLabel = 0;
			return XFoam_PointHit<Point>(false, a_, static_cast<XFoam_Scalar>((a_ - p).mag()), true);
		}

		const Point bp = p - b_;
		XFoam_Scalar d3 = ab & bp;
		XFoam_Scalar d4 = ac & bp;

		if (d3 >= 0.0 && d4 <= d3)
		{
			nearType = proxPoint;
			nearLabel = 1;
			return XFoam_PointHit<Point>(false, b_, static_cast<XFoam_Scalar>((b_ - p).mag()), true);
		}

		XFoam_Scalar vc = d1 * d4 - d3 * d2;

		if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
		{
			if ((d1 - d3) < XFoam_rootSmall)
			{
				nearType = proxPoint;
				nearLabel = 0;
				return XFoam_PointHit<Point>(false, a_, static_cast<XFoam_Scalar>((a_ - p).mag()), true);
			}

			const XFoam_Scalar vv = d1 / (d1 - d3);
			const Point nearPt = a_ + ab * vv;
			nearType = proxEdge;
			nearLabel = 0;
			return XFoam_PointHit<Point>(
				false, nearPt, static_cast<XFoam_Scalar>((nearPt - p).mag()), true);
		}

		const Point cp = p - c_;
		XFoam_Scalar d5 = ab & cp;
		XFoam_Scalar d6 = ac & cp;

		if (d6 >= 0.0 && d5 <= d6)
		{
			nearType = proxPoint;
			nearLabel = 2;
			return XFoam_PointHit<Point>(false, c_, static_cast<XFoam_Scalar>((c_ - p).mag()), true);
		}

		XFoam_Scalar vb = d5 * d2 - d1 * d6;

		if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
		{
			if ((d2 - d6) < XFoam_rootSmall)
			{
				nearType = proxPoint;
				nearLabel = 0;
				return XFoam_PointHit<Point>(false, a_, static_cast<XFoam_Scalar>((a_ - p).mag()), true);
			}

			const XFoam_Scalar w = d2 / (d2 - d6);
			const Point nearPt = a_ + ac * w;
			nearType = proxEdge;
			nearLabel = 2;
			return XFoam_PointHit<Point>(
				false, nearPt, static_cast<XFoam_Scalar>((nearPt - p).mag()), true);
		}

		XFoam_Scalar va = d3 * d6 - d5 * d4;

		if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
		{
			if (((d4 - d3) + (d5 - d6)) < XFoam_rootSmall)
			{
				nearType = proxPoint;
				nearLabel = 1;
				return XFoam_PointHit<Point>(false, b_, static_cast<XFoam_Scalar>((b_ - p).mag()), true);
			}

			const XFoam_Scalar w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			const Point nearPt = b_ + (c_ - b_) * w;
			nearType = proxEdge;
			nearLabel = 1;
			return XFoam_PointHit<Point>(
				false, nearPt, static_cast<XFoam_Scalar>((nearPt - p).mag()), true);
		}

		if ((va + vb + vc) < XFoam_rootSmall)
		{
			const Point nearPt = centre();
			nearType = proxNone;
			nearLabel = -1;
			return XFoam_PointHit<Point>(
				true, nearPt, static_cast<XFoam_Scalar>((nearPt - p).mag()), false);
		}

		const XFoam_Scalar denom = 1.0 / (va + vb + vc);
		const XFoam_Scalar v = vb * denom;
		const XFoam_Scalar w = vc * denom;

		const Point nearPt = a_ + ab * v + ac * w;
		nearType = proxNone;
		nearLabel = -1;
		return XFoam_PointHit<Point>(true, nearPt, static_cast<XFoam_Scalar>((nearPt - p).mag()), false);
	}

	XFoam_PointHit<Point> nearestPoint(const Point& p) const
	{
		XFoam_Label nearType = -1;
		XFoam_Label nearLabel = -1;
		return nearestPointClassify(p, nearType, nearLabel);
	}

	bool classify(const Point& p, XFoam_Label& nearType, XFoam_Label& nearLabel) const
	{
		return nearestPointClassify(p, nearType, nearLabel).hit();
	}
};

typedef XFoam_Triangle<XFoam_Vector3D, XFoam_Vector3D> XFoam_TrianglePoints;
typedef XFoam_Triangle<XFoam_Vector3D, const XFoam_Vector3D&> XFoam_TrianglePointRef;

#endif
