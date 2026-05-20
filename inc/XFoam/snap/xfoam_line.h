#ifndef XFoam_Line_H_
#define XFoam_Line_H_

// 线段图元，结构与 OpenFOAM meshes/primitiveShapes/line 对齐（本文件合并 line.H / lineI.H；无 I/O）。
// OpenFOAM 无 line.C（头内联）。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/snap/xfoam_pointhit.h"

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// Class XFoam_Line Declaration
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

template<class Point, class PointRef>
class XFoam_Line
{
	PointRef a_;
	PointRef b_;

public:
	// Constructors

	XFoam_Line() = default;

	inline XFoam_Line(const Point& start, const Point& end)
		: a_(start)
		, b_(end)
	{}

	inline XFoam_Line(
		const XFoam_UList<Point>& points,
		const XFoam_FixedList<XFoam_Label, 2>& indices)
		: a_(points[indices[0]])
		, b_(points[indices[1]])
	{}

	// Access

	inline PointRef start() const { return a_; }
	inline PointRef end() const { return b_; }

	// Properties

	inline Point centre() const { return 0.5 * (a_ + b_); }

	inline XFoam_Scalar mag() const { return static_cast<XFoam_Scalar>(vec().mag()); }

	inline Point vec() const { return b_ - a_; }

	XFoam_PointHit<Point> nearestDist(const Point& p) const;

	template<class EdgePointRef>
	XFoam_Scalar nearestDist(
		const XFoam_Line<Point, EdgePointRef>& edge,
		Point& thisPt,
		Point& edgePt) const;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
//   Member Functions — nearestDist (lineI.H)
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

template<class Point, class PointRef>
inline XFoam_PointHit<Point> XFoam_Line<Point, PointRef>::nearestDist(const Point& p) const
{
	Point v = vec();

	Point w(p - a_);

	XFoam_Scalar c1 = v & w;

	if (c1 <= 0)
	{
		return XFoam_PointHit<Point>(false, a_, static_cast<XFoam_Scalar>((p - a_).mag()), true);
	}

	XFoam_Scalar c2 = v & v;

	if (c2 <= c1)
	{
		return XFoam_PointHit<Point>(false, b_, static_cast<XFoam_Scalar>((p - b_).mag()), true);
	}

	XFoam_Scalar b = c1 / c2;

	Point pb(a_ + b * v);

	return XFoam_PointHit<Point>(true, pb, static_cast<XFoam_Scalar>((p - pb).mag()), false);
}

template<class Point, class PointRef>
template<class EdgePointRef>
inline XFoam_Scalar XFoam_Line<Point, PointRef>::nearestDist(
	const XFoam_Line<Point, EdgePointRef>& edge,
	Point& thisPt,
	Point& edgePt) const
{
	Point a(end() - start());
	Point b(edge.end() - edge.start());
	Point c(edge.start() - start());

	Point crossab = a ^ b;
	XFoam_Scalar magCrossSqr = crossab.magSqr();

	if (magCrossSqr > XFoam_vSmall)
	{
		XFoam_Scalar s = ((c ^ b) & crossab) / magCrossSqr;
		XFoam_Scalar t = ((c ^ a) & crossab) / magCrossSqr;

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
		{
			thisPt = start() + a * s;
			edgePt = edge.start() + b * t;
		}
		else
		{
			XFoam_PointHit<Point> this0(nearestDist(edge.start()));
			XFoam_PointHit<Point> this1(nearestDist(edge.end()));
			XFoam_Scalar thisDist = XFoam_min(this0.distance(), this1.distance());

			XFoam_PointHit<Point> edge0(edge.nearestDist(start()));
			XFoam_PointHit<Point> edge1(edge.nearestDist(end()));
			XFoam_Scalar edgeDist = XFoam_min(edge0.distance(), edge1.distance());

			if (thisDist < edgeDist)
			{
				if (this0.distance() < this1.distance())
				{
					thisPt = this0.rawPoint();
					edgePt = edge.start();
				}
				else
				{
					thisPt = this1.rawPoint();
					edgePt = edge.end();
				}
			}
			else
			{
				if (edge0.distance() < edge1.distance())
				{
					thisPt = start();
					edgePt = edge0.rawPoint();
				}
				else
				{
					thisPt = end();
					edgePt = edge1.rawPoint();
				}
			}
		}
	}
	else
	{
		XFoam_Scalar edge0 = edge.start() & a;
		XFoam_Scalar edge1 = edge.end() & a;
		bool edgeOrder = edge0 < edge1;

		XFoam_Scalar minEdge = (edgeOrder ? edge0 : edge1);
		XFoam_Scalar maxEdge = (edgeOrder ? edge1 : edge0);
		const Point& minEdgePt = (edgeOrder ? edge.start() : edge.end());
		const Point& maxEdgePt = (edgeOrder ? edge.end() : edge.start());

		XFoam_Scalar this0 = start() & a;
		XFoam_Scalar this1 = end() & a;
		bool thisOrder = this0 < this1;

		XFoam_Scalar minThis = XFoam_min(this0, this1);
		XFoam_Scalar maxThis = XFoam_max(this1, this0);
		const Point& minThisPt = (thisOrder ? start() : end());
		const Point& maxThisPt = (thisOrder ? end() : start());

		if (maxEdge < minThis)
		{
			edgePt = maxEdgePt;
			thisPt = minThisPt;
		}
		else if (maxEdge < maxThis)
		{
			edgePt = maxEdgePt;
			thisPt = nearestDist(edgePt).rawPoint();
		}
		else
		{
			if (minEdge < minThis)
			{
				thisPt = minThisPt;
				edgePt = edge.nearestDist(thisPt).rawPoint();
			}
			else if (minEdge < maxThis)
			{
				edgePt = minEdgePt;
				thisPt = nearestDist(edgePt).rawPoint();
			}
			else
			{
				edgePt = minEdgePt;
				thisPt = maxThisPt;
			}
		}
	}

	return static_cast<XFoam_Scalar>((thisPt - edgePt).mag());
}

typedef XFoam_Line<XFoam_Vector3D, XFoam_Vector3D> XFoam_LinePoints;
typedef XFoam_Line<XFoam_Vector3D, const XFoam_Vector3D&> XFoam_LinePointRef;

#endif
