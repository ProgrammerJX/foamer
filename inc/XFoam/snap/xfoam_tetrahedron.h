#ifndef XFoam_Tetrahedron_H_
#define XFoam_Tetrahedron_H_

// 四面体图元，对齐 OpenFOAM Foam::tetrahedron（tetrahedron.H / tetrahedronI.H）。
// 面顶点顺序与 OF tet单元一致；未移植：Istream 构造与 OF token 流输出。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/snap/xfoam_barycentric.h"
#include "XFoam/snap/xfoam_triangle.h"
#include "XFoam/snap/xfoam_pointhit.h"

template<class Point, class PointRef>
class XFoam_Tetrahedron
{
	PointRef a_;
	PointRef b_;
	PointRef c_;
	PointRef d_;

public:
	enum
	{
		nVertices = 4,
		nEdges = 6
	};

	XFoam_Tetrahedron() = default;

	XFoam_Tetrahedron(const Point& a, const Point& b, const Point& c, const Point& d)
		: a_(a)
		, b_(b)
		, c_(c)
		, d_(d)
	{}

	XFoam_Tetrahedron(const XFoam_UList<Point>& points, const XFoam_FixedList<XFoam_Label, 4>& indices)
		: a_(points[indices[0]])
		, b_(points[indices[1]])
		, c_(points[indices[2]])
		, d_(points[indices[3]])
	{}

	const Point& a() const { return a_; }
	const Point& b() const { return b_; }
	const Point& c() const { return c_; }
	const Point& d() const { return d_; }

	// 面顺序与 OF tetrahedron::tri / tet形状一致
	XFoam_Triangle<Point, PointRef> tri(const XFoam_Label facei) const
	{
		if (facei == 0)
		{
			return XFoam_Triangle<Point, PointRef>(b_, c_, d_);
		}
		if (facei == 1)
		{
			return XFoam_Triangle<Point, PointRef>(a_, d_, c_);
		}
		if (facei == 2)
		{
			return XFoam_Triangle<Point, PointRef>(a_, b_, d_);
		}
		if (facei == 3)
		{
			return XFoam_Triangle<Point, PointRef>(a_, c_, b_);
		}
		throw XFoam_Error(
			XFoam_String("XFoam_Tetrahedron::tri: face index out of range 0-3, facei=")
				+ std::to_string(facei));
	}

	Point Sa() const { return XFoam_Triangle<Point, PointRef>(b_, c_, d_).area(); }

	Point Sb() const { return XFoam_Triangle<Point, PointRef>(a_, d_, c_).area(); }

	Point Sc() const { return XFoam_Triangle<Point, PointRef>(a_, b_, d_).area(); }

	Point Sd() const { return XFoam_Triangle<Point, PointRef>(a_, c_, b_).area(); }

	Point centre() const { return (a_ + b_ + c_ + d_) * (1.0 / 4.0); }

	XFoam_Scalar mag() const
	{
		return static_cast<XFoam_Scalar>(
			(1.0 / 6.0) * static_cast<double>(((b_ - a_) ^ (c_ - a_)) & (d_ - a_)));
	}

	XFoam_Tuple2<Point, XFoam_Scalar> circumSphere() const
	{
		// 与 OpenFOAM tetrahedronI.H 相同记号：a=b_-a_, b=c_-a_, c=d_-a_
		const Point edgeAB = b_ - a_;
		const Point edgeAC = c_ - a_;
		const Point edgeAD = d_ - a_;

		const Point crossBA = edgeAC ^ edgeAB;
		const Point crossCA = edgeAD ^ edgeAB;

		const XFoam_Scalar lambda =
			static_cast<XFoam_Scalar>(edgeAD.magSqr()) - (edgeAB & edgeAD);
		const XFoam_Scalar mu =
			static_cast<XFoam_Scalar>(edgeAC.magSqr()) - (edgeAB & edgeAC);

		const Point num = crossBA * lambda - crossCA * mu;
		const XFoam_Scalar denom = edgeAD & crossBA;

		if (XFoam_mag(denom) < XFoam_rootVSmall)
		{
			static const XFoam_Scalar sqrt3 = static_cast<XFoam_Scalar>(std::sqrt(3.0));
			return XFoam_Tuple2<Point, XFoam_Scalar>(
				Point(XFoam_great, XFoam_great, XFoam_great), sqrt3 * XFoam_great);
		}

		const Point v = (edgeAB + num / denom) * (1.0 / 2.0);
		return XFoam_Tuple2<Point, XFoam_Scalar>(
			a_ + v, static_cast<XFoam_Scalar>(v.mag()));
	}

	XFoam_Scalar quality() const
	{
		const XFoam_Scalar r = circumSphere().second();
		static const XFoam_Scalar sqrt3 = static_cast<XFoam_Scalar>(std::sqrt(3.0));
		return mag()
			/ ((8.0 / 27.0) * sqrt3 * XFoam_pow(XFoam_min(r, XFoam_great), 3) + XFoam_rootSmall);
	}

	Point randomPoint(XFoam_RandomGenerator& rndGen) const
	{
		const XFoam_Scalar w0 =
			-static_cast<XFoam_Scalar>(std::log(std::max(static_cast<double>(rndGen.scalar01()), 1e-30)));
		const XFoam_Scalar w1 =
			-static_cast<XFoam_Scalar>(std::log(std::max(static_cast<double>(rndGen.scalar01()), 1e-30)));
		const XFoam_Scalar w2 =
			-static_cast<XFoam_Scalar>(std::log(std::max(static_cast<double>(rndGen.scalar01()), 1e-30)));
		const XFoam_Scalar w3 =
			-static_cast<XFoam_Scalar>(std::log(std::max(static_cast<double>(rndGen.scalar01()), 1e-30)));
		const XFoam_Scalar s = w0 + w1 + w2 + w3;
		const XFoam_Barycentric br(w0 / s, w1 / s, w2 / s, w3 / s);
		return barycentricToPoint(br);
	}

	Point barycentricToPoint(const XFoam_Barycentric& bary) const
	{
		return bary[0] * a_ + bary[1] * b_ + bary[2] * c_ + bary[3] * d_;
	}

	XFoam_Barycentric pointToBarycentric(const Point& pt) const
	{
		XFoam_Barycentric bary;
		pointToBarycentric(pt, bary);
		return bary;
	}

	XFoam_Scalar pointToBarycentric(const Point& pt, XFoam_Barycentric& bary) const
	{
		const Point e0 = a_ - d_;
		const Point e1 = b_ - d_;
		const Point e2 = c_ - d_;

		const XFoam_Tensor<XFoam_Scalar> t(
			e0.x(),
			e1.x(),
			e2.x(),
			e0.y(),
			e1.y(),
			e2.y(),
			e0.z(),
			e1.z(),
			e2.z());

		const XFoam_Scalar detT = XFoam_det(t);

		if (XFoam_mag(detT) < XFoam_small)
		{
			bary = XFoam_Barycentric(0.25, 0.25, 0.25, 0.25);
			return detT;
		}

		const Point res = XFoam_inv(t, detT) & (pt - d_);

		bary[0] = res.x();
		bary[1] = res.y();
		bary[2] = res.z();
		bary[3] = 1.0 - (res.x() + res.y() + res.z());

		return detT;
	}

	XFoam_PointHit<Point> nearestPoint(const Point& p) const
	{
		Point closestPt = p;
		XFoam_Scalar minOutsideDistance = XFoam_vGreat;
		bool insideAll = true;

		if (((p - b_) & Sa()) >= 0)
		{
			const XFoam_PointHit<Point> info =
				XFoam_Triangle<Point, PointRef>(b_, c_, d_).nearestPoint(p);
			insideAll = false;
			if (info.distance() < minOutsideDistance)
			{
				closestPt = info.rawPoint();
				minOutsideDistance = info.distance();
			}
		}

		if (((p - a_) & Sb()) >= 0)
		{
			const XFoam_PointHit<Point> info =
				XFoam_Triangle<Point, PointRef>(a_, d_, c_).nearestPoint(p);
			insideAll = false;
			if (info.distance() < minOutsideDistance)
			{
				closestPt = info.rawPoint();
				minOutsideDistance = info.distance();
			}
		}

		if (((p - a_) & Sc()) >= 0)
		{
			const XFoam_PointHit<Point> info =
				XFoam_Triangle<Point, PointRef>(a_, b_, d_).nearestPoint(p);
			insideAll = false;
			if (info.distance() < minOutsideDistance)
			{
				closestPt = info.rawPoint();
				minOutsideDistance = info.distance();
			}
		}

		if (((p - a_) & Sd()) >= 0)
		{
			const XFoam_PointHit<Point> info =
				XFoam_Triangle<Point, PointRef>(a_, c_, b_).nearestPoint(p);
			insideAll = false;
			if (info.distance() < minOutsideDistance)
			{
				closestPt = info.rawPoint();
				minOutsideDistance = info.distance();
			}
		}

		if (insideAll)
		{
			minOutsideDistance = 0;
		}

		return XFoam_PointHit<Point>(insideAll, closestPt, minOutsideDistance, !insideAll);
	}

	bool inside(const Point& pt) const
	{
		Point n;

		{
			const Point& basePt = b_;
			n = Sa();
			const XFoam_Scalar mn = static_cast<XFoam_Scalar>(n.mag());
			n /= mn + XFoam_vSmall;
			if (((pt - basePt) & n) > XFoam_small)
			{
				return false;
			}
		}

		{
			const Point& basePt = c_;
			n = Sb();
			const XFoam_Scalar mn = static_cast<XFoam_Scalar>(n.mag());
			n /= mn + XFoam_vSmall;
			if (((pt - basePt) & n) > XFoam_small)
			{
				return false;
			}
		}

		{
			const Point& basePt = b_;
			n = Sc();
			const XFoam_Scalar mn = static_cast<XFoam_Scalar>(n.mag());
			n /= mn + XFoam_vSmall;
			if (((pt - basePt) & n) > XFoam_small)
			{
				return false;
			}
		}

		{
			const Point& basePt = b_;
			n = Sd();
			const XFoam_Scalar mn = static_cast<XFoam_Scalar>(n.mag());
			n /= mn + XFoam_vSmall;
			if (((pt - basePt) & n) > XFoam_small)
			{
				return false;
			}
		}

		return true;
	}

	XFoam_BoundBox bounds() const
	{
		const Point lo(
			XFoam_min(XFoam_min(a().x(), b().x()), XFoam_min(c().x(), d().x())),
			XFoam_min(XFoam_min(a().y(), b().y()), XFoam_min(c().y(), d().y())),
			XFoam_min(XFoam_min(a().z(), b().z()), XFoam_min(c().z(), d().z())));
		const Point hi(
			XFoam_max(XFoam_max(a().x(), b().x()), XFoam_max(c().x(), d().x())),
			XFoam_max(XFoam_max(a().y(), b().y()), XFoam_max(c().y(), d().y())),
			XFoam_max(XFoam_max(a().z(), b().z()), XFoam_max(c().z(), d().z())));
		return XFoam_BoundBox(lo, hi);
	}
};

typedef XFoam_Tetrahedron<XFoam_Vector3D, XFoam_Vector3D> XFoam_TetrahedronPoints;
typedef XFoam_Tetrahedron<XFoam_Vector3D, const XFoam_Vector3D&> XFoam_TetrahedronPointRef;

#endif
