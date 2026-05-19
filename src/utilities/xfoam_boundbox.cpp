#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_types.h"

namespace
{
inline XFoam_Vector3D minCmpt(const XFoam_Vector3D& a, const XFoam_Vector3D& b)
{
	return XFoam_Vector3D(
		std::min(a.x(), b.x()),
		std::min(a.y(), b.y()),
		std::min(a.z(), b.z()));
}

inline XFoam_Vector3D maxCmpt(const XFoam_Vector3D& a, const XFoam_Vector3D& b)
{
	return XFoam_Vector3D(
		std::max(a.x(), b.x()),
		std::max(a.y(), b.y()),
		std::max(a.z(), b.z()));
}
} // namespace

const XFoam_Scalar XFoam_BoundBox::great = XFoam_great;

const XFoam_BoundBox XFoam_BoundBox::greatBox(
	XFoam_Vector3D(-great, -great, -great),
	XFoam_Vector3D(great, great, great));

const XFoam_BoundBox XFoam_BoundBox::invertedBox(
	XFoam_Vector3D(great, great, great),
	XFoam_Vector3D(-great, -great, -great));

XFoam_BoundBox::XFoam_BoundBox()
	: min_(XFoam_Zero_v)
	, max_(XFoam_Zero_v)
{}

XFoam_BoundBox::XFoam_BoundBox(const XFoam_Vector3D& minPt, const XFoam_Vector3D& maxPt)
	: min_(minPt)
	, max_(maxPt)
{}

void XFoam_BoundBox::calculate(const XFoam_UList<XFoam_Vector3D>& points)
{
	if (points.empty())
	{
		min_ = XFoam_Vector3D(XFoam_Zero_v);
		max_ = XFoam_Vector3D(XFoam_Zero_v);
		return;
	}
	min_ = points[0];
	max_ = points[0];
	for (XFoam_Label i = 1; i < points.size(); ++i)
	{
		min_ = minCmpt(min_, points[i]);
		max_ = maxCmpt(max_, points[i]);
	}
}

XFoam_BoundBox::XFoam_BoundBox(const XFoam_UList<XFoam_Vector3D>& points)
	: min_(XFoam_Zero_v)
	, max_(XFoam_Zero_v)
{
	calculate(points);
}

XFoam_BoundBox::XFoam_BoundBox(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_UList<XFoam_Label>& indices)
	: min_(XFoam_Zero_v)
	, max_(XFoam_Zero_v)
{
	if (points.empty() || indices.empty())
	{
		return;
	}
	min_ = points[indices[0]];
	max_ = points[indices[0]];
	for (XFoam_Label i = 1; i < indices.size(); ++i)
	{
		min_ = minCmpt(min_, points[indices[i]]);
		max_ = maxCmpt(max_, points[indices[i]]);
	}
}

XFoam_Vector3D XFoam_BoundBox::midpoint() const
{
	return XFoam_Vector3D(
		0.5 * (max_.x() + min_.x()),
		0.5 * (max_.y() + min_.y()),
		0.5 * (max_.z() + min_.z()));
}

XFoam_Vector3D XFoam_BoundBox::span() const { return max_ - min_; }

XFoam_Scalar XFoam_BoundBox::mag() const { return static_cast<XFoam_Scalar>((max_ - min_).mag()); }

XFoam_Scalar XFoam_BoundBox::volume() const
{
	const XFoam_Vector3D s = span();
	return s.x() * s.y() * s.z();
}

XFoam_Scalar XFoam_BoundBox::minDim() const
{
	const XFoam_Vector3D s = span();
	return std::min(std::min(s.x(), s.y()), s.z());
}

XFoam_Scalar XFoam_BoundBox::maxDim() const
{
	const XFoam_Vector3D s = span();
	return std::max(std::max(s.x(), s.y()), s.z());
}

XFoam_Scalar XFoam_BoundBox::avgDim() const
{
	const XFoam_Vector3D s = span();
	return (s.x() + s.y() + s.z()) / 3.0;
}

XFoam_List<XFoam_Vector3D> XFoam_BoundBox::cornerPoints() const
{
	XFoam_List<XFoam_Vector3D> pt(8);
	pt[0] = min_;
	pt[1] = XFoam_Vector3D(max_.x(), min_.y(), min_.z());
	pt[2] = XFoam_Vector3D(max_.x(), max_.y(), min_.z());
	pt[3] = XFoam_Vector3D(min_.x(), max_.y(), min_.z());
	pt[4] = XFoam_Vector3D(min_.x(), min_.y(), max_.z());
	pt[5] = XFoam_Vector3D(max_.x(), min_.y(), max_.z());
	pt[6] = max_;
	pt[7] = XFoam_Vector3D(min_.x(), max_.y(), max_.z());
	return pt;
}

XFoam_List<XFoam_FixedList<XFoam_Label, 4>> XFoam_BoundBox::faces()
{
	XFoam_List<XFoam_FixedList<XFoam_Label, 4>> f(6);
	f[0] = XFoam_FixedList<XFoam_Label, 4>({0, 1, 2, 3});
	f[1] = XFoam_FixedList<XFoam_Label, 4>({2, 6, 7, 3});
	f[2] = XFoam_FixedList<XFoam_Label, 4>({0, 4, 5, 1});
	f[3] = XFoam_FixedList<XFoam_Label, 4>({4, 7, 6, 5});
	f[4] = XFoam_FixedList<XFoam_Label, 4>({3, 7, 4, 0});
	f[5] = XFoam_FixedList<XFoam_Label, 4>({1, 5, 6, 2});
	return f;
}

void XFoam_BoundBox::inflate(const XFoam_Scalar s)
{
	const XFoam_Scalar m = mag();
	const XFoam_Scalar h = s * m;
	const XFoam_Vector3D ext(h, h, h);
	min_ -= ext;
	max_ += ext;
}

bool XFoam_BoundBox::overlaps(const XFoam_BoundBox& bb) const
{
	return bb.max_.x() >= min_.x() && bb.min_.x() <= max_.x()
		&& bb.max_.y() >= min_.y() && bb.min_.y() <= max_.y()
		&& bb.max_.z() >= min_.z() && bb.min_.z() <= max_.z();
}

bool XFoam_BoundBox::overlaps(
	const XFoam_Vector3D& centre,
	const XFoam_Scalar radiusSqr) const
{
	XFoam_Scalar distSqr = 0;
	for (int dir = 0; dir < 3; ++dir)
	{
		const XFoam_Scalar c = centre[static_cast<XFoam_Size>(dir)];
		const XFoam_Scalar d0 = min_[static_cast<XFoam_Size>(dir)] - c;
		const XFoam_Scalar d1 = max_[static_cast<XFoam_Size>(dir)] - c;
		const bool inside = (d0 > 0) != (d1 > 0);
		if (!inside)
		{
			if (std::abs(d0) < std::abs(d1))
			{
				distSqr += d0 * d0;
			}
			else
			{
				distSqr += d1 * d1;
			}
		}
		if (distSqr > radiusSqr)
		{
			return false;
		}
	}
	return true;
}

bool XFoam_BoundBox::contains(const XFoam_Vector3D& pt) const
{
	return pt.x() >= min_.x() && pt.x() <= max_.x() && pt.y() >= min_.y() && pt.y() <= max_.y()
		&& pt.z() >= min_.z() && pt.z() <= max_.z();
}

bool XFoam_BoundBox::contains(const XFoam_BoundBox& bb) const
{
	return contains(bb.min()) && contains(bb.max());
}

bool XFoam_BoundBox::containsInside(const XFoam_Vector3D& pt) const
{
	return pt.x() > min_.x() && pt.x() < max_.x() && pt.y() > min_.y() && pt.y() < max_.y()
		&& pt.z() > min_.z() && pt.z() < max_.z();
}

bool XFoam_BoundBox::contains(const XFoam_UList<XFoam_Vector3D>& points) const
{
	if (points.empty())
	{
		return true;
	}
	for (XFoam_Label i = 0; i < points.size(); ++i)
	{
		if (!contains(points[i]))
		{
			return false;
		}
	}
	return true;
}

bool XFoam_BoundBox::contains(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_UList<XFoam_Label>& indices) const
{
	if (points.empty() || indices.empty())
	{
		return true;
	}
	for (XFoam_Label i = 0; i < indices.size(); ++i)
	{
		if (!contains(points[indices[i]]))
		{
			return false;
		}
	}
	return true;
}

bool XFoam_BoundBox::containsAny(const XFoam_UList<XFoam_Vector3D>& points) const
{
	if (points.empty())
	{
		return true;
	}
	for (XFoam_Label i = 0; i < points.size(); ++i)
	{
		if (contains(points[i]))
		{
			return true;
		}
	}
	return false;
}

bool XFoam_BoundBox::containsAny(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_UList<XFoam_Label>& indices) const
{
	if (points.empty() || indices.empty())
	{
		return true;
	}
	for (XFoam_Label i = 0; i < indices.size(); ++i)
	{
		if (contains(points[indices[i]]))
		{
			return true;
		}
	}
	return true;
}

XFoam_Vector3D XFoam_BoundBox::nearest(const XFoam_Vector3D& pt) const
{
	const XFoam_Scalar sx = std::max(std::min(pt.x(), max_.x()), min_.x());
	const XFoam_Scalar sy = std::max(std::min(pt.y(), max_.y()), min_.y());
	const XFoam_Scalar sz = std::max(std::min(pt.z(), max_.z()), min_.z());
	return XFoam_Vector3D(sx, sy, sz);
}

XFoam_API bool operator==(const XFoam_BoundBox& a, const XFoam_BoundBox& b)
{
	return a.min() == b.min() && a.max() == b.max();
}

XFoam_API bool operator!=(const XFoam_BoundBox& a, const XFoam_BoundBox& b)
{
	return !(a == b);
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BoundBox& bb)
{
	os << bb.min() << ' ' << bb.max();
	return os;
}

XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_BoundBox& bb)
{
	is >> bb.min() >> bb.max();
	return is;
}
