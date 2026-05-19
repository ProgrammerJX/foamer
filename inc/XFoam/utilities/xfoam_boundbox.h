#ifndef XFoam_BoundBox_H_
#define XFoam_BoundBox_H_

// 轴对齐包围盒，对齐 OpenFOAM Foam::boundBox（boundBox.H / boundBoxI.H / boundBox.C）。
// 点类型为 XFoam_Vector3D；无 MPI，不提供 doReduce 语义。

#include "XFoam/utilities/xfoam_list.h"
#include "XFoam/utilities/xfoam_vector.h"

class XFoam_API XFoam_BoundBox
{
	XFoam_Vector3D min_;
	XFoam_Vector3D max_;

	void calculate(const XFoam_UList<XFoam_Vector3D>& points);

public:
	// 与 OpenFOAM boundBox::great 同量级（此处用 XFoam_great）
	static const XFoam_Scalar great;
	static const XFoam_BoundBox greatBox;
	static const XFoam_BoundBox invertedBox;

	XFoam_BoundBox();
	XFoam_BoundBox(const XFoam_Vector3D& minPt, const XFoam_Vector3D& maxPt);

	explicit XFoam_BoundBox(const XFoam_UList<XFoam_Vector3D>& points);
	XFoam_BoundBox(
		const XFoam_UList<XFoam_Vector3D>& points,
		const XFoam_UList<XFoam_Label>& indices);

	template<unsigned Size>
	XFoam_BoundBox(
		const XFoam_UList<XFoam_Vector3D>& points,
		const XFoam_FixedList<XFoam_Label, Size>& indices);

	const XFoam_Vector3D& min() const { return min_; }
	const XFoam_Vector3D& max() const { return max_; }
	XFoam_Vector3D& min() { return min_; }
	XFoam_Vector3D& max() { return max_; }

	XFoam_Vector3D midpoint() const;
	XFoam_Vector3D span() const;
	XFoam_Scalar mag() const;
	XFoam_Scalar volume() const;
	XFoam_Scalar minDim() const;
	XFoam_Scalar maxDim() const;
	XFoam_Scalar avgDim() const;

	// 与 boundBox::points() 相同的 8 角点顺序（底面 z=min，再顶面 z=max）
	XFoam_List<XFoam_Vector3D> cornerPoints() const;

	// 与 boundBox::faces() 相同的 6 个四边形面（顶点为 cornerPoints 的下标）
	static XFoam_List<XFoam_FixedList<XFoam_Label, 4>> faces();

	void inflate(XFoam_Scalar factor);

	bool overlaps(const XFoam_BoundBox& bb) const;
	bool overlaps(const XFoam_Vector3D& centre, XFoam_Scalar radiusSqr) const;
	bool contains(const XFoam_Vector3D& pt) const;
	bool contains(const XFoam_BoundBox& bb) const;
	bool containsInside(const XFoam_Vector3D& pt) const;

	bool contains(const XFoam_UList<XFoam_Vector3D>& points) const;
	bool contains(
		const XFoam_UList<XFoam_Vector3D>& points,
		const XFoam_UList<XFoam_Label>& indices) const;

	template<unsigned Size>
	bool contains(
		const XFoam_UList<XFoam_Vector3D>& points,
		const XFoam_FixedList<XFoam_Label, Size>& indices) const;

	bool containsAny(const XFoam_UList<XFoam_Vector3D>& points) const;
	bool containsAny(
		const XFoam_UList<XFoam_Vector3D>& points,
		const XFoam_UList<XFoam_Label>& indices) const;

	template<unsigned Size>
	bool containsAny(
		const XFoam_UList<XFoam_Vector3D>& points,
		const XFoam_FixedList<XFoam_Label, Size>& indices) const;

	XFoam_Vector3D nearest(const XFoam_Vector3D& pt) const;
};

XFoam_API bool operator==(const XFoam_BoundBox& a, const XFoam_BoundBox& b);
XFoam_API bool operator!=(const XFoam_BoundBox& a, const XFoam_BoundBox& b);

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BoundBox& bb);
XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_BoundBox& bb);

template<unsigned Size>
inline XFoam_BoundBox::XFoam_BoundBox(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_FixedList<XFoam_Label, Size>& indices)
	: min_(XFoam_Zero_v)
	, max_(XFoam_Zero_v)
{
	if (points.empty())
	{
		return;
	}
	min_ = points[indices[0]];
	max_ = points[indices[0]];
	for (unsigned i = 1; i < Size; ++i)
	{
		const XFoam_Vector3D& p = points[indices[i]];
		min_.x() = std::min(min_.x(), p.x());
		min_.y() = std::min(min_.y(), p.y());
		min_.z() = std::min(min_.z(), p.z());
		max_.x() = std::max(max_.x(), p.x());
		max_.y() = std::max(max_.y(), p.y());
		max_.z() = std::max(max_.z(), p.z());
	}
}

template<unsigned Size>
inline bool XFoam_BoundBox::contains(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_FixedList<XFoam_Label, Size>& indices) const
{
	if (points.empty())
	{
		return false;
	}
	for (unsigned i = 0; i < Size; ++i)
	{
		if (!contains(points[indices[i]]))
		{
			return false;
		}
	}
	return true;
}

template<unsigned Size>
inline bool XFoam_BoundBox::containsAny(
	const XFoam_UList<XFoam_Vector3D>& points,
	const XFoam_FixedList<XFoam_Label, Size>& indices) const
{
	if (points.empty())
	{
		return false;
	}
	for (unsigned i = 0; i < Size; ++i)
	{
		if (contains(points[indices[i]]))
		{
			return true;
		}
	}
	return false;
}

#endif
