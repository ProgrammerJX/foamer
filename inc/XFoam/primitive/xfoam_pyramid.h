#ifndef XFoam_Pyramid_H_
#define XFoam_Pyramid_H_

// 棱锥图元，对齐 OpenFOAM Foam::pyramid（pyramid.H / pyramidI.H）。
// 底面为参数化多边形 polygonRef，须对网格点表提供 centre(points)、area(points)（有向面积向量）。
// 带索引的底面视图为类内 IndexedFace<n>，避免翻译单元级辅助类型。
// 未移植：Istream 构造与流运算符。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/primitive/xfoam_triangle.h"

class XFoam_Face;

template<class Point, class PointRef, class polygonRef>
class XFoam_Pyramid
{
public:
	/// 与 OpenFOAM pyramid 底面 polygon 一致：按全局顶点索引在点表中取点；area 为扇形三角剖分面积向量之和。
	template<XFoam_Label nVerts>
	class IndexedFace
	{
		XFoam_FixedList<XFoam_Label, nVerts> labels_;

	public:
		IndexedFace() = default;

		explicit IndexedFace(XFoam_FixedList<XFoam_Label, nVerts> labels)
			: labels_(XFoam_move(labels))
		{}

		const XFoam_FixedList<XFoam_Label, nVerts>& labels() const { return labels_; }
		XFoam_FixedList<XFoam_Label, nVerts>& labels() { return labels_; }

		Point centre(const XFoam_UList<Point>& points) const
		{
			Point c = Point(XFoam_Zero_v);
			for (XFoam_Label i = 0; i < nVerts; ++i)
			{
				c += points[labels_[i]];
			}
			return c * (1.0 / static_cast<XFoam_Scalar>(nVerts));
		}

		Point area(const XFoam_UList<Point>& points) const
		{
			if (nVerts < 3)
			{
				return Point(XFoam_Zero_v);
			}
			Point sum = Point(XFoam_Zero_v);
			const Point& p0 = points[labels_[0]];
			for (XFoam_Label i = 1; i < nVerts - 1; ++i)
			{
				sum += XFoam_Triangle<Point, const Point&>(
					p0, points[labels_[i]], points[labels_[i + 1]])
					.area();
			}
			return sum;
		}
	};

private:
	polygonRef base_;
	PointRef apex_;

public:
	XFoam_Pyramid(polygonRef base, const Point& apex)
		: base_(XFoam_move(base))
		, apex_(apex)
	{}

	const Point& apex() const { return apex_; }

	polygonRef base() const { return base_; }

	Point centre(const XFoam_UList<Point>& points) const
	{
		return base_.centre(points) * (3.0 / 4.0) + Point(apex_) * (1.0 / 4.0);
	}

	Point height(const XFoam_UList<Point>& points) const { return Point(apex_) - base_.centre(points); }

	XFoam_Scalar mag(const XFoam_UList<Point>& points) const
	{
		return static_cast<XFoam_Scalar>((1.0 / 3.0) * (base_.area(points) & height(points)));
	}
};

// char 仅作占位 polygonRef，用于写出 IndexedFace<4> 的完整嵌套名；勿用于构造 XFoam_Pyramid<..., char>。
typedef XFoam_Pyramid<XFoam_Vector3D, const XFoam_Vector3D&, char>::IndexedFace<4> XFoam_PyramidQuadBase;

typedef XFoam_Pyramid<XFoam_Vector3D, const XFoam_Vector3D&, XFoam_PyramidQuadBase> XFoam_PyramidQuadLabels;
#endif

