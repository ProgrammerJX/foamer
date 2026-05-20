#ifndef XFoam_Primitive_Barycentric_H_
#define XFoam_Primitive_Barycentric_H_

// 重心坐标类型：三角形 3分量与四面体 4 分量（对齐 OpenFOAM 用法）。

#include "XFoam/utilities/xfoam_common.h"

// 三角形重心坐标 (u,v,w)，u+v+w=1。
class XFoam_Barycentric2D
{
	XFoam_Scalar v_[3];

public:
	XFoam_Barycentric2D()
		: v_{0, 0, 0}
	{}
	XFoam_Barycentric2D(XFoam_Scalar a, XFoam_Scalar b, XFoam_Scalar c)
		: v_{a, b, c}
	{}

	XFoam_Scalar& operator[](int i) { return v_[static_cast<unsigned>(i)]; }
	const XFoam_Scalar& operator[](int i) const { return v_[static_cast<unsigned>(i)]; }
};

// 四面体重心坐标 (w0,w1,w2,w3)，w0+w1+w2+w3=1。
class XFoam_Barycentric
{
	XFoam_Scalar v_[4];

public:
	XFoam_Barycentric()
		: v_{0, 0, 0, 0}
	{}
	XFoam_Barycentric(XFoam_Scalar a, XFoam_Scalar b, XFoam_Scalar c, XFoam_Scalar d)
		: v_{a, b, c, d}
	{}

	XFoam_Scalar& operator[](int i) { return v_[static_cast<unsigned>(i)]; }
	const XFoam_Scalar& operator[](int i) const { return v_[static_cast<unsigned>(i)]; }
};

#endif
