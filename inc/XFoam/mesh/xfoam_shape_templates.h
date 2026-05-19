#ifndef XFoam_Shape_Templates_H_
#define XFoam_Shape_Templates_H_

// XFoam_Face 静态/模板成员实现，对齐 OpenFOAM meshes/meshShapes/face/faceTemplates.C（dev）。
// 由 xfoam_shape.h 末尾包含；勿单独包含本文件。

namespace XFoam_FaceTemplatesDetail
{
inline XFoam_Label fcIndex(XFoam_Label pi, XFoam_Label n)
{
	return n ? ((pi + 1) % n) : 0;
}

} // namespace XFoam_FaceTemplatesDetail

template<class PointField>
inline XFoam_Vector3D XFoam_Face::area(const PointField& ps)
{
	const XFoam_Label n = static_cast<XFoam_Label>(ps.size());
	if (n == 3)
	{
		return static_cast<XFoam_Scalar>(0.5)
			* ((ps[1] - ps[0]) ^ (ps[2] - ps[0]));
	}

	XFoam_Vector3D pAvg(XFoam_Zero_v);
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		pAvg += ps[pi];
	}
	pAvg /= static_cast<XFoam_Scalar>(n);

	XFoam_Vector3D sumA(XFoam_Zero_v);
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		const XFoam_Label pin = XFoam_FaceTemplatesDetail::fcIndex(pi, n);
		const XFoam_Vector3D& p = ps[pi];
		const XFoam_Vector3D& pNext = ps[pin];
		sumA += (pNext - p) ^ (pAvg - p);
	}
	return static_cast<XFoam_Scalar>(0.5) * sumA;
}

template<class PointField>
inline XFoam_Tuple2<XFoam_Vector3D, XFoam_Vector3D> XFoam_Face::areaAndCentre(
	const PointField& ps)
{
	const XFoam_Label n = static_cast<XFoam_Label>(ps.size());
	if (n == 3)
	{
		return XFoam_Tuple2<XFoam_Vector3D, XFoam_Vector3D>(
			static_cast<XFoam_Scalar>(0.5) * ((ps[1] - ps[0]) ^ (ps[2] - ps[0])),
		(static_cast<XFoam_Scalar>(1.0) / static_cast<XFoam_Scalar>(3.0))
				* (ps[0] + ps[1] + ps[2]));
	}

	XFoam_Vector3D pAvg(XFoam_Zero_v);
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		pAvg += ps[pi];
	}
	pAvg /= static_cast<XFoam_Scalar>(n);

	XFoam_Vector3D sumA(XFoam_Zero_v);
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		const XFoam_Label pin = XFoam_FaceTemplatesDetail::fcIndex(pi, n);
		const XFoam_Vector3D& p = ps[pi];
		const XFoam_Vector3D& pNext = ps[pin];
		sumA += (pNext - p) ^ (pAvg - p);
	}
	const XFoam_Vector3D sumAHat = XFoam_normalised(sumA);

	XFoam_Scalar sumAn = 0;
	XFoam_Vector3D sumAnc(XFoam_Zero_v);
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		const XFoam_Label pin = XFoam_FaceTemplatesDetail::fcIndex(pi, n);
		const XFoam_Vector3D& p = ps[pi];
		const XFoam_Vector3D& pNext = ps[pin];
		const XFoam_Vector3D a = (pNext - p) ^ (pAvg - p);
		const XFoam_Vector3D c = p + pNext + pAvg;
		const XFoam_Scalar an = a & sumAHat;
		sumAn += an;
		sumAnc += an * c;
	}

	return XFoam_Tuple2<XFoam_Vector3D, XFoam_Vector3D>(
		static_cast<XFoam_Scalar>(0.5) * sumA,
		sumAn > XFoam_vSmall
			? (static_cast<XFoam_Scalar>(1.0) / static_cast<XFoam_Scalar>(3.0))
	* sumAnc / sumAn
			: pAvg);
}

template<class PointField>
inline XFoam_Vector3D XFoam_Face::centre(const PointField& ps)
{
	return areaAndCentre(ps).second();
}

template<class PointField>
inline XFoam_Tuple2<XFoam_Vector3D, XFoam_Vector3D>
XFoam_Face::areaAndCentreStabilised(const PointField& ps)
{
	// OpenFOAM 使用 vectorAndError / scalarAndError；XFoam 无对应类型时退化为 areaAndCentre。
	if (static_cast<XFoam_Label>(ps.size()) == 3)
	{
		return areaAndCentre(ps);
	}
	return areaAndCentre(ps);
}

template<class Type>
inline Type XFoam_Face::average(
	const XFoam_UList<XFoam_Vector3D>& ps,
	const XFoam_UList<Type>& fld) const
{
	const XFoam_Label n = size();
	if (n == 3)
	{
		return (static_cast<XFoam_Scalar>(1.0) / static_cast<XFoam_Scalar>(3.0))
			* (fld[operator[](0)] + fld[operator[](1)] + fld[operator[](2)]);
	}

	XFoam_Vector3D pAvg(XFoam_Zero_v);
	Type fldAvg = Type();
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		pAvg += ps[operator[](pi)];
		fldAvg += fld[operator[](pi)];
	}
	pAvg /= static_cast<XFoam_Scalar>(n);
	fldAvg /= static_cast<XFoam_Scalar>(n);

	XFoam_Vector3D sumA(XFoam_Zero_v);
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		const XFoam_Label pin = XFoam_FaceTemplatesDetail::fcIndex(pi, n);
		const XFoam_Vector3D& p = ps[operator[](pi)];
		const XFoam_Vector3D& pNext = ps[operator[](pin)];
		sumA += (pNext - p) ^ (pAvg - p);
	}
	const XFoam_Vector3D sumAHat = XFoam_normalised(sumA);

	XFoam_Scalar sumAn = 0;
	Type sumAnf = Type();
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		const XFoam_Label pin = XFoam_FaceTemplatesDetail::fcIndex(pi, n);
		const XFoam_Vector3D& p = ps[operator[](pi)];
		const XFoam_Vector3D& pNext = ps[operator[](pin)];
		const XFoam_Vector3D a = (pNext - p) ^ (pAvg - p);
		const Type f =
			fld[operator[](pi)] + fld[operator[](pin)] + fldAvg;
		const XFoam_Scalar an = a & sumAHat;
		sumAn += an;
		sumAnf += an * f;
	}

	if (sumAn > XFoam_vSmall)
	{
		return (static_cast<XFoam_Scalar>(1.0) / static_cast<XFoam_Scalar>(3.0))
			* sumAnf / sumAn;
	}
	return fldAvg;
}

#endif
