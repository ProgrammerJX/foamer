#ifndef XFoam_unitConversion_H_
#define XFoam_unitConversion_H_

// 对标 OpenFOAM src/OpenFOAM/unitConversion/unitConversion.H（unitConversion / 字典默认单位路径）。
// 命名规范：foam_code.md
// 移植规范：foam_code.md
//
// 当前为最小占位：保留乘子字段，供 XFoam_Dictionary::readType(..., defaultUnits, ...) 与
// assertNoConvertUnits 使用；维度集、namedUnitConversion、unitConversions 全局表等后续对齐 OF。

#include "XFoam/utilities/xfoam_types.h"

class XFoam_API XFoam_UnitConversion
{
	XFoam_Scalar multiplier_;

public:
	/// 与 OF 中用于「无单位/不换算」上下文的默认对象类似。
	static const XFoam_UnitConversion& null();

	XFoam_UnitConversion();

	explicit XFoam_UnitConversion(const XFoam_Scalar multiplier);

	XFoam_UnitConversion(const XFoam_UnitConversion&) = default;
	XFoam_UnitConversion& operator=(const XFoam_UnitConversion&) = default;
	XFoam_UnitConversion(XFoam_UnitConversion&&) = default;
	XFoam_UnitConversion& operator=(XFoam_UnitConversion&&) = default;

	XFoam_Scalar multiplier() const noexcept { return multiplier_; }
};

inline XFoam_UnitConversion::XFoam_UnitConversion()
	: multiplier_(static_cast<XFoam_Scalar>(1))
{}

inline XFoam_UnitConversion::XFoam_UnitConversion(const XFoam_Scalar multiplier)
	: multiplier_(multiplier)
{}

inline const XFoam_UnitConversion& XFoam_UnitConversion::null()
{
	static const XFoam_UnitConversion u(static_cast<XFoam_Scalar>(1));
	return u;
}

#endif
