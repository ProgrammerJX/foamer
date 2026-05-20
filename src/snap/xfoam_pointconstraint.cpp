#include "XFoam/snap/xfoam_pointconstraint.h"

#include <algorithm>
#include <cmath>

namespace
{
inline XFoam_Vector3D xfoam_anyPerp(const XFoam_Vector3D& t)
{
	// 给定 unit 向量 t 找一个与之正交的单位向量。先用与 |t| 分量最小的轴做叉乘，
	// 避免与 t 几乎平行造成数值不稳。
	const XFoam_Scalar ax = std::fabs(t.x());
	const XFoam_Scalar ay = std::fabs(t.y());
	const XFoam_Scalar az = std::fabs(t.z());
	XFoam_Vector3D ref;
	if (ax <= ay && ax <= az)      ref = XFoam_Vector3D(1, 0, 0);
	else if (ay <= az)             ref = XFoam_Vector3D(0, 1, 0);
	else                           ref = XFoam_Vector3D(0, 0, 1);
	XFoam_Vector3D p(
		t.y() * ref.z() - t.z() * ref.y(),
		t.z() * ref.x() - t.x() * ref.z(),
		t.x() * ref.y() - t.y() * ref.x());
	const XFoam_Scalar n = std::sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z());
	if (n > 0)
	{
		const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / n;
		p.x() *= inv; p.y() *= inv; p.z() *= inv;
	}
	return p;
}

inline XFoam_Scalar xfoam_dot3(const XFoam_Vector3D& a, const XFoam_Vector3D& b)
{
	return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
}

inline XFoam_Vector3D xfoam_cross3(const XFoam_Vector3D& a, const XFoam_Vector3D& b)
{
	return XFoam_Vector3D(
		a.y() * b.z() - a.z() * b.y(),
		a.z() * b.x() - a.x() * b.z(),
		a.x() * b.y() - a.y() * b.x());
}

inline XFoam_Vector3D xfoam_normalize(const XFoam_Vector3D& v)
{
	const XFoam_Scalar n = std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
	if (n == 0) return XFoam_Vector3D(0, 0, 0);
	const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / n;
	return XFoam_Vector3D(v.x() * inv, v.y() * inv, v.z() * inv);
}
} // namespace

XFoam_PointConstraint XFoam_PointConstraint::plane(const XFoam_Vector3D& unitNormal)
{
	XFoam_PointConstraint pc;
	pc.nConstraints_ = 1;
	pc.first_ = unitNormal;
	return pc;
}

XFoam_PointConstraint XFoam_PointConstraint::line(const XFoam_Vector3D& unitTangent)
{
	XFoam_PointConstraint pc;
	pc.nConstraints_ = 2;
	pc.first_ = unitTangent;
	pc.second_ = xfoam_anyPerp(unitTangent);
	return pc;
}

XFoam_PointConstraint XFoam_PointConstraint::fixed()
{
	XFoam_PointConstraint pc;
	pc.nConstraints_ = 3;
	return pc;
}

XFoam_Vector3D XFoam_PointConstraint::constrainDisplacement(const XFoam_Vector3D& d) const
{
	switch (nConstraints_)
	{
		case 0:
			return d;
		case 1:
		{
			// d - (d·n) n；first_ 必须是 unit 法向
			const XFoam_Scalar dn = xfoam_dot3(d, first_);
			return XFoam_Vector3D(
				d.x() - dn * first_.x(),
				d.y() - dn * first_.y(),
				d.z() - dn * first_.z());
		}
		case 2:
		{
			// 投影到 line 方向 t = first_（按定义 unit）。在 line 模式下 second_ 仅做
			// 内部「正交平面法向」记录，constrain 不用它。
			const XFoam_Scalar dt = xfoam_dot3(d, first_);
			return XFoam_Vector3D(
				dt * first_.x(),
				dt * first_.y(),
				dt * first_.z());
		}
		default:
			return XFoam_Vector3D(0, 0, 0);
	}
}

void XFoam_PointConstraint::combine(const XFoam_PointConstraint& other)
{
	if (other.nConstraints_ == 0) return;
	if (nConstraints_ == 3) return;
	if (other.nConstraints_ == 3) { *this = fixed(); return; }

	if (nConstraints_ == 0)
	{
		*this = other;
		return;
	}

	// 两个 plane：同向（含反向）→ 保留原 plane；异向 → 升 line（沿两法向叉乘方向）。
	if (nConstraints_ == 1 && other.nConstraints_ == 1)
	{
		const XFoam_Scalar c = xfoam_dot3(first_, other.first_);
		if (std::fabs(c) >= kParallelCos) return; // already covered
		const XFoam_Vector3D t = xfoam_normalize(xfoam_cross3(first_, other.first_));
		*this = line(t);
		return;
	}

	// plane + line：line 切向是否在 plane 内？若 t ⟂ n → line 已被 plane 包含；
	// 否则 line 与 plane 只剩交点一个 DOF → fixed。
	if (nConstraints_ == 1 && other.nConstraints_ == 2)
	{
		const XFoam_Scalar c = xfoam_dot3(other.first_, first_);
		if (std::fabs(c) <= static_cast<XFoam_Scalar>(1) - kParallelCos)
		{
			*this = other; // line is in plane → keep line
		}
		else
		{
			*this = fixed();
		}
		return;
	}
	if (nConstraints_ == 2 && other.nConstraints_ == 1)
	{
		const XFoam_Scalar c = xfoam_dot3(first_, other.first_);
		if (std::fabs(c) > static_cast<XFoam_Scalar>(1) - kParallelCos)
		{
			*this = fixed();
		}
		return;
	}

	// 两个 line：方向平行（含反向）→ 保留；否则交点为 0 DOF → fixed。
	if (nConstraints_ == 2 && other.nConstraints_ == 2)
	{
		const XFoam_Scalar c = xfoam_dot3(first_, other.first_);
		if (std::fabs(c) < kParallelCos)
		{
			*this = fixed();
		}
	}
}
