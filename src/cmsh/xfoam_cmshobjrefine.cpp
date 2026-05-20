#include "XFoam/cmsh/xfoam_cmshobjrefine.h"

#include <algorithm>
#include <cmath>

bool XFoam_CMshBoxRefine::boxIntersects(const XFoam_BoundBox& cellBox) const
{
	return box.overlaps(cellBox);
}

bool XFoam_CMshSphereRefine::boxIntersects(const XFoam_BoundBox& cellBox) const
{
	if (radius <= 0) return false;
	// 球-AABB 最短距离 ≤ R 等价相交
	XFoam_Scalar d2 = 0;
	for (int i = 0; i < 3; ++i)
	{
		const XFoam_Scalar v  = centre[i];
		const XFoam_Scalar lo = cellBox.min()[i];
		const XFoam_Scalar hi = cellBox.max()[i];
		if      (v < lo) { const XFoam_Scalar dx = lo - v; d2 += dx * dx; }
		else if (v > hi) { const XFoam_Scalar dx = v - hi; d2 += dx * dx; }
	}
	return d2 <= radius * radius;
}

bool XFoam_CMshConeRefine::boxIntersects(const XFoam_BoundBox& cellBox) const
{
	// 简化测试：以 (a, b, ra, rb) 定义一条 swept-sphere 截锥。
	// 在 cellBox 8 corner + center 这 9 个采样点中，任一点同时满足以下条件即视为相交：
	//   * 投影参数 t = clamp((p - a) · u / L, 0, 1)，u = (b - a)/L
	//   * |p - (a + t*L*u)| ≤ ra + (rb - ra) * t
	const XFoam_Vector3D axis = b - a;
	const XFoam_Scalar   L    = axis.mag();
	if (L <= 0) return false;
	const XFoam_Vector3D u = axis * (XFoam_Scalar(1) / L);

	auto inside = [&](const XFoam_Vector3D& p) -> bool {
		const XFoam_Vector3D pa = p - a;
		XFoam_Scalar         tL = pa.x() * u.x() + pa.y() * u.y() + pa.z() * u.z();
		tL = std::max<XFoam_Scalar>(0, std::min<XFoam_Scalar>(L, tL));
		const XFoam_Vector3D axisPt = a + u * tL;
		const XFoam_Scalar   r      = (p - axisPt).mag();
		const XFoam_Scalar   t      = tL / L;
		const XFoam_Scalar   rLim   = radiusA + (radiusB - radiusA) * t;
		return r <= rLim;
	};

	const XFoam_Vector3D mn = cellBox.min();
	const XFoam_Vector3D mx = cellBox.max();
	const XFoam_Vector3D corners[9] = {
		XFoam_Vector3D(mn.x(), mn.y(), mn.z()),
		XFoam_Vector3D(mx.x(), mn.y(), mn.z()),
		XFoam_Vector3D(mn.x(), mx.y(), mn.z()),
		XFoam_Vector3D(mx.x(), mx.y(), mn.z()),
		XFoam_Vector3D(mn.x(), mn.y(), mx.z()),
		XFoam_Vector3D(mx.x(), mn.y(), mx.z()),
		XFoam_Vector3D(mn.x(), mx.y(), mx.z()),
		XFoam_Vector3D(mx.x(), mx.y(), mx.z()),
		(mn + mx) * static_cast<XFoam_Scalar>(0.5)
	};
	for (int i = 0; i < 9; ++i)
	{
		if (inside(corners[i])) return true;
	}
	return false;
}
