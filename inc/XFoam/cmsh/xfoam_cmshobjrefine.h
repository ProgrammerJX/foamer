#ifndef XFoam_CMshObjRefine_H_
#define XFoam_CMshObjRefine_H_

// 对标 cfMesh: meshLibrary/utilities/octrees/meshOctree/refinementControls/
//              objectRefinement/{objectRefinement,boxRefinement,sphereRefinement,
//                                coneRefinement}.{H,C}
//
// 几何 / 区域类 refinement 触发器。在 octree 阶段：
//   * 调 refineByObject(*obj) 把任何 cell.box 与 obj 相交的 leaf 加密到 obj.level
//
// MVP 支持：
//   * Box     ：boxA & boxB 相交（已等价 RegionRefine，但走统一接口）
//   * Sphere  ：球-AABB 最短距离 ≤ R
//   * Cone    ：截锥（两端不同半径）；近似为 line-AABB 距离 + 半径线性插值

#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <string>

/*---------------------------------------------------------------------------*\
                    Class XFoam_CMshObjRefine Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshObjRefine
{
public:
	XFoam_CMshObjRefine() = default;
	virtual ~XFoam_CMshObjRefine() = default;

	/// 是否需要把与该 cellBox 相交的 leaf 加密。被 octree 调用。
	virtual bool boxIntersects(const XFoam_BoundBox& cellBox) const = 0;

	int         level = 0;     ///< 目标加密 level
	std::string name;          ///< 仅做日志识别用
};

/*---------------------------------------------------------------------------*\
                  Class XFoam_CMshBoxRefine Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshBoxRefine : public XFoam_CMshObjRefine
{
public:
	XFoam_BoundBox box;
	bool boxIntersects(const XFoam_BoundBox& cellBox) const override;
};

/*---------------------------------------------------------------------------*\
                 Class XFoam_CMshSphereRefine Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshSphereRefine : public XFoam_CMshObjRefine
{
public:
	XFoam_Vector3D centre = XFoam_Vector3D(0, 0, 0);
	XFoam_Scalar   radius = 0;
	bool boxIntersects(const XFoam_BoundBox& cellBox) const override;
};

/*---------------------------------------------------------------------------*\
                  Class XFoam_CMshConeRefine Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshConeRefine : public XFoam_CMshObjRefine
{
public:
	XFoam_Vector3D a       = XFoam_Vector3D(0, 0, 0);  ///< 起点
	XFoam_Vector3D b       = XFoam_Vector3D(1, 0, 0);  ///< 终点
	XFoam_Scalar   radiusA = 0;
	XFoam_Scalar   radiusB = 0;
	bool boxIntersects(const XFoam_BoundBox& cellBox) const override;
};

#endif // XFoam_CMshObjRefine_H_
