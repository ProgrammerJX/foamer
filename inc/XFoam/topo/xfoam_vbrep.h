#ifndef XFoam_VBrep_H_
#define XFoam_VBrep_H_

#include "XFoam/utilities/xfoam_common.h"

// 虚拓扑（Virtual Topology）容器：记录与 BRep 相关的虚拟边/面等关系，供 XFoam_TopoModel 使用。

class XFoam_API XFoam_VBrep
{
public:
	XFoam_VBrep();
	virtual ~XFoam_VBrep();

	XFoam_VBrep(const XFoam_VBrep&) = default;
	XFoam_VBrep& operator=(const XFoam_VBrep&) = default;
	XFoam_VBrep(XFoam_VBrep&&) noexcept = default;
	XFoam_VBrep& operator=(XFoam_VBrep&&) noexcept = default;

	virtual void clear();
	virtual XFoam_BoundBox bounds() const = 0;
};

#endif
