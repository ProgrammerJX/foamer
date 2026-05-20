#ifndef XFoam_searchablesurface_H_
#define XFoam_searchablesurface_H_

// 对标 OpenFOAM searchableSurface / searchableSurfaceList 的最小子集，供 blockMesh（类型名 XFoam_SearchableSurfaceList）。
// blockFaces::projectFace 的 findNearest 与按名称查找使用。
// 完整几何管线未移植；可插入 XFoam_TrivialSearchableSurface 做恒等投影测试。

#include "XFoam/utilities/xfoam_common.h"

class XFoam_API XFoam_searchableSurface
{
public:
	virtual ~XFoam_searchableSurface() = default;

	virtual XFoam_String name() const = 0;

	virtual void findNearest(
		const XFoam_Field<XFoam_Vector3D>& points,
		const XFoam_Field<XFoam_Scalar>& nearestDistSqr,
		XFoam_List<XFoam_PointIndexHit>& hits) const = 0;
};

/// 将每个采样点标记为命中在自身位置（对标“已在曲面上”的退化情形）。
class XFoam_API XFoam_TrivialSearchableSurface : public XFoam_searchableSurface
{
	XFoam_String name_;

public:
	explicit XFoam_TrivialSearchableSurface(const XFoam_String& n)
		: name_(n)
	{}

	XFoam_String name() const override { return name_; }

	void findNearest(
		const XFoam_Field<XFoam_Vector3D>& points,
		const XFoam_Field<XFoam_Scalar>&,
		XFoam_List<XFoam_PointIndexHit>& hits) const override;
};

class XFoam_API XFoam_SearchableSurfaceList
{
	XFoam_List<XFoam_AutoPtr<XFoam_searchableSurface>> items_;

public:
	XFoam_SearchableSurfaceList() = default;

	XFoam_Label size() const { return items_.size(); }

	void append(XFoam_AutoPtr<XFoam_searchableSurface> s)
	{
		const XFoam_Label n = items_.size();
		items_.setSize(n + 1);
		items_[n] = XFoam_move(s);
	}

	const XFoam_searchableSurface& operator[](const XFoam_Label i) const { return *items_[i]; }
};

#endif
