#ifndef XFoam_MBrep_H_
#define XFoam_MBrep_H_

#include "XFoam/topo/xfoam_vbrep.h"
#include "XFoam/utilities/xfoam_common.h"

// MBrep：XFoam_VBrep 的派生实现；可从 Nastran BDF 读入节点与壳单元（CTRIA3 / CQUAD4）。
// faces_：与 elems_ 不同；仅收录 BDF 中 $HMMOVE 定义的、且所含单元 ID 全部为已读壳单元的
//         component（逻辑同 XData bdf_readcomps.cpp / BDF_readHMMove）。

class XFoam_API XFoam_MBrep : public XFoam_VBrep
{
	XFoam_List<XFoam_Vector3D> positions_;
	XFoam_List<XFoam_FixedList<XFoam_Label, 4>> elems_;
	XFoam_List<XFoam_FixedList<XFoam_Label, 2>> segments_;
	// 每个面 patch：壳单元在 elems_ 中的下标列表（仅“纯面单元”的 HMMOVE 组件）。
	XFoam_List<XFoam_LabelList> faces_;
	XFoam_List<XFoam_LabelList> edges_;

public:
	XFoam_MBrep();
	~XFoam_MBrep() override;

	XFoam_MBrep(const XFoam_MBrep&) = default;
	XFoam_MBrep& operator=(const XFoam_MBrep&) = default;
	XFoam_MBrep(XFoam_MBrep&&) noexcept = default;
	XFoam_MBrep& operator=(XFoam_MBrep&&) noexcept = default;

	void clear() override;
	void readFromBdf(const XFoam_String& fileName);

	const XFoam_List<XFoam_Vector3D>& positions() const { return positions_; }
	const XFoam_List<XFoam_FixedList<XFoam_Label, 4>>& elems() const { return elems_; }
	const XFoam_List<XFoam_LabelList>& faces() const { return faces_; }

	XFoam_BoundBox bounds() const override;
};

#endif
