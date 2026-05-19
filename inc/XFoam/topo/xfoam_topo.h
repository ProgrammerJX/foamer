#ifndef XFoam_Topo_H_
#define XFoam_Topo_H_

#include "XFoam/utilities/xfoam_common.h"

class XFoam_TopoModel;
class XFoam_VBrep;
class XFoam_MBrep;
class XFoam_BoundBox;

// 拓扑实体基类：持有所属 XFoam_TopoModel 的指针（不参与所有权）。
class XFoam_API XFoam_TopoEntity
{
protected:
	const XFoam_TopoModel* model_;

public:
	explicit XFoam_TopoEntity(const XFoam_TopoModel* model = nullptr)
		: model_(model)
	{}

	const XFoam_TopoModel* model() const { return model_; }

	virtual ~XFoam_TopoEntity();
};

class XFoam_API XFoam_TopoVert : public XFoam_TopoEntity
{
public:
	XFoam_TopoVert() = default;
	explicit XFoam_TopoVert(const XFoam_TopoModel* m)
		: XFoam_TopoEntity(m)
	{}
};

class XFoam_API XFoam_TopoEdge : public XFoam_TopoEntity
{
public:
	XFoam_TopoEdge() = default;
	explicit XFoam_TopoEdge(const XFoam_TopoModel* m)
		: XFoam_TopoEntity(m)
	{}
};

class XFoam_API XFoam_TopoFace : public XFoam_TopoEntity
{
public:
	XFoam_TopoFace() = default;
	explicit XFoam_TopoFace(const XFoam_TopoModel* m)
		: XFoam_TopoEntity(m)
	{}
};

class XFoam_API XFoam_TopoBody : public XFoam_TopoEntity
{
public:
	XFoam_TopoBody() = default;
	explicit XFoam_TopoBody(const XFoam_TopoModel* m)
		: XFoam_TopoEntity(m)
	{}
};

// 拓扑模型：以 XFoam_VBrep 接口持有实现（当前为 XFoam_MBrep），并作为各 XFoam_TopoEntity 的上下文。
class XFoam_API XFoam_TopoModel
{
private:
	XFoam_AutoPtr<XFoam_VBrep> brep_;

public:
	XFoam_TopoModel();
	~XFoam_TopoModel();

	XFoam_TopoModel(const XFoam_TopoModel&) = delete;
	XFoam_TopoModel& operator=(const XFoam_TopoModel&) = delete;
	XFoam_TopoModel(XFoam_TopoModel&&) noexcept = default;
	XFoam_TopoModel& operator=(XFoam_TopoModel&&) noexcept = default;

	const XFoam_VBrep& brep() const { return brep_(); }
	XFoam_VBrep& brep() { return brep_(); }

	const XFoam_VBrep& virtualBrep() const { return brep_(); }
	XFoam_VBrep& virtualBrep() { return brep_(); }
	const XFoam_MBrep& mbrep() const;
	XFoam_MBrep& mbrep();

	XFoam_BoundBox bounds() const;

	void readFromBdf(const XFoam_String& fileName);
};

#endif
