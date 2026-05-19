#include "XFoam/topo/xfoam_mbrep.h"
#include "XFoam/topo/xfoam_topo.h"

XFoam_TopoEntity::~XFoam_TopoEntity() = default;

XFoam_TopoModel::XFoam_TopoModel()
	: brep_(new XFoam_MBrep())
{}

XFoam_TopoModel::~XFoam_TopoModel() = default;

const XFoam_MBrep& XFoam_TopoModel::mbrep() const
{
	return static_cast<const XFoam_MBrep&>(brep_());
}

XFoam_MBrep& XFoam_TopoModel::mbrep()
{
	return static_cast<XFoam_MBrep&>(brep_());
}

void XFoam_TopoModel::readFromBdf(const XFoam_String& fileName)
{
	mbrep().readFromBdf(fileName);
}

XFoam_BoundBox XFoam_TopoModel::bounds() const
{
	return brep_().bounds();
}
