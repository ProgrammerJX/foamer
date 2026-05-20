#include "XFoam/snap/xfoam_polypatch.h"
#include "XFoam/snap/xfoam_polyboundarymesh.h"
#include "XFoam/snap/xfoam_polymesh.h"

int XFoam_PolyPatch::disallowGenericPolyPatch = 0;

XFoam_PolyPatch::XFoam_PolyPatch(
	const XFoam_String& name,
	const XFoam_Label size,
	const XFoam_Label start,
	const XFoam_Label index,
	const XFoam_PolyBoundaryMesh& bm,
	const XFoam_String& patchType)
	: XFoam_polyPatchPrimitivePatch(
		XFoam_SubList<XFoam_Face>(bm.mesh().faces(), size, start),
		bm.mesh().points())
	, name_(name)
	, index_(index)
	, physicalType_()
	, inGroups_()
	, start_(start)
	, boundaryMesh_(bm)
{
	(void)patchType;
}

XFoam_PolyPatch::XFoam_PolyPatch(
	const XFoam_String& name,
	const XFoam_Label size,
	const XFoam_Label start,
	const XFoam_Label index,
	const XFoam_PolyBoundaryMesh& bm,
	const XFoam_String& physicalType,
	const XFoam_StringList& inGroups)
	: XFoam_polyPatchPrimitivePatch(
		XFoam_SubList<XFoam_Face>(bm.mesh().faces(), size, start),
		bm.mesh().points())
	, name_(name)
	, index_(index)
	, physicalType_(physicalType)
	, inGroups_(inGroups)
	, start_(start)
	, boundaryMesh_(bm)
{}

XFoam_PolyPatch::XFoam_PolyPatch(const XFoam_PolyPatch& pp, const XFoam_PolyBoundaryMesh& bm)
	: XFoam_polyPatchPrimitivePatch(
		XFoam_SubList<XFoam_Face>(bm.mesh().faces(), pp.size(), pp.start_),
		bm.mesh().points())
	, name_(pp.name_)
	, index_(pp.index_)
	, physicalType_(pp.physicalType_)
	, inGroups_(pp.inGroups_)
	, start_(pp.start_)
	, boundaryMesh_(bm)
{}

XFoam_PolyPatch::XFoam_PolyPatch(
	const XFoam_PolyPatch& pp,
	const XFoam_PolyBoundaryMesh& bm,
	const XFoam_Label index,
	const XFoam_Label newSize,
	const XFoam_Label newStart)
	: XFoam_polyPatchPrimitivePatch(
		XFoam_SubList<XFoam_Face>(bm.mesh().faces(), newSize, newStart),
		bm.mesh().points())
	, name_(pp.name_)
	, index_(index)
	, physicalType_(pp.physicalType_)
	, inGroups_(pp.inGroups_)
	, start_(newStart)
	, boundaryMesh_(bm)
{}

XFoam_PolyPatch::XFoam_PolyPatch(
	const XFoam_PolyPatch& pp,
	const XFoam_PolyBoundaryMesh& bm,
	const XFoam_Label index,
	const XFoam_UList<XFoam_Label>& mapAddressing,
	const XFoam_Label newStart)
	: XFoam_polyPatchPrimitivePatch(
		XFoam_SubList<XFoam_Face>(bm.mesh().faces(), mapAddressing.size(), newStart),
		bm.mesh().points())
	, name_(pp.name_)
	, index_(index)
	, physicalType_(pp.physicalType_)
	, inGroups_(pp.inGroups_)
	, start_(newStart)
	, boundaryMesh_(bm)
{
	(void)pp;
}

XFoam_PolyPatch::XFoam_PolyPatch(const XFoam_PolyPatch& p)
	: XFoam_polyPatchPrimitivePatch(
		XFoam_SubList<XFoam_Face>(
			p.boundaryMesh_.mesh().faces(),
			p.size(),
			p.start_),
		p.boundaryMesh_.mesh().points())
	, name_(p.name_)
	, index_(p.index_)
	, physicalType_(p.physicalType_)
	, inGroups_(p.inGroups_)
	, start_(p.start_)
	, boundaryMesh_(p.boundaryMesh_)
{}

XFoam_PolyPatch::XFoam_PolyPatch(const XFoam_PolyPatch& p, const XFoam_LabelList& faceCells)
	: XFoam_PolyPatch(p)
{
	faceCellsPtr_.reset(
		new XFoam_SubList<XFoam_Label>(faceCells, faceCells.size(), 0));
}

XFoam_PolyPatch::~XFoam_PolyPatch()
{
	clearAddressing();
}

XFoam_AutoPtr<XFoam_PolyPatch> XFoam_PolyPatch::New(
	const XFoam_String& patchType,
	const XFoam_String& name,
	const XFoam_Label size,
	const XFoam_Label start,
	const XFoam_Label index,
	const XFoam_PolyBoundaryMesh& bm)
{
	if (patchType != XFoam_String(typeName))
	{
		XFoam_FatalErrorInFunction
			<< "XFoam_PolyPatch::New : unknown patch type " << patchType
			<< " (only " << typeName << " is available without run-time selection)"
			<< XFoam_abort(XFoam_FatalError);
	}
	return XFoam_AutoPtr<XFoam_PolyPatch>(
		new XFoam_PolyPatch(name, size, start, index, bm, patchType));
}

void XFoam_PolyPatch::movePoints(const XFoam_Field<XFoam_Vector3D>& p)
{
	XFoam_polyPatchPrimitivePatch::movePoints(p);
}

void XFoam_PolyPatch::movePoints(XFoam_PstreamBuffers& pBufs, const XFoam_Field<XFoam_Vector3D>& p)
{
	(void)pBufs;
	movePoints(p);
}

void XFoam_PolyPatch::topoChange(XFoam_PstreamBuffers& pBufs)
{
	(void)pBufs;
	clearAddressing();
	clearPatchMeshAddr();
	clearTopology();
}

XFoam_AutoPtr<XFoam_PolyPatch> XFoam_PolyPatch::New(
	const XFoam_String& name,
	const XFoam_Dictionary&,
	const XFoam_Label index,
	const XFoam_PolyBoundaryMesh& bm,
	const XFoam_String& patchType)
{
	(void)name;
	(void)index;
	(void)bm;
	(void)patchType;
	XFoam_FatalErrorInFunction
		<< "XFoam_PolyPatch dictionary construction is disabled (no XFoam_Dictionary / IOdictionary pipeline)."
		<< XFoam_abort(XFoam_FatalError);
	return XFoam_AutoPtr<XFoam_PolyPatch>();
}

XFoam_AutoPtr<XFoam_PolyPatch> XFoam_PolyPatch::New(
	const XFoam_String& patchType,
	const XFoam_String& name,
	const XFoam_Dictionary&,
	const XFoam_Label index,
	const XFoam_PolyBoundaryMesh& bm)
{
	(void)patchType;
	(void)name;
	(void)index;
	(void)bm;
	XFoam_FatalErrorInFunction
		<< "XFoam_PolyPatch dictionary construction is disabled (no XFoam_Dictionary / IOdictionary pipeline)."
		<< XFoam_abort(XFoam_FatalError);
	return XFoam_AutoPtr<XFoam_PolyPatch>();
}

void XFoam_PolyPatch::clearGeom()
{
	areaFractionPtr_.reset();
	XFoam_polyPatchPrimitivePatch::clearGeom();
}

XFoam_Label XFoam_PolyPatch::offset() const noexcept
{
	return start_ - boundaryMesh_.start();
}

const XFoam_PolyMesh& XFoam_PolyPatch::mesh() const
{
	return boundaryMesh_.mesh();
}

bool XFoam_PolyPatch::constraintType(const XFoam_String&)
{
	return false;
}

XFoam_StringList XFoam_PolyPatch::constraintTypes()
{
	return XFoam_StringList();
}

void XFoam_PolyPatch::write(XFoam_OStream& os) const
{
	os << name_ << " nFaces " << this->size() << " startFace " << start_;
}

const XFoam_SubList<XFoam_Face> XFoam_PolyPatch::faces() const
{
	return patchSlice(mesh().faces());
}

const XFoam_SubList<XFoam_Label> XFoam_PolyPatch::faceOwner() const
{
	return patchSlice(mesh().faceOwner());
}

const XFoam_SubField<XFoam_Vector3D> XFoam_PolyPatch::faceCentres() const
{
	return patchSlice(mesh().faceCentres());
}

const XFoam_SubField<XFoam_Vector3D> XFoam_PolyPatch::faceAreas() const
{
	return patchSlice(mesh().faceAreas());
}

XFoam_Tmp<XFoam_Field<XFoam_Vector3D>> XFoam_PolyPatch::faceCellCentres() const
{
	auto tcc = XFoam_Tmp<XFoam_Field<XFoam_Vector3D>>(
		new XFoam_Field<XFoam_Vector3D>(this->size()));
	XFoam_Field<XFoam_Vector3D>& cc = tcc.ref();
	const XFoam_Field<XFoam_Vector3D>& gcc = boundaryMesh_.mesh().cellCentres();
	const XFoam_UList<XFoam_Label>& fc = faceCells();
	for (XFoam_Label facei = 0; facei < fc.size(); ++facei)
	{
		cc[facei] = gcc[fc[facei]];
	}
	return tcc;
}

XFoam_Tmp<XFoam_Field<XFoam_Scalar>> XFoam_PolyPatch::areaFraction(
	const XFoam_Field<XFoam_Vector3D>& pts) const
{
	auto tfraction = XFoam_Tmp<XFoam_Field<XFoam_Scalar>>(
		new XFoam_Field<XFoam_Scalar>(this->size()));
	XFoam_Field<XFoam_Scalar>& fraction = tfraction.ref();
	const XFoam_SubField<XFoam_Vector3D> fa = this->faceAreas();
	for (XFoam_Label facei = 0; facei < this->size(); ++facei)
	{
		const XFoam_Face& f = this->operator[](facei);
		fraction[facei] = static_cast<XFoam_Scalar>(
			fa[facei].mag() / (f.mag(pts) + XFoam_rootVSmall));
	}
	return tfraction;
}

XFoam_Tmp<XFoam_Field<XFoam_Scalar>> XFoam_PolyPatch::areaFraction() const
{
	if (areaFractionPtr_)
	{
		return XFoam_Tmp<XFoam_Field<XFoam_Scalar>>(
			new XFoam_Field<XFoam_Scalar>(*areaFractionPtr_));
	}
	return XFoam_Tmp<XFoam_Field<XFoam_Scalar>>();
}

void XFoam_PolyPatch::areaFraction(const XFoam_Scalar fraction)
{
	areaFractionPtr_.reset(new XFoam_Field<XFoam_Scalar>(this->size(), fraction));
}

void XFoam_PolyPatch::areaFraction(const XFoam_Tmp<XFoam_Field<XFoam_Scalar>>& fraction)
{
	if (fraction.valid())
	{
		areaFractionPtr_.reset(const_cast<XFoam_Tmp<XFoam_Field<XFoam_Scalar>>&>(fraction).ptr());
	}
	else
	{
		areaFractionPtr_.reset();
	}
}

const XFoam_UList<XFoam_Label>& XFoam_PolyPatch::faceCells() const
{
	if (!faceCellsPtr_)
	{
		faceCellsPtr_.reset(new XFoam_SubList<XFoam_Label>(patchSlice(mesh().faceOwner())));
	}
	return *faceCellsPtr_;
}

const XFoam_LabelList& XFoam_PolyPatch::meshEdges() const
{
	if (!mePtr_)
	{
		mePtr_.reset(new XFoam_LabelList(XFoam_polyPatchPrimitivePatch::meshEdges(
			mesh().edges(),
			mesh().pointEdges())));
	}
	return *mePtr_;
}

void XFoam_PolyPatch::clearAddressing()
{
	faceCellsPtr_.reset();
	mePtr_.reset();
	areaFractionPtr_.reset();
}

bool XFoam_PolyPatch::order(
	XFoam_PstreamBuffers& pBufs,
	const XFoam_polyPatchPrimitivePatch&,
	XFoam_LabelList&,
	XFoam_LabelList&) const
{
	(void)pBufs;
	return false;
}

void XFoam_PolyPatch::operator=(const XFoam_PolyPatch& p)
{
	clearAddressing();
	name_ = p.name_;
	index_ = p.index_;
	physicalType_ = p.physicalType_;
	inGroups_ = p.inGroups_;
	start_ = p.start_;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_PolyPatch& p)
{
	p.write(os);
	return os;
}
