#ifndef XFoam_PolyPatch_H_
#define XFoam_PolyPatch_H_

// 对标 OpenFOAM-13：meshes/polyMesh/polyPatches/polyPatch/polyPatch.H / polyPatch.C
// 参考：https://gitee.com/runfengtsui/OpenFOAM-13/tree/master/src/OpenFOAM
// 本地对照：D:/git/simulation/tmp/src/OpenFOAM（若未克隆则以上述 URL 为准）。
// XFoam_Dictionary：仅前向声明；dictionary / IOdictionary 构造与 runTimeSelection 未实现，见 .cpp 中 FatalError 或 = delete。
// XFoam_PstreamBuffers：见 xfoam_types.h（与 OF PstreamBuffers 形参位置一致）。

#include "XFoam/mesh/xfoam_primitivepatch.h"
#include "XFoam/mesh/xfoam_shape.h"
#include "XFoam/utilities/xfoam_common.h"

class XFoam_PolyBoundaryMesh;
class XFoam_PolyMesh;
class XFoam_PolyPatch;
class XFoam_PolyTopoChange;
class XFoam_Dictionary;

typedef XFoam_PrimitivePatch<
	XFoam_SubList<XFoam_Face>,
	const XFoam_UList<XFoam_Vector3D>&>
	XFoam_polyPatchPrimitivePatch;

typedef XFoam_PtrList<XFoam_PolyPatch> XFoam_PolyPatchList;

class XFoam_API XFoam_PolyPatch
	: public XFoam_polyPatchPrimitivePatch
{
	XFoam_String name_;
	XFoam_Label index_;
	XFoam_String physicalType_;
	XFoam_StringList inGroups_;

	XFoam_Label start_;
	const XFoam_PolyBoundaryMesh& boundaryMesh_;

	mutable std::unique_ptr<XFoam_SubList<XFoam_Label>> faceCellsPtr_;
	mutable std::unique_ptr<XFoam_LabelList> mePtr_;
	std::unique_ptr<XFoam_Field<XFoam_Scalar>> areaFractionPtr_;

protected:
	friend class XFoam_PolyBoundaryMesh;

	virtual void initCalcGeometry(XFoam_PstreamBuffers&) {}
	virtual void calcGeometry(XFoam_PstreamBuffers&) {}
	virtual void initMovePoints(XFoam_PstreamBuffers&, const XFoam_Field<XFoam_Vector3D>&) {}
	virtual void movePoints(const XFoam_Field<XFoam_Vector3D>& p) override;
	virtual void movePoints(XFoam_PstreamBuffers&, const XFoam_Field<XFoam_Vector3D>& p);
	virtual void initTopoChange(XFoam_PstreamBuffers&) {}
	virtual void topoChange(XFoam_PstreamBuffers&);

public:
	static constexpr const char* typeName = "polyPatch";
	static int disallowGenericPolyPatch;

	XFoam_PolyPatch(
		const XFoam_String& name,
		XFoam_Label size,
		XFoam_Label start,
		XFoam_Label index,
		const XFoam_PolyBoundaryMesh& bm,
		const XFoam_String& patchType);

	XFoam_PolyPatch(
		const XFoam_String& name,
		XFoam_Label size,
		XFoam_Label start,
		XFoam_Label index,
		const XFoam_PolyBoundaryMesh& bm,
		const XFoam_String& physicalType,
		const XFoam_List<XFoam_String>& inGroups);

	XFoam_PolyPatch(const XFoam_PolyPatch& pp, const XFoam_PolyBoundaryMesh& bm);

	XFoam_PolyPatch(
		const XFoam_PolyPatch& pp,
		const XFoam_PolyBoundaryMesh& bm,
		XFoam_Label index,
		XFoam_Label newSize,
		XFoam_Label newStart);

	XFoam_PolyPatch(
		const XFoam_PolyPatch& pp,
		const XFoam_PolyBoundaryMesh& bm,
		XFoam_Label index,
		const XFoam_UList<XFoam_Label>& mapAddressing,
		XFoam_Label newStart);

	XFoam_PolyPatch(const XFoam_PolyPatch& p);

	XFoam_PolyPatch(const XFoam_PolyPatch& p, const XFoam_LabelList& faceCells);

	virtual XFoam_AutoPtr<XFoam_PolyPatch> clone(const XFoam_LabelList& faceCells) const
	{
		return XFoam_AutoPtr<XFoam_PolyPatch>(new XFoam_PolyPatch(*this, faceCells));
	}

	virtual XFoam_AutoPtr<XFoam_PolyPatch> clone(const XFoam_PolyBoundaryMesh& bm) const
	{
		return XFoam_AutoPtr<XFoam_PolyPatch>(new XFoam_PolyPatch(*this, bm));
	}

	virtual XFoam_AutoPtr<XFoam_PolyPatch> clone() const
	{
		return clone(boundaryMesh());
	}

	virtual XFoam_AutoPtr<XFoam_PolyPatch> clone(
		const XFoam_PolyBoundaryMesh& bm,
		XFoam_Label index,
		XFoam_Label newSize,
		XFoam_Label newStart) const
	{
		return XFoam_AutoPtr<XFoam_PolyPatch>(
			new XFoam_PolyPatch(*this, bm, index, newSize, newStart));
	}

	virtual XFoam_AutoPtr<XFoam_PolyPatch> clone(
		const XFoam_PolyBoundaryMesh& bm,
		XFoam_Label index,
		const XFoam_UList<XFoam_Label>& mapAddressing,
		XFoam_Label newStart) const
	{
		return XFoam_AutoPtr<XFoam_PolyPatch>(
			new XFoam_PolyPatch(*this, bm, index, mapAddressing, newStart));
	}

	static XFoam_AutoPtr<XFoam_PolyPatch> New(
		const XFoam_String& patchType,
		const XFoam_String& name,
		XFoam_Label size,
		XFoam_Label start,
		XFoam_Label index,
		const XFoam_PolyBoundaryMesh& bm);

	// XFoam：无 dictionary 模块；声明与 OF 一致，实现中 FatalError（不从 dictionary 构造）。
	static XFoam_AutoPtr<XFoam_PolyPatch> New(
		const XFoam_String& name,
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_PolyBoundaryMesh& bm,
		const XFoam_String& patchType);
	static XFoam_AutoPtr<XFoam_PolyPatch> New(
		const XFoam_String& patchType,
		const XFoam_String& name,
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_PolyBoundaryMesh& bm);

	virtual ~XFoam_PolyPatch();

	virtual void newInternalProcFaces(XFoam_Label&, XFoam_Label&) const {}

	virtual const XFoam_UList<XFoam_Label>& nbrCells() const
	{
		return XFoam_UList<XFoam_Label>::null();
	}

	virtual XFoam_Label neighbPolyPatchID() const { return -1; }

	virtual XFoam_AutoPtr<XFoam_List<XFoam_LabelList>> mapCollocatedFaces() const
	{
		return XFoam_AutoPtr<XFoam_List<XFoam_LabelList>>();
	}

	virtual bool masterImplicit() const { return false; }

	virtual XFoam_String neighbRegionID() const { return XFoam_String("none"); }

	XFoam_Label offset() const noexcept;

	XFoam_Label start() const noexcept { return start_; }

	XFoam_Tuple2<XFoam_Label, XFoam_Label> range() const
	{
		return XFoam_Tuple2<XFoam_Label, XFoam_Label>(start_, this->size());
	}

	const XFoam_PolyBoundaryMesh& boundaryMesh() const noexcept { return boundaryMesh_; }

	const XFoam_PolyMesh& mesh() const;

	virtual bool coupled() const { return false; }

	static bool constraintType(const XFoam_String& patchType);

	static XFoam_StringList constraintTypes();

	template<class T>
	XFoam_UIndirectList<T> patchInternalList(const XFoam_UList<T>& internalValues) const
	{
		return XFoam_UIndirectList<T>(internalValues, faceCells());
	}

	template<class T>
	XFoam_SubList<T> patchSlice(const XFoam_UList<T>& values) const
	{
		return XFoam_SubList<T>(values, this->size(), start_);
	}

	template<class T>
	XFoam_SubList<T> boundarySlice(const XFoam_List<T>& values) const
	{
		return XFoam_SubList<T>(values, this->size(), offset());
	}

	template<class T>
	XFoam_SubField<T> patchSlice(const XFoam_Field<T>& values) const
	{
		return XFoam_SubField<T>(values, this->size(), start_);
	}

	virtual void write(XFoam_OStream& os) const;

	const XFoam_SubList<XFoam_Face> faces() const;

	const XFoam_SubList<XFoam_Label> faceOwner() const;

	const XFoam_SubField<XFoam_Vector3D> faceCentres() const;

	const XFoam_SubField<XFoam_Vector3D> faceAreas() const;

	XFoam_Tmp<XFoam_Field<XFoam_Vector3D>> faceCellCentres() const;

	XFoam_Tmp<XFoam_Field<XFoam_Scalar>> areaFraction(
		const XFoam_Field<XFoam_Vector3D>& points) const;

	XFoam_Tmp<XFoam_Field<XFoam_Scalar>> areaFraction() const;

	void areaFraction(const XFoam_Scalar fraction);

	void areaFraction(const XFoam_Tmp<XFoam_Field<XFoam_Scalar>>& fraction);

	const XFoam_UList<XFoam_Label>& faceCells() const;

	const XFoam_LabelList& meshEdges() const;

	virtual void clearAddressing();

	virtual XFoam_Label whichFace(const XFoam_Label facei) const noexcept
	{
		return facei - start_;
	}

	virtual void initOrder(XFoam_PstreamBuffers&, const XFoam_polyPatchPrimitivePatch&) const {}

	virtual bool order(
		XFoam_PstreamBuffers&,
		const XFoam_polyPatchPrimitivePatch&,
		XFoam_LabelList& faceMap,
		XFoam_LabelList& rotation) const;

	virtual bool changeTopology() const { return false; }

	virtual bool setTopology(XFoam_PolyTopoChange&) { return false; }

	void operator=(const XFoam_PolyPatch& p);

	const XFoam_String& name() const noexcept { return name_; }

	XFoam_Label index() const noexcept { return index_; }

	const XFoam_String& physicalType() const noexcept { return physicalType_; }

	const XFoam_StringList& inGroups() const noexcept { return inGroups_; }

	void clearGeom() override;

	friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_PolyPatch& p);
};

#endif
