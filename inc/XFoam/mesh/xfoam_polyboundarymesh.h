#ifndef XFoam_PolyBoundaryMesh_H_
#define XFoam_PolyBoundaryMesh_H_

// 对标 OpenFOAM-13 meshes/polyMesh/polyBoundaryMesh/polyBoundaryMesh.H / polyBoundaryMesh.C。
// 未移植：IOobject / regIOobject、从 dictionary 读入、wordRe 相关匹配。
// 接口命名与 OF 一致：topoChange()（非 updateMesh）。

#include "XFoam/mesh/xfoam_polypatch.h"
#include "XFoam/utilities/xfoam_common.h"

class XFoam_PolyMesh;

class XFoam_API XFoam_PolyBoundaryMesh
	: public XFoam_PolyPatchList
{
	const XFoam_PolyMesh& mesh_;

	mutable XFoam_AutoPtr<XFoam_LabelList> patchIDPtr_;
	mutable XFoam_AutoPtr<XFoam_HashTable<XFoam_LabelList, XFoam_String>> groupIDsPtr_;
	mutable XFoam_AutoPtr<XFoam_List<XFoam_LabelList>> neighbourEdgesPtr_;

	void calcGeometry();
	bool hasGroupIDs() const;
	void calcGroupIDs() const;
	void clearLocalAddressing();

public:
	static constexpr const char* typeName = "polyBoundaryMesh";

	friend class XFoam_PolyMesh;

	XFoam_PolyBoundaryMesh(const XFoam_PolyBoundaryMesh&) = delete;
	void operator=(const XFoam_PolyBoundaryMesh&) = delete;

	explicit XFoam_PolyBoundaryMesh(const XFoam_PolyMesh& mesh);

	XFoam_PolyBoundaryMesh(const XFoam_PolyMesh& mesh, XFoam_Label size);

	XFoam_PolyBoundaryMesh(const XFoam_PolyMesh& mesh, const XFoam_PolyPatchList& list);

	XFoam_PolyBoundaryMesh(const XFoam_PolyMesh& mesh, XFoam_PolyPatchList&& list);

	~XFoam_PolyBoundaryMesh() = default;

	void clear();

	void clearGeom();

	void clearAddressing();

	XFoam_Label start() const noexcept;

	XFoam_Label nFaces() const noexcept;

	XFoam_Tuple2<XFoam_Label, XFoam_Label> range() const noexcept;

	XFoam_Tuple2<XFoam_Label, XFoam_Label> range(const XFoam_Label patchi) const;

	const XFoam_PolyMesh& mesh() const noexcept { return mesh_; }

	const XFoam_PolyPatch& operator[](const XFoam_Label patchi) const
	{
		return XFoam_PolyPatchList::operator[](patchi);
	}

	XFoam_PolyPatch& operator[](const XFoam_Label patchi)
	{
		return XFoam_PolyPatchList::operator[](patchi);
	}

	const XFoam_PolyPatch& operator[](const XFoam_String& patchName) const;

	void topoChange();

	/// 对标 OpenFOAM polyMesh：先对所有 patch `initMovePoints`，再对所有 patch `movePoints`（PstreamBuffers 串行占位）。
	void initMovePoints(XFoam_PstreamBuffers& pBufs, const XFoam_Field<XFoam_Vector3D>& newPoints);
	void movePoints(XFoam_PstreamBuffers& pBufs, const XFoam_Field<XFoam_Vector3D>& newPoints);

	XFoam_Label whichPatch(const XFoam_Label meshFacei) const;

	XFoam_Tuple2<XFoam_Label, XFoam_Label> whichPatchFace(const XFoam_Label meshFacei) const;

	XFoam_Label findPatchID(const XFoam_String& patchName, const bool allowNotFound = true) const;

	XFoam_List<XFoam_String> names() const;

	XFoam_LabelList patchSizes() const;

	XFoam_LabelList patchStarts() const;

	XFoam_List<XFoam_Tuple2<XFoam_Label, XFoam_Label>> patchRanges() const;

	const XFoam_LabelList& patchID() const;

	const XFoam_HashTable<XFoam_LabelList, XFoam_String>& groupPatchIDs() const;

	const XFoam_List<XFoam_LabelList>& neighbourEdges() const;

	XFoam_SubList<XFoam_Face> faces() const;
};

#endif
