#include "XFoam/mesh/xfoam_polyboundarymesh.h"
#include "XFoam/mesh/xfoam_polymesh.h"

XFoam_PolyBoundaryMesh::XFoam_PolyBoundaryMesh(const XFoam_PolyMesh& mesh)
	: XFoam_PolyPatchList()
	, mesh_(mesh)
{}

XFoam_PolyBoundaryMesh::XFoam_PolyBoundaryMesh(const XFoam_PolyMesh& mesh, const XFoam_Label size)
	: XFoam_PolyPatchList()
	, mesh_(mesh)
{
	this->setSize(size);
}

XFoam_PolyBoundaryMesh::XFoam_PolyBoundaryMesh(
	const XFoam_PolyMesh& mesh,
	const XFoam_PolyPatchList& list)
	: XFoam_PolyPatchList(list)
	, mesh_(mesh)
{}

XFoam_PolyBoundaryMesh::XFoam_PolyBoundaryMesh(const XFoam_PolyMesh& mesh, XFoam_PolyPatchList&& list)
	: XFoam_PolyPatchList(XFoam_move(list))
	, mesh_(mesh)
{}

void XFoam_PolyBoundaryMesh::clear()
{
	clearGeom();
	clearAddressing();
	XFoam_PolyPatchList::clear();
}

void XFoam_PolyBoundaryMesh::clearGeom()
{
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		if (XFoam_PolyPatchList::set(i))
		{
			XFoam_PolyPatchList::operator[](i).clearGeom();
		}
	}
}

void XFoam_PolyBoundaryMesh::clearAddressing()
{
	clearLocalAddressing();
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		if (XFoam_PolyPatchList::set(i))
		{
			XFoam_PolyPatchList::operator[](i).clearAddressing();
		}
	}
}

void XFoam_PolyBoundaryMesh::clearLocalAddressing()
{
	patchIDPtr_.clear();
	groupIDsPtr_.clear();
	neighbourEdgesPtr_.clear();
}

XFoam_Label XFoam_PolyBoundaryMesh::start() const noexcept
{
	return mesh_.nInternalFaces();
}

XFoam_Label XFoam_PolyBoundaryMesh::nFaces() const noexcept
{
	return mesh_.nBoundaryFaces();
}

XFoam_Tuple2<XFoam_Label, XFoam_Label> XFoam_PolyBoundaryMesh::range() const noexcept
{
	return XFoam_Tuple2<XFoam_Label, XFoam_Label>(start(), nFaces());
}

XFoam_Tuple2<XFoam_Label, XFoam_Label> XFoam_PolyBoundaryMesh::range(const XFoam_Label patchi) const
{
	if (patchi < 0)
	{
		return XFoam_Tuple2<XFoam_Label, XFoam_Label>(mesh_.nInternalFaces(), 0);
	}
	return (*this)[patchi].range();
}

const XFoam_PolyPatch& XFoam_PolyBoundaryMesh::operator[](const XFoam_String& patchName) const
{
	const XFoam_Label i = findPatchID(patchName, false);
	return XFoam_PolyPatchList::operator[](i);
}

void XFoam_PolyBoundaryMesh::topoChange()
{
	neighbourEdgesPtr_.clear();
	patchIDPtr_.clear();
	groupIDsPtr_.clear();

	XFoam_PstreamBuffers pBufs;
	for (XFoam_Label patchi = 0; patchi < this->size(); ++patchi)
	{
		if (XFoam_PolyPatchList::set(patchi))
		{
			XFoam_PolyPatchList::operator[](patchi).initTopoChange(pBufs);
		}
	}
	for (XFoam_Label patchi = 0; patchi < this->size(); ++patchi)
	{
		if (XFoam_PolyPatchList::set(patchi))
		{
			XFoam_PolyPatchList::operator[](patchi).topoChange(pBufs);
		}
	}
}

void XFoam_PolyBoundaryMesh::calcGeometry()
{
	XFoam_PstreamBuffers pBufs;
	for (XFoam_Label patchi = 0; patchi < this->size(); ++patchi)
	{
		if (XFoam_PolyPatchList::set(patchi))
		{
			XFoam_PolyPatchList::operator[](patchi).initCalcGeometry(pBufs);
		}
	}
	for (XFoam_Label patchi = 0; patchi < this->size(); ++patchi)
	{
		if (XFoam_PolyPatchList::set(patchi))
		{
			XFoam_PolyPatchList::operator[](patchi).calcGeometry(pBufs);
		}
	}
}

void XFoam_PolyBoundaryMesh::initMovePoints(
	XFoam_PstreamBuffers& pBufs, const XFoam_Field<XFoam_Vector3D>& newPoints)
{
	for (XFoam_Label patchi = 0; patchi < this->size(); ++patchi)
	{
		if (XFoam_PolyPatchList::set(patchi))
		{
			XFoam_PolyPatchList::operator[](patchi).initMovePoints(pBufs, newPoints);
		}
	}
}

void XFoam_PolyBoundaryMesh::movePoints(
	XFoam_PstreamBuffers& pBufs, const XFoam_Field<XFoam_Vector3D>& newPoints)
{
	for (XFoam_Label patchi = 0; patchi < this->size(); ++patchi)
	{
		if (XFoam_PolyPatchList::set(patchi))
		{
			XFoam_PolyPatchList::operator[](patchi).movePoints(pBufs, newPoints);
		}
	}
}

XFoam_Label XFoam_PolyBoundaryMesh::whichPatch(const XFoam_Label meshFacei) const
{
	if (meshFacei < mesh_.nInternalFaces() || meshFacei >= mesh_.nFaces())
	{
		return -1;
	}
	XFoam_Label lo = 0;
	XFoam_Label hi = this->size() - 1;
	while (lo <= hi)
	{
		const XFoam_Label mid = (lo + hi) / 2;
		const XFoam_Label ps = (*this)[mid].start();
		const XFoam_Label pe = ps + (*this)[mid].size() - 1;
		if (meshFacei < ps)
		{
			hi = mid - 1;
		}
		else if (meshFacei > pe)
		{
			lo = mid + 1;
		}
		else
		{
			return mid;
		}
	}
	return -1;
}

XFoam_Tuple2<XFoam_Label, XFoam_Label> XFoam_PolyBoundaryMesh::whichPatchFace(
	const XFoam_Label meshFacei) const
{
	const XFoam_Label p = whichPatch(meshFacei);
	if (p < 0)
	{
		return XFoam_Tuple2<XFoam_Label, XFoam_Label>(-1, meshFacei);
	}
	return XFoam_Tuple2<XFoam_Label, XFoam_Label>(p, meshFacei - (*this)[p].start());
}

XFoam_Label XFoam_PolyBoundaryMesh::findPatchID(
	const XFoam_String& patchName,
	const bool allowNotFound) const
{
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		if (XFoam_PolyPatchList::set(i) && (*this)[i].name() == patchName)
		{
			return i;
		}
	}
	if (allowNotFound)
	{
		return -1;
	}
	XFoam_FatalErrorInFunction
		<< "Cannot find patch " << patchName << XFoam_abort(XFoam_FatalError);
	return -1;
}

XFoam_List<XFoam_String> XFoam_PolyBoundaryMesh::names() const
{
	XFoam_List<XFoam_String> n(this->size());
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		n[i] = (*this)[i].name();
	}
	return n;
}

XFoam_LabelList XFoam_PolyBoundaryMesh::patchSizes() const
{
	XFoam_LabelList s(this->size());
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		s[i] = (*this)[i].size();
	}
	return s;
}

XFoam_LabelList XFoam_PolyBoundaryMesh::patchStarts() const
{
	XFoam_LabelList s(this->size());
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		s[i] = (*this)[i].start();
	}
	return s;
}

XFoam_List<XFoam_Tuple2<XFoam_Label, XFoam_Label>> XFoam_PolyBoundaryMesh::patchRanges() const
{
	XFoam_List<XFoam_Tuple2<XFoam_Label, XFoam_Label>> r(this->size());
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		const XFoam_Tuple2<XFoam_Label, XFoam_Label> rng = (*this)[i].range();
		r[i] = rng;
	}
	return r;
}

const XFoam_LabelList& XFoam_PolyBoundaryMesh::patchID() const
{
	if (!patchIDPtr_.valid())
	{
		XFoam_LabelList* p = new XFoam_LabelList(nFaces());
		for (XFoam_Label facei = 0; facei < p->size(); ++facei)
		{
			const XFoam_Label meshFacei = mesh_.nInternalFaces() + facei;
			(*p)[facei] = whichPatch(meshFacei);
		}
		patchIDPtr_.set(p);
	}
	return *patchIDPtr_;
}

bool XFoam_PolyBoundaryMesh::hasGroupIDs() const
{
	for (XFoam_Label i = 0; i < this->size(); ++i)
	{
		if ((*this)[i].inGroups().size())
		{
			return true;
		}
	}
	return false;
}

void XFoam_PolyBoundaryMesh::calcGroupIDs() const
{
	if (!groupIDsPtr_.valid())
	{
		groupIDsPtr_.set(new XFoam_HashTable<XFoam_LabelList, XFoam_String>());
	}
	XFoam_HashTable<XFoam_LabelList, XFoam_String>& g = *groupIDsPtr_;
	g.clear();
	for (XFoam_Label patchi = 0; patchi < this->size(); ++patchi)
	{
		const XFoam_StringList& gr = (*this)[patchi].inGroups();
		for (XFoam_Label gi = 0; gi < gr.size(); ++gi)
		{
			const XFoam_String& gn = gr[gi];
			if (!g.found(gn))
			{
				g.insert(gn, XFoam_LabelList());
			}
			XFoam_LabelList& lst = g[gn];
			const XFoam_Label oldSz = lst.size();
			lst.setSize(oldSz + 1);
			lst[oldSz] = patchi;
		}
	}
}

const XFoam_HashTable<XFoam_LabelList, XFoam_String>& XFoam_PolyBoundaryMesh::groupPatchIDs() const
{
	if (hasGroupIDs())
	{
		if (!groupIDsPtr_.valid())
		{
			calcGroupIDs();
		}
	}
	else if (!groupIDsPtr_.valid())
	{
		groupIDsPtr_.set(new XFoam_HashTable<XFoam_LabelList, XFoam_String>());
	}
	return *groupIDsPtr_;
}

const XFoam_List<XFoam_LabelList>& XFoam_PolyBoundaryMesh::neighbourEdges() const
{
	if (!neighbourEdgesPtr_.valid())
	{
		neighbourEdgesPtr_.set(new XFoam_List<XFoam_LabelList>());
	}
	return *neighbourEdgesPtr_;
}

XFoam_SubList<XFoam_Face> XFoam_PolyBoundaryMesh::faces() const
{
	return XFoam_SubList<XFoam_Face>(mesh_.faces(), nFaces(), start());
}
