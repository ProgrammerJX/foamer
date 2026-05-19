#include "XFoam/mesh/xfoam_polymesh.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_fem.h"
#include <algorithm>
#include <map>
#include <utility>
#include <vector>

namespace
{
XFoam_Label calcNCells_(const XFoam_UList<XFoam_Label>& own)
{
	XFoam_Label nCells = 0;
	for (XFoam_Label i = 0; i < own.size(); ++i)
	{
		nCells = std::max(nCells, own[i] + 1);
	}
	return nCells;
}

static std::pair<XFoam_Label, XFoam_Label> canonicalEdge(const XFoam_Edge& e)
{
	const XFoam_Label a = e.start();
	const XFoam_Label b = e.end();
	return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

using XFoam_FaceKey = std::vector<XFoam_Label>;

static XFoam_FaceKey faceSortedKey(const XFoam_UList<XFoam_Label>& f)
{
	XFoam_FaceKey k(static_cast<size_t>(f.size()));
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		k[static_cast<size_t>(i)] = f[i];
	}
	std::sort(k.begin(), k.end());
	return k;
}

using Occ = std::pair<XFoam_Face, XFoam_Label>;

struct InternalRec
{
	XFoam_Face f;
	XFoam_Label own;
	XFoam_Label nei;
};

static void validateFacePointRange(
	const XFoam_UList<XFoam_Label>& f,
	const XFoam_Label nPoints,
	const char* ctx)
{
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		const XFoam_Label v = f[i];
		if (v < 0 || v >= nPoints)
		{
			throw XFoam_Error(XFoam_String(ctx) + ": face point label out of range");
		}
	}
}

static XFoam_Word patchTypeForPatchName(
	const XFoam_PtrListDictionary<XFoam_Dictionary>& boundaryPatchDicts,
	const XFoam_Word& patchName,
	const XFoam_Word& defaultBoundaryPatchType)
{
	const XFoam_Label idx = boundaryPatchDicts.findIndex(patchName);
	if (idx < 0)
	{
		return defaultBoundaryPatchType;
	}
	return boundaryPatchDicts[idx].lookupOrDefault(
		XFoam_Word("type"), defaultBoundaryPatchType);
}

/// 由 cellShape 合并面并写入 faces/owner/neighbour/patch 元数据（与此前 XFoam_PolyMesh(cellShapes) 行为一致）。
/// setTopology 中 OpenFOAM polyMeshFromShapeMesh 全量移植未完成时，构造函数仍走此路径以保证正确性与测试通过。
static void xfoamPolyMesh_fillFromCellShapesMerge_(
	const XFoam_PointField& pointsField,
	const XFoam_CellShapeList& cellsAsShapes,
	const XFoam_FaceListList& boundaryFaces,
	const XFoam_WordList& boundaryPatchNames,
	XFoam_FaceList& faces_,
	XFoam_LabelList& owner_,
	XFoam_LabelList& neighbour_,
	XFoam_LabelList& patchSizesST,
	XFoam_LabelList& patchStartsST,
	XFoam_Label& defaultPatchStart,
	XFoam_Label& nFacesOut,
	XFoam_CellList& cells)
{
	const XFoam_Label nMeshPoints = static_cast<XFoam_Label>(pointsField.size());
	std::map<XFoam_FaceKey, std::vector<Occ>> faceOcc;

	for (XFoam_Label celli = 0; celli < cellsAsShapes.size(); ++celli)
	{
		const XFoam_CellShape& cell = cellsAsShapes[celli];
		const XFoam_List<XFoam_LabelList> cfaces = cell.collapsedFaces();
		for (XFoam_Label ffi = 0; ffi < cfaces.size(); ++ffi)
		{
			const XFoam_LabelList& fl = cfaces[ffi];
			if (fl.size() < 3)
			{
				continue;
			}
			XFoam_Face meshFace(fl);
			validateFacePointRange(meshFace, nMeshPoints, "XFoam_PolyMesh(cellShapes merge)");
			const XFoam_FaceKey key = faceSortedKey(meshFace);
			faceOcc[key].push_back(Occ(XFoam_move(meshFace), celli));
		}
	}

	std::vector<InternalRec> internalRec;
	std::map<XFoam_FaceKey, Occ> orphanByKey;

	for (auto& kv : faceOcc)
	{
		std::vector<Occ>& occ = kv.second;
		if (occ.size() == 1)
		{
			orphanByKey[kv.first] = XFoam_move(occ[0]);
		}
		else if (occ.size() == 2)
		{
			const XFoam_Label c0 = occ[0].second;
			const XFoam_Label c1 = occ[1].second;
			if (c0 == c1)
			{
				throw XFoam_Error(XFoam_String(
					"XFoam_PolyMesh(cellShapes): duplicate face within the same cell"));
			}
			const XFoam_Label cOwn = std::min(c0, c1);
			const XFoam_Label cNei = std::max(c0, c1);
			const XFoam_Face& meshF = (c0 == cOwn) ? occ[0].first : occ[1].first;
			internalRec.push_back(InternalRec{meshF, cOwn, cNei});
		}
		else
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_PolyMesh(cellShapes): non-manifold face (more than two cells sharing one face)"));
		}
	}

	std::sort(
		internalRec.begin(),
		internalRec.end(),
		[](const InternalRec& a, const InternalRec& b) {
			if (a.own != b.own)
			{
				return a.own < b.own;
			}
			if (a.nei != b.nei)
			{
				return a.nei < b.nei;
			}
			return faceSortedKey(a.f) < faceSortedKey(b.f);
		});

	std::vector<Occ> boundaryOrdered;

	if (boundaryFaces.empty())
	{
		boundaryOrdered.reserve(orphanByKey.size());
		for (const auto& kv : orphanByKey)
		{
			boundaryOrdered.push_back(kv.second);
		}
		std::sort(
			boundaryOrdered.begin(),
			boundaryOrdered.end(),
			[](const Occ& a, const Occ& b) { return faceSortedKey(a.first) < faceSortedKey(b.first); });
	}
	else
	{
		if (boundaryPatchNames.size() != boundaryFaces.size())
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_PolyMesh(cellShapes): boundaryPatchNames size must match boundaryFaces"));
		}
		for (XFoam_Label patchi = 0; patchi < boundaryFaces.size(); ++patchi)
		{
			const XFoam_List<XFoam_Face>& patchFaces = boundaryFaces[patchi];
			for (XFoam_Label fi = 0; fi < patchFaces.size(); ++fi)
			{
				const XFoam_Face& bf = patchFaces[fi];
				validateFacePointRange(bf, nMeshPoints, "XFoam_PolyMesh(boundaryFaces merge)");
				const XFoam_FaceKey key = faceSortedKey(bf);
				const auto it = orphanByKey.find(key);
				if (it == orphanByKey.end())
				{
					throw XFoam_Error(XFoam_String(
						"XFoam_PolyMesh(cellShapes): boundary face does not match any exposed cell face"));
				}
				const Occ skin = it->second;
				orphanByKey.erase(it);
				boundaryOrdered.push_back(Occ(skin.first, skin.second));
			}
		}
		if (!orphanByKey.empty())
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_PolyMesh(cellShapes): not all mesh boundary faces are listed in boundaryFaces"));
		}
	}

	const XFoam_Label nInt = static_cast<XFoam_Label>(internalRec.size());
	const XFoam_Label nBnd = static_cast<XFoam_Label>(boundaryOrdered.size());

	faces_.setSize(nInt + nBnd);
	owner_.setSize(nInt + nBnd);
	neighbour_.setSize(nInt);

	for (XFoam_Label i = 0; i < nInt; ++i)
	{
		faces_[i] = internalRec[static_cast<size_t>(i)].f;
		owner_[i] = internalRec[static_cast<size_t>(i)].own;
		neighbour_[i] = internalRec[static_cast<size_t>(i)].nei;
	}
	for (XFoam_Label i = 0; i < nBnd; ++i)
	{
		faces_[nInt + i] = boundaryOrdered[static_cast<size_t>(i)].first;
		owner_[nInt + i] = boundaryOrdered[static_cast<size_t>(i)].second;
	}

	std::map<XFoam_FaceKey, XFoam_Label> keyToFace;
	for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
	{
		keyToFace[faceSortedKey(faces_[fi])] = fi;
	}

	cells.setSize(cellsAsShapes.size());
	for (XFoam_Label celli = 0; celli < cellsAsShapes.size(); ++celli)
	{
		const XFoam_List<XFoam_LabelList> cfaces = cellsAsShapes[celli].collapsedFaces();
		XFoam_Label nUse = 0;
		for (XFoam_Label ffi = 0; ffi < cfaces.size(); ++ffi)
		{
			if (cfaces[ffi].size() >= 3)
			{
				++nUse;
			}
		}
		cells[celli].setSize(nUse);
		XFoam_Label outi = 0;
		for (XFoam_Label ffi = 0; ffi < cfaces.size(); ++ffi)
		{
			const XFoam_LabelList& fl = cfaces[ffi];
			if (fl.size() < 3)
			{
				continue;
			}
			const XFoam_FaceKey key = faceSortedKey(XFoam_Face(fl));
			const auto it = keyToFace.find(key);
			if (it == keyToFace.end())
			{
				throw XFoam_Error(XFoam_String(
					"XFoam_PolyMesh(cellShapes merge): cell face key not found in merged mesh faces"));
			}
			cells[celli][outi++] = it->second;
		}
	}

	if (boundaryFaces.empty())
	{
		patchSizesST.setSize(1);
		patchSizesST[0] = nBnd;
		patchStartsST.setSize(1);
		patchStartsST[0] = nInt;
		defaultPatchStart = nInt;
	}
	else
	{
		const XFoam_Label nPatches = boundaryFaces.size();
		patchSizesST.setSize(nPatches);
		patchStartsST.setSize(nPatches);
		XFoam_Label start = nInt;
		for (XFoam_Label patchi = 0; patchi < nPatches; ++patchi)
		{
			patchSizesST[patchi] = boundaryFaces[patchi].size();
			patchStartsST[patchi] = start;
			start += patchSizesST[patchi];
		}
		defaultPatchStart = XFoam_Label(-1);
	}

	nFacesOut = static_cast<XFoam_Label>(faces_.size());
}

static XFoam_Label patchNameListFindIndex(
	const XFoam_WordList& names,
	const XFoam_Word& key)
{
	for (XFoam_Label i = 0; i < names.size(); ++i)
	{
		if (names[i] == key)
		{
			return i;
		}
	}
	return static_cast<XFoam_Label>(-1);
}

} // namespace

// 移植源码: OpenFOAM src/OpenFOAM/meshes/polyMesh/polyMesh/polyMesh.C（pointField / faceList / owner / neighbour + 边界 patch 元数据）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
XFoam_PolyMesh::XFoam_PolyMesh(
	XFoam_PointField&& points,
	XFoam_FaceList&& faces,
	XFoam_LabelList&& own,
	XFoam_LabelList&& nei,
	const XFoam_WordList& patchNames,
	const XFoam_LabelList& patchSizes,
	const XFoam_WordList& patchTypes)
	: XFoam_PrimitiveMesh(
		static_cast<XFoam_Label>(points.size()),
		nei.size(),
		own.size(),
		calcNCells_(own))
	, points_(XFoam_move(points))
	, faces_(XFoam_move(faces))
	, owner_(XFoam_move(own))
	, neighbour_(XFoam_move(nei))
	, boundary_(*this)
{
	if (owner_.size() != faces_.size())
	{
		throw XFoam_Error(
			XFoam_String("XFoam_PolyMesh: faceOwner size must equal number of faces"));
	}
	if (neighbour_.size() > owner_.size())
	{
		throw XFoam_Error(
			XFoam_String("XFoam_PolyMesh: faceNeighbour size cannot exceed faceOwner size"));
	}
	for (XFoam_Label facei = 0; facei < neighbour_.size(); ++facei)
	{
		const XFoam_Label n = neighbour_[facei];
		if (n >= 0 && n >= nCells())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_PolyMesh: faceNeighbour out of range"));
		}
	}
	for (XFoam_Label facei = 0; facei < owner_.size(); ++facei)
	{
		const XFoam_Label o = owner_[facei];
		if (o < 0 || o >= nCells())
		{
			throw XFoam_Error(XFoam_String("XFoam_PolyMesh: faceOwner out of range"));
		}
	}

	bounds_ = XFoam_BoundBox(points_);
	calcCellFaces();
	this->reset(
		static_cast<XFoam_Label>(points_.size()),
		neighbour_.size(),
		static_cast<XFoam_Label>(faces_.size()),
		nCells());

	if (patchNames.size() != patchSizes.size() || patchNames.size() != patchTypes.size())
	{
		throw XFoam_Error(
			XFoam_String("XFoam_PolyMesh: patch name/size/type list size mismatch"));
	}
	XFoam_Label sumPatchFaces = 0;
	for (XFoam_Label i = 0; i < patchSizes.size(); ++i)
	{
		sumPatchFaces += patchSizes[i];
	}
	if (sumPatchFaces != nBoundaryFaces())
	{
		throw XFoam_Error(
			XFoam_String("XFoam_PolyMesh: sum of patch sizes must equal nBoundaryFaces"));
	}

	boundary_.setSize(patchNames.size());
	XFoam_Label startFace = nInternalFaces();
	for (XFoam_Label patchi = 0; patchi < patchNames.size(); ++patchi)
	{
		// 不能写 boundary_[patchi]：PolyBoundaryMesh::operator[] 会解引用 AutoPtr<PolyPatch>，
		// 构造当前 patch 时槽位仍为空，触发 XFoam_AutoPtr::operator* FatalError。
		boundary_.set(
			patchi,
			XFoam_AutoPtr<XFoam_PolyPatch>(new XFoam_PolyPatch(
				static_cast<const XFoam_String&>(patchNames[patchi]),
				patchSizes[patchi],
				startFace,
				patchi,
				boundary_,
				static_cast<const XFoam_String&>(patchTypes[patchi]))));
		startFace += patchSizes[patchi];
	}

	boundary_.topoChange();
	boundary_.calcGeometry();
}

XFoam_LabelListList XFoam_PolyMesh::cellShapePointCells
(
    const XFoam_CellShapeList& cellsAsShapes
) const
{
    XFoam_DynamicList<XFoam_LabelList, XFoam_PrimitiveMesh::cellsPerPoint_>
        pc(static_cast<XFoam_Label>(points_.size()));

    // For each cell
    XFoam_forAll(cellsAsShapes, i)
    {
        // For each vertex
        const XFoam_LabelList& labels = cellsAsShapes[i];

        XFoam_forAll(labels, j)
        {
            // Set working point label
            XFoam_Label curPoint = labels[j];
            XFoam_LabelList& curPointCells = pc[curPoint];

            // Enter the cell label in the point's cell list
            curPointCells.append(i);
        }
    }

    XFoam_LabelListList pointCellAddr(pc.size());

    XFoam_forAll(pc, pointi)
    {
        pointCellAddr[pointi].transfer(pc[pointi]);
    }

    return pointCellAddr;
}

XFoam_LabelList XFoam_PolyMesh::facePatchFaceCells
(
    const XFoam_FaceList& patchFaces,
    const XFoam_LabelListList& pointCells,
    const XFoam_FaceListList& cellsFaceShapes,
    const XFoam_Label patchID
) const
{
    bool found;

    XFoam_LabelList FaceCells(patchFaces.size());

    XFoam_forAll(patchFaces, fI)
    {
        found = false;

        const XFoam_Face& curFace = patchFaces[fI];
        const XFoam_LabelList& facePoints = patchFaces[fI];

        XFoam_forAll(facePoints, pointi)
        {
            const XFoam_LabelList& facePointCells = pointCells[facePoints[pointi]];

            XFoam_forAll(facePointCells, celli)
            {
                XFoam_FaceList cellFaces = cellsFaceShapes[facePointCells[celli]];

                XFoam_forAll(cellFaces, cellFace)
                {
                    if (XFoam_Face::sameVertices(cellFaces[cellFace], curFace))
                    {
                        // Found the cell corresponding to this face
                        FaceCells[fI] = facePointCells[celli];

                        found = true;
                    }
                    if (found) break;
                }
                if (found) break;
            }
            if (found) break;
        }

        if (!found)
        {
            XFoam_FatalErrorInFunction
                << "face " << fI << " in patch " << patchID
                << " does not have neighbour cell"
                << " (face size " << patchFaces[fI].size() << ")"
                << XFoam_exit(XFoam_FatalError, 1);
        }
    }

    return FaceCells;
}



// 移植源码: OpenFOAM src/OpenFOAM/meshes/polyMesh/polyMesh/polyMeshFromShapeMesh.C
// 命名规范: foam_code.md
// 移植规范: foam_code.md
void XFoam_PolyMesh::setTopology(
	const XFoam_CellShapeList& cellsAsShapes,
	const XFoam_FaceListList& boundaryFaces,
	const XFoam_WordList& boundaryPatchNames,
	XFoam_LabelList& patchSizes,
	XFoam_LabelList& patchStarts,
	XFoam_Label& defaultPatchStart,
	XFoam_Label& nFaces,
	XFoam_CellList& cells)
{
	
    // Calculate the faces of all cells
    // Initialise maximum possible number of mesh faces to 0
    XFoam_Label maxFaces = 0;

    // Set up a list of face shapes for each cell
    XFoam_FaceListList cellsFaceShapes(cellsAsShapes.size());
    cells.setSize(cellsAsShapes.size());

    XFoam_forAll(cellsFaceShapes, celli)
    {
        const XFoam_List<XFoam_LabelList> faceVerts = cellsAsShapes[celli].faces();
        XFoam_FaceList& cellFaceRow = cellsFaceShapes[celli];
        cellFaceRow.setSize(faceVerts.size());
        for (XFoam_Label ffi = 0; ffi < faceVerts.size(); ++ffi)
        {
            cellFaceRow[ffi] = XFoam_Face(faceVerts[ffi]);
        }

        cells[celli].setSize(cellsFaceShapes[celli].size());

        // Initialise cells to -1 to flag undefined faces
        static_cast<XFoam_LabelList&>(cells[celli]) = -1;

        // Count maximum possible number of mesh faces
        maxFaces += cellsFaceShapes[celli].size();
    }

    // Set size of faces array to maximum possible number of mesh faces
    faces_.setSize(maxFaces);

    // Initialise number of faces to 0
    nFaces = 0;

    // Set reference to point-cell addressing
    XFoam_LabelListList PointCells = cellShapePointCells(cellsAsShapes);

    bool found = false;

    XFoam_forAll(cells, celli)
    {
        // Note:
        // Insertion cannot be done in one go as the faces need to be
        // added into the list in the increasing order of neighbour
        // cells.  Therefore, all neighbours will be detected first
        // and then added in the correct order.

        const XFoam_FaceList& curFaces = cellsFaceShapes[celli];

        // Record the neighbour cell
        XFoam_LabelList neiCells(curFaces.size(), -1);

        // Record the face of neighbour cell
        XFoam_LabelList faceOfNeiCell(curFaces.size(), -1);

        XFoam_Label nNeighbours = 0;

        // For all faces ...
        XFoam_forAll(curFaces, facei)
        {
            // Skip faces that have already been matched
            if (cells[celli][facei] >= 0) continue;

            found = false;

            const XFoam_Face& curFace = curFaces[facei];

            // Get the list of labels
            const XFoam_LabelList& curPoints = curFace;

            // For all points
            XFoam_forAll(curPoints, pointi)
            {
                // dGget the list of cells sharing this point
                const XFoam_LabelList& curNeighbours =
                    PointCells[curPoints[pointi]];

                // For all neighbours
                XFoam_forAll(curNeighbours, neiI)
                {
                    XFoam_Label curNei = curNeighbours[neiI];

                    // Reject neighbours with the lower label
                    if (curNei > celli)
                    {
                        // Get the list of search faces
                        const XFoam_FaceList& searchFaces = cellsFaceShapes[curNei];

                        XFoam_forAll(searchFaces, neiFacei)
                        {
                            // 与 OpenFOAM 一致：按顶点集合判同面；仅用 operator==（Face::compare）时，
                            // 部分环向/起点组合下 compare 可返回 0，导致内面匹配失败（多体元 block 网格）。
                            if (XFoam_Face::sameVertices(searchFaces[neiFacei], curFace))
                            {
                                // Match!!
                                found = true;

                                // Record the neighbour cell and face
                                neiCells[facei] = curNei;
                                faceOfNeiCell[facei] = neiFacei;
                                nNeighbours++;

                                break;
                            }
                        }
                        if (found) break;
                    }
                    if (found) break;
                }
                if (found) break;
            } // End of current points
        }  // End of current faces

        // Add the faces in the increasing order of neighbours
        for (XFoam_Label neiSearch = 0; neiSearch < nNeighbours; neiSearch++)
        {
            // Find the lowest neighbour which is still valid
            XFoam_Label nextNei = -1;
            XFoam_Label minNei = cells.size();

            XFoam_forAll(neiCells, ncI)
            {
                if (neiCells[ncI] > -1 && neiCells[ncI] < minNei)
                {
                    nextNei = ncI;
                    minNei = neiCells[ncI];
                }
            }

            if (nextNei > -1)
            {
                // Add the face to the list of faces
                faces_[nFaces] = curFaces[nextNei];

                // Set cell-face and cell-neighbour-face to current face label
                cells[celli][nextNei] = nFaces;
                cells[neiCells[nextNei]][faceOfNeiCell[nextNei]] = nFaces;

                // Stop the neighbour from being used again
                neiCells[nextNei] = -1;

                // Increment number of faces counter
                nFaces++;
            }
            else
            {
                XFoam_FatalErrorInFunction
                    << "Error in internal face insertion"
                    << XFoam_exit(XFoam_FatalError, 1);
            }
        }
    }

    const XFoam_Label nInternalFacesFromShapes = nFaces;

    // Do boundary faces

    patchSizes.setSize(boundaryFaces.size(), -1);
    patchStarts.setSize(boundaryFaces.size(), -1);

    XFoam_forAll(boundaryFaces, patchi)
    {
        const XFoam_FaceList& patchFaces = boundaryFaces[patchi];

        XFoam_LabelList curPatchFaceCells =
            facePatchFaceCells
            (
                patchFaces,
                PointCells,
                cellsFaceShapes,
                patchi
            );

        // Grab the start label
        XFoam_Label curPatchStart = nFaces;

        XFoam_forAll(patchFaces, facei)
        {
            const XFoam_Face& curFace = patchFaces[facei];

            const XFoam_Label cellInside = curPatchFaceCells[facei];

            // Get faces of the cell inside
            const XFoam_FaceList& facesOfCellInside = cellsFaceShapes[cellInside];

            bool found = false;

            XFoam_forAll(facesOfCellInside, cellFacei)
            {
                if (XFoam_Face::sameVertices(facesOfCellInside[cellFacei], curFace))
                {
                    if (cells[cellInside][cellFacei] >= 0)
                    {
                        XFoam_FatalErrorInFunction
                            << "Trying to specify a boundary face (size " << curFace.size() << ")"
                            << " on the face on cell " << cellInside
                            << " which is either an internal face or already "
                            << "belongs to some other patch.  This is face "
                            << facei << " of patch "
                            << patchi << " named "
                            << boundaryPatchNames[patchi] << "."
                            << XFoam_exit(XFoam_FatalError, 1);
                    }

                    found = true;

                    // Set the patch face to corresponding cell-face
                    faces_[nFaces] = facesOfCellInside[cellFacei];

                    cells[cellInside][cellFacei] = nFaces;

                    break;
                }
            }

            if (!found)
            {
                XFoam_FatalErrorInFunction
                    << "face " << facei << " of patch " << patchi
                    << " does not seem to belong to cell " << cellInside
                    << " which, according to the addressing, "
                    << "should be next to it."
                    << XFoam_exit(XFoam_FatalError, 1);
            }

            // Increment the counter of faces
            nFaces++;
        }

        patchSizes[patchi] = nFaces - curPatchStart;
        patchStarts[patchi] = curPatchStart;
    }

    // Grab "non-existing" faces and put them into a default patch

    defaultPatchStart = nFaces;

    XFoam_forAll(cells, celli)
    {
        XFoam_LabelList& curCellFaces = cells[celli];

        XFoam_forAll(curCellFaces, facei)
        {
            if (curCellFaces[facei] == -1) // "non-existent" face
            {
                curCellFaces[facei] = nFaces;
                faces_[nFaces] = cellsFaceShapes[celli][facei];

                nFaces++;
            }
        }
    }

    if (boundaryFaces.empty())
    {
        patchSizes.setSize(1);
        patchStarts.setSize(1);
        patchStarts[0] = defaultPatchStart;
        patchSizes[0] = nFaces - defaultPatchStart;
    }

    // Reset the size of the face list
    faces_.setSize(nFaces);

    owner_.setSize(nFaces);
    neighbour_.setSize(nInternalFacesFromShapes);
    for (XFoam_Label fi = 0; fi < nFaces; ++fi)
    {
        XFoam_Label firstC = -1;
        XFoam_Label secondC = -1;
        for (XFoam_Label celli = 0; celli < cells.size(); ++celli)
        {
            const XFoam_Cell& cc = cells[celli];
            for (XFoam_Label j = 0; j < cc.size(); ++j)
            {
                if (cc[j] != fi)
                {
                    continue;
                }
                if (firstC < 0)
                {
                    firstC = celli;
                }
                else if (celli != firstC && secondC < 0)
                {
                    secondC = celli;
                }
            }
        }
        if (secondC < 0)
        {
            owner_[fi] = firstC;
        }
        else
        {
            owner_[fi] = std::min(firstC, secondC);
            if (fi < nInternalFacesFromShapes)
            {
                neighbour_[fi] = std::max(firstC, secondC);
            }
        }
    }
}

// 移植源码: OpenFOAM src/OpenFOAM/meshes/polyMesh/polyMesh/polyMeshFromShapeMesh.C
// 命名规范: foam_code.md
// 移植规范: foam_code.md
XFoam_PolyMesh::XFoam_PolyMesh(
	XFoam_PointField pointsField,
	const XFoam_CellShapeList& cellsAsShapes,
	const XFoam_FaceListList& boundaryFaces,
	XFoam_WordList boundaryPatchNames,
	const XFoam_PtrListDictionary<XFoam_Dictionary>& boundaryPatchDicts,
	const XFoam_Word& defaultBoundaryPatchName,
	const XFoam_Word& defaultBoundaryPatchType)
	: XFoam_PrimitiveMesh()
	, points_(XFoam_move(pointsField))
	, boundary_(*this)
{
	XFoam_LabelList patchSizes;
	XFoam_LabelList patchStarts;
	XFoam_Label defaultPatchStart;
	XFoam_Label nFaces;
	XFoam_CellList cells;
	setTopology(cellsAsShapes, boundaryFaces, boundaryPatchNames, patchSizes, patchStarts, defaultPatchStart, nFaces, cells);

	XFoam_WordList boundaryPatchTypes(boundaryFaces.size());
	for (XFoam_Label patchi = 0; patchi < boundaryFaces.size(); ++patchi)
	{
		boundaryPatchTypes[patchi] = patchTypeForPatchName(
			boundaryPatchDicts,
			boundaryPatchNames[patchi],
			defaultBoundaryPatchType);
	}

	// Warning: Patches can only be added once the face list is
	// completed, as they hold a subList of the face list
	boundary_.setSize(boundaryFaces.size());
	XFoam_forAll(boundaryFaces, patchi)
	{
		boundary_.set(
			patchi,
			XFoam_PolyPatch::New(
				boundaryPatchTypes[patchi],
				boundaryPatchNames[patchi],
				patchSizes[patchi],
				patchStarts[patchi],
				patchi,
				boundary_));

#if 0
	    // 本项目不使用 polyPatch::physicalType（OpenFOAM 边界物理类型字典项）
	    if
	    (
	        boundaryPatchPhysicalTypes.size()
	     && boundaryPatchPhysicalTypes[patchi].size()
	    )
	    {
	        boundary_[patchi].physicalType() =
	            boundaryPatchPhysicalTypes[patchi];
	    }
#endif
	}

	XFoam_Label nAllPatches = boundaryFaces.size();

	XFoam_Label nDefaultFaces = nFaces - defaultPatchStart;
	if (nDefaultFaces > 0)
	{
	    if (XFoam_debug)
	    {
	        XFoam_infoInFunction
	            << "Found " << nDefaultFaces
	            << " undefined faces in mesh; adding to default patch." << XFoam_endl;
	    }

	    // Check if there already exists a defaultFaces patch as last patch
	    // and reuse it.
	    XFoam_Label patchi = patchNameListFindIndex(boundaryPatchNames, defaultBoundaryPatchName);

	    if (patchi != -1)
	    {
	        if (patchi != boundaryFaces.size()-1 || boundary_[patchi].size())
	        {
	            XFoam_FatalErrorInFunction
	                << "Default patch " << boundary_[patchi].name()
	                << " already has faces in it or is not"
	                << " last in list of patches." << XFoam_exit(XFoam_FatalError, 1);
	        }

	        XFoam_infoInFunction
	            << "Reusing existing patch " << patchi
	            << " for undefined faces." << XFoam_endl;

	        boundary_.set(
	            patchi,
	            XFoam_PolyPatch::New(
	                boundaryPatchTypes[patchi],
	                boundaryPatchNames[patchi],
	                nFaces - defaultPatchStart,
	                defaultPatchStart,
	                patchi,
	                boundary_));
	    }
	    else
	    {
	        boundary_.setSize(nAllPatches + 1);
	        boundary_.set(
	            nAllPatches,
	            XFoam_PolyPatch::New(
	                defaultBoundaryPatchType,
	                defaultBoundaryPatchName,
	                nFaces - defaultPatchStart,
	                defaultPatchStart,
	                nAllPatches,
	                boundary_));

	        nAllPatches++;
	    }
	}

	// Reset the size of the boundary
	boundary_.setSize(nAllPatches);

	bounds_ = XFoam_BoundBox(points_);
	calcCellFaces();
	this->reset(
		static_cast<XFoam_Label>(points_.size()),
		neighbour_.size(),
		static_cast<XFoam_Label>(faces_.size()),
		calcNCells_(owner_));
	boundary_.topoChange();
	boundary_.calcGeometry();
}

XFoam_PolyMesh::~XFoam_PolyMesh() = default;

void XFoam_PolyMesh::removeFiles() const {}

bool XFoam_PolyMesh::write(const bool /*doWrite*/) const
{
	return true;
}

bool XFoam_PolyMesh::writeFEM(XFoam_FileName fileName, bool doWrite) const
{
	if (!doWrite)
	{
		return true;
	}
	try
	{
		XFoam_FEM fem;
		for (XFoam_Label pi = 0; pi < points_.size(); ++pi)
		{
			fem.addNode(points_[pi], static_cast<int>(pi + 1));
		}
		int eid = 1;
		for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
		{
			const XFoam_Face& f = faces_[fi];
			if (f.size() == 4)
			{
				int nv[4] = {
					static_cast<int>(f[0]),
					static_cast<int>(f[1]),
					static_cast<int>(f[2]),
					static_cast<int>(f[3])};
				fem.addQuad(nv, eid++);
			}
		}
		for (const XFoam_Edge& ed : edges())
		{
			int nv[2] = {static_cast<int>(ed.start()), static_cast<int>(ed.end())};
			fem.addLine(nv, eid++);
		}
		fem.writeBdf(fileName);
		return true;
	}
	catch (const XFoam_Error&)
	{
		return false;
	}
}

void XFoam_PolyMesh::calcCellFaces()
{
	const XFoam_Label nC = nCells();
	cellFaces_.setSize(nC);
	for (XFoam_Label celli = 0; celli < nC; ++celli)
	{
		cellFaces_[celli].clear();
	}

	XFoam_LabelList nPerCell(nC, 0);
	for (XFoam_Label facei = 0; facei < owner_.size(); ++facei)
	{
		const XFoam_Label o = owner_[facei];
		nPerCell[o]++;
	}
	for (XFoam_Label facei = 0; facei < neighbour_.size(); ++facei)
	{
		const XFoam_Label n = neighbour_[facei];
		if (n >= 0)
		{
			nPerCell[n]++;
		}
	}
	for (XFoam_Label celli = 0; celli < nC; ++celli)
	{
		cellFaces_[celli].setSize(nPerCell[celli]);
	}
	for (XFoam_Label celli = 0; celli < nC; ++celli)
	{
		nPerCell[celli] = 0;
	}
	for (XFoam_Label facei = 0; facei < owner_.size(); ++facei)
	{
		const XFoam_Label c = owner_[facei];
		cellFaces_[c][nPerCell[c]++] = facei;
	}
	for (XFoam_Label facei = 0; facei < neighbour_.size(); ++facei)
	{
		const XFoam_Label c = neighbour_[facei];
		if (c >= 0)
		{
			cellFaces_[c][nPerCell[c]++] = facei;
		}
	}
}

void XFoam_PolyMesh::calcEdges() const
{
	if (edgesValid_)
	{
		return;
	}
	edges_.clear();
	std::map<std::pair<XFoam_Label, XFoam_Label>, XFoam_Label> edgeMap;
	for (XFoam_Label facei = 0; facei < faces_.size(); ++facei)
	{
		const XFoam_Face& f = faces_[facei];
		const XFoam_Label n = f.size();
		for (XFoam_Label fp = 0; fp < n; ++fp)
		{
			const XFoam_Edge e = f.faceEdge(fp);
			const std::pair<XFoam_Label, XFoam_Label> ck = canonicalEdge(e);
			if (edgeMap.find(ck) == edgeMap.end())
			{
				const XFoam_Label idx = static_cast<XFoam_Label>(edges_.size());
				edgeMap[ck] = idx;
				edges_.append(XFoam_Edge(ck.first, ck.second));
			}
		}
	}

	pointEdges_.clear();
	pointEdges_.setSize(points_.size());
	for (XFoam_Label ei = 0; ei < edges_.size(); ++ei)
	{
		const XFoam_Edge& e = edges_[ei];
		pointEdges_[e.start()].append(ei);
		pointEdges_[e.end()].append(ei);
	}
	edgesValid_ = true;
}

void XFoam_PolyMesh::calcFaceGeom() const
{
	if (faceGeomValid_)
	{
		return;
	}
	faceCentres_.setSize(faces_.size());
	faceAreas_.setSize(faces_.size());
	for (XFoam_Label facei = 0; facei < faces_.size(); ++facei)
	{
		faceCentres_[facei] = faces_[facei].centre(points_);
		faceAreas_[facei] = faces_[facei].area(points_);
	}
	faceGeomValid_ = true;
}

void XFoam_PolyMesh::calcCellCentres() const
{
	if (cellCentresValid_)
	{
		return;
	}
	cellCentres_.setSize(nCells());
	for (XFoam_Label celli = 0; celli < nCells(); ++celli)
	{
		cellCentres_[celli] = cellFaces_[celli].centre(points_, faces_);
	}
	cellCentresValid_ = true;
}

const XFoam_List<XFoam_Edge>& XFoam_PolyMesh::edges() const
{
	calcEdges();
	return edges_;
}

const XFoam_List<XFoam_LabelList>& XFoam_PolyMesh::pointEdges() const
{
	calcEdges();
	return pointEdges_;
}

const XFoam_Field<XFoam_Vector3D>& XFoam_PolyMesh::faceCentres() const
{
	calcFaceGeom();
	return faceCentres_;
}

const XFoam_Field<XFoam_Vector3D>& XFoam_PolyMesh::faceAreas() const
{
	calcFaceGeom();
	return faceAreas_;
}

const XFoam_Field<XFoam_Vector3D>& XFoam_PolyMesh::cellCentres() const
{
	calcCellCentres();
	return cellCentres_;
}

void XFoam_PolyMesh::clearGeom()
{
	XFoam_PrimitiveMesh::clearGeom();
	edges_.clear();
	pointEdges_.clear();
	edgesValid_ = false;
	faceCentres_.clear();
	faceAreas_.clear();
	faceGeomValid_ = false;
	cellCentres_.clear();
	cellCentresValid_ = false;
	boundary_.clearGeom();
}
