#ifndef XFoam_PrimitivePatchI_H_
#define XFoam_PrimitivePatchI_H_

// 对标 OpenFOAM PrimitivePatch*.C 的实现（模板内联）。

#include "XFoam/utilities/xfoam_common.h"
#include <type_traits>

template<class PF>
inline void XFoam_primitivePatchAssignPoints_(PF& dst, const PF& src, std::true_type)
{
	dst = src;
}

template<class PF>
inline void XFoam_primitivePatchAssignPoints_(PF&, const PF&, std::false_type)
{}

// * * * * * * * * * * * * * * * Constructors / destructor * * * * * * * * //

template<class FaceList, class PointField>
inline XFoam_PrimitivePatch<FaceList, PointField>::XFoam_PrimitivePatch(
	const FaceList& faces,
	const PointField& points)
	: FaceList(faces)
	, points_(points)
{}

template<class FaceList, class PointField>
inline XFoam_PrimitivePatch<FaceList, PointField>::XFoam_PrimitivePatch(
	FaceList&& faces,
	const PointField& points)
	: FaceList(XFoam_move(faces))
	, points_(points)
{}

template<class FaceList, class PointField>
inline XFoam_PrimitivePatch<FaceList, PointField>::XFoam_PrimitivePatch(
	const XFoam_PrimitivePatch& pp)
	: FaceList(static_cast<const FaceList&>(pp))
	, points_(pp.points_)
{}

template<class FaceList, class PointField>
inline XFoam_PrimitivePatch<FaceList, PointField>::XFoam_PrimitivePatch(
	XFoam_PrimitivePatch&& pp) noexcept
	: FaceList(XFoam_move(static_cast<FaceList&>(pp)))
	, points_(pp.points_)
{}

template<class FaceList, class PointField>
inline XFoam_PrimitivePatch<FaceList, PointField>::~XFoam_PrimitivePatch()
{
	clearOut();
}

// * * * * * * * * * * * * * * * clear * * * * * * * * * * * * * * * * * * * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::clearGeom()
{
	if (XFoam_debug)
	{
		XFoam_pout
			<< "XFoam_PrimitivePatch::clearGeom() : Clearing geometric data" << XFoam_endl;
	}
	localPointsValid_ = false;
	localPoints_.clear();
	faceCentresValid_ = false;
	faceCentres_.clear();
	faceAreasValid_ = false;
	faceAreas_.clear();
	magFaceAreasValid_ = false;
	magFaceAreas_.clear();
	faceNormalsValid_ = false;
	faceNormals_.clear();
	pointNormalsValid_ = false;
	pointNormals_.clear();
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::clearTopology()
{
	if (XFoam_debug)
	{
		XFoam_pout
			<< "XFoam_PrimitivePatch::clearTopology() : Clearing patch addressing"
			<< XFoam_endl;
	}
	edgeTopologyValid_ = false;
	edges_.clear();
	faceFaces_.clear();
	edgeFaces_.clear();
	faceEdges_.clear();
	nInternalEdges_ = -1;
	boundaryPointsValid_ = false;
	boundaryPoints_.clear();
	pointEdgesValid_ = false;
	pointEdges_.clear();
	pointFacesValid_ = false;
	pointFaces_.clear();
	edgeLoopsValid_ = false;
	edgeLoops_.clear();
	localPointOrderValid_ = false;
	localPointOrder_.clear();
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::clearPatchMeshAddr()
{
	if (XFoam_debug)
	{
		XFoam_pout
			<< "XFoam_PrimitivePatch::clearPatchMeshAddr() : Clearing patch-mesh addressing"
			<< XFoam_endl;
	}
	meshDataValid_ = false;
	meshPoints_.clear();
	localFaces_.clear();
	meshPointMapValid_ = false;
	meshPointMap_.clear();
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::clearOut()
{
	clearGeom();
	clearTopology();
	clearPatchMeshAddr();
}

// * * * * * * * * * * * * * * * calcMeshData / calcMeshPointMap * * * * * * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcMeshData_() const
{
	if (meshDataValid_)
	{
		return;
	}
	if (XFoam_debug)
	{
		XFoam_pout << "XFoam_PrimitivePatch::calcMeshData_()" << XFoam_endl;
	}
	if (!meshPoints_.empty() || !localFaces_.empty())
	{
		XFoam_FatalErrorInFunction
			<< "meshPoints_ or localFaces_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	meshPointMap_.clear();
	meshPointMapValid_ = false;

	XFoam_Map<XFoam_Label> markedPoints(
		4 * static_cast<XFoam_Label>(this->size()));
	XFoam_DynamicList<XFoam_Label> meshPts(
		2 * static_cast<XFoam_Label>(this->size()));
	for (XFoam_Label facei = 0; facei < static_cast<XFoam_Label>(this->size());
	++facei)
	{
		const face_type& curPoints = this->operator[](facei);
		for (XFoam_Label pointi = 0; pointi < curPoints.size(); ++pointi)
		{
			if (markedPoints.insert(curPoints[pointi], meshPts.size()))
			{
				meshPts.append(curPoints[pointi]);
			}
		}
	}
	meshPts.shrink();
	meshPoints_ = XFoam_LabelList(meshPts.begin(), meshPts.end());

	localFaces_.setSize(static_cast<XFoam_Label>(this->size()));
	for (XFoam_Label facei = 0; facei < static_cast<XFoam_Label>(this->size());
		++facei)
	{
		localFaces_[facei] = this->operator[](facei);
	}
	for (XFoam_Label facei = 0; facei < static_cast<XFoam_Label>(this->size());
		++facei)
	{
		const face_type& curFace = this->operator[](facei);
		face_type& lf = localFaces_[facei];
		lf.setSize(curFace.size());
		for (XFoam_Label labelI = 0; labelI < curFace.size(); ++labelI)
		{
			lf[labelI] = markedPoints.find(curFace[labelI])();
		}
	}
	meshDataValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcMeshPointMap_() const
{
	if (meshPointMapValid_)
	{
		return;
	}
	if (XFoam_debug)
	{
		XFoam_pout << "XFoam_PrimitivePatch::calcMeshPointMap_()" << XFoam_endl;
	}
	if (!meshPointMap_.empty())
	{
		XFoam_FatalErrorInFunction << "meshPointMap_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_LabelList& mp = meshPoints();
	meshPointMap_ = MeshPointMap((mp.size() > 0 ? 2 * mp.size() : 0));
	for (XFoam_Label i = 0; i < mp.size(); ++i)
	{
		meshPointMap_.insert(mp[i], i);
	}
	meshPointMapValid_ = true;
}

// * * * * * * * * * * * * * * * calcPointFaces / calcPointEdges * * * * * * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcPointFaces_() const
{
	if (pointFacesValid_)
	{
		return;
	}
	if (XFoam_debug)
	{
		XFoam_pout << "XFoam_PrimitivePatch::calcPointFaces_()" << XFoam_endl;
	}
	if (!pointFaces_.empty())
	{
		XFoam_FatalErrorInFunction << "pointFaces already calculated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const LocalFaceList& f = localFaces();
	std::vector<std::list<XFoam_Label>> pointFcs(
		static_cast<XFoam_Size>(meshPoints().size()));
	for (XFoam_Label facei = 0; facei < f.size(); ++facei)
	{
		const face_type& curPoints = f[facei];
		for (XFoam_Label pointi = 0; pointi < curPoints.size(); ++pointi)
		{
			pointFcs[static_cast<XFoam_Size>(curPoints[pointi])].push_back(facei);
		}
	}
	pointFaces_.setSize(static_cast<XFoam_Label>(pointFcs.size()));
	for (XFoam_Label pointi = 0; pointi < pointFaces_.size(); ++pointi)
	{
		const std::list<XFoam_Label>& lst = pointFcs[static_cast<XFoam_Size>(pointi)];
		pointFaces_[pointi].setSize(static_cast<XFoam_Label>(lst.size()));
		XFoam_Label i = 0;
		for (XFoam_Label fi : lst)
		{
			pointFaces_[pointi][i++] = fi;
		}
	}
	pointFacesValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcPointEdges_() const
{
	if (pointEdgesValid_)
	{
		return;
	}
	if (XFoam_debug)
	{
		XFoam_pout << "XFoam_PrimitivePatch::calcPointEdges_()" << XFoam_endl;
	}
	if (!pointEdges_.empty())
	{
		XFoam_FatalErrorInFunction << "pointEdges already calculated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_EdgeList& ed = edges();
	const XFoam_Label nPt = meshPoints().size();
	XFoam_LabelList counts(nPt, 0);
	for (XFoam_Label ei = 0; ei < ed.size(); ++ei)
	{
		counts[ed[ei].start()]++;
		counts[ed[ei].end()]++;
	}
	pointEdges_.setSize(nPt);
	for (XFoam_Label pi = 0; pi < nPt; ++pi)
	{
		pointEdges_[pi].setSize(counts[pi]);
		counts[pi] = 0;
	}
	for (XFoam_Label ei = 0; ei < ed.size(); ++ei)
	{
		const XFoam_Edge& e = ed[ei];
		pointEdges_[e.start()][counts[e.start()]++] = ei;
		pointEdges_[e.end()][counts[e.end()]++] = ei;
	}
	pointEdgesValid_ = true;
}

// * * * * * * * * * * * * * * * calcAddressing (PrimitivePatchAddressing.C) * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcAddressing_() const
{
	if (edgeTopologyValid_)
	{
		return;
	}
	if (XFoam_debug)
	{
		XFoam_pout << "XFoam_PrimitivePatch::calcAddressing_()" << XFoam_endl;
	}
	if (!edges_.empty() || !faceFaces_.empty() || !edgeFaces_.empty()
		|| !faceEdges_.empty())
	{
		XFoam_FatalErrorInFunction << "addressing already calculated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const LocalFaceList& locFcs = localFaces();
	const XFoam_LabelListList& pf = pointFaces();

	XFoam_Label maxEdges = 0;
	for (XFoam_Label facei = 0; facei < locFcs.size(); ++facei)
	{
		maxEdges += locFcs[facei].size();
	}
	edges_.setSize(maxEdges);
	XFoam_LabelListList edgeFaces;
	edgeFaces.setSize(maxEdges);

	XFoam_List<XFoam_DynamicList<XFoam_Label>> ff(
		static_cast<XFoam_Label>(locFcs.size()));
	for (XFoam_Label i = 0; i < ff.size(); ++i)
	{
		ff[i] = XFoam_DynamicList<XFoam_Label>(8);
	}

	faceEdges_.setSize(locFcs.size());

	XFoam_LabelList noFaceFaces(locFcs.size());

	XFoam_List<XFoam_EdgeList> faceIntoEdges(locFcs.size());
	for (XFoam_Label facei = 0; facei < locFcs.size(); ++facei)
	{
		faceIntoEdges[facei] = locFcs[facei].edges();
		XFoam_LabelList& curFaceEdges = faceEdges_[facei];
		curFaceEdges.setSize(faceIntoEdges[facei].size());
		for (XFoam_Label fe = 0; fe < curFaceEdges.size(); ++fe)
		{
			curFaceEdges[fe] = -1;
		}
	}
	(void)noFaceFaces;

	XFoam_Label nEdges = 0;
	bool found = false;

	for (XFoam_Label facei = 0; facei < locFcs.size(); ++facei)
	{
		const face_type& curF = locFcs[facei];
		const XFoam_EdgeList& curEdges = faceIntoEdges[facei];

		XFoam_List<XFoam_DynamicList<XFoam_Label>> neiFaces(curF.size());
		XFoam_List<XFoam_DynamicList<XFoam_Label>> edgeOfNeiFace(curF.size());
		for (XFoam_Label k = 0; k < curF.size(); ++k)
		{
			neiFaces[k] = XFoam_DynamicList<XFoam_Label>(4);
			edgeOfNeiFace[k] = XFoam_DynamicList<XFoam_Label>(4);
		}

		XFoam_Label nNeighbours = 0;

		for (XFoam_Label edgeI = 0; edgeI < curEdges.size(); ++edgeI)
		{
			if (faceEdges_[facei][edgeI] >= 0)
			{
				continue;
			}
			found = false;
			const XFoam_Edge& e = curEdges[edgeI];
			const XFoam_LabelList& nbrFaces = pf[e.start()];
			for (XFoam_Label nbrFacei = 0; nbrFacei < nbrFaces.size(); ++nbrFacei)
			{
				XFoam_Label curNei = nbrFaces[nbrFacei];
				if (curNei > facei)
				{
					const XFoam_EdgeList& searchEdges = faceIntoEdges[curNei];
					for (XFoam_Label neiEdgeI = 0; neiEdgeI < searchEdges.size();
						++neiEdgeI)
					{
						if (searchEdges[neiEdgeI] == e)
						{
							found = true;
							neiFaces[edgeI].append(curNei);
							edgeOfNeiFace[edgeI].append(neiEdgeI);
							ff[facei].append(curNei);
							ff[curNei].append(facei);
						}
					}
				}
			}
			if (found)
			{
				nNeighbours++;
			}
		}

		for (XFoam_Label neiSearch = 0; neiSearch < nNeighbours; ++neiSearch)
		{
			XFoam_Label nextNei = -1;
			XFoam_Label minNei = locFcs.size();
			for (XFoam_Label nfI = 0; nfI < neiFaces.size(); ++nfI)
			{
				if (neiFaces[nfI].size() && neiFaces[nfI][0] < minNei)
				{
					nextNei = nfI;
					minNei = neiFaces[nfI][0];
				}
			}
			if (nextNei > -1)
			{
				edges_[nEdges] = curEdges[nextNei];
				faceEdges_[facei][nextNei] = nEdges;
				XFoam_DynamicList<XFoam_Label>& cnf = neiFaces[nextNei];
				XFoam_DynamicList<XFoam_Label>& eonf = edgeOfNeiFace[nextNei];
				XFoam_LabelList& curEf = edgeFaces[nEdges];
				curEf.setSize(static_cast<XFoam_Label>(cnf.size() + 1));
				curEf[0] = facei;
				for (XFoam_Label cnfI = 0; cnfI < cnf.size(); ++cnfI)
				{
					faceEdges_[cnf[cnfI]][eonf[cnfI]] = nEdges;
					curEf[cnfI + 1] = cnf[cnfI];
				}
				cnf.clear();
				eonf.clear();
				nEdges++;
			}
			else
			{
				XFoam_FatalErrorInFunction << "Error in internal edge insertion"
					<< XFoam_abort(XFoam_FatalError);
			}
		}
	}

	nInternalEdges_ = nEdges;

	for (XFoam_Label facei = 0; facei < faceEdges_.size(); ++facei)
	{
		XFoam_LabelList& curE = faceEdges_[facei];
		for (XFoam_Label edgeI = 0; edgeI < curE.size(); ++edgeI)
		{
			if (curE[edgeI] < 0)
			{
				edges_[nEdges] = faceIntoEdges[facei][edgeI];
				curE[edgeI] = nEdges;
				XFoam_LabelList& curEf = edgeFaces[nEdges];
				curEf.setSize(1);
				curEf[0] = facei;
				nEdges++;
			}
		}
	}
	edges_.setSize(nEdges);
	edgeFaces.setSize(nEdges);

	faceFaces_.setSize(locFcs.size());
	for (XFoam_Label facei = 0; facei < locFcs.size(); ++facei)
	{
		faceFaces_[facei] = XFoam_LabelList(ff[facei].begin(), ff[facei].end());
	}
	edgeFaces_ = XFoam_move(edgeFaces);
	edgeTopologyValid_ = true;
}

// * * * * * * * * * * * * * * * calcBdryPoints * * * * * * * * * * * * * * * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcBdryPoints_() const
{
	if (boundaryPointsValid_)
	{
		return;
	}
	if (XFoam_debug)
	{
		XFoam_pout << "XFoam_PrimitivePatch::calcBdryPoints_()" << XFoam_endl;
	}
	if (!boundaryPoints_.empty())
	{
		XFoam_FatalErrorInFunction << "edge types already calculated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_EdgeList& e = edges();
	XFoam_LabelHashSet bp(2 * e.size());
	for (XFoam_Label edgeI = nInternalEdges_; edgeI < e.size(); edgeI++)
	{
		const XFoam_Edge& curEdge = e[edgeI];
		bp.insert(curEdge.start());
		bp.insert(curEdge.end());
	}
	boundaryPoints_.setSize(static_cast<XFoam_Label>(bp.size()));
	XFoam_Label w = 0;
	for (const XFoam_Label v : bp)
	{
		boundaryPoints_[w++] = v;
	}
	std::sort(boundaryPoints_.begin(), boundaryPoints_.end());
	boundaryPointsValid_ = true;
}

// * * * * * * * * * * * * * * * Local / geometry calcs * * * * * * * * * * * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcLocalPoints_() const
{
	if (localPointsValid_)
	{
		return;
	}
	if (!localPoints_.empty())
	{
		XFoam_FatalErrorInFunction << "localPointsPtr_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_LabelList& meshPts = meshPoints();
	localPoints_.setSize(meshPts.size());
	for (XFoam_Label pointi = 0; pointi < meshPts.size(); ++pointi)
	{
		localPoints_[pointi] = points_[meshPts[pointi]];
	}
	localPointsValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcLocalPointOrder_() const
{
	if (localPointOrderValid_)
	{
		return;
	}
	if (!localPointOrder_.empty())
	{
		XFoam_FatalErrorInFunction << "local point order already calculated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const LocalFaceList& lf = localFaces();
	const XFoam_LabelListList& ff = faceFaces();
	XFoam_List<XFoam_UInt8> visitedFace(lf.size(), 0);
	localPointOrder_.setSize(meshPoints().size(), -1);
	XFoam_List<XFoam_UInt8> visitedPoint(localPointOrder_.size(), 0);
	XFoam_Label nPts = 0;
	for (XFoam_Label facei = 0; facei < lf.size(); ++facei)
	{
		if (!visitedFace[facei])
		{
			std::list<XFoam_Label> faceOrder;
			faceOrder.push_back(facei);
			do
			{
				const XFoam_Label curFace = faceOrder.front();
				faceOrder.pop_front();
				if (!visitedFace[curFace])
				{
					visitedFace[curFace] = 1;
					const face_type& curPoints = lf[curFace];
					for (XFoam_Label pointi = 0; pointi < curPoints.size(); ++pointi)
					{
						const XFoam_Label lp = curPoints[pointi];
						if (!visitedPoint[lp])
						{
							visitedPoint[lp] = 1;
							localPointOrder_[nPts++] = lp;
						}
					}
					const XFoam_LabelList& nbrs = ff[curFace];
					for (XFoam_Label nbrI = 0; nbrI < nbrs.size(); ++nbrI)
					{
						if (!visitedFace[nbrs[nbrI]])
						{
							faceOrder.push_back(nbrs[nbrI]);
						}
					}
				}
			} while (!faceOrder.empty());
		}
	}
	localPointOrderValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcFaceCentres_() const
{
	if (faceCentresValid_)
	{
		return;
	}
	if (!faceCentres_.empty())
	{
		XFoam_FatalErrorInFunction << "faceCentres_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	faceCentres_.setSize(static_cast<XFoam_Label>(this->size()));
	for (XFoam_Label facei = 0; facei < static_cast<XFoam_Label>(this->size());
		++facei)
	{
		faceCentres_[facei] = this->operator[](facei).centre(pointsUList_(points_));
	}
	faceCentresValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcFaceAreas_() const
{
	if (faceAreasValid_)
	{
		return;
	}
	if (!faceAreas_.empty())
	{
		XFoam_FatalErrorInFunction << "faceAreas_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	faceAreas_.setSize(static_cast<XFoam_Label>(this->size()));
	for (XFoam_Label facei = 0; facei < static_cast<XFoam_Label>(this->size());
		++facei)
	{
		faceAreas_[facei] = this->operator[](facei).area(pointsUList_(points_));
	}
	faceAreasValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcMagFaceAreas_() const
{
	if (magFaceAreasValid_)
	{
		return;
	}
	if (!magFaceAreas_.empty())
	{
		XFoam_FatalErrorInFunction << "magFaceAreas_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_Field<point_type>& ar = faceAreas();
	magFaceAreas_.setSize(ar.size());
	for (XFoam_Label i = 0; i < ar.size(); ++i)
	{
		magFaceAreas_[i] = static_cast<XFoam_Scalar>(ar[i].mag());
	}
	magFaceAreasValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcFaceNormals_() const
{
	if (faceNormalsValid_)
	{
		return;
	}
	if (!faceNormals_.empty())
	{
		XFoam_FatalErrorInFunction << "faceNormals_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	faceNormals_.setSize(static_cast<XFoam_Label>(this->size()));
	for (XFoam_Label facei = 0; facei < static_cast<XFoam_Label>(this->size());
		++facei)
	{
		faceNormals_[facei] = this->operator[](facei).normal(pointsUList_(points_));
	}
	faceNormalsValid_ = true;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcPointNormals_() const
{
	if (pointNormalsValid_)
	{
		return;
	}
	if (!pointNormals_.empty())
	{
		XFoam_FatalErrorInFunction << "pointNormalsPtr_ already allocated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_Field<point_type>& faceUnitNormals = faceNormals();
	const XFoam_LabelListList& pf = pointFaces();
	pointNormals_.setSize(meshPoints().size());
	for (XFoam_Label i = 0; i < pointNormals_.size(); ++i)
	{
		pointNormals_[i] = point_type{};
	}
	for (XFoam_Label pointi = 0; pointi < pf.size(); ++pointi)
	{
		point_type& curNormal = pointNormals_[pointi];
		const XFoam_LabelList& curFaces = pf[pointi];
		for (XFoam_Label facei = 0; facei < curFaces.size(); ++facei)
		{
			curNormal += faceUnitNormals[curFaces[facei]];
		}
		const XFoam_Scalar m = static_cast<XFoam_Scalar>(curNormal.mag());
		curNormal /= (m + XFoam_small);
	}
	pointNormalsValid_ = true;
}

// * * * * * * * * * * * * * * * calcEdgeLoops * * * * * * * * * * * * * * * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::calcEdgeLoops_() const
{
	if (edgeLoopsValid_)
	{
		return;
	}
	if (!edgeLoops_.empty())
	{
		XFoam_FatalErrorInFunction << "edge loops already calculated"
			<< XFoam_abort(XFoam_FatalError);
	}
	const XFoam_EdgeList& patchEdges = edges();
	const XFoam_Label nIntEdges = nInternalEdges();
	const XFoam_Label nBdryEdges = patchEdges.size() - nIntEdges;
	if (nBdryEdges == 0)
	{
		edgeLoops_.clear();
		edgeLoopsValid_ = true;
		return;
	}
	const XFoam_LabelListList& patchPointEdges = pointEdges();
	XFoam_LabelList loopNumber(nBdryEdges, -1);
	edgeLoops_.setSize(nBdryEdges);
	XFoam_Label loopI = 0;
	while (true)
	{
		XFoam_Label currentEdgeI = -1;
		for (XFoam_Label edgeI = nIntEdges; edgeI < patchEdges.size(); edgeI++)
		{
			if (loopNumber[edgeI - nIntEdges] == -1)
			{
				currentEdgeI = edgeI;
				break;
			}
		}
		if (currentEdgeI == -1)
		{
			break;
		}
		XFoam_DynamicList<XFoam_Label> loop(nBdryEdges);
		XFoam_Label currentVertI = patchEdges[currentEdgeI].start();
		do
		{
			loop.append(currentVertI);
			loopNumber[currentEdgeI - nIntEdges] = loopI;
			currentVertI = patchEdges[currentEdgeI].otherVertex(currentVertI);
			const XFoam_LabelList& curEdges = patchPointEdges[currentVertI];
			currentEdgeI = -1;
			for (XFoam_Label pI = 0; pI < curEdges.size(); pI++)
			{
				XFoam_Label edgeI = curEdges[pI];
				if (edgeI >= nIntEdges && (loopNumber[edgeI - nIntEdges] == -1))
				{
					currentEdgeI = edgeI;
					break;
				}
			}
		} while (currentEdgeI != -1);
		edgeLoops_[loopI] = XFoam_LabelList(loop.begin(), loop.end());
		loopI++;
	}
	edgeLoops_.setSize(loopI);
	edgeLoopsValid_ = true;
}

// * * * * * * * * * * * * * * * visit / check * * * * * * * * * * * * * * * //

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::visitPointRegion(
	const XFoam_Label pointi,
	const XFoam_LabelList& pFaces,
	const XFoam_Label startFacei,
	const XFoam_Label startEdgeI,
	XFoam_UList<XFoam_UInt8>& pFacesHad) const
{
	XFoam_Label index = -1;
	for (XFoam_Label i = 0; i < pFaces.size(); ++i)
	{
		if (pFaces[i] == startFacei)
		{
			index = i;
			break;
		}
	}
	if (index < 0)
	{
		XFoam_FatalErrorInFunction << "findIndex failed for startFacei"
			<< XFoam_abort(XFoam_FatalError);
	}
	if (!pFacesHad[index])
	{
		pFacesHad[index] = 1;
		const XFoam_LabelList& fEdges = faceEdges()[startFacei];
		XFoam_Label nextEdgeI = -1;
		for (XFoam_Label i = 0; i < fEdges.size(); ++i)
		{
			XFoam_Label edgeI = fEdges[i];
			const XFoam_Edge& e = edges()[edgeI];
			if (edgeI != startEdgeI && (e.start() == pointi || e.end() == pointi))
			{
				nextEdgeI = edgeI;
				break;
			}
		}
		if (nextEdgeI == -1)
		{
			XFoam_FatalErrorInFunction				<< "Problem: cannot find edge out of faceEdges on face " << startFacei
				<< " that uses point " << pointi << " and is not edge " << startEdgeI
				<< XFoam_abort(XFoam_FatalError);
		}
		const XFoam_LabelList& eFaces = edgeFaces()[nextEdgeI];
		for (XFoam_Label i = 0; i < eFaces.size(); ++i)
		{
			if (eFaces[i] != startFacei)
			{
				visitPointRegion(pointi, pFaces, eFaces[i], nextEdgeI, pFacesHad);
			}
		}
	}
}

template<class FaceList, class PointField>
inline typename XFoam_PrimitivePatch<FaceList, PointField>::SurfaceTopo
XFoam_PrimitivePatch<FaceList, PointField>::surfaceType(
	XFoam_LabelHashSet* badEdgesPtr) const
{
	const XFoam_LabelListList& edgeFcs = edgeFaces();
	SurfaceTopo pType = MANIFOLD;
	for (XFoam_Label edgeI = 0; edgeI < edgeFcs.size(); ++edgeI)
	{
		const XFoam_Label nNbrs = edgeFcs[edgeI].size();
		if (nNbrs < 1 || nNbrs > 2)
		{
			if (badEdgesPtr)
			{
				badEdgesPtr->insert(edgeI);
			}
			return ILLEGAL;
		}
		if (nNbrs == 1)
		{
			pType = OPEN;
		}
	}
	return pType;
}

template<class FaceList, class PointField>
inline bool XFoam_PrimitivePatch<FaceList, PointField>::checkTopology(
	const bool report,
	XFoam_LabelHashSet* setPtr) const
{
	const XFoam_LabelListList& edgeFcs = edgeFaces();
	bool illegalTopo = false;
	for (XFoam_Label edgeI = 0; edgeI < edgeFcs.size(); ++edgeI)
	{
		const XFoam_Label nNbrs = edgeFcs[edgeI].size();
		if (nNbrs < 1 || nNbrs > 2)
		{
			illegalTopo = true;
			if (report)
			{
				XFoam_pout << "Edge " << edgeI << " has " << nNbrs << " face neighbours"
					<< XFoam_endl;
			}
			if (setPtr)
			{
				const XFoam_Edge& e = edges()[edgeI];
				setPtr->insert(meshPoints()[e.start()]);
				setPtr->insert(meshPoints()[e.end()]);
			}
		}
	}
	return illegalTopo;
}

template<class FaceList, class PointField>
inline bool XFoam_PrimitivePatch<FaceList, PointField>::checkPointManifold(
	const bool report,
	XFoam_LabelHashSet* setPtr) const
{
	const XFoam_LabelListList& pf = pointFaces();
	const XFoam_LabelListList& pe = pointEdges();
	const XFoam_LabelListList& ef = edgeFaces();
	const XFoam_LabelList& mp = meshPoints();
	bool foundError = false;
	for (XFoam_Label pointi = 0; pointi < pf.size(); ++pointi)
	{
		const XFoam_LabelList& pFaces = pf[pointi];
		XFoam_List<XFoam_UInt8> pFacesHad(pFaces.size(), 0);
		const XFoam_LabelList& pEdges = pe[pointi];
		const XFoam_Label startEdgeI = pEdges[0];
		const XFoam_LabelList& eFaces = ef[startEdgeI];
		for (XFoam_Label i = 0; i < eFaces.size(); ++i)
		{
			visitPointRegion(pointi, pFaces, eFaces[i], startEdgeI, pFacesHad);
		}
		XFoam_Label unset = -1;
		for (XFoam_Label i = 0; i < pFacesHad.size(); ++i)
		{
			if (!pFacesHad[i])
			{
				unset = i;
				break;
			}
		}
		if (unset != -1)
		{
			foundError = true;
			const XFoam_Label meshPointi = mp[pointi];
			if (setPtr)
			{
				setPtr->insert(meshPointi);
			}
			if (report)
			{
				XFoam_pout << "Point " << meshPointi << " multiply connected" << XFoam_endl;
			}
		}
	}
	return foundError;
}

// * * * * * * * * * * * * * * * Accessors * * * * * * * * * * * * * * * * * //

template<class FaceList, class PointField>
inline const XFoam_EdgeList& XFoam_PrimitivePatch<FaceList, PointField>::edges()
	const
{
	if (!edgeTopologyValid_)
	{
		calcAddressing_();
	}
	return edges_;
}

template<class FaceList, class PointField>
inline XFoam_SubList<XFoam_Edge>
XFoam_PrimitivePatch<FaceList, PointField>::internalEdges() const
{
	const XFoam_EdgeList& e = edges();
	return XFoam_SubList<XFoam_Edge>(e, nInternalEdges(), 0);
}

template<class FaceList, class PointField>
inline XFoam_SubList<XFoam_Edge>
XFoam_PrimitivePatch<FaceList, PointField>::boundaryEdges() const
{
	const XFoam_EdgeList& e = edges();
	return XFoam_SubList<XFoam_Edge>(e, e.size() - nInternalEdges(), nInternalEdges());
}

template<class FaceList, class PointField>
inline XFoam_Label XFoam_PrimitivePatch<FaceList, PointField>::nInternalEdges() const
{
	if (!edgeTopologyValid_)
	{
		calcAddressing_();
	}
	return nInternalEdges_;
}

template<class FaceList, class PointField>
inline XFoam_Label XFoam_PrimitivePatch<FaceList, PointField>::nBoundaryEdges() const
{
	return nEdges() - nInternalEdges();
}

template<class FaceList, class PointField>
inline const XFoam_LabelList&
XFoam_PrimitivePatch<FaceList, PointField>::boundaryPoints() const
{
	if (!boundaryPointsValid_)
	{
		calcBdryPoints_();
	}
	return boundaryPoints_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelListList&
XFoam_PrimitivePatch<FaceList, PointField>::faceFaces() const
{
	if (!edgeTopologyValid_)
	{
		calcAddressing_();
	}
	return faceFaces_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelListList&
XFoam_PrimitivePatch<FaceList, PointField>::edgeFaces() const
{
	if (!edgeTopologyValid_)
	{
		calcAddressing_();
	}
	return edgeFaces_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelListList&
XFoam_PrimitivePatch<FaceList, PointField>::faceEdges() const
{
	if (!edgeTopologyValid_)
	{
		calcAddressing_();
	}
	return faceEdges_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelListList&
XFoam_PrimitivePatch<FaceList, PointField>::pointEdges() const
{
	if (!pointEdgesValid_)
	{
		calcPointEdges_();
	}
	return pointEdges_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelListList&
XFoam_PrimitivePatch<FaceList, PointField>::pointFaces() const
{
	if (!pointFacesValid_)
	{
		calcPointFaces_();
	}
	return pointFaces_;
}

template<class FaceList, class PointField>
inline const typename XFoam_PrimitivePatch<FaceList, PointField>::LocalFaceList&
XFoam_PrimitivePatch<FaceList, PointField>::localFaces() const
{
	if (!meshDataValid_)
	{
		calcMeshData_();
	}
	return localFaces_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelList& XFoam_PrimitivePatch<FaceList, PointField>::meshPoints() const
{
	if (!meshDataValid_)
	{
		calcMeshData_();
	}
	return meshPoints_;
}

template<class FaceList, class PointField>
inline const typename XFoam_PrimitivePatch<FaceList, PointField>::MeshPointMap&
XFoam_PrimitivePatch<FaceList, PointField>::meshPointMap() const
{
	if (!meshPointMapValid_)
	{
		calcMeshPointMap_();
	}
	return meshPointMap_;
}

template<class FaceList, class PointField>
inline const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&
XFoam_PrimitivePatch<FaceList, PointField>::localPoints() const
{
	if (!localPointsValid_)
	{
		calcLocalPoints_();
	}
	return localPoints_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelList&
XFoam_PrimitivePatch<FaceList, PointField>::localPointOrder() const
{
	if (!localPointOrderValid_)
	{
		calcLocalPointOrder_();
	}
	return localPointOrder_;
}

template<class FaceList, class PointField>
inline XFoam_Label XFoam_PrimitivePatch<FaceList, PointField>::whichPoint(
	const XFoam_Label gp) const
{
	calcMeshPointMap_();
	if (meshPointMap_.found(gp))
	{
		return meshPointMap_.find(gp)();
	}
	return -1;
}

template<class FaceList, class PointField>
inline const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&
XFoam_PrimitivePatch<FaceList, PointField>::faceCentres() const
{
	if (!faceCentresValid_)
	{
		calcFaceCentres_();
	}
	return faceCentres_;
}

template<class FaceList, class PointField>
inline const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&
XFoam_PrimitivePatch<FaceList, PointField>::faceAreas() const
{
	if (!faceAreasValid_)
	{
		calcFaceAreas_();
	}
	return faceAreas_;
}

template<class FaceList, class PointField>
inline const XFoam_Field<XFoam_Scalar>&
XFoam_PrimitivePatch<FaceList, PointField>::magFaceAreas() const
{
	if (!magFaceAreasValid_)
	{
		calcMagFaceAreas_();
	}
	return magFaceAreas_;
}

template<class FaceList, class PointField>
inline const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&
XFoam_PrimitivePatch<FaceList, PointField>::faceNormals() const
{
	if (!faceNormalsValid_)
	{
		calcFaceNormals_();
	}
	return faceNormals_;
}

template<class FaceList, class PointField>
inline const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&
XFoam_PrimitivePatch<FaceList, PointField>::pointNormals() const
{
	if (!pointNormalsValid_)
	{
		calcPointNormals_();
	}
	return pointNormals_;
}

template<class FaceList, class PointField>
inline const XFoam_LabelListList&
XFoam_PrimitivePatch<FaceList, PointField>::edgeLoops() const
{
	if (!edgeLoopsValid_)
	{
		calcEdgeLoops_();
	}
	return edgeLoops_;
}

template<class FaceList, class PointField>
inline XFoam_LabelList XFoam_PrimitivePatch<FaceList, PointField>::boundaryFaces() const
{
	XFoam_LabelHashSet fs;
	for (XFoam_Label ei = nInternalEdges(); ei < nEdges(); ++ei)
	{
		const XFoam_LabelList& ef = edgeFaces()[ei];
		for (XFoam_Label k = 0; k < ef.size(); ++k)
		{
			fs.insert(ef[k]);
		}
	}
	XFoam_LabelList out(static_cast<XFoam_Label>(fs.size()));
	XFoam_Label w = 0;
	for (const XFoam_Label f : fs)
	{
		out[w++] = f;
	}
	std::sort(out.begin(), out.end());
	return out;
}

template<class FaceList, class PointField>
inline XFoam_LabelList XFoam_PrimitivePatch<FaceList, PointField>::uniqBoundaryFaces() const
{
	return boundaryFaces();
}

template<class FaceList, class PointField>
inline XFoam_Edge XFoam_PrimitivePatch<FaceList, PointField>::meshEdge(
	const XFoam_Label edgei) const
{
	const XFoam_Edge& pe = edges()[edgei];
	const XFoam_LabelList& mp = meshPoints();
	return XFoam_Edge(mp[pe.start()], mp[pe.end()]);
}

template<class FaceList, class PointField>
inline XFoam_Edge XFoam_PrimitivePatch<FaceList, PointField>::meshEdge(
	const XFoam_Edge& e) const
{
	const XFoam_LabelList& mp = meshPoints();
	return XFoam_Edge(mp[e.start()], mp[e.end()]);
}

template<class FaceList, class PointField>
inline XFoam_Label XFoam_PrimitivePatch<FaceList, PointField>::findEdge(
	const XFoam_Edge& e) const
{
	const XFoam_EdgeList& Edges = edges();
	if (e.start() > -1 && e.start() < nPoints())
	{
		const XFoam_LabelList& pe = pointEdges()[e.start()];
		for (XFoam_Label peI = 0; peI < pe.size(); ++peI)
		{
			if (e == Edges[pe[peI]])
			{
				return pe[peI];
			}
		}
	}
	return -1;
}

template<class FaceList, class PointField>
inline XFoam_LabelList XFoam_PrimitivePatch<FaceList, PointField>::meshEdges(
	const XFoam_EdgeList& allEdges,
	const XFoam_LabelListList& cellEdges,
	const XFoam_LabelList& faceCells) const
{
	const XFoam_EdgeList& PatchEdges = edges();
	const XFoam_LabelListList& EdgeFaces = edgeFaces();
	XFoam_LabelList meshEdgeLabels(PatchEdges.size());
	const XFoam_LabelList& pp = meshPoints();
	for (XFoam_Label edgeI = 0; edgeI < PatchEdges.size(); ++edgeI)
	{
		const XFoam_Edge curEdge(pp[PatchEdges[edgeI].start()], pp[PatchEdges[edgeI].end()]);
		bool found = false;
		const XFoam_LabelList& curFaces = EdgeFaces[edgeI];
		for (XFoam_Label facei = 0; facei < curFaces.size(); ++facei)
		{
			const XFoam_Label curCell = faceCells[curFaces[facei]];
			const XFoam_LabelList& ce = cellEdges[curCell];
			for (XFoam_Label cellEdgeI = 0; cellEdgeI < ce.size(); ++cellEdgeI)
			{
				if (allEdges[ce[cellEdgeI]] == curEdge)
				{
					found = true;
					meshEdgeLabels[edgeI] = ce[cellEdgeI];
					break;
				}
			}
			if (found)
			{
				break;
			}
		}
	}
	return meshEdgeLabels;
}

template<class FaceList, class PointField>
inline XFoam_LabelList XFoam_PrimitivePatch<FaceList, PointField>::meshEdges(
	const XFoam_EdgeList& allEdges,
	const XFoam_LabelListList& meshPointEdges) const
{
	const XFoam_EdgeList& PatchEdges = edges();
	XFoam_LabelList meshEdgeLabels(PatchEdges.size());
	const XFoam_LabelList& pp = meshPoints();
	for (XFoam_Label edgeI = 0; edgeI < PatchEdges.size(); ++edgeI)
	{
		const XFoam_Label globalPointi = pp[PatchEdges[edgeI].start()];
		const XFoam_Edge curEdge(globalPointi, pp[PatchEdges[edgeI].end()]);
		const XFoam_LabelList& pe = meshPointEdges[globalPointi];
		for (XFoam_Label i = 0; i < pe.size(); ++i)
		{
			if (allEdges[pe[i]] == curEdge)
			{
				meshEdgeLabels[edgeI] = pe[i];
				break;
			}
		}
	}
	return meshEdgeLabels;
}

template<class FaceList, class PointField>
inline XFoam_Label XFoam_PrimitivePatch<FaceList, PointField>::meshEdge(
	const XFoam_Label edgei,
	const XFoam_EdgeList& allEdges,
	const XFoam_LabelListList& meshPointEdges) const
{
	const XFoam_EdgeList& PatchEdges = edges();
	const XFoam_LabelList& pp = meshPoints();
	const XFoam_Label globalPointi = pp[PatchEdges[edgei].start()];
	const XFoam_Edge curEdge(globalPointi, pp[PatchEdges[edgei].end()]);
	const XFoam_LabelList& pe = meshPointEdges[globalPointi];
	for (XFoam_Label i = 0; i < pe.size(); ++i)
	{
		if (allEdges[pe[i]] == curEdge)
		{
			return pe[i];
		}
	}
	return -1;
}

template<class FaceList, class PointField>
inline XFoam_LabelList XFoam_PrimitivePatch<FaceList, PointField>::meshEdges(
	const XFoam_UList<XFoam_Label>& edgeLabels,
	const XFoam_EdgeList& allEdges,
	const XFoam_LabelListList& meshPointEdges) const
{
	XFoam_LabelList out(edgeLabels.size());
	for (XFoam_Label i = 0; i < edgeLabels.size(); ++i)
	{
		out[i] = meshEdge(edgeLabels[i], allEdges, meshPointEdges);
	}
	return out;
}

template<class FaceList, class PointField>
inline XFoam_Tuple2<
	typename XFoam_PrimitivePatch<FaceList, PointField>::point_type,
	typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>
XFoam_PrimitivePatch<FaceList, PointField>::box() const
{
	using Point = typename XFoam_PrimitivePatch<FaceList, PointField>::point_type;
	const XFoam_LabelList& mp = meshPoints();
	const XFoam_UList<XFoam_Vector3D>& pts = pointsUList_(points_);
	Point pMin = pts[mp[0]];
	Point pMax = pts[mp[0]];
	for (XFoam_Label i = 1; i < mp.size(); ++i)
	{
		const Point& p = pts[mp[i]];
		pMin = Point(
			std::min(pMin.x(), p.x()),
			std::min(pMin.y(), p.y()),
			std::min(pMin.z(), p.z()));
		pMax = Point(
			std::max(pMax.x(), p.x()),
			std::max(pMax.y(), p.y()),
			std::max(pMax.z(), p.z()));
	}
	return XFoam_Tuple2<Point, Point>(pMin, pMax);
}

template<class FaceList, class PointField>
inline XFoam_Scalar XFoam_PrimitivePatch<FaceList, PointField>::sphere(
	const XFoam_Label facei) const
{
	using Point = typename XFoam_PrimitivePatch<FaceList, PointField>::point_type;
	const Point fc = faceCentres()[facei];
	const LocalFaceList& lf = localFaces();
	const XFoam_LabelList& mp = meshPoints();
	const XFoam_Vector3DUList& pts = pointsUList_(points_);
	XFoam_Scalar r2 = 0;
	const face_type& f = lf[facei];
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		const Point d = pts[mp[f[i]]] - fc;
		const XFoam_Scalar m = static_cast<XFoam_Scalar>(d.magSqr());
		if (m > r2)
		{
			r2 = m;
		}
	}
	return r2;
}

template<class FaceList, class PointField>
inline void XFoam_PrimitivePatch<FaceList, PointField>::movePoints(
	const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&)
{
	clearGeom();
}

template<class FaceList, class PointField>
inline XFoam_PrimitivePatch<FaceList, PointField>& XFoam_PrimitivePatch<
	FaceList,
	PointField>::operator=(const XFoam_PrimitivePatch& pp)
{
	if (this != &pp)
	{
		clearOut();
		FaceList::operator=(static_cast<const FaceList&>(pp));
		XFoam_primitivePatchAssignPoints_<PointField>(
			points_,
			pp.points_,
			std::integral_constant<bool, !std::is_reference<PointField>::value>());
	}
	return *this;
}

template<class FaceList, class PointField>
inline XFoam_PrimitivePatch<FaceList, PointField>& XFoam_PrimitivePatch<
	FaceList,
	PointField>::operator=(XFoam_PrimitivePatch&& pp) noexcept
{
	if (this != &pp)
	{
		clearOut();
		FaceList::operator=(XFoam_move(static_cast<FaceList&>(pp)));
		XFoam_primitivePatchAssignPoints_<PointField>(
			points_,
			pp.points_,
			std::integral_constant<bool, !std::is_reference<PointField>::value>());
	}
	return *this;
}

template<class FaceList, class PointField>
template<class ToPatch>
inline typename XFoam_PrimitivePatch<FaceList, PointField>::ObjectHitList
XFoam_PrimitivePatch<FaceList, PointField>::projectPoints(
	const ToPatch&,
	const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&,
	const XFoam_Intersection::algorithm,
	const XFoam_Intersection::direction) const
{
	XFoam_FatalErrorInFunction
		<< "projectPoints requires OpenFOAM face::ray / objectHit pipeline; "
		   "XFoam_Face has no ray() yet."
		<< XFoam_abort(XFoam_FatalError);
	return {};
}

template<class FaceList, class PointField>
template<class ToPatch>
inline typename XFoam_PrimitivePatch<FaceList, PointField>::ObjectHitList
XFoam_PrimitivePatch<FaceList, PointField>::projectFaceCentres(
	const ToPatch&,
	const XFoam_Field<typename XFoam_PrimitivePatch<FaceList, PointField>::point_type>&,
	const XFoam_Intersection::algorithm,
	const XFoam_Intersection::direction) const
{
	XFoam_FatalErrorInFunction
		<< "projectFaceCentres requires OpenFOAM face::ray / bandCompression / "
		   "objectHit pipeline; not fully ported to XFoam."
		<< XFoam_abort(XFoam_FatalError);
	return {};
}

#endif
