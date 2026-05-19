#include "XFoam/mesh/xfoam_primitivemesh.h"

// 移植源码: OpenFOAM-13 meshes/primitiveMesh/primitiveMesh（clearOut / movePoints / reset 子集）。
// 命名规范: foam_code.md
// 移植规范: foam_code.md

// * * * * * * * * * * * * * * XFoam_PrimitiveMesh * * * * * * * * * * * * //

XFoam_PrimitiveMesh::XFoam_PrimitiveMesh()
	: nInternalPoints_(0)
	, nPoints_(0)
	, nEdges_(-1)
	, nInternalFaces_(0)
	, nFaces_(0)
	, nCells_(0)
{}

XFoam_PrimitiveMesh::XFoam_PrimitiveMesh(
	const XFoam_Label nPoints,
	const XFoam_Label nInternalFaces,
	const XFoam_Label nFaces,
	const XFoam_Label nCells)
	: nInternalPoints_(-1)
	, nPoints_(nPoints)
	, nEdges_(-1)
	, nInternalFaces_(nInternalFaces)
	, nFaces_(nFaces)
	, nCells_(nCells)
{}

XFoam_PrimitiveMesh::~XFoam_PrimitiveMesh()
{
	clearOut();
}

void XFoam_PrimitiveMesh::clearOut()
{
	clearGeom();
}

void XFoam_PrimitiveMesh::reset(
	const XFoam_Label nPoints,
	const XFoam_Label nInternalFaces,
	const XFoam_Label nFaces,
	const XFoam_Label nCells)
{
	clearOut();

	nPoints_ = nPoints;
	nEdges_ = -1;
	nInternalFaces_ = nInternalFaces;
	nFaces_ = nFaces;
	nCells_ = nCells;

	XFoam_Label nInternalPts = 0;
	XFoam_LabelList pointMap;
	const bool ordered = calcPointOrder(
		nInternalPts,
		pointMap,
		faces(),
		nInternalFaces_,
		nPoints_);

	nInternalPoints_ = ordered ? nInternalPts : static_cast<XFoam_Label>(-1);
}

XFoam_ScalarList XFoam_PrimitiveMesh::movePoints(
	const XFoam_Vector3DUList& newPoints,
	const XFoam_Vector3DUList& oldPoints)
{
	if (newPoints.size() < nPoints() || oldPoints.size() < nPoints())
	{
		throw XFoam_Error(
			XFoam_String("XFoam_PrimitiveMesh::movePoints: point list smaller than nPoints()"));
	}

	const XFoam_FaceUList& f = faces();
	XFoam_ScalarList swept(f.size());
	for (XFoam_Label facei = 0; facei < f.size(); ++facei)
	{
		swept[facei] = f[facei].sweptVol(oldPoints, newPoints);
	}
	clearGeom();
	return swept;
}

bool XFoam_PrimitiveMesh::calcPointOrder(
	XFoam_Label& nInternalPoints,
	XFoam_LabelList& oldToNew,
	const XFoam_FaceUList& meshFaces,
	const XFoam_Label nInternalFaces,
	const XFoam_Label nPoints)
{
	oldToNew.setSize(nPoints);
	for (XFoam_Label i = 0; i < nPoints; ++i)
	{
		oldToNew[i] = static_cast<XFoam_Label>(-1);
	}

	XFoam_Label nBoundaryPoints = 0;
	for (XFoam_Label facei = nInternalFaces; facei < meshFaces.size(); ++facei)
	{
		const XFoam_Face& f = meshFaces[facei];
		for (XFoam_Label fp = 0; fp < f.size(); ++fp)
		{
			const XFoam_Label pointi = f[fp];
			if (oldToNew[pointi] == static_cast<XFoam_Label>(-1))
			{
				oldToNew[pointi] = nBoundaryPoints++;
			}
		}
	}

	nInternalPoints = nPoints - nBoundaryPoints;

	for (XFoam_Label pointi = 0; pointi < nPoints; ++pointi)
	{
		if (oldToNew[pointi] != static_cast<XFoam_Label>(-1))
		{
			oldToNew[pointi] += nInternalPoints;
		}
	}

	XFoam_Label internalPointi = 0;
	bool ordered = true;

	for (XFoam_Label facei = 0; facei < nInternalFaces; ++facei)
	{
		const XFoam_Face& f = meshFaces[facei];
		for (XFoam_Label fp = 0; fp < f.size(); ++fp)
		{
			const XFoam_Label pointi = f[fp];
			if (oldToNew[pointi] == static_cast<XFoam_Label>(-1))
			{
				if (pointi >= nInternalPoints)
				{
					ordered = false;
				}
				oldToNew[pointi] = internalPointi++;
			}
		}
	}

	return ordered;
}

void XFoam_PrimitiveMesh::calcCells(
	XFoam_CellList& cellFaces,
	const XFoam_LabelUList& own,
	const XFoam_LabelUList& nei,
	const XFoam_Label nCells)
{
	if (own.size() != nei.size())
	{
		throw XFoam_Error(
			XFoam_String("XFoam_PrimitiveMesh::calcCells: faceOwner and faceNeighbour size mismatch"));
	}

	cellFaces.setSize(nCells);
	for (XFoam_Label celli = 0; celli < nCells; ++celli)
	{
		cellFaces[celli].clear();
	}

	XFoam_LabelList nPerCell(nCells, 0);

	for (XFoam_Label facei = 0; facei < own.size(); ++facei)
	{
		const XFoam_Label o = own[facei];
		if (o < 0 || o >= nCells)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_PrimitiveMesh::calcCells: faceOwner out of range"));
		}
		nPerCell[o]++;
	}

	for (XFoam_Label facei = 0; facei < nei.size(); ++facei)
	{
		const XFoam_Label n = nei[facei];
		if (n >= 0)
		{
			if (n >= nCells)
			{
				throw XFoam_Error(
					XFoam_String("XFoam_PrimitiveMesh::calcCells: faceNeighbour out of range"));
			}
			nPerCell[n]++;
		}
	}

	for (XFoam_Label celli = 0; celli < nCells; ++celli)
	{
		cellFaces[celli].setSize(nPerCell[celli]);
	}

	for (XFoam_Label celli = 0; celli < nCells; ++celli)
	{
		nPerCell[celli] = 0;
	}

	for (XFoam_Label facei = 0; facei < own.size(); ++facei)
	{
		const XFoam_Label c = own[facei];
		cellFaces[c][nPerCell[c]++] = facei;
	}

	for (XFoam_Label facei = 0; facei < nei.size(); ++facei)
	{
		const XFoam_Label c = nei[facei];
		if (c >= 0)
		{
			cellFaces[c][nPerCell[c]++] = facei;
		}
	}
}
