#include "XFoam/mesh/xfoam_shape.h"
#include "XFoam/primitive/xfoam_pyramid.h"
#include "XFoam/primitive/xfoam_triangle.h"

namespace
{
void checkPointLabels(XFoam_Label need, const XFoam_UList<XFoam_Label>& pointLabels, const char* ctx)
{
	if (pointLabels.size() < need)
	{
		throw XFoam_Error(
			XFoam_String(ctx) + ": pointLabels.size() < nPoints");
	}
}

XFoam_Vector3D polyCentroid(const XFoam_List<XFoam_Vector3D>& r)
{
	XFoam_Vector3D s(XFoam_Zero_v);
	for (XFoam_Label i = 0; i < r.size(); ++i)
	{
		s += r[i];
	}
	return s / static_cast<XFoam_Scalar>(r.size());
}

XFoam_Vector3D fanAreaVector(const XFoam_List<XFoam_Vector3D>& r)
{
	const XFoam_Label n = r.size();
	XFoam_Vector3D a(XFoam_Zero_v);
	if (n < 3)
	{
		return a;
	}
	for (XFoam_Label i = 1; i < n - 1; ++i)
	{
		a += 0.5 * ((r[i] - r[0]) ^ (r[i + 1] - r[0]));
	}
	return a;
}

XFoam_Scalar pyramidMag(const XFoam_List<XFoam_Vector3D>& basePts, const XFoam_Vector3D& apex)
{
	const XFoam_Vector3D baseC = polyCentroid(basePts);
	const XFoam_Vector3D nRaw = fanAreaVector(basePts);
	const XFoam_Vector3D h = apex - baseC;
	const XFoam_Scalar ndh = nRaw & h;
	const XFoam_Vector3D n = (ndh > XFoam_small) ? -nRaw : nRaw;
	return (1.0 / 3.0) * (n & h);
}

XFoam_Vector3D pyramidCentre(const XFoam_List<XFoam_Vector3D>& basePts, const XFoam_Vector3D& apex)
{
	const XFoam_Vector3D baseC = polyCentroid(basePts);
	return 0.75 * baseC + 0.25 * apex;
}

// 对齐 OpenFOAM pyramidPointFaceRef：多边形底面 + 空间顶点（cell 分解用）。
struct XFoam_PyramidPointFaceRef
{
	const XFoam_Face& face_;
	XFoam_Vector3D apex_;

	XFoam_PyramidPointFaceRef(const XFoam_Face& f, const XFoam_Vector3D& apex)
		: face_(f)
		, apex_(apex)
	{}

	XFoam_Scalar mag(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		const XFoam_Vector3D a = face_.area(meshPoints);
		const XFoam_Vector3D bc = face_.centre(meshPoints);
		const XFoam_Vector3D h = apex_ - bc;
		return static_cast<XFoam_Scalar>((1.0 / 3.0) * (a & h));
	}

	XFoam_Vector3D centre(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		const XFoam_Vector3D bc = face_.centre(meshPoints);
		return bc * static_cast<XFoam_Scalar>(0.75) + apex_ * static_cast<XFoam_Scalar>(0.25);
	}
};

XFoam_List<XFoam_Vector3D> faceGlobalToPoints(
	const XFoam_LabelList& faceGlobal,
	const XFoam_UList<XFoam_Vector3D>& points)
{
	XFoam_List<XFoam_Vector3D> out(faceGlobal.size());
	for (XFoam_Label i = 0; i < faceGlobal.size(); ++i)
	{
		out[i] = points[faceGlobal[i]];
	}
	return out;
}

XFoam_CellModel makeHexCellModel()
{
	XFoam_List<XFoam_LabelList> f(6);
	f[0] = XFoam_LabelList({0, 3, 7, 4});
	f[1] = XFoam_LabelList({1, 2, 6, 5});
	f[2] = XFoam_LabelList({0, 1, 5, 4});
	f[3] = XFoam_LabelList({3, 2, 6, 7});
	f[4] = XFoam_LabelList({0, 1, 2, 3});
	f[5] = XFoam_LabelList({4, 5, 6, 7});
	XFoam_List<XFoam_FixedList<XFoam_Label, 2>> e(12);
	e[0] = XFoam_FixedList<XFoam_Label, 2>({0, 1});
	e[1] = XFoam_FixedList<XFoam_Label, 2>({1, 2});
	e[2] = XFoam_FixedList<XFoam_Label, 2>({2, 3});
	e[3] = XFoam_FixedList<XFoam_Label, 2>({3, 0});
	e[4] = XFoam_FixedList<XFoam_Label, 2>({4, 5});
	e[5] = XFoam_FixedList<XFoam_Label, 2>({5, 6});
	e[6] = XFoam_FixedList<XFoam_Label, 2>({6, 7});
	e[7] = XFoam_FixedList<XFoam_Label, 2>({7, 4});
	e[8] = XFoam_FixedList<XFoam_Label, 2>({0, 4});
	e[9] = XFoam_FixedList<XFoam_Label, 2>({1, 5});
	e[10] = XFoam_FixedList<XFoam_Label, 2>({2, 6});
	e[11] = XFoam_FixedList<XFoam_Label, 2>({3, 7});
	return XFoam_CellModel(XFoam_String("hex"), 0, 8, XFoam_move(f), XFoam_move(e));
}
const XFoam_CellModel& lookupCellModel(const XFoam_String& modelName)
{
	if (modelName == "hex")
	{
		return XFoam_CellModel::hex();
	}
	throw XFoam_Error(
		XFoam_String("XFoam_CellShape: unknown cell model \"") + modelName + XFoam_String("\""));
}
bool faceLabelsEqual(const XFoam_LabelList& a, const XFoam_LabelList& b)
{
	if (a.size() != b.size())
	{
		return false;
	}
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		if (a[i] != b[i])
		{
			return false;
		}
	}
	return true;
}
bool edgeEndpointsEqual(const XFoam_FixedList<XFoam_Label, 2>& a, const XFoam_FixedList<XFoam_Label, 2>& b)
{
	return (a[0] == b[0] && a[1] == b[1]) || (a[0] == b[1] && a[1] == b[0]);
}
} // namespace

XFoam_Label XFoam_Face::collapse()
{
	if (size() > 1)
	{
		XFoam_Label ci = 0;
		for (XFoam_Label i = 1; i < size(); ++i)
		{
			if (operator[](i) != operator[](ci))
			{
				operator[](++ci) = operator[](i);
			}
		}
		if (operator[](ci) != operator[](0))
		{
			++ci;
		}
		setSize(ci);
	}
	return size();
}

XFoam_Face XFoam_Face::reverseFace() const
{
	const XFoam_Label n = size();
	XFoam_Face out;
	out.setSize(n);
	if (n == 0)
	{
		return out;
	}
	out[0] = operator[](0);
	for (XFoam_Label pointi = 1; pointi < n; ++pointi)
	{
		out[pointi] = operator[](n - pointi);
	}
	return out;
}

XFoam_Label XFoam_Face::which(const XFoam_Label globalIndex) const
{
	for (XFoam_Label localIdx = 0; localIdx < size(); ++localIdx)
	{
		if (operator[](localIdx) == globalIndex)
		{
			return localIdx;
		}
	}
	return -1;
}

XFoam_Scalar XFoam_Face::sweptVol(
	const XFoam_UList<XFoam_Vector3D>& oldPoints,
	const XFoam_UList<XFoam_Vector3D>& newPoints) const
{
	if (size() < 3)
	{
		return 0;
	}
	XFoam_Scalar sv = 0;
	const XFoam_Vector3D centreOldPoint = centre(oldPoints);
	const XFoam_Vector3D centreNewPoint = centre(newPoints);
	const XFoam_Label nPoints = size();
	for (XFoam_Label pi = 0; pi < nPoints - 1; ++pi)
	{
		const XFoam_TrianglePoints tOld(
			centreOldPoint, oldPoints[operator[](pi)], oldPoints[operator[](pi + 1)]);
		const XFoam_TrianglePoints tNew(
			centreNewPoint, newPoints[operator[](pi)], newPoints[operator[](pi + 1)]);
		sv += tOld.sweptVol(tNew);
	}
	const XFoam_TrianglePoints tOld(
		centreOldPoint, oldPoints[operator[](nPoints - 1)], oldPoints[operator[](0)]);
	const XFoam_TrianglePoints tNew(
		centreNewPoint, newPoints[operator[](nPoints - 1)], newPoints[operator[](0)]);
	sv += tOld.sweptVol(tNew);
	return sv;
}

XFoam_List<XFoam_Edge> XFoam_Face::edges() const
{
	const XFoam_Label n = size();
	XFoam_List<XFoam_Edge> e(n);
	if (n < 2)
	{
		return e;
	}
	for (XFoam_Label pointi = 0; pointi < n - 1; ++pointi)
	{
		e[pointi] = XFoam_Edge(operator[](pointi), operator[](pointi + 1));
	}
	e[n - 1] = XFoam_Edge(operator[](n - 1), operator[](0));
	return e;
}

int XFoam_Face::edgeDirection(const XFoam_Edge& edg) const
{
	for (XFoam_Label i = 0; i < size(); ++i)
	{
		if (operator[](i) == edg.start())
		{
			if (operator[](rcIndex_(i)) == edg.end())
			{
				return -1;
			}
			if (operator[](fcIndex_(i)) == edg.end())
			{
				return 1;
			}
			return 0;
		}
		if (operator[](i) == edg.end())
		{
			if (operator[](rcIndex_(i)) == edg.start())
			{
				return 1;
			}
			if (operator[](fcIndex_(i)) == edg.start())
			{
				return -1;
			}
			return 0;
		}
	}
	return 0;
}

int XFoam_Face::compare(const XFoam_Face& a, const XFoam_Face& b)
{
	const XFoam_Label sizeA = a.size();
	const XFoam_Label sizeB = b.size();
	if (sizeA != sizeB || sizeA == 0)
	{
		return 0;
	}
	if (sizeA == 1)
	{
		return a[0] == b[0] ? 1 : 0;
	}

	XFoam_ConstCirculator<XFoam_LabelList> aCirc(a);
	XFoam_ConstCirculator<XFoam_LabelList> bCirc(b);

	do
	{
		if (aCirc() == bCirc())
		{
			bCirc.setFulcrumToIterator();
			++aCirc;
			++bCirc;
			break;
		}
	} while (bCirc.circulate(XFoam_CirculatorBase::direction::clockwise));

	if (!bCirc.circulate())
	{
		return 0;
	}

	do
	{
		if (aCirc() != bCirc())
		{
			break;
		}
	} while (
		aCirc.circulate(XFoam_CirculatorBase::direction::clockwise),
		bCirc.circulate(XFoam_CirculatorBase::direction::clockwise));

	if (!aCirc.circulate())
	{
		return 1;
	}
	else
	{
		aCirc.setIteratorToFulcrum();
		bCirc.setIteratorToFulcrum();
		++aCirc;
		--bCirc;

		do
		{
			if (aCirc() != bCirc())
			{
				break;
			}
		} while (
			aCirc.circulate(XFoam_CirculatorBase::direction::clockwise),
			bCirc.circulate(XFoam_CirculatorBase::direction::anticlockwise));

		if (!aCirc.circulate())
		{
			return -1;
		}

		return 0;
	}
}

bool XFoam_Face::sameVertices(const XFoam_Face& a, const XFoam_Face& b)
{
	const XFoam_Label sizeA = a.size();
	const XFoam_Label sizeB = b.size();
	if (sizeA != sizeB)
	{
		return false;
	}
	if (sizeA == 1)
	{
		return a[0] == b[0];
	}
	for (XFoam_Label i = 0; i < sizeA; ++i)
	{
		XFoam_Label aOcc = 0;
		for (XFoam_Label j = 0; j < sizeA; ++j)
		{
			if (a[i] == a[j])
			{
				++aOcc;
			}
		}
		XFoam_Label bOcc = 0;
		for (XFoam_Label j = 0; j < sizeB; ++j)
		{
			if (a[i] == b[j])
			{
				++bOcc;
			}
		}
		if (aOcc != bOcc)
		{
			return false;
		}
	}
	return true;
}

XFoam_Label XFoam_longestEdge(const XFoam_Face& f, const XFoam_UList<XFoam_Vector3D>& pts)
{
	const XFoam_List<XFoam_Edge> eds = f.edges();
	XFoam_Label longestEdgeI = -1;
	XFoam_Scalar longestEdgeLength = -XFoam_small;
	for (XFoam_Label edI = 0; edI < eds.size(); ++edI)
	{
		const XFoam_Scalar edgeLength = eds[edI].mag(pts);
		if (edgeLength > longestEdgeLength)
		{
			longestEdgeI = edI;
			longestEdgeLength = edgeLength;
		}
	}
	return longestEdgeI;
}

XFoam_LabelList XFoam_Cell::labels(
	const XFoam_Cell& c, const XFoam_UList<XFoam_Face>& meshFaces)
{
	if (c.size() == 0)
	{
		return XFoam_LabelList();
	}
	XFoam_Label maxVert = 0;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		maxVert += meshFaces[c[facei]].size();
	}
	XFoam_LabelList p(maxVert);
	const XFoam_Face& firstFace = meshFaces[c[0]];
	for (XFoam_Label pointi = 0; pointi < firstFace.size(); ++pointi)
	{
		p[pointi] = firstFace[pointi];
	}
	maxVert = firstFace.size();
	for (XFoam_Label facei = 1; facei < c.size(); ++facei)
	{
		const XFoam_Face& curFace = meshFaces[c[facei]];
		for (XFoam_Label pointi = 0; pointi < curFace.size(); ++pointi)
		{
			const XFoam_Label curPoint = curFace[pointi];
			bool foundDup = false;
			for (XFoam_Label checkI = 0; checkI < maxVert; ++checkI)
			{
				if (curPoint == p[checkI])
				{
					foundDup = true;
					break;
				}
			}
			if (!foundDup)
			{
				p[maxVert] = curPoint;
				++maxVert;
			}
		}
	}
	p.setSize(maxVert);
	return p;
}

XFoam_List<XFoam_Vector3D> XFoam_Cell::points(
	const XFoam_Cell& c,
	const XFoam_UList<XFoam_Face>& meshFaces,
	const XFoam_UList<XFoam_Vector3D>& meshPoints)
{
	const XFoam_LabelList pointLabels = labels(c, meshFaces);
	XFoam_List<XFoam_Vector3D> out(pointLabels.size());
	for (XFoam_Label i = 0; i < pointLabels.size(); ++i)
	{
		out[i] = meshPoints[pointLabels[i]];
	}
	return out;
}

XFoam_List<XFoam_Edge> XFoam_Cell::edges(
	const XFoam_Cell& c, const XFoam_UList<XFoam_Face>& meshFaces)
{
	XFoam_Label maxNoEdges = 0;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		maxNoEdges += meshFaces[c[facei]].nEdges();
	}
	XFoam_List<XFoam_Edge> allEdges(maxNoEdges);
	XFoam_Label nEdge = 0;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		const XFoam_List<XFoam_Edge> curFaceEdges = meshFaces[c[facei]].edges();
		for (XFoam_Label faceEdgeI = 0; faceEdgeI < curFaceEdges.size(); ++faceEdgeI)
		{
			const XFoam_Edge& curEdge = curFaceEdges[faceEdgeI];
			bool edgeFound = false;
			for (XFoam_Label addedEdgeI = 0; addedEdgeI < nEdge; ++addedEdgeI)
			{
				if (allEdges[addedEdgeI] == curEdge)
				{
					edgeFound = true;
					break;
				}
			}
			if (!edgeFound)
			{
				allEdges[nEdge] = curEdge;
				++nEdge;
			}
		}
	}
	allEdges.setSize(nEdge);
	return allEdges;
}

XFoam_Vector3D XFoam_Cell::centre(
	const XFoam_Cell& c,
	const XFoam_UList<XFoam_Vector3D>& meshPoints,
	const XFoam_UList<XFoam_Face>& meshFaces)
{
	XFoam_Vector3D cEst(XFoam_Zero_v);
	XFoam_Scalar sumArea = 0.0;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		const XFoam_Face& f = meshFaces[c[facei]];
		const XFoam_Scalar a = f.mag(meshPoints);
		cEst += f.centre(meshPoints) * a;
		sumArea += a;
	}
	cEst /= sumArea + XFoam_small;
	XFoam_Vector3D sumVc(XFoam_Zero_v);
	XFoam_Scalar sumV = 0.0;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		const XFoam_Face& f = meshFaces[c[facei]];
		const XFoam_PyramidPointFaceRef pyr(f, cEst);
		XFoam_Scalar pyrVol = pyr.mag(meshPoints);
		const XFoam_Vector3D pyrC = pyr.centre(meshPoints);
		if (pyrVol < 0.0)
		{
			pyrVol = -pyrVol;
		}
		sumVc += pyrVol * pyrC;
		sumV += pyrVol;
	}
	return sumVc / (sumV + XFoam_small);
}

XFoam_Scalar XFoam_Cell::mag(
	const XFoam_Cell& c,
	const XFoam_UList<XFoam_Vector3D>& meshPoints,
	const XFoam_UList<XFoam_Face>& meshFaces)
{
	XFoam_Vector3D cEst(XFoam_Zero_v);
	XFoam_Scalar nCellFaces = 0.0;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		cEst += meshFaces[c[facei]].centre(meshPoints);
		nCellFaces += 1.0;
	}
	cEst /= nCellFaces;
	XFoam_Scalar v = 0.0;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		v += XFoam_mag(
			XFoam_PyramidPointFaceRef(meshFaces[c[facei]], cEst).mag(meshPoints));
	}
	return v;
}

XFoam_BoundBox XFoam_Cell::bb(
	const XFoam_Cell& c,
	const XFoam_UList<XFoam_Vector3D>& meshPoints,
	const XFoam_UList<XFoam_Face>& meshFaces)
{
	XFoam_BoundBox result = XFoam_BoundBox::invertedBox;
	for (XFoam_Label cfi = 0; cfi < c.size(); ++cfi)
	{
		const XFoam_Face& f = meshFaces[c[cfi]];
		for (XFoam_Label fpi = 0; fpi < f.size(); ++fpi)
		{
			const XFoam_Vector3D& p = meshPoints[f[fpi]];
			XFoam_Vector3D& mn = result.min();
			XFoam_Vector3D& mx = result.max();
			mn.x() = std::min(mn.x(), p.x());
			mn.y() = std::min(mn.y(), p.y());
			mn.z() = std::min(mn.z(), p.z());
			mx.x() = std::max(mx.x(), p.x());
			mx.y() = std::max(mx.y(), p.y());
			mx.z() = std::max(mx.z(), p.z());
		}
	}
	return result;
}

XFoam_Label XFoam_Cell::opposingFaceLabel(
	const XFoam_Cell& c,
	const XFoam_Label masterFaceLabel,
	const XFoam_UList<XFoam_Face>& meshFaces)
{
	const XFoam_Face& masterFace = meshFaces[masterFaceLabel];
	XFoam_Label oppositeFaceLabel = -1;
	for (XFoam_Label facei = 0; facei < c.size(); ++facei)
	{
		const XFoam_Face& curFace = meshFaces[c[facei]];
		if (c[facei] != masterFaceLabel && curFace.size() == masterFace.size())
		{
			bool sharedPoint = false;
			for (XFoam_Label pointi = 0; pointi < curFace.size(); ++pointi)
			{
				const XFoam_Label l = curFace[pointi];
				for (XFoam_Label masterPointi = 0; masterPointi < masterFace.size(); ++masterPointi)
				{
					if (masterFace[masterPointi] == l)
					{
						sharedPoint = true;
						break;
					}
				}
				if (sharedPoint)
				{
					break;
				}
			}
			if (!sharedPoint)
			{
				if (oppositeFaceLabel == -1)
				{
					oppositeFaceLabel = c[facei];
				}
				else
				{
					return -2;
				}
			}
		}
	}
	return oppositeFaceLabel;
}

XFoam_OppositeFace XFoam_Cell::opposingFace(
	const XFoam_Cell& c,
	const XFoam_Label masterFaceLabel,
	const XFoam_UList<XFoam_Face>& meshFaces)
{
	const XFoam_Label oppFaceLabel = opposingFaceLabel(c, masterFaceLabel, meshFaces);
	if (oppFaceLabel < 0)
	{
		return XFoam_OppositeFace(XFoam_Face(), masterFaceLabel, oppFaceLabel);
	}
	const XFoam_Face& masterFace = meshFaces[masterFaceLabel];
	const XFoam_Face& slaveFace = meshFaces[oppFaceLabel];
	const XFoam_List<XFoam_Edge> e = edges(c, meshFaces);
	std::vector<bool> usedEdges(static_cast<XFoam_Size>(e.size()), false);
	XFoam_Face oppFaceData(static_cast<XFoam_Label>(masterFace.size()));
	for (XFoam_Label pointi = 0; pointi < masterFace.size(); ++pointi)
	{
		for (XFoam_Label edgeI = 0; edgeI < e.size(); ++edgeI)
		{
			if (!usedEdges[static_cast<XFoam_Size>(edgeI)])
			{
				const XFoam_Label otherVertex = e[edgeI].otherVertex(masterFace[pointi]);
				if (otherVertex != -1)
				{
					for (XFoam_Label slavePointi = 0; slavePointi < slaveFace.size(); ++slavePointi)
					{
						if (slaveFace[slavePointi] == otherVertex)
						{
							usedEdges[static_cast<XFoam_Size>(edgeI)] = true;
							oppFaceData[pointi] = otherVertex;
							break;
						}
					}
				}
			}
		}
	}
	return XFoam_OppositeFace(XFoam_move(oppFaceData), masterFaceLabel, oppFaceLabel);
}

bool operator==(const XFoam_Cell& a, const XFoam_Cell& b)
{
	if (a.size() != b.size())
	{
		return false;
	}
	std::vector<char> fnd(static_cast<XFoam_Size>(a.size()), 0);
	for (XFoam_Label bI = 0; bI < b.size(); ++bI)
	{
		const XFoam_Label curLabel = b[bI];
		bool found = false;
		for (XFoam_Label aI = 0; aI < a.size(); ++aI)
		{
			if (a[aI] == curLabel)
			{
				found = true;
				fnd[static_cast<XFoam_Size>(aI)] = 1;
				break;
			}
		}
		if (!found)
		{
			return false;
		}
	}
	for (XFoam_Label aI = 0; aI < a.size(); ++aI)
	{
		if (!fnd[static_cast<XFoam_Size>(aI)])
		{
			return false;
		}
	}
	return true;
}

XFoam_CellModel::XFoam_CellModel(
	XFoam_String name,
	XFoam_Label index,
	XFoam_Label nPoints,
	XFoam_List<XFoam_LabelList> faces,
	XFoam_List<XFoam_FixedList<XFoam_Label, 2>> edges)
	: name_(XFoam_move(name))
	, index_(index)
	, nPoints_(nPoints)
	, faces_(XFoam_move(faces))
	, edges_(XFoam_move(edges))
{}
XFoam_List<XFoam_FixedList<XFoam_Label, 2>> XFoam_CellModel::edges(
	const XFoam_UList<XFoam_Label>& pointLabels) const
{
	checkPointLabels(nPoints_, pointLabels, "XFoam_CellModel::edges");
	XFoam_List<XFoam_FixedList<XFoam_Label, 2>> e(edges_.size());
	for (XFoam_Label ei = 0; ei < edges_.size(); ++ei)
	{
		const XFoam_Label a = pointLabels[edges_[ei][0]];
		const XFoam_Label b = pointLabels[edges_[ei][1]];
		e[ei] = XFoam_FixedList<XFoam_Label, 2>({a, b});
	}
	return e;
}
XFoam_List<XFoam_LabelList> XFoam_CellModel::faces(
	const XFoam_UList<XFoam_Label>& pointLabels) const
{
	checkPointLabels(nPoints_, pointLabels, "XFoam_CellModel::faces");
	XFoam_List<XFoam_LabelList> f(faces_.size());
	for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
	{
		const XFoam_LabelList& loc = faces_[fi];
		XFoam_LabelList g(loc.size());
		for (XFoam_Label i = 0; i < loc.size(); ++i)
		{
			g[i] = pointLabels[loc[i]];
		}
		f[fi] = XFoam_move(g);
	}
	return f;
}
XFoam_Vector3D XFoam_CellModel::centre(
	const XFoam_UList<XFoam_Label>& pointLabels,
	const XFoam_UList<XFoam_Vector3D>& points) const
{
	checkPointLabels(nPoints_, pointLabels, "XFoam_CellModel::centre");
	XFoam_Vector3D cEst(XFoam_Zero_v);
	for (XFoam_Label i = 0; i < nPoints_; ++i)
	{
		cEst += points[pointLabels[i]];
	}
	cEst /= static_cast<XFoam_Scalar>(nPoints_);
	const XFoam_List<XFoam_LabelList> cellFaces = faces(pointLabels);
	XFoam_Scalar sumV = 0.0;
	XFoam_Vector3D sumVc(XFoam_Zero_v);
	for (XFoam_Label fi = 0; fi < cellFaces.size(); ++fi)
	{
		const XFoam_List<XFoam_Vector3D> basePts = faceGlobalToPoints(cellFaces[fi], points);
		const XFoam_Scalar pyrVol = pyramidMag(basePts, cEst);
		sumVc -= pyrVol * pyramidCentre(basePts, cEst);
		sumV -= pyrVol;
	}
	return sumVc / (sumV + XFoam_small);
}
XFoam_Scalar XFoam_CellModel::mag(
	const XFoam_UList<XFoam_Label>& pointLabels,
	const XFoam_UList<XFoam_Vector3D>& points) const
{
	checkPointLabels(nPoints_, pointLabels, "XFoam_CellModel::mag");
	XFoam_Vector3D cEst(XFoam_Zero_v);
	for (XFoam_Label i = 0; i < nPoints_; ++i)
	{
		cEst += points[pointLabels[i]];
	}
	cEst /= static_cast<XFoam_Scalar>(nPoints_);
	const XFoam_List<XFoam_LabelList> cellFaces = faces(pointLabels);
	XFoam_Scalar v = 0.0;
	for (XFoam_Label fi = 0; fi < cellFaces.size(); ++fi)
	{
		const XFoam_List<XFoam_Vector3D> basePts = faceGlobalToPoints(cellFaces[fi], points);
		const XFoam_Scalar pyrVol = pyramidMag(basePts, cEst);
		v -= pyrVol;
	}
	return v;
}
XFoam_AutoPtr<XFoam_CellModel> XFoam_CellModel::clone() const
{
	return XFoam_AutoPtr<XFoam_CellModel>(new XFoam_CellModel(*this));
}
const XFoam_CellModel& XFoam_CellModel::hex()
{
	static const XFoam_CellModel inst(makeHexCellModel());
	return inst;
}
XFoam_CellShape::XFoam_CellShape()
	: XFoam_LabelList()
	, m_(nullptr)
{}
XFoam_CellShape::XFoam_CellShape(
	const XFoam_CellModel& model,
	const XFoam_LabelList& labels,
	const bool doCollapse)
	: XFoam_LabelList(labels)
	, m_(&model)
{
	if (doCollapse)
	{
		collapse();
	}
}
XFoam_CellShape::XFoam_CellShape(
	const XFoam_String& modelName,
	const XFoam_LabelList& labels,
	const bool doCollapse)
	: XFoam_LabelList(labels)
	, m_(&lookupCellModel(modelName))
{
	if (doCollapse)
	{
		collapse();
	}
}
XFoam_CellShape::XFoam_CellShape(
	const XFoam_String& modelName,
	const XFoam_FixedList<XFoam_Label, 8>& labels)
	: XFoam_LabelList(8)
	, m_(&lookupCellModel(modelName))
{
	for (XFoam_Label i = 0; i < 8; ++i)
	{
		(*this)[i] = labels[static_cast<unsigned>(i)];
	}
}
XFoam_List<XFoam_Vector3D> XFoam_CellShape::points(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
{
	const XFoam_Label n = XFoam_LabelList::size();
	XFoam_List<XFoam_Vector3D> p(n);
	for (XFoam_Label i = 0; i < n; ++i)
	{
		p[i] = meshPoints[(*this)[i]];
	}
	return p;
}
const XFoam_CellModel& XFoam_CellShape::model() const
{
	if (!m_)
	{
		throw XFoam_Error(XFoam_String("XFoam_CellShape::model: null cell model"));
	}
	return *m_;
}
XFoam_LabelList XFoam_CellShape::meshFaces(
	const XFoam_List<XFoam_LabelList>& allFaces,
	const XFoam_LabelList& cellFaceIndices) const
{
	const XFoam_List<XFoam_LabelList> localFaces = faces();
	XFoam_LabelList modelToMesh(localFaces.size(), XFoam_Label(-1));
	for (XFoam_Label i = 0; i < localFaces.size(); ++i)
	{
		const XFoam_LabelList& localF = localFaces[i];
		for (XFoam_Label j = 0; j < cellFaceIndices.size(); ++j)
		{
			const XFoam_Label meshFacei = cellFaceIndices[j];
			if (meshFacei >= 0 && meshFacei < allFaces.size()
				&& faceLabelsEqual(allFaces[meshFacei], localF))
			{
				modelToMesh[i] = meshFacei;
				break;
			}
		}
	}
	return modelToMesh;
}
XFoam_LabelList XFoam_CellShape::meshEdges(
	const XFoam_List<XFoam_FixedList<XFoam_Label, 2>>& allEdges,
	const XFoam_LabelList& cellEdgeIndices) const
{
	const XFoam_List<XFoam_FixedList<XFoam_Label, 2>> localEdges = edges();
	XFoam_LabelList modelToMesh(localEdges.size(), XFoam_Label(-1));
	for (XFoam_Label i = 0; i < localEdges.size(); ++i)
	{
		const XFoam_FixedList<XFoam_Label, 2>& e = localEdges[i];
		for (XFoam_Label j = 0; j < cellEdgeIndices.size(); ++j)
		{
			const XFoam_Label edgeI = cellEdgeIndices[j];
			if (edgeI >= 0 && edgeI < allEdges.size() && edgeEndpointsEqual(allEdges[edgeI], e))
			{
				modelToMesh[i] = edgeI;
				break;
			}
		}
	}
	return modelToMesh;
}
XFoam_List<XFoam_LabelList> XFoam_CellShape::faces() const
{
	return model().faces(static_cast<const XFoam_UList<XFoam_Label>&>(*this));
}
XFoam_List<XFoam_LabelList> XFoam_CellShape::collapsedFaces() const
{
	const XFoam_List<XFoam_LabelList> oldFaces = faces();
	XFoam_List<XFoam_LabelList> newFaces(oldFaces.size());
	XFoam_Label newFacei = 0;
	for (XFoam_Label oldFacei = 0; oldFacei < oldFaces.size(); ++oldFacei)
	{
		const XFoam_LabelList& f = oldFaces[oldFacei];
		XFoam_LabelList newF(f.size());
		XFoam_Label newFp = 0;
		XFoam_Label prevVertI = -1;
		for (XFoam_Label fp = 0; fp < f.size(); ++fp)
		{
			const XFoam_Label vertI = f[fp];
			if (vertI != prevVertI)
			{
				newF[newFp++] = vertI;
				prevVertI = vertI;
			}
		}
		if ((newFp > 1) && (newF[newFp - 1] == newF[0]))
		{
			--newFp;
		}
		if (newFp > 2)
		{
			newF.setSize(newFp);
			newFaces[newFacei++] = XFoam_move(newF);
		}
	}
	newFaces.setSize(newFacei);
	return newFaces;
}
XFoam_Label XFoam_CellShape::nFaces() const
{
	return model().nFaces();
}
XFoam_List<XFoam_FixedList<XFoam_Label, 2>> XFoam_CellShape::edges() const
{
	return model().edges(static_cast<const XFoam_UList<XFoam_Label>&>(*this));
}
XFoam_Label XFoam_CellShape::nEdges() const
{
	return model().nEdges();
}
XFoam_Label XFoam_CellShape::nPoints() const
{
	return XFoam_LabelList::size();
}
XFoam_Vector3D XFoam_CellShape::centre(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
{
	return model().centre(static_cast<const XFoam_UList<XFoam_Label>&>(*this), meshPoints);
}
XFoam_Scalar XFoam_CellShape::mag(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
{
	return model().mag(static_cast<const XFoam_UList<XFoam_Label>&>(*this), meshPoints);
}
void XFoam_CellShape::collapse()
{
	// OpenFOAM：operator=(degenerateMatcher::match(*this))；degenerateMatcher 未移植时保持不修改。
}
XFoam_AutoPtr<XFoam_CellShape> XFoam_CellShape::clone() const
{
	return XFoam_AutoPtr<XFoam_CellShape>(new XFoam_CellShape(*this));
}
XFoam_FixedList<XFoam_Label, 4> XFoam_CellShape::faceVertexLabels(const XFoam_Label facei) const
{
	const XFoam_CellModel& cm = model();
	if (facei < 0 || facei >= cm.nFaces())
	{
		throw XFoam_Error(XFoam_String("XFoam_CellShape::faceVertexLabels: facei out of range"));
	}
	const XFoam_LabelList& loc = cm.modelFaces()[facei];
	if (loc.size() != 4)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_CellShape::faceVertexLabels: only quad faces are supported"));
	}
	XFoam_FixedList<XFoam_Label, 4> out;
	for (unsigned e = 0; e < 4u; ++e)
	{
		out[e] = (*this)[loc[e]];
	}
	return out;
}
XFoam_API bool operator==(const XFoam_CellShape& a, const XFoam_CellShape& b)
{
	const XFoam_UList<XFoam_Label>& labsA = a.pointLabelUList();
	const XFoam_UList<XFoam_Label>& labsB = b.pointLabelUList();
	XFoam_Label sizeA = labsA.size();
	XFoam_Label sizeB = labsB.size();
	if (sizeA != sizeB)
	{
		return false;
	}
	XFoam_Label Bptr = -1;
	const XFoam_Label firstA = labsA[0];
	for (XFoam_Label i = 0; i < labsB.size(); ++i)
	{
		if (labsB[i] == firstA)
		{
			Bptr = i;
			break;
		}
	}
	if (Bptr < 0)
	{
		return false;
	}
	const XFoam_Label secondA = labsA[1];
	XFoam_Label dir = 0;
	Bptr++;
	if (Bptr == labsB.size())
	{
		Bptr = 0;
	}
	if (labsB[Bptr] == secondA)
	{
		dir = 1;
	}
	else
	{
		Bptr -= 2;
		if (Bptr < 0)
		{
			if (Bptr == -1)
			{
				Bptr = labsB.size() - 1;
			}
			else
			{
				Bptr = labsB.size() - 2;
			}
		}
		if (labsB[Bptr] == secondA)
		{
			dir = -1;
		}
	}
	if (dir == 0)
	{
		return false;
	}
	sizeA -= 2;
	XFoam_Label Aptr = 1;
	if (dir > 0)
	{
		while (sizeA--)
		{
			Aptr++;
			if (Aptr >= labsA.size())
			{
				Aptr = 0;
			}
			Bptr++;
			if (Bptr >= labsB.size())
			{
				Bptr = 0;
			}
			if (labsA[Aptr] != labsB[Bptr])
			{
				return false;
			}
		}
	}
	else
	{
		while (sizeA--)
		{
			Aptr++;
			if (Aptr >= labsA.size())
			{
				Aptr = 0;
			}
			Bptr--;
			if (Bptr < 0)
			{
				Bptr = labsB.size() - 1;
			}
			if (labsA[Aptr] != labsB[Bptr])
			{
				return false;
			}
		}
	}
	return true;
}
XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_CellModel& m)
{
	os << m.name() << ' ' << m.index() << ' ' << m.nPoints() << ' ' << m.nFaces() << ' '
	   << m.nEdges();
	return os;
}
XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_CellShape& s)
{
	os << '(' << s.model().index();
	for (XFoam_Label i = 0; i < s.labels().size(); ++i)
	{
		os << ' ' << s.labels()[i];
	}
	os << ')';
	return os;
}