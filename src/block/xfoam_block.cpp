#include "XFoam/block/xfoam_block.h"
#include "XFoam/block/xfoam_blockmesh.h"
#include <array>
#include <algorithm>

namespace
{
bool sameVertices4(
	const XFoam_UList<XFoam_Label>& a,
	const XFoam_FixedList<XFoam_Label, 4>& b)
{
	if (a.size() != 4)
	{
		return false;
	}
	std::array<XFoam_Label, 4> sa{};
	std::array<XFoam_Label, 4> sb{};
	for (int i = 0; i < 4; ++i)
	{
		sa[static_cast<XFoam_Size>(i)] = a[i];
		sb[static_cast<XFoam_Size>(i)] = b[static_cast<unsigned>(i)];
	}
	std::sort(sa.begin(), sa.end());
	std::sort(sb.begin(), sb.end());
	return sa == sb;
}

// 对标 OpenFOAM blockDescriptor.C：readExpansionList 对应 List<gradingDescriptors>(is)。
XFoam_List<XFoam_GradingDescriptors> readExpansionGradingList(XFoam_IStream& is)
{
	XFoam_List<XFoam_GradingDescriptors> expRatios;
	XFoam_Token open(is);
	if (open.isPunctuation() && open.pToken() == XFoam_Token::BEGIN_LIST)
	{
		for (;;)
		{
			XFoam_Token t(is);
			if (t.good() && t.isPunctuation() && t.pToken() == XFoam_Token::END_LIST)
			{
				break;
			}
			if (!t.good() || t.error())
			{
				XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
					<< "unexpected token reading expansion ratios list"
					<< XFoam_exit(XFoam_FatalIOError, 1);
			}
			is.putBack(t);
			XFoam_GradingDescriptors gd;
			is >> gd;
			expRatios.append(gd);
		}
	}
	else
	{
		is.putBack(open);
		XFoam_GradingDescriptors gd;
		is >> gd;
		expRatios.append(gd);
	}
	return expRatios;
}

XFoam_Label readHexVertexLabel(XFoam_IStream& is, const XFoam_Dictionary& dict)
{
	(void)dict;
	XFoam_Token tok(is);
	if (tok.isLabel())
	{
		return tok.labelToken();
	}
	if (tok.isWord())
	{
		// 未移植：Foam::blockMeshTools::read + dict.subDict("namedVertices") 变量名展开。
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
			<< "namedVertices / word vertex id not supported yet: " << tok.wordToken()
			<< XFoam_exit(XFoam_FatalIOError, 1);
	}
	XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
		<< "expected label for hex vertex index"
		<< XFoam_exit(XFoam_FatalIOError, 1);
	return 0;
}

void checkVertexIndices_(
	const XFoam_CellShape& shape,
	const XFoam_UList<XFoam_Vector3D>& vertices,
	XFoam_IStream& is)
{
	const XFoam_Label n = shape.size();
	for (XFoam_Label pi = 0; pi < n; ++pi)
	{
		const XFoam_Label vid = shape[pi];
		if (vid < 0 || vid >= vertices.size())
		{
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
				<< "Point label " << vid << " out of range 0.." << (vertices.size() - 1)
				<< XFoam_exit(XFoam_FatalIOError, 1);
		}
	}
}

XFoam_BlockDescriptor makeBlockDescriptorFromMeshStream(
	const XFoam_Dictionary& dict,
	const XFoam_PointField& vertices,
	const XFoam_BlockEdgeList& edges,
	const XFoam_BlockFaceList& faces,
	XFoam_IStream& is)
{
	XFoam_Word model;
	is >> model;
	if (model != XFoam_Word("hex"))
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
			<< "XFoam_Block: only cell model 'hex' is supported, got " << model
			<< XFoam_exit(XFoam_FatalIOError, 1);
	}
	(void)is.readBeginList("XFoam_Block hex vertices");
	XFoam_FixedList<XFoam_Label, 8> vlab;
	for (unsigned i = 0; i < 8u; ++i)
	{
		vlab[static_cast<unsigned>(i)] = readHexVertexLabel(is, dict);
	}
	(void)is.readEndList("XFoam_Block hex vertices");
	const XFoam_CellShape blockShape(static_cast<const XFoam_String&>(model), vlab);
	checkVertexIndices_(blockShape, vertices, is);

	XFoam_String zoneName;
	XFoam_Token t(is);
	if (t.isWord())
	{
		zoneName = XFoam_String(static_cast<const XFoam_String&>(t.wordToken()));
		is >> t;
	}
	is.putBack(t);

	XFoam_Vector<XFoam_Label> density(0, 0, 0);
	if (t.isPunctuation() && t.pToken() == XFoam_Token::BEGIN_LIST)
	{
		(void)is.readBeginList("XFoam_Block density");
		XFoam_Token ta(is);
		XFoam_Token tb(is);
		XFoam_Token tc(is);
		if (!ta.isLabel() || !tb.isLabel() || !tc.isLabel())
		{
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
				<< "expected three labels inside ( n_x n_y n_z ) for block density"
				<< XFoam_exit(XFoam_FatalIOError, 1);
		}
		density.x() = ta.labelToken();
		density.y() = tb.labelToken();
		density.z() = tc.labelToken();
		(void)is.readEndList("XFoam_Block density");
	}
	else
	{
		XFoam_Token tx(is);
		XFoam_Token ty(is);
		XFoam_Token tz(is);
		if (!tx.isLabel() || !ty.isLabel() || !tz.isLabel())
		{
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
				<< "expected three labels for old-style block density n_x n_y n_z"
				<< XFoam_exit(XFoam_FatalIOError, 1);
		}
		density.x() = tx.labelToken();
		density.y() = ty.labelToken();
		density.z() = tz.labelToken();
	}

	XFoam_Token tGrading(is);
	if (!tGrading.isWord())
	{
		is.putBack(tGrading);
	}
	// else: 与 Foam 一致，丢弃 simpleGrading / edgeGrading 等关键字，后续由 List 读入括号列表。

	const XFoam_List<XFoam_GradingDescriptors> expRatios = readExpansionGradingList(is);

	XFoam_List<XFoam_GradingDescriptors> expand;
	expand.setSize(12);
	if (expRatios.size() == 1)
	{
		for (XFoam_Label e = 0; e < 12; ++e)
		{
			expand[e] = expRatios[0];
		}
	}
	else if (expRatios.size() == 3)
	{
		for (XFoam_Label e = 0; e < 4; ++e)
		{
			expand[e] = expRatios[0];
		}
		for (XFoam_Label e = 4; e < 8; ++e)
		{
			expand[e] = expRatios[1];
		}
		for (XFoam_Label e = 8; e < 12; ++e)
		{
			expand[e] = expRatios[2];
		}
	}
	else if (expRatios.size() == 12)
	{
		for (XFoam_Label e = 0; e < 12; ++e)
		{
			expand[e] = expRatios[e];
		}
	}
	else
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
			<< "Unknown definition of expansion ratios: list size " << expRatios.size()
			<< " (expected 1, 3 or 12)"
			<< XFoam_exit(XFoam_FatalIOError, 1);
	}

	if (XFoam_BlockMesh::checkBlockFaceOrientation)
	{
		// 未移植：Foam::blockDescriptor::check(is) 面外向 / inside-out（blockDescriptor.C）。
	}

	return XFoam_BlockDescriptor(blockShape, vertices, edges, faces, density, expand, zoneName);
}
} // namespace

void XFoam_BlockDescriptor::check() const
{
	for (XFoam_Label pi = 0; pi < blockShape_.size(); ++pi)
	{
		const XFoam_Label vid = blockShape_[pi];
		if (vid < 0 || vid >= vertices_.size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_BlockDescriptor::check: vertex label out of range"));
		}
	}
}

void XFoam_BlockDescriptor::findCurvedFaces()
{
	nCurvedFaces_ = 0;
	curvedFaces_ = XFoam_FixedList<XFoam_Label, 6>(XFoam_Label(-1));
	for (XFoam_Label blockFacei = 0; blockFacei < 6; ++blockFacei)
	{
		const XFoam_FixedList<XFoam_Label, 4> blockFaceVerts =
			blockShape_.faceVertexLabels(blockFacei);
		for (XFoam_Label facei = 0; facei < faces_.size(); ++facei)
		{
			if (!faces_.set(facei))
			{
				continue;
			}
			if (sameVertices4(faces_[facei].vertices(), blockFaceVerts))
			{
				curvedFaces_[static_cast<unsigned>(blockFacei)] = facei;
				++nCurvedFaces_;
				break;
			}
		}
	}
}

XFoam_BlockDescriptor::XFoam_BlockDescriptor(
	const XFoam_CellShape& shape,
	const XFoam_UList<XFoam_Vector3D>& vertices,
	const XFoam_BlockEdgeList& edges,
	const XFoam_BlockFaceList& faces,
	const XFoam_Vector<XFoam_Label>& density,
	const XFoam_UList<XFoam_GradingDescriptors>& expand,
	const XFoam_String& zoneName)
	: vertices_(vertices)
	, edges_(edges)
	, faces_(faces)
	, blockShape_(shape)
	, density_(density)
	, expand_()
	, zoneName_(zoneName)
	, curvedFaces_(XFoam_Label(-1))
	, nCurvedFaces_(0)
{
	if (expand.size() != 12)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_BlockDescriptor: expand list must have size 12"));
	}
	expand_.setSize(12);
	for (XFoam_Label i = 0; i < 12; ++i)
	{
		expand_[i] = expand[i];
	}
	if (density_.x() < 1 || density_.y() < 1 || density_.z() < 1)
	{
		throw XFoam_Error(XFoam_String("XFoam_BlockDescriptor: density must be >= 1"));
	}
	check();
	findCurvedFaces();
}

// 移植参考: OpenFOAM src/mesh/blockMesh/blockDescriptor/blockDescriptorEdges.C。
// 与 OF 差异：边点 λ 仅按传入的 nDiv（来自 density_ 的 ni/nj/nk）均匀划分，不调用 lineDivide，不读 expand_。

XFoam_Label XFoam_BlockDescriptor::edgePointsWeights_(
	XFoam_FixedList<XFoam_PointField, 12>& edgePoints,
	XFoam_FixedList<XFoam_ScalarList, 12>& edgeWeights,
	const XFoam_Label edgei,
	const XFoam_Label startIdx,
	const XFoam_Label endIdx,
	const XFoam_Label nDiv) const
{
	const XFoam_Label lblA = blockShape_[startIdx];
	const XFoam_Label lblB = blockShape_[endIdx];
	const XFoam_Label nSeg = (nDiv > 0) ? nDiv : 1;
	XFoam_ScalarList lambdas(nSeg + 1);
	{
		const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / static_cast<XFoam_Scalar>(nSeg);
		for (XFoam_Label k = 0; k <= nSeg; ++k)
		{
			lambdas[k] = static_cast<XFoam_Scalar>(k) * inv;
		}
	}

	for (XFoam_Label ei = 0; ei < edges_.size(); ++ei)
	{
		if (!edges_.set(ei))
		{
			continue;
		}
		const XFoam_BlockEdge& cedge = edges_[ei];
		const int cmp = cedge.compare(lblA, lblB);
		if (cmp)
		{
			// cmp<0：边方向与块顶点序相反，点序与 λ 同时反转，权重为 1−λ。
			const XFoam_Tmp<XFoam_Field<XFoam_Vector3D>> tpos = cedge.position(lambdas);
			const XFoam_Field<XFoam_Vector3D>& p = tpos();
			const XFoam_Label np = p.size();
			edgePoints[static_cast<unsigned>(edgei)].setSize(np);
			edgeWeights[static_cast<unsigned>(edgei)].setSize(np);
			if (cmp > 0)
			{
				for (XFoam_Label pi = 0; pi < np; ++pi)
				{
					edgePoints[static_cast<unsigned>(edgei)][pi] = p[pi];
					edgeWeights[static_cast<unsigned>(edgei)][pi] = lambdas[pi];
				}
			}
			else
			{
				const XFoam_Label pn = np - 1;
				for (XFoam_Label pi = 0; pi < np; ++pi)
				{
					const XFoam_Label q = pn - pi;
					edgePoints[static_cast<unsigned>(edgei)][pi] = p[q];
					edgeWeights[static_cast<unsigned>(edgei)][pi] = static_cast<XFoam_Scalar>(1) - lambdas[q];
				}
			}
			return 1;
		}
	}

	const XFoam_LineEdge straight(vertices_, lblA, lblB);
	const XFoam_BlockEdge& straightBase = straight;
	const XFoam_Tmp<XFoam_Field<XFoam_Vector3D>> tpos = straightBase.position(lambdas);
	const XFoam_Field<XFoam_Vector3D>& p = tpos();
	const XFoam_Label np = p.size();
	edgePoints[static_cast<unsigned>(edgei)].setSize(np);
	edgeWeights[static_cast<unsigned>(edgei)].setSize(np);
	for (XFoam_Label pi = 0; pi < np; ++pi)
	{
		edgePoints[static_cast<unsigned>(edgei)][pi] = p[pi];
		edgeWeights[static_cast<unsigned>(edgei)][pi] = lambdas[pi];
	}
	return 0;
}

XFoam_Label XFoam_BlockDescriptor::edgesPointsWeights(
	XFoam_FixedList<XFoam_PointField, 12>& edgePoints,
	XFoam_FixedList<XFoam_ScalarList, 12>& edgeWeights) const
{
	for (unsigned e = 0; e < 12u; ++e)
	{
		edgePoints[e].clear();
		edgeWeights[e].clear();
	}
	XFoam_Label nCurvedEdges = 0;
	const XFoam_Label ni = density_.x();
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 0, 0, 1, ni);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 1, 3, 2, ni);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 2, 7, 6, ni);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 3, 4, 5, ni);
	const XFoam_Label nj = density_.y();
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 4, 0, 3, nj);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 5, 1, 2, nj);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 6, 5, 6, nj);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 7, 4, 7, nj);
	const XFoam_Label nk = density_.z();
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 8, 0, 4, nk);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 9, 1, 5, nk);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 10, 2, 6, nk);
	nCurvedEdges += edgePointsWeights_(edgePoints, edgeWeights, 11, 3, 7, nk);
	return nCurvedEdges;
}

XFoam_FixedList<XFoam_List<XFoam_Vector3D>, 6> XFoam_BlockDescriptor::facePoints(
	const XFoam_UList<XFoam_Vector3D>& points) const
{
	const XFoam_Label ni = density_.x();
	const XFoam_Label nj = density_.y();
	const XFoam_Label nk = density_.z();

	XFoam_FixedList<XFoam_List<XFoam_Vector3D>, 6> facePoints;
	facePoints[0].setSize((nj + 1) * (nk + 1));
	facePoints[1].setSize((nj + 1) * (nk + 1));
	for (XFoam_Label j = 0; j <= nj; ++j)
	{
		for (XFoam_Label k = 0; k <= nk; ++k)
		{
			facePoints[0][facePointLabel(0, j, k)] = points[pointLabel(0, j, k)];
			facePoints[1][facePointLabel(1, j, k)] = points[pointLabel(ni, j, k)];
		}
	}
	facePoints[2].setSize((ni + 1) * (nk + 1));
	facePoints[3].setSize((ni + 1) * (nk + 1));
	for (XFoam_Label i = 0; i <= ni; ++i)
	{
		for (XFoam_Label k = 0; k <= nk; ++k)
		{
			facePoints[2][facePointLabel(2, i, k)] = points[pointLabel(i, 0, k)];
			facePoints[3][facePointLabel(3, i, k)] = points[pointLabel(i, nj, k)];
		}
	}
	facePoints[4].setSize((ni + 1) * (nj + 1));
	facePoints[5].setSize((ni + 1) * (nj + 1));
	for (XFoam_Label i = 0; i <= ni; ++i)
	{
		for (XFoam_Label j = 0; j <= nj; ++j)
		{
			facePoints[4][facePointLabel(4, i, j)] = points[pointLabel(i, j, 0)];
			facePoints[5][facePointLabel(5, i, j)] = points[pointLabel(i, j, nk)];
		}
	}
	return facePoints;
}

void XFoam_BlockDescriptor::correctFacePoints(
	XFoam_FixedList<XFoam_List<XFoam_Vector3D>, 6>& facePts) const
{
	for (unsigned blockFacei = 0; blockFacei < 6u; ++blockFacei)
	{
		const XFoam_Label facei = curvedFaces_[blockFacei];
		if (facei < 0)
		{
			continue;
		}
		if (!faces_.set(facei))
		{
			continue;
		}
		XFoam_List<XFoam_Vector3D>& fp = facePts[blockFacei];
		XFoam_Field<XFoam_Vector3D> pts(fp);
		faces_[facei].project(*this, static_cast<XFoam_Label>(blockFacei), pts);
		for (XFoam_Label pi = 0; pi < pts.size(); ++pi)
		{
			fp[pi] = pts[pi];
		}
	}
}

void XFoam_BlockDescriptor::write(XFoam_OStream& os, XFoam_Label blocki, const void* dict)
{
	(void)dict;
	os << blocki;
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BlockDescriptor& bd)
{
	os << bd.blockShape().model().name() << ' ' << bd.density().x() << ' ' << bd.density().y() << ' '
	   << bd.density().z() << ' ' << bd.zoneName();
	return os;
}

const char* const XFoam_Block::typeName = "block";

XFoam_Block::XFoam_Block(const XFoam_BlockDescriptor& bd)
	: XFoam_BlockDescriptor(bd)
{
	createPoints();
	createBoundary();
}

// 移植源码: OpenFOAM src/mesh/blockMesh/blocks/block/block.C（Foam::block::block）
//          + src/mesh/blockMesh/blockDescriptor/blockDescriptor.C（Foam::blockDescriptor::blockDescriptor Istream）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
XFoam_Block::XFoam_Block(
	const XFoam_Dictionary& dict,
	XFoam_Label index,
	const XFoam_PointField& vertices,
	const XFoam_BlockEdgeList& edges,
	const XFoam_BlockFaceList& faces,
	XFoam_IStream& is)
	: XFoam_BlockDescriptor(makeBlockDescriptorFromMeshStream(dict, vertices, edges, faces, is))
{
	(void)index;
	createPoints();
	createBoundary();
}

XFoam_Block::~XFoam_Block() = default;

// 移植源码: OpenFOAM src/mesh/blockMesh/blocks/block/blockCreate.C（Foam::block::createPoints）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
void XFoam_Block::createPoints()
{
	const XFoam_Label ni = density().x();
	const XFoam_Label nj = density().y();
	const XFoam_Label nk = density().z();

	const XFoam_Vector3D& p000 = blockPoint(0);
	const XFoam_Vector3D& p100 = blockPoint(1);
	const XFoam_Vector3D& p110 = blockPoint(2);
	const XFoam_Vector3D& p010 = blockPoint(3);

	const XFoam_Vector3D& p001 = blockPoint(4);
	const XFoam_Vector3D& p101 = blockPoint(5);
	const XFoam_Vector3D& p111 = blockPoint(6);
	const XFoam_Vector3D& p011 = blockPoint(7);

	XFoam_FixedList<XFoam_PointField, 12> p;
	XFoam_FixedList<XFoam_ScalarList, 12> w;
	const XFoam_Label nCurvedEdges = edgesPointsWeights(p, w);

	points_.setSize(nPoints());
   
	   points_[pointLabel(0,  0,  0)] = p000;
	   points_[pointLabel(ni, 0,  0)] = p100;
	   points_[pointLabel(ni, nj, 0)] = p110;
	   points_[pointLabel(0,  nj, 0)] = p010;
	   points_[pointLabel(0,  0,  nk)] = p001;
	   points_[pointLabel(ni, 0,  nk)] = p101;
	   points_[pointLabel(ni, nj, nk)] = p111;
	   points_[pointLabel(0,  nj, nk)] = p011;
   
	   for (XFoam_Label k=0; k<=nk; k++)
	   {
		   for (XFoam_Label j=0; j<=nj; j++)
		   {
			   for (XFoam_Label i=0; i<=ni; i++)
			   {
				   if (vertex(i, j, k))
				   {
					   continue;
				   }
   
				   const XFoam_Label vijk = pointLabel(i, j, k);
   
				   const XFoam_Scalar ww0 = w[static_cast<unsigned>(0)][i];
				   const XFoam_Scalar ww1 = w[static_cast<unsigned>(1)][i];
				   const XFoam_Scalar ww2 = w[static_cast<unsigned>(2)][i];
				   const XFoam_Scalar ww3 = w[static_cast<unsigned>(3)][i];
				   const XFoam_Scalar ww4 = w[static_cast<unsigned>(4)][j];
				   const XFoam_Scalar ww5 = w[static_cast<unsigned>(5)][j];
				   const XFoam_Scalar ww6 = w[static_cast<unsigned>(6)][j];
				   const XFoam_Scalar ww7 = w[static_cast<unsigned>(7)][j];
				   const XFoam_Scalar ww8 = w[static_cast<unsigned>(8)][k];
				   const XFoam_Scalar ww9 = w[static_cast<unsigned>(9)][k];
				   const XFoam_Scalar ww10 = w[static_cast<unsigned>(10)][k];
				   const XFoam_Scalar ww11 = w[static_cast<unsigned>(11)][k];
   
				   XFoam_Scalar wx1 =
					   (1 - ww0) * (1 - ww4) * (1 - ww8) + ww0 * (1 - ww5) * (1 - ww9);
				   XFoam_Scalar wx2 =
					   (1 - ww1) * ww4 * (1 - ww11) + ww1 * ww5 * (1 - ww10);
				   XFoam_Scalar wx3 = (1 - ww2) * ww7 * ww11 + ww2 * ww6 * ww10;
				   XFoam_Scalar wx4 = (1 - ww3) * (1 - ww7) * ww8 + ww3 * (1 - ww6) * ww9;
   
				   const XFoam_Scalar sumWx = wx1 + wx2 + wx3 + wx4;
				   wx1 /= sumWx;
				   wx2 /= sumWx;
				   wx3 /= sumWx;
				   wx4 /= sumWx;
   
   
				   XFoam_Scalar wy1 =
					   (1 - ww4) * (1 - ww0) * (1 - ww8) + ww4 * (1 - ww1) * (1 - ww11);
				   XFoam_Scalar wy2 = (1 - ww5) * ww0 * (1 - ww9) + ww5 * ww1 * (1 - ww10);
				   XFoam_Scalar wy3 = (1 - ww6) * ww3 * ww9 + ww6 * ww2 * ww10;
				   XFoam_Scalar wy4 = (1 - ww7) * (1 - ww3) * ww8 + ww7 * (1 - ww2) * ww11;
	
				   const XFoam_Scalar sumWy = wy1 + wy2 + wy3 + wy4;
				   wy1 /= sumWy;
				   wy2 /= sumWy;
				   wy3 /= sumWy;
				   wy4 /= sumWy;
   
   
				   XFoam_Scalar wz1 =
					   (1 - ww8) * (1 - ww0) * (1 - ww4) + ww8 * (1 - ww3) * (1 - ww7);
				   XFoam_Scalar wz2 = (1 - ww9) * ww0 * (1 - ww5) + ww9 * ww3 * (1 - ww6);
				   XFoam_Scalar wz3 = (1 - ww10) * ww1 * ww5 + ww10 * ww2 * ww6;
				   XFoam_Scalar wz4 = (1 - ww11) * (1 - ww1) * ww4 + ww11 * (1 - ww2) * ww7;
   
				   const XFoam_Scalar sumWz = wz1 + wz2 + wz3 + wz4;
				   wz1 /= sumWz;
				   wz2 /= sumWz;
				   wz3 /= sumWz;
				   wz4 /= sumWz;
   
   
				   const XFoam_Vector3D edgex1 = p000 + (p100 - p000) * ww0;
				   const XFoam_Vector3D edgex2 = p010 + (p110 - p010) * ww1;
				   const XFoam_Vector3D edgex3 = p011 + (p111 - p011) * ww2;
				   const XFoam_Vector3D edgex4 = p001 + (p101 - p001) * ww3;
   
				   const XFoam_Vector3D edgey1 = p000 + (p010 - p000) * ww4;
				   const XFoam_Vector3D edgey2 = p100 + (p110 - p100) * ww5;
				   const XFoam_Vector3D edgey3 = p101 + (p111 - p101) * ww6;
				   const XFoam_Vector3D edgey4 = p001 + (p011 - p001) * ww7;
   
				   const XFoam_Vector3D edgez1 = p000 + (p001 - p000) * ww8;
				   const XFoam_Vector3D edgez2 = p100 + (p101 - p100) * ww9;
				   const XFoam_Vector3D edgez3 = p110 + (p111 - p110) * ww10;
				   const XFoam_Vector3D edgez4 = p010 + (p011 - p010) * ww11;
   
				   // Add the contributions
				   points_[vijk] = (wx1 * edgex1 + wx2 * edgex2 + wx3 * edgex3 + wx4 * edgex4
					   + wy1 * edgey1 + wy2 * edgey2 + wy3 * edgey3 + wy4 * edgey4
					   + wz1 * edgez1 + wz2 * edgez2 + wz3 * edgez3 + wz4 * edgez4)
					   / static_cast<XFoam_Scalar>(3);
   
   
				   // Apply curved-edge correction if block has curved edges
				   if (nCurvedEdges)
				   {
					   const XFoam_Vector3D corx1 = wx1 * (p[static_cast<unsigned>(0)][i] - edgex1);
					   const XFoam_Vector3D corx2 = wx2 * (p[static_cast<unsigned>(1)][i] - edgex2);
					   const XFoam_Vector3D corx3 = wx3 * (p[static_cast<unsigned>(2)][i] - edgex3);
					   const XFoam_Vector3D corx4 = wx4 * (p[static_cast<unsigned>(3)][i] - edgex4);
   
					   const XFoam_Vector3D cory1 = wy1 * (p[static_cast<unsigned>(4)][j] - edgey1);
					   const XFoam_Vector3D cory2 = wy2 * (p[static_cast<unsigned>(5)][j] - edgey2);
					   const XFoam_Vector3D cory3 = wy3 * (p[static_cast<unsigned>(6)][j] - edgey3);
					   const XFoam_Vector3D cory4 = wy4 * (p[static_cast<unsigned>(7)][j] - edgey4);
   
					   const XFoam_Vector3D corz1 = wz1 * (p[static_cast<unsigned>(8)][k] - edgez1);
					   const XFoam_Vector3D corz2 = wz2 * (p[static_cast<unsigned>(9)][k] - edgez2);
					   const XFoam_Vector3D corz3 = wz3 * (p[static_cast<unsigned>(10)][k] - edgez3);
					   const XFoam_Vector3D corz4 = wz4 * (p[static_cast<unsigned>(11)][k] - edgez4);
   
					   points_[vijk] +=
					   (
						   corx1 + corx2 + corx3 + corx4
						 + cory1 + cory2 + cory3 + cory4
						 + corz1 + corz2 + corz3 + corz4
					   );
				   }
			   }
		   }
	   }
   
	   if (!nCurvedFaces())
	   {
		   return;
	   }
   
	   XFoam_FixedList<XFoam_List<XFoam_Vector3D>, 6> facePts(this->facePoints(points_));
	   correctFacePoints(facePts);
   
	   for (XFoam_Label ii = 0; ii <= ni; ++ii)
	   {
		   const XFoam_Label i = (ii + 1) % (ni + 1);
   
		   for (XFoam_Label j = 0; j <= nj; ++j)
		   {
			   for (XFoam_Label k = 0; k <= nk; ++k)
			   {
				   if (flatFaceOrEdge(i, j, k))
				   {
					   continue;
				   }
   
				   const XFoam_Label vijk = pointLabel(i, j, k);
   
				   const XFoam_Scalar ww0 = w[static_cast<unsigned>(0)][i];
				   const XFoam_Scalar ww1 = w[static_cast<unsigned>(1)][i];
				   const XFoam_Scalar ww2 = w[static_cast<unsigned>(2)][i];
				   const XFoam_Scalar ww3 = w[static_cast<unsigned>(3)][i];
				   const XFoam_Scalar ww4 = w[static_cast<unsigned>(4)][j];
				   const XFoam_Scalar ww5 = w[static_cast<unsigned>(5)][j];
				   const XFoam_Scalar ww6 = w[static_cast<unsigned>(6)][j];
				   const XFoam_Scalar ww7 = w[static_cast<unsigned>(7)][j];
				   const XFoam_Scalar ww8 = w[static_cast<unsigned>(8)][k];
				   const XFoam_Scalar ww9 = w[static_cast<unsigned>(9)][k];
				   const XFoam_Scalar ww10 = w[static_cast<unsigned>(10)][k];
				   const XFoam_Scalar ww11 = w[static_cast<unsigned>(11)][k];
   
				   XFoam_Scalar wf0 = (1 - ww0) * (1 - ww4) * (1 - ww8) + (1 - ww1) * ww4 * (1 - ww11)
					   + (1 - ww2) * ww7 * ww11 + (1 - ww3) * (1 - ww7) * ww8;
   
				   XFoam_Scalar wf1 = ww0 * (1 - ww5) * (1 - ww9) + ww1 * ww5 * (1 - ww10) + ww2 * ww5 * ww10
					   + ww3 * (1 - ww6) * ww9;
   
				   const XFoam_Scalar sumWf = wf0 + wf1;
				   wf0 /= sumWf;
				   wf1 /= sumWf;
   
				   points_[vijk] +=
					   wf0 * (facePts[0][facePointLabel(0, j, k)] - points_[pointLabel(0, j, k)])
					   + wf1 * (facePts[1][facePointLabel(1, j, k)] - points_[pointLabel(ni, j, k)]);
			   }
		   }
	   }
   
	   for (XFoam_Label jj = 0; jj <= nj; ++jj)
	   {
		   const XFoam_Label j = (jj + 1) % (nj + 1);
   
		   for (XFoam_Label i = 0; i <= ni; ++i)
		   {
			   for (XFoam_Label k = 0; k <= nk; ++k)
			   {
				   if (flatFaceOrEdge(i, j, k))
				   {
					   continue;
				   }
   
				   const XFoam_Label vijk = pointLabel(i, j, k);
   
				   const XFoam_Scalar ww0 = w[static_cast<unsigned>(0)][i];
				   const XFoam_Scalar ww1 = w[static_cast<unsigned>(1)][i];
				   const XFoam_Scalar ww2 = w[static_cast<unsigned>(2)][i];
				   const XFoam_Scalar ww3 = w[static_cast<unsigned>(3)][i];
				   const XFoam_Scalar ww4 = w[static_cast<unsigned>(4)][j];
				   const XFoam_Scalar ww5 = w[static_cast<unsigned>(5)][j];
				   const XFoam_Scalar ww6 = w[static_cast<unsigned>(6)][j];
				   const XFoam_Scalar ww7 = w[static_cast<unsigned>(7)][j];
				   const XFoam_Scalar ww8 = w[static_cast<unsigned>(8)][k];
				   const XFoam_Scalar ww9 = w[static_cast<unsigned>(9)][k];
				   const XFoam_Scalar ww10 = w[static_cast<unsigned>(10)][k];
				   const XFoam_Scalar ww11 = w[static_cast<unsigned>(11)][k];
   
				   XFoam_Scalar wf2 = (1 - ww4) * (1 - ww1) * (1 - ww8) + (1 - ww5) * ww0 * (1 - ww9)
					   + (1 - ww6) * ww3 * ww9 + (1 - ww7) * (1 - ww3) * ww8;
   
				   XFoam_Scalar wf3 = ww4 * (1 - ww1) * (1 - ww11) + ww5 * ww1 * (1 - ww10) + ww6 * ww2 * ww10
					   + ww7 * (1 - ww2) * ww11;
   
				   const XFoam_Scalar sumWf = wf2 + wf3;
				   wf2 /= sumWf;
				   wf3 /= sumWf;
   
				   points_[vijk] +=
					   wf2 * (facePts[2][facePointLabel(2, i, k)] - points_[pointLabel(i, 0, k)])
					   + wf3 * (facePts[3][facePointLabel(3, i, k)] - points_[pointLabel(i, nj, k)]);
			   }
		   }
	   }
   
	   for (XFoam_Label kk = 0; kk <= nk; ++kk)
	   {
		   const XFoam_Label k = (kk + 1) % (nk + 1);
   
		   for (XFoam_Label i = 0; i <= ni; ++i)
		   {
			   for (XFoam_Label j = 0; j <= nj; ++j)
			   {
				   if (flatFaceOrEdge(i, j, k))
				   {
					   continue;
				   }
   
				   const XFoam_Label vijk = pointLabel(i, j, k);
   
				   const XFoam_Scalar ww0 = w[static_cast<unsigned>(0)][i];
				   const XFoam_Scalar ww1 = w[static_cast<unsigned>(1)][i];
				   const XFoam_Scalar ww2 = w[static_cast<unsigned>(2)][i];
				   const XFoam_Scalar ww3 = w[static_cast<unsigned>(3)][i];
				   const XFoam_Scalar ww4 = w[static_cast<unsigned>(4)][j];
				   const XFoam_Scalar ww5 = w[static_cast<unsigned>(5)][j];
				   const XFoam_Scalar ww6 = w[static_cast<unsigned>(6)][j];
				   const XFoam_Scalar ww7 = w[static_cast<unsigned>(7)][j];
				   const XFoam_Scalar ww8 = w[static_cast<unsigned>(8)][k];
				   const XFoam_Scalar ww9 = w[static_cast<unsigned>(9)][k];
				   const XFoam_Scalar ww10 = w[static_cast<unsigned>(10)][k];
				   const XFoam_Scalar ww11 = w[static_cast<unsigned>(11)][k];
   
				   XFoam_Scalar wf4 = (1 - ww8) * (1 - ww0) * (1 - ww4) + (1 - ww9) * ww0 * (1 - ww5)
					   + (1 - ww10) * ww1 * ww5 + (1 - ww11) * (1 - ww1) * ww4;
   
				   XFoam_Scalar wf5 = ww8 * (1 - ww3) * (1 - ww7) + ww9 * ww3 * (1 - ww6) + ww10 * ww2 * ww6
					   + ww11 * (1 - ww2) * ww7;
   
				   const XFoam_Scalar sumWf = wf4 + wf5;
				   wf4 /= sumWf;
				   wf5 /= sumWf;
   
				   points_[vijk] +=
					   wf4 * (facePts[4][facePointLabel(4, i, j)] - points_[pointLabel(i, j, 0)])
					   + wf5 * (facePts[5][facePointLabel(5, i, j)] - points_[pointLabel(i, j, nk)]);
			   }
		   }
	   }
}

// 移植源码: OpenFOAM src/mesh/blockMesh/blocks/block/blockCreate.C（Foam::block::createBoundary）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
void XFoam_Block::createBoundary()
{
	for (unsigned pi = 0; pi < 6u; ++pi)
	{
		boundaryPatches_[pi].clear();
	}

	const XFoam_Label ni = density().x();
	const XFoam_Label nj = density().y();
	const XFoam_Label nk = density().z();

	const XFoam_Face quad4(4);

	XFoam_Label patchi = 0;
	XFoam_Label facei = 0;

	boundaryPatches_[patchi].setSize(nj * nk, quad4);
	for (XFoam_Label k = 0; k < nk; ++k)
	{
		for (XFoam_Label j = 0; j < nj; ++j)
		{
			XFoam_Face& f = boundaryPatches_[patchi][facei];
			f[0] = pointLabel(0, j, k);
			f[1] = pointLabel(0, j, k + 1);
			f[2] = pointLabel(0, j + 1, k + 1);
			f[3] = pointLabel(0, j + 1, k);
			++facei;
		}
	}

	++patchi;
	facei = 0;
	boundaryPatches_[patchi].setSize(nj * nk, quad4);
	for (XFoam_Label k = 0; k < nk; ++k)
	{
		for (XFoam_Label j = 0; j < nj; ++j)
		{
			XFoam_Face& f = boundaryPatches_[patchi][facei];
			f[0] = pointLabel(ni, j, k);
			f[1] = pointLabel(ni, j + 1, k);
			f[2] = pointLabel(ni, j + 1, k + 1);
			f[3] = pointLabel(ni, j, k + 1);
			++facei;
		}
	}

	++patchi;
	facei = 0;
	boundaryPatches_[patchi].setSize(ni * nk, quad4);
	for (XFoam_Label i = 0; i < ni; ++i)
	{
		for (XFoam_Label k = 0; k < nk; ++k)
		{
			XFoam_Face& f = boundaryPatches_[patchi][facei];
			f[0] = pointLabel(i, 0, k);
			f[1] = pointLabel(i + 1, 0, k);
			f[2] = pointLabel(i + 1, 0, k + 1);
			f[3] = pointLabel(i, 0, k + 1);
			++facei;
		}
	}

	++patchi;
	facei = 0;
	boundaryPatches_[patchi].setSize(ni * nk, quad4);
	for (XFoam_Label i = 0; i < ni; ++i)
	{
		for (XFoam_Label k = 0; k < nk; ++k)
		{
			XFoam_Face& f = boundaryPatches_[patchi][facei];
			f[0] = pointLabel(i, nj, k);
			f[1] = pointLabel(i, nj, k + 1);
			f[2] = pointLabel(i + 1, nj, k + 1);
			f[3] = pointLabel(i + 1, nj, k);
			++facei;
		}
	}

	++patchi;
	facei = 0;
	boundaryPatches_[patchi].setSize(ni * nj, quad4);
	for (XFoam_Label i = 0; i < ni; ++i)
	{
		for (XFoam_Label j = 0; j < nj; ++j)
		{
			XFoam_Face& f = boundaryPatches_[patchi][facei];
			f[0] = pointLabel(i, j, 0);
			f[1] = pointLabel(i, j + 1, 0);
			f[2] = pointLabel(i + 1, j + 1, 0);
			f[3] = pointLabel(i + 1, j, 0);
			++facei;
		}
	}

	++patchi;
	facei = 0;
	boundaryPatches_[patchi].setSize(ni * nj, quad4);
	for (XFoam_Label i = 0; i < ni; ++i)
	{
		for (XFoam_Label j = 0; j < nj; ++j)
		{
			XFoam_Face& f = boundaryPatches_[patchi][facei];
			f[0] = pointLabel(i, j, nk);
			f[1] = pointLabel(i + 1, j, nk);
			f[2] = pointLabel(i + 1, j + 1, nk);
			f[3] = pointLabel(i, j + 1, nk);
			++facei;
		}
	}
}


// 移植源码: OpenFOAM src/mesh/blockMesh/blocks/block/block.C（Foam::block::New）
// 命名规范: foam_code.md
// 移植规范: foam_code.md
XFoam_AutoPtr<XFoam_Block> XFoam_Block::New(
	const XFoam_Dictionary& dict,
	XFoam_Label index,
	const XFoam_PointField& vertices,
	const XFoam_BlockEdgeList& edges,
	const XFoam_BlockFaceList& faces,
	XFoam_IStream& is)
{
	return XFoam_AutoPtr<XFoam_Block>(
		new XFoam_Block(dict, index, vertices, edges, faces, is));
}

XFoam_AutoPtr<XFoam_Block> XFoam_Block::clone() const
{
	return XFoam_AutoPtr<XFoam_Block>();
}

XFoam_List<XFoam_FixedList<XFoam_Label, 8>> XFoam_Block::cells() const
{
	// 移植参考: OpenFOAM src/mesh/blockMesh/blocks/block/block.C（Foam::block::cells）
	// 体素 (i,j,k) 的 8 角点顺序与字典中 hex (0 1 2 3 4 5 6 7) 及 XFoam_CellModel::hex 一致。
	const XFoam_Label ni = density().x();
	const XFoam_Label nj = density().y();
	const XFoam_Label nk = density().z();
	const XFoam_Label nCells = ni * nj * nk;
	XFoam_List<XFoam_FixedList<XFoam_Label, 8>> out(nCells);
	XFoam_Label celli = 0;
	for (XFoam_Label k = 0; k < nk; ++k)
	{
		for (XFoam_Label j = 0; j < nj; ++j)
		{
			for (XFoam_Label i = 0; i < ni; ++i)
			{
				XFoam_FixedList<XFoam_Label, 8>& c = out[celli];
				c[0] = pointLabel(i, j, k);
				c[1] = pointLabel(i + 1, j, k);
				c[2] = pointLabel(i + 1, j + 1, k);
				c[3] = pointLabel(i, j + 1, k);
				c[4] = pointLabel(i, j, k + 1);
				c[5] = pointLabel(i + 1, j, k + 1);
				c[6] = pointLabel(i + 1, j + 1, k + 1);
				c[7] = pointLabel(i, j + 1, k + 1);
				++celli;
			}
		}
	}
	return out;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Block& b)
{
	(void)b;
	os << "XFoam_Block";
	return os;
}
