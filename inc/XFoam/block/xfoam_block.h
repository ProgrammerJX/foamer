#ifndef XFoam_Block_H_
#define XFoam_Block_H_

// blockMesh 块描述，对齐 OpenFOAM Foam::blockDescriptor（blockDescriptor.H / blockDescriptorI.H）。
// 曲线边/面的离散与 Istream 构造暂未移植；expand 仍按字典读入 12 条边的 grading，但边点离散仅按 density_ 均匀取 λ，不使用 expand_。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/mesh/xfoam_shape.h"
#include "XFoam/block/xfoam_blockedge.h"
#include "XFoam/block/xfoam_blockface.h"
#include "XFoam/block/xfoam_gradingdescriptor.h"

class XFoam_API XFoam_BlockDescriptor
{
	const XFoam_UList<XFoam_Vector3D>& vertices_;
	const XFoam_BlockEdgeList& edges_;
	const XFoam_BlockFaceList& faces_;

	XFoam_CellShape blockShape_;
	XFoam_Vector<XFoam_Label> density_;
	XFoam_List<XFoam_GradingDescriptors> expand_;
	XFoam_String zoneName_;
	XFoam_FixedList<XFoam_Label, 6> curvedFaces_;
	XFoam_Label nCurvedFaces_;

	void check() const;
	void findCurvedFaces();

	// 对应 OF blockDescriptorEdges.C::edgePointsWeights；此处 nDiv 为沿该边轴向的 density_ 分量（ni/nj/nk）。
	// 在曲线参数 λ∈[0,1] 上按 nDiv 均匀分段采样（与块内网格线一致），不使用 expand_/lineDivide 的非均匀划分。
	XFoam_Label edgePointsWeights_(
		XFoam_FixedList<XFoam_PointField, 12>& edgePoints,
		XFoam_FixedList<XFoam_ScalarList, 12>& edgeWeights,
		XFoam_Label edgei,
		XFoam_Label startIdx,
		XFoam_Label endIdx,
		XFoam_Label nDiv) const;

public:
	XFoam_BlockDescriptor(
		const XFoam_CellShape& shape,
		const XFoam_UList<XFoam_Vector3D>& vertices,
		const XFoam_BlockEdgeList& edges,
		const XFoam_BlockFaceList& faces,
		const XFoam_Vector<XFoam_Label>& density,
		const XFoam_UList<XFoam_GradingDescriptors>& expand,
		const XFoam_String& zoneName = XFoam_String());

	XFoam_BlockDescriptor(const XFoam_BlockDescriptor&) = default;
	XFoam_BlockDescriptor& operator=(const XFoam_BlockDescriptor&) = delete;

	const XFoam_UList<XFoam_Vector3D>& vertices() const { return vertices_; }
	const XFoam_BlockFaceList& faces() const { return faces_; }
	const XFoam_CellShape& blockShape() const { return blockShape_; }
	const XFoam_Vector<XFoam_Label>& density() const { return density_; }
	const XFoam_String& zoneName() const { return zoneName_; }

	XFoam_Label nPoints() const;
	XFoam_Label nCells() const;

	const XFoam_FixedList<XFoam_Label, 6>& curvedFaces() const { return curvedFaces_; }
	XFoam_Label nCurvedFaces() const { return nCurvedFaces_; }

	const XFoam_Vector3D& blockPoint(XFoam_Label i) const;

	XFoam_Label pointLabel(XFoam_Label i, XFoam_Label j, XFoam_Label k) const;

	XFoam_Label facePointLabel(XFoam_Label facei, XFoam_Label i, XFoam_Label j) const;

	bool vertex(XFoam_Label i, XFoam_Label j, XFoam_Label k) const;
	bool edge(XFoam_Label i, XFoam_Label j, XFoam_Label k) const;

	// 十二条边：分段数分别取 density_.x/y/z（与 pointLabel 拓扑一致）；λ 均匀，忽略 expand_。
	XFoam_Label edgesPointsWeights(
		XFoam_FixedList<XFoam_PointField, 12>& edgePoints,
		XFoam_FixedList<XFoam_ScalarList, 12>& edgeWeights) const;

	bool flatFaceOrEdge(XFoam_Label i, XFoam_Label j, XFoam_Label k) const;

	XFoam_FixedList<XFoam_List<XFoam_Vector3D>, 6> facePoints(
		const XFoam_UList<XFoam_Vector3D>& points) const;

	void correctFacePoints(XFoam_FixedList<XFoam_List<XFoam_Vector3D>, 6>& facePts) const;

	static void write(XFoam_OStream& os, XFoam_Label blocki, const void* dict /* reserved */);
};

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BlockDescriptor& bd);

inline XFoam_Label XFoam_BlockDescriptor::nPoints() const
{
	return (density_.x() + 1) * (density_.y() + 1) * (density_.z() + 1);
}

inline XFoam_Label XFoam_BlockDescriptor::nCells() const
{
	return density_.x() * density_.y() * density_.z();
}

inline const XFoam_Vector3D& XFoam_BlockDescriptor::blockPoint(XFoam_Label i) const
{
	return vertices_[blockShape_[i]];
}

inline XFoam_Label XFoam_BlockDescriptor::pointLabel(
	XFoam_Label i,
	XFoam_Label j,
	XFoam_Label k) const
{
	return i + j * (density_.x() + 1) + k * (density_.x() + 1) * (density_.y() + 1);
}

inline XFoam_Label XFoam_BlockDescriptor::facePointLabel(
	XFoam_Label facei,
	XFoam_Label i,
	XFoam_Label j) const
{
	if (facei == 0 || facei == 1)
	{
		return i + j * (density_.y() + 1);
	}
	if (facei == 2 || facei == 3)
	{
		return i + j * (density_.x() + 1);
	}
	return i + j * (density_.x() + 1);
}

inline bool XFoam_BlockDescriptor::vertex(XFoam_Label i, XFoam_Label j, XFoam_Label k) const
{
	const bool iEnd = (i == 0 || i == density_.x());
	const bool jEnd = (j == 0 || j == density_.y());
	const bool kEnd = (k == 0 || k == density_.z());
	return iEnd && jEnd && kEnd;
}

inline bool XFoam_BlockDescriptor::edge(XFoam_Label i, XFoam_Label j, XFoam_Label k) const
{
	const bool iEnd = (i == 0 || i == density_.x());
	const bool jEnd = (j == 0 || j == density_.y());
	const bool kEnd = (k == 0 || k == density_.z());
	return (iEnd && jEnd) || (iEnd && kEnd) || (jEnd && kEnd);
}

inline bool XFoam_BlockDescriptor::flatFaceOrEdge(XFoam_Label i, XFoam_Label j, XFoam_Label k) const
{
	if (i == 0 && curvedFaces_[0] == -1)
	{
		return true;
	}
	if (i == density_.x() && curvedFaces_[1] == -1)
	{
		return true;
	}
	if (j == 0 && curvedFaces_[2] == -1)
	{
		return true;
	}
	if (j == density_.y() && curvedFaces_[3] == -1)
	{
		return true;
	}
	if (k == 0 && curvedFaces_[4] == -1)
	{
		return true;
	}
	if (k == density_.z() && curvedFaces_[5] == -1)
	{
		return true;
	}
	return edge(i, j, k);
}

/*---------------------------------------------------------------------------*\
                          Class XFoam_Block Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_Block
	: public XFoam_BlockDescriptor
{
	XFoam_PointField points_;
	XFoam_FixedList<XFoam_List<XFoam_Face>, 6> boundaryPatches_;

	void createPoints();
	void createBoundary();

public:
	static const char* const typeName;

	static XFoam_AutoPtr<XFoam_Block> New(
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_PointField& vertices,
		const XFoam_BlockEdgeList& edges,
		const XFoam_BlockFaceList& faces,
		XFoam_IStream& is);

	class INew
	{
		const XFoam_Dictionary& dict_;
		const XFoam_PointField& points_;
		const XFoam_BlockEdgeList& edges_;
		const XFoam_BlockFaceList& faces_;
		mutable XFoam_Label index_;

	public:
		INew(
			const XFoam_Dictionary& dict,
			const XFoam_PointField& points,
			const XFoam_BlockEdgeList& edges,
			const XFoam_BlockFaceList& faces)
			: dict_(dict)
			, points_(points)
			, edges_(edges)
			, faces_(faces)
			, index_(0)
		{
		}

		XFoam_AutoPtr<XFoam_Block> operator()(XFoam_IStream& is) const
		{
			return XFoam_Block::New(dict_, index_++, points_, edges_, faces_, is);
		}
	};

	XFoam_Block(
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_PointField& vertices,
		const XFoam_BlockEdgeList& edges,
		const XFoam_BlockFaceList& faces,
		XFoam_IStream& is);

	XFoam_Block(const XFoam_BlockDescriptor&);

	XFoam_Block(const XFoam_Block&) = delete;
	void operator=(const XFoam_Block&) = delete;

	XFoam_AutoPtr<XFoam_Block> clone() const;

	virtual ~XFoam_Block();

	const XFoam_PointField& points() const { return points_; }

	XFoam_List<XFoam_FixedList<XFoam_Label, 8>> cells() const;

	const XFoam_FixedList<XFoam_List<XFoam_Face>, 6>& boundaryPatches() const { return boundaryPatches_; }

	friend XFoam_OStream& operator<<(XFoam_OStream&, const XFoam_Block&);
};

typedef XFoam_PtrList<XFoam_Block> XFoam_BlockList;

#endif
