#ifndef XFoam_blockEdge_H_
#define XFoam_blockEdge_H_

// 对标 OpenFOAM-13：src/mesh/blockMesh/blockEdges/blockEdge/blockEdge.H / blockEdge.C / blockEdgeI.H
// 及 blockEdges/lineEdge/lineEdge.H / lineEdge.C（Foam::blockEdges::lineEdge）。
// 参考：OpenFOAM Foundation v13 API 文档（cpp.openfoam.org）。
// XFoam_Dictionary / XFoam_SearchableSurfaceList：占位前向声明；无名称反查、无几何检索。
// XFoam_declareRunTimeSelectionTable / addToRunTimeSelectionTable：未移植（串行、无 RTTI 表）。
// XFoam_BlockEdge::New：仅识别边类型 "line" 并构造 XFoam_LineEdge；其余 FatalError。

#include "XFoam/snap/xfoam_searchablesurface.h"
#include "XFoam/snap/xfoam_shape.h"
#include "XFoam/utilities/xfoam_common.h"

class XFoam_Dictionary;

class XFoam_API XFoam_BlockEdge
{
protected:
	const XFoam_UList<XFoam_Vector3D>& points_;
	XFoam_Label start_;
	XFoam_Label end_;

public:
	static constexpr const char* typeName = "blockEdge";

	XFoam_BlockEdge(const XFoam_UList<XFoam_Vector3D>& pts, XFoam_Label start, XFoam_Label end);

	XFoam_BlockEdge(
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_UList<XFoam_Vector3D>& pts,
		XFoam_IStream& is);

	virtual ~XFoam_BlockEdge();

	XFoam_Label start() const;
	XFoam_Label end() const;

	int compare(XFoam_Label start, XFoam_Label end) const;
	int compare(const XFoam_BlockEdge& e) const;
	int compare(const XFoam_Edge& e) const;

	virtual XFoam_Vector3D position(XFoam_Scalar lambda) const = 0;

	// 对标 OF blockEdge::position(scalarList) → tmp<pointField>；XFoam 为 tmp<Field<vector>>。
	XFoam_Tmp<XFoam_Field<XFoam_Vector3D>> position(const XFoam_ScalarList& lambdas) const;

	virtual XFoam_Scalar length() const = 0;

	static XFoam_List<XFoam_Vector3D> appendEndPoints(
		const XFoam_UList<XFoam_Vector3D>& points,
		XFoam_Label start,
		XFoam_Label end,
		const XFoam_UList<XFoam_Vector3D>& otherKnots);

	void write(XFoam_OStream& os, const XFoam_Dictionary& d) const;

	// XFoam：OpenFOAM 中 clone 依赖完整类型注册表；此处返回空 autoPtr（头文件约定：未实现返回空）。
	virtual XFoam_AutoPtr<XFoam_BlockEdge> clone() const;

	static XFoam_AutoPtr<XFoam_BlockEdge> New(
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_SearchableSurfaceList& geometry,
		const XFoam_UList<XFoam_Vector3D>& points,
		XFoam_IStream& is);

	class INew
	{
		const XFoam_Dictionary& dict_;
		const XFoam_SearchableSurfaceList& geometry_;
		const XFoam_UList<XFoam_Vector3D>& points_;
		mutable XFoam_Label index_;

	public:
		INew(
			const XFoam_Dictionary& dict,
			const XFoam_SearchableSurfaceList& geometry,
			const XFoam_UList<XFoam_Vector3D>& points)
			: dict_(dict)
			, geometry_(geometry)
			, points_(points)
			, index_(0)
		{}

		XFoam_AutoPtr<XFoam_BlockEdge> operator()(XFoam_IStream& is) const
		{
			return XFoam_BlockEdge::New(dict_, index_++, geometry_, points_, is);
		}
	};

	friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BlockEdge& p);
};

inline XFoam_Label XFoam_BlockEdge::start() const
{
	return start_;
}

inline XFoam_Label XFoam_BlockEdge::end() const
{
	return end_;
}

inline int XFoam_BlockEdge::compare(XFoam_Label start, XFoam_Label end) const
{
	if (start_ == start && end_ == end)
	{
		return 1;
	}
	if (start_ == end && end_ == start)
	{
		return -1;
	}
	return 0;
}

inline int XFoam_BlockEdge::compare(const XFoam_BlockEdge& e) const
{
	return compare(e.start(), e.end());
}

inline int XFoam_BlockEdge::compare(const XFoam_Edge& e) const
{
	return compare(e.start(), e.end());
}

class XFoam_API XFoam_LineEdge : public XFoam_BlockEdge
{
public:
	static constexpr const char* typeName = "lineEdge";

	XFoam_LineEdge(
		const XFoam_UList<XFoam_Vector3D>& points,
		XFoam_Label start,
		XFoam_Label end);

	XFoam_LineEdge(
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_SearchableSurfaceList& geometry,
		const XFoam_UList<XFoam_Vector3D>& points,
		XFoam_IStream& is);

	~XFoam_LineEdge() override;

	XFoam_Vector3D position(XFoam_Scalar lambda) const override;
	XFoam_Scalar length() const override;
};

// 对标 OpenFOAM blockEdgeList / PtrList<blockEdge>（XFoam 命名：XFoam_BlockEdgeList）。
typedef XFoam_PtrList<XFoam_BlockEdge> XFoam_BlockEdgeList;

#endif

