#ifndef XFoam_blockFace_H_
#define XFoam_blockFace_H_

// 对标 OpenFOAM-13：src/mesh/blockMesh/blockFaces/blockFace/* 与 projectFace/*。
// XFoam_declareRunTimeSelectionTable / addToRunTimeSelectionTable：未移植；XFoam_BlockFace::New 仅识别 "plane"、"project"。
// XFoam_Dictionary：前向声明；Istream 构造中无 namedVertices 反查（仅流读顶点编号）。
// XFoam_BlockFace::clone：返回空 autoPtr（与 OF NotImplemented 一致，头文件注明）。
// XFoam_blockFaces::XFoam_PlaneFace：对标 planeFace（OF 独立类），供默认平面离散。

#include "XFoam/snap/xfoam_searchablesurface.h"
#include "XFoam/snap/xfoam_shape.h"
#include "XFoam/utilities/xfoam_common.h"

class XFoam_BlockDescriptor;
class XFoam_Dictionary;

class XFoam_API XFoam_BlockFace
{
protected:
	const XFoam_Face vertices_;

public:
	static constexpr const char* typeName = "blockFace";

	XFoam_BlockFace(const XFoam_Face& vertices);

	XFoam_BlockFace(const XFoam_Dictionary& dict, XFoam_Label index, XFoam_IStream& is);

	virtual ~XFoam_BlockFace();

	const XFoam_Face& vertices() const;

	bool compare(const XFoam_BlockFace& bf) const;
	bool compare(const XFoam_Face& f) const;

	virtual void project(
		const XFoam_BlockDescriptor& desc,
		XFoam_Label blockFacei,
		XFoam_Field<XFoam_Vector3D>& points) const = 0;

	void write(XFoam_OStream& os, const XFoam_Dictionary& d) const;

	virtual XFoam_AutoPtr<XFoam_BlockFace> clone() const;

	static XFoam_AutoPtr<XFoam_BlockFace> New(
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_SearchableSurfaceList& geometry,
		XFoam_IStream& is);

	class INew
	{
		const XFoam_Dictionary& dict_;
		const XFoam_SearchableSurfaceList& geometry_;
		mutable XFoam_Label index_;

	public:
		INew(const XFoam_Dictionary& dict, const XFoam_SearchableSurfaceList& geometry)
			: dict_(dict)
			, geometry_(geometry)
			, index_(0)
		{}

		XFoam_AutoPtr<XFoam_BlockFace> operator()(XFoam_IStream& is) const
		{
			return XFoam_BlockFace::New(dict_, index_++, geometry_, is);
		}
	};

	friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BlockFace& p);
};

inline const XFoam_Face& XFoam_BlockFace::vertices() const
{
	return vertices_;
}

inline bool XFoam_BlockFace::compare(const XFoam_BlockFace& bf) const
{
	return compare(bf.vertices());
}

inline bool XFoam_BlockFace::compare(const XFoam_Face& f) const
{
	return XFoam_Face::sameVertices(vertices_, f);
}

namespace XFoam_blockFaces
{
class XFoam_API XFoam_PlaneFace : public XFoam_BlockFace
{
public:
	static constexpr const char* typeName = "plane";

	XFoam_PlaneFace(const XFoam_Face& vertices);

	XFoam_PlaneFace(const XFoam_Dictionary& dict, XFoam_Label index, XFoam_IStream& is);

	~XFoam_PlaneFace() override = default;

	void project(
		const XFoam_BlockDescriptor& desc,
		XFoam_Label blockFacei,
		XFoam_Field<XFoam_Vector3D>& points) const override;
};

class XFoam_API XFoam_BlockProjectFace : public XFoam_BlockFace
{
	const XFoam_searchableSurface& surface_;

	XFoam_BlockProjectFace(const XFoam_BlockProjectFace&) = delete;
	void operator=(const XFoam_BlockProjectFace&) = delete;

	const XFoam_searchableSurface& lookupSurface(
		const XFoam_SearchableSurfaceList& geometry,
		XFoam_IStream& is) const;

	XFoam_Label index(
		const XFoam_Tuple2<XFoam_Label, XFoam_Label>& n,
		const XFoam_Tuple2<XFoam_Label, XFoam_Label>& coord) const;

	void calcLambdas(
		const XFoam_Tuple2<XFoam_Label, XFoam_Label>& n,
		const XFoam_Field<XFoam_Vector3D>& points,
		XFoam_Field<XFoam_Scalar>& lambdaI,
		XFoam_Field<XFoam_Scalar>& lambdaJ) const;

public:
	static constexpr const char* typeName = "project";

	XFoam_BlockProjectFace(
		const XFoam_Dictionary& dict,
		XFoam_Label index,
		const XFoam_SearchableSurfaceList& geometry,
		XFoam_IStream& is);

	~XFoam_BlockProjectFace() override;

	void project(
		const XFoam_BlockDescriptor& desc,
		XFoam_Label blockFacei,
		XFoam_Field<XFoam_Vector3D>& points) const override;
};
} // namespace XFoam_blockFaces

// 对标 OpenFOAM blockFaceList / PtrList<blockFace>（XFoam 命名：XFoam_BlockFaceList）。
typedef XFoam_PtrList<XFoam_BlockFace> XFoam_BlockFaceList;

#endif
