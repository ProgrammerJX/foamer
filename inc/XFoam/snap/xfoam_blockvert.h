#ifndef XFoam_blockVertex_H_
#define XFoam_blockVertex_H_

// 对标 OpenFOAM-13：src/mesh/blockMesh/blockVertices/blockVertex/*、pointVertex/*。
// XFoam_declareRunTimeSelectionTable / addToRunTimeSelectionTable：未移植。
// XFoam_BlockVert::New：首 token 为 '(' 时构造 XFoam_PointVert；为 word 时仅识别 "point"（其余 FatalError）。
// namedVertices / blockMeshTools::read|write：无 dictionary 子字典管线；read/write 退化为标量 label 直读直写（头文件注明）。
// XFoam_BlockVert::clone：与 OF NotImplemented 一致，返回空 autoPtr（头文件注明）。

#include "XFoam/snap/xfoam_searchablesurface.h"
#include "XFoam/utilities/xfoam_common.h"

class XFoam_Dictionary;

class XFoam_API XFoam_BlockVert
{
public:
	static constexpr const char* typeName = "blockVertex";

	XFoam_BlockVert();

	virtual XFoam_AutoPtr<XFoam_BlockVert> clone() const;

	static XFoam_AutoPtr<XFoam_BlockVert> New(
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

		XFoam_AutoPtr<XFoam_BlockVert> operator()(XFoam_IStream& is) const
		{
			return XFoam_BlockVert::New(dict_, index_++, geometry_, is);
		}
	};

	virtual ~XFoam_BlockVert();

	virtual operator XFoam_Vector3D() const = 0;

	static XFoam_Label read(XFoam_IStream& is, const XFoam_Dictionary& dict);

	static void write(XFoam_OStream& os, const XFoam_Label val, const XFoam_Dictionary& d);
};

class XFoam_API XFoam_PointVert : public XFoam_BlockVert
{
protected:
	XFoam_Vector3D vertex_;

public:
	static constexpr const char* typeName = "point";

	XFoam_PointVert(
		const XFoam_Dictionary&,
		XFoam_Label index,
		const XFoam_SearchableSurfaceList& geometry,
		XFoam_IStream& is);

	~XFoam_PointVert() override;

	operator XFoam_Vector3D() const override;
};

// 对标 OpenFOAM blockVertList / PtrList<blockVert>（XFoam 命名：XFoam_BlockVertList）。
typedef XFoam_PtrList<XFoam_BlockVert> XFoam_BlockVertList;

inline XFoam_PointField XFoam_vertices(const XFoam_BlockVertList& bvl)
{
    XFoam_PointField vertices(bvl.size());
    for (XFoam_Label pi = 0; pi < bvl.size(); ++pi)
    {
        vertices[pi] = static_cast<XFoam_Vector3D>(bvl[pi]);
    }
    return vertices;
}

#endif
