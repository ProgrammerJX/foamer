#ifndef XFoam_PrimitiveMesh_H_
#define XFoam_PrimitiveMesh_H_

// 拓扑骨架：XFoam_PrimitivePatch（PrimitivePatch/PrimitivePatch1.H）与 XFoam_PrimitiveMesh（primitiveMesh.H）。
// 对照 OpenFOAM-13：src/OpenFOAM/meshes/primitiveMesh/primitiveMesh/primitiveMesh.H
// 参考：https://gitee.com/runfengtsui/OpenFOAM-13/tree/master/src/OpenFOAM
// 说明：完整 primitiveMesh（全部 demand-driven 指针成员与算法）依赖 objectRegistry、PackedBoolList、
// DynamicList 等类型链；当前 XFoam 仅为可运行的拓扑子集 + 纯虚点/面/owner/neighbour 访问与 reset/
// movePoints/calcCells 等辅助。补齐需本地克隆 OF 源码后逐项移植（见 tmp/src/OpenFOAM）。
// clearOut / movePoints 为 virtual，便于 XFoam_PolyMesh 等在析构/reset 链上扩展（对标 OF 多态清理）。

#include "XFoam/mesh/xfoam_shape.h"
#include "XFoam/mesh/xfoam_primitivepatch.h"

typedef XFoam_PrimitivePatch<XFoam_FaceList, XFoam_Vector3DUList> XFoam_FacePrimitivePatch;

/// 与 OpenFOAM Foam::primitiveMesh 对齐的最小接口：尺寸、纯虚访问器、reset、静态单元装配与点序检测。
class XFoam_API XFoam_PrimitiveMesh
{
protected:
	//- Mesh dimensions（对标 primitiveMesh 尺寸成员；nEdges_ / nInternalPoints_ 可为 -1 表示未计算或未排序）
	XFoam_Label nInternalPoints_;
	XFoam_Label nPoints_;
	XFoam_Label nEdges_;
	XFoam_Label nInternalFaces_;
	XFoam_Label nFaces_;
	XFoam_Label nCells_;
public:
	//- Estimated number of cells per edge
	static const unsigned cellsPerEdge_ = 4;

	//- Estimated number of cells per point
	static const unsigned cellsPerPoint_ = 8;

	//- Estimated number of faces per cell
	static const unsigned facesPerCell_ = 6;

	//- Estimated number of faces per edge
	static const unsigned facesPerEdge_ = 4;

	//- Estimated number of faces per point
	static const unsigned facesPerPoint_ = 12;

	//- Estimated number of edges per cell
	static const unsigned edgesPerCell_ = 12;

	//- Estimated number of edges per cell
	static const unsigned edgesPerFace_ = 4;

	//- Estimated number of edges per point
	static const unsigned edgesPerPoint_ = 6;

	//- Estimated number of points per cell
	static const unsigned pointsPerCell_ = 8;

	//- Estimated number of points per face
	static const unsigned pointsPerFace_ = 4;

public:
	static constexpr const char* typeName = "primitiveMesh";

	XFoam_PrimitiveMesh();
	XFoam_PrimitiveMesh(
		XFoam_Label nPoints,
		XFoam_Label nInternalFaces,
		XFoam_Label nFaces,
		XFoam_Label nCells);

	XFoam_PrimitiveMesh(const XFoam_PrimitiveMesh&) = delete;
	XFoam_PrimitiveMesh& operator=(const XFoam_PrimitiveMesh&) = delete;
	XFoam_PrimitiveMesh(XFoam_PrimitiveMesh&&) noexcept = default;
	XFoam_PrimitiveMesh& operator=(XFoam_PrimitiveMesh&&) noexcept = default;

	virtual ~XFoam_PrimitiveMesh();

	XFoam_Label nInternalPoints() const { return nInternalPoints_; }
	XFoam_Label nPoints() const { return nPoints_; }
	XFoam_Label nEdges() const { return nEdges_; }
	XFoam_Label nInternalFaces() const { return nInternalFaces_; }
	XFoam_Label nFaces() const { return nFaces_; }
	XFoam_Label nCells() const { return nCells_; }

	bool isInternalFace(const XFoam_Label facei) const { return facei < nInternalFaces_; }

	virtual const XFoam_Vector3DUList& points() const = 0;
	virtual const XFoam_FaceUList& faces() const = 0;
	virtual const XFoam_LabelUList& faceOwner() const = 0;
	virtual const XFoam_LabelUList& faceNeighbour() const = 0;

	virtual const XFoam_Vector3DUList& oldPoints() const { return points(); }

	virtual void clearGeom() {}
	virtual void clearOut();

	virtual XFoam_ScalarList movePoints(
		const XFoam_Vector3DUList& newPoints,
		const XFoam_Vector3DUList& oldPoints);

	void reset(
		XFoam_Label nPoints,
		XFoam_Label nInternalFaces,
		XFoam_Label nFaces,
		XFoam_Label nCells);

	static bool calcPointOrder(
		XFoam_Label& nInternalPoints,
		XFoam_LabelList& oldToNew,
		const XFoam_FaceUList& meshFaces,
		XFoam_Label nInternalFaces,
		XFoam_Label nPoints);

	static void calcCells(
		XFoam_CellList& cellFaces,
		const XFoam_LabelUList& own,
		const XFoam_LabelUList& nei,
		XFoam_Label nCells);
};

#endif
