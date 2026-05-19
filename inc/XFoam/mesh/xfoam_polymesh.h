#ifndef XFoam_PolyMesh_H_
#define XFoam_PolyMesh_H_

// 对标 OpenFOAM-13：meshes/polyMesh/polyMesh/polyMesh.H / polyMesh.C（拓扑与边界子集）。
// 参考：https://gitee.com/runfengtsui/OpenFOAM-13/tree/master/src/OpenFOAM
// 未移植：objectRegistry 继承、point/face/cell zones、IOobject/IOdictionary 读入、globalMeshData、
// 并行与动网格 demand-driven 成员等。当前仅保留点/面/owner/neighbour、边界与局部几何缓存。

#include "XFoam/mesh/xfoam_polyboundarymesh.h"
#include "XFoam/mesh/xfoam_primitivemesh.h"
#include "XFoam/utilities/xfoam_common.h"

class XFoam_API XFoam_PolyMesh
	: public XFoam_PrimitiveMesh
{
	XFoam_PointField points_;
	XFoam_FaceList faces_;
	XFoam_LabelList owner_;
	XFoam_LabelList neighbour_;

	bool clearedPrimitives_{false};

	XFoam_PolyBoundaryMesh boundary_;

	mutable XFoam_BoundBox bounds_;

	mutable XFoam_List<XFoam_Edge> edges_;
	mutable XFoam_List<XFoam_LabelList> pointEdges_;
	mutable bool edgesValid_{false};

	mutable XFoam_Field<XFoam_Vector3D> faceCentres_;
	mutable XFoam_Field<XFoam_Vector3D> faceAreas_;
	mutable bool faceGeomValid_{false};

	mutable XFoam_Field<XFoam_Vector3D> cellCentres_;
	mutable bool cellCentresValid_{false};

	mutable XFoam_List<XFoam_Cell> cellFaces_;

	void calcEdges() const;
	void calcFaceGeom() const;
	void calcCellCentres() const;
	void calcCellFaces();

	/// 移植参考: OpenFOAM polyMeshFromShapeMesh.C（由 cellShape 合并面、owner/neighbour、patch 起址与 cell→face 表）。
	/// 要求 points_ 已设好尺寸，用于校验顶点号；写入 faces_/owner_/neighbour_ 及输出参数。
	void setTopology
	(
		const XFoam_CellShapeList& cellsAsShapes,
		const XFoam_FaceListList& boundaryFaces,
		const XFoam_WordList& boundaryPatchNames,
		XFoam_LabelList& patchSizes,
		XFoam_LabelList& patchStarts,
		XFoam_Label& defaultPatchStart,
		XFoam_Label& nFaces,
		XFoam_CellList& cells
	);
	XFoam_LabelListList cellShapePointCells(const XFoam_CellShapeList& cellsAsShapes) const;
	XFoam_LabelList facePatchFaceCells
	(
		const XFoam_FaceList& patchFaces,
		const XFoam_LabelListList& pointCells,
		const XFoam_FaceListList& cellsFaceShapes,
		const XFoam_Label patchID
	) const;
public:
	static constexpr const char* typeName = "polyMesh";

	XFoam_PolyMesh(const XFoam_PolyMesh&) = delete;
	void operator=(const XFoam_PolyMesh&) = delete;

	/// 移植源码: OpenFOAM src/OpenFOAM/meshes/polyMesh/polyMesh/polyMesh.C（pointField / faceList / owner / neighbour + 边界 patch 元数据；无 IOobject 的拓扑装配子集）
	/// 命名规范: foam_code.md
	/// 移植规范: foam_code.md
	XFoam_PolyMesh(
		XFoam_PointField&& points,
		XFoam_FaceList&& faces,
		XFoam_LabelList&& own,
		XFoam_LabelList&& nei,
		const XFoam_WordList& patchNames,
		const XFoam_LabelList& patchSizes,
		const XFoam_WordList& patchTypes);

	/// 由 cellShapeList + boundaryFaces + patch 元数据装配 polyMesh（合并原 blockMesh 风格与 polyMeshFromShapeMesh 风格；已去除 meshDatabasePath / meshRegion）。
	/// 移植参考: polyMesh.C（拓扑子集）与 polyMeshFromShapeMesh.C；按 cellShape 面合并内面、装配 owner/neighbour；边界可由未配对 cell 面或显式 boundaryFaces 给出（当前 cell 模型以 hex 为主）。
	/// 命名规范: foam_code.md
	/// 移植规范: foam_code.md
	XFoam_PolyMesh(
		XFoam_PointField pointsField,
		const XFoam_CellShapeList& cellsAsShapes,
		const XFoam_FaceListList& boundaryFaces,
		XFoam_WordList boundaryPatchNames,
		const XFoam_PtrListDictionary<XFoam_Dictionary>& boundaryPatchDicts,
		const XFoam_Word& defaultBoundaryPatchName,
		const XFoam_Word& defaultBoundaryPatchType
	);

	~XFoam_PolyMesh() override;

	const XFoam_PolyMesh& mesh() const noexcept { return *this; }

	const XFoam_UList<XFoam_Vector3D>& points() const override { return points_; }

	const XFoam_UList<XFoam_Face>& faces() const override { return faces_; }

	const XFoam_UList<XFoam_Label>& faceOwner() const override { return owner_; }

	const XFoam_UList<XFoam_Label>& faceNeighbour() const override { return neighbour_; }

	const XFoam_PolyBoundaryMesh& boundary() const { return boundary_; }

	XFoam_PolyBoundaryMesh& boundary() { return boundary_; }

	XFoam_Label nBoundaryFaces() const noexcept { return nFaces() - nInternalFaces(); }

	const XFoam_BoundBox& bounds() const noexcept { return bounds_; }

	const XFoam_List<XFoam_Edge>& edges() const;

	const XFoam_List<XFoam_LabelList>& pointEdges() const;

	const XFoam_Field<XFoam_Vector3D>& faceCentres() const;

	const XFoam_Field<XFoam_Vector3D>& faceAreas() const;

	const XFoam_Field<XFoam_Vector3D>& cellCentres() const;

	void clearGeom() override;

	void removeFiles() const;
	bool write(const bool doWrite = true) const;
	/// \brief 写入有限元模型到文件
	/// \param fileName 默认输出bdf文件
	/// \param doWrite 是否写入文件
	/// \return 是否成功
	/// \note 仅输出四边形的Face和所有的Edge
	bool writeFEM(XFoam_FileName fileName = XFoam_FileName("fem.bdf"), bool doWrite = true) const;
};

#endif
