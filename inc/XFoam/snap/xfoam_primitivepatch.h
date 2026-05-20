#ifndef XFoam_PrimitivePatch_H_
#define XFoam_PrimitivePatch_H_

// 对标 OpenFOAM-13：meshes/primitiveMesh/PrimitivePatch/PrimitivePatch1.H；实现见 xfoam_primitivepatchI.h。
// 参考：https://gitee.com/runfengtsui/OpenFOAM-13/tree/master/src/OpenFOAM
// projectPoints / projectFaceCentres：需 face::ray 与 objectHit 管线，XFoam 中标注为不可用（FatalError）。
// 并发：无 UPstream 交换；几何与拓扑计算均为单线程。
// clearOut / clearGeom / clearTopology / clearPatchMeshAddr 为 virtual，供 XFoam_PolyPatch 等 override（对齐 OF）。

#include "XFoam/snap/xfoam_shape.h"
#include "XFoam/snap/xfoam_intersection.h"

template<class FaceList, class PointField>
class XFoam_PrimitivePatch : public FaceList
{
public:
	typedef typename std::remove_reference<FaceList>::type::value_type face_type;
	typedef typename std::remove_reference<PointField>::type::value_type point_type;

	typedef FaceList FaceListType;
	typedef PointField PointFieldType;
	typedef face_type FaceType;

	typedef XFoam_Tuple2<bool, XFoam_Label> objectHit;
	using ObjectHitList = XFoam_List<objectHit>;
	using LocalFaceList = XFoam_List<face_type>;
	using MeshPointMap = XFoam_Map<XFoam_Label>;

	enum SurfaceTopo
	{
		MANIFOLD,
		OPEN,
		ILLEGAL
	};

private:
	//- Patch points（与构造传入的网格点视图或场一致；movePoints 后失效几何缓存）
	PointField points_;

	//- Edge / face–face 拓扑（demand-driven；edgeTopologyValid_ 统一守卫）
	mutable XFoam_Label nInternalEdges_{-1};
	mutable XFoam_EdgeList edges_;
	mutable bool edgeTopologyValid_{false};
	mutable XFoam_LabelListList faceFaces_;
	mutable XFoam_LabelListList edgeFaces_;
	mutable XFoam_LabelListList faceEdges_;

	//- Point-centred 拓扑
	mutable XFoam_LabelList boundaryPoints_;
	mutable bool boundaryPointsValid_{false};
	mutable XFoam_LabelListList pointEdges_;
	mutable bool pointEdgesValid_{false};
	mutable XFoam_LabelListList pointFaces_;
	mutable bool pointFacesValid_{false};

	//- Mesh 点映射与局部面副本（patch 局部编号 ↔ 全局点）
	mutable bool meshDataValid_{false};
	mutable XFoam_LabelList meshPoints_;
	mutable LocalFaceList localFaces_;
	mutable bool meshPointMapValid_{false};
	mutable MeshPointMap meshPointMap_;

	mutable XFoam_LabelListList edgeLoops_;
	mutable bool edgeLoopsValid_{false};

	//- 局部点序与面几何（clearGeom 清除）
	mutable XFoam_Field<point_type> localPoints_;
	mutable bool localPointsValid_{false};
	mutable XFoam_LabelList localPointOrder_;
	mutable bool localPointOrderValid_{false};
	mutable XFoam_Field<point_type> faceCentres_;
	mutable bool faceCentresValid_{false};
	mutable XFoam_Field<point_type> faceAreas_;
	mutable bool faceAreasValid_{false};
	mutable XFoam_Field<XFoam_Scalar> magFaceAreas_;
	mutable bool magFaceAreasValid_{false};
	mutable XFoam_Field<point_type> faceNormals_;
	mutable bool faceNormalsValid_{false};
	mutable XFoam_Field<point_type> pointNormals_;
	mutable bool pointNormalsValid_{false};

	static const XFoam_Vector3DUList& pointsUList_(const PointField& p)
	{
		return static_cast<const XFoam_Vector3DUList&>(p);
	}

	void calcInternPoints_() const {}

	void calcBdryPoints_() const;
	void calcAddressing_() const;
	void calcPointEdges_() const;
	void calcPointFaces_() const;
	void calcMeshData_() const;
	void calcMeshPointMap_() const;
	void calcEdgeLoops_() const;
	void calcLocalPoints_() const;
	void calcLocalPointOrder_() const;
	void calcFaceCentres_() const;
	void calcFaceAreas_() const;
	void calcMagFaceAreas_() const;
	void calcFaceNormals_() const;
	void calcPointNormals_() const;

	void visitPointRegion(
		const XFoam_Label pointi,
		const XFoam_LabelList& pFaces,
		const XFoam_Label startFacei,
		const XFoam_Label startEdgeI,
		XFoam_UList<XFoam_UInt8>& pFacesVisited) const;

public:
	XFoam_PrimitivePatch(const FaceList& faces, const PointField& points);
	XFoam_PrimitivePatch(FaceList&& faces, const PointField& points);
	XFoam_PrimitivePatch(const XFoam_PrimitivePatch& pp);
	XFoam_PrimitivePatch(XFoam_PrimitivePatch&& pp) noexcept;

	virtual ~XFoam_PrimitivePatch();

	void swap(XFoam_PrimitivePatch&) = delete;

	virtual void clearOut();
	virtual void clearGeom();
	virtual void clearTopology();
	virtual void clearPatchMeshAddr();

	const PointField& points() const noexcept { return points_; }

	XFoam_Label nFaces() const noexcept
	{
		return static_cast<XFoam_Label>(this->size());
	}

	XFoam_Label nPoints() const { return meshPoints().size(); }
	XFoam_Label nEdges() const { return edges().size(); }

	const XFoam_EdgeList& edges() const;

	XFoam_SubList<XFoam_Edge> internalEdges() const;
	XFoam_SubList<XFoam_Edge> boundaryEdges() const;

	XFoam_Label nInternalEdges() const;
	XFoam_Label nBoundaryEdges() const;

	bool isInternalEdge(const XFoam_Label edgei) const
	{
		return edgei < nInternalEdges();
	}

	const XFoam_LabelList& boundaryPoints() const;

	const XFoam_LabelListList& faceFaces() const;
	const XFoam_LabelListList& edgeFaces() const;
	const XFoam_LabelListList& faceEdges() const;
	const XFoam_LabelListList& pointEdges() const;
	const XFoam_LabelListList& pointFaces() const;

	const LocalFaceList& localFaces() const;

	XFoam_LabelList boundaryFaces() const;
	XFoam_LabelList uniqBoundaryFaces() const;

	const XFoam_LabelList& meshPoints() const;
	const MeshPointMap& meshPointMap() const;

	const XFoam_Field<point_type>& localPoints() const;
	const XFoam_LabelList& localPointOrder() const;

	XFoam_Label whichPoint(const XFoam_Label gp) const;

	XFoam_Edge meshEdge(const XFoam_Label edgei) const;
	XFoam_Edge meshEdge(const XFoam_Edge& e) const;
	XFoam_Label findEdge(const XFoam_Edge& e) const;

	XFoam_LabelList meshEdges(
		const XFoam_EdgeList& allEdges,
		const XFoam_LabelListList& cellEdges,
		const XFoam_LabelList& faceCells) const;

	XFoam_LabelList meshEdges(
		const XFoam_EdgeList& allEdges,
		const XFoam_LabelListList& meshPointEdges) const;

	XFoam_Label meshEdge(
		const XFoam_Label edgei,
		const XFoam_EdgeList& allEdges,
		const XFoam_LabelListList& meshPointEdges) const;

	XFoam_LabelList meshEdges(
		const XFoam_LabelUList& edgeLabels,
		const XFoam_EdgeList& allEdges,
		const XFoam_LabelListList& meshPointEdges) const;

	const XFoam_Field<point_type>& faceCentres() const;
	const XFoam_Field<point_type>& faceAreas() const;
	const XFoam_Field<XFoam_Scalar>& magFaceAreas() const;
	const XFoam_Field<point_type>& faceNormals() const;
	const XFoam_Field<point_type>& pointNormals() const;

	XFoam_Tuple2<point_type, point_type> box() const;
	XFoam_Scalar sphere(const XFoam_Label facei) const;

	bool hasFaceAreas() const { return faceAreasValid_; }
	bool hasFaceCentres() const { return faceCentresValid_; }
	bool hasFaceNormals() const { return faceNormalsValid_; }
	bool hasPointNormals() const { return pointNormalsValid_; }
	bool hasBoundaryPoints() const { return boundaryPointsValid_; }
	bool hasEdges() const { return edgeTopologyValid_; }
	bool hasFaceFaces() const { return edgeTopologyValid_; }
	bool hasEdgeFaces() const { return edgeTopologyValid_; }
	bool hasFaceEdges() const { return edgeTopologyValid_; }
	bool hasPointEdges() const { return pointEdgesValid_; }
	bool hasPointFaces() const { return pointFacesValid_; }
	bool hasMeshPoints() const { return meshDataValid_; }
	bool hasMeshPointMap() const { return meshPointMapValid_; }

	template<class ToPatch>
	ObjectHitList projectPoints(
		const ToPatch& targetPatch,
		const XFoam_Field<point_type>& projectionDirection,
		const XFoam_Intersection::algorithm alg = XFoam_Intersection::algorithm::fullRay,
		const XFoam_Intersection::direction dir = XFoam_Intersection::direction::vector) const;

	template<class ToPatch>
	ObjectHitList projectFaceCentres(
		const ToPatch& targetPatch,
		const XFoam_Field<point_type>& projectionDirection,
		const XFoam_Intersection::algorithm alg = XFoam_Intersection::algorithm::fullRay,
		const XFoam_Intersection::direction dir = XFoam_Intersection::direction::vector) const;

	const XFoam_LabelListList& edgeLoops() const;

	SurfaceTopo surfaceType(XFoam_LabelHashSet* badEdgesPtr = nullptr) const;

	bool checkTopology(const bool report = false, XFoam_LabelHashSet* pointSetPtr = nullptr)
		const;

	bool checkPointManifold(const bool report = false, XFoam_LabelHashSet* pointSetPtr = nullptr)
		const;

	virtual void movePoints(const XFoam_Field<point_type>&);

	XFoam_PrimitivePatch& operator=(const XFoam_PrimitivePatch& pp);
	XFoam_PrimitivePatch& operator=(XFoam_PrimitivePatch&& pp) noexcept;

	XFoam_Label whichEdge(const XFoam_Edge& e) const { return findEdge(e); }
};

// 头文件包含位置：见 doc/foam_code.md「配套 *I.h」例外（须在模板类主声明闭合之后）。
#include "XFoam/snap/xfoam_primitivepatchI.h"

#endif
