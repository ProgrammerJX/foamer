#ifndef XFoam_VBrep_H_
#define XFoam_VBrep_H_

#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_common.h"

#include <vector>

// =============================================================================
// XFoam_VBrep ── 离散（三角化）B-rep
//
// 设计意图：承接所有"已经离散为三角面"的输入 —— STL、Nastran shell mesh (BDF)、
// 由 MBrep::tessellate() 烘焙出来的三角网，等等。提供三层结构：
//   vertex(positions_) → edge(edges_) → face(faces_，三角面)
// edges_ 不是必需的（很多输入只给点和三角面），调用 buildEdgesFromFaces() 才
// 会从 faces_ 重建 edge 表并标 feature / boundary。
//
// 几何查询（BrepBase 接口）：原先在独立类 XFoam_TriSurface 里的 BVH /
// closestPoint / contains / boxIntersects / feature edge 检测都已并入本类。
// 私有的 triCache_ / bvhNodes_ / featureEdges_ / featureVerts_ 是查询用的延迟
// 缓存：第一次调用查询接口时由 ensureAcceleration() 从 positions_ + faces_
// 重建；如果外部改动了 positions_ / faces_，**必须**调 invalidateAcceleration()
// 让下次查询重新建。
// =============================================================================

class XFoam_API XFoam_VBrep : public XFoam_BrepBase
{
public:
	// 一条三角面之间的"边"。verts 排过序（小 → 大）便于做 face-邻接哈希。
	struct DiscreteEdge
	{
		XFoam_FixedList<XFoam_Label, 2> verts;
		XFoam_Label  faceL       = -1;
		XFoam_Label  faceR       = -1;       ///< 第二张邻面；-1 表示 boundary edge
		XFoam_Scalar dihedralCos = 1;        ///< buildEdges() 算出来；1 表示共面
		bool         isFeature   = false;
		bool         isBoundary  = false;
	};

	// 离散三角面。patchId 用于一对多组织 patch（STL solid 名 / BDF HMMOVE id 等）。
	struct DiscreteFace
	{
		XFoam_FixedList<XFoam_Label, 3> verts;
		XFoam_Label patchId = -1;
	};

	XFoam_VBrep();
	~XFoam_VBrep() override;

	XFoam_VBrep(const XFoam_VBrep&)            = default;
	XFoam_VBrep& operator=(const XFoam_VBrep&) = default;
	XFoam_VBrep(XFoam_VBrep&&) noexcept        = default;
	XFoam_VBrep& operator=(XFoam_VBrep&&) noexcept = default;

	XFoam_BrepKind kind() const override { return XFoam_BrepKind::Triangulated; }

	// ---- 数据 ----
	XFoam_Label nVerts() const { return positions_.size(); }
	XFoam_Label nEdges() const { return edges_.size(); }
	XFoam_Label nFaces() const { return faces_.size(); }
	const XFoam_List<XFoam_Vector3D>& positions() const { return positions_; }
	const XFoam_List<DiscreteEdge>&   edges()     const { return edges_; }
	const XFoam_List<DiscreteFace>&   faces()     const { return faces_; }
	XFoam_List<XFoam_Vector3D>&       positionsRef() { return positions_; }
	XFoam_List<DiscreteEdge>&         edgesRef()     { return edges_; }
	XFoam_List<DiscreteFace>&         facesRef()     { return faces_; }

	// ---- 每 patchId 的元数据（patch 名 / 类型）。patchId 直接做下标；
	//      未命名时取 "patch_<id>"、wall 类型。----
	XFoam_Label nPatches() const { return patchNames_.size(); }
	const XFoam_WordList& patchNames() const { return patchNames_; }
	const XFoam_WordList& patchTypes() const { return patchTypes_; }
	/// 确保至少有 nPatches 个 patch 元数据条目（缺的补默认）。
	void ensurePatches(XFoam_Label nPatches);
	void setPatchName(XFoam_Label id, const XFoam_Word& name);
	void setPatchType(XFoam_Label id, const XFoam_Word& type);

	// ---- I/O ----
	/// ASCII STL；solid 名分别落成不同 patchId（按出现顺序编号）。
	void readStlAscii(const XFoam_String& fileName);
	/// Binary STL；attribute byte count 当 patchId（典型 0）。
	void readStlBinary(const XFoam_String& fileName);
	/// Nastran BDF：CTRIA3 / CQUAD4 + GRID / GRID*；
	/// $HMMOVE 块每个 component → 单独 patchId（保留旧 MBrep 行为）。
	void readFromBdf(const XFoam_String& fileName);
	/// ASCII STL 写出。每个 patchId 一个 solid 块。
	void writeStl(const XFoam_String& fileName) const;

	/// 便利包装：吞掉 readStlAscii 抛出的 XFoam_Error / std::exception，转成 bool
	/// 返回。兼容原 XFoam_TriSurface::read() 的 bool 语义，方便 tests / samples 无痛
	/// 迁移。目前只走 ASCII；待 readStlBinary 实现后会按文件首字节自动分派。
	bool read(const std::string& path);

	// ---- 构建 / 修补（首期 stub，签名稳定，先返回 0 / 无操作）----
	/// 从 faces_ 重建 edges_，按 featureAngleDeg 标 dihedral 阈值；
	/// 同时给只被 1 个面引用的 edge 标 boundary。首期：实现简化版（按 vert 对
	/// 去重 + dihedral 计算），不做 manifold 完备性检查。
	void buildEdgesFromFaces(XFoam_Scalar featureAngleDeg);

	/// 合并 < tol 距离的 vertex（dangling）。首期 stub，返回 0。
	XFoam_Label stitchBorders(XFoam_Scalar tol);

	/// 共面三角面归到同一 patchId（dihedral < tolDeg）。首期 stub，返回 0。
	XFoam_Label unifySameDomain(XFoam_Scalar coplanarTolDeg);

	void clear() override;
	XFoam_BoundBox bounds() const override;
	XFoam_BoundBox refBounds(const XFoam_BrepRef& r) const override;

	// ---- BrepBase 几何查询 API（从原 XFoam_TriSurface 吸收）----
	bool empty() const override { return faces_.size() == 0; }
	bool contains(const XFoam_Vector3D& p) const override;
	bool boxIntersects(const XFoam_BoundBox& box) const override;
	void closestPointAndNormal(
		const XFoam_Vector3D& p,
		XFoam_Vector3D&       outClosest,
		XFoam_Vector3D&       outNormal) const override;
	XFoam_Label nFeatureEdges() const override
	{
		return static_cast<XFoam_Label>(featureEdges_.size());
	}
	XFoam_Label nFeatureVertices() const override
	{
		return static_cast<XFoam_Label>(featureVerts_.size());
	}
	XFoam_Vector3D featureVertexPosition(XFoam_Label i) const override
	{
		ensureAcceleration();
		if (i < 0 || i >= static_cast<XFoam_Label>(featureVerts_.size()))
			return XFoam_Vector3D(0, 0, 0);
		return featureVerts_[static_cast<std::size_t>(i)].p;
	}
	FeatureKind closestFeature(
		const XFoam_Vector3D& p,
		XFoam_Scalar          searchRadius,
		XFoam_Vector3D&       outClosest,
		XFoam_Vector3D&       outTangent) const override;
	/// VBrep 的 buildFeatures 等同 buildEdgesFromFaces + 把 isFeature/isBoundary
	/// edge 物化到独立 featureEdges_ 列表（snap 查询用）。会顺带调
	/// invalidateAcceleration() —— 之后调任何查询都会重建 BVH。
	void buildFeatures(XFoam_Scalar featureAngleDeg) override;

	// ---- sub-patch（一面一 patch）API ----
	// VBrep 把 patchId（DiscreteFace.patchId，源自 STL solid / BDF component
	// / MBrep face id）当 sub-patch。closestSubPatchId 走 BVH 找最近 tri 后
	// 返回该 tri 的 patchId。
	XFoam_Label nSubPatches() const override
	{
		return static_cast<XFoam_Label>(patchNames_.size());
	}
	XFoam_String subPatchName(XFoam_Label id) const override;
	XFoam_Label closestSubPatchId(const XFoam_Vector3D& p) const override;
	XFoam_BoundBox subPatchBounds(XFoam_Label id) const override;
	XFoam_Scalar subPatchMinFeatureLength(XFoam_Label id) const override;

	/// 最短 feature edge 段（DiscreteEdge）长度。先 ensureAcceleration() 确保
	/// featureEdges_ 被填好；buildFeatures 未调用过时返回 0。
	XFoam_Scalar minFeatureLength() const override;

	/// 把 BVH / feature 缓存标记为"过期"。外部直接改 positions_ / faces_ 后必须调。
	/// 下次 closestPoint 等会按需重建。
	void invalidateAcceleration() const { accelBuilt_ = false; }

private:
	XFoam_List<XFoam_Vector3D> positions_;
	XFoam_List<DiscreteEdge>   edges_;
	XFoam_List<DiscreteFace>   faces_;
	XFoam_WordList             patchNames_;
	XFoam_WordList             patchTypes_;

	// ======== 查询加速缓存（mutable，延迟构建）========
	// 选用 std::vector 而非 XFoam_List：内部纯私有，且 BVH 算法要 std::sort，
	// 配 std::vector 最自然。
	struct TriCache
	{
		XFoam_Vector3D v0, v1, v2;
		XFoam_Vector3D normal;
		XFoam_BoundBox bbox;
	};
	struct BvhNode
	{
		XFoam_BoundBox bbox;
		int leftIdx  = -1;   ///< internal: left child idx；leaf: -1
		int rightIdx = -1;   ///< internal: right child idx；leaf: -1
		int firstTri = -1;   ///< leaf: bvhOrder_ 起点；internal: -1
		int triCount = 0;    ///< leaf: tri count（>0）；internal: 0
	};
	struct FeatureEdgeGeom { XFoam_Vector3D p1, p2; };
	struct FeatureVertGeom { XFoam_Vector3D p; };

	mutable std::vector<TriCache>        triCache_;
	mutable std::vector<BvhNode>         bvhNodes_;
	mutable std::vector<XFoam_Label>     bvhOrder_;
	mutable XFoam_BoundBox               cachedBounds_;
	mutable std::vector<FeatureEdgeGeom> featureEdges_;
	mutable std::vector<FeatureVertGeom> featureVerts_;
	mutable bool                         accelBuilt_   = false;
	mutable bool                         featuresBuilt_ = false;
	mutable XFoam_Scalar                 featureAngleDegCached_ = 30; ///< 上次 buildFeatures 用的阈值
	static constexpr int                 kBvhLeafLimit = 4;

	void ensureAcceleration() const; ///< triCache_ + BVH 没建过就建一次
	void buildBvh() const;
	int  buildBvhRecursive(int lo, int hi) const;
	static XFoam_Scalar bboxMinDistSqr(
		const XFoam_Vector3D& p, const XFoam_BoundBox& bb);
	void bvhClosestPoint(
		const XFoam_Vector3D& p,
		XFoam_Scalar&         bestD2,
		XFoam_Vector3D&       bestQ,
		XFoam_Label&          bestTri,
		int                   nodeIdx) const;
	int  bvhRayCountPlusX(const XFoam_Vector3D& p, int nodeIdx) const;
	bool bvhBoxIntersects(const XFoam_BoundBox& q, int nodeIdx) const;
	int  rayCountPlusX(const XFoam_Vector3D& p) const;
	void rebuildFeaturesGeomFromEdges(XFoam_Scalar featureAngleDeg) const;
};

#endif // XFoam_VBrep_H_
