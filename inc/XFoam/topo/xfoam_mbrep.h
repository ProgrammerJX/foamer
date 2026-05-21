#ifndef XFoam_MBrep_H_
#define XFoam_MBrep_H_

#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/topo/xfoam_vbrep.h"
#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_common.h"

#include <vector>

// =============================================================================
// XFoam_MBrep ── 参数化（model / mathematical）B-rep
//
// 承接 STEP / IGES 等 CAD 输入；实体粒度与 OCCT 的 TopoDS_Vertex / Edge /
// Face / Solid 一一对应。OCCT 句柄通过 pImpl (OcctData) 隐藏在 cpp 内，避免把
// OCCT 头文件挤进公有头。
//
// 编译开关 XFOAM_WITH_OCCT：
//   ON  → 完整支持 readFromStep / readFromIges / tessellate（走 BRepMesh）
//   OFF → 三个 read* 抛 XFoam_Error("OCCT disabled at build time")；
//         tessellate() 仅在 ParametricFace::meshedTris 已被外部填好时是 no-op，
//         否则也抛错。这样头文件 / API 完全一致，但功能矩阵随构建变。
//
// 离散化 → snappy：snappy 只吃三角面集合。MBrep 自身不直接走 snap，必须先
// tessellate() → toVBrep() 把所有 ParametricFace 烘成一个 VBrep。TopoModel
// 的 exportToSnappy() 内部会替用户做这一步。
// =============================================================================

class XFoam_API XFoam_MBrep : public XFoam_BrepBase
{
public:
	struct ParametricVertex
	{
		XFoam_Vector3D p;

		ParametricVertex() : p(0, 0, 0) {}
		explicit ParametricVertex(const XFoam_Vector3D& q) : p(q) {}
	};

	struct ParametricEdge
	{
		XFoam_Label  v0         = -1;
		XFoam_Label  v1         = -1;
		XFoam_Scalar firstParam = 0;
		XFoam_Scalar lastParam  = 0;
		/// 弦高离散点；OCCT ON 时 tessellate() 自动填充，OFF 时由 reader 直接填。
		std::vector<XFoam_Vector3D> sampled;
	};

	struct ParametricFace
	{
		XFoam_LabelList              outerLoop;  ///< 外环 edge id 序列（首尾相接）
		std::vector<XFoam_LabelList> innerLoops; ///< 孔环列表
		/// tessellate() 结果：以本 face 局部 vertex 表 + 三角面索引为载体。
		std::vector<XFoam_Vector3D>                  meshedPts;
		std::vector<XFoam_FixedList<XFoam_Label, 3>> meshedTris;
	};

	struct ParametricBody
	{
		XFoam_LabelList faces;            ///< 边界面 id 列表
		bool            closed = true;    ///< 封闭实体 vs 开壳
	};

	XFoam_MBrep();
	~XFoam_MBrep() override;

	XFoam_MBrep(const XFoam_MBrep&)            = delete;  ///< OcctData 不可平凡拷贝
	XFoam_MBrep& operator=(const XFoam_MBrep&) = delete;
	XFoam_MBrep(XFoam_MBrep&&) noexcept;
	XFoam_MBrep& operator=(XFoam_MBrep&&) noexcept;

	XFoam_BrepKind kind() const override { return XFoam_BrepKind::Parametric; }

	// ---- 数据 ----
	XFoam_Label nVerts()  const { return verts_.size(); }
	XFoam_Label nEdges()  const { return edges_.size(); }
	XFoam_Label nFaces()  const { return faces_.size(); }
	XFoam_Label nBodies() const { return bodies_.size(); }
	const XFoam_List<ParametricVertex>& verts()  const { return verts_; }
	const XFoam_List<ParametricEdge>&   edges()  const { return edges_; }
	const XFoam_List<ParametricFace>&   faces()  const { return faces_; }
	const XFoam_List<ParametricBody>&   bodies() const { return bodies_; }
	XFoam_List<ParametricVertex>&       vertsRef()  { return verts_; }
	XFoam_List<ParametricEdge>&         edgesRef()  { return edges_; }
	XFoam_List<ParametricFace>&         facesRef()  { return faces_; }
	XFoam_List<ParametricBody>&         bodiesRef() { return bodies_; }

	// ---- patch 元数据：与 VBrep 等同（按 face id 落 patch 名）----
	const XFoam_WordList& faceNames() const { return faceNames_; }
	const XFoam_WordList& faceTypes() const { return faceTypes_; }
	void setFaceName(XFoam_Label id, const XFoam_Word& name);
	void setFaceType(XFoam_Label id, const XFoam_Word& type);

	// ---- I/O ----
	/// 仅在 XFOAM_WITH_OCCT=ON 下可用；否则抛 XFoam_Error。
	void readFromStep(const XFoam_String& fileName);
	void readFromIges(const XFoam_String& fileName);

	// ---- 离散化 ----
	/// 对每个 ParametricFace 三角化（弦高 deflection）。OCCT ON 时调用
	/// BRepMesh_IncrementalMesh；OFF 时仅在 meshedTris 已外部填好时通过。
	void tessellate(XFoam_Scalar deflection);

	/// 把当前 MBrep（必须先 tessellate）拷成一个 VBrep —— snappy 输入前的
	/// 固化步骤。每个 ParametricFace 的 meshedPts 全部并到 VBrep.positions_，
	/// 三角面写入 VBrep.faces_ 时把 patchId 设为本 face 的 id。
	XFoam_AutoPtr<XFoam_VBrep> toVBrep() const;

	void clear() override;
	XFoam_BoundBox bounds() const override;
	XFoam_BoundBox refBounds(const XFoam_BrepRef& r) const override;

	// ---- BrepBase 几何查询 API（OCCT 后端）----
	// VBrep 用 BVH-over-triangles：~µs / 查询，误差 = O(deflection)。
	// MBrep 用 OCCT analytic：BRepExtrema_DistShapeShape / IntCurvesFace 等，
	// ~ms / 查询，误差到数值精度。MBrep 路径不需要 tessellate 也能查 ──
	// 但代价是慢 ~1000×。如果需要 viz / STL 导出可以另外 tessellate()。
	//
	// 接 OCCT 之前的 OFF 构建：所有查询都抛 XFoam_Error。
	bool empty() const override { return faces_.size() == 0; }
	bool contains(const XFoam_Vector3D& p) const override;
	bool boxIntersects(const XFoam_BoundBox& box) const override;
	void closestPointAndNormal(
		const XFoam_Vector3D& p,
		XFoam_Vector3D&       outClosest,
		XFoam_Vector3D&       outNormal) const override;
	XFoam_Label nFeatureEdges() const override
	{
		return static_cast<XFoam_Label>(featureEdgeIdx_.size());
	}
	XFoam_Label nFeatureVertices() const override
	{
		return static_cast<XFoam_Label>(featureVertIdx_.size());
	}
	FeatureKind closestFeature(
		const XFoam_Vector3D& p,
		XFoam_Scalar          searchRadius,
		XFoam_Vector3D&       outClosest,
		XFoam_Vector3D&       outTangent) const override;
	/// MBrep 的 buildFeatures 按 dihedral 阈值过一遍所有 TopoDS_Edge，凡相邻
	/// 两 face 法向夹角 > 阈值，或只被 1 个 face 引用（free edge），都登记为
	/// feature edge；feature vertex 则是入射 3 条以上 feature edge 的 TopoDS_Vertex。
	void buildFeatures(XFoam_Scalar featureAngleDeg) override;

	// ---- sub-patch（一面一 patch）API ----
	// MBrep 的 sub-patch = TopoDS_Face id（一 face 一 patch），name 走 faceNames_
	// （未命名 → "face_<id>"）。closestSubPatchId 走 query proxy 的 closestSubPatchId
	// —— proxy 是用 toVBrep() 做的，每张 ParametricFace 的 tri 都带 patchId =
	// 该 face id，所以代理 BVH 找最近 tri 自然给出 face id。
	XFoam_Label nSubPatches() const override { return faces_.size(); }
	XFoam_String subPatchName(XFoam_Label id) const override;
	XFoam_Label closestSubPatchId(const XFoam_Vector3D& p) const override;
	XFoam_BoundBox subPatchBounds(XFoam_Label id) const override;

	/// 最短 feature TopoDS_Edge 长度（sampled 多段线累计弦长）。buildFeatures
	/// 未调用过时返回 0。
	XFoam_Scalar minFeatureLength() const override;

	// pImpl：仅在 XFOAM_WITH_OCCT=ON 时有内容。前向声明 + AutoPtr 让头文件
	// 不必拉入 OCCT。OFF 时仍可以构造（指向空的 OcctData）。
	// 暴露为 public 仅为同 .cpp 内的 free helper（rebuildFromShape）访问；外
	// 部代码看不到其完整定义。
	struct OcctData;

private:
	XFoam_List<ParametricVertex> verts_;
	XFoam_List<ParametricEdge>   edges_;
	XFoam_List<ParametricFace>   faces_;
	XFoam_List<ParametricBody>   bodies_;
	XFoam_WordList               faceNames_;
	XFoam_WordList               faceTypes_;
	XFoam_AutoPtr<OcctData>      occt_;

	// feature 缓存：buildFeatures 后填，存 edges_/verts_ 的下标即可
	// （不存 gp_Pnt，避免拉 OCCT 类型到 .h；具体几何 closestFeature 再去
	// 反查 edges_[idx].sampled / verts_[idx].p）。
	mutable std::vector<XFoam_Label> featureEdgeIdx_;
	mutable std::vector<XFoam_Label> featureVertIdx_;

	// 几何查询粗筛缓存：boxIntersects / closestPointAndNormal / contains 共用，
	// 第一次几何查询时由 ensureBboxCache() 一次性建好。
	//
	// 设计上跟 VBrep 的 BVH 缓存对应 —— VBrep 缓存三角面 BVH，MBrep 缓存
	// per-face / per-solid bbox（粒度比 BVH 粗，因为 N_face 通常 ≪ N_tri）。
	//
	// 索引规约：faceBboxCache_[i] 对应 occt_().faceMap.FindKey(i+1)；
	//          solidBboxCache_[i] 对应 occt_().solidMap.FindKey(i+1)。
	mutable std::vector<XFoam_BoundBox> faceBboxCache_;
	mutable std::vector<XFoam_BoundBox> solidBboxCache_;
	mutable bool                        bboxCacheBuilt_ = false;

	void ensureBboxCache() const;
	void invalidateBboxCache() const { bboxCacheBuilt_ = false; }

	// =========================================================================
	// closestPointAndNormal 查询代理：内置 VBrep。
	//
	// 动机：直接走 BRepExtrema_DistShapeShape 对 1000+ face 的 assembly 每次
	// 查询要 ~ms（0.step 76s wall time 主要花在这）。把 MBrep tessellate 一遍
	// 烘成 VBrep，BVH-based closest-point 走离散三角面就够 µs 量级。
	//
	// 取舍：position 误差 ≈ queryDeflection_（默认 = bbox 对角 × 1e-3），normal
	// 是 tri-flat normal（vs OCCT analytic normal 的 sub-degree）。对 snap 收敛
	// 完全够用 —— snap 把点投到 closest，position 精度 = deflection ≪ cell
	// size；normal 仅供 snap 内层方向判定，flat normal 不影响最终几何。
	//
	// 真正需要 OCCT 解析精度的场景：closestFeature（仍然两阶段 sampled+OCCT
	// refine）；contains（SolidClassifier，解析）；boxIntersects（per-face bbox，
	// OCCT 算 bbox）。这三个保持 OCCT 后端不变。
	// =========================================================================
	mutable XFoam_AutoPtr<XFoam_VBrep> queryProxy_;
	mutable bool                       proxyBuilt_ = false;
	XFoam_Scalar                       queryDeflection_ = -1; ///< <0 = 自适应

	void ensureQueryProxy() const;
	void invalidateQueryProxy() const { proxyBuilt_ = false; }

public:
	/// closestPointAndNormal 用的离散弦高。<=0 → bbox 对角 × 1e-3 自适应；外部
	/// 在需要更高精度 / 更快建表时手动设。值改动会让代理失效，下次查询重建。
	void setQueryDeflection(XFoam_Scalar d)
	{
		queryDeflection_ = d;
		invalidateQueryProxy();
	}
	XFoam_Scalar queryDeflection() const { return queryDeflection_; }
};

#endif // XFoam_MBrep_H_
