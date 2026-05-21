#ifndef XFoam_Brep_H_
#define XFoam_Brep_H_

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_vector.h"

// =============================================================================
// 虚拓扑（Virtual Topology）层。本文件定义：
//
//   * 底层 B-rep 抽象基类 XFoam_BrepBase（兄弟实现：XFoam_VBrep 离散三角化 /
//     XFoam_MBrep OCC 参数化），及其类型枚举 XFoam_BrepKind 与引用句柄
//     XFoam_BrepRef。
//   * 虚拓扑实体 XFoam_TopoEntity / Vert / Edge / Face / Body —— mesher 直接
//     看到的"逻辑拓扑"。每个实体在所属 XFoam_TopoModel 中拥有稳定 id，可被
//     标记为 suppressed（mesher 视而不见），并通过若干 XFoam_BrepRef 引用
//     底层 BrepBase 实体（merge 就是把多个底层引用归并到同一虚实体）。
//   * 顶层容器 XFoam_TopoModel —— 持有一个底层 brep + 四张虚拓扑实体表 +
//     编辑/查询/导出接口。snappyHexMesh 通过 exportToSnappy() 取到按虚 patch
//     分组的离散三角面集合即可工作。
//
// 设计要点：
//   * 兄弟式抽象：XFoam_BrepBase 是抽象基类，VBrep / MBrep 都派生自它。
//     TopoModel.brep_ 用 XFoam_AutoPtr<XFoam_BrepBase> 多态持有。BrepRef
//     用 (kind, idx) 而非裸指针，避免底层容器扩容把虚拓扑打废。
//   * id 稳定：merge 时 src 实体保留 id 但 suppressed=true + clearRefs，
//     上游持有的 id 仍可解析（解析到 suppressed=true 即知已被吸收）。
//   * 与 OCCT 解耦：本头文件不引入任何 OCCT 类型；OCCT 仅在 MBrep 的 cpp 内
//     通过 pImpl 引入，由 CMake -DXFOAM_WITH_OCCT 控制是否参与编译。
// =============================================================================

class XFoam_TopoModel;
class XFoam_VBrep;
class XFoam_MBrep;
class XFoam_BrepBase;

// 底层几何类型枚举。BrepRef 携带，便于在不暴露 dynamic_cast 的前提下安全地
// 解到具体派生类型；None 表示"未绑定"。
enum class XFoam_BrepKind : int
{
	None         = 0,
	Triangulated = 1, ///< → XFoam_VBrep
	Parametric   = 2  ///< → XFoam_MBrep
};

// 对底层 BrepBase 实体（vertex / edge / face / body）的不可变句柄。
// (kind, idx) 二元组：kind 告诉解析方"这个 idx 落在哪种 brep 的哪个数组",
// idx 是该数组里的下标。不用裸指针 → 底层 std::vector 扩容不会失效。
struct XFoam_API XFoam_BrepRef
{
	XFoam_BrepKind kind  = XFoam_BrepKind::None;
	XFoam_Label    idx   = -1; ///< 在底层 vertex / edge / face / body 数组里的下标
	XFoam_Label    sub   = -1; ///< 可选二级 idx（如 face 内 sub-region）；通常 -1

	XFoam_BrepRef() = default;
	XFoam_BrepRef(XFoam_BrepKind k, XFoam_Label i, XFoam_Label s = -1)
		: kind(k), idx(i), sub(s)
	{}

	bool valid() const { return kind != XFoam_BrepKind::None && idx >= 0; }
};

// 共同抽象基。VBrep / MBrep 都派生自它；TopoModel 多态持有。
// 派生类必须实现：kind / clear / bounds / refBounds（用于上层不必类型分支）。
class XFoam_API XFoam_BrepBase
{
public:
	XFoam_BrepBase() = default;
	virtual ~XFoam_BrepBase();

	XFoam_BrepBase(const XFoam_BrepBase&) = default;
	XFoam_BrepBase& operator=(const XFoam_BrepBase&) = default;
	XFoam_BrepBase(XFoam_BrepBase&&) noexcept = default;
	XFoam_BrepBase& operator=(XFoam_BrepBase&&) noexcept = default;

	virtual XFoam_BrepKind kind() const = 0;
	virtual void clear() = 0;
	virtual XFoam_BoundBox bounds() const = 0;

	/// 用 BrepRef 反查 bbox。无效 / 越界时返回空 bbox（min > max）。让 TopoEntity
	/// 计算自身 bbox 时不必类型分支。
	virtual XFoam_BoundBox refBounds(const XFoam_BrepRef& r) const = 0;

	// =========================================================================
	// 几何查询 API ── snappyHexMesh / 任何 mesher 都通过这组虚函数访问几何，
	// 不需要知道底层是离散 BVH 还是 OCCT 参数化。
	//
	// 实现差异 / 精度 / 性能对照：
	//   VBrep（离散）：BVH 加速，~µs 量级，误差 = O(deflection)。
	//   MBrep（参数化，OCCT）：BRepExtrema_DistShapeShape，~ms 量级，误差 = 数值
	//   精度（10^-14 量级），且 feature edge 永远精确（无 tessellation 锯齿）。
	//
	// 选型经验：snappy 的 refine 粗筛大量调 boxIntersects，应该 VBrep；snappy
	// 的 snap 收敛阶段只调 closestPoint/closestFeature，可以 MBrep 直接喂获得
	// 数值精度。
	// =========================================================================

	enum class FeatureKind
	{
		None,
		Edge,
		Vertex
	};

	/// 总三角面 / 参数面数为 0 时为 true。snappy 用它来跳过空 surface 槽位。
	virtual bool empty() const = 0;

	/// 点是否在水密 brep 内部。开壳（VBrep boundary edge 存在 / MBrep 非 SOLID）
	/// 时实现应返回 false。
	virtual bool contains(const XFoam_Vector3D& p) const = 0;

	/// bbox 与底层几何有任何相交（含相切 / 包含）→ true。粗筛用。
	virtual bool boxIntersects(const XFoam_BoundBox& box) const = 0;

	/// p 到几何上的最近点 + 该点法向（朝外 = +）。空表面时返回 outClosest = p,
	/// outNormal = 0。
	virtual void closestPointAndNormal(
		const XFoam_Vector3D& p,
		XFoam_Vector3D&       outClosest,
		XFoam_Vector3D&       outNormal) const = 0;

	/// feature edge / vertex 数。buildFeatures() 调用之前对 VBrep 返回 0；对
	/// MBrep 永远返回真实数（每条 TopoDS_Edge / TopoDS_Vertex 自然就是 feature
	/// 候选）。
	virtual XFoam_Label nFeatureEdges() const = 0;
	virtual XFoam_Label nFeatureVertices() const = 0;

	/// p 半径 searchRadius 以内最近 feature。feature vertex 在半径内时优先返回
	/// （snap 到尖角效果更好）。outTangent 仅在 Edge 时填（unit），否则 (0,0,0)。
	virtual FeatureKind closestFeature(
		const XFoam_Vector3D& p,
		XFoam_Scalar          searchRadius,
		XFoam_Vector3D&       outClosest,
		XFoam_Vector3D&       outTangent) const = 0;

	/// 二面角 > featureAngleDeg 的 edge 升级为 feature；与会 3 条以上 feature
	/// edge 的 vertex 升级为 feature vertex。VBrep 时必填；MBrep 时通常 no-op
	/// （feature 候选直接来自 TopoDS_Edge，仅按角度筛选）。
	virtual void buildFeatures(XFoam_Scalar featureAngleDeg) = 0;

	/// 便利包装：调 closestPointAndNormal 算 |p - closest|，单测里查"球心到球面
	/// 距离 ≈ R"等场景常用。空 brep 返回 0。
	XFoam_Scalar distance(const XFoam_Vector3D& p) const
	{
		if (empty()) return 0;
		XFoam_Vector3D q, n;
		closestPointAndNormal(p, q, n);
		return (p - q).mag();
	}

	// =========================================================================
	// 子-patch（一面一 patch）支持。snappy 可在 perFacePatches=true 时把
	// boundary face 按归属的 sub-patch 分桶 emit 多个 polyMesh patch。
	//
	// 语义对齐：
	//   * VBrep：sub-patch = DiscreteFace::patchId（STL solid / BDF HMMOVE
	//     component），nSubPatches() = patchNames_.size()。
	//   * MBrep：sub-patch = TopoDS_Face id（1 patch per parametric face），
	//     nSubPatches() = nFaces()。
	//
	// closestSubPatchId(p) 返回 0..nSubPatches()-1 之间的 id，或 -1 表示"未
	// 知 / 该 brep 没有 sub-patch 概念"。snappy 见 -1 时回退到原 (surf-id) 单
	// patch 行为。空 brep 返回 -1。
	// =========================================================================
	virtual XFoam_Label nSubPatches() const { return 0; }
	virtual XFoam_String subPatchName(XFoam_Label /*id*/) const { return XFoam_String(); }
	virtual XFoam_Label closestSubPatchId(const XFoam_Vector3D& /*p*/) const { return -1; }

	/// 单个 sub-patch 的 bbox。MBrep 走 BRepBndLib 取 TopoDS_Face bbox；VBrep
	/// 走该 patchId 下所有三角面 union。无效 id / 不支持时返回 invalid bbox。
	/// 用途：creator 的 per-face fitFeatures 按每张 TpFace 自己的尺度反推
	/// 所需 surfLevel，而非全局一刀切。
	virtual XFoam_BoundBox subPatchBounds(XFoam_Label /*id*/) const
	{
		return XFoam_BoundBox(); // invalid
	}

	/// 单个 sub-patch 的最短 feature 边长（比 minFeatureLength 更精细，按
	/// 每张 TpFace 自己的边算）。返回 0 表示 "未知 / 不支持 / 没有 feature"。
	///   * MBrep：扫该 face 的 outerLoop + innerLoops，取最短 ParametricEdge
	///     的 sampled 多段线总长。
	///   * VBrep：扫该 patchId 下所有三角面，取最短三角边（粗估，不区分
	///     feature/internal）。
	/// 用途：creator 的 per-face fitFeatures 取 min(bbox_min_side, this)
	/// 作为 cellSize 目标，既保 bbox 完整又保 feature 不被网格抹平。
	virtual XFoam_Scalar subPatchMinFeatureLength(XFoam_Label /*id*/) const
	{
		return 0;
	}

	/// 最小 feature 尺度（mm 等线性单位）。返回的语义：
	///   * VBrep：featureEdges_ 中最短的一段（一条 DiscreteEdge）长度。
	///   * MBrep：featureEdgeIdx_ 对应 ParametricEdge 的 sampled 多段线总长
	///            最短者（≈ 最短 TopoDS_Edge 的弦长）。
	/// 用于 snappy 的 fitFeatures 自动调参：cell size 应 ≤ k × minFeatureLength
	/// 才能让相邻 feature 不在网格上合并。无 feature 或 buildFeatures 未调用
	/// 时返回 0。
	virtual XFoam_Scalar minFeatureLength() const { return 0; }
};

// 拓扑实体基类。持有：
//   * 所属 TopoModel 指针（不参与所有权）
//   * 在 TopoModel 内的稳定 id（数组下标；merge/split 后不变）
//   * suppressed 标志（mesher 跳过；用于短边抑制 / merge 后吸收的 src）
//   * refs_：对底层 BrepBase 实体的引用集合（merge → refs 合并即可）
class XFoam_API XFoam_TopoEntity
{
protected:
	const XFoam_TopoModel*    model_      = nullptr;
	XFoam_Label               id_         = -1;
	bool                      suppressed_ = false;
	XFoam_List<XFoam_BrepRef> refs_;

public:
	XFoam_TopoEntity() = default;
	explicit XFoam_TopoEntity(const XFoam_TopoModel* m, XFoam_Label id = -1)
		: model_(m), id_(id)
	{}
	virtual ~XFoam_TopoEntity();

	const XFoam_TopoModel* model() const { return model_; }
	XFoam_Label id() const { return id_; }
	void setId(XFoam_Label i) { id_ = i; }

	bool suppressed() const { return suppressed_; }
	void setSuppressed(bool s) { suppressed_ = s; }

	XFoam_Label nRefs() const { return refs_.size(); }
	const XFoam_BrepRef& ref(XFoam_Label i) const { return refs_[i]; }
	const XFoam_List<XFoam_BrepRef>& refs() const { return refs_; }
	void addRef(const XFoam_BrepRef& r) { refs_.append(r); }
	void clearRefs() { refs_.clear(); }
	void absorbRefs(const XFoam_TopoEntity& other); ///< merge 时把 other.refs_ 追加进来

	/// 子类必须给出几何 bbox（通常 = union of refBounds over refs_）。
	virtual XFoam_BoundBox bounds() const = 0;
};

// 0-D 虚拓扑实体：尖角 / 端点 / 用户手加的 control point。
class XFoam_API XFoam_TopoVert : public XFoam_TopoEntity
{
public:
	XFoam_TopoVert() = default;
	explicit XFoam_TopoVert(const XFoam_TopoModel* m, XFoam_Label id = -1)
		: XFoam_TopoEntity(m, id)
	{}

	/// 返回该虚 vert 的代表位置（refs_ 内所有底层 vertex 位置的平均值；
	/// refs_ 为空时返回原点）。
	XFoam_Vector3D position() const;
	XFoam_BoundBox bounds() const override;
};

// 1-D 虚拓扑实体：feature line / 共线 polyline / 用户手加虚棱。
class XFoam_API XFoam_TopoEdge : public XFoam_TopoEntity
{
	XFoam_Label v0_ = -1; ///< 起点 TopoVert id（-1 = 未绑定）
	XFoam_Label v1_ = -1; ///< 终点 TopoVert id

public:
	XFoam_TopoEdge() = default;
	explicit XFoam_TopoEdge(const XFoam_TopoModel* m, XFoam_Label id = -1)
		: XFoam_TopoEntity(m, id)
	{}

	XFoam_Label v0() const { return v0_; }
	XFoam_Label v1() const { return v1_; }
	void setEndpoints(XFoam_Label a, XFoam_Label b) { v0_ = a; v1_ = b; }

	/// 沿 refs_ 所引用的所有底层 edge 累加段长。
	XFoam_Scalar length() const;
	XFoam_BoundBox bounds() const override;
};

// 2-D 虚拓扑实体：→ polyMesh.boundary 的一个 patch。可包含多张底层 face
// （merge）或仅一张底层 face 的子集（split，sub idx 落在 BrepRef.sub）。
class XFoam_API XFoam_TopoFace : public XFoam_TopoEntity
{
	XFoam_LabelList edgeIds_; ///< 该 face 的虚边界 TopoEdge id 序列
	XFoam_Word      name_;    ///< 落盘成 polyMesh patch 名（snappy 直接用）
	XFoam_Word      patchType_ = XFoam_Word("wall"); ///< wall / patch / symmetry / ...

public:
	XFoam_TopoFace() = default;
	explicit XFoam_TopoFace(const XFoam_TopoModel* m, XFoam_Label id = -1)
		: XFoam_TopoEntity(m, id)
	{}

	const XFoam_Word& name() const { return name_; }
	void setName(const XFoam_Word& n) { name_ = n; }
	const XFoam_Word& patchType() const { return patchType_; }
	void setPatchType(const XFoam_Word& t) { patchType_ = t; }
	const XFoam_LabelList& edgeIds() const { return edgeIds_; }
	void addEdge(XFoam_Label e) { edgeIds_.append(e); }
	void setEdgeIds(const XFoam_LabelList& es) { edgeIds_ = es; }

	/// 通过 refs_ 累加所有底层 triangle 面积（VBrep）或 sampled 三角面积
	/// （MBrep tessellate 后）。
	XFoam_Scalar area() const;
	XFoam_BoundBox bounds() const override;
};

// 3-D 虚拓扑实体：solid body / region；用 cellZone / locationInMesh 落盘。
class XFoam_API XFoam_TopoBody : public XFoam_TopoEntity
{
	XFoam_LabelList faceIds_;
	XFoam_Vector3D  locationInMesh_ = XFoam_Vector3D(0, 0, 0);
	bool            hasLocation_    = false;

public:
	XFoam_TopoBody() = default;
	explicit XFoam_TopoBody(const XFoam_TopoModel* m, XFoam_Label id = -1)
		: XFoam_TopoEntity(m, id)
	{}

	const XFoam_LabelList& faceIds() const { return faceIds_; }
	void addFace(XFoam_Label f) { faceIds_.append(f); }
	void setFaceIds(const XFoam_LabelList& fs) { faceIds_ = fs; }

	bool hasLocationInMesh() const { return hasLocation_; }
	const XFoam_Vector3D& locationInMesh() const { return locationInMesh_; }
	void setLocationInMesh(const XFoam_Vector3D& p)
	{
		locationInMesh_ = p;
		hasLocation_    = true;
	}

	XFoam_BoundBox bounds() const override;
};

// 拓扑模型 = 一个底层 BrepBase + 四张虚拓扑实体表 + 编辑/查询/导出。
//
// 生命周期：构造时默认 brep_ = new XFoam_VBrep()（空），适配 "default model →
// load → use" 的常用流程。readFromStep() / setBrep() 会替换底层。
class XFoam_API XFoam_TopoModel
{
private:
	XFoam_AutoPtr<XFoam_BrepBase> brep_;
	XFoam_List<XFoam_TopoVert>    verts_;
	XFoam_List<XFoam_TopoEdge>    edges_;
	XFoam_List<XFoam_TopoFace>    faces_;
	XFoam_List<XFoam_TopoBody>    bodies_;

public:
	XFoam_TopoModel();
	~XFoam_TopoModel();

	XFoam_TopoModel(const XFoam_TopoModel&)            = delete;
	XFoam_TopoModel& operator=(const XFoam_TopoModel&) = delete;
	XFoam_TopoModel(XFoam_TopoModel&&) noexcept;
	XFoam_TopoModel& operator=(XFoam_TopoModel&&) noexcept;

	// ----- 底层 brep 访问 -----
	XFoam_BrepKind brepKind() const;
	const XFoam_BrepBase& brep() const { return brep_(); }
	XFoam_BrepBase&       brep()       { return brep_(); }

	/// 仅 brepKind() == Triangulated 时安全；否则抛 XFoam_Error。
	const XFoam_VBrep& vbrep() const;
	XFoam_VBrep&       vbrep();

	/// 仅 brepKind() == Parametric 时安全；否则抛 XFoam_Error。
	const XFoam_MBrep& mbrep() const;
	XFoam_MBrep&       mbrep();

	/// 兼容旧 API（之前 brep_ 总指向"虚拓扑容器"自己），等同于 brep()。
	const XFoam_BrepBase& virtualBrep() const { return brep_(); }
	XFoam_BrepBase&       virtualBrep()       { return brep_(); }

	/// 替换底层 brep（接管所有权）。会同时清空虚拓扑实体表（id 失去意义）。
	void setBrep(XFoam_AutoPtr<XFoam_BrepBase> b);

	// ----- 读入：根据扩展名分派给 VBrep / MBrep -----
	/// .stl → VBrep
	void readFromStl(const XFoam_String& fileName);
	/// .bdf → VBrep（原 MBrep::readFromBdf 已迁过来）
	void readFromBdf(const XFoam_String& fileName);
	/// .step / .stp → MBrep；OCCT 未启用时抛 XFoam_Error。
	void readFromStep(const XFoam_String& fileName);
	/// .iges / .igs → MBrep；OCCT 未启用时抛 XFoam_Error。
	void readFromIges(const XFoam_String& fileName);

	// ----- 虚拓扑实体只读访问 -----
	XFoam_Label nVerts()  const { return verts_.size(); }
	XFoam_Label nEdges()  const { return edges_.size(); }
	XFoam_Label nFaces()  const { return faces_.size(); }
	XFoam_Label nBodies() const { return bodies_.size(); }
	const XFoam_TopoVert& vert(XFoam_Label i) const { return verts_[i]; }
	const XFoam_TopoEdge& edge(XFoam_Label i) const { return edges_[i]; }
	const XFoam_TopoFace& face(XFoam_Label i) const { return faces_[i]; }
	const XFoam_TopoBody& body(XFoam_Label i) const { return bodies_[i]; }
	XFoam_TopoVert& vertRef(XFoam_Label i) { return verts_[i]; }
	XFoam_TopoEdge& edgeRef(XFoam_Label i) { return edges_[i]; }
	XFoam_TopoFace& faceRef(XFoam_Label i) { return faces_[i]; }
	XFoam_TopoBody& bodyRef(XFoam_Label i) { return bodies_[i]; }

	// ----- 虚拓扑编辑（"identity" 阶段已实现；编辑操作首期为 stub） -----
	/// 新增空 vert，返回 id；可选填一个 BrepRef。
	XFoam_Label addVert(const XFoam_BrepRef& r = XFoam_BrepRef());
	XFoam_Label addEdge(XFoam_Label v0, XFoam_Label v1, const XFoam_BrepRef& r = XFoam_BrepRef());
	XFoam_Label addFace(const XFoam_Word& name, const XFoam_Word& patchType = XFoam_Word("wall"));
	XFoam_Label addBody(const XFoam_Word& name = XFoam_Word());

	/// merge：把 src[] 的 refs 追加到 dst.refs，src 各项 suppressed=true + clearRefs；
	/// id 不删 → 上游持有的 id 仍可解析（看到 suppressed 即知已被吸收）。
	/// 当前实现：函数已实现，TopoFace / TopoEdge 都走同一套 absorb 逻辑。
	void mergeFaces(const XFoam_LabelList& src, XFoam_Label dst);
	void mergeEdges(const XFoam_LabelList& src, XFoam_Label dst);

	/// split：沿已存在的虚 edge 把 face 一分为二，返回 {leftId, rightId}。
	/// **首期 stub**：抛 XFoam_Error("not implemented")，TODO 后续接 ear-clipping。
	XFoam_LabelList splitFaceByEdge(XFoam_Label faceId, XFoam_Label cutEdgeId);

	/// 抑制：把短 edge / 小 face 标 suppressed=true。返回被标的数量。
	/// **首期 stub**：扫描 + 标志，但 mesher 端尚未消费 suppressed 标志。
	XFoam_Label suppressEdgesShorterThan(XFoam_Scalar minLen);
	XFoam_Label suppressFacesSmallerThan(XFoam_Scalar minArea);

	/// 从底层 brep 自动构建一份"实拓扑等同"的虚拓扑：每张底层 face 一个
	/// TopoFace，每条 feature edge 一条 TopoEdge，每个 feature vertex 一个
	/// TopoVert。merge / split 还没发生时应在 read 完之后调用一次。
	///
	/// 参数 featureAngleDeg：传给底层 VBrep::buildEdgesFromFaces 的二面角阈值
	/// （MBrep 时忽略，因为 feature edge 是参数化的明示边）。
	void rebuildIdentityFromBrep(XFoam_Scalar featureAngleDeg);

	XFoam_BoundBox bounds() const;

	// ----- 给 snappyHexMesh 的导出适配器 -----
	struct ExportedSurface
	{
		XFoam_Word                    name;       ///< = TopoFace.name()
		XFoam_Word                    patchType;  ///< = TopoFace.patchType()
		std::vector<XFoam_Label>      triIdx;     ///< 底层 VBrep faces() 的下标集合
	};
	/// 把所有非 suppressed TopoFace 导出成 (name, tri-idx-list)。
	/// brepKind() 必须是 Triangulated；Parametric 时需要先 convertMBrepToVBrep()
	/// 把模型烘成离散，否则本函数只跑 tessellate 然后返回空集合（提示调用方下
	/// 一步该 convertMBrepToVBrep）。triIdx 的所有下标都对应 vbrep().faces()。
	std::vector<ExportedSurface> exportToSnappy(XFoam_Scalar deflection) const;

	/// Parametric → Triangulated 一步切换：
	///   1) mbrep().tessellate(deflection) 把 OCCT BRepMesh 跑一遍
	///   2) mbrep().toVBrep() 得到 VBrep（每张 ParametricFace 一个 patchId）
	///   3) setBrep() 替换底层，原 MBrep 释放
	///   4) rebuildIdentityFromBrep(featureAngleDeg) 重新建虚拓扑实体表
	/// 调用前必须 brepKind() == Parametric；否则抛 XFoam_Error。
	void convertMBrepToVBrep(XFoam_Scalar deflection, XFoam_Scalar featureAngleDeg);
};

#endif // XFoam_Brep_H_
