#ifndef XFoam_TriSurface_H_
#define XFoam_TriSurface_H_

// 三角面片表面：STL 读取 + 最近点 / 距离 / 内外判定 / 包围盒相交。
// 用于 snappyHexMesh 阶段对几何做切割与边界 snap。
//
// 对标 OpenFOAM 的 Foam::triSurface（src/triSurface），但极度简化：
//   - 仅 ascii 与 binary STL；不读 FreeSurfer/nas/ftr 等
//   - 自带 中位数分割 BVH（AABB tree，叶 4 个三角面），所有空间查询都走 BVH：
//       boxIntersects   ~ O(log N + k)
//       contains/rayCountPlusX ~ O(log N + k)
//       distance/closestPoint  ~ O(log N + 极少 leaf)
//     （k = 与查询 bbox 真正相交的三角面数）
//   - 不区分 region；整个表面就一个 patch
//   - 法向量直接取自 STL header 中的 facet normal；不重算
// 适合教学/演示规模（几千～几十万三角面，BVH 后查询时间近似常数）。

#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <string>
#include <vector>

class XFoam_API XFoam_TriSurface
{
public:
	struct Triangle
	{
		XFoam_Vector3D v0, v1, v2;
		XFoam_Vector3D normal; // 单位化；构造时计算
		XFoam_BoundBox bbox;   // 缓存避免重复 min/max
	};

	XFoam_TriSurface() = default;

	/// 自动按 ascii / binary 嗅探。返回 true 即至少读到 1 个三角面。
	bool read(const std::string& path);

	bool readAsciiSTL(const std::string& path);
	bool readBinarySTL(const std::string& path);

	XFoam_Label size() const { return static_cast<XFoam_Label>(tris_.size()); }
	const std::vector<Triangle>& tris() const { return tris_; }
	const XFoam_BoundBox& bounds() const { return bounds_; }
	bool empty() const { return tris_.empty(); }

	/// 距离查询：返回 p 到所有三角面最近距离的平方根；line scan，O(N)。
	XFoam_Scalar distance(const XFoam_Vector3D& p) const;

	/// 最近点 + 法向。法向取自承载三角面的 facet 法向（带 sign，朝外）。
	void closestPointAndNormal(
		const XFoam_Vector3D& p,
		XFoam_Vector3D& outClosest,
		XFoam_Vector3D& outNormal) const;

	/// 沿 +X 方向从 p 射线交叉次数。奇/偶判定 inside / outside（仅对水密 STL 有效）。
	int rayCountPlusX(const XFoam_Vector3D& p) const;

	/// 包围盒粗筛：盒内/相交 → true。用于细化阶段的"和表面相交的 cell"快筛。
	bool boxIntersects(const XFoam_BoundBox& box) const;

	/// 是否包含点：基于 +X 射线奇数次穿过判定（要求水密 STL）。
	/// 在 p 的 y/z 上加一个相对包围盒的小抖动，避免射线穿过顶点 / 棱时（如球心
	/// (0,0,0) 对 +X 等于 R 处的等分顶点）出现奇偶判定不稳。
	bool contains(const XFoam_Vector3D& p) const;

	// ------- Snap #7 feature edges / corners -------
	// 任何被 2 个三角面共享、且两面法向夹角超过 angleDegThresh（默认 OF 30 deg）的边
	// 都视为 feature；只被 1 个三角面引用的开边 / 多面共享的非流形边也算 feature。
	// featureVertex = 至少 3 条 feature edge 入射的顶点（典型尖角点）。
	//
	// 必须在 read*() 之后显式调用一次；否则查询会返回 None。重复调用会重建。
	void buildFeatures(XFoam_Scalar angleDegThresh);
	XFoam_Label nFeatureEdges() const { return static_cast<XFoam_Label>(featureEdges_.size()); }
	XFoam_Label nFeatureVertices() const { return static_cast<XFoam_Label>(featureVerts_.size()); }

	enum class FeatureKind
	{
		None,
		Edge,
		Vertex
	};

	/// 在 searchRadius 半径内查最近 feature。若 feature vertex 在半径内则优先（snap 到
	/// 尖角效果更好）。找到时填 outClosest 并返回 Edge / Vertex；否则 outClosest 保持不动，
	/// 返回 None。tolerance 比较走平方距离避免开方。
	FeatureKind closestFeature(
		const XFoam_Vector3D& p,
		XFoam_Scalar searchRadius,
		XFoam_Vector3D& outClosest) const;

private:
	std::vector<Triangle> tris_;
	XFoam_BoundBox bounds_;

	// ----- BVH（AABB tree） -----
	// 每个 node 存子 bbox + 子节点索引（internal）或三角面区间（leaf）。整棵树
	// 用扁平 std::vector 存，索引而非指针，避免重新 alloc 时被踩坏。triOrder_
	// 是三角面的下标置换：每个 leaf 的 [firstTri, firstTri + triCount) 范围内的
	// triOrder_[i] 就是该 leaf 包含的三角面。
	//
	// 不暴露在公有 API；构造在 read*() 末尾自动 build，外部只看到加速过的查询。
	struct BvhNode
	{
		XFoam_BoundBox bbox;
		int leftIdx;    ///< internal: 左子下标 (>=0)；leaf: -1
		int rightIdx;   ///< internal: 右子下标 (>=0)；leaf: -1
		int firstTri;   ///< leaf: triOrder_ 起点；internal: -1
		int triCount;   ///< leaf: 包含三角面数 (>0)；internal: 0

		BvhNode() : leftIdx(-1), rightIdx(-1), firstTri(-1), triCount(0) {}
	};

	std::vector<BvhNode> bvhNodes_;
	std::vector<XFoam_Label> bvhOrder_;
	static constexpr int kBvhLeafLimit = 4; ///< leaf 最多承载的三角面数（小常数 → 查询常数更小）

	// ------- Snap #7 feature storage -------
	// featureEdges_[i] = (p1, p2) 端点。featureVerts_[i] = p。线性数组够用：典型 STL
	// feature 数远小于三角面数（cylinder1 1620 tri → ~200 feature edge）。
	struct FeatureEdge
	{
		XFoam_Vector3D p1, p2;
	};
	struct FeatureVertex
	{
		XFoam_Vector3D p;
	};
	std::vector<FeatureEdge> featureEdges_;
	std::vector<FeatureVertex> featureVerts_;

	void rebuildBounds();
	void addTriangle(
		const XFoam_Vector3D& a,
		const XFoam_Vector3D& b,
		const XFoam_Vector3D& c,
		const XFoam_Vector3D* explicitNormal);

	void buildBvh();
	/// 自顶向下递归建树；返回当前节点在 bvhNodes_ 里的下标。
	/// 调用前必须保证 bvhNodes_ 已 reserve 到 ≤ 2N，否则递归过程的 push_back 触发
	/// realloc 不会让索引失效（我们用 int 索引而非引用），但仍会反复 copy node。
	int buildBvhRecursive(int lo, int hi);

	/// p 到 AABB 的最短距离平方；p 在盒内时为 0。
	static XFoam_Scalar bboxMinDistSqr(const XFoam_Vector3D& p, const XFoam_BoundBox& bb);

	// 下面三个是带 BVH 剪枝的查询；tris_ 为空 / BVH 未建时回退线性扫描（addTriangle 后
	// 没立刻 rebuild）。
	void bvhClosestPoint(
		const XFoam_Vector3D& p,
		XFoam_Scalar& bestD2,
		XFoam_Vector3D& bestQ,
		XFoam_Label& bestTri,
		int nodeIdx) const;
	int  bvhRayCountPlusX(const XFoam_Vector3D& p, int nodeIdx) const;
	bool bvhBoxIntersects(const XFoam_BoundBox& q, int nodeIdx) const;
};

#endif
