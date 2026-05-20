#ifndef XFoam_CMshOctree_H_
#define XFoam_CMshOctree_H_

// =============================================================================
// 对标 cfMesh: meshLibrary/utilities/octrees/meshOctree/meshOctree.{H,C}
//   + meshOctreeCube/meshOctreeCubeCoordinates / meshOctreeCubeBasic
//
// 与 cfMesh 原版的差异（MVP 阶段刻意精简）：
//   * surface 输入：cfMesh 用 const Foam::Module::triSurf&（cfmesh 自家三角面
//     格式）；myfoam 用 const XFoam_BrepBase&（虚拓扑抽象层）。VBrep / MBrep
//     都可以直接接入，几何查询全走 BrepBase 虚函数。
//   * 没有 parallel slot / Hilbert / Morton：MVP 单进程，leaves_ 按建树顺序
//     收集；后续 phase 再加。
//   * cube payload：cfMesh 在 cube 内挂 containedTrianglesLabel /
//     containedEdges 等用于缓存与该 cube 相交的几何元素；这里直接走 brep 的
//     BVH / OCCT 重算，省内存。代价是每次 boxIntersects/contains 多一次 brep
//     virtual call；BrepBase 已经分别加了 BVH/proxy 加速，目前测试下来代价
//     可接受。
//
// 数据模型：
//   * CubeCoords：(posX,posY,posZ, level) 32+32+32+8 bit。child/parent 全靠
//     位移；与 cfMesh meshOctreeCubeCoordinates 直接对应。
//   * Cube：CubeCoords + cubeType + children[8] + leafIdx。析构时 cascade
//     释放 children。
//   * Octree：root + leaves[] + bbox + brep ref + 一组构建 / 查询 op；
//     leaves_ 在每次 refine* 之后由 rebuildLeaves() 重新收集。
// =============================================================================

#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

class XFoam_BrepBase;

/*---------------------------------------------------------------------------*\
                   Class XFoam_CMshOctreeCubeCoords Declaration
\*---------------------------------------------------------------------------*/

/// 八叉树 cube 的逻辑坐标。posX/posY/posZ 是当前 level 上整数格坐标
/// （范围 0 .. 2^level - 1），level=0 时 (0,0,0) 即 root cube。
class XFoam_API XFoam_CMshOctreeCubeCoords
{
public:
	XFoam_Label  posX  = 0;
	XFoam_Label  posY  = 0;
	XFoam_Label  posZ  = 0;
	std::uint8_t level = 0;

	XFoam_CMshOctreeCubeCoords() = default;
	XFoam_CMshOctreeCubeCoords(XFoam_Label x, XFoam_Label y, XFoam_Label z, std::uint8_t l)
		: posX(x), posY(y), posZ(z), level(l)
	{}

	/// 该 cube 在 root bbox 内对应的 (min,max)。
	void cubeBox(const XFoam_BoundBox& root,
	             XFoam_Vector3D&       outMin,
	             XFoam_Vector3D&       outMax) const;

	/// 该 cube 在 root bbox 内对应的中心点。
	XFoam_Vector3D centre(const XFoam_BoundBox& root) const;

	/// 该 cube 的边长（root.span / 2^level，三轴各异时取 X 分量）。
	XFoam_Scalar size(const XFoam_BoundBox& root) const;

	/// 第 childIdx (0..7) 个 child 的坐标。childIdx 位 0/1/2 分别表 x/y/z 半。
	XFoam_CMshOctreeCubeCoords childCoords(int childIdx) const;
};

/*---------------------------------------------------------------------------*\
                       enum XFoam_CMshCubeType
\*---------------------------------------------------------------------------*/

/// 与 cfMesh meshOctreeCubeBasic::typesOfCubes 对齐。Data = "cube 边界 box
/// 与 surface 相交"；Inside/Outside = "cube 完全在 surface 内 / 外"；Unknown
/// = "还未分类"。
enum class XFoam_CMshCubeType : std::uint8_t
{
	Unknown = 1,
	Outside = 2,
	Data    = 4,
	Inside  = 8
};

/*---------------------------------------------------------------------------*\
                     Class XFoam_CMshOctreeCube Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshOctreeCube : public XFoam_CMshOctreeCubeCoords
{
public:
	XFoam_CMshCubeType    type     = XFoam_CMshCubeType::Unknown;
	XFoam_CMshOctreeCube* children[8] = {nullptr, nullptr, nullptr, nullptr,
	                                     nullptr, nullptr, nullptr, nullptr};
	XFoam_Label           leafIdx  = -1; ///< 在 Octree.leaves_ 中下标；非 leaf 为 -1

	XFoam_CMshOctreeCube() = default;
	explicit XFoam_CMshOctreeCube(const XFoam_CMshOctreeCubeCoords& c)
		: XFoam_CMshOctreeCubeCoords(c)
	{}

	~XFoam_CMshOctreeCube();
	XFoam_CMshOctreeCube(const XFoam_CMshOctreeCube&)            = delete;
	XFoam_CMshOctreeCube& operator=(const XFoam_CMshOctreeCube&) = delete;

	bool isLeaf() const noexcept { return children[0] == nullptr; }

	/// 一次性建好 8 个子 cube；子 cube 的 type=Unknown, children 各自 null。
	/// 已经有 children 时 no-op。
	void subdivide();
};

/*---------------------------------------------------------------------------*\
                        Class XFoam_CMshOctree Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshOctree
{
public:
	/// rootBox 通常 = brep.bounds() 后略 inflate；OctreeCreator 会代为处理。
	XFoam_CMshOctree(const XFoam_BrepBase& surface, const XFoam_BoundBox& rootBox);
	~XFoam_CMshOctree();

	XFoam_CMshOctree(const XFoam_CMshOctree&)            = delete;
	XFoam_CMshOctree& operator=(const XFoam_CMshOctree&) = delete;

	const XFoam_BoundBox& rootBox() const noexcept { return rootBox_; }
	const XFoam_BrepBase& surface() const noexcept { return surface_; }

	XFoam_Label nLeaves() const noexcept
	{
		return static_cast<XFoam_Label>(leaves_.size());
	}
	const XFoam_CMshOctreeCube& leaf(XFoam_Label i) const { return *leaves_[i]; }

	/// 把所有"box 与 surface 相交"的 leaf 一直加密到 targetLevel。已经 ≥
	/// targetLevel 的不动；与 surface 不相交的也不动（cfMesh
	/// "adjust octree to surface"）。
	void refineToSurface(int targetLevel);

	/// 把所有"box 与 region overlaps"的 leaf 加密到 targetLevel。
	void refineRegion(const XFoam_BoundBox& region, int targetLevel);

	/// 全 leaf 一次性细化到 baseLevel；用作建初始格。
	void refineUniform(int baseLevel);

	/// 给 leaf 标 Inside/Outside/Data：
	///   * 与 surface 相交 → Data
	///   * 否则按 center 走 BrepBase.contains → Inside / Outside
	/// 调用前 leaf 应至少已经 refineToSurface 过一次。
	void classifyLeaves();

	/// 报每个 level 上 leaf 数量；out 索引就是 level。
	void countLeavesByLevel(std::vector<XFoam_Label>& out) const;

	/// 报每种 cube type 的 leaf 数。
	void countLeavesByType(XFoam_Label& nUnknown,
	                       XFoam_Label& nOutside,
	                       XFoam_Label& nData,
	                       XFoam_Label& nInside) const;

	/// 顺序遍历 leaves（read-only）。
	void forEachLeaf(const std::function<void(const XFoam_CMshOctreeCube&)>& fn) const;

	/// 用 objectRefinement 谓词加密：把所有 obj.boxIntersects(leaf.box) 的 leaf
	/// 迭代加密到 targetLevel。对应 cfMesh 的 refineByObject。targetLevel < 0
	/// 时使用 obj.level。
	void refineByObject(const class XFoam_CMshObjRefine& obj, int targetLevel = -1);

	/// 2:1 balance：保证任意两个相邻 leaf 的 level 差 ≤ 1。CartesianExtractor
	/// 要求 1-irregularity 才能正确做面切割。算法：每个 leaf 看 26 邻位（face / edge
	/// / vertex），若有 level ≥ self.level+2 的，把 self 升一级；反复迭代直到稳定。
	void balance21();

	/// 按 (level, x, y, z) 查 leaf；返回 nullptr = 该 cube 不是 leaf（被细分）
	/// 或不存在。在 rebuildLeavesAndIndex() 之后才能用。
	const XFoam_CMshOctreeCube* findLeafByCoords(
		std::uint8_t level, XFoam_Label x, XFoam_Label y, XFoam_Label z) const;

	/// 按 3D 坐标查包含该点的 leaf。pos 落在 root 外 → nullptr；
	/// 否则从 root 逐级 descend 直到 leaf 命中。O(maxLevel)。
	const XFoam_CMshOctreeCube* findLeafContaining(const XFoam_Vector3D& pos) const;

	/// 沿 face 方向 d (0..5: -x +x -y +y -z +z) 查 leaf 的 face 邻居。
	///   * Same：邻居与 self 同 level（恰 1 个）
	///   * Coarser：邻居比 self 粗 1 级（恰 1 个）
	///   * Finer：邻居比 self 细 1 级（最多 4 个；返回 std::vector）
	///   * None：face 在 root 边界外
	/// 要求已 balance21()；mixed-level (差 ≥ 2) 时退化为 None。
	enum class FaceNbrKind : std::uint8_t { None, Same, Coarser, Finer };
	struct FaceNbrResult
	{
		FaceNbrKind kind = FaceNbrKind::None;
		const XFoam_CMshOctreeCube* same = nullptr;        ///< Same / Coarser
		const XFoam_CMshOctreeCube* finer[4] = {nullptr, nullptr, nullptr, nullptr}; ///< Finer
	};
	FaceNbrResult faceNeighbour(const XFoam_CMshOctreeCube& self, int d) const;

private:
	const XFoam_BrepBase&                surface_;
	XFoam_BoundBox                       rootBox_;
	XFoam_CMshOctreeCube*                root_ = nullptr;
	std::vector<XFoam_CMshOctreeCube*>   leaves_;

	// (level, x, y, z) → leaf*。 rebuildLeaves() 后填好；balance21 / extractor 走它做 O(1) 邻居查询。
	// key = packBits(level | x<<8 | y<<32 | z<<48)，因 level ≤ 14 → x/y/z 范围 [0, 16384)。
	std::unordered_map<std::uint64_t, const XFoam_CMshOctreeCube*> leafByCoords_;

	void collectLeavesRecursive(XFoam_CMshOctreeCube* c);
	void rebuildLeaves();
	void rebuildIndex();

	static std::uint64_t packCoord(
		std::uint8_t level, XFoam_Label x, XFoam_Label y, XFoam_Label z);
};

#endif // XFoam_CMshOctree_H_
