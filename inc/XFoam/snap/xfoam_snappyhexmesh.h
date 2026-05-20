#ifndef XFoam_SnappyHexMesh_H_
#define XFoam_SnappyHexMesh_H_

// snappyHexMesh 的极简单 region 实现：
//   1) 把背景 blockMesh 单 hex 块按 (2^L) 在三个轴上一并加密
//   2) 用 STL 切除 locationInMesh 不可达的一侧
//   3) 把新暴露的边界点 snap 到 STL 最近点
// 然后把结果写成标准 OpenFOAM polyMesh 目录。
//
// 对标 OpenFOAM-13 src/mesh/snappyHexMesh/snappyHexMeshDriver。
// 未移植：
//   - 自适应 octree（不同 cell 不同 level）；当前是全局统一 level
//   - 多 region / cellZones / faceZones
//   - addLayers（layer 阶段算法 都没写）
//   - feature edge snap、wordRe 模式

#include "XFoam/snap/xfoam_hex8ref.h"
#include "XFoam/snap/xfoam_layerparameters.h"
#include "XFoam/snap/xfoam_refinementparameters.h"
#include "XFoam/snap/xfoam_snapparameters.h"
#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <string>
#include <vector>

class XFoam_BlockMesh;
class XFoam_Dictionary;
class XFoam_VBrep;

/*---------------------------------------------------------------------------*\
                    Class XFoam_SnappyHexMesh Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_SnappyHexMesh
{
public:
	struct PhaseFlags
	{
		bool castellatedMesh = true; // 加密
		bool snap = true;            // 表面 snap
		bool addLayers = false;      // 未实现，默认关
		/// 一面一 patch：boundary face 按归属的 brep sub-patch（VBrep
		/// DiscreteFace.patchId / MBrep TopoDS_Face id）分桶 emit 多个
		/// polyMesh patch。默认 false（保留旧 1 patch/surface 行为）。
		/// snappyHexMeshDict 顶层 `perFacePatches true;` 打开。
		bool perFacePatches = false;

		/// 自动调整 mesh 参数到能保留所有 feature。打开后 run() 会按
		/// brep->minFeatureLength() 反推所需 maxLevel 并就地修改 surface
		/// maxLevel / snap implicitFeatureSnap / nFeatureSnapIter / tolerance /
		/// resolveFeatureAngle。为避免 cell 爆炸，maxLevel 提升幅度同时受
		/// fitFeaturesMaxLevelBump 与 maxLocalCells 双重封顶。snappyHexMeshDict
		/// 顶层 `fitFeatures true;` 打开。
		bool fitFeatures = false;
		/// fitFeatures 时 maxLevel 相对原值的最大提升幅度。默认 +1 是个安全
		/// 起点；user 在 dict 里 `fitFeaturesMaxLevelBump 2` (或更高) 可以让
		/// 加密追到更小 feature，但 MBrep + OCCT analytic 查询路径下 +2 可能
		/// 让 castellated 阶段慢 1-2 个数量级（base × 4^extraLevel × nFaces）。
		XFoam_Label fitFeaturesMaxLevelBump = 1;
	};

	/// 一个 refinementSurfaces.<name> 入口；resolveStl() 阶段调用方按 surface name
	/// 把 STL 装进 stlBySurface 向量并按 surfaces() 的顺序传给 run()。
	///
	/// level(min max)：min 当前作为"全局最低"未使用，max 是该 surface 上 cell 的目标 level。
	/// 文件路径 file：来自 geometry.<name>.file 或 geometry 入口 key 本身（去掉 .stl 同名）。
	struct SurfaceSpec
	{
		XFoam_Word name;        ///< refinementSurfaces 里的 key（也是 patch 名）
		XFoam_FileName file;    ///< 对应 geometry 入口里的 STL 文件名（不带路径）
		XFoam_Label minLevel = 0;
		XFoam_Label maxLevel = 0;
	};

	struct Stats
	{
		XFoam_Label nBgCells = 0;          // 背景 blockMesh 的 cell 数（单 block 时即 nx*ny*nz）
		XFoam_Label nRefinedCells = 0;     // 细化后未做剔除时的 sub-cell 总数
		XFoam_Label nKeptCells = 0;        // STL 切除后保留 cell 数（== 最终 polyMesh 的 nCells）
		XFoam_Label nSnappedPoints = 0;    // 投到 STL 上的边界点数
		XFoam_Scalar maxSnapDistance = 0;
		XFoam_Label nSmoothedInternalPoints = 0; ///< motionSmoother 移动了的内部点数（不含 patch 点）
		XFoam_Scalar maxInternalSmoothMove = 0;  ///< motionSmoother 对内部点造成的最大移动距离
		XFoam_Label nFeatureEdgeSnaps = 0;       ///< Snap #7 替换为 feature edge 投影的 boundary 点数
		XFoam_Label nFeatureVertexSnaps = 0;     ///< Snap #7 替换为 feature vertex 的 boundary 点数

		// Snap #9 pointConstraint 分类统计；relax / 内部 smoother 都按这份 DOF 约束做投影。
		XFoam_Label nPlaneConstrained = 0;       ///< 单 plane 约束的 boundary 点数（普通 surface snap）
		XFoam_Label nLineConstrained  = 0;       ///< 沿 line 约束的 boundary 点数（feature edge / 双 plane）
		XFoam_Label nFixedConstrained = 0;       ///< 完全锁死的 boundary 点数（feature vertex / 三 plane）

		// addLayers 阶段（按 patch 名启用 layers { sphere { nSurfaceLayers N; } }）。
		// 未启用 / 没匹配 patch 时全部保持 0。
		XFoam_Label nLayerPatches = 0;           ///< 实际跑过 layer 扩张的 patch 数
		XFoam_Label nLayerCellsAdded = 0;        ///< 新增的 prism layer cell 总数
		XFoam_Label nLayerPointsAdded = 0;       ///< 新增的 point 数（每层 × patch 上 unique 点数）
		XFoam_Label nLayerFacesAdded = 0;        ///< 新增的 face 数（含 side quads + bottom faces）
		XFoam_Label nLayerCellsNegative = 0;     ///< 新增 layer cell 中 V≤0 的数量（用作 quality 报警）
		XFoam_Scalar minLayerCellVolume = 0;     ///< 所有新增 layer cell 的最小体积

		// Snap #6 validate-and-relax：捕捉 snap 之后还残留的负体积/退化 cell。relax 没生效时
		// 这些字段保持 0。
		XFoam_Label  nBadCellsInitial = 0;       ///< snap+smoother 之后第一次 cellVolume 扫到 ≤0 的 cell 数
		XFoam_Label  nBadCellsFinal = 0;         ///< 全部 nRelaxIter 回退完仍 ≤0 的 cell 数
		XFoam_Label  nRelaxIterationsUsed = 0;   ///< 实际 relax 轮数（提前清零 bad 会 < nRelaxIter）
		XFoam_Scalar minCellVolumeInitial = 0;   ///< snap+smoother 之后最小 cell 体积（含负值）
		XFoam_Scalar minCellVolumeFinal = 0;     ///< relax 完成后最小 cell 体积
		XFoam_Word stlPatchName;           // STL 在输出中的 patch 名（来自 dict.geometry 首项）
		XFoam_Label refinementLevel = 0;   // 全局最大 level（来自 refinementSurfaces.<first>.level 第二个数）
		XFoam_Label maxAdaptiveLevel = 0;  // 实际出现的最大 base-cell level（考虑 buffer 扩张后的最大值）
		XFoam_Label perLevelCells[XFoam_Hex8Ref::kMaxLevelBuckets] = {0};
		XFoam_Label nPoints = 0;
		XFoam_Label nFaces = 0;
		XFoam_Label nInternalFaces = 0;
		XFoam_Label nBoundaryFaces = 0;
		XFoam_Label nSplitFaces = 0;       // 因相邻 level 不同被切的面数（diff=1 → 1 face 切成 4 sub-quads）
		XFoam_Label nPolyhedralCells = 0;  // 至少有 1 面被切的 sub-cell 数（topology 上是 polyhedron）
		XFoam_WordList outPatchNames;
		XFoam_WordList outPatchTypes;      // 与 outPatchNames 一一对应
	};

	/// 读 snappyHexMeshDict 子段；不持有 BlockMesh / STL。
	explicit XFoam_SnappyHexMesh(const XFoam_Dictionary& snappyDict);

	const XFoam_RefinementParameters& refineParams() const noexcept { return refine_; }
	const XFoam_SnapParameters& snapParams() const noexcept { return snap_; }
	const XFoam_LayerParameters& layerParams() const noexcept { return layer_; }
	const PhaseFlags& phases() const noexcept { return phases_; }

	const std::vector<SurfaceSpec>& surfaces() const noexcept { return surfaces_; }

	/// 全局最高 level（所有 surface 的 maxLevel 的 max）；buffer 后 oct 真实达到的 level
	/// 在 Stats::maxAdaptiveLevel。
	XFoam_Label globalRefinementLevel() const noexcept;

	/// 向后兼容：仅返回第一个 surface 的 name/file。
	const XFoam_Word& firstSurfaceName() const noexcept;
	const XFoam_FileName& firstSurfaceFile() const noexcept;

	/// 自适应 castellatedMesh + snap，直接把生成的 polyMesh 目录写到磁盘。
	///
	/// 算法概要：
	///   1) 基于背景 blockMesh 第 0 个 hex 块建结构化基网格 (nx0, ny0, nz0)。
	///   2) 每个 base-cell 取所有 surface 中 (bbox 相交 → 该 surface maxLevel) 的 max；
	///      没有任何 surface 相交者取 0。再按 nCellsBetweenLevels 做缓冲带扩张。
	///   3) 每个 base-cell 内按 2^level 三轴均匀剖分成 strict-hex sub-cells，cell 中心
	///      做 inside/outside：与 locationInMesh 同侧者保留（"inside" = 任意一个 STL contains）。
	///   4) 跨 base-cell 的相邻面如果两侧 level 不同（差恰为 1）：粗一侧的 sub-cell 把
	///      该面切成 2x2 的 4 个 sub-quad（用细一侧已经产生的中点/中心 Steiner 点），
	///      coarse 侧 sub-cell 变成 9-面 多面体。
	///   5) 每个 STL boundary face 按 face centroid 到各 STL 的最近距离归类到对应 patch；
	///      snap 时该 patch 上的点投到对应 STL 的最近点。
	///   6) 把全局点表、faces、owner/neighbour、patch 表（walls + 各 STL）直接写出 polyMesh。
	///
	/// 多 surface 版本：调用方按 surfaces() 的顺序逐个把 VBrep / MBrep 实例传进来；
	/// surfs.size() 必须等于 surfaces().size()。允许某项为 nullptr，那对应 surface
	/// 视为"不参与"。所有几何调用走 XFoam_BrepBase 虚函数 ── VBrep（BVH）和
	/// MBrep（OCCT analytic）任意混搭都行。snap 阶段如需 feature 投影，需要先调
	/// surfs[i]->buildFeatures(featureAngleDeg)。
	bool run(
		const XFoam_BlockMesh&                       bg,
		const std::vector<const XFoam_BrepBase*>&    surfs,
		const XFoam_FileName&                        outPolyMeshDir,
		Stats&                                       stats) const;

	/// 向后兼容：单 surface 的情况包装成 1 项 vector 后调多 surface 版本。
	bool run(
		const XFoam_BlockMesh& bg,
		const XFoam_BrepBase&  surf,
		const XFoam_FileName&  outPolyMeshDir,
		Stats&                 stats) const;

private:
	// snap_ / refine_ / surfaces_ 在 tuneForFeatures() 里就地修改（fitFeatures
	// 开关下根据 brep 的 minFeatureLength 反推后调高 maxLevel 等）。tune 在
	// const run() 路径里调用，故声明 mutable。
	mutable XFoam_RefinementParameters refine_;
	mutable XFoam_SnapParameters snap_;
	XFoam_LayerParameters layer_;
	PhaseFlags phases_;

	mutable std::vector<SurfaceSpec> surfaces_;
	mutable bool tunedForFeatures_ = false; ///< tuneForFeatures 已跑过

	void readPhaseFlags(const XFoam_Dictionary& snappyDict);
	void readRefinementSurfaces(const XFoam_Dictionary& snappyDict);
	void readGeometry(const XFoam_Dictionary& snappyDict);

	/// 当 phases_.fitFeatures=true 时，在 run() 开头按 brep 反报的最小 feature
	/// 尺度自动调高 surface maxLevel / 强制 implicitFeatureSnap / 拉宽 snap
	/// tolerance / 拉低 resolveFeatureAngle。base_cell_min_extent 从 bg 推。
	/// 一次性修改，重复调 run() 不会反复调（已调过的不再触发；可由 dict 重读
	/// 或 setter 复位）。
	void tuneForFeatures(
		const XFoam_BlockMesh&                       bg,
		const std::vector<const XFoam_BrepBase*>&    surfs) const;
};

#endif
