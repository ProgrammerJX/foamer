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
#include "XFoam/snap/xfoam_trisurface.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <string>
#include <vector>

class XFoam_BlockMesh;
class XFoam_Dictionary;

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
	/// 多 surface 版本：调用方按 surfaces() 的顺序逐个 stl.read() 后传入；stls.size()
	/// 必须等于 surfaces().size()。允许某项为 nullptr，那对应 surface 视为"不参与"。
	bool run(
		const XFoam_BlockMesh& bg,
		const std::vector<const XFoam_TriSurface*>& stls,
		const XFoam_FileName& outPolyMeshDir,
		Stats& stats) const;

	/// 向后兼容：单 STL 的情况包装成 1 项 vector 后调多 surface 版本。
	bool run(
		const XFoam_BlockMesh& bg,
		const XFoam_TriSurface& stl,
		const XFoam_FileName& outPolyMeshDir,
		Stats& stats) const;

private:
	XFoam_RefinementParameters refine_;
	XFoam_SnapParameters snap_;
	XFoam_LayerParameters layer_;
	PhaseFlags phases_;

	std::vector<SurfaceSpec> surfaces_;

	void readPhaseFlags(const XFoam_Dictionary& snappyDict);
	void readRefinementSurfaces(const XFoam_Dictionary& snappyDict);
	void readGeometry(const XFoam_Dictionary& snappyDict);
};

#endif
