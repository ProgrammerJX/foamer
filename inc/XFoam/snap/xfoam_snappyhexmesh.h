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
//   - 多 STL；只用 dict 中 refinementSurfaces 的第一个

#include "XFoam/snap/xfoam_layerparameters.h"
#include "XFoam/snap/xfoam_refinementparameters.h"
#include "XFoam/snap/xfoam_snapparameters.h"
#include "XFoam/snap/xfoam_trisurface.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <string>

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

	struct Stats
	{
		XFoam_Label nBgCells = 0;          // 背景 blockMesh 的 cell 数（单 block 时即 nx*ny*nz）
		XFoam_Label nRefinedCells = 0;     // 细化后未做剔除时的 sub-cell 总数
		XFoam_Label nKeptCells = 0;        // STL 切除后保留 cell 数（== 最终 polyMesh 的 nCells）
		XFoam_Label nSnappedPoints = 0;    // 投到 STL 上的边界点数
		XFoam_Scalar maxSnapDistance = 0;
		XFoam_Word stlPatchName;           // STL 在输出中的 patch 名（来自 dict.geometry 首项）
		XFoam_Label refinementLevel = 0;   // 全局最大 level（来自 refinementSurfaces.<first>.level 第二个数）
		XFoam_Label maxAdaptiveLevel = 0;  // 实际出现的最大 base-cell level（考虑 buffer 扩张后的最大值）
		XFoam_Label perLevelCells[8] = {0, 0, 0, 0, 0, 0, 0, 0};
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

	XFoam_Label globalRefinementLevel() const noexcept { return globalLevel_; }
	const XFoam_Word& firstSurfaceName() const noexcept { return firstSurfaceName_; }
	const XFoam_FileName& firstSurfaceFile() const noexcept { return firstSurfaceFile_; }

	/// 自适应 castellatedMesh + snap，直接把生成的 polyMesh 目录写到磁盘。
	///
	/// 算法概要：
	///   1) 基于背景 blockMesh 第 0 个 hex 块建结构化基网格 (nx0, ny0, nz0)。
	///   2) 为每个 base-cell 分配一个 level：bbox 与 STL 三角面 bbox 有交集的取
	///      refinementSurfaces.<first>.level 的 max，其他取 0。再按 nCellsBetweenLevels
	///      做缓冲带扩张，保证相邻 level 差不超过 1。
	///   3) 每个 base-cell 内按 2^level 三轴均匀剖分成 strict-hex sub-cells，cell 中心
	///      做 STL 内/外判定，保留与 locationInMesh 同侧者。
	///   4) 跨 base-cell 的相邻面如果两侧 level 不同（差恰为 1）：粗一侧的 sub-cell 把
	///      该面切成 2x2 的 4 个 sub-quad（用细一侧已经产生的中点/中心 Steiner 点），
	///      coarse 侧 sub-cell 变成 9-面 多面体。
	///   5) 把全局点表、faces、owner/neighbour、patch 表直接写出 polyMesh 目录。
	///
	/// 假设：单 hex 块；最高 level 与 buffer 之后相邻 level 差为 1；nCellsBetweenLevels >= 1。
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

	XFoam_Label globalLevel_ = 0;
	XFoam_Word firstSurfaceName_; // refinementSurfaces 第一个 key
	XFoam_FileName firstSurfaceFile_;

	void readPhaseFlags(const XFoam_Dictionary& snappyDict);
	void readRefinementSurfacesFirst(const XFoam_Dictionary& snappyDict);
	void readGeometryFirst(const XFoam_Dictionary& snappyDict);
};

#endif
