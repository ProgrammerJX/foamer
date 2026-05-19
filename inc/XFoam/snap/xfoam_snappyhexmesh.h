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

#include <string>

class XFoam_BlockMesh;
class XFoam_Dictionary;
class XFoam_PolyMesh;

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
		XFoam_Label nBgCells = 0;
		XFoam_Label nRefinedCells = 0;
		XFoam_Label nKeptCells = 0;
		XFoam_Label nSnappedPoints = 0;
		XFoam_Scalar maxSnapDistance = 0;
		XFoam_Word stlPatchName;
		XFoam_Label refinementLevel = 0;
		// 与生成的 PolyMesh.boundary() 一一对应的 patch type 列表（如 "wall", "patch"），
		// 用作 XFoam_PolyMesh::writePolyMeshDir 的 patchTypes 入参。
		XFoam_WordList outPatchTypes;
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

	/// 执行 castellatedMesh + snap，构造一个新的 polyMesh 返回。
	/// \param bg          背景 BlockMesh（必须是单 hex 块，density >= 1）
	/// \param stl         读好的三角面（不能空，必须水密）
	/// \param outPolyMesh 出参：填入 owned 新 polyMesh（caller 持有 AutoPtr）
	/// \param stats       出参：诊断统计
	/// \return true 若成功；false 即非可恢复错误。
	bool run(
		const XFoam_BlockMesh& bg,
		const XFoam_TriSurface& stl,
		XFoam_AutoPtr<XFoam_PolyMesh>& outPolyMesh,
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
