// 端到端 smoke test for the snap module:
//   1) 解析 snappyHexMeshDict 把三个 sub-dict 装到三个参数结构
//   2) 跑 sphere-in-box 全流程：blockMesh → SnappyHexMesh.run → PolyMesh
//   3) 校验 cell/face/point 数量级 + snap 位移
//
// 不验精确数字（依赖 sphere.stl 的三角化），只确认：
//   - 加密因子对：refined cells = bg cells * (2^L)^3
//   - 至少切掉了一些 cell
//   - 至少 snap 了一些点

#include "doctest/doctest.h"
#include "tcommon.h"

#include "XFoam/block/xfoam_blockmesh.h"
#include "XFoam/snap/xfoam_snappyhexmesh.h"
#include "XFoam/snap/xfoam_trisurface.h"
#include "XFoam/utilities/xfoam_dictionary.h"

#include <boost/filesystem.hpp>
#include <cstdio>

namespace
{
inline XFoam_FileName XFoamTests_blockBg(const char* f)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(f) / "dict" / "blockMeshDict_snappyBackground")
			.lexically_normal().generic_string());
}
inline XFoam_FileName XFoamTests_snappyDict(const char* f)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(f) / "dict" / "snappyHexMeshDict")
			.lexically_normal().generic_string());
}
inline std::string XFoamTests_sphereStl(const char* f)
{
	return (XFoamTests_dataDir(f) / "triSurface" / "sphere.stl")
		.lexically_normal().generic_string();
}
inline XFoam_FileName XFoamTests_blockCyl(const char* f)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(f) / "dict" / "blockMeshDict_cylinderBackground")
			.lexically_normal().generic_string());
}
inline XFoam_FileName XFoamTests_snappyCyl(const char* f)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(f) / "dict" / "snappyHexMeshDict_cylinder")
			.lexically_normal().generic_string());
}
inline std::string XFoamTests_cylinderStl(const char* f)
{
	return (XFoamTests_dataDir(f) / "triSurface" / "cylinder1.stl")
		.lexically_normal().generic_string();
}
} // namespace

TEST_CASE("snap: dict parsing populates refine/snap/layer + level + surface name")
{
	const XFoam_FileName dictPath = XFoamTests_snappyDict(__FILE__);
	REQUIRE(dictPath.type() == XFoam_FileType::file);

	const XFoam_IODictionary dictIO(XFoam_systemDictIO(dictPath));
	const XFoam_SnappyHexMesh snap(dictIO);

	// 来自 castellatedMeshControls
	CHECK(snap.refineParams().maxLocalCells == 100000);
	CHECK(snap.refineParams().maxGlobalCells == 200000);
	CHECK(snap.refineParams().nCellsBetweenLevels == 1);
	CHECK(snap.refineParams().hasLocationInMesh);
	CHECK(snap.refineParams().locationInMesh.x() == doctest::Approx(1.7));
	CHECK(snap.refineParams().locationInMesh.y() == doctest::Approx(0.0));
	CHECK(snap.refineParams().locationInMesh.z() == doctest::Approx(0.0));

	// 来自 snapControls
	CHECK(snap.snapParams().nSmoothPatch == 3);
	CHECK(snap.snapParams().tolerance == doctest::Approx(2.0));

	// 来自 addLayersControls
	CHECK(snap.layerParams().expansionRatio == doctest::Approx(1.2));
	CHECK(snap.layerParams().relativeSizes == true);

	// 来自 refinementSurfaces.sphere.level (1 1) → globalLevel = 1
	CHECK(snap.globalRefinementLevel() == 1);
	CHECK(static_cast<const std::string&>(static_cast<const XFoam_String&>(snap.firstSurfaceName())) == "sphere");

	// 来自 geometry 第一项
	const std::string fileName = static_cast<const std::string&>(
		static_cast<const XFoam_String&>(snap.firstSurfaceFile()));
	CHECK(fileName == "sphere.stl");

	// phase 标志
	CHECK(snap.phases().castellatedMesh == true);
	CHECK(snap.phases().snap == true);
	CHECK(snap.phases().addLayers == false);
}

TEST_CASE("snap: TriSurface reads sphere.stl and inside/outside test works")
{
	XFoam_TriSurface stl;
	const std::string path = XFoamTests_sphereStl(__FILE__);
	REQUIRE(stl.read(path));
	CHECK(stl.size() == 120); // PowerShell-generated UV sphere: 12 lon * 6 lat = 120 tris

	// 球心在内
	CHECK(stl.contains(XFoam_Vector3D(0, 0, 0)));
	// 远点在外
	CHECK(!stl.contains(XFoam_Vector3D(1.7, 0, 0)));
	CHECK(!stl.contains(XFoam_Vector3D(-1.7, 0, 0)));
	CHECK(!stl.contains(XFoam_Vector3D(0, 0, 1.7)));

	// 距离单调：球心 ≈ R，球外远点 > R
	const XFoam_Scalar R = 0.7;
	CHECK(stl.distance(XFoam_Vector3D(0, 0, 0)) == doctest::Approx(R).epsilon(0.05));
}

TEST_CASE("snap: end-to-end sphere-in-box → refine + cull + snap (uniform level=1)")
{
	namespace fs = boost::filesystem;
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockBg(__FILE__)));
	XFoam_BlockMesh bg(bgIO);
	REQUIRE(bg.cells().size() == 64); // 4 x 4 x 4 background

	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyDict(__FILE__)));
	const XFoam_SnappyHexMesh snappy(sIO);

	XFoam_TriSurface stl;
	REQUIRE(stl.read(XFoamTests_sphereStl(__FILE__)));

	// 临时输出目录（用 source-file 路径区分）
	const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_sphere";
	fs::remove_all(outDir);
	const XFoam_FileName outF(outDir.generic_string());

	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, outF, stats));

	CHECK(stats.nBgCells == 64);
	// dict 里 refinementSurfaces.sphere.level = (1 1)。
	// 自适应版本只把 bbox 与球（r=0.7）相交的 base-cell 升到 level 1，其它保持 level 0：
	// box=[-2,2]^3, 4x4x4=64 base cells（cell 边长 1），球的 8 个中心 cell 命中 → level 1，
	// 加密后 sub-cell 数 = (64-8)*1 + 8*8 = 56 + 64 = 120。
	CHECK(stats.maxAdaptiveLevel == 1);
	CHECK(stats.nBgCells > stats.perLevelCells[0]); // 至少一个 base-cell 不在 level 0
	CHECK(stats.perLevelCells[1] > 0);              // 至少一个 base-cell 在 level 1
	CHECK(stats.nRefinedCells == 120);
	CHECK(stats.nKeptCells < stats.nRefinedCells); // 切掉了球内一些 cell
	CHECK(stats.nKeptCells > 0);
	CHECK(stats.nSnappedPoints > 0);
	// 既然有 level 混合，必然有 split face 与 polyhedral cell
	CHECK(stats.nSplitFaces > 0);
	CHECK(stats.nPolyhedralCells > 0);
	CHECK(stats.outPatchNames.size() == 2); // walls + sphere
	CHECK(stats.outPatchTypes.size() == 2);

	CHECK(fs::is_regular_file(outDir / "points"));
	CHECK(fs::is_regular_file(outDir / "faces"));
	CHECK(fs::is_regular_file(outDir / "owner"));
	CHECK(fs::is_regular_file(outDir / "neighbour"));
	CHECK(fs::is_regular_file(outDir / "boundary"));
}

TEST_CASE("snap: end-to-end cylinder-in-box → 自适应 level 0/1/2 + 过渡带")
{
	namespace fs = boost::filesystem;
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockCyl(__FILE__)));
	XFoam_BlockMesh bg(bgIO);
	REQUIRE(bg.cells().size() == 512); // 8 x 8 x 8 background

	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyCyl(__FILE__)));
	const XFoam_SnappyHexMesh snappy(sIO);
	REQUIRE(snappy.globalRefinementLevel() == 2);

	XFoam_TriSurface stl;
	REQUIRE(stl.read(XFoamTests_cylinderStl(__FILE__)));

	const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_cyl";
	fs::remove_all(outDir);
	const XFoam_FileName outF(outDir.generic_string());

	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, outF, stats));

	CHECK(stats.nBgCells == 512);
	CHECK(stats.maxAdaptiveLevel == 2);
	// 三层都应该有 cell：L=0（远处）、L=1（缓冲）、L=2（贴 STL）。
	CHECK(stats.perLevelCells[0] > 0);
	CHECK(stats.perLevelCells[1] > 0);
	CHECK(stats.perLevelCells[2] > 0);
	// 八叉树叶子直接累加（不像 per-base-cell 那样每个 L 还要乘 8^L）。
	const XFoam_Label expectRef =
		stats.perLevelCells[0] + stats.perLevelCells[1] + stats.perLevelCells[2];
	CHECK(stats.nRefinedCells == expectRef);
	CHECK(stats.nKeptCells > 0);
	CHECK(stats.nKeptCells < stats.nRefinedCells);
	CHECK(stats.nSnappedPoints > 0);
	// 必然有切面（L0/L1、L1/L2 边界都会）
	CHECK(stats.nSplitFaces > 0);
	CHECK(stats.nPolyhedralCells > 0);
	// 两个 patch
	CHECK(stats.outPatchNames.size() == 2);

	CHECK(fs::is_regular_file(outDir / "boundary"));
}

inline std::string XFoamTests_snappyMulti(const char* f)
{
	return (XFoamTests_dataDir(f) / "dict" / "snappyHexMeshDict_multi")
		.generic_string();
}

TEST_CASE("snap: multi-surface dict parsing (sphere + cylinder1, level 1 and 2)")
{
	const XFoam_FileName dictPath(XFoamTests_snappyMulti(__FILE__));
	REQUIRE(dictPath.type() == XFoam_FileType::file);

	const XFoam_IODictionary dictIO(XFoam_systemDictIO(dictPath));
	const XFoam_SnappyHexMesh snap(dictIO);

	REQUIRE(snap.surfaces().size() == 2);
	CHECK(static_cast<const std::string&>(static_cast<const XFoam_String&>(snap.surfaces()[0].name)) == "sphere");
	CHECK(static_cast<const std::string&>(static_cast<const XFoam_String&>(snap.surfaces()[1].name)) == "cylinder1");
	CHECK(snap.surfaces()[0].maxLevel == 1);
	CHECK(snap.surfaces()[1].maxLevel == 2);
	CHECK(static_cast<const std::string&>(static_cast<const XFoam_String&>(snap.surfaces()[0].file)) == "sphere.stl");
	CHECK(static_cast<const std::string&>(static_cast<const XFoam_String&>(snap.surfaces()[1].file)) == "cylinder1.stl");

	CHECK(snap.globalRefinementLevel() == 2);
	CHECK(static_cast<const std::string&>(static_cast<const XFoam_String&>(snap.firstSurfaceName())) == "sphere");
}

TEST_CASE("snap: multi-surface end-to-end (sphere L1 + cylinder L2 in [-1,1]^3 box)")
{
	namespace fs = boost::filesystem;
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockCyl(__FILE__)));
	XFoam_BlockMesh bg(bgIO);
	REQUIRE(bg.cells().size() == 512);

	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(XFoamTests_snappyMulti(__FILE__))));
	const XFoam_SnappyHexMesh snappy(sIO);
	REQUIRE(snappy.surfaces().size() == 2);

	XFoam_TriSurface sphereStl;
	XFoam_TriSurface cylStl;
	REQUIRE(sphereStl.read(XFoamTests_sphereStl(__FILE__)));
	REQUIRE(cylStl.read(XFoamTests_cylinderStl(__FILE__)));

	std::vector<const XFoam_TriSurface*> stls;
	stls.push_back(&sphereStl);
	stls.push_back(&cylStl);

	const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_multi";
	fs::remove_all(outDir);
	const XFoam_FileName outF(outDir.generic_string());

	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stls, outF, stats));

	// 两个 STL 都应触发 refinement：L1 来自 sphere bbox-相交、L2 来自 cylinder。
	CHECK(stats.maxAdaptiveLevel == 2);
	CHECK(stats.perLevelCells[1] > 0);
	CHECK(stats.perLevelCells[2] > 0);

	// 至少有 walls patch；其它 surface patch 取决于 cull 后是否有外侧暴露面
	// (本测试里 cylinder ⊂ sphere → cylinder 的所有 cell 也在 sphere 内被 cull，
	//  几何上不再有 cylinder 面)。所以可能只看到 walls + sphere。
	REQUIRE(stats.outPatchNames.size() >= 1);
	CHECK(static_cast<const std::string&>(static_cast<const XFoam_String&>(stats.outPatchNames[0])) == "walls");

	CHECK(fs::is_regular_file(outDir / "boundary"));
}
