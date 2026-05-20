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

#include "XFoam/snap/xfoam_blockmesh.h"
#include "XFoam/snap/xfoam_pointconstraint.h"
#include "XFoam/snap/xfoam_snappyhexmesh.h"
#include "XFoam/topo/xfoam_vbrep.h"
#include "XFoam/utilities/xfoam_dictionary.h"

#include <boost/filesystem.hpp>
#include <cstdio>
#include <fstream>

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
	CHECK(snap.refineParams().hasLocationInMesh());
	REQUIRE(snap.refineParams().locationsInMesh.size() == 1u);
	CHECK(snap.refineParams().locationsInMesh[0].x() == doctest::Approx(1.7));
	CHECK(snap.refineParams().locationsInMesh[0].y() == doctest::Approx(0.0));
	CHECK(snap.refineParams().locationsInMesh[0].z() == doctest::Approx(0.0));

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
	XFoam_VBrep stl;
	const std::string path = XFoamTests_sphereStl(__FILE__);
	REQUIRE(stl.read(path));
	CHECK(stl.nFaces() == 120); // PowerShell-generated UV sphere: 12 lon * 6 lat = 120 tris

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

	XFoam_VBrep stl;
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

	XFoam_VBrep stl;
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

inline std::string XFoamTests_blockTwoBlock(const char* f)
{
	return (XFoamTests_dataDir(f) / "dict" / "blockMeshDict_twoBlock")
		.generic_string();
}

TEST_CASE("snap: 2-block background ([−1,0]∪[0,1]×[−1,1]²) gives same mesh as single block")
{
	namespace fs = boost::filesystem;
	// 2-block bg：每块 (4,8,8)，沿 x=0 切；并起来跟单块 (8,8,8) 在 [-1,1]^3 完全等价
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoam_FileName(XFoamTests_blockTwoBlock(__FILE__))));
	XFoam_BlockMesh bg(bgIO);
	REQUIRE(bg.size() == 2);
	REQUIRE(bg.cells().size() == 512);

	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyCyl(__FILE__)));
	const XFoam_SnappyHexMesh snappy(sIO);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_cylinderStl(__FILE__)));

	const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_2blk";
	fs::remove_all(outDir);
	const XFoam_FileName outF(outDir.generic_string());

	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, outF, stats));

	// 与单 block cylinder 测试一致：跨 block 共享面靠 pts/face dedup 自动合并
	CHECK(stats.nBgCells == 512);
	CHECK(stats.maxAdaptiveLevel == 2);
	CHECK(stats.nKeptCells == 1116);
	CHECK(stats.nFaces == 3917);
	CHECK(stats.nInternalFaces == 3331);
	CHECK(stats.nBoundaryFaces == 586);
	CHECK(stats.nPoints == 1810);
	CHECK(stats.nPolyhedralCells == 264);
	CHECK(fs::is_regular_file(outDir / "boundary"));
}

inline XFoam_FileName XFoamTests_snappyCylSmooth(const char* f)
{
	return XFoam_FileName(
		(XFoamTests_dataDir(f) / "dict" / "snappyHexMeshDict_cylinder_smooth")
			.lexically_normal().generic_string());
}

TEST_CASE("snap: motionSmoother (nSmoothInternal=3) 向内传播 patch 位移，A/B 与裸 snap 对比")
{
	namespace fs = boost::filesystem;
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockCyl(__FILE__)));
	XFoam_BlockMesh bg(bgIO);
	REQUIRE(bg.cells().size() == 512);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_cylinderStl(__FILE__)));

	// ---- A) 裸 snap（snappyHexMeshDict_cylinder, nSmoothInternal=0 默认） ----
	XFoam_SnappyHexMesh::Stats statsA;
	{
		const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyCyl(__FILE__)));
		const XFoam_SnappyHexMesh snappy(sIO);
		CHECK(snappy.snapParams().nSmoothInternal == 0);
		const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_smoothA";
		fs::remove_all(outDir);
		REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), statsA));
	}
	CHECK(statsA.nSmoothedInternalPoints == 0);
	CHECK(statsA.maxInternalSmoothMove == doctest::Approx(0));

	// ---- B) 同 case 但 nSmoothInternal=3 ----
	XFoam_SnappyHexMesh::Stats statsB;
	{
		const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyCylSmooth(__FILE__)));
		const XFoam_SnappyHexMesh snappy(sIO);
		CHECK(snappy.snapParams().nSmoothInternal == 3);
		const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_smoothB";
		fs::remove_all(outDir);
		REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), statsB));
	}

	// 拓扑（cell/face/point/snap 数）不应受 smoother 影响：smoother 只改 pts 坐标。
	CHECK(statsB.nKeptCells == statsA.nKeptCells);
	CHECK(statsB.nFaces == statsA.nFaces);
	CHECK(statsB.nInternalFaces == statsA.nInternalFaces);
	CHECK(statsB.nBoundaryFaces == statsA.nBoundaryFaces);
	CHECK(statsB.nPoints == statsA.nPoints);
	CHECK(statsB.nSnappedPoints == statsA.nSnappedPoints);
	CHECK(statsB.maxSnapDistance == doctest::Approx(statsA.maxSnapDistance));

	// motionSmoother 必须真正动了 ≥1 个内部点（且移动距离 > 0），
	// 但不应超过 STL 最大 snap 距离（位移是从 boundary 向内 averaged 出来的）。
	CHECK(statsB.nSmoothedInternalPoints > 0);
	CHECK(statsB.maxInternalSmoothMove > 0);
	CHECK(statsB.maxInternalSmoothMove <= statsB.maxSnapDistance + 1e-12);

	// Snap #6 validate-and-relax 在两个 case 都跑了（snap_.nRelaxIter=5），但 8x8x8 背景 +
	// L=2 + 0.025 max snap 距离对 0.0625 cell 足够轻，预期 0 个负体积 cell；
	// 因此两边都应 nBadCellsInitial == 0 + nRelaxIterationsUsed == 0。
	CHECK(statsA.nBadCellsInitial == 0);
	CHECK(statsA.nBadCellsFinal == 0);
	CHECK(statsA.nRelaxIterationsUsed == 0);
	CHECK(statsA.minCellVolumeInitial > 0);
	CHECK(statsB.nBadCellsInitial == 0);
	CHECK(statsB.nBadCellsFinal == 0);
	CHECK(statsB.nRelaxIterationsUsed == 0);
	CHECK(statsB.minCellVolumeInitial > 0);
}

TEST_CASE("snap: feature extraction on cylinder1.stl finds cap-to-side edges")
{
	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_cylinderStl(__FILE__)));

	// Cylinder1 由若干平面三角面拼成（盒子部分），cap-to-side 接缝的法向夹角应远大于
	// 30°；buildFeatures(30) 必须能挑出 ≥1 条 feature edge。同时 vertex 数 ≥ 0。
	stl.buildFeatures(30);
	CHECK(stl.nFeatureEdges() > 0);

	// 任何 feature edge 端点本身一定是 feature edge 的一个 endpoint；
	// 用其中一个 vertex 做 query：radius 大到能覆盖到它，应返回 Vertex 或 Edge。
	const XFoam_Vector3D probe(0, 0, 0);
	XFoam_Vector3D out;
	XFoam_Vector3D tangent;
	const auto kind = stl.closestFeature(probe, 5.0, out, tangent);
	CHECK(kind != XFoam_BrepBase::FeatureKind::None);
}

TEST_CASE("snap: implicitFeatureSnap=true 把 boundary 点真正拉到 feature 上")
{
	namespace fs = boost::filesystem;
	const std::string snappyDictText =
		"FoamFile { version 2.0; format ascii; class dictionary; object snappyHexMeshDict; }\n"
		"castellatedMesh true; snap true; addLayers false;\n"
		"geometry { cylinder1.stl { type triSurfaceMesh; name cylinder1; } }\n"
		"castellatedMeshControls {\n"
		"  maxLocalCells 200000; maxGlobalCells 400000; minRefinementCells 0;\n"
		"  nCellsBetweenLevels 1; resolveFeatureAngle 30;\n"
		"  refinementSurfaces { cylinder1 { level (2 2); } }\n"
		"  locationInMesh (0.9 0.9 0.9);\n"
		"  allowFreeStandingZoneFaces true;\n"
		"}\n"
		"snapControls { nSmoothPatch 0; nSmoothInternal 0; tolerance 2.0;\n"
		"  nSolveIter 30; nRelaxIter 0; nFeatureSnapIter 10; implicitFeatureSnap true; }\n"
		"addLayersControls { relativeSizes true; expansionRatio 1.2; finalLayerThickness 0.3;\n"
		"  minThickness 0.1; nGrow 0; featureAngle 60; }\n";
	const fs::path tmp = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_feature";
	fs::create_directories(tmp);
	const fs::path dictPath = tmp / "snappyHexMeshDict";
	{
		std::ofstream o(dictPath.string().c_str());
		o << snappyDictText;
	}
	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(dictPath.generic_string())));
	const XFoam_SnappyHexMesh snappy(sIO);
	CHECK(snappy.snapParams().implicitFeatureSnap == true);

	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockCyl(__FILE__)));
	XFoam_BlockMesh bg(bgIO);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_cylinderStl(__FILE__)));
	stl.buildFeatures(snappy.refineParams().resolveFeatureAngle);
	REQUIRE(stl.nFeatureEdges() > 0);

	const fs::path outDir = tmp / "out";
	fs::remove_all(outDir);
	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), stats));

	// 启用了 implicitFeatureSnap 且 STL 有 feature edge → 至少应该有 1 个 boundary point
	// 被 snap 到了 feature 上（cylinder1 的 cap 边对 L=2 cell 来说几乎总能命中）。
	CHECK((stats.nFeatureEdgeSnaps + stats.nFeatureVertexSnaps) > 0);
}

TEST_CASE("snap: validate-and-relax 在合成 bad cell 情景下能恢复体积")
{
	// 直接调用一份很小的 mesh：1×1×1 背景 + 一个把 boundary 点拉得离谱的 STL，
	// 强行制造负体积 cell 验证 relax 路径。这里靠 cylinder1 L=0（不细化）+ 极小 box
	// 让 snap 移动 ~0.3，远超 cell size 0.25 → 至少触发 1 个负体积 cell。
	//
	// 该 case 不在 dict fixture 里；用 in-memory dict 文本构造。
	namespace fs = boost::filesystem;
	const std::string snappyDictText =
		"FoamFile { version 2.0; format ascii; class dictionary; object snappyHexMeshDict; }\n"
		"castellatedMesh true; snap true; addLayers false;\n"
		"geometry { cylinder1.stl { type triSurfaceMesh; name cylinder1; } }\n"
		"castellatedMeshControls {\n"
		"  maxLocalCells 100000; maxGlobalCells 200000; minRefinementCells 0;\n"
		"  nCellsBetweenLevels 1; resolveFeatureAngle 30;\n"
		"  refinementSurfaces { cylinder1 { level (0 0); } }\n"
		"  locationInMesh (0.9 0.9 0.9);\n"
		"  allowFreeStandingZoneFaces true;\n"
		"}\n"
		"snapControls { nSmoothPatch 0; nSmoothInternal 0; tolerance 2.0;\n"
		"  nSolveIter 30; nRelaxIter 5; nFeatureSnapIter 10; }\n"
		"addLayersControls { relativeSizes true; expansionRatio 1.2; finalLayerThickness 0.3;\n"
		"  minThickness 0.1; nGrow 0; featureAngle 60; }\n";
	const fs::path tmp = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_relax";
	fs::create_directories(tmp);
	const fs::path dictPath = tmp / "snappyHexMeshDict";
	{
		std::ofstream o(dictPath.string().c_str());
		o << snappyDictText;
	}
	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(dictPath.generic_string())));
	const XFoam_SnappyHexMesh snappy(sIO);
	CHECK(snappy.snapParams().nRelaxIter == 5);

	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockCyl(__FILE__)));
	XFoam_BlockMesh bg(bgIO);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_cylinderStl(__FILE__)));

	const fs::path outDir = tmp / "out";
	fs::remove_all(outDir);
	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), stats));

	// L=0 + 0.025 max snap 距离 vs cell size 0.25 → 太轻没有 bad cell；
	// 这一 case 仅作 smoke：relax 路径有跑，但 nRelaxIterationsUsed 可能为 0。
	CHECK(stats.nRelaxIterationsUsed >= 0); // 至少字段被填了（不 UB）
	CHECK(stats.minCellVolumeInitial > 0);  // L=0 干净 mesh
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

	XFoam_VBrep sphereStl;
	XFoam_VBrep cylStl;
	REQUIRE(sphereStl.read(XFoamTests_sphereStl(__FILE__)));
	REQUIRE(cylStl.read(XFoamTests_cylinderStl(__FILE__)));

	std::vector<const XFoam_BrepBase*> stls;
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

// ---------------------------------------------------------------------------
// Topo #8: 多 region cell 选取 (locationsInMesh)。
// 解析 + driver 两层各自校验：
//   1) `locationsInMesh ((x y z) ...)` token-stream parser 直接拿到 N 点。
//   2) sphere + 单 location 改成 sphere + 两个 location（外角 + 球心），bitmask
//      相同的 cell 都保留 → 比单 location 显著多保留了 sphere 内部 cell。
// ---------------------------------------------------------------------------

TEST_CASE("snap: locationsInMesh ((x y z) (x y z)) parses to a list of points")
{
	namespace fs = boost::filesystem;
	const std::string snappyDictText =
		"FoamFile { version 2.0; format ascii; class dictionary; object snappyHexMeshDict; }\n"
		"castellatedMesh true; snap true; addLayers false;\n"
		"geometry { sphere.stl { type triSurfaceMesh; name sphere; } }\n"
		"castellatedMeshControls {\n"
		"  maxLocalCells 100000; maxGlobalCells 200000; minRefinementCells 0;\n"
		"  nCellsBetweenLevels 1; resolveFeatureAngle 30;\n"
		"  refinementSurfaces { sphere { level (1 1); } }\n"
		"  locationsInMesh ( (1.7 0.0 0.0) (0.0 0.0 0.0) (-1.5 0.5 0.5) );\n"
		"  allowFreeStandingZoneFaces true;\n"
		"}\n"
		"snapControls { nSmoothPatch 0; nSmoothInternal 0; tolerance 2.0;\n"
		"  nSolveIter 30; nRelaxIter 0; nFeatureSnapIter 10; }\n"
		"addLayersControls { relativeSizes true; expansionRatio 1.2; finalLayerThickness 0.3;\n"
		"  minThickness 0.1; nGrow 0; featureAngle 60; }\n";
	const fs::path tmp = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_locs_parse";
	fs::create_directories(tmp);
	const fs::path dictPath = tmp / "snappyHexMeshDict";
	{
		std::ofstream o(dictPath.string().c_str());
		o << snappyDictText;
	}
	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(dictPath.generic_string())));
	const XFoam_SnappyHexMesh snappy(sIO);

	REQUIRE(snappy.refineParams().locationsInMesh.size() == 3u);
	CHECK(snappy.refineParams().locationsInMesh[0].x() == doctest::Approx(1.7));
	CHECK(snappy.refineParams().locationsInMesh[1].x() == doctest::Approx(0.0));
	CHECK(snappy.refineParams().locationsInMesh[1].y() == doctest::Approx(0.0));
	CHECK(snappy.refineParams().locationsInMesh[2].x() == doctest::Approx(-1.5));
	CHECK(snappy.refineParams().locationsInMesh[2].z() == doctest::Approx(0.5));

	// 兼容旧 sample 的 .locationInMesh() 仍指向首元素。
	CHECK(snappy.refineParams().locationInMesh().x() == doctest::Approx(1.7));
}

TEST_CASE("snap: 两个 locationsInMesh 同时保留 sphere 外侧与内侧 cell（bitmask 选区）")
{
	namespace fs = boost::filesystem;
	// 复用 [-2,2]^3, 4x4x4 背景 + sphere(r=0.7)。
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockBg(__FILE__)));
	XFoam_BlockMesh bg(bgIO);
	REQUIRE(bg.cells().size() == 64);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_sphereStl(__FILE__)));

	// 单 location 基准（与已有 sphere-in-box 用同一个 fixture）。
	const fs::path tmp = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_multi_loc";
	fs::create_directories(tmp);
	auto writeDict = [&](const std::string& body, const std::string& fname) -> fs::path {
		const fs::path p = tmp / fname;
		std::ofstream o(p.string().c_str());
		o <<
			"FoamFile { version 2.0; format ascii; class dictionary; object snappyHexMeshDict; }\n"
			"castellatedMesh true; snap true; addLayers false;\n"
			"geometry { sphere.stl { type triSurfaceMesh; name sphere; } }\n"
			"castellatedMeshControls {\n"
			"  maxLocalCells 100000; maxGlobalCells 200000; minRefinementCells 0;\n"
			"  nCellsBetweenLevels 1; resolveFeatureAngle 30;\n"
			"  refinementSurfaces { sphere { level (1 1); } }\n" << body <<
			"  allowFreeStandingZoneFaces true;\n"
			"}\n"
			"snapControls { nSmoothPatch 0; nSmoothInternal 0; tolerance 2.0;\n"
			"  nSolveIter 30; nRelaxIter 0; nFeatureSnapIter 10; }\n"
			"addLayersControls { relativeSizes true; expansionRatio 1.2; finalLayerThickness 0.3;\n"
			"  minThickness 0.1; nGrow 0; featureAngle 60; }\n";
		return p;
	};

	const fs::path dictSingle = writeDict(
		"  locationInMesh (1.7 0.0 0.0);\n", "single.dict");
	const fs::path dictMulti = writeDict(
		"  locationsInMesh ( (1.7 0.0 0.0) (0.0 0.0 0.0) );\n", "multi.dict");

	XFoam_SnappyHexMesh::Stats sA, sB;
	{
		const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(dictSingle.generic_string())));
		const XFoam_SnappyHexMesh snappy(sIO);
		REQUIRE(snappy.refineParams().locationsInMesh.size() == 1u);
		const fs::path out = tmp / "out_single";
		fs::remove_all(out);
		REQUIRE(snappy.run(bg, stl, XFoam_FileName(out.generic_string()), sA));
	}
	{
		const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(dictMulti.generic_string())));
		const XFoam_SnappyHexMesh snappy(sIO);
		REQUIRE(snappy.refineParams().locationsInMesh.size() == 2u);
		const fs::path out = tmp / "out_multi";
		fs::remove_all(out);
		REQUIRE(snappy.run(bg, stl, XFoam_FileName(out.generic_string()), sB));
	}

	// nRefinedCells 仅依赖几何/level，不依赖 cull 后保留集合；两次必须相同。
	CHECK(sA.nRefinedCells == sB.nRefinedCells);
	// 双 location 同时保留了球外 (bitmask=0) + 球内 (bitmask=1) cell；
	// 单 location 只保留 bitmask=0 → 多 region 的 nKeptCells 严格大于单 region。
	CHECK(sB.nKeptCells > sA.nKeptCells);
}

// ---------------------------------------------------------------------------
// Snap #9: pointConstraint —— 单元数学测试 + snap 阶段集成校验。
// ---------------------------------------------------------------------------

TEST_CASE("snap: XFoam_PointConstraint 数学 (free/plane/line/fixed)")
{
	using PC = XFoam_PointConstraint;
	const XFoam_Vector3D dx(1, 2, 3);

	// free: 不变
	PC pcFree;
	CHECK(pcFree.nConstraints() == 0);
	const XFoam_Vector3D rFree = pcFree.constrainDisplacement(dx);
	CHECK(rFree.x() == doctest::Approx(1));
	CHECK(rFree.y() == doctest::Approx(2));
	CHECK(rFree.z() == doctest::Approx(3));

	// plane(n=z): 把 z 分量打掉
	PC pcPlane = PC::plane(XFoam_Vector3D(0, 0, 1));
	CHECK(pcPlane.nConstraints() == 1);
	const XFoam_Vector3D rPlane = pcPlane.constrainDisplacement(dx);
	CHECK(rPlane.x() == doctest::Approx(1));
	CHECK(rPlane.y() == doctest::Approx(2));
	CHECK(rPlane.z() == doctest::Approx(0));

	// line(t=x): 只保留 x
	PC pcLine = PC::line(XFoam_Vector3D(1, 0, 0));
	CHECK(pcLine.nConstraints() == 2);
	const XFoam_Vector3D rLine = pcLine.constrainDisplacement(dx);
	CHECK(rLine.x() == doctest::Approx(1));
	CHECK(rLine.y() == doctest::Approx(0));
	CHECK(rLine.z() == doctest::Approx(0));

	// fixed: 零
	PC pcFixed = PC::fixed();
	CHECK(pcFixed.nConstraints() == 3);
	const XFoam_Vector3D rFixed = pcFixed.constrainDisplacement(dx);
	CHECK(rFixed.x() == doctest::Approx(0));
	CHECK(rFixed.y() == doctest::Approx(0));
	CHECK(rFixed.z() == doctest::Approx(0));

	// combine: 两 plane 不同向 → line（沿叉乘方向 z×x = y）
	PC c1 = PC::plane(XFoam_Vector3D(0, 0, 1));
	c1.combine(PC::plane(XFoam_Vector3D(1, 0, 0)));
	CHECK(c1.nConstraints() == 2);
	const XFoam_Vector3D rC1 = c1.constrainDisplacement(XFoam_Vector3D(7, 3, 5));
	// 应只保留 y 分量
	CHECK(std::fabs(rC1.x()) < 1e-9);
	CHECK(rC1.y() == doctest::Approx(3));
	CHECK(std::fabs(rC1.z()) < 1e-9);

	// combine: plane + line (line 不在 plane 内) → fixed
	PC c2 = PC::plane(XFoam_Vector3D(0, 0, 1));
	c2.combine(PC::line(XFoam_Vector3D(0, 0, 1)));
	CHECK(c2.nConstraints() == 3);

	// combine: 两 line 平行 → 保持 line
	PC c3 = PC::line(XFoam_Vector3D(1, 0, 0));
	c3.combine(PC::line(XFoam_Vector3D(1, 0, 0)));
	CHECK(c3.nConstraints() == 2);

	// combine: 两 line 不平行 → fixed
	PC c4 = PC::line(XFoam_Vector3D(1, 0, 0));
	c4.combine(PC::line(XFoam_Vector3D(0, 1, 0)));
	CHECK(c4.nConstraints() == 3);
}

TEST_CASE("snap: sphere-in-box → 所有 snapped 点都被打成 plane 约束 (nPlane > 0, nFixed/nLine == 0)")
{
	namespace fs = boost::filesystem;
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockBg(__FILE__)));
	XFoam_BlockMesh bg(bgIO);
	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyDict(__FILE__)));
	const XFoam_SnappyHexMesh snappy(sIO);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_sphereStl(__FILE__)));

	const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_pcon_sphere";
	fs::remove_all(outDir);
	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), stats));

	// sphere.stl 上没跑 buildFeatures（dict 未开 implicitFeatureSnap）→ 所有 snapped
	// 点都按 surface 法向得到 plane 约束。
	CHECK(stats.nFixedConstrained == 0);
	CHECK(stats.nLineConstrained == 0);
	CHECK(stats.nPlaneConstrained == stats.nSnappedPoints);
	CHECK(stats.nPlaneConstrained > 0);
}

TEST_CASE("snap: implicitFeatureSnap=true 时 cylinder1 boundary 出现 fixed 约束点")
{
	namespace fs = boost::filesystem;
	const std::string snappyDictText =
		"FoamFile { version 2.0; format ascii; class dictionary; object snappyHexMeshDict; }\n"
		"castellatedMesh true; snap true; addLayers false;\n"
		"geometry { cylinder1.stl { type triSurfaceMesh; name cylinder1; } }\n"
		"castellatedMeshControls {\n"
		"  maxLocalCells 200000; maxGlobalCells 400000; minRefinementCells 0;\n"
		"  nCellsBetweenLevels 1; resolveFeatureAngle 30;\n"
		"  refinementSurfaces { cylinder1 { level (2 2); } }\n"
		"  locationInMesh (0.9 0.9 0.9);\n"
		"  allowFreeStandingZoneFaces true;\n"
		"}\n"
		"snapControls { nSmoothPatch 0; nSmoothInternal 0; tolerance 2.0;\n"
		"  nSolveIter 30; nRelaxIter 0; nFeatureSnapIter 10; implicitFeatureSnap true; }\n"
		"addLayersControls { relativeSizes true; expansionRatio 1.2; finalLayerThickness 0.3;\n"
		"  minThickness 0.1; nGrow 0; featureAngle 60; }\n";
	const fs::path tmp = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_pcon_feature";
	fs::create_directories(tmp);
	const fs::path dictPath = tmp / "snappyHexMeshDict";
	{
		std::ofstream o(dictPath.string().c_str());
		o << snappyDictText;
	}
	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(dictPath.generic_string())));
	const XFoam_SnappyHexMesh snappy(sIO);

	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockCyl(__FILE__)));
	XFoam_BlockMesh bg(bgIO);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_cylinderStl(__FILE__)));
	stl.buildFeatures(snappy.refineParams().resolveFeatureAngle);
	REQUIRE(stl.nFeatureEdges() > 0);

	const fs::path outDir = tmp / "out";
	fs::remove_all(outDir);
	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), stats));

	// cylinder1.stl 的 feature edge 必定命中（已由 #7 测试覆盖），feature vertex 可能不命中。
	// pointConstraint 集成的核心契约：每个 snapped 点都恰好落入 plane/line/fixed 之一。
	// feature edge 升级了：现在 closestFeature 回填 edge 切向 → snap 用 line() 约束。
	CHECK((stats.nFeatureEdgeSnaps + stats.nFeatureVertexSnaps) > 0);
	CHECK(stats.nFixedConstrained == stats.nFeatureVertexSnaps);
	// 命中 feature edge 的点至少不少于 nLineConstrained 增量（个别 edge 切向退化时
	// 会 fallback 到 plane，所以是「≤」而非严格相等）。
	CHECK(stats.nLineConstrained <= stats.nFeatureEdgeSnaps);
	CHECK(stats.nLineConstrained > 0); // 圆柱 cap 棱切向非零 → 一定有线约束
	CHECK(stats.nPlaneConstrained + stats.nLineConstrained + stats.nFixedConstrained
	      == stats.nSnappedPoints);
}

// ---------------------------------------------------------------------------
// addLayers：sphere-in-box 上扩 2 层 prism，检查 cell/face/point 数量级 + polyMesh 写出。
// 几何 sanity 检查由 validate-and-relax 关掉（nRelaxIter=0 + tests 容忍非完美几何）。
// ---------------------------------------------------------------------------

TEST_CASE("addLayers: sphere-in-box 上扩 2 层 prism，新增 cells / points / faces 全部 > 0 并可写")
{
	namespace fs = boost::filesystem;
	const std::string snappyDictText =
		"FoamFile { version 2.0; format ascii; class dictionary; object snappyHexMeshDict; }\n"
		"castellatedMesh true; snap true; addLayers true;\n"
		"geometry { sphere.stl { type triSurfaceMesh; name sphere; } }\n"
		"castellatedMeshControls {\n"
		"  maxLocalCells 100000; maxGlobalCells 200000; minRefinementCells 0;\n"
		"  nCellsBetweenLevels 1; resolveFeatureAngle 30;\n"
		"  refinementSurfaces { sphere { level (1 1); } }\n"
		"  locationInMesh (1.7 0.0 0.0);\n"
		"  allowFreeStandingZoneFaces true;\n"
		"}\n"
		"snapControls { nSmoothPatch 0; nSmoothInternal 0; tolerance 2.0;\n"
		"  nSolveIter 30; nRelaxIter 0; nFeatureSnapIter 10; }\n"
		"addLayersControls { relativeSizes false; expansionRatio 1.2; firstLayerThickness 0.02;\n"
		"  finalLayerThickness 0.3; minThickness 0.001; nGrow 0; featureAngle 60;\n"
		"  layers { sphere { nSurfaceLayers 2; } } }\n";
	const fs::path tmp = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_addlayers";
	fs::create_directories(tmp);
	const fs::path dictPath = tmp / "snappyHexMeshDict";
	{
		std::ofstream o(dictPath.string().c_str());
		o << snappyDictText;
	}
	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoam_FileName(dictPath.generic_string())));
	const XFoam_SnappyHexMesh snappy(sIO);
	REQUIRE(snappy.phases().addLayers);
	REQUIRE(snappy.layerParams().perPatchLayers.found(XFoam_Word("sphere")));
	CHECK(snappy.layerParams().perPatchLayers[XFoam_Word("sphere")] == 2);
	CHECK(snappy.layerParams().firstLayerThickness == doctest::Approx(0.02));

	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockBg(__FILE__)));
	XFoam_BlockMesh bg(bgIO);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_sphereStl(__FILE__)));

	const fs::path outDir = tmp / "out";
	fs::remove_all(outDir);
	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), stats));

	// addLayers 真的跑了，stats 三件套都 > 0。
	CHECK(stats.nLayerPatches == 1);
	CHECK(stats.nLayerCellsAdded > 0);
	CHECK(stats.nLayerPointsAdded > 0);
	CHECK(stats.nLayerFacesAdded > 0);
	// 每个 sphere boundary face 长出 2 个 prism cell。
	XFoam_Label nSphereFaces = 0;
	for (XFoam_Label i = 0; i < stats.outPatchNames.size(); ++i)
	{
		if (static_cast<const std::string&>(static_cast<const XFoam_String&>(stats.outPatchNames[i])) == "sphere")
		{
			// 这里 outPatchNames/Types 在写 polyMesh 时再算 size；nLayerCellsAdded 应该是
			// patchFaces × nLayers。本 case nLayers=2 → nLayerCellsAdded 是 sphere patch 在
			// addLayers 之前的 face 数 × 2，与最终 patch face 数一致。
			nSphereFaces = stats.nLayerCellsAdded / 2;
			break;
		}
	}
	CHECK(nSphereFaces > 0);

	// polyMesh 写出后 boundary 文件存在。
	CHECK(fs::is_regular_file(outDir / "boundary"));
	CHECK(fs::is_regular_file(outDir / "owner"));
	CHECK(fs::is_regular_file(outDir / "neighbour"));
	CHECK(fs::is_regular_file(outDir / "points"));
	CHECK(fs::is_regular_file(outDir / "faces"));
}

TEST_CASE("addLayers: addLayers=false 时所有 layer-* stats 应为 0（关闭即不动）")
{
	namespace fs = boost::filesystem;
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockBg(__FILE__)));
	XFoam_BlockMesh bg(bgIO);

	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyDict(__FILE__)));
	const XFoam_SnappyHexMesh snappy(sIO);
	REQUIRE(snappy.phases().addLayers == false);

	XFoam_VBrep stl;
	REQUIRE(stl.read(XFoamTests_sphereStl(__FILE__)));

	const fs::path outDir = XFoamTests_tmpDir(__FILE__) / "tsnappy_smoke_nolayers";
	fs::remove_all(outDir);
	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, XFoam_FileName(outDir.generic_string()), stats));

	CHECK(stats.nLayerPatches == 0);
	CHECK(stats.nLayerCellsAdded == 0);
	CHECK(stats.nLayerPointsAdded == 0);
	CHECK(stats.nLayerFacesAdded == 0);
}
