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
#include "XFoam/mesh/xfoam_polymesh.h"
#include "XFoam/snap/xfoam_snappyhexmesh.h"
#include "XFoam/snap/xfoam_trisurface.h"
#include "XFoam/utilities/xfoam_dictionary.h"

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

TEST_CASE("snap: end-to-end sphere-in-box → refine + cull + snap")
{
	const XFoam_IODictionary bgIO(XFoam_systemDictIO(XFoamTests_blockBg(__FILE__)));
	XFoam_BlockMesh bg(bgIO);
	REQUIRE(bg.cells().size() == 64); // 4 x 4 x 4 background

	const XFoam_IODictionary sIO(XFoam_systemDictIO(XFoamTests_snappyDict(__FILE__)));
	const XFoam_SnappyHexMesh snappy(sIO);

	XFoam_TriSurface stl;
	REQUIRE(stl.read(XFoamTests_sphereStl(__FILE__)));

	XFoam_AutoPtr<XFoam_PolyMesh> mesh;
	XFoam_SnappyHexMesh::Stats stats;
	REQUIRE(snappy.run(bg, stl, mesh, stats));
	REQUIRE(mesh.valid());

	// 加密因子: 64 * (2^1)^3 = 512
	CHECK(stats.nBgCells == 64);
	CHECK(stats.nRefinedCells == 512);
	CHECK(stats.nKeptCells < stats.nRefinedCells); // 切掉了至少一些 cell
	CHECK(stats.nKeptCells > 0);
	CHECK(stats.nSnappedPoints > 0);

	// snap 位移上界：球面附近 cell 大小 ~0.5（box=-2..2, 8 cells/边）；
	// 移动量应 < 一个 cell 对角线长 ~0.87
	CHECK(stats.maxSnapDistance < 0.9);

	const XFoam_PolyMesh& m = *mesh;
	CHECK(m.nCells() == stats.nKeptCells);
	// 两个 patch：原 box wall + 新 STL patch
	CHECK(m.boundary().size() == 2);
	CHECK(stats.outPatchTypes.size() == 2);
}
