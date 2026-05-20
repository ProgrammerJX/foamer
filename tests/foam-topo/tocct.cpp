// =============================================================================
// foam-topo / tocct.cpp
//
// 端到端测试 OCCT 路径：
//   * BRepPrimAPI_MakeBox 自造一个 1×1×1 立方体 TopoDS_Shape
//   * STEPControl_Writer 落盘到 %TEMP% 下
//   * XFoam_MBrep::readFromStep 回读 → 验证 verts/edges/faces/bodies 数量
//   * tessellate() → meshedTris 非空 + 总三角形数 ≥ 12（box 至少 12 tri）
//   * toVBrep() → VBrep 全局顶点数 / 面数对得上
//   * TopoModel::convertMBrepToVBrep + exportToSnappy → ExportedSurface
//
// 本 group 只在 XFOAM_WITH_OCCT=ON 时被编译（见 tests/CMakeLists.txt），所以
// 这里可以放心 include OCCT 头。
// =============================================================================

#include "doctest/doctest.h"

#include "XFoam/topo/xfoam_topo.h"

#include <cmath>

#include <BRepPrimAPI_MakeBox.hxx>
#include <STEPControl_Writer.hxx>
#include <IGESControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <TopoDS_Shape.hxx>

#include <boost/filesystem.hpp>

namespace
{

/// 用 BRepPrimAPI_MakeBox 造一个 box，写到 stepPath。返回 box 的总 face 数。
/// 立方体的拓扑：8 vertex / 12 edge / 6 face / 1 solid（实际 OCCT 给 24 edge
/// 因为每条 wire 都自带一份 edge 实例，会与共享 edge 去重后变 12）。
int writeBoxStep(const std::string& stepPath, double sx, double sy, double sz)
{
	BRepPrimAPI_MakeBox mk(sx, sy, sz);
	const TopoDS_Shape s = mk.Shape();
	REQUIRE(!s.IsNull());

	// STEP 写出（AP214）
	Interface_Static::SetCVal("write.step.schema", "AP214IS");
	STEPControl_Writer w;
	const IFSelect_ReturnStatus rs = w.Transfer(s, STEPControl_AsIs);
	REQUIRE(rs == IFSelect_RetDone);
	const IFSelect_ReturnStatus ws = w.Write(stepPath.c_str());
	REQUIRE(ws == IFSelect_RetDone);
	return 6;
}

boost::filesystem::path tempPath(const std::string& name)
{
	auto p = boost::filesystem::temp_directory_path() / "xfoam_occt_tests";
	boost::filesystem::create_directories(p);
	return p / name;
}

} // anonymous

TEST_CASE("XFoam_MBrep::readFromStep / box.stp → 8 vert, 12 edge, 6 face, 1 solid")
{
	const auto stepPath = tempPath("box.stp");
	writeBoxStep(stepPath.string(), 1.0, 2.0, 3.0);

	XFoam_TopoModel m;
	m.setBrep(XFoam_AutoPtr<XFoam_BrepBase>(new XFoam_MBrep()));
	REQUIRE(m.brepKind() == XFoam_BrepKind::Parametric);
	REQUIRE_NOTHROW(m.readFromStep(XFoam_String(stepPath.string())));

	const XFoam_MBrep& br = m.mbrep();
	CHECK(br.nVerts()  == 8);
	CHECK(br.nEdges()  == 12);
	CHECK(br.nFaces()  == 6);
	CHECK(br.nBodies() == 1);

	const XFoam_BoundBox bb = br.bounds();
	// Bnd_Box 自带 ~1e-7 gap（取决于 shape tolerance），不是严格紧 bbox；
	// 0 附近用 absolute 容差，正值用 doctest::Approx 默认相对容差。
	CHECK(std::abs(bb.min().x()) < 1e-5);
	CHECK(std::abs(bb.min().y()) < 1e-5);
	CHECK(std::abs(bb.min().z()) < 1e-5);
	CHECK(bb.max().x() == doctest::Approx(1.0));
	CHECK(bb.max().y() == doctest::Approx(2.0));
	CHECK(bb.max().z() == doctest::Approx(3.0));
}

TEST_CASE("XFoam_MBrep::tessellate / box.stp → 每张 face 至少 2 个三角形")
{
	const auto stepPath = tempPath("box.stp");
	writeBoxStep(stepPath.string(), 1.0, 1.0, 1.0);

	XFoam_TopoModel m;
	m.setBrep(XFoam_AutoPtr<XFoam_BrepBase>(new XFoam_MBrep()));
	m.readFromStep(XFoam_String(stepPath.string()));
	REQUIRE_NOTHROW(m.mbrep().tessellate(1.0e-2));

	const XFoam_MBrep& br = m.mbrep();
	XFoam_Label nTriTotal = 0;
	for (XFoam_Label i = 0; i < br.nFaces(); ++i)
	{
		const auto& f = br.faces()[i];
		// box 每张面至少 2 tri（4 顶点对角切一次）
		CHECK(f.meshedPts.size()  >= 4u);
		CHECK(f.meshedTris.size() >= 2u);
		nTriTotal += static_cast<XFoam_Label>(f.meshedTris.size());
	}
	CHECK(nTriTotal >= 12);
}

TEST_CASE("XFoam_MBrep::toVBrep / VBrep patchId == ParametricFace id")
{
	const auto stepPath = tempPath("box.stp");
	writeBoxStep(stepPath.string(), 1.0, 1.0, 1.0);

	XFoam_TopoModel m;
	m.setBrep(XFoam_AutoPtr<XFoam_BrepBase>(new XFoam_MBrep()));
	m.readFromStep(XFoam_String(stepPath.string()));
	m.mbrep().tessellate(1.0e-2);

	XFoam_AutoPtr<XFoam_VBrep> vbAuto = m.mbrep().toVBrep();
	const XFoam_VBrep& vb = vbAuto();
	CHECK(vb.nFaces() >= 12);
	CHECK(vb.nVerts() >= 24); // 6 face × 4 vert（不共享）= 24
	CHECK(vb.nPatches() == 6);

	// 每个 face 的 patchId 应在 [0,6)
	for (XFoam_Label i = 0; i < vb.nFaces(); ++i)
	{
		const auto& f = vb.faces()[i];
		CHECK(f.patchId >= 0);
		CHECK(f.patchId < 6);
	}
}

TEST_CASE("XFoam_TopoModel::convertMBrepToVBrep + exportToSnappy / 一步到位走 snappy")
{
	const auto stepPath = tempPath("box.stp");
	writeBoxStep(stepPath.string(), 1.0, 1.0, 1.0);

	XFoam_TopoModel m;
	m.setBrep(XFoam_AutoPtr<XFoam_BrepBase>(new XFoam_MBrep()));
	m.readFromStep(XFoam_String(stepPath.string()));

	REQUIRE_NOTHROW(m.convertMBrepToVBrep(/*deflection*/ 1.0e-2,
	                                      /*featureAngleDeg*/ 30.0));
	CHECK(m.brepKind() == XFoam_BrepKind::Triangulated);

	const auto surfs = m.exportToSnappy(/*deflection*/ 1.0e-2);
	// rebuildIdentityFromBrep 会按 patchId 分组 → box 6 个 patch → 6 个 surf
	CHECK(surfs.size() == 6u);
	XFoam_Label nTris = 0;
	for (const auto& s : surfs)
	{
		CHECK(!s.triIdx.empty());
		nTris += static_cast<XFoam_Label>(s.triIdx.size());
	}
	CHECK(nTris >= 12);
}

TEST_CASE("XFoam_MBrep::readFromIges / box.igs round-trip")
{
	const auto igsPath = tempPath("box.igs");
	{
		BRepPrimAPI_MakeBox mk(2.0, 1.0, 1.0);
		IGESControl_Writer w;
		REQUIRE(w.AddShape(mk.Shape()));
		w.ComputeModel();
		REQUIRE(w.Write(igsPath.string().c_str()));
	}

	XFoam_TopoModel m;
	m.setBrep(XFoam_AutoPtr<XFoam_BrepBase>(new XFoam_MBrep()));
	REQUIRE_NOTHROW(m.readFromIges(XFoam_String(igsPath.string())));
	// IGES 写 box 出来一般是 6 个独立 face（NURBS surfaces）没有 solid。
	// 我们只验证 face 数 ≥ 6，因为不同 OCCT 版本/选项可能把每张 face 拆/合。
	CHECK(m.mbrep().nFaces() >= 6);
}
