#include "doctest/doctest.h"
#include "XFoam/topo/xfoam_topo.h"
#include "XFoam/utilities/xfoam_common.h"
#include <boost/filesystem.hpp>
#include <cstdio>
#include <fstream>

namespace
{

// 写一个最小可用的 Nastran BDF 到指定路径。8 字段 Nastran short format：每个字段恰好
// 8 字符宽（GRID 卡 6 字段 = 48 char；CTRIA3 6 字段 = 48 char；CQUAD4 7 字段 = 56 char）。
// 内容：
//   * 8 个 GRID（单位立方体角点）
//   * 5 张 CQUAD4 + 2 张 CTRIA3（顶面 z=1 用两个 CTRIA3 拆，测试三角化路径）
//   * 3 个 $HMMOVE 块：comp 100 = inlet (1 quad)，comp 200 = outlet (2 tri)，
//     comp 300 = walls (4 quad)
// 这样可在不依赖任何外部 fixture 的前提下覆盖 readFromBdf / rebuildIdentityFromBrep /
// mergeFaces / exportToSnappy 主路径。
struct TestBdfFile
{
	boost::filesystem::path path;

	static void writeGrid(std::ofstream& ofs, int id, double x, double y, double z)
	{
		char buf[80];
		std::snprintf(buf, sizeof(buf),
			"%-8s%-8d%-8s%-8s%-8s%-8s\n",
			"GRID", id, "0",
			toFixed(x).c_str(), toFixed(y).c_str(), toFixed(z).c_str());
		ofs << buf;
	}
	static void writeTria(std::ofstream& ofs, int id, int pid, int n1, int n2, int n3)
	{
		char buf[80];
		std::snprintf(buf, sizeof(buf),
			"%-8s%-8d%-8d%-8d%-8d%-8d\n",
			"CTRIA3", id, pid, n1, n2, n3);
		ofs << buf;
	}
	static void writeQuad(std::ofstream& ofs, int id, int pid,
	                      int n1, int n2, int n3, int n4)
	{
		char buf[80];
		std::snprintf(buf, sizeof(buf),
			"%-8s%-8d%-8d%-8d%-8d%-8d%-8d\n",
			"CQUAD4", id, pid, n1, n2, n3, n4);
		ofs << buf;
	}
	static void writeHmMoveHeader(std::ofstream& ofs, int compId)
	{
		char buf[80];
		std::snprintf(buf, sizeof(buf), "%-8s%-8d\n", "$HMMOVE", compId);
		ofs << buf;
	}
	/// 连续 elem id 行。XFoam 的 readHmMoveBlock 要求续行以 "$ " 开头（dollar +
	/// space/tab），后续每个 8-char 字段当作一个 elemId（或 "THRU"）。
	static void writeHmMoveIds(std::ofstream& ofs, const std::vector<int>& ids)
	{
		std::string line = "$       "; // 8 chars: '$' + 7 spaces
		for (int eid : ids)
		{
			char field[16];
			std::snprintf(field, sizeof(field), "%-8d", eid);
			line += field;
		}
		ofs << line << '\n';
	}
	static std::string toFixed(double v)
	{
		// 保留足够精度，长度 ≤ 8 chars 字段宽。"0.0" 之类用 %-.4g 较稳。
		char buf[16];
		std::snprintf(buf, sizeof(buf), "%.4g", v);
		return std::string(buf);
	}

	explicit TestBdfFile(const char* tag)
	{
		path = boost::filesystem::temp_directory_path()
			/ boost::filesystem::unique_path(std::string("xfoam_test_") + tag + "_%%%%%%%%.bdf");
		std::ofstream ofs(path.string());
		REQUIRE(ofs);
		ofs << "BEGIN BULK\n";
		writeGrid(ofs, 1, 0.0, 0.0, 0.0);
		writeGrid(ofs, 2, 1.0, 0.0, 0.0);
		writeGrid(ofs, 3, 1.0, 1.0, 0.0);
		writeGrid(ofs, 4, 0.0, 1.0, 0.0);
		writeGrid(ofs, 5, 0.0, 0.0, 1.0);
		writeGrid(ofs, 6, 1.0, 0.0, 1.0);
		writeGrid(ofs, 7, 1.0, 1.0, 1.0);
		writeGrid(ofs, 8, 0.0, 1.0, 1.0);
		// inlet (z=0)
		writeQuad(ofs, 1, 1, 1, 2, 3, 4);
		// outlet (z=1) 拆 2 tri
		writeTria(ofs, 2, 1, 5, 6, 7);
		writeTria(ofs, 3, 1, 5, 7, 8);
		// walls 4 quad
		writeQuad(ofs, 4, 1, 1, 4, 8, 5);
		writeQuad(ofs, 5, 1, 2, 6, 7, 3);
		writeQuad(ofs, 6, 1, 1, 5, 6, 2);
		writeQuad(ofs, 7, 1, 4, 3, 7, 8);
		writeHmMoveHeader(ofs, 100); writeHmMoveIds(ofs, {1});
		writeHmMoveHeader(ofs, 200); writeHmMoveIds(ofs, {2, 3});
		writeHmMoveHeader(ofs, 300); writeHmMoveIds(ofs, {4, 5, 6, 7});
		ofs << "ENDDATA\n";
	}

	~TestBdfFile()
	{
		boost::system::error_code ec;
		boost::filesystem::remove(path, ec);
	}

	std::string str() const { return path.string(); }
};

} // namespace

TEST_CASE("foam-mesh smoke")
{
	XFoam_TopoModel model;
	// 默认底层 brep = 空 VBrep；bounds() 走 VBrep::bounds() → invertedBox
	// （volume 退化为负；这里只要求"非崩溃"）。
	CHECK(model.brepKind() == XFoam_BrepKind::Triangulated);
	CHECK_NOTHROW(model.bounds());
	CHECK_NOTHROW(model.brep().clear());

	XFoam_TopoVert v(&model);
	CHECK(v.model() == &model);

	// 默认是 VBrep；mbrep() 在 VBrep 模式下应当抛 XFoam_Error。
	CHECK_THROWS_AS(model.mbrep(), const XFoam_Error&);
	CHECK_NOTHROW(model.vbrep().clear());
}

TEST_CASE("XFoam_TopoModel readFromBdf cube.bdf → VBrep（三角化 + HMMOVE patch）")
{
	TestBdfFile fx("read");
	XFoam_TopoModel model;
	CHECK_NOTHROW(model.readFromBdf(fx.str()));

	// 现在 BDF → VBrep；mbrep() 应当抛
	CHECK(model.brepKind() == XFoam_BrepKind::Triangulated);
	CHECK_THROWS_AS(model.mbrep(), const XFoam_Error&);

	const XFoam_VBrep& br = model.vbrep();
	const XFoam_BoundBox bb = model.bounds();
	CHECK(bb.min().x() < bb.max().x());
	CHECK(bb.volume() > 0.0);
	CHECK(bb.contains(br.positions()));
	CHECK(br.bounds() == bb);
	CHECK(br.nVerts() == 8);

	// 5 个 CQUAD4 → 10 个 tri；2 个 CTRIA3 → 2 个 tri；共 12 个 tri。
	CHECK(br.nFaces() == 12);

	// 3 个 HMMOVE → 3 个 patch。
	CHECK(br.nPatches() == 3);
	for (XFoam_Label fi = 0; fi < br.nFaces(); ++fi)
	{
		const auto& f = br.faces()[fi];
		for (int k = 0; k < 3; ++k)
		{
			CHECK((f.verts[k] >= 0 && f.verts[k] < br.nVerts()));
		}
		CHECK(f.patchId >= 0);
		CHECK(f.patchId < br.nPatches());
	}
}

TEST_CASE("XFoam_TopoModel readFromBdf missing file throws")
{
	XFoam_TopoModel model;
	CHECK_THROWS_AS(model.readFromBdf("___nonexistent_bdf_file___.bdf"),
	                const XFoam_Error&);
}

TEST_CASE("XFoam_TopoModel rebuildIdentityFromBrep 建 identity 虚拓扑")
{
	TestBdfFile fx("identity");
	XFoam_TopoModel model;
	model.readFromBdf(fx.str());

	model.rebuildIdentityFromBrep(30.0);

	// 3 个 patch → 3 个 TopoFace
	CHECK(model.nFaces() == 3);
	for (XFoam_Label fi = 0; fi < model.nFaces(); ++fi)
	{
		const auto& tf = model.face(fi);
		CHECK_FALSE(tf.suppressed());
		CHECK(tf.nRefs() > 0);
		const std::string nm = static_cast<const std::string&>(
			static_cast<const XFoam_String&>(tf.name()));
		CHECK(nm.find("comp_") == 0); // readFromBdf 设的 "comp_100" 等
	}

	// 默认 body 把所有 TopoFace 都收编。
	CHECK(model.nBodies() == 1);
	CHECK(model.body(0).faceIds().size() == model.nFaces());

	// 立方体每条棱都是 90° feature edge。每张 face 4 条边、6 张 face → 12 条
	// 立方体棱（共享后）。三角化把对角线也加进 edges_，对角线两侧是同 face 的
	// 两个 tri（共面）→ 不是 feature；所以 feature edge 数 = 12。
	CHECK(model.nEdges() == 12);

	// 立方体有 8 个 corner，每个 corner 被 3 条 feature edge 入射 → 都是 feature
	// vertex。
	CHECK(model.nVerts() == 8);
}

TEST_CASE("XFoam_TopoModel mergeFaces 保留 id 但 src.suppressed=true")
{
	TestBdfFile fx("merge");
	XFoam_TopoModel model;
	model.readFromBdf(fx.str());
	model.rebuildIdentityFromBrep(30.0);

	REQUIRE(model.nFaces() == 3);
	const XFoam_Label nRefsDstBefore = model.face(0).nRefs();
	const XFoam_Label nRefsSrcBefore = model.face(1).nRefs();

	XFoam_LabelList src;
	src.append(1);
	model.mergeFaces(src, 0);

	// dst 收纳了 src 的全部 refs
	CHECK(model.face(0).nRefs() == nRefsDstBefore + nRefsSrcBefore);
	// src 仍然存在 id，但 suppressed + refs 已清空
	CHECK(model.face(1).suppressed());
	CHECK(model.face(1).nRefs() == 0);
	// 其他 face 不受影响
	CHECK_FALSE(model.face(2).suppressed());
}

TEST_CASE("XFoam_TopoModel exportToSnappy 只导出非 suppressed TopoFace")
{
	TestBdfFile fx("export");
	XFoam_TopoModel model;
	model.readFromBdf(fx.str());
	model.rebuildIdentityFromBrep(30.0);

	auto exp1 = model.exportToSnappy(0.0);
	CHECK(exp1.size() == 3);
	XFoam_Label totalTris = 0;
	for (const auto& es : exp1) totalTris += static_cast<XFoam_Label>(es.triIdx.size());
	CHECK(totalTris == model.vbrep().nFaces());

	// 把 face 1 merge 进 0：导出后只剩 2 个 patch，总 tri 数不变。
	XFoam_LabelList src;
	src.append(1);
	model.mergeFaces(src, 0);
	auto exp2 = model.exportToSnappy(0.0);
	CHECK(exp2.size() == 2);
	XFoam_Label totalTris2 = 0;
	for (const auto& es : exp2) totalTris2 += static_cast<XFoam_Label>(es.triIdx.size());
	CHECK(totalTris2 == model.vbrep().nFaces());
}

TEST_CASE("XFoam_TopoModel suppressFacesSmallerThan 标 suppress + 不导出")
{
	TestBdfFile fx("suppress");
	XFoam_TopoModel model;
	model.readFromBdf(fx.str());
	model.rebuildIdentityFromBrep(30.0);
	REQUIRE(model.nFaces() == 3);

	// 立方体每张面面积 = 1.0。outlet 是两个 0.5 tri 合成的 1.0；inlet 同。
	// 把阈值定 0.5：所有 TopoFace area >= 1.0，没有被抑制。
	const XFoam_Label nSup1 = model.suppressFacesSmallerThan(0.5);
	CHECK(nSup1 == 0);
	auto exp1 = model.exportToSnappy(0.0);
	CHECK(exp1.size() == 3);

	// 把阈值定 100：所有 TopoFace area < 100，全部 suppress；export 空。
	const XFoam_Label nSup2 = model.suppressFacesSmallerThan(100.0);
	CHECK(nSup2 == 3);
	auto exp2 = model.exportToSnappy(0.0);
	CHECK(exp2.size() == 0);
}
