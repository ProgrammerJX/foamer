// =============================================================================
// x-sample-foam-occt
//
// 命令行：
//   x-sample-foam-occt -in <step|iges>
//                      [-deflection 1e-2] [-feature 30]
//                      [-stlOut out.stl]
//                      [-case <dir>] [-snappyOut <polyMeshDir>]
//                      [-via mbrep|vbrep]
//
// 流程（三档模式）：
//
//   档 1（默认，仅 CAD 探查）：
//     readFromStep / readFromIges → 打印 verts / edges / faces / solids 计数。
//
//   档 2（追加 -stlOut）：
//     MBrep.tessellate(deflection) → toVBrep → VBrep.writeStl
//     纯 viz 用途；ParaView 直接打开。
//
//   档 3（追加 -snappyOut + -case，可选 -via）：
//     a) -via mbrep（默认推荐）：MBrep 直接喂进 XFoam_SnappyHexMesh，
//        snap 阶段 closestPointAndNormal 走 OCCT analytic（误差到数值精度），
//        feature edge 也是 OCCT TopoDS_Edge 本身，没有 tessellation 锯齿。
//        全程 STL 不落地。
//     b) -via vbrep：convertMBrepToVBrep(deflection, featureAngle) → snappy。
//        与 x-sample-foam-snappyhexmesh 走同一条 BVH 路径，便于精度对比。
//
//   档 3 需要的 dict（从 -case 解析）：
//     <case>/system/blockMeshDict
//     <case>/system/snappyHexMeshDict
//
// 仅在 -DXFOAM_WITH_OCCT=ON 时编译（见 samples/CMakeLists.txt）。
// =============================================================================

#include "XFoam/XFoam_API.h"
#include "XFoam/block/xfoam_blockmesh.h"
#include "XFoam/snap/xfoam_snappyhexmesh.h"
#include "XFoam/topo/xfoam_topo.h"
#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_error.h"

#include <boost/filesystem.hpp>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace
{

struct Args
{
	std::string  in;
	std::string  stlOut;
	std::string  caseDir;
	std::string  snappyOutDir;
	std::string  via = "mbrep"; // mbrep | vbrep
	XFoam_Scalar deflection      = 1.0e-2;
	XFoam_Scalar featureAngleDeg = 30.0;
	bool         help = false;
};

void usage(std::ostream& os, const char* a0)
{
	os << "usage: " << a0
	   << " -in <step|iges>\n"
	   << "       [-deflection 1e-2] [-feature 30]\n"
	   << "       [-stlOut out.stl]\n"
	   << "       [-case <dir> -snappyOut <polyMeshDir>] [-via mbrep|vbrep]\n";
}

bool parse(int argc, char** argv, Args& a)
{
	for (int i = 1; i < argc; ++i)
	{
		const char* s = argv[i];
		if (!std::strcmp(s, "-h") || !std::strcmp(s, "--help")) { a.help = true; return true; }
		if (!std::strcmp(s, "-in")         && i + 1 < argc) { a.in           = argv[++i]; continue; }
		if (!std::strcmp(s, "-stlOut")     && i + 1 < argc) { a.stlOut       = argv[++i]; continue; }
		if (!std::strcmp(s, "-case")       && i + 1 < argc) { a.caseDir      = argv[++i]; continue; }
		if (!std::strcmp(s, "-snappyOut")  && i + 1 < argc) { a.snappyOutDir = argv[++i]; continue; }
		if (!std::strcmp(s, "-via")        && i + 1 < argc) { a.via          = argv[++i]; continue; }
		if (!std::strcmp(s, "-deflection") && i + 1 < argc)
		{
			a.deflection = static_cast<XFoam_Scalar>(std::atof(argv[++i]));
			continue;
		}
		if (!std::strcmp(s, "-feature")    && i + 1 < argc)
		{
			a.featureAngleDeg = static_cast<XFoam_Scalar>(std::atof(argv[++i]));
			continue;
		}
		std::cerr << "unknown arg: " << s << "\n";
		return false;
	}
	return !a.in.empty();
}

bool endsWith(const std::string& s, const std::string& suf)
{
	if (s.size() < suf.size()) return false;
	for (size_t i = 0; i < suf.size(); ++i)
	{
		const char a = static_cast<char>(std::tolower(s[s.size() - suf.size() + i]));
		const char b = static_cast<char>(std::tolower(suf[i]));
		if (a != b) return false;
	}
	return true;
}

XFoam_FileName toX(const boost::filesystem::path& p)
{
	return XFoam_FileName(p.generic_string());
}

// 把当前 m 里的 brep 当成"待 snappy 的几何源"喂给 snappy.run。
// via == "mbrep"：直接拿 MBrep 当 BrepBase；snap 走 OCCT analytic。
// via == "vbrep"：先 convertMBrepToVBrep 烘成离散，按 BVH 路径走。
int runSnappyOnBrep(
	XFoam_TopoModel&             m,
	const Args&                  A,
	const boost::filesystem::path& caseDir)
{
	namespace fs = boost::filesystem;
	const fs::path blockDict  = caseDir / "system" / "blockMeshDict";
	const fs::path snappyDict = caseDir / "system" / "snappyHexMeshDict";
	if (!fs::is_regular_file(blockDict))
	{
		std::cerr << "blockMeshDict not found: " << blockDict.string() << "\n";
		return 1;
	}
	if (!fs::is_regular_file(snappyDict))
	{
		std::cerr << "snappyHexMeshDict not found: " << snappyDict.string() << "\n";
		return 1;
	}

	const XFoam_IODictionary bIO(XFoam_systemDictIO(toX(blockDict)));
	XFoam_BlockMesh bg(bIO);
	std::cout << "Background blockMesh: " << bg.cells().size() << " coarse cells\n";

	const XFoam_IODictionary sIO(XFoam_systemDictIO(toX(snappyDict)));
	XFoam_SnappyHexMesh snappy(sIO);

	const auto& specs = snappy.surfaces();
	if (specs.empty())
	{
		std::cerr << "No refinementSurfaces in snappyHexMeshDict.\n";
		return 1;
	}
	if (specs.size() != 1)
	{
		std::cerr << "this sample currently supports exactly 1 surface (got "
		          << specs.size() << "); future: split MBrep per TopoDS_Face.\n";
		return 1;
	}

	// 准备 BrepBase 指针。MBrep 路径直接拿 m.mbrep()；VBrep 路径先 convert。
	std::unique_ptr<XFoam_BrepBase> placeholder;
	const XFoam_BrepBase* surf = nullptr;
	if (A.via == "mbrep")
	{
		// 把 mbrep 当 BrepBase 用；如果 implicitFeatureSnap 打开，先在 OCCT 上算
		// feature edge 集合（dihedral 阈值用 dict 里的 resolveFeatureAngle）。
		if (snappy.snapParams().implicitFeatureSnap)
		{
			m.mbrep().buildFeatures(snappy.refineParams().resolveFeatureAngle);
		}
		surf = &m.mbrep();
		std::cout << "snap source: MBrep (OCCT analytic; "
		          << m.mbrep().nFaces() << " parametric faces, "
		          << m.mbrep().nFeatureEdges() << " feature edges, "
		          << m.mbrep().nFeatureVertices() << " feature verts)\n";
	}
	else if (A.via == "vbrep")
	{
		m.convertMBrepToVBrep(A.deflection, A.featureAngleDeg);
		if (snappy.snapParams().implicitFeatureSnap)
		{
			m.vbrep().buildFeatures(snappy.refineParams().resolveFeatureAngle);
		}
		surf = &m.vbrep();
		std::cout << "snap source: VBrep (BVH on tessellated triangles; "
		          << m.vbrep().nFaces() << " tris, "
		          << m.vbrep().nFeatureEdges() << " feature edges, "
		          << m.vbrep().nFeatureVertices() << " feature verts)\n";
	}
	else
	{
		std::cerr << "unknown -via value: '" << A.via << "' (need mbrep | vbrep)\n";
		return 1;
	}

	std::vector<const XFoam_BrepBase*> surfs(1, surf);

	XFoam_SnappyHexMesh::Stats stats;
	const fs::path outDir = fs::absolute(A.snappyOutDir);
	if (!snappy.run(bg, surfs, toX(outDir), stats))
	{
		std::cerr << "snappy.run() failed.\n";
		return 1;
	}
	std::cout << "Refined cells      : " << stats.nRefinedCells
	          << "  (max level " << stats.maxAdaptiveLevel << ")\n"
	          << "Kept cells         : " << stats.nKeptCells << "\n"
	          << "Snapped points     : " << stats.nSnappedPoints
	          << "  (max move " << stats.maxSnapDistance << ")\n";
	if (stats.nFeatureEdgeSnaps > 0 || stats.nFeatureVertexSnaps > 0)
	{
		std::cout << "Feature snaps      : "
		          << stats.nFeatureVertexSnaps << " vertex + "
		          << stats.nFeatureEdgeSnaps << " edge\n";
	}
	std::cout << "PolyMesh out       : "
	          << stats.nPoints << " pts, "
	          << stats.nKeptCells << " cells, "
	          << stats.nFaces << " faces ("
	          << stats.nInternalFaces << " int, "
	          << stats.nBoundaryFaces << " bnd)\n"
	          << "Wrote " << outDir.string() << "\n";

	// 把 case 写个最小 controlDict + .foam（ParaView 直接打开）
	const fs::path maybeCase = outDir.parent_path().parent_path();
	if (outDir.filename() == "polyMesh"
		&& outDir.parent_path().filename() == "constant")
	{
		const fs::path sysDir = maybeCase / "system";
		fs::create_directories(sysDir);
		const fs::path cd = sysDir / "controlDict";
		if (!fs::exists(cd))
		{
			std::ofstream of(cd.string());
			of << "FoamFile\n{\n    version 2.0;\n    format ascii;\n    class dictionary;\n"
			   << "    location \"system\";\n    object controlDict;\n}\n\n"
			   << "application snappyHexMesh; startFrom startTime; startTime 0;\n"
			   << "stopAt endTime; endTime 1; deltaT 1; writeControl runTime; writeInterval 1;\n";
		}
		const fs::path foamFile = maybeCase / (maybeCase.filename().string() + ".foam");
		if (!fs::exists(foamFile))
		{
			std::ofstream(foamFile.string()).close();
		}
	}
	return 0;
}

} // namespace

int main(int argc, char** argv)
{
	Args A;
	if (!parse(argc, argv, A)) { usage(std::cerr, argv[0]); return 2; }
	if (A.help)                { usage(std::cout, argv[0]); return 0; }

	try
	{
		XFoam_TopoModel m;
		m.setBrep(XFoam_AutoPtr<XFoam_BrepBase>(new XFoam_MBrep()));
		if (endsWith(A.in, ".step") || endsWith(A.in, ".stp"))
		{
			m.readFromStep(XFoam_String(A.in));
		}
		else if (endsWith(A.in, ".iges") || endsWith(A.in, ".igs"))
		{
			m.readFromIges(XFoam_String(A.in));
		}
		else
		{
			std::cerr << "unsupported CAD extension (need .step/.stp/.iges/.igs): "
			          << A.in << "\n";
			return 2;
		}
		std::cout << "CAD loaded: " << m.mbrep().nVerts() << " vert / "
		          << m.mbrep().nEdges() << " edge / "
		          << m.mbrep().nFaces() << " face / "
		          << m.mbrep().nBodies() << " solid\n";
		{
			const XFoam_BoundBox bb = m.mbrep().bounds();
			std::cout << "CAD bounds: ["
			          << bb.min().x() << "," << bb.max().x() << "] x ["
			          << bb.min().y() << "," << bb.max().y() << "] x ["
			          << bb.min().z() << "," << bb.max().z() << "]\n"
			          << "extent    : "
			          << (bb.max().x() - bb.min().x()) << " x "
			          << (bb.max().y() - bb.min().y()) << " x "
			          << (bb.max().z() - bb.min().z()) << "\n";
		}

		// 档 3：snappy 直接走 MBrep（或可选先转 VBrep）
		if (!A.snappyOutDir.empty())
		{
			if (A.caseDir.empty())
			{
				std::cerr << "-snappyOut requires -case <dir> "
				             "(need blockMeshDict + snappyHexMeshDict).\n";
				return 2;
			}
			const boost::filesystem::path caseDir
				= boost::filesystem::absolute(A.caseDir);
			const int rc = runSnappyOnBrep(m, A, caseDir);
			if (rc != 0) return rc;
		}

		// 档 2：STL 副产物（viz 用途；只在 MBrep 路径下需要时跑一次 tessellate）
		if (!A.stlOut.empty())
		{
			// 如果上面 snappy 走的是 -via vbrep，brep_ 已经被替换成 VBrep；
			// 否则 brep_ 仍是 MBrep，需要再 convertMBrepToVBrep 一次。
			if (m.brepKind() != XFoam_BrepKind::Triangulated)
			{
				m.convertMBrepToVBrep(A.deflection, A.featureAngleDeg);
			}
			const XFoam_VBrep& vb = m.vbrep();
			std::cout << "tessellated: " << vb.nVerts() << " vert / "
			          << vb.nFaces() << " tri / "
			          << vb.nPatches() << " patch\n";
			vb.writeStl(XFoam_String(A.stlOut));
			std::cout << "wrote STL: " << A.stlOut << "\n";
		}
		return 0;
	}
	catch (const XFoam_Error& e)
	{
		std::cerr << "XFoam_Error: " << e.what() << "\n";
		return 1;
	}
	catch (const std::exception& e)
	{
		std::cerr << "std::exception: " << e.what() << "\n";
		return 1;
	}
}
