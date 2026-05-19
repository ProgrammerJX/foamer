// x-sample-foam-snappyhexmesh: 命令行版 snappyHexMesh（单 region 简化实现）。
//
// 仿 OpenFOAM applications/utilities/mesh/generation/snappyHexMesh：
//   x-sample-foam-snappyhexmesh [-case <dir>]
//     [-blockMeshDict <dict>] [-snappyDict <dict>] [-stl <path>]
//     [-out <polyMeshDir>] [-level N]
//
// 输入：
//   <case>/system/blockMeshDict        (背景网格)
//   <case>/system/snappyHexMeshDict    (refine + snap 控制)
//   <case>/constant/triSurface/<name>  (STL，名字来自 dict.geometry)
// 输出：
//   <case>/constant/polyMesh/{points,faces,owner,neighbour,boundary}
//
// 单 region；不做 layer；自适应 refine 用全局 level 代替；多 STL 只用第一个。

#include "XFoam/XFoam_API.h"
#include "XFoam/block/xfoam_blockmesh.h"
#include "XFoam/snap/xfoam_snappyhexmesh.h"
#include "XFoam/snap/xfoam_trisurface.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_error.h"

#include <boost/filesystem.hpp>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

namespace
{
struct Args
{
	std::string caseDir = ".";
	std::string blockDict;
	std::string snappyDict;
	std::string stl;
	std::string outDir;
	bool help = false;
};

void usage(std::ostream& os, const char* a0)
{
	os << "usage: " << a0
	   << " [-case <dir>] [-blockMeshDict <f>] [-snappyDict <f>] [-stl <f>] [-out <dir>]\n";
}

bool parse(int argc, char** argv, Args& a)
{
	for (int i = 1; i < argc; ++i)
	{
		const char* s = argv[i];
		if (!std::strcmp(s, "-h") || !std::strcmp(s, "--help")) { a.help = true; return true; }
		if (!std::strcmp(s, "-case")          && i + 1 < argc) { a.caseDir    = argv[++i]; continue; }
		if (!std::strcmp(s, "-blockMeshDict") && i + 1 < argc) { a.blockDict  = argv[++i]; continue; }
		if (!std::strcmp(s, "-snappyDict")    && i + 1 < argc) { a.snappyDict = argv[++i]; continue; }
		if (!std::strcmp(s, "-stl")           && i + 1 < argc) { a.stl        = argv[++i]; continue; }
		if (!std::strcmp(s, "-out")           && i + 1 < argc) { a.outDir     = argv[++i]; continue; }
		std::cerr << "unknown arg: " << s << "\n";
		return false;
	}
	return true;
}

XFoam_FileName toX(const boost::filesystem::path& p) { return XFoam_FileName(p.generic_string()); }
} // namespace

int main(int argc, char** argv)
{
	namespace fs = boost::filesystem;
	Args A;
	if (!parse(argc, argv, A)) { usage(std::cerr, argv[0]); return 2; }
	if (A.help) { usage(std::cout, argv[0]); return 0; }

	const fs::path caseDir = fs::absolute(A.caseDir);
	const fs::path blockDict = !A.blockDict.empty()
		? fs::absolute(A.blockDict)
		: (caseDir / "system" / "blockMeshDict");
	const fs::path snappyDict = !A.snappyDict.empty()
		? fs::absolute(A.snappyDict)
		: (caseDir / "system" / "snappyHexMeshDict");
	const fs::path outDir = !A.outDir.empty()
		? fs::absolute(A.outDir)
		: (caseDir / "constant" / "polyMesh");

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

	std::cout << "blockMeshDict      : " << blockDict.string() << "\n"
	          << "snappyHexMeshDict  : " << snappyDict.string() << "\n";

	try
	{
		const XFoam_IODictionary bDictIO(XFoam_systemDictIO(toX(blockDict)));
		XFoam_BlockMesh bg(bDictIO);
		std::cout << "Background blockMesh: "
		          << bg.size() << " block(s), "
		          << bg.points().size() << " coarse points, "
		          << bg.cells().size() << " coarse cells.\n";

		const XFoam_IODictionary sDictIO(XFoam_systemDictIO(toX(snappyDict)));
		XFoam_SnappyHexMesh snappy(sDictIO);

		// stl 路径解析：每个 surface 一个 STL，按 dict 顺序读取。
		// CLI -stl 仅在 surface 数量为 1 时支持，作为覆盖项。
		const auto& specs = snappy.surfaces();
		if (specs.empty())
		{
			std::cerr << "No refinementSurfaces in snappyHexMeshDict.\n";
			return 1;
		}
		std::vector<XFoam_TriSurface> stlStorage(specs.size());
		std::vector<const XFoam_TriSurface*> stlPtrs(specs.size(), nullptr);
		for (size_t si = 0; si < specs.size(); ++si)
		{
			fs::path stlPath;
			if (!A.stl.empty() && specs.size() == 1)
			{
				stlPath = fs::absolute(A.stl);
			}
			else
			{
				const std::string n = static_cast<const std::string&>(
					static_cast<const XFoam_String&>(specs[si].file));
				if (n.empty())
				{
					std::cerr << "No STL file for surface '"
					          << static_cast<const std::string&>(static_cast<const XFoam_String&>(specs[si].name))
					          << "' (geometry.<name>.file missing).\n";
					return 1;
				}
				stlPath = caseDir / "constant" / "triSurface" / n;
			}
			std::cout << "STL[" << si << "] (" 
			          << static_cast<const std::string&>(static_cast<const XFoam_String&>(specs[si].name))
			          << "): " << stlPath.string() << "\n";
			if (!fs::is_regular_file(stlPath))
			{
				std::cerr << "STL not found: " << stlPath.string() << "\n";
				return 1;
			}
			if (!stlStorage[si].read(stlPath.string()))
			{
				std::cerr << "failed to read STL: " << stlPath.string() << "\n";
				return 1;
			}
			stlPtrs[si] = &stlStorage[si];
			std::cout << "       " << stlStorage[si].size() << " tris, bbox ("
			          << stlStorage[si].bounds().min().x() << ',' << stlStorage[si].bounds().min().y() << ',' << stlStorage[si].bounds().min().z() << ") .. ("
			          << stlStorage[si].bounds().max().x() << ',' << stlStorage[si].bounds().max().y() << ',' << stlStorage[si].bounds().max().z() << ")"
			          << "  level=(" << specs[si].minLevel << ' ' << specs[si].maxLevel << ")\n";
		}

		std::cout << "Refinement level   : " << snappy.globalRefinementLevel()
		          << "  (locationInMesh="
		          << snappy.refineParams().locationInMesh.x() << ','
		          << snappy.refineParams().locationInMesh.y() << ','
		          << snappy.refineParams().locationInMesh.z() << ')' << std::endl;

		XFoam_SnappyHexMesh::Stats stats;
		if (!snappy.run(bg, stlPtrs, toX(outDir), stats))
		{
			std::cerr << "snappy.run() failed.\n";
			return 1;
		}
		std::cout << "Refined cells      : " << stats.nRefinedCells
		          << "  (max level reached " << stats.maxAdaptiveLevel << ")\n";
		std::cout << "Level distribution :";
		for (int L = 0; L <= 7; ++L)
		{
			if (stats.perLevelCells[L] > 0)
			{
				std::cout << "  L" << L << "=" << stats.perLevelCells[L];
			}
		}
		std::cout << " (base cells)\n";
		std::cout << "Kept cells         : " << stats.nKeptCells << "\n"
		          << "Polyhedral cells   : " << stats.nPolyhedralCells
		          << "  (split faces=" << stats.nSplitFaces << ")\n"
		          << "Snapped points     : " << stats.nSnappedPoints
		          << "  (max move " << stats.maxSnapDistance << ")\n"
		          << "PolyMesh out       : "
		          << stats.nPoints << " pts, "
		          << stats.nKeptCells << " cells, "
		          << stats.nFaces << " faces ("
		          << stats.nInternalFaces << " int, "
		          << stats.nBoundaryFaces << " bnd)\n"
		          << "Wrote " << outDir.string() << "\n";

		// 顺便给 case 写个最小 controlDict（ParaView 通过它识别 case），
		// 只有当 outDir 看起来是 <case>/constant/polyMesh 时才写。
		const fs::path maybeCase = outDir.parent_path().parent_path();
		if (outDir.filename() == "polyMesh" && outDir.parent_path().filename() == "constant")
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
			// 同时写一个空白的 .foam 提示文件，方便 ParaView 直接打开。
			const fs::path foamFile = maybeCase / (maybeCase.filename().string() + ".foam");
			if (!fs::exists(foamFile))
			{
				std::ofstream(foamFile.string()).close();
			}
		}
	}
	catch (const XFoam_Error& e)
	{
		std::cerr << "FATAL: snappy aborted (XFoam_Error: " << e.what() << ")\n";
		return 1;
	}
	catch (const std::exception& e)
	{
		std::cerr << "FATAL: " << e.what() << "\n";
		return 1;
	}
	std::cout << "End\n";
	return 0;
}
