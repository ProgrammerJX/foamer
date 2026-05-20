// x-sample-foam-blockmesh: 命令行版 blockMesh。
//
// 仿照 OpenFOAM applications/utilities/mesh/generation/blockMesh：
//   1) 从 <case>/system/blockMeshDict 读字典（或 -dict <path> 旁路）
//   2) 用 XFoam_BlockMesh 装配 block 拓扑
//   3) 用 XFoam_PolyMesh 走 polyMeshFromShapeMesh ctor 合并内面、定 owner/neighbour
//   4) 写出 <case>/constant/polyMesh/{points,faces,owner,neighbour,boundary}
//
// 未移植：OF 的 mergePatchPairs、defaultPatch、多 region、binary 格式、
// 命令行 -region/-time 等。当前 -case 默认 .，dict 默认 <case>/system/blockMeshDict。

#include "XFoam/XFoam_API.h"
#include "XFoam/snap/xfoam_blockmesh.h"
#include "XFoam/snap/xfoam_polymesh.h"
#include "XFoam/snap/xfoam_polypatch.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_error.h"

#include <boost/filesystem.hpp>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

namespace
{
struct CliArgs
{
	std::string caseDir = ".";
	std::string dictPath;
	std::string outDir;
	bool help = false;
};

void printUsage(std::ostream& os, const char* argv0)
{
	os << "usage: " << argv0
	   << " [-case <dir>] [-dict <blockMeshDict>] [-out <polyMeshDir>]\n"
	   << "  -case  case directory (default: .)\n"
	   << "         reads <case>/system/blockMeshDict\n"
	   << "         writes <case>/constant/polyMesh/\n"
	   << "  -dict  override dict path (absolute or relative to cwd)\n"
	   << "  -out   override output polyMesh dir\n";
}

bool parseArgs(int argc, char** argv, CliArgs& a)
{
	for (int i = 1; i < argc; ++i)
	{
		const char* s = argv[i];
		if (!std::strcmp(s, "-h") || !std::strcmp(s, "--help"))
		{
			a.help = true;
			return true;
		}
		if (!std::strcmp(s, "-case") && i + 1 < argc) { a.caseDir = argv[++i]; continue; }
		if (!std::strcmp(s, "-dict") && i + 1 < argc) { a.dictPath = argv[++i]; continue; }
		if (!std::strcmp(s, "-out")  && i + 1 < argc) { a.outDir   = argv[++i]; continue; }
		std::cerr << "unknown arg: " << s << "\n";
		return false;
	}
	return true;
}

XFoam_FileName toXFoamFile(const boost::filesystem::path& p)
{
	return XFoam_FileName(p.generic_string());
}
} // namespace

int main(int argc, char** argv)
{
	namespace fs = boost::filesystem;
	CliArgs args;
	if (!parseArgs(argc, argv, args)) { printUsage(std::cerr, argv[0]); return 2; }
	if (args.help) { printUsage(std::cout, argv[0]); return 0; }

	const fs::path caseDir = fs::absolute(args.caseDir);
	const fs::path dictPath = !args.dictPath.empty()
		? fs::absolute(args.dictPath)
		: (caseDir / "system" / "blockMeshDict");
	const fs::path outDir = !args.outDir.empty()
		? fs::absolute(args.outDir)
		: (caseDir / "constant" / "polyMesh");

	if (!fs::is_regular_file(dictPath))
	{
		std::cerr << "blockMeshDict not found: " << dictPath.string() << "\n";
		return 1;
	}

	std::cout << "Reading " << dictPath.string() << "\n";
	std::cout << "Writing " << outDir.string()  << "\n";

	try
	{
		// XFoam_systemDictIO 接受 dict 文件路径，构造一个把 dict 当 system 字典看待的 IOobject。
		// 与 tests/foam-mesh/tpolymesh_assembly.cpp 走同一条加载路径，行为一致。
		const XFoam_IODictionary meshDictIO(XFoam_systemDictIO(toXFoamFile(dictPath)));
		XFoam_BlockMesh blocks(meshDictIO);

		std::cout << "BlockMesh: "
				  << blocks.size()       << " block(s), "
				  << blocks.points().size() << " mesh points, "
				  << blocks.cells().size()  << " cells, "
				  << blocks.patches().size() << " patch(es).\n";

		XFoam_PointField points(blocks.points());
		XFoam_CellShapeList cellShapes(blocks.cells());
		XFoam_FaceListList patches(blocks.patches());
		XFoam_WordList patchNames(blocks.patchNames());
		const XFoam_WordList& patchTypes = blocks.patchTypes();
		const XFoam_PtrListDictionary<XFoam_Dictionary> patchDictsEmpty;

		XFoam_PolyMesh mesh(
			XFoam_move(points),
			cellShapes,
			patches,
			XFoam_move(patchNames),
			patchDictsEmpty,
			XFoam_Word("defaultFaces"),
			XFoam_Word(XFoam_PolyPatch::typeName));

		std::cout << "PolyMesh:  "
				  << mesh.nPoints()        << " points, "
				  << mesh.nCells()         << " cells, "
				  << mesh.nFaces()         << " faces ("
				  << mesh.nInternalFaces() << " internal, "
				  << mesh.nBoundaryFaces() << " boundary).\n";

		if (!mesh.writePolyMeshDir(toXFoamFile(outDir), patchTypes))
		{
			std::cerr << "failed to write polyMesh to " << outDir.string() << "\n";
			return 1;
		}
	}
	catch (const XFoam_Error& e)
	{
		// XFoam_Error 已经把消息写到 stderr / 内部缓冲。再补一行简短反馈即可。
		std::cerr << "FATAL: blockMesh aborted (XFoam_Error: " << e.what() << ")\n";
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
