// cmsh MVP smoke：
//   1) 从 .stl / .step 读 BrepBase（VBrep 走 STL，MBrep 走 STEP，OCCT 启用时
//      STEP 自动走 MBrep；否则只支持 STL）
//   2) 建 XFoam_CMshOctreeCreator，设 maxCellSize 与 per-surface level
//   3) build() → 报告 leaf 数 / per-level 分布 / inside-outside-data 分类
//
// usage:
//   x-sample-foam-cmsh -in <stl|step> [-maxCellSize 0.5] [-surfLevel 3]
//                      [-featureAngle 30] [-deflection 1e-2]

#include "XFoam/cmsh/xfoam_cmshcartesianextractor.h"
#include "XFoam/cmsh/xfoam_cmshoctree.h"
#include "XFoam/cmsh/xfoam_cmshoctreecreator.h"
#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/cmsh/xfoam_cmshsurfaceedgeextractor.h"
#include "XFoam/cmsh/xfoam_cmshsurfacemapper.h"
#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/topo/xfoam_mbrep.h"
#include "XFoam/topo/xfoam_topo.h"
#include "XFoam/topo/xfoam_vbrep.h"
#include "XFoam/utilities/xfoam_boundbox.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

namespace
{
struct Args
{
	std::string  in;
	double       maxCellSize  = 0.5;
	int          surfLevel    = 3;
	double       featureAngle = 30;
	double       deflection   = 1e-2;
	std::string  polyMeshOut; ///< 若非空，写出 polyMesh 5 件套到此目录
	bool         mapSurface = false;
	int          mapIter    = 1;
	double       mapRelax   = 1.0;
	bool         snapFeatures = false;
	double       featureSearchRadius = 0; ///< 0 = auto
};

bool parse(int argc, char** argv, Args& a)
{
	for (int i = 1; i < argc; ++i)
	{
		const char* arg = argv[i];
		auto next = [&](double& d) {
			if (i + 1 >= argc) return false;
			d = std::atof(argv[++i]);
			return true;
		};
		auto nexti = [&](int& v) {
			if (i + 1 >= argc) return false;
			v = std::atoi(argv[++i]);
			return true;
		};
		if (std::strcmp(arg, "-in") == 0)
		{
			if (i + 1 >= argc) return false;
			a.in = argv[++i];
		}
		else if (std::strcmp(arg, "-maxCellSize") == 0) { if (!next(a.maxCellSize)) return false; }
		else if (std::strcmp(arg, "-surfLevel")   == 0) { if (!nexti(a.surfLevel))   return false; }
		else if (std::strcmp(arg, "-featureAngle")== 0) { if (!next(a.featureAngle)) return false; }
		else if (std::strcmp(arg, "-deflection")  == 0) { if (!next(a.deflection))   return false; }
		else if (std::strcmp(arg, "-polyMeshOut") == 0)
		{
			if (i + 1 >= argc) return false;
			a.polyMeshOut = argv[++i];
		}
		else if (std::strcmp(arg, "-mapSurface") == 0) { a.mapSurface = true; }
		else if (std::strcmp(arg, "-mapIter")    == 0) { if (!nexti(a.mapIter))  return false; }
		else if (std::strcmp(arg, "-mapRelax")   == 0) { if (!next(a.mapRelax))  return false; }
		else if (std::strcmp(arg, "-snapFeatures") == 0) { a.snapFeatures = true; }
		else if (std::strcmp(arg, "-featureSearchRadius") == 0) { if (!next(a.featureSearchRadius)) return false; }
		else
		{
			std::cerr << "unknown arg: " << arg << "\n";
			return false;
		}
	}
	if (a.in.empty()) return false;
	return true;
}

std::string lower(std::string s)
{
	std::transform(s.begin(), s.end(), s.begin(),
	               [](unsigned char c) { return std::tolower(c); });
	return s;
}
} // namespace

int main(int argc, char** argv)
{
	Args A;
	if (!parse(argc, argv, A))
	{
		std::cerr <<
			"usage: x-sample-foam-cmsh -in <stl|step> [-maxCellSize 0.5]\n"
			"       [-surfLevel 3] [-featureAngle 30] [-deflection 1e-2]\n";
		return 1;
	}

	try
	{
		XFoam_TopoModel m;
		const std::string ext = lower(
			A.in.size() >= 4 ? A.in.substr(A.in.size() - 4) : A.in);
		if (ext == ".stl")
		{
			m.readFromStl(XFoam_String(A.in));
		}
		else if (ext == ".stp" || ext == "step" || ext == ".igs" || ext == "iges")
		{
#ifdef XFOAM_WITH_OCCT
			if (ext == ".stp" || ext == "step")
				m.readFromStep(XFoam_String(A.in));
			else
				m.readFromIges(XFoam_String(A.in));
#else
			std::cerr << "OCCT disabled at build time; cannot read " << A.in << "\n";
			return 1;
#endif
		}
		else
		{
			std::cerr << "unsupported extension: " << ext << "\n";
			return 1;
		}

		// 用 BrepBase 接口取 surface
		XFoam_BrepBase& brep = m.brep();
		if (brep.empty())
		{
			std::cerr << "loaded brep is empty.\n";
			return 1;
		}
		const XFoam_BoundBox box = brep.bounds();
		const XFoam_Vector3D sp = box.span();
		std::cout << "brep loaded:\n"
		          << "  bounds   : [" << box.min().x() << "," << box.max().x() << "] x ["
		                                << box.min().y() << "," << box.max().y() << "] x ["
		                                << box.min().z() << "," << box.max().z() << "]\n"
		          << "  extent   : " << sp.x() << " x " << sp.y() << " x " << sp.z() << "\n";

		brep.buildFeatures(static_cast<XFoam_Scalar>(A.featureAngle));
		std::cout << "  features : " << brep.nFeatureEdges() << " edges, "
		          << brep.nFeatureVertices() << " verts (angle=" << A.featureAngle << ")\n";

		XFoam_CMshOctreeCreator::Params p;
		p.maxCellSize = static_cast<XFoam_Scalar>(A.maxCellSize);
		p.maxLevel    = A.surfLevel + 4;
		XFoam_CMshOctreeCreator cr(box, p);
		cr.addSurfaceRefine(brep, A.surfLevel);

		std::cout << "building octree (maxCellSize=" << A.maxCellSize
		          << ", surfLevel=" << A.surfLevel << ")...\n" << std::flush;
		const auto t0 = std::chrono::steady_clock::now();
		auto oct = cr.build();
		const auto t1 = std::chrono::steady_clock::now();
		const double ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
		std::cout << "build done in " << ms << " ms\n";

		std::vector<XFoam_Label> perLevel;
		oct().countLeavesByLevel(perLevel);
		std::cout << "leaves: " << oct().nLeaves() << "\n";
		for (std::size_t lv = 0; lv < perLevel.size(); ++lv)
		{
			std::cout << "  level " << lv << ": " << perLevel[lv] << " leaves\n";
		}

		XFoam_Label nU, nO, nD, nI;
		oct().countLeavesByType(nU, nO, nD, nI);
		std::cout << "type:   "
		          << "Unknown=" << nU
		          << "  Outside=" << nO
		          << "  Data="    << nD
		          << "  Inside="  << nI << "\n";

		if (!A.polyMeshOut.empty())
		{
			XFoam_CMshCartesianExtractor::Params ep;
			ep.keepInside = true;
			ep.keepData   = true;
			ep.keepOutside = false;
			XFoam_CMshCartesianExtractor ex(oct(), ep);
			XFoam_CMshPolyMeshGen pm;
			std::cout << "extracting polyMesh (balance21 + face dedup)..." << std::endl;
			const auto t2 = std::chrono::steady_clock::now();
			if (!ex.extract(pm))
			{
				std::cerr << "cmsh extractor: empty mesh (no in-mesh leaves).\n";
				return 1;
			}
			const auto t3 = std::chrono::steady_clock::now();
			const double mse = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t3 - t2).count();
			std::cout << "extract done in " << mse << " ms\n"
			          << "polyMesh: pts=" << pm.points.size()
			          << "  faces=" << pm.nFaces()
			          << " (int " << pm.nInternalFaces()
			          << " / bnd " << pm.nBoundaryFaces() << ")"
			          << "  cells=" << pm.nCells << "\n";

			if (A.mapSurface || A.snapFeatures)
			{
				std::vector<const XFoam_BrepBase*> breps = {&brep};
				XFoam_CMshSurfaceMapper::Params mp;
				mp.nIterations = A.mapIter;
				mp.relaxFactor = A.mapRelax;
				mp.verbose     = true;
				XFoam_CMshSurfaceMapper mapper(pm, breps, mp);
				if (A.mapSurface)
				{
					std::cout << "mapping boundary points to surface ("
					          << mapper.boundaryPointIds().size() << " pts, "
					          << mp.nIterations << " iter, relax=" << mp.relaxFactor << ")...\n";
					const auto t4 = std::chrono::steady_clock::now();
					const XFoam_Label moved = mapper.mapToSurface();
					const auto t5 = std::chrono::steady_clock::now();
					const double mse2 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t5 - t4).count();
					std::cout << "mapper done in " << mse2 << " ms, moved " << moved << " pts\n";
				}
				if (A.snapFeatures)
				{
					XFoam_CMshSurfaceEdgeExtractor::Params ep2;
					ep2.searchRadius = A.featureSearchRadius;
					ep2.cellSizeHint = A.maxCellSize / static_cast<double>(1 << A.surfLevel);
					ep2.verbose      = true;
					XFoam_CMshSurfaceEdgeExtractor edx(pm, breps, mapper.boundaryPointIds(), ep2);
					std::cout << "snapping features (R=" << (ep2.searchRadius > 0 ? ep2.searchRadius : ep2.cellSizeHint * 0.5)
					          << ", cellHint=" << ep2.cellSizeHint << ")...\n";
					const auto t6 = std::chrono::steady_clock::now();
					const auto st = edx.snap();
					const auto t7 = std::chrono::steady_clock::now();
					const double mse3 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t7 - t6).count();
					std::cout << "edge extractor done in " << mse3 << " ms (corners=" << st.nCornerSnap
					          << ", edges=" << st.nEdgeSnap
					          << ", maxSnapDist=" << st.maxSnapDist << ")\n";
				}
			}

			if (!pm.writeToDir(XFoam_String(A.polyMeshOut)))
			{
				std::cerr << "writeToDir failed: " << A.polyMeshOut << "\n";
				return 1;
			}
			std::cout << "wrote " << A.polyMeshOut << "\n";
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
