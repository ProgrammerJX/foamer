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
#include "XFoam/cmsh/xfoam_cmshmeshoptimizer.h"
#include "XFoam/cmsh/xfoam_cmshobjrefine.h"
#include "XFoam/cmsh/xfoam_cmshoctree.h"
#include "XFoam/cmsh/xfoam_cmshoctreecreator.h"
#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/cmsh/xfoam_cmshsurfaceedgeextractor.h"
#include "XFoam/cmsh/xfoam_cmshsurfacemapper.h"

#include <memory>
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
	bool         perFacePatches = false;
	bool         optimize       = false;
	int          optIter        = 3;
	double       optRelax       = 0.5;
	bool         fitFeatures    = false;
	double       fitFeatSafety  = 0.5;
	int          fitFeatBump    = 2;

	/// 命令行多次指定的 object refines
	struct BoxR    { double xmn, ymn, zmn, xmx, ymx, zmx; int level; };
	struct SphereR { double cx, cy, cz, r; int level; };
	struct ConeR   { double ax, ay, az, bx, by, bz, ra, rb; int level; };
	std::vector<BoxR>    refineBoxes;
	std::vector<SphereR> refineSpheres;
	std::vector<ConeR>   refineCones;
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
		else if (std::strcmp(arg, "-perFacePatches") == 0) { a.perFacePatches = true; }
		else if (std::strcmp(arg, "-optimize")  == 0) { a.optimize = true; }
		else if (std::strcmp(arg, "-optIter")   == 0) { if (!nexti(a.optIter))  return false; }
		else if (std::strcmp(arg, "-optRelax")  == 0) { if (!next(a.optRelax))  return false; }
		else if (std::strcmp(arg, "-fitFeatures") == 0) { a.fitFeatures = true; }
		else if (std::strcmp(arg, "-fitFeatSafety") == 0) { if (!next(a.fitFeatSafety)) return false; }
		else if (std::strcmp(arg, "-fitFeatBump")   == 0) { if (!nexti(a.fitFeatBump))  return false; }
		else if (std::strcmp(arg, "-refineBox") == 0)
		{
			Args::BoxR b;
			if (!next(b.xmn) || !next(b.ymn) || !next(b.zmn)
			    || !next(b.xmx) || !next(b.ymx) || !next(b.zmx)
			    || !nexti(b.level))
				return false;
			a.refineBoxes.push_back(b);
		}
		else if (std::strcmp(arg, "-refineSphere") == 0)
		{
			Args::SphereR s;
			if (!next(s.cx) || !next(s.cy) || !next(s.cz)
			    || !next(s.r) || !nexti(s.level))
				return false;
			a.refineSpheres.push_back(s);
		}
		else if (std::strcmp(arg, "-refineCone") == 0)
		{
			Args::ConeR c;
			if (!next(c.ax) || !next(c.ay) || !next(c.az)
			    || !next(c.bx) || !next(c.by) || !next(c.bz)
			    || !next(c.ra) || !next(c.rb) || !nexti(c.level))
				return false;
			a.refineCones.push_back(c);
		}
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
		p.fitFeatures             = A.fitFeatures;
		p.fitFeaturesSafety       = static_cast<XFoam_Scalar>(A.fitFeatSafety);
		p.fitFeaturesMaxLevelBump = A.fitFeatBump;
		XFoam_CMshOctreeCreator cr(box, p);
		cr.addSurfaceRefine(brep, A.surfLevel);
		for (const auto& b : A.refineBoxes)
		{
			auto ob = std::make_unique<XFoam_CMshBoxRefine>();
			ob->box = XFoam_BoundBox(
				XFoam_Vector3D(b.xmn, b.ymn, b.zmn),
				XFoam_Vector3D(b.xmx, b.ymx, b.zmx));
			ob->level = b.level;
			ob->name  = "box";
			cr.addObjectRefine(std::move(ob));
		}
		for (const auto& s : A.refineSpheres)
		{
			auto os = std::make_unique<XFoam_CMshSphereRefine>();
			os->centre = XFoam_Vector3D(s.cx, s.cy, s.cz);
			os->radius = s.r;
			os->level  = s.level;
			os->name   = "sphere";
			cr.addObjectRefine(std::move(os));
		}
		for (const auto& c : A.refineCones)
		{
			auto oc = std::make_unique<XFoam_CMshConeRefine>();
			oc->a       = XFoam_Vector3D(c.ax, c.ay, c.az);
			oc->b       = XFoam_Vector3D(c.bx, c.by, c.bz);
			oc->radiusA = c.ra;
			oc->radiusB = c.rb;
			oc->level   = c.level;
			oc->name    = "cone";
			cr.addObjectRefine(std::move(oc));
		}

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
			ep.perFacePatches = A.perFacePatches;
			std::vector<const XFoam_BrepBase*> brepsForExtract;
			std::vector<std::string>            brepNamesForExtract;
			if (ep.perFacePatches)
			{
				brepsForExtract.push_back(&brep);
				brepNamesForExtract.push_back("surface");
			}
			XFoam_CMshCartesianExtractor ex(oct(), ep, brepsForExtract, brepNamesForExtract);
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
				if (A.optimize)
				{
					XFoam_CMshMeshOptimizer::Params op;
					op.nIterations  = A.optIter;
					op.relaxFactor  = A.optRelax;
					op.reproject    = true;
					op.snapFeatures = A.snapFeatures;
					op.cellSizeHint = A.maxCellSize / static_cast<double>(1 << A.surfLevel);
					op.verbose      = true;
					XFoam_CMshMeshOptimizer optr(pm, breps, op);
					std::cout << "optimizing (boundary Laplacian + reproject, iter=" << op.nIterations
					          << " relax=" << op.relaxFactor << ")...\n";
					const auto t8 = std::chrono::steady_clock::now();
					const auto ost = optr.optimize();
					const auto t9 = std::chrono::steady_clock::now();
					const double mse4 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t9 - t8).count();
					std::cout << "optimizer done in " << mse4 << " ms (lastIter moved=" << ost.nMoved
					          << ", avg=" << ost.avgMove
					          << ", max=" << ost.maxMove << ")\n";
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
