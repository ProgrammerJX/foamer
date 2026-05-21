// cmsh MVP smoke：
//   1) 从 .stl / .step 读 BrepBase（VBrep 走 STL，MBrep 走 STEP，OCCT 启用时
//      STEP 自动走 MBrep；否则只支持 STL）
//   2) 建 XFoam_CMshOctreeCreator，设 maxCellSize 与 per-surface level
//   3) build() → 报告 leaf 数 / per-level 分布 / inside-outside-data 分类
//
// usage:
//   x-sample-foam-cmsh -in <stl|step> [-maxCellSize 0.5] [-surfLevel 3]
//                      [-featureAngle 30] [-deflection 1e-2]

#include "XFoam/cmsh/xfoam_cmshobjrefine.h"
#include "XFoam/cmsh/xfoam_cmshpipeline.h"
#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"

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
	int          maxLevel     = 14;
	double       featureAngle = 30;
	double       deflection   = 1e-2;
	std::string  polyMeshOut; ///< 若非空，写出 polyMesh 5 件套到此目录
	bool         mapSurface = false;
	int          mapIter    = 1;
	double       mapRelax   = 1.0;
	bool         snapFeatures = false;
	double       featureSearchRadius = 0; ///< 0 = auto
	bool         perFacePatches = false;
	bool         fillAllSubPatches = true;  ///< 默认开：保所有 TopoDS_Face 在 boundary 列表中
	bool         noRepatch      = false;
	bool         optimize       = false;
	int          optIter        = 3;
	double       optRelax       = 0.5;
	bool         optQuality     = false;
	double       optMinDot      = 0.5;
	double       optMinAreaR    = 0.1;
	bool         fitFeatures    = false;
	double       fitFeatSafety  = 0.5;
	int          fitFeatBump    = 2;
	bool         perFaceFit     = false;
	double       perFaceFitSafety = 0.5;
	bool         localFeatRef   = false;
	double       localFeatSafety = 0.5;
	double       localFeatSearchMul = 2.0;
	bool         coverAllFaces  = false;
	int          coverRounds    = 3;
	bool         pinFeatures    = false;
	double       pinRadius      = 0;
	bool         edgeInsert     = false;
	double       edgeInsertR    = 0;
	bool         untangle       = false;
	int          untangleIter   = 3;
	std::vector<std::pair<std::string, int>> patchRefine; ///< "name=level,..."
	bool         useLocationInMesh = false;
	double       lim[3] = {0, 0, 0};

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
		else if (std::strcmp(arg, "-maxLevel")    == 0) { if (!nexti(a.maxLevel))    return false; }
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
		else if (std::strcmp(arg, "-fillAllSubPatches")   == 0) { a.fillAllSubPatches = true; }
		else if (std::strcmp(arg, "-noFillAllSubPatches") == 0) { a.fillAllSubPatches = false; }
		else if (std::strcmp(arg, "-noRepatch") == 0) { a.noRepatch = true; }
		else if (std::strcmp(arg, "-optimize")  == 0) { a.optimize = true; }
		else if (std::strcmp(arg, "-optIter")   == 0) { if (!nexti(a.optIter))  return false; }
		else if (std::strcmp(arg, "-optRelax")  == 0) { if (!next(a.optRelax))  return false; }
		else if (std::strcmp(arg, "-optQuality") == 0) { a.optQuality = true; }
		else if (std::strcmp(arg, "-optMinDot")  == 0) { if (!next(a.optMinDot))   return false; }
		else if (std::strcmp(arg, "-optMinAreaR") == 0){ if (!next(a.optMinAreaR)) return false; }
		else if (std::strcmp(arg, "-fitFeatures") == 0) { a.fitFeatures = true; }
		else if (std::strcmp(arg, "-fitFeatSafety") == 0) { if (!next(a.fitFeatSafety)) return false; }
		else if (std::strcmp(arg, "-fitFeatBump")   == 0) { if (!nexti(a.fitFeatBump))  return false; }
		else if (std::strcmp(arg, "-perFaceFit")    == 0) { a.perFaceFit = true; }
		else if (std::strcmp(arg, "-perFaceFitSafety") == 0) { if (!next(a.perFaceFitSafety)) return false; }
		else if (std::strcmp(arg, "-localFeatRef")  == 0) { a.localFeatRef = true; }
		else if (std::strcmp(arg, "-localFeatSafety") == 0) { if (!next(a.localFeatSafety)) return false; }
		else if (std::strcmp(arg, "-localFeatSearchMul") == 0) { if (!next(a.localFeatSearchMul)) return false; }
		else if (std::strcmp(arg, "-coverAllFaces") == 0) { a.coverAllFaces = true; }
		else if (std::strcmp(arg, "-coverRounds")   == 0) { if (!nexti(a.coverRounds)) return false; }
		else if (std::strcmp(arg, "-pinFeatures")   == 0) { a.pinFeatures = true; }
		else if (std::strcmp(arg, "-pinRadius")     == 0) { if (!next(a.pinRadius)) return false; }
		else if (std::strcmp(arg, "-edgeInsert")    == 0) { a.edgeInsert = true; }
		else if (std::strcmp(arg, "-edgeInsertRadius") == 0) { if (!next(a.edgeInsertR)) return false; }
		else if (std::strcmp(arg, "-untangle")      == 0) { a.untangle = true; }
		else if (std::strcmp(arg, "-untangleIter")  == 0) { if (!nexti(a.untangleIter)) return false; }
		else if (std::strcmp(arg, "-patchRefine")   == 0)
		{
			if (i + 1 >= argc) return false;
			const std::string s = argv[++i];
			std::size_t pos = 0;
			while (pos < s.size())
			{
				const std::size_t comma = s.find(',', pos);
				const std::string tok = s.substr(pos, comma - pos);
				const std::size_t eq = tok.find('=');
				if (eq != std::string::npos)
				{
					try {
						a.patchRefine.emplace_back(tok.substr(0, eq), std::stoi(tok.substr(eq + 1)));
					} catch (...) {
						std::cerr << "  bad -patchRefine entry: '" << tok << "'\n";
					}
				}
				if (comma == std::string::npos) break;
				pos = comma + 1;
			}
		}
		else if (std::strcmp(arg, "-locationInMesh") == 0)
		{
			if (!next(a.lim[0]) || !next(a.lim[1]) || !next(a.lim[2])) return false;
			a.useLocationInMesh = true;
		}
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
		          << brep.nFeatureVertices() << " verts (angle=" << A.featureAngle << ")\n"
		          << "  subPatches (TopoDS_Face count): " << brep.nSubPatches() << "\n";

		// 全套参数打包成 XFoam_CMshPipeline::Params
		XFoam_CMshPipeline::Params pp;
		pp.maxCellSize             = static_cast<XFoam_Scalar>(A.maxCellSize);
		pp.surfLevel               = A.surfLevel;
		pp.maxLevel                = A.maxLevel;
		pp.fitFeatures               = A.fitFeatures;
		pp.fitFeaturesSafety         = static_cast<XFoam_Scalar>(A.fitFeatSafety);
		pp.fitFeaturesMaxLevelBump   = A.fitFeatBump;
		pp.perFaceFitFeatures        = A.perFaceFit;
		pp.perFaceFitFeaturesSafety  = static_cast<XFoam_Scalar>(A.perFaceFitSafety);
		pp.localFeatureRefine        = A.localFeatRef;
		pp.localFeatureSafety        = static_cast<XFoam_Scalar>(A.localFeatSafety);
		pp.localFeatureSearchMul     = static_cast<XFoam_Scalar>(A.localFeatSearchMul);
		pp.coverAllFaces             = A.coverAllFaces;
		pp.coverAllFacesMaxRounds    = A.coverRounds;
		pp.enableFeaturePinner       = A.pinFeatures;
		pp.pinRadius                 = static_cast<XFoam_Scalar>(A.pinRadius);
		pp.enableEdgeInsert          = A.edgeInsert;
		pp.edgeInsertRadius          = static_cast<XFoam_Scalar>(A.edgeInsertR);
		pp.enableUntangler           = A.untangle;
		pp.untanglerIter             = A.untangleIter;
		pp.patchRefine               = A.patchRefine;
		pp.perFacePatches          = A.perFacePatches;
		pp.fillAllSubPatches       = A.fillAllSubPatches;
		pp.repatchAfterMap         = !A.noRepatch;
		pp.useLocationInMesh       = A.useLocationInMesh;
		pp.locationInMesh          = XFoam_Vector3D(A.lim[0], A.lim[1], A.lim[2]);
		pp.enableMapper            = A.mapSurface;
		pp.mapIter                 = A.mapIter;
		pp.mapRelax                = static_cast<XFoam_Scalar>(A.mapRelax);
		pp.enableEdgeSnap          = A.snapFeatures;
		pp.featureSearchRadius     = static_cast<XFoam_Scalar>(A.featureSearchRadius);
		pp.enableOptimizer         = A.optimize;
		pp.optIter                 = A.optIter;
		pp.optRelax                = static_cast<XFoam_Scalar>(A.optRelax);
		pp.optSnapFeatures         = A.snapFeatures;
		pp.optQuality              = A.optQuality;
		pp.optMinFaceNormalDot     = static_cast<XFoam_Scalar>(A.optMinDot);
		pp.optMinFaceAreaRatio     = static_cast<XFoam_Scalar>(A.optMinAreaR);
		pp.verbose                 = true;

		XFoam_CMshPipeline pipeline(pp);
		for (const auto& b : A.refineBoxes)
		{
			pipeline.addBoxRefine(
				XFoam_BoundBox(
					XFoam_Vector3D(b.xmn, b.ymn, b.zmn),
					XFoam_Vector3D(b.xmx, b.ymx, b.zmx)),
				b.level);
		}
		for (const auto& s : A.refineSpheres)
		{
			pipeline.addSphereRefine(
				XFoam_Vector3D(s.cx, s.cy, s.cz),
				static_cast<XFoam_Scalar>(s.r), s.level);
		}
		for (const auto& c : A.refineCones)
		{
			pipeline.addConeRefine(
				XFoam_Vector3D(c.ax, c.ay, c.az),
				XFoam_Vector3D(c.bx, c.by, c.bz),
				static_cast<XFoam_Scalar>(c.ra),
				static_cast<XFoam_Scalar>(c.rb),
				c.level);
		}

		if (!A.polyMeshOut.empty())
		{
			XFoam_CMshPolyMeshGen pm;
			const auto st = pipeline.run(brep, pm);
			(void) st;

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
