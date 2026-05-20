#include "XFoam/cmsh/xfoam_cmshpipeline.h"

#include "XFoam/cmsh/xfoam_cmshcartesianextractor.h"
#include "XFoam/cmsh/xfoam_cmshoctree.h"
#include "XFoam/cmsh/xfoam_cmshoctreecreator.h"
#include "XFoam/cmsh/xfoam_cmshrepatcher.h"
#include "XFoam/cmsh/xfoam_cmshsurfacemapper.h"
#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_boundbox.h"

#include <chrono>
#include <iostream>
#include <utility>

namespace
{
double ms(const std::chrono::steady_clock::time_point& a,
          const std::chrono::steady_clock::time_point& b)
{
	return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(b - a).count();
}
} // namespace

XFoam_CMshPipeline::XFoam_CMshPipeline(const Params& p)
	: p_(p)
{}

XFoam_CMshPipeline& XFoam_CMshPipeline::addBoxRefine(
	const XFoam_BoundBox& box, int level, const std::string& name)
{
	auto o = std::make_unique<XFoam_CMshBoxRefine>();
	o->box = box; o->level = level; o->name = name;
	objects_.push_back(std::move(o));
	return *this;
}

XFoam_CMshPipeline& XFoam_CMshPipeline::addSphereRefine(
	const XFoam_Vector3D& centre, XFoam_Scalar radius, int level,
	const std::string& name)
{
	auto o = std::make_unique<XFoam_CMshSphereRefine>();
	o->centre = centre; o->radius = radius; o->level = level; o->name = name;
	objects_.push_back(std::move(o));
	return *this;
}

XFoam_CMshPipeline& XFoam_CMshPipeline::addConeRefine(
	const XFoam_Vector3D& a, const XFoam_Vector3D& b,
	XFoam_Scalar radiusA, XFoam_Scalar radiusB, int level,
	const std::string& name)
{
	auto o = std::make_unique<XFoam_CMshConeRefine>();
	o->a = a; o->b = b; o->radiusA = radiusA; o->radiusB = radiusB;
	o->level = level; o->name = name;
	objects_.push_back(std::move(o));
	return *this;
}

XFoam_CMshPipeline& XFoam_CMshPipeline::addObjectRefine(
	std::unique_ptr<XFoam_CMshObjRefine> obj)
{
	if (obj) objects_.push_back(std::move(obj));
	return *this;
}

XFoam_CMshPipeline::Stats
XFoam_CMshPipeline::run(XFoam_BrepBase& brep, XFoam_CMshPolyMeshGen& pm)
{
	Stats stats;

	// === 1) octree ===
	XFoam_CMshOctreeCreator::Params cp;
	cp.maxCellSize             = p_.maxCellSize;
	cp.maxLevel                = p_.maxLevel;
	cp.inflateRoot             = p_.inflateRoot;
	cp.rootInflate             = p_.rootInflate;
	cp.fitFeatures             = p_.fitFeatures;
	cp.fitFeaturesSafety       = p_.fitFeaturesSafety;
	cp.fitFeaturesMaxLevelBump = p_.fitFeaturesMaxLevelBump;

	XFoam_CMshOctreeCreator cr(brep.bounds(), cp);
	cr.addSurfaceRefine(brep, p_.surfLevel);
	for (auto& op : objects_)
	{
		if (op) cr.addObjectRefine(std::move(op));
	}
	objects_.clear();

	if (p_.verbose)
	{
		std::cout << "[cmsh pipeline] building octree (maxCellSize=" << p_.maxCellSize
		          << ", surfLevel=" << p_.surfLevel << ")...\n";
	}
	auto t0 = std::chrono::steady_clock::now();
	auto oct = cr.build();
	auto t1 = std::chrono::steady_clock::now();
	stats.msOctree = ms(t0, t1);
	stats.nLeaves  = oct().nLeaves();
	if (p_.verbose)
	{
		std::cout << "[cmsh pipeline] octree done in " << stats.msOctree
		          << " ms, leaves=" << stats.nLeaves << "\n";
	}

	// === 2) extractor ===
	XFoam_CMshCartesianExtractor::Params ep;
	ep.keepInside        = p_.keepInside;
	ep.keepData          = p_.keepData;
	ep.keepOutside       = p_.keepOutside;
	ep.perFacePatches    = p_.perFacePatches;
	ep.useLocationInMesh = p_.useLocationInMesh;
	ep.locationInMesh    = p_.locationInMesh;
	ep.defaultPatchName  = p_.defaultPatchName;
	ep.defaultPatchType  = p_.defaultPatchType;
	ep.fillAllSubPatches = p_.fillAllSubPatches;
	XFoam_CMshCartesianExtractor ex(oct(), ep, &brep);

	t0 = std::chrono::steady_clock::now();
	const bool extracted = ex.extract(pm);
	t1 = std::chrono::steady_clock::now();
	stats.msExtract = ms(t0, t1);
	if (!extracted)
	{
		if (p_.verbose) std::cerr << "[cmsh pipeline] extract returned empty mesh.\n";
		return stats;
	}
	stats.nCells         = pm.nCells;
	stats.nPoints        = static_cast<XFoam_Label>(pm.points.size());
	stats.nFaces         = pm.nFaces();
	stats.nInternalFaces = pm.nInternalFaces();
	stats.nPatches       = static_cast<XFoam_Label>(pm.patches.size());
	if (p_.verbose)
	{
		std::cout << "[cmsh pipeline] extract done in " << stats.msExtract
		          << " ms (cells=" << stats.nCells
		          << ", pts=" << stats.nPoints
		          << ", faces=" << stats.nFaces
		          << " [int " << stats.nInternalFaces
		          << " / bnd " << pm.nBoundaryFaces() << "]"
		          << ", patches=" << stats.nPatches << ")\n";
	}

	// === 3) mapper ===
	std::vector<int> bndPoints;
	if (p_.enableMapper || p_.enableEdgeSnap || p_.enableOptimizer)
	{
		XFoam_CMshSurfaceMapper::Params mp;
		mp.nIterations = p_.mapIter;
		mp.relaxFactor = p_.mapRelax;
		mp.maxDist     = p_.mapMaxDist;
		mp.verbose     = p_.verbose;
		XFoam_CMshSurfaceMapper mapper(pm, brep, mp);
		bndPoints = mapper.boundaryPointIds();

		if (p_.enableMapper)
		{
			t0 = std::chrono::steady_clock::now();
			stats.mapperMoved = mapper.mapToSurface();
			t1 = std::chrono::steady_clock::now();
			stats.msMapper = ms(t0, t1);
			if (p_.verbose)
			{
				std::cout << "[cmsh pipeline] mapper done in " << stats.msMapper
				          << " ms (moved=" << stats.mapperMoved << ")\n";
			}
		}
	}

	const XFoam_Scalar cellSizeHint =
		p_.maxCellSize / static_cast<XFoam_Scalar>(1 << p_.surfLevel);

	// === 4) edge snap ===
	if (p_.enableEdgeSnap && !bndPoints.empty())
	{
		XFoam_CMshSurfaceEdgeExtractor::Params epp;
		epp.searchRadius = p_.featureSearchRadius;
		epp.cellSizeHint = cellSizeHint;
		epp.snapCorners  = p_.snapCorners;
		epp.snapEdges    = p_.snapEdges;
		epp.verbose      = p_.verbose;
		XFoam_CMshSurfaceEdgeExtractor edx(pm, brep, bndPoints, epp);

		t0 = std::chrono::steady_clock::now();
		stats.edgeStats = edx.snap();
		t1 = std::chrono::steady_clock::now();
		stats.msEdge = ms(t0, t1);
		if (p_.verbose)
		{
			std::cout << "[cmsh pipeline] edge snap done in " << stats.msEdge
			          << " ms (corners=" << stats.edgeStats.nCornerSnap
			          << ", edges=" << stats.edgeStats.nEdgeSnap
			          << ", maxSnapDist=" << stats.edgeStats.maxSnapDist << ")\n";
		}
	}

	// === 5) optimizer ===
	if (p_.enableOptimizer)
	{
		XFoam_CMshMeshOptimizer::Params op;
		op.nIterations         = p_.optIter;
		op.relaxFactor         = p_.optRelax;
		op.reproject           = p_.optReproject;
		op.snapFeatures        = p_.optSnapFeatures;
		op.cellSizeHint        = cellSizeHint;
		op.qualityCheck        = p_.optQuality;
		op.minFaceNormalDot    = p_.optMinFaceNormalDot;
		op.minFaceAreaRatio    = p_.optMinFaceAreaRatio;
		op.verbose             = p_.verbose;
		XFoam_CMshMeshOptimizer optr(pm, brep, op);

		t0 = std::chrono::steady_clock::now();
		stats.optimizerStats = optr.optimize();
		t1 = std::chrono::steady_clock::now();
		stats.msOptimizer = ms(t0, t1);
		if (p_.verbose)
		{
			std::cout << "[cmsh pipeline] optimizer done in " << stats.msOptimizer
			          << " ms (lastIter moved=" << stats.optimizerStats.nMoved
			          << ", avg=" << stats.optimizerStats.avgMove
			          << ", max=" << stats.optimizerStats.maxMove
			          << ", rollback=" << stats.optimizerStats.nRollback << ")\n";
		}
	}

	// === 6) repatch（perFacePatches 下，用 post-mapped centroid 重写 patches）===
	if (p_.perFacePatches && p_.repatchAfterMap
	    && (p_.enableMapper || p_.enableEdgeSnap || p_.enableOptimizer))
	{
		XFoam_CMshRepatcher::Params rp;
		rp.defaultPatchType  = p_.defaultPatchType;
		rp.fillAllSubPatches = p_.fillAllSubPatches;
		rp.verbose           = p_.verbose;
		XFoam_CMshRepatcher rep(pm, brep, rp);

		t0 = std::chrono::steady_clock::now();
		stats.repatchStats = rep.repatch();
		t1 = std::chrono::steady_clock::now();
		stats.msRepatch = ms(t0, t1);
		stats.nPatches = static_cast<XFoam_Label>(pm.patches.size());
		if (p_.verbose)
		{
			std::cout << "[cmsh pipeline] repatch done in " << stats.msRepatch
			          << " ms (reassigned=" << stats.repatchStats.nReassigned
			          << ", patches " << stats.repatchStats.nPatchesBefore
			          << " -> " << stats.repatchStats.nPatchesAfter
			          << ", empty added " << stats.repatchStats.nEmptyAdded << ")\n";
		}
	}

	if (p_.verbose)
	{
		const double total = stats.msOctree + stats.msExtract + stats.msMapper
		                   + stats.msEdge + stats.msOptimizer + stats.msRepatch;
		std::cout << "[cmsh pipeline] total " << total << " ms\n";
	}
	return stats;
}
