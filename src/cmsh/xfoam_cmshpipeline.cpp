#include "XFoam/cmsh/xfoam_cmshpipeline.h"

#include "XFoam/cmsh/xfoam_cmshcartesianextractor.h"
#include "XFoam/cmsh/xfoam_cmshedgeinserter.h"
#include "XFoam/cmsh/xfoam_cmshfeaturepinner.h"
#include "XFoam/cmsh/xfoam_cmshmeshuntangler.h"
#include "XFoam/cmsh/xfoam_cmshoctree.h"
#include "XFoam/cmsh/xfoam_cmshsurfaceengine.h"
#include "XFoam/cmsh/xfoam_cmshoctreecreator.h"
#include "XFoam/cmsh/xfoam_cmshrepatcher.h"
#include "XFoam/cmsh/xfoam_cmshsurfacemapper.h"
#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_boundbox.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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
	cp.fitFeatures               = p_.fitFeatures;
	cp.fitFeaturesSafety         = p_.fitFeaturesSafety;
	cp.fitFeaturesMaxLevelBump   = p_.fitFeaturesMaxLevelBump;
	cp.perFaceFitFeatures        = p_.perFaceFitFeatures;
	cp.perFaceFitFeaturesSafety  = p_.perFaceFitFeaturesSafety;
	cp.localFeatureRefine        = p_.localFeatureRefine;
	cp.localFeatureSafety        = p_.localFeatureSafety;
	cp.localFeatureSearchMul     = p_.localFeatureSearchMul;
	cp.curvatureRefine           = p_.curvatureRefine;
	cp.curvatureSafety           = p_.curvatureSafety;

	XFoam_CMshOctreeCreator cr(brep.bounds(), cp);
	cr.addSurfaceRefine(brep, p_.surfLevel);
	for (auto& op : objects_)
	{
		if (op) cr.addObjectRefine(std::move(op));
	}
	objects_.clear();

	for (const auto& kv : p_.patchRefine)
		cr.addPatchRefine(kv.first, kv.second);

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
	stats.nCoveredSubs   = ex.stats().nCoveredSubs;
	stats.nMissingSubs   = static_cast<XFoam_Label>(ex.stats().missingSubIds.size());
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

	// === 2b) coverAllFaces：cfMesh::automaticRefinement 风格的自适应循环 ===
	// 对每个未命中的 sub-patch，调 refineRegion(subPatchBounds, target)，
	// target = ceil(log2(rootSpan / (bbox_min_side * safety)))，cap 在 maxLevel。
	// 突破 perFaceFitFeatures 的 bumpCap（只针对 sliver 面，不全局炸 leaf）。
	if (p_.coverAllFaces && p_.perFacePatches
	    && stats.nMissingSubs > 0
	    && p_.coverAllFacesMaxRounds > 0)
	{
		auto tCover0 = std::chrono::steady_clock::now();
		const XFoam_BoundBox rootBox = brep.bounds(); // 注：oct 用的是 inflate 后的，但 brep.bounds() 用来反推 level 够精度
		const XFoam_Vector3D rs = rootBox.span();
		const XFoam_Scalar   rsMax = std::max(rs.x(), std::max(rs.y(), rs.z()));
		const XFoam_Scalar   safety = p_.perFaceFitFeaturesSafety > 0
		                                ? p_.perFaceFitFeaturesSafety
		                                : static_cast<XFoam_Scalar>(0.5);

		for (int round = 0; round < p_.coverAllFacesMaxRounds; ++round)
		{
			const auto& mids = ex.stats().missingSubIds;
			if (mids.empty()) break;

			XFoam_Label nRequested = 0;
			XFoam_Label nClampedToMax = 0;
			for (XFoam_Label sid : mids)
			{
				const XFoam_BoundBox bb = brep.subPatchBounds(sid);
				if (bb.max().x() < bb.min().x()) continue; // invalid
				const XFoam_Vector3D fs = bb.span();
				XFoam_Scalar minSide = std::min({fs.x(), fs.y(), fs.z()});
				if (minSide <= 0) continue;
				const XFoam_Scalar wanted = minSide * safety;
				if (wanted <= 0 || rsMax <= 0) continue;
				const double ratio = static_cast<double>(rsMax) / static_cast<double>(wanted);
				if (ratio <= 1.0) continue;
				int needed = static_cast<int>(std::ceil(std::log(ratio) / std::log(2.0)));
				// 每一轮额外 + round 以保证从来没成功的 face 至少多 push 一级
				needed += round;
				const int target = std::min(needed, p_.maxLevel);
				if (target == p_.maxLevel) ++nClampedToMax;
				// 注意：用 brep.subPatchBounds(sid) 当 region；OCCT 给的精度比 leaf 大，
				// refineRegion 走 overlaps，已包含 in-leaf 的小 face。
				oct().refineRegion(bb, target);
				++nRequested;
			}
			oct().balance21();
			oct().classifyLeaves();
			if (p_.verbose)
			{
				std::cout << "[cmsh pipeline] coverAllFaces round " << round
				          << ": missing=" << mids.size()
				          << " requested " << nRequested
				          << " (" << nClampedToMax << " clamped to maxLevel="
				          << p_.maxLevel << "), leaves=" << oct().nLeaves() << "\n";
			}

			// re-extract（overwrite pm）
			auto re0 = std::chrono::steady_clock::now();
			const bool reok = ex.extract(pm);
			auto re1 = std::chrono::steady_clock::now();
			if (!reok) break;
			stats.nCells         = pm.nCells;
			stats.nPoints        = static_cast<XFoam_Label>(pm.points.size());
			stats.nFaces         = pm.nFaces();
			stats.nInternalFaces = pm.nInternalFaces();
			stats.nPatches       = static_cast<XFoam_Label>(pm.patches.size());
			stats.nCoveredSubs   = ex.stats().nCoveredSubs;
			stats.nMissingSubs   = static_cast<XFoam_Label>(ex.stats().missingSubIds.size());
			stats.coverRounds    = round + 1;
			stats.nLeaves        = oct().nLeaves();
			if (p_.verbose)
			{
				std::cout << "[cmsh pipeline] coverAllFaces round " << round
				          << " re-extract in " << ms(re0, re1)
				          << " ms, covered=" << stats.nCoveredSubs
				          << "/" << ex.stats().nSubPatches
				          << ", missing=" << stats.nMissingSubs << "\n";
			}
			if (stats.nMissingSubs == 0) break;
		}
		auto tCover1 = std::chrono::steady_clock::now();
		stats.msCoverLoop = ms(tCover0, tCover1);
	}

	// === 2c) SurfaceEngine：一次性建好所有 boundary topology cache，让后
	// 续 mapper / pinner / edgeInserter / optimizer 都复用。edgeInserter
	// 改 topology 后会重建一次。
	auto buildSE = [&]() {
		return std::make_unique<XFoam_CMshSurfaceEngine>(pm);
	};
	std::unique_ptr<XFoam_CMshSurfaceEngine> se = buildSE();
	if (p_.verbose)
	{
		std::cout << "[cmsh pipeline] surface engine: bndPoints=" << se->nBndPoints()
		          << ", bndFaces=" << se->nBndFaces()
		          << ", edges=" << se->nEdges() << "\n";
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

	// === 4b) feature pinner (B1: TpVertex 钉死) ===
	std::vector<int> pinnedPts;
	if (p_.enableFeaturePinner)
	{
		XFoam_CMshFeaturePinner::Params pp;
		pp.pinRadius    = p_.pinRadius;
		pp.cellSizeHint = cellSizeHint;
		pp.verbose      = p_.verbose;
		XFoam_CMshFeaturePinner pinner(pm, brep, pp, *se);
		t0 = std::chrono::steady_clock::now();
		stats.pinStats = pinner.pin();
		t1 = std::chrono::steady_clock::now();
		stats.msPin = ms(t0, t1);
		pinnedPts = pinner.pinnedPoints();
		if (p_.verbose)
		{
			std::cout << "[cmsh pipeline] pinner done in " << stats.msPin
			          << " ms (tpVerts=" << stats.pinStats.nTpVerts
			          << ", pinned=" << stats.pinStats.nPinned
			          << ", oor=" << stats.pinStats.nOutOfRange
			          << ", conflict=" << stats.pinStats.nConflictSkipped << ")\n";
		}
	}

	// === 4c) edge inserter (B2: TpEdge densify; cfMesh createEdgeVertices) ===
	if (p_.enableEdgeInsert)
	{
		XFoam_CMshEdgeInserter::Params ep2;
		ep2.searchRadius       = p_.edgeInsertRadius;
		ep2.cellSizeHint       = cellSizeHint;
		ep2.requireEdgeFeature = p_.edgeInsertRequireFeature;
		ep2.verbose            = p_.verbose;
		// SE 只在 perFacePatches 下能正确识别 cross-patch（即 cross-TpFace）
		XFoam_CMshEdgeInserter inserter = p_.perFacePatches
			? XFoam_CMshEdgeInserter(pm, brep, ep2, *se)
			: XFoam_CMshEdgeInserter(pm, brep, ep2);
		t0 = std::chrono::steady_clock::now();
		stats.edgeInsertStats = inserter.insert();
		t1 = std::chrono::steady_clock::now();
		stats.msEdgeInsert = ms(t0, t1);
		// 新增的 mesh point 也加入 fixedPoints（已落到 TpEdge，optimizer 别动）
		for (int pid : inserter.newPoints()) pinnedPts.push_back(pid);
		stats.nPoints = static_cast<XFoam_Label>(pm.points.size());
		stats.nFaces  = pm.nFaces();
		// topology 变了（新 point + face.verts 改），SurfaceEngine 必须重建
		se = buildSE();
		if (p_.verbose)
		{
			std::cout << "[cmsh pipeline] surface engine rebuilt after edgeInsert:"
			          << " bndPoints=" << se->nBndPoints()
			          << ", bndFaces=" << se->nBndFaces()
			          << ", edges=" << se->nEdges() << "\n";
		}
		if (p_.verbose)
		{
			std::cout << "[cmsh pipeline] edgeInsert done in " << stats.msEdgeInsert
			          << " ms (boundaryEdges=" << stats.edgeInsertStats.nBoundaryEdges
			          << ", crossPatch=" << stats.edgeInsertStats.nCrossPatch
			          << ", inserted=" << stats.edgeInsertStats.nInserted
			          << ", projFail=" << stats.edgeInsertStats.nProjFail
			          << ", facesGrown=" << stats.edgeInsertStats.nFacesGrown
			          << ", maxProjDist=" << stats.edgeInsertStats.maxProjDist << ")\n";
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
		XFoam_CMshMeshOptimizer optr(pm, brep, op, *se);
		if (!pinnedPts.empty()) optr.setFixedPoints(pinnedPts);

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

	// === 5b) untangler ===
	if (p_.enableUntangler)
	{
		XFoam_CMshMeshUntangler::Params up;
		up.nIterations  = p_.untanglerIter;
		up.tangleDot    = p_.untanglerTangleDot;
		up.cellSizeHint = cellSizeHint;
		up.reproject    = p_.optReproject;
		up.verbose      = p_.verbose;
		XFoam_CMshMeshUntangler unt(pm, &brep, up, *se);
		if (!pinnedPts.empty()) unt.setFixedPoints(pinnedPts);
		t0 = std::chrono::steady_clock::now();
		stats.untanglerStats = unt.untangle();
		t1 = std::chrono::steady_clock::now();
		stats.msUntangler = ms(t0, t1);
		if (p_.verbose)
		{
			std::cout << "[cmsh pipeline] untangler done in " << stats.msUntangler
			          << " ms (tangled " << stats.untanglerStats.nFacesTangled0
			          << " -> " << stats.untanglerStats.nFacesTangledN
			          << ", touched=" << stats.untanglerStats.nPointsTouched
			          << ", improved=" << stats.untanglerStats.nPointsImproved
			          << ", maxMove=" << stats.untanglerStats.maxMove << ")\n";
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
		const double total = stats.msOctree + stats.msExtract + stats.msCoverLoop
		                   + stats.msMapper + stats.msEdge + stats.msPin
		                   + stats.msEdgeInsert + stats.msOptimizer
		                   + stats.msUntangler + stats.msRepatch;
		std::cout << "[cmsh pipeline] total " << total << " ms\n";
	}
	return stats;
}
