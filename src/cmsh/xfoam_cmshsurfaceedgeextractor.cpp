#include "XFoam/cmsh/xfoam_cmshsurfaceedgeextractor.h"

#include <cmath>
#include <iostream>
#include <limits>

XFoam_CMshSurfaceEdgeExtractor::XFoam_CMshSurfaceEdgeExtractor(
	XFoam_CMshPolyMeshGen& pm,
	const std::vector<const XFoam_BrepBase*>& breps,
	const std::vector<int>& bndPointIds,
	const Params& p)
	: pm_(pm), breps_(breps), bndPoints_(bndPointIds), p_(p)
{
}

XFoam_CMshSurfaceEdgeExtractor::Stats
XFoam_CMshSurfaceEdgeExtractor::snap()
{
	Stats stats;
	if (breps_.empty() || bndPoints_.empty()) return stats;

	XFoam_Scalar R = p_.searchRadius;
	if (R <= 0)
	{
		R = (p_.cellSizeHint > 0) ? (p_.cellSizeHint * 0.5)
		                          : static_cast<XFoam_Scalar>(0.5);
	}

	for (int vid : bndPoints_)
	{
		const XFoam_Vector3D& p = pm_.points[static_cast<std::size_t>(vid)];

		XFoam_Vector3D                  bestQ = p;
		XFoam_Scalar                    bestD = std::numeric_limits<XFoam_Scalar>::infinity();
		XFoam_BrepBase::FeatureKind     bestKind = XFoam_BrepBase::FeatureKind::None;

		for (const auto* b : breps_)
		{
			if (!b || b->empty()) continue;
			XFoam_Vector3D q, t;
			const auto kind = b->closestFeature(p, R, q, t);
			if (kind == XFoam_BrepBase::FeatureKind::None) continue;
			if (kind == XFoam_BrepBase::FeatureKind::Vertex && !p_.snapCorners) continue;
			if (kind == XFoam_BrepBase::FeatureKind::Edge   && !p_.snapEdges) continue;
			const XFoam_Scalar d = (p - q).mag();
			if (d >= bestD) continue;
			bestD    = d;
			bestQ    = q;
			bestKind = kind;
		}
		if (!std::isfinite(bestD) || bestD > p_.maxSnapDist) continue;
		if (bestKind == XFoam_BrepBase::FeatureKind::None) continue;

		pm_.points[static_cast<std::size_t>(vid)] = bestQ;
		if (bestKind == XFoam_BrepBase::FeatureKind::Vertex) ++stats.nCornerSnap;
		else                                                  ++stats.nEdgeSnap;
		if (bestD > stats.maxSnapDist) stats.maxSnapDist = bestD;
	}

	if (p_.verbose)
	{
		std::cout << "  edge extractor: corners=" << stats.nCornerSnap
		          << "  edges=" << stats.nEdgeSnap
		          << "  maxSnapDist=" << stats.maxSnapDist << std::endl;
	}
	return stats;
}
