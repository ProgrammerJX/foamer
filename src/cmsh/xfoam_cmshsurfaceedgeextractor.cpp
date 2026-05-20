#include "XFoam/cmsh/xfoam_cmshsurfaceedgeextractor.h"

#include "XFoam/topo/xfoam_brep.h"

#include <cmath>
#include <iostream>
#include <limits>

XFoam_CMshSurfaceEdgeExtractor::XFoam_CMshSurfaceEdgeExtractor(
	XFoam_CMshPolyMeshGen& pm,
	const XFoam_BrepBase& brep,
	const std::vector<int>& bndPointIds,
	const Params& p)
	: pm_(pm), brep_(brep), bndPoints_(bndPointIds), p_(p)
{
}

XFoam_CMshSurfaceEdgeExtractor::Stats
XFoam_CMshSurfaceEdgeExtractor::snap()
{
	Stats stats;
	if (brep_.empty() || bndPoints_.empty()) return stats;

	XFoam_Scalar R = p_.searchRadius;
	if (R <= 0)
	{
		R = (p_.cellSizeHint > 0) ? (p_.cellSizeHint * 0.5)
		                          : static_cast<XFoam_Scalar>(0.5);
	}

	for (int vid : bndPoints_)
	{
		const XFoam_Vector3D& p = pm_.points[static_cast<std::size_t>(vid)];
		XFoam_Vector3D q, t;
		const auto kind = brep_.closestFeature(p, R, q, t);
		if (kind == XFoam_BrepBase::FeatureKind::None) continue;
		if (kind == XFoam_BrepBase::FeatureKind::Vertex && !p_.snapCorners) continue;
		if (kind == XFoam_BrepBase::FeatureKind::Edge   && !p_.snapEdges) continue;
		const XFoam_Scalar d = (p - q).mag();
		if (!std::isfinite(d) || d > p_.maxSnapDist) continue;

		pm_.points[static_cast<std::size_t>(vid)] = q;
		if (kind == XFoam_BrepBase::FeatureKind::Vertex) ++stats.nCornerSnap;
		else                                              ++stats.nEdgeSnap;
		if (d > stats.maxSnapDist) stats.maxSnapDist = d;
	}

	if (p_.verbose)
	{
		std::cout << "  edge extractor: corners=" << stats.nCornerSnap
		          << "  edges=" << stats.nEdgeSnap
		          << "  maxSnapDist=" << stats.maxSnapDist << std::endl;
	}
	return stats;
}
