#include "XFoam/cmsh/xfoam_cmshsurfacemapper.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_set>

XFoam_CMshSurfaceMapper::XFoam_CMshSurfaceMapper(
	XFoam_CMshPolyMeshGen& pm,
	const std::vector<const XFoam_BrepBase*>& breps,
	const Params& p)
	: pm_(pm), breps_(breps), p_(p)
{
	collectBoundaryPoints();
}

void XFoam_CMshSurfaceMapper::collectBoundaryPoints()
{
	bndPoints_.clear();
	std::unordered_set<int> seen;
	seen.reserve(pm_.points.size() / 4);
	const int nInt = pm_.nInternalFaces();
	const int nAll = pm_.nFaces();
	for (int fi = nInt; fi < nAll; ++fi)
	{
		for (int v : pm_.faces[static_cast<std::size_t>(fi)].verts)
		{
			if (seen.insert(v).second) bndPoints_.push_back(v);
		}
	}
	std::sort(bndPoints_.begin(), bndPoints_.end());
}

XFoam_Label XFoam_CMshSurfaceMapper::mapToSurface()
{
	if (breps_.empty() || bndPoints_.empty()) return 0;
	XFoam_Label totalMoved = 0;

	for (int iter = 0; iter < p_.nIterations; ++iter)
	{
		XFoam_Label movedThisIter = 0;
		XFoam_Scalar maxDist = 0;

		for (int vid : bndPoints_)
		{
			const XFoam_Vector3D& p = pm_.points[static_cast<std::size_t>(vid)];

			// 找最近的 brep
			XFoam_Vector3D bestQ = p;
			XFoam_Scalar   bestD = std::numeric_limits<XFoam_Scalar>::infinity();
			for (const auto* b : breps_)
			{
				if (!b || b->empty()) continue;
				XFoam_Vector3D q, n;
				b->closestPointAndNormal(p, q, n);
				const XFoam_Scalar d = (p - q).mag();
				if (d < bestD)
				{
					bestD = d;
					bestQ = q;
				}
			}
			if (!std::isfinite(bestD) || bestD > p_.maxDist) continue;

			const XFoam_Vector3D newP = p + (bestQ - p) * p_.relaxFactor;
			pm_.points[static_cast<std::size_t>(vid)] = newP;
			++movedThisIter;
			if (bestD > maxDist) maxDist = bestD;
		}
		totalMoved += movedThisIter;
		if (p_.verbose)
		{
			std::cout << "  mapper iter " << iter << ": moved " << movedThisIter
			          << "  maxDist=" << maxDist << std::endl;
		}
	}
	return totalMoved;
}
