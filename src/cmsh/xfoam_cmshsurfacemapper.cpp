#include "XFoam/cmsh/xfoam_cmshsurfacemapper.h"

#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_set>

XFoam_CMshSurfaceMapper::XFoam_CMshSurfaceMapper(
	XFoam_CMshPolyMeshGen& pm,
	const XFoam_BrepBase& brep,
	const Params& p)
	: pm_(pm), brep_(brep), p_(p)
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
	if (brep_.empty() || bndPoints_.empty()) return 0;
	XFoam_Label totalMoved = 0;

	for (int iter = 0; iter < p_.nIterations; ++iter)
	{
		XFoam_Label movedThisIter = 0;
		XFoam_Scalar maxDist = 0;

		for (int vid : bndPoints_)
		{
			const XFoam_Vector3D& p = pm_.points[static_cast<std::size_t>(vid)];
			XFoam_Vector3D q, n;
			brep_.closestPointAndNormal(p, q, n);
			const XFoam_Scalar d = (p - q).mag();
			if (!std::isfinite(d) || d > p_.maxDist) continue;

			const XFoam_Vector3D newP = p + (q - p) * p_.relaxFactor;
			pm_.points[static_cast<std::size_t>(vid)] = newP;
			++movedThisIter;
			if (d > maxDist) maxDist = d;
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
