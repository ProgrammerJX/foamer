#include "XFoam/cmsh/xfoam_cmshmeshoptimizer.h"

#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <unordered_set>

XFoam_CMshMeshOptimizer::XFoam_CMshMeshOptimizer(
	XFoam_CMshPolyMeshGen& pm,
	const XFoam_BrepBase& brep,
	const Params& p)
	: pm_(pm), brep_(brep), p_(p)
{
	isFixed_.assign(pm_.points.size(), 0);
	buildBoundaryAdjacency();
}

void XFoam_CMshMeshOptimizer::setFixedPoints(std::vector<int> fixedPointIds)
{
	std::sort(fixedPointIds.begin(), fixedPointIds.end());
	fixedPointIds.erase(
		std::unique(fixedPointIds.begin(), fixedPointIds.end()),
		fixedPointIds.end());
	isFixed_.assign(pm_.points.size(), 0);
	for (int pid : fixedPointIds)
	{
		if (pid >= 0 && pid < static_cast<int>(isFixed_.size()))
			isFixed_[static_cast<std::size_t>(pid)] = 1;
	}
}

void XFoam_CMshMeshOptimizer::buildBoundaryAdjacency()
{
	// 1) 收 unique boundary points
	std::unordered_set<int> seen;
	const int nInt = pm_.nInternalFaces();
	const int nAll = pm_.nFaces();
	seen.reserve(static_cast<std::size_t>(nAll - nInt) * 4);
	for (int fi = nInt; fi < nAll; ++fi)
	{
		for (int v : pm_.faces[static_cast<std::size_t>(fi)].verts) seen.insert(v);
	}
	bndPoints_.assign(seen.begin(), seen.end());
	std::sort(bndPoints_.begin(), bndPoints_.end());

	// 2) 反查 globalVID → 在 bndPoints_ 中的位置
	std::unordered_map<int, int> g2l;
	g2l.reserve(bndPoints_.size() * 2);
	for (std::size_t i = 0; i < bndPoints_.size(); ++i)
	{
		g2l[bndPoints_[i]] = static_cast<int>(i);
	}

	// 3) 用 set 暂存避免重复；同面共点之间互相加邻
	std::vector<std::unordered_set<int>> tmp(bndPoints_.size());
	for (int fi = nInt; fi < nAll; ++fi)
	{
		const auto& verts = pm_.faces[static_cast<std::size_t>(fi)].verts;
		const std::size_t n = verts.size();
		for (std::size_t i = 0; i < n; ++i)
		{
			const int vi = verts[i];
			const auto itI = g2l.find(vi);
			if (itI == g2l.end()) continue;
			for (std::size_t j = 0; j < n; ++j)
			{
				if (i == j) continue;
				const int vj = verts[j];
				tmp[static_cast<std::size_t>(itI->second)].insert(vj);
			}
		}
	}

	// 4) 摊平
	nbrs_.assign(bndPoints_.size(), {});
	for (std::size_t i = 0; i < bndPoints_.size(); ++i)
	{
		nbrs_[i].assign(tmp[i].begin(), tmp[i].end());
	}

	// 5) 顺便建 incidentFaces_：boundary point → 该点所在 boundary face 列表
	incidentFaces_.assign(bndPoints_.size(), {});
	for (int fi = nInt; fi < nAll; ++fi)
	{
		for (int v : pm_.faces[static_cast<std::size_t>(fi)].verts)
		{
			const auto it = g2l.find(v);
			if (it == g2l.end()) continue;
			incidentFaces_[static_cast<std::size_t>(it->second)].push_back(fi);
		}
	}
}

void XFoam_CMshMeshOptimizer::faceNormalAndArea(
	int faceIdx,
	XFoam_Vector3D& outNormal,
	XFoam_Scalar&   outArea) const
{
	// Newell's formula：多边形法向 = Σ (v_i × v_{i+1})，面积 = 0.5 * |normal|
	const auto& verts = pm_.faces[static_cast<std::size_t>(faceIdx)].verts;
	outNormal = XFoam_Vector3D(0, 0, 0);
	outArea   = 0;
	const std::size_t n = verts.size();
	if (n < 3) return;
	XFoam_Vector3D N(0, 0, 0);
	for (std::size_t i = 0; i < n; ++i)
	{
		const auto& a = pm_.points[static_cast<std::size_t>(verts[i])];
		const auto& b = pm_.points[static_cast<std::size_t>(verts[(i + 1) % n])];
		N += XFoam_Vector3D(
			(a.y() - b.y()) * (a.z() + b.z()),
			(a.z() - b.z()) * (a.x() + b.x()),
			(a.x() - b.x()) * (a.y() + b.y()));
	}
	const XFoam_Scalar mag = N.mag();
	outArea = static_cast<XFoam_Scalar>(0.5) * mag;
	if (mag > 0)
	{
		outNormal = N * (XFoam_Scalar(1) / mag);
	}
}

void XFoam_CMshMeshOptimizer::reprojectOne(int vid)
{
	if (brep_.empty()) return;
	const XFoam_Vector3D& p = pm_.points[static_cast<std::size_t>(vid)];
	XFoam_Vector3D q, n;
	brep_.closestPointAndNormal(p, q, n);
	if (std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()))
	{
		pm_.points[static_cast<std::size_t>(vid)] = q;
	}
}

XFoam_CMshMeshOptimizer::Stats XFoam_CMshMeshOptimizer::optimize()
{
	Stats stats;
	if (bndPoints_.empty()) return stats;

	XFoam_Scalar R = p_.featureSearchRadius;
	if (R <= 0)
	{
		R = (p_.cellSizeHint > 0) ? (p_.cellSizeHint * static_cast<XFoam_Scalar>(0.5))
		                          : static_cast<XFoam_Scalar>(0.5);
	}

	for (int iter = 0; iter < p_.nIterations; ++iter)
	{
		// 快照旧位置避免迭代污染
		std::vector<XFoam_Vector3D> snap(bndPoints_.size());
		for (std::size_t i = 0; i < bndPoints_.size(); ++i)
		{
			snap[i] = pm_.points[static_cast<std::size_t>(bndPoints_[i])];
		}

		// 若开 quality check，先记下所有受影响 boundary face 的 (normal, area)
		std::unordered_map<int, std::pair<XFoam_Vector3D, XFoam_Scalar>> oldFaceQ;
		if (p_.qualityCheck)
		{
			std::unordered_set<int> seenF;
			for (const auto& vf : incidentFaces_)
			{
				for (int fi : vf) seenF.insert(fi);
			}
			oldFaceQ.reserve(seenF.size() * 2);
			for (int fi : seenF)
			{
				XFoam_Vector3D n; XFoam_Scalar a;
				faceNormalAndArea(fi, n, a);
				oldFaceQ.emplace(fi, std::make_pair(n, a));
			}
		}

		stats = Stats{};
		XFoam_Scalar accMove = 0;

		// 1) Laplacian
		for (std::size_t i = 0; i < bndPoints_.size(); ++i)
		{
			const int vid = bndPoints_[i];
			if (!isFixed_.empty() && isFixed_[static_cast<std::size_t>(vid)]) continue;
			const auto& nb = nbrs_[i];
			if (nb.empty()) continue;
			XFoam_Vector3D mean(0, 0, 0);
			for (int gn : nb)
			{
				mean += pm_.points[static_cast<std::size_t>(gn)];
			}
			mean *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(nb.size()));
			const XFoam_Vector3D oldP = snap[i];
			const XFoam_Vector3D newP = oldP + (mean - oldP) * p_.relaxFactor;
			pm_.points[static_cast<std::size_t>(vid)] = newP;
		}

		// 2) re-project
		if (p_.reproject && !brep_.empty())
		{
			for (int vid : bndPoints_)
			{
				if (!isFixed_.empty() && isFixed_[static_cast<std::size_t>(vid)]) continue;
				reprojectOne(vid);
			}
		}

		// 3) 可选 feature snap
		if (p_.snapFeatures && !brep_.empty())
		{
			for (int vid : bndPoints_)
			{
				if (!isFixed_.empty() && isFixed_[static_cast<std::size_t>(vid)]) continue;
				const XFoam_Vector3D& p = pm_.points[static_cast<std::size_t>(vid)];
				XFoam_Vector3D q, t;
				const auto k = brep_.closestFeature(p, R, q, t);
				if (k == XFoam_BrepBase::FeatureKind::None) continue;
				pm_.points[static_cast<std::size_t>(vid)] = q;
			}
		}

		// 3b) quality check + rollback
		if (p_.qualityCheck)
		{
			for (std::size_t i = 0; i < bndPoints_.size(); ++i)
			{
				bool bad = false;
				for (int fi : incidentFaces_[i])
				{
					const auto it = oldFaceQ.find(fi);
					if (it == oldFaceQ.end()) continue;
					XFoam_Vector3D nNew; XFoam_Scalar aNew;
					faceNormalAndArea(fi, nNew, aNew);
					const auto& nOld = it->second.first;
					const XFoam_Scalar aOld = it->second.second;
					// face 退化（aNew == 0）算坏
					if (aNew <= 0) { bad = true; break; }
					const XFoam_Scalar dot = nOld.x() * nNew.x() + nOld.y() * nNew.y() + nOld.z() * nNew.z();
					if (dot < p_.minFaceNormalDot) { bad = true; break; }
					if (aOld > 0 && aNew < p_.minFaceAreaRatio * aOld) { bad = true; break; }
				}
				if (bad)
				{
					pm_.points[static_cast<std::size_t>(bndPoints_[i])] = snap[i];
					++stats.nRollback;
				}
			}
		}

		// 4) 统计
		for (std::size_t i = 0; i < bndPoints_.size(); ++i)
		{
			const XFoam_Scalar mv =
				(pm_.points[static_cast<std::size_t>(bndPoints_[i])] - snap[i]).mag();
			if (mv > 0)
			{
				++stats.nMoved;
				accMove += mv;
				if (mv > stats.maxMove) stats.maxMove = mv;
			}
		}
		stats.avgMove = (stats.nMoved > 0)
			? accMove / static_cast<XFoam_Scalar>(stats.nMoved)
			: static_cast<XFoam_Scalar>(0);

		if (p_.verbose)
		{
			std::cout << "  optimizer iter " << iter << ": moved=" << stats.nMoved
			          << "  avg=" << stats.avgMove
			          << "  max=" << stats.maxMove;
			if (p_.qualityCheck) std::cout << "  rollback=" << stats.nRollback;
			std::cout << std::endl;
		}
	}
	return stats;
}
