#include "XFoam/cmsh/xfoam_cmshoctreecreator.h"

#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/utilities/xfoam_error.h"

#include <algorithm>
#include <cmath>
#include <iostream>

XFoam_CMshOctreeCreator::XFoam_CMshOctreeCreator(
	const XFoam_BoundBox& rootBox,
	const Params&         p)
	: rootBox_(rootBox)
	, p_(p)
{}

XFoam_CMshOctreeCreator& XFoam_CMshOctreeCreator::addSurfaceRefine(
	const XFoam_BrepBase& s, int level)
{
	surfs_.push_back(SurfaceRefine{&s, level});
	return *this;
}

XFoam_CMshOctreeCreator& XFoam_CMshOctreeCreator::addRegionRefine(
	const XFoam_BoundBox& region, int level)
{
	regions_.push_back(RegionRefine{region, level});
	return *this;
}

XFoam_CMshOctreeCreator& XFoam_CMshOctreeCreator::addObjectRefine(
	std::unique_ptr<XFoam_CMshObjRefine> obj)
{
	if (obj) objects_.push_back(std::move(obj));
	return *this;
}

XFoam_AutoPtr<XFoam_CMshOctree> XFoam_CMshOctreeCreator::build() const
{
	if (surfs_.empty() || surfs_.front().brep == nullptr)
	{
		throw XFoam_Error(XFoam_String(
			"XFoam_CMshOctreeCreator::build: needs at least one SurfaceRefine "
			"(the primary surface for inside/outside classification)."));
	}

	// 1) inflate root
	XFoam_BoundBox box = rootBox_;
	if (p_.inflateRoot)
	{
		const XFoam_Scalar diag = box.mag();
		const XFoam_Scalar pad  = diag * p_.rootInflate;
		box.min() = XFoam_Vector3D(box.min().x() - pad,
		                           box.min().y() - pad,
		                           box.min().z() - pad);
		box.max() = XFoam_Vector3D(box.max().x() + pad,
		                           box.max().y() + pad,
		                           box.max().z() + pad);
	}

	// 2) primary 几何 → root
	XFoam_AutoPtr<XFoam_CMshOctree> oct(
		new XFoam_CMshOctree(*surfs_.front().brep, box));

	// 3) 反推 baseLevel：maxRootSpan / maxCellSize 取 log2 上取整
	const XFoam_Vector3D s = box.span();
	const XFoam_Scalar   sMax = std::max(s.x(), std::max(s.y(), s.z()));
	int baseLevel = 0;
	if (p_.maxCellSize > 0 && sMax > 0)
	{
		const double ratio = static_cast<double>(sMax) / static_cast<double>(p_.maxCellSize);
		if (ratio > 1.0)
		{
			baseLevel = static_cast<int>(std::ceil(std::log(ratio) / std::log(2.0)));
		}
	}
	baseLevel = std::min(baseLevel, p_.maxLevel);
	if (baseLevel > 0)
	{
		oct().refineUniform(baseLevel);
	}

	// 3b) fitFeatures：可选根据 brep.minFeatureLength() 提 surface level
	std::vector<SurfaceRefine> surfsResolved = surfs_;
	if (p_.fitFeatures)
	{
		for (auto& sr : surfsResolved)
		{
			if (!sr.brep || sr.brep->empty()) continue;
			const XFoam_Scalar minFeat = sr.brep->minFeatureLength();
			if (minFeat <= 0) continue;
			const XFoam_Scalar wanted = minFeat * p_.fitFeaturesSafety;
			if (wanted <= 0 || sMax <= 0) continue;
			const double ratio = static_cast<double>(sMax) / static_cast<double>(wanted);
			if (ratio <= 1.0) continue;
			const int needed = static_cast<int>(std::ceil(std::log(ratio) / std::log(2.0)));
			const int bumpCap = sr.level + p_.fitFeaturesMaxLevelBump;
			const int newLv = std::min({needed, bumpCap, p_.maxLevel});
			if (newLv > sr.level)
			{
				std::cout << "  fitFeatures: bump surface level "
				          << sr.level << " -> " << newLv
				          << "  (minFeat=" << minFeat
				          << ", wanted cellSize=" << wanted
				          << ", needed=" << needed
				          << ", bumpCap=" << bumpCap
				          << ", maxLevel=" << p_.maxLevel << ")\n";
				sr.level = newLv;
			}
		}
	}

	// 4) surface refines（每条 SurfaceRefine 独立刷一遍；level clamp 到 maxLevel）
	//    注意：refineToSurface 只看 oct.surface_（= primary）；MVP 阶段同一棵
	//    octree 不支持多 brep 各自加密。要做就给每个 brep 单独建 octree 再合并。
	for (const auto& sr : surfsResolved)
	{
		const int lv = std::min(sr.level, p_.maxLevel);
		if (lv <= baseLevel) continue;
		oct().refineToSurface(lv);
	}

	// 4b) perFaceFitFeatures：按每张 TopoDS_Face 自己 bbox 反推额外 level
	//     只对 primary surface 生效（与 oct.surface_ 一致才能用 closestSubPatchId）。
	if (p_.perFaceFitFeatures && !surfsResolved.empty())
	{
		const auto& sr  = surfsResolved.front();
		const auto* bp  = sr.brep;
		if (bp && !bp->empty())
		{
			const XFoam_Label nSub = bp->nSubPatches();
			if (nSub > 0)
			{
				const int baseSurfLv = std::min(sr.level, p_.maxLevel);
				const int bumpCap    = std::min(baseSurfLv + p_.fitFeaturesMaxLevelBump, p_.maxLevel);
				std::vector<int> perFaceLevel(static_cast<std::size_t>(nSub), baseSurfLv);
				int nBumped     = 0;
				int nCapped     = 0;     ///< 想要的 level 被 bumpCap/maxLevel 截断
				int nSkipped    = 0;     ///< degenerate / 无效 bbox 跳过
				int maxRequest  = baseSurfLv; ///< 真正想要的最大 level（截断前）
				int nWonByEdge  = 0;     ///< 控制边长被最短 feature edge 决定，不是 bbox
				std::vector<int> levelHist(static_cast<std::size_t>(p_.maxLevel + 1), 0);
				for (XFoam_Label s = 0; s < nSub; ++s)
				{
					const XFoam_BoundBox b = bp->subPatchBounds(s);
					if (b.max().x() < b.min().x()) { ++nSkipped; continue; }
					const XFoam_Vector3D fs = b.span();
					// 取 bbox 三轴最短边
					XFoam_Scalar minSide = std::min({fs.x(), fs.y(), fs.z()});
					if (minSide <= 0) { ++nSkipped; continue; }
					// 同时看该 face 自己的最短 feature edge；两者取小（cellSize
					// 必须同时小到能保 bbox 也能保 feature）。
					const XFoam_Scalar minEdge = bp->subPatchMinFeatureLength(s);
					XFoam_Scalar scale = minSide;
					if (minEdge > 0 && minEdge < scale) { scale = minEdge; ++nWonByEdge; }
					const XFoam_Scalar wanted = scale * p_.perFaceFitFeaturesSafety;
					if (wanted <= 0 || sMax <= 0) { ++nSkipped; continue; }
					const double ratio = static_cast<double>(sMax) / static_cast<double>(wanted);
					if (ratio <= 1.0) continue;
					const int needed = static_cast<int>(std::ceil(std::log(ratio) / std::log(2.0)));
					if (needed > maxRequest) maxRequest = needed;
					const int target = std::min({needed, bumpCap, p_.maxLevel});
					if (needed > target) ++nCapped;
					if (target > perFaceLevel[static_cast<std::size_t>(s)])
					{
						perFaceLevel[static_cast<std::size_t>(s)] = target;
						++nBumped;
					}
				}
				for (XFoam_Label s = 0; s < nSub; ++s)
				{
					const int lv = perFaceLevel[static_cast<std::size_t>(s)];
					if (lv >= 0 && lv <= p_.maxLevel) ++levelHist[static_cast<std::size_t>(lv)];
				}
				if (nBumped > 0)
				{
					std::cout << "  perFaceFitFeatures: bumped " << nBumped
					          << "/" << nSub << " TopoDS_Face levels (cap="
					          << bumpCap << ", max wanted=" << maxRequest;
					if (nCapped > 0)    std::cout << ", " << nCapped << " capped";
					if (nWonByEdge > 0) std::cout << ", " << nWonByEdge << " by minEdge";
					if (nSkipped > 0)   std::cout << ", " << nSkipped << " degenerate skipped";
					std::cout << ")\n  per-face level histogram: ";
					for (int lv = 0; lv <= p_.maxLevel; ++lv)
					{
						if (levelHist[static_cast<std::size_t>(lv)] == 0) continue;
						std::cout << "L" << lv << "=" << levelHist[static_cast<std::size_t>(lv)] << " ";
					}
					std::cout << "\n";
					oct().refineToSurfacePerFace(perFaceLevel, p_.maxLevel);
				}
				else
				{
					std::cout << "  perFaceFitFeatures: no TopoDS_Face required "
					             "bump above surfLevel=" << baseSurfLv << "\n";
				}
			}
		}
	}

	// 5) region refines
	for (const auto& rr : regions_)
	{
		const int lv = std::min(rr.level, p_.maxLevel);
		oct().refineRegion(rr.box, lv);
	}

	// 5b) object refines（box / sphere / cone / 任意 XFoam_CMshObjRefine 子类）
	for (const auto& op : objects_)
	{
		if (!op) continue;
		const int lv = std::min(op->level, p_.maxLevel);
		oct().refineByObject(*op, lv);
	}

	// 6) inside/outside 分类
	oct().classifyLeaves();

	return oct;
}
