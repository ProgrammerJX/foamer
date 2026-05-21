#include "XFoam/cmsh/xfoam_cmshmeshuntangler.h"

#include "XFoam/cmsh/xfoam_cmshsurfaceengine.h"
#include "XFoam/topo/xfoam_brep.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <unordered_set>

namespace
{
// 单 face 几何
struct FaceGeom { XFoam_Vector3D centre, normal; XFoam_Scalar area; };

void computeFaceGeom(
	const XFoam_CMshPolyMeshGen& pm,
	int                          faceIdx,
	FaceGeom&                    out)
{
	const auto& f = pm.faces[static_cast<std::size_t>(faceIdx)];
	const std::size_t nv = f.verts.size();
	out.centre = XFoam_Vector3D(0, 0, 0);
	out.normal = XFoam_Vector3D(0, 0, 0);
	out.area   = 0;
	if (nv < 3) return;
	XFoam_Vector3D c(0, 0, 0);
	for (int v : f.verts) c += pm.points[static_cast<std::size_t>(v)];
	c *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(nv));
	out.centre = c;
	XFoam_Vector3D N(0, 0, 0);
	for (std::size_t k = 0; k < nv; ++k)
	{
		const auto& a = pm.points[static_cast<std::size_t>(f.verts[k])];
		const auto& b = pm.points[static_cast<std::size_t>(f.verts[(k + 1) % nv])];
		N += XFoam_Vector3D(
			(a.y() - b.y()) * (a.z() + b.z()),
			(a.z() - b.z()) * (a.x() + b.x()),
			(a.x() - b.x()) * (a.y() + b.y()));
	}
	const XFoam_Scalar mag = N.mag();
	out.area = static_cast<XFoam_Scalar>(0.5) * mag;
	if (mag > 0) out.normal = N * (XFoam_Scalar(1) / mag);
}
} // namespace

XFoam_CMshMeshUntangler::XFoam_CMshMeshUntangler(
	XFoam_CMshPolyMeshGen& pm,
	const XFoam_BrepBase*  brep,
	const Params&          p)
	: pm_(pm), brep_(brep), p_(p), seExt_(nullptr)
{
	isFixed_.assign(pm_.points.size(), 0);
}

XFoam_CMshMeshUntangler::XFoam_CMshMeshUntangler(
	XFoam_CMshPolyMeshGen&         pm,
	const XFoam_BrepBase*          brep,
	const Params&                  p,
	const XFoam_CMshSurfaceEngine& se)
	: pm_(pm), brep_(brep), p_(p), seExt_(&se)
{
	isFixed_.assign(pm_.points.size(), 0);
}

void XFoam_CMshMeshUntangler::setFixedPoints(std::vector<int> ids)
{
	std::sort(ids.begin(), ids.end());
	ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
	isFixed_.assign(pm_.points.size(), 0);
	for (int pid : ids)
	{
		if (pid >= 0 && pid < static_cast<int>(isFixed_.size()))
			isFixed_[static_cast<std::size_t>(pid)] = 1;
	}
}

XFoam_CMshMeshUntangler::Stats XFoam_CMshMeshUntangler::untangle()
{
	Stats st;
	if (pm_.points.empty() || pm_.faces.empty()) return st;

	// SE：要么外部传入，要么自建（每轮不重建，因为 topology 不变）
	std::unique_ptr<XFoam_CMshSurfaceEngine> seOwn;
	if (!seExt_)
	{
		seOwn = std::make_unique<XFoam_CMshSurfaceEngine>(pm_);
	}
	const XFoam_CMshSurfaceEngine& se = seExt_ ? *seExt_ : *seOwn;

	const XFoam_Scalar areaScale = (p_.cellSizeHint > 0)
		? p_.cellSizeHint * p_.cellSizeHint
		: static_cast<XFoam_Scalar>(1.0);
	const XFoam_Scalar areaThresh = std::max<XFoam_Scalar>(
		p_.areaEps * areaScale,
		static_cast<XFoam_Scalar>(1e-30));

	const auto& bndIds     = se.bndFaceIds();
	const auto& bndPts     = se.bndPoints();
	const auto& bp         = se.bp();
	const auto& pointFaces = se.pointFaces();
	const auto& pointPts   = se.pointPoints();
	const std::size_t nBndF = bndIds.size();
	const std::size_t nBndP = bndPts.size();
	st.nFacesInitial = static_cast<XFoam_Label>(nBndF);

	std::vector<FaceGeom> fg(nBndF);
	for (std::size_t i = 0; i < nBndF; ++i)
		computeFaceGeom(pm_, bndIds[i], fg[i]);

	auto isTangled = [&](std::size_t bfi) -> bool {
		if (fg[bfi].area <= areaThresh) return true;
		// 邻面平均法向：通过共享 bnd-point 收集（不要求 faceFaces 域）
		const auto& f = pm_.faces[static_cast<std::size_t>(bndIds[bfi])];
		XFoam_Vector3D navg(0, 0, 0);
		int cnt = 0;
		for (int v : f.verts)
		{
			if (v < 0 || v >= static_cast<int>(bp.size())) continue;
			const int li = bp[static_cast<std::size_t>(v)];
			if (li < 0) continue;
			for (int nbf : pointFaces[static_cast<std::size_t>(li)])
			{
				if (nbf == static_cast<int>(bfi)) continue;
				navg += fg[static_cast<std::size_t>(nbf)].normal;
				++cnt;
			}
		}
		if (cnt == 0) return false;
		navg *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(cnt));
		const XFoam_Scalar mg = navg.mag();
		if (mg <= 0) return false;
		navg *= (XFoam_Scalar(1) / mg);
		const XFoam_Scalar dot =
			fg[bfi].normal.x() * navg.x() +
			fg[bfi].normal.y() * navg.y() +
			fg[bfi].normal.z() * navg.z();
		return dot < p_.tangleDot;
	};

	// 单 boundary point 的 "最小质量" 评估：min(area / cellSize^2 OR
	// normal·navg) over incident faces。统一为 [-1, 1]：area 项映射成
	// 1 - exp(-area/areaScale)；normal·navg 直接用。取最小值。
	auto pointQuality = [&](int li) -> XFoam_Scalar {
		const auto& inc = pointFaces[static_cast<std::size_t>(li)];
		if (inc.empty()) return 1; // 没有 face 就给个高分
		XFoam_Scalar worst = std::numeric_limits<XFoam_Scalar>::infinity();
		for (int bfi : inc)
		{
			const FaceGeom& gf = fg[static_cast<std::size_t>(bfi)];
			XFoam_Scalar q1 = (gf.area > 0)
				? static_cast<XFoam_Scalar>(1.0
				  - std::exp(-static_cast<double>(gf.area) / static_cast<double>(areaScale)))
				: static_cast<XFoam_Scalar>(-1);
			// 邻面平均法向（同 isTangled 的逻辑）
			const auto& f = pm_.faces[static_cast<std::size_t>(bndIds[static_cast<std::size_t>(bfi)])];
			XFoam_Vector3D navg(0, 0, 0); int cnt = 0;
			for (int v : f.verts)
			{
				if (v < 0 || v >= static_cast<int>(bp.size())) continue;
				const int li2 = bp[static_cast<std::size_t>(v)];
				if (li2 < 0) continue;
				for (int nbf : pointFaces[static_cast<std::size_t>(li2)])
				{
					if (nbf == bfi) continue;
					navg += fg[static_cast<std::size_t>(nbf)].normal;
					++cnt;
				}
			}
			XFoam_Scalar q2 = 1;
			if (cnt > 0)
			{
				navg *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(cnt));
				const XFoam_Scalar mg = navg.mag();
				if (mg > 0)
				{
					navg *= (XFoam_Scalar(1) / mg);
					q2 = gf.normal.x() * navg.x() + gf.normal.y() * navg.y() + gf.normal.z() * navg.z();
				}
			}
			const XFoam_Scalar q = std::min(q1, q2);
			if (q < worst) worst = q;
		}
		return worst;
	};

	for (int iter = 0; iter < p_.nIterations; ++iter)
	{
		// 1) 标 tangled face
		std::vector<char> ft(nBndF, 0);
		XFoam_Label nT = 0;
		for (std::size_t bfi = 0; bfi < nBndF; ++bfi)
		{
			if (isTangled(bfi)) { ft[bfi] = 1; ++nT; }
		}
		if (iter == 0) st.nFacesTangled0 = nT;
		st.nFacesTangledN = nT;
		if (nT == 0) break;

		// 2) tangled face → tangled point 集
		std::unordered_set<int> tangPts;
		for (std::size_t bfi = 0; bfi < nBndF; ++bfi)
		{
			if (!ft[bfi]) continue;
			const auto& f = pm_.faces[static_cast<std::size_t>(bndIds[bfi])];
			for (int v : f.verts)
			{
				if (v < 0 || v >= static_cast<int>(bp.size())) continue;
				const int li = bp[static_cast<std::size_t>(v)];
				if (li < 0) continue;
				if (!isFixed_.empty() && isFixed_[static_cast<std::size_t>(v)]) continue;
				tangPts.insert(li);
			}
		}
		st.nPointsTouched += static_cast<XFoam_Label>(tangPts.size());

		// 3) 对每个 tangled point 试候选
		for (int li : tangPts)
		{
			const int gv = bndPts[static_cast<std::size_t>(li)];
			const XFoam_Vector3D orig = pm_.points[static_cast<std::size_t>(gv)];
			const XFoam_Scalar qOrig = pointQuality(li);

			XFoam_Vector3D bestP = orig;
			XFoam_Scalar   bestQ = qOrig;

			auto tryPos = [&](const XFoam_Vector3D& candidate, bool tryProject) {
				XFoam_Vector3D candFinal = candidate;
				if (tryProject && brep_ && !brep_->empty())
				{
					XFoam_Vector3D q, n;
					brep_->closestPointAndNormal(candidate, q, n);
					if (std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()))
						candFinal = q;
				}
				// 评估：临时改 pm.points[gv] + 重算所有 incident face geom
				const XFoam_Vector3D savedP = pm_.points[static_cast<std::size_t>(gv)];
				std::vector<FaceGeom> savedG;
				const auto& inc = pointFaces[static_cast<std::size_t>(li)];
				savedG.reserve(inc.size());
				for (int bfi : inc) savedG.push_back(fg[static_cast<std::size_t>(bfi)]);
				pm_.points[static_cast<std::size_t>(gv)] = candFinal;
				for (int bfi : inc) computeFaceGeom(pm_, bndIds[static_cast<std::size_t>(bfi)], fg[static_cast<std::size_t>(bfi)]);
				const XFoam_Scalar qNew = pointQuality(li);
				if (qNew > bestQ)
				{
					bestQ = qNew;
					bestP = candFinal;
				}
				// 回滚（除非这是当前最佳，留到最后一并 apply）
				pm_.points[static_cast<std::size_t>(gv)] = savedP;
				for (std::size_t k = 0; k < inc.size(); ++k)
					fg[static_cast<std::size_t>(inc[k])] = savedG[k];
			};

			// 候选 (a) Laplacian target = nbr 平均
			XFoam_Vector3D lapTarget(0, 0, 0);
			int nNbr = 0;
			for (int ln : pointPts[static_cast<std::size_t>(li)])
			{
				const int gnv = bndPts[static_cast<std::size_t>(ln)];
				lapTarget += pm_.points[static_cast<std::size_t>(gnv)];
				++nNbr;
			}
			if (nNbr > 0)
			{
				lapTarget *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(nNbr));
				tryPos(lapTarget, false);
				if (p_.reproject) tryPos(lapTarget, true);
			}

			// 候选 (c) incident face centroid 平均
			XFoam_Vector3D cAvg(0, 0, 0);
			int nC = 0;
			for (int bfi : pointFaces[static_cast<std::size_t>(li)])
			{
				cAvg += fg[static_cast<std::size_t>(bfi)].centre;
				++nC;
			}
			if (nC > 0)
			{
				cAvg *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(nC));
				tryPos(cAvg, false);
				if (p_.reproject) tryPos(cAvg, true);
			}

			// 候选 (d) "good" nbr 平均（排除 tangled incident face 顶点）
			XFoam_Vector3D goodAvg(0, 0, 0);
			int nGood = 0;
			std::unordered_set<int> badPts;
			for (int bfi : pointFaces[static_cast<std::size_t>(li)])
			{
				if (!ft[static_cast<std::size_t>(bfi)]) continue;
				const auto& f = pm_.faces[static_cast<std::size_t>(bndIds[static_cast<std::size_t>(bfi)])];
				for (int v : f.verts)
				{
					if (v < 0 || v >= static_cast<int>(bp.size())) continue;
					const int li2 = bp[static_cast<std::size_t>(v)];
					if (li2 >= 0) badPts.insert(li2);
				}
			}
			for (int ln : pointPts[static_cast<std::size_t>(li)])
			{
				if (badPts.count(ln)) continue;
				const int gnv = bndPts[static_cast<std::size_t>(ln)];
				goodAvg += pm_.points[static_cast<std::size_t>(gnv)];
				++nGood;
			}
			if (nGood > 0)
			{
				goodAvg *= static_cast<XFoam_Scalar>(1.0 / static_cast<double>(nGood));
				tryPos(goodAvg, false);
				if (p_.reproject) tryPos(goodAvg, true);
			}

			// 应用最佳
			if (bestQ > qOrig)
			{
				pm_.points[static_cast<std::size_t>(gv)] = bestP;
				const XFoam_Scalar mv = (bestP - orig).mag();
				if (mv > st.maxMove) st.maxMove = mv;
				++st.nPointsImproved;
				const auto& inc2 = pointFaces[static_cast<std::size_t>(li)];
				for (int bfi : inc2) computeFaceGeom(pm_, bndIds[static_cast<std::size_t>(bfi)], fg[static_cast<std::size_t>(bfi)]);
			}
		}

		if (p_.verbose)
		{
			std::cout << "  untangler iter " << iter
			          << ": tangled=" << nT
			          << "  touched(cum)=" << st.nPointsTouched
			          << "  improved(cum)=" << st.nPointsImproved
			          << "  maxMove=" << st.maxMove << "\n";
		}
	}

	// 最后再扫一遍 tangle 数（看修复效果）
	XFoam_Label finalT = 0;
	for (std::size_t bfi = 0; bfi < nBndF; ++bfi)
	{
		computeFaceGeom(pm_, bndIds[bfi], fg[bfi]);
		if (isTangled(bfi)) ++finalT;
	}
	st.nFacesTangledN = finalT;
	return st;
}
