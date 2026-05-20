#ifndef XFoam_CMshRepatcher_H_
#define XFoam_CMshRepatcher_H_

// Repatch pm.patches[] using current pm.points 算出来的 face centroid。
//
// 背景：XFoam_CMshCartesianExtractor 在 perFacePatches=true 时按整数 grid 上
// 的 face centroid 查 brep.closestSubPatchId()，那会儿 boundary 还没贴表面，
// 容易把小 TpFace 附近的 boundary face 错分到隔壁 TpFace 名下。等 mapper（+
// edgeSnap）跑完，face centroid 已落到表面，再 query 一次准确度高得多。
//
// Repatcher 做：
//   1) 扫所有 boundary face，用 post-mapped 4-vertex 平均算 centroid
//   2) 查 brep.closestSubPatchId(centroid) 得 newSub
//   3) 按 newSub 重排 boundary 段（faces / owner）
//   4) 重写 pm.patches[]，可选 fillAllSubPatches 补空 TpFace
//
// 不动 internal face。

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"

#include <string>

class XFoam_BrepBase;

/*---------------------------------------------------------------------------*\
                    Class XFoam_CMshRepatcher Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshRepatcher
{
public:
	struct Params
	{
		std::string defaultPatchType  = "wall";
		bool        fillAllSubPatches = false;
		bool        verbose           = false;
	};

	struct Stats
	{
		XFoam_Label nBoundaryFaces  = 0;
		XFoam_Label nReassigned     = 0; ///< 之前 patch 与现在 patch 不一致的 face 数
		XFoam_Label nPatchesBefore  = 0;
		XFoam_Label nPatchesAfter   = 0;
		XFoam_Label nEmptyAdded     = 0;
	};

	XFoam_CMshRepatcher(
		XFoam_CMshPolyMeshGen& pm,
		const XFoam_BrepBase& brep,
		const Params& p);

	Stats repatch();

private:
	XFoam_CMshPolyMeshGen& pm_;
	const XFoam_BrepBase&  brep_;
	Params                 p_;
};

#endif // XFoam_CMshRepatcher_H_
