#ifndef XFoam_SnapParameters_H_
#define XFoam_SnapParameters_H_

// 对标 OpenFOAM-13: src/mesh/snappyHexMesh/snapParameters/snapParameters.H
// 仅承载 snappyHexMeshDict.snapControls 段的字段；snap 阶段算法 未移植。

#include "XFoam/utilities/xfoam_common.h"

class XFoam_Dictionary;

/*---------------------------------------------------------------------------*\
                       Class XFoam_SnapParameters Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_SnapParameters
{
public:
	XFoam_Label nSmoothPatch = 3;
	XFoam_Label nSmoothInternal = 0;
	XFoam_Scalar tolerance = 2.0;
	XFoam_Label nSolveIter = 30;
	XFoam_Label nRelaxIter = 5;
	XFoam_Label nFeatureSnapIter = 10;
	bool explicitFeatureSnap = false;
	bool implicitFeatureSnap = false;
	bool multiRegionFeatureSnap = false;

	XFoam_SnapParameters() = default;
	explicit XFoam_SnapParameters(const XFoam_Dictionary& snapDict);

	bool readDict(const XFoam_Dictionary& snapDict);
};

#endif
