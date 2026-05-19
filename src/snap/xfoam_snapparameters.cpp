#include "XFoam/snap/xfoam_snapparameters.h"
#include "XFoam/utilities/xfoam_dictionary.h"

XFoam_SnapParameters::XFoam_SnapParameters(const XFoam_Dictionary& snapDict)
{
	(void)readDict(snapDict);
}

bool XFoam_SnapParameters::readDict(const XFoam_Dictionary& dict)
{
	// 移植参考: OpenFOAM src/mesh/snappyHexMesh/snapParameters/snapParameters.C
	// 命名规范: foam_code.md
	// 移植规范: foam_code.md
	bool any = false;
	auto pickInt = [&](const char* k, XFoam_Label& d) {
		if (dict.readIfPresent(XFoam_Word(k), d)) any = true;
	};
	auto pickScalar = [&](const char* k, XFoam_Scalar& d) {
		if (dict.readIfPresent(XFoam_Word(k), d)) any = true;
	};
	auto pickBool = [&](const char* k, bool& d) {
		if (dict.readIfPresent(XFoam_Word(k), d)) any = true;
	};

	pickInt("nSmoothPatch", nSmoothPatch);
	pickInt("nSmoothInternal", nSmoothInternal);
	pickScalar("tolerance", tolerance);
	pickInt("nSolveIter", nSolveIter);
	pickInt("nRelaxIter", nRelaxIter);
	pickInt("nFeatureSnapIter", nFeatureSnapIter);
	pickBool("explicitFeatureSnap", explicitFeatureSnap);
	pickBool("implicitFeatureSnap", implicitFeatureSnap);
	pickBool("multiRegionFeatureSnap", multiRegionFeatureSnap);
	return any;
}
