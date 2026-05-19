#include "XFoam/snap/xfoam_refinementparameters.h"
#include "XFoam/utilities/xfoam_dictionary.h"

XFoam_RefinementParameters::XFoam_RefinementParameters(const XFoam_Dictionary& castellatedDict)
{
	(void)readDict(castellatedDict);
}

bool XFoam_RefinementParameters::readDict(const XFoam_Dictionary& dict)
{
	// 移植参考: OpenFOAM src/mesh/snappyHexMesh/refinementParameters/refinementParameters.C
	// 命名规范: foam_code.md
	// 移植规范: foam_code.md
	// 字段全部走 readIfPresent / lookupOrDefault，缺省值保留构造默认；
	// 至少识别到一个 key 才认为 dict 非空。
	bool any = false;

	auto pickInt = [&](const char* key, XFoam_Label& dst) {
		const XFoam_Word k(key);
		if (dict.readIfPresent(k, dst))
		{
			any = true;
		}
	};
	auto pickScalar = [&](const char* key, XFoam_Scalar& dst) {
		const XFoam_Word k(key);
		if (dict.readIfPresent(k, dst))
		{
			any = true;
		}
	};
	auto pickBool = [&](const char* key, bool& dst) {
		const XFoam_Word k(key);
		if (dict.readIfPresent(k, dst))
		{
			any = true;
		}
	};

	pickInt("maxLocalCells", maxLocalCells);
	pickInt("maxGlobalCells", maxGlobalCells);
	pickInt("minRefinementCells", minRefinementCells);
	pickInt("nCellsBetweenLevels", nCellsBetweenLevels);
	pickScalar("resolveFeatureAngle", resolveFeatureAngle);
	pickScalar("maxLoadUnbalance", maxLoadUnbalance);
	pickBool("allowFreeStandingZoneFaces", allowFreeStandingZoneFaces);

	const XFoam_Word locKey("locationInMesh");
	if (dict.readIfPresent(locKey, locationInMesh))
	{
		hasLocationInMesh = true;
		any = true;
	}
	return any;
}
