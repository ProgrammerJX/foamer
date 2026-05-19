#include "XFoam/snap/xfoam_layerparameters.h"
#include "XFoam/utilities/xfoam_dictionary.h"

XFoam_LayerParameters::XFoam_LayerParameters(const XFoam_Dictionary& addLayersDict)
{
	(void)readDict(addLayersDict);
}

bool XFoam_LayerParameters::readDict(const XFoam_Dictionary& dict)
{
	// 移植参考: OpenFOAM src/mesh/snappyHexMesh/layerParameters/layerParameters.C
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

	pickBool("relativeSizes", relativeSizes);
	pickScalar("expansionRatio", expansionRatio);
	pickScalar("firstLayerThickness", firstLayerThickness);
	pickScalar("finalLayerThickness", finalLayerThickness);
	pickScalar("thickness", thickness);
	pickScalar("minThickness", minThickness);
	pickInt("nGrow", nGrow);
	pickScalar("featureAngle", featureAngle);
	pickScalar("slipFeatureAngle", slipFeatureAngle);
	pickInt("nRelaxIter", nRelaxIter);
	pickInt("nSmoothSurfaceNormals", nSmoothSurfaceNormals);
	pickInt("nSmoothNormals", nSmoothNormals);
	pickInt("nSmoothThickness", nSmoothThickness);
	pickScalar("maxFaceThicknessRatio", maxFaceThicknessRatio);
	pickScalar("maxThicknessToMedialRatio", maxThicknessToMedialRatio);
	pickScalar("minMedialAxisAngle", minMedialAxisAngle);
	pickInt("nBufferCellsNoExtrude", nBufferCellsNoExtrude);
	pickInt("nLayerIter", nLayerIter);
	pickInt("nRelaxedIter", nRelaxedIter);

	// layers { patchName { nSurfaceLayers N; } } 子段，无 wordRe 通配。
	const XFoam_Dictionary* layersSub = dict.subDictPtr(XFoam_Word("layers"));
	perPatchLayers.clear();
	if (layersSub)
	{
		any = true;
		const XFoam_WordList keys = layersSub->toc();
		for (XFoam_Label i = 0; i < keys.size(); ++i)
		{
			const XFoam_Word& patchName = keys[i];
			const XFoam_Dictionary* pp = layersSub->subDictPtr(patchName);
			if (!pp) continue;
			XFoam_Label n = 0;
			if (pp->readIfPresent(XFoam_Word("nSurfaceLayers"), n))
			{
				perPatchLayers.insert(patchName, n);
			}
		}
	}
	return any;
}
