#ifndef XFoam_LayerParameters_H_
#define XFoam_LayerParameters_H_

// 对标 OpenFOAM-13: src/mesh/snappyHexMesh/layerParameters/layerParameters.H
// 仅承载 snappyHexMeshDict.addLayersControls 段的标量字段；layer 阶段算法 未移植。
// per-patch layer 数量（dict 中 layers { … } 子段）目前以 XFoam_HashTable 收集，
// 还没有 wordRe 模式匹配，下一步在 driver 里再补。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_hash.h"

class XFoam_Dictionary;

/*---------------------------------------------------------------------------*\
                       Class XFoam_LayerParameters Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_LayerParameters
{
public:
	bool relativeSizes = true;
	XFoam_Scalar expansionRatio = 1.2;
	XFoam_Scalar firstLayerThickness = -1.0; // <0 表示未指定
	XFoam_Scalar finalLayerThickness = -1.0;
	XFoam_Scalar thickness = -1.0;
	XFoam_Scalar minThickness = 0.1;
	XFoam_Label nGrow = 0;
	XFoam_Scalar featureAngle = 60;
	XFoam_Scalar slipFeatureAngle = 30;
	XFoam_Label nRelaxIter = 3;
	XFoam_Label nSmoothSurfaceNormals = 1;
	XFoam_Label nSmoothNormals = 3;
	XFoam_Label nSmoothThickness = 10;
	XFoam_Scalar maxFaceThicknessRatio = 0.5;
	XFoam_Scalar maxThicknessToMedialRatio = 0.3;
	XFoam_Scalar minMedialAxisAngle = 90;
	XFoam_Label nBufferCellsNoExtrude = 0;
	XFoam_Label nLayerIter = 50;
	XFoam_Label nRelaxedIter = 20;

	// patch-name (literal, 还没接 wordRe) -> nSurfaceLayers
	XFoam_HashTable<XFoam_Label, XFoam_Word> perPatchLayers;

	XFoam_LayerParameters() = default;
	explicit XFoam_LayerParameters(const XFoam_Dictionary& addLayersDict);

	bool readDict(const XFoam_Dictionary& addLayersDict);
};

#endif
