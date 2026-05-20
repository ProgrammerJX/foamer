#include "XFoam/snap/xfoam_layerparameters.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_stream.h"

#include <memory>

namespace
{
// XFoam 的 dict 解析器对 `key { ... }` 形式会把内容存为 primitive stream（而不是
// sub-dictionary）。本工具按需把 stream 解析成 dict；与 xfoam_snappyhexmesh.cpp 里
// 的同名工具一致（移植自 OpenFOAM-13 dictionary.C 的子 dict 解析路径）。
std::unique_ptr<XFoam_Dictionary> xf_streamToDict(XFoam_ITstream& s)
{
	std::unique_ptr<XFoam_Dictionary> out;
	s.rewind();
	XFoam_Token t;
	s >> t;
	if (!t.good() || !(t == XFoam_Token::BEGIN_BLOCK))
	{
		return out;
	}
	XFoam_TokenList inner;
	int depth = 1;
	while (s >> t, t.good())
	{
		if (t == XFoam_Token::BEGIN_BLOCK || t == XFoam_Token::BEGIN_LIST) { ++depth; inner.append(t); }
		else if (t == XFoam_Token::END_BLOCK || t == XFoam_Token::END_LIST)
		{
			--depth;
			if (depth == 0) break;
			inner.append(t);
		}
		else inner.append(t);
	}
	out.reset(new XFoam_Dictionary());
	XFoam_ITstream is(XFoam_KeyType("__layerparams_inner__"), inner);
	is.rewind();
	if (!out->read(is)) out.reset();
	return out;
}
} // namespace

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
	// dict 实现里 `key { ... }` 既可能被存成 sub-dict（isDict==true），也可能被存成
	// primitive stream（OF-13 mainline 行为：dictionary.C read() 始终新建 PrimitiveEntry，
	// 由调用方按需 subDict）。两种形式都要支持。
	struct DictView { const XFoam_Dictionary* dict = nullptr; std::unique_ptr<XFoam_Dictionary> owned; };
	auto asSub = [](const XFoam_Dictionary& d, const XFoam_Word& k) -> DictView {
		DictView v;
		XFoam_Entry* e = const_cast<XFoam_Dictionary&>(d).lookupEntryPtr(k, false, true);
		if (!e) return v;
		if (e->isDict()) { v.dict = &e->dict(); return v; }
		if (e->isStream()) { v.owned = xf_streamToDict(e->stream()); v.dict = v.owned.get(); }
		return v;
	};

	perPatchLayers.clear();
	DictView lv = asSub(dict, XFoam_Word("layers"));
	if (lv.dict)
	{
		any = true;
		const XFoam_WordList keys = lv.dict->toc();
		for (XFoam_Label i = 0; i < keys.size(); ++i)
		{
			const XFoam_Word& patchName = keys[i];
			DictView pv = asSub(*lv.dict, patchName);
			if (!pv.dict) continue;
			XFoam_Label n = 0;
			if (pv.dict->readIfPresent(XFoam_Word("nSurfaceLayers"), n))
			{
				perPatchLayers.insert(patchName, n);
			}
		}
	}
	return any;
}
