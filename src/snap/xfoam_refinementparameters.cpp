#include "XFoam/snap/xfoam_refinementparameters.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_stream.h"

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

	// locationsInMesh ((x y z) (x y z) ...)；OF-13 风格的多 region 锚点。
	// 不强行依赖 XFoam_List<XFoam_Vector3D> 的 operator>>（当前未实现 token 流读入），
	// 直接走 ITstream + token 的手写解析，结构与 vector::operator>> 一致：
	//   '(' { vector } ')'
	const XFoam_Word locsKey("locationsInMesh");
	if (dict.found(locsKey))
	{
		XFoam_ITstream& is = dict.lookup(locsKey);
		XFoam_Token tok;
		is >> tok;
		if (is.good() && tok.isPunctuation()
		    && tok.pToken() == XFoam_Token::BEGIN_LIST)
		{
			locationsInMesh.clear();
			while (true)
			{
				XFoam_Token peek;
				is >> peek;
				if (!is.good()) break;
				if (peek.isPunctuation() && peek.pToken() == XFoam_Token::END_LIST)
				{
					any = true;
					break;
				}
				is.putBack(peek);
				XFoam_Vector3D v;
				is >> v;
				if (!is.good()) break;
				locationsInMesh.push_back(v);
			}
		}
	}

	// 兼容旧 dict：单点 locationInMesh (x y z)；只有未配置 locationsInMesh 时才生效。
	if (locationsInMesh.empty())
	{
		const XFoam_Word locKey("locationInMesh");
		XFoam_Vector3D single;
		if (dict.readIfPresent(locKey, single))
		{
			locationsInMesh.push_back(single);
			any = true;
		}
	}
	return any;
}
