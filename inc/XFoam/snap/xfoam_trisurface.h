#ifndef XFoam_TriSurface_H_
#define XFoam_TriSurface_H_

// 三角面片表面：STL 读取 + 最近点 / 距离 / 内外判定 / 包围盒相交。
// 用于 snappyHexMesh 阶段对几何做切割与边界 snap。
//
// 对标 OpenFOAM 的 Foam::triSurface（src/triSurface），但极度简化：
//   - 仅 ascii 与 binary STL；不读 FreeSurfer/nas/ftr 等
//   - 不建 octree / 不建 BVH，所有查询走线性扫描（小 STL 够用）
//   - 不区分 region；整个表面就一个 patch
//   - 法向量直接取自 STL header 中的 facet normal；不重算
// 适合教学/演示规模（几千三角面），不适合工业级网格。

#include "XFoam/utilities/xfoam_boundbox.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <string>
#include <vector>

class XFoam_API XFoam_TriSurface
{
public:
	struct Triangle
	{
		XFoam_Vector3D v0, v1, v2;
		XFoam_Vector3D normal; // 单位化；构造时计算
		XFoam_BoundBox bbox;   // 缓存避免重复 min/max
	};

	XFoam_TriSurface() = default;

	/// 自动按 ascii / binary 嗅探。返回 true 即至少读到 1 个三角面。
	bool read(const std::string& path);

	bool readAsciiSTL(const std::string& path);
	bool readBinarySTL(const std::string& path);

	XFoam_Label size() const { return static_cast<XFoam_Label>(tris_.size()); }
	const std::vector<Triangle>& tris() const { return tris_; }
	const XFoam_BoundBox& bounds() const { return bounds_; }
	bool empty() const { return tris_.empty(); }

	/// 距离查询：返回 p 到所有三角面最近距离的平方根；line scan，O(N)。
	XFoam_Scalar distance(const XFoam_Vector3D& p) const;

	/// 最近点 + 法向。法向取自承载三角面的 facet 法向（带 sign，朝外）。
	void closestPointAndNormal(
		const XFoam_Vector3D& p,
		XFoam_Vector3D& outClosest,
		XFoam_Vector3D& outNormal) const;

	/// 沿 +X 方向从 p 射线交叉次数。奇/偶判定 inside / outside（仅对水密 STL 有效）。
	int rayCountPlusX(const XFoam_Vector3D& p) const;

	/// 包围盒粗筛：盒内/相交 → true。用于细化阶段的"和表面相交的 cell"快筛。
	bool boxIntersects(const XFoam_BoundBox& box) const;

	/// 是否包含点：基于 +X 射线奇数次穿过判定（要求水密 STL）。
	/// 在 p 的 y/z 上加一个相对包围盒的小抖动，避免射线穿过顶点 / 棱时（如球心
	/// (0,0,0) 对 +X 等于 R 处的等分顶点）出现奇偶判定不稳。
	bool contains(const XFoam_Vector3D& p) const;

private:
	std::vector<Triangle> tris_;
	XFoam_BoundBox bounds_;

	void rebuildBounds();
	void addTriangle(
		const XFoam_Vector3D& a,
		const XFoam_Vector3D& b,
		const XFoam_Vector3D& c,
		const XFoam_Vector3D* explicitNormal);
};

#endif
