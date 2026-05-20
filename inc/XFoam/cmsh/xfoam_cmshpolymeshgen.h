#ifndef XFoam_CMshPolyMeshGen_H_
#define XFoam_CMshPolyMeshGen_H_

// 对标 cfMesh: meshLibrary/utilities/meshes/polyMeshGen/polyMeshGen.{H,C}
//
// 极简 polyMesh 容器 + ASCII 写出器。承载 cmsh 阶段在生成 polyMesh 时需要的
// 5 张表：points / faces / owner / neighbour / boundary。与 snap 的 polyMesh
// 写出 完全独立（snap 把这些表深埋在 SnappyHexMesh::run 内部的 std::vector
// 里）；cmsh 单独走一份是为了让两个 module 完全解耦，更便于将来分别迭代。
//
// 数据约束（对齐 OpenFOAM polyMesh 规范）：
//   * faces 排序：先所有 internal face（owner < neighbour，按 (owner, neighbour)
//     升序），再各 boundary patch 的 face（按 patch 顺序拼接）。
//   * owner.size() == faces.size()，neighbour.size() == nInternalFaces。
//   * Patch.startFace = patch 在 faces 数组中的起点；nFaces = patch face 数。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <string>
#include <vector>

/*---------------------------------------------------------------------------*\
                    Class XFoam_CMshPolyMeshGen Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_CMshPolyMeshGen
{
public:
	struct Face
	{
		std::vector<int> verts;  ///< polygon vertex indices, CCW with normal owner→neighbour
	};

	struct Patch
	{
		std::string name;
		std::string type = "patch";   ///< "wall" / "patch" / "symmetry" / ...
		int         startFace = 0;
		int         nFaces    = 0;
	};

	std::vector<XFoam_Vector3D> points;     ///< 全局点表，唯一
	std::vector<Face>           faces;      ///< 内 + boundary 按 polyMesh 排序
	std::vector<int>            owner;      ///< faces.size()
	std::vector<int>            neighbour;  ///< nInternalFaces
	std::vector<Patch>          patches;    ///< boundary patch metadata
	int                         nCells = 0;

	XFoam_CMshPolyMeshGen() = default;

	int nFaces() const { return static_cast<int>(faces.size()); }
	int nInternalFaces() const { return static_cast<int>(neighbour.size()); }
	int nBoundaryFaces() const { return nFaces() - nInternalFaces(); }

	/// 写出 polyMesh 5 件套 + 一个空 .foam 占位文件：
	///   <dir>/points
	///   <dir>/faces
	///   <dir>/owner
	///   <dir>/neighbour
	///   <dir>/boundary
	/// dir 不存在会被创建。返回 false 若任一文件写失败。
	bool writeToDir(const XFoam_String& dir) const;
};

#endif // XFoam_CMshPolyMeshGen_H_
