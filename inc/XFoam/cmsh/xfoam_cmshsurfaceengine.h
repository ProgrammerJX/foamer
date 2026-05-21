#ifndef XFoam_CMshSurfaceEngine_H_
#define XFoam_CMshSurfaceEngine_H_

// 对标 cfMesh: meshLibrary/utilities/surfaceTools/meshSurfaceEngine/
//              meshSurfaceEngine.{H,C}
//
// 集中缓存 polyMeshGen 的所有 boundary topology：
//   * boundary point 列表 + global→local 反查（bp）
//   * boundary face 列表 + 其 patchId / owner cell
//   * point→faces / point→points / point→patches
//   * 唯一 boundary edge 列表 + edge→faces / face→edges / edge→patches
//   * face centroid / normal / area；point 平均 normal
//
// 与 cfMesh 差异（精简）：
//   * 不带 parallel 字段（globalBoundaryPointLabel / bpAtProcs / ...）
//   * 不带 pointInFaces（slot 信息）—— 当前调用方不需要
//   * 一次性 eager build；topology 改变（例如 edgeInserter 给 face 加点）后
//     必须显式 invalidate() 再重建，cfMesh 是 lazy ptr clear 模式，这里
//     简化为重新构造一个新对象。
//
// 受益方：
//   * XFoam_CMshSurfaceMapper / EdgeExtractor：bndPoints + pointPoints
//   * XFoam_CMshMeshOptimizer：bndPoints + pointPoints + pointFaces +
//     faceNormals + faceAreas（quality check rollback）
//   * XFoam_CMshFeaturePinner：bndPoints（搜最近 mesh 点用）
//   * XFoam_CMshEdgeInserter：edges + edgeFaces + faceEdges（替换原本
//     edgeKey hash）
//   * XFoam_CMshMeshUntangler（下一步）：faceNormals + pointFaces 找翻转
//
// 用法：每个 pipeline.run() 顶端建一次，传给所有子工具复用；edgeInserter
// 之后必须重建（pm.faces 变了）。

#include "XFoam/cmsh/xfoam_cmshpolymeshgen.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

#include <vector>

class XFoam_API XFoam_CMshSurfaceEngine
{
public:
	struct Edge
	{
		int v0 = -1, v1 = -1;
		Edge() = default;
		Edge(int a, int b) : v0(a), v1(b) {}
	};

	/// 构造时立即 build 所有 cache（约 200ms / 1M boundary face）。pm 必须
	/// 是已经 extract 完毕的 polyMesh（internal + boundary face 分段对齐）。
	explicit XFoam_CMshSurfaceEngine(const XFoam_CMshPolyMeshGen& pm);

	// ---- boundary point ----
	/// 升序 unique boundary point global vid 列表。
	const std::vector<int>&            bndPoints() const  { return bndPoints_; }
	/// pm.points.size()；非 boundary = -1；boundary = 在 bndPoints_ 中的下标。
	const std::vector<int>&            bp() const         { return bp_; }
	XFoam_Label                        nBndPoints() const { return static_cast<XFoam_Label>(bndPoints_.size()); }

	// ---- boundary face ----
	/// pm 中 boundary face 的 global face id 列表 = [nInternalFaces, nFaces)。
	const std::vector<int>&            bndFaceIds() const { return bndFaceIds_; }
	/// 每个 boundary face 的 patchId（pm.patches 下标）。
	const std::vector<int>&            facePatch() const  { return facePatch_; }
	/// 每个 boundary face 的 owner cell。
	const std::vector<int>&            faceOwner() const  { return faceOwner_; }
	XFoam_Label                        nBndFaces() const  { return static_cast<XFoam_Label>(bndFaceIds_.size()); }

	// ---- point → faces / points / patches（bnd-local 下标） ----
	const std::vector<std::vector<int>>& pointFaces() const   { return pointFaces_; }
	const std::vector<std::vector<int>>& pointPoints() const  { return pointPoints_; }
	const std::vector<std::vector<int>>& pointPatches() const { return pointPatches_; }

	// ---- face 几何 ----
	const std::vector<XFoam_Vector3D>& faceCentres() const { return faceCentres_; }
	const std::vector<XFoam_Vector3D>& faceNormals() const { return faceNormals_; }
	const std::vector<XFoam_Scalar>&   faceAreas() const   { return faceAreas_; }
	const std::vector<XFoam_Vector3D>& pointNormals() const { return pointNormals_; }

	// ---- edges ----
	/// 唯一 boundary edge 列表（小端点编号在前）。
	const std::vector<Edge>&             edges() const     { return edges_; }
	/// edge id → 邻 boundary face id 列表（≤2 通常，feature 处可能 1）。
	const std::vector<std::vector<int>>& edgeFaces() const { return edgeFaces_; }
	/// boundary face id → edge id 列表，与 face.verts 同步：第 k 条 edge 是
	/// (verts[k], verts[(k+1)%n])。
	const std::vector<std::vector<int>>& faceEdges() const { return faceEdges_; }
	/// edge id → 邻面的唯一 patchId 列表（用于判定 cross-patch edge）。
	const std::vector<std::vector<int>>& edgePatches() const { return edgePatches_; }
	/// bnd-local point id → 入射 edge id 列表。
	const std::vector<std::vector<int>>& bpEdges() const   { return bpEdges_; }
	XFoam_Label                          nEdges() const    { return static_cast<XFoam_Label>(edges_.size()); }

private:
	const XFoam_CMshPolyMeshGen& pm_;

	std::vector<int> bndPoints_;
	std::vector<int> bp_;

	std::vector<int> bndFaceIds_;
	std::vector<int> facePatch_;
	std::vector<int> faceOwner_;

	std::vector<std::vector<int>> pointFaces_;
	std::vector<std::vector<int>> pointPoints_;
	std::vector<std::vector<int>> pointPatches_;

	std::vector<XFoam_Vector3D> faceCentres_;
	std::vector<XFoam_Vector3D> faceNormals_;
	std::vector<XFoam_Scalar>   faceAreas_;
	std::vector<XFoam_Vector3D> pointNormals_;

	std::vector<Edge>             edges_;
	std::vector<std::vector<int>> edgeFaces_;
	std::vector<std::vector<int>> faceEdges_;
	std::vector<std::vector<int>> edgePatches_;
	std::vector<std::vector<int>> bpEdges_;

	void buildBndPoints();
	void buildFaceMeta();
	void buildFaceGeometry();   // centre / normal / area
	void buildPointAdjacency(); // pointFaces / pointPoints / pointPatches / pointNormals
	void buildEdges();          // edges + faceEdges + edgeFaces + edgePatches + bpEdges
};

#endif // XFoam_CMshSurfaceEngine_H_
