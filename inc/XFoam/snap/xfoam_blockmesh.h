#ifndef XFoam_blockMesh_H
#define XFoam_blockMesh_H

// 对标 OpenFOAM：src/mesh/blockMesh/blockMesh/blockMesh.H + blocks/block/blockList.H
// 未移植项在对应成员旁以「未移植」注释标出；实现见 xfoam_blockmesh.cpp（逻辑按 OF 源文件照抄，能落地的部分；并发已去除）。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/snap/xfoam_block.h"
#include "XFoam/snap/xfoam_blockvert.h"
#include "XFoam/snap/xfoam_blockedge.h"
#include "XFoam/snap/xfoam_blockface.h"
#include "XFoam/snap/xfoam_searchablesurface.h"
#include "XFoam/snap/xfoam_polymesh.h"
#include "XFoam/snap/xfoam_shape.h"
#include "XFoam/snap/xfoam_polypatch.h"

// XFoam_IStream / XFoam_OStream 为全局 typedef（见 xfoam_types.h）。
// XFoam_CellShapeList / XFoam_FaceListList 见 xfoam_shape.h；XFoam_PtrListDictionary 见 xfoam_dictionary.h。

/*---------------------------------------------------------------------------*\
                          Class XFoam_BlockMesh Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_BlockMesh
	: public XFoam_BlockList
{
	const XFoam_Dictionary& meshDict_;

	bool verboseOutput;

	bool checkFaceCorrespondence_;

	XFoam_SearchableSurfaceList geometry_;

	XFoam_Scalar scaleFactor_;

	XFoam_BlockVertList blockVertices_;

	XFoam_PointField vertices_;

	XFoam_BlockEdgeList edges_;

	XFoam_BlockFaceList faces_;

	XFoam_PolyMesh* topologyPtr_;

	XFoam_Label nPoints_;

	XFoam_Label nCells_;

	XFoam_LabelList blockOffsets_;

	XFoam_LabelList mergeList_;

	mutable XFoam_PointField points_;

	mutable XFoam_CellShapeList cells_;

	mutable XFoam_FaceListList patches_;

	// 由 createPatches() 与 patches_ 一起填充，避免上层调用 patchNames()/patchDicts() 时再
	// 解析一次 boundary。patchNames_ 在过去由 topology().boundary().names() 取，
	// 那只反映单 hex 占位拓扑的边界，多 block dict 下并不可靠；
	// 改成从 dict 直接缓存，多 block 下也是正确的。
	mutable XFoam_WordList patchNames_;
	mutable XFoam_WordList patchTypes_;

	template<class Source>
	void checkPatchLabels(
		const Source& source,
		const XFoam_Word& patchName,
		const XFoam_PointField& points,
		XFoam_List<XFoam_Face>& patchShapes) const;

	void readPatches(
		const XFoam_Dictionary& meshDescription,
		XFoam_FaceListList& tmpBlocksPatches,
		XFoam_WordList& patchNames,
		XFoam_WordList& patchTypes,
		XFoam_WordList& nbrPatchNames);

	void readBoundary(
		const XFoam_Dictionary& meshDescription,
		XFoam_WordList& patchNames,
		XFoam_FaceListList& tmpBlocksPatches,
		XFoam_PtrListDictionary<XFoam_Dictionary>& patchDicts) const;

	void readBoundaryFromSubDict(
		const XFoam_Dictionary& bnd,
		const XFoam_Dictionary& varDict,
		XFoam_WordList& patchNames,
		XFoam_FaceListList& tmpBlocksPatches,
		XFoam_PtrListDictionary<XFoam_Dictionary>& patchDicts) const;

	void readBoundaryFromPatchEntryStream(
		XFoam_ITstream& patchStream,
		const XFoam_Dictionary& meshDescription,
		const XFoam_Dictionary& varDict,
		XFoam_WordList& patchNames,
		XFoam_FaceListList& tmpBlocksPatches,
		XFoam_PtrListDictionary<XFoam_Dictionary>& patchDicts) const;

	void createCellShapes(XFoam_CellShapeList& tmpBlockCells);

	void defaultPatchError(const XFoam_Word& defaultPatchName, const XFoam_Dictionary& meshDescription) const;

	/// 忽略 meshPath 与 regionName 参数，仅保留 dictionary 参数。
	XFoam_PolyMesh* createTopology(
		const XFoam_Dictionary&,
		const XFoam_FileName& meshPath = "",
		const XFoam_Word& regionName = "");

	void check(const XFoam_PolyMesh&, const XFoam_Dictionary&) const;

	void calcMergeInfo();

	void calcMergeInfoFast();

	XFoam_List<XFoam_Face> createPatchFaces(const XFoam_PolyPatch& patchTopologyFaces) const;

	XFoam_Pair<XFoam_Scalar> xCellSizes(
		const XFoam_Block& b,
		const XFoam_PointField& blockPoints,
		XFoam_Label j,
		XFoam_Label k) const;

	XFoam_Pair<XFoam_Scalar> yCellSizes(
		const XFoam_Block& b,
		const XFoam_PointField& blockPoints,
		XFoam_Label i,
		XFoam_Label k) const;

	XFoam_Pair<XFoam_Scalar> zCellSizes(
		const XFoam_Block& b,
		const XFoam_PointField& blockPoints,
		XFoam_Label i,
		XFoam_Label j) const;

	void printCellSizeRange(const XFoam_Pair<XFoam_Scalar>& cellSizes) const;

	void printCellSizeRanges(int d, const XFoam_FixedList<XFoam_Pair<XFoam_Scalar>, 4>& cellSizes) const;

	void createPoints() const;
	void createCells() const;
	void createPatches() const;

public:
	static const char* const typeName;

	static bool checkBlockFaceOrientation;
	/// 忽略 meshPath 与 regionName 参数，仅保留 dictionary 参数。
	XFoam_BlockMesh(const XFoam_Dictionary&, const XFoam_FileName& meshPath = "", const XFoam_Word& regionNamef = "");

	XFoam_BlockMesh(const XFoam_BlockMesh&) = delete;

	virtual ~XFoam_BlockMesh();

	const XFoam_Dictionary& meshDict() const { return meshDict_; }

	const XFoam_SearchableSurfaceList& geometry() const { return geometry_; }

	const XFoam_PointField& vertices() const;

	const XFoam_PolyMesh& topology() const;

	const XFoam_BlockEdgeList& edges() const { return edges_; }

	const XFoam_BlockFaceList& faces() const { return faces_; }

	XFoam_Scalar scaleFactor() const;

	const XFoam_PointField& points() const;

	const XFoam_CellShapeList& cells() const;

	const XFoam_FaceListList& patches() const;

	XFoam_PtrListDictionary<XFoam_Dictionary> patchDicts() const;

	XFoam_WordList patchNames() const;

	/// 与 patches() 一一对应的 patch 类型（默认 "patch"）。
	/// 由 createPatches() 在解析 boundary 字典时缓存，无需重复解析。
	const XFoam_WordList& patchTypes() const;

	XFoam_Label numZonedBlocks() const;

	void verbose(const bool on = true);

	void writeTopology(XFoam_OStream&) const;

	void operator=(const XFoam_BlockMesh&) = delete;
};

template<class Source>
inline void XFoam_BlockMesh::checkPatchLabels(
	const Source& source,
	const XFoam_Word& patchName,
	const XFoam_PointField& points,
	XFoam_List<XFoam_Face>& patchFaces) const
{
	const XFoam_IOerrorLocation loc(static_cast<const XFoam_String&>(source.name()));
	for (XFoam_Label facei = 0; facei < patchFaces.size(); ++facei)
	{
		XFoam_Face& f = patchFaces[facei];
		if (f.size() == 2)
		{
			const XFoam_Label bi = f[0];
			const XFoam_Label fi = f[1];
			if (bi >= this->size())
			{
				XFoam_FatalIOErrorInFunction(loc)
					<< "Block index out of range for patch face (size " << f.size() << ")\n"
					<< " Number of blocks = " << this->size() << ", index = " << f[0] << '\n'
					<< " on patch " << patchName << ", face " << facei << XFoam_exit(XFoam_FatalIOError, 1);
			}
			if (!this->set(bi))
			{
				XFoam_FatalIOErrorInFunction(loc)
					<< "Block index " << bi << " not set for patch face on patch " << patchName << ", face "
					<< facei << XFoam_exit(XFoam_FatalIOError, 1);
			}
			const XFoam_List<XFoam_LabelList>& bf = (*this)[bi].blockShape().faces();
			if (fi < 0 || fi >= bf.size())
			{
				XFoam_FatalIOErrorInFunction(loc)
					<< "Block face index out of range for patch face (size " << f.size() << ")\n"
					<< " Number of block faces = " << bf.size() << ", index = " << f[1] << '\n'
					<< " on patch " << patchName << ", face " << facei << XFoam_exit(XFoam_FatalIOError, 1);
			}
			const XFoam_LabelList& gf = bf[fi];
			f = XFoam_Face(gf);
		}
		else
		{
			for (XFoam_Label fp = 0; fp < f.size(); ++fp)
			{
				if (f[fp] < 0)
				{
					XFoam_FatalIOErrorInFunction(loc)
						<< "Negative point label " << f[fp] << '\n'
						<< " on patch " << patchName << ", face " << facei << XFoam_exit(XFoam_FatalIOError, 1);
				}
				if (f[fp] >= points.size())
				{
					XFoam_FatalIOErrorInFunction(loc)
						<< "Point label " << f[fp] << " out of range 0.." << points.size() - 1 << '\n'
						<< " on patch " << patchName << ", face " << facei << XFoam_exit(XFoam_FatalIOError, 1);
				}
			}
		}
	}
}

#endif
