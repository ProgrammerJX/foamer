#ifndef XFOAM_FEM_H
#define XFOAM_FEM_H

#include "XFoam/utilities/xfoam_hash.h"
#include "XFoam/utilities/xfoam_list.h"
#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

/// \brief XFoam_FEM：有限元模型（BDF / key 入口；当前 BDF 子集：NASTRAN 小字段 GRID、CTRIA3）。
/// \brief originId 为文件中的原始编号；列表下标为内部连续 id（0..n-1）。
class XFoam_API XFoam_FEM
{
public:
	XFoam_FEM();
	~XFoam_FEM();

	struct FemNode
	{
		int originId;
		XFoam_Vector3D position;
	};

	enum FemElementType
	{
		eFemElementTypeUnknown = 0,
		eFemElementTypeLine = 102,
		eFemElementTypeTriangle = 203,
		eFemElementTypeQuad = 204,
		eFemElementTypeTetra = 304,
		eFemElementTypePyramid = 305,
		eFemElementTypePrism = 306,
		eFemElementTypeHex = 308
	};

	struct FemElement
	{
		int originId;
		FemElementType type;
		XFoam_FixedList<int, 20> nodes;
	};

	struct FemSet
	{
		int originId;
		XFoam_List<int> elements;
	};

	XFoam_Label nNodes() const { return nodes_.size(); }
	XFoam_Label nElements() const { return elements_.size(); }

	void addNode(const XFoam_Vector3D& position, int originId = -1);
	void addLine(int* nodes, int originId = -1);
	void addTriangle(int* nodes, int originId = -1);
	void addQuad(int* nodes, int originId = -1);
	void addTetra(int* nodes, int originId = -1);
	void addHex(int* nodes, int originId = -1);
	void addElementToSet(int elementId, int setId);
	XFoam_Vector3D getNodePosition(int id) const;
	FemElement getElement(int id) const;

	void write(const XFoam_FileName& fileName);
	void read(const XFoam_FileName& fileName);
	void writeBdf(const XFoam_FileName& fileName);
	void readBdf(const XFoam_FileName& fileName);
	void readKey(const XFoam_FileName& fileName);
	void writeKey(const XFoam_FileName& fileName);

private:
	void clear();
	XFoam_Label findNodeInternalByOrigin_(int originId) const;

	XFoam_List<FemNode> nodes_;
	XFoam_List<FemElement> elements_;
	XFoam_Map<FemSet> sets_;
};

#endif
