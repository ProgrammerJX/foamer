#ifndef XFoam_Shape_H_
#define XFoam_Shape_H_

// 体元：XFoam_CellModel（cellModel.H）与 XFoam_CellShape（cellShape.H / cellShapeI.H）。
// 网格单元面表：XFoam_Cell（cell.H / cell.C / oppositeCellFace.C）。
// 网格边：XFoam_Edge（meshShapes/edge/edge.H / edgeI.H）。

#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/snap/xfoam_line.h"

/// 两个顶点全局编号构成的边，语义对齐 OpenFOAM Foam::edge（edge.H / edgeI.H）。
/// 未移植：Istream 构造、Hash<> 特化、contiguous<>。
class XFoam_Edge : public XFoam_FixedList<XFoam_Label, 2>
{
public:
	static constexpr const char* typeName = "edge";

	XFoam_Edge() = default;

	XFoam_Edge(const XFoam_Label a, const XFoam_Label b)
	{
		start() = a;
		end() = b;
	}

	explicit XFoam_Edge(const XFoam_FixedList<XFoam_Label, 2>& e)
	{
		start() = e[0];
		end() = e[1];
	}

	XFoam_Label start() const { return operator[](0); }
	XFoam_Label& start() { return operator[](0); }

	XFoam_Label end() const { return operator[](1); }
	XFoam_Label& end() { return operator[](1); }

	bool connected(const XFoam_Edge& a) const
	{
		return start() == a.start() || start() == a.end() || end() == a.start() || end() == a.end();
	}

	// 共享顶点；无公共顶点时返回 -1。
	XFoam_Label commonVertex(const XFoam_Edge& a) const
	{
		if (start() == a.start() || start() == a.end())
		{
			return start();
		}
		if (end() == a.start() || end() == a.end())
		{
			return end();
		}
		return -1;
	}

	/// 已知一端顶点标号，返回另一端；a 不在边上时返回 -1。
	XFoam_Label otherVertex(const XFoam_Label a) const
	{
		if (a == start())
		{
			return end();
		}
		if (a == end())
		{
			return start();
		}
		return -1;
	}

	void flip() { std::swap(operator[](0), operator[](1)); }

	XFoam_Edge reverseEdge() const { return XFoam_Edge(end(), start()); }

	XFoam_Vector3D centre(const XFoam_UList<XFoam_Vector3D>& p) const
	{
		return 0.5 * (p[start()] + p[end()]);
	}

	XFoam_Vector3D vec(const XFoam_UList<XFoam_Vector3D>& p) const { return p[end()] - p[start()]; }

	XFoam_Scalar mag(const XFoam_UList<XFoam_Vector3D>& p) const
	{
		return static_cast<XFoam_Scalar>(vec(p).mag());
	}

	XFoam_LinePointRef line(const XFoam_UList<XFoam_Vector3D>& p) const
	{
		return XFoam_LinePointRef(p[start()], p[end()]);
	}

	/// 0：不同边；+1：同向；-1：反向同边。
	static int compare(const XFoam_Edge& a, const XFoam_Edge& b)
	{
		if (a.start() == b.start() && a.end() == b.end())
		{
			return 1;
		}
		if (a.start() == b.end() && a.end() == b.start())
		{
			return -1;
		}
		return 0;
	}
};

/// 与 OpenFOAM 一致：无向相同（同向或反向）视为相等。
inline bool operator==(const XFoam_Edge& a, const XFoam_Edge& b)
{
	return XFoam_Edge::compare(a, b) != 0;
}

inline bool operator!=(const XFoam_Edge& a, const XFoam_Edge& b)
{
	return XFoam_Edge::compare(a, b) == 0;
}

/// 网格面：顶点全局编号列表，对齐 OpenFOAM Foam::face（face.H / faceI.H / face.C）。
/// 未移植：triFace 构造、Istream、ray / intersection / nearestPointClassify、contactSphere、
/// areaInContact、inertia、Hash / offsetOp 等。faceTemplates.C 中模板见 xfoam_shape_templates.h。
class XFoam_Face : public XFoam_LabelList
{
public:
	enum ProxType
	{
		proxNone = 0,
		proxPoint = 1,
		proxEdge = 2
	};

	XFoam_Face() = default;

	explicit XFoam_Face(const XFoam_Label n)
		: XFoam_LabelList(n, static_cast<XFoam_Label>(-1))
	{}

	explicit XFoam_Face(const XFoam_UList<XFoam_Label>& lst)
		: XFoam_LabelList(lst.cbegin(), lst.cend())
	{}

	explicit XFoam_Face(const XFoam_LabelList& lst)
		: XFoam_LabelList(lst)
	{}

	explicit XFoam_Face(XFoam_LabelList&& lst) noexcept
		: XFoam_LabelList(XFoam_move(lst))
	{}

	XFoam_Face(std::initializer_list<XFoam_Label> lst)
		: XFoam_LabelList(lst)
	{}

	void operator=(XFoam_LabelList&& l)
	{
		XFoam_LabelList::operator=(XFoam_move(l));
	}

	template<class PointField>
	static XFoam_Vector3D area(const PointField& ps);

	template<class PointField>
	static XFoam_Vector3D centre(const PointField& ps);

	template<class PointField>
	static XFoam_Tuple2<XFoam_Vector3D, XFoam_Vector3D> areaAndCentre(const PointField& ps);

	template<class PointField>
	static XFoam_Tuple2<XFoam_Vector3D, XFoam_Vector3D> areaAndCentreStabilised(const PointField& ps);

	template<class Type>
	Type average(const XFoam_UList<XFoam_Vector3D>& ps, const XFoam_UList<Type>& fld) const;

	XFoam_API XFoam_Label collapse();
	void flip()
	{
		const XFoam_Label n = size();
		if (n > 2)
		{
			for (XFoam_Label i = 1; i < (n + 1) / 2; ++i)
			{
				std::swap(operator[](i), operator[](n - i));
			}
		}
	}

	XFoam_List<XFoam_Vector3D> points(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		XFoam_List<XFoam_Vector3D> p(size());
		for (XFoam_Label i = 0; i < size(); ++i)
		{
			p[i] = meshPoints[operator[](i)];
		}
		return p;
	}

	XFoam_Vector3D centre(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		return XFoam_Face::centre<XFoam_List<XFoam_Vector3D>>(points(meshPoints));
	}

	XFoam_Vector3D area(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		return XFoam_Face::area<XFoam_List<XFoam_Vector3D>>(points(meshPoints));
	}

	XFoam_Scalar mag(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		return static_cast<XFoam_Scalar>(area(meshPoints).mag());
	}

	XFoam_Vector3D normal(const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		return XFoam_normalised(area(meshPoints));
	}

	XFoam_API XFoam_Face reverseFace() const;

	XFoam_API XFoam_Label which(XFoam_Label globalIndex) const;

	XFoam_Label nextLabel(const XFoam_Label i) const { return operator[](fcIndex_(i)); }

	XFoam_Label prevLabel(const XFoam_Label i) const { return operator[](rcIndex_(i)); }

	XFoam_API XFoam_Scalar sweptVol(
		const XFoam_UList<XFoam_Vector3D>& oldPoints,
		const XFoam_UList<XFoam_Vector3D>& newPoints) const;

	XFoam_Label nEdges() const { return size(); }

	XFoam_API XFoam_List<XFoam_Edge> edges() const;

	XFoam_Edge faceEdge(const XFoam_Label n) const
	{
		return XFoam_Edge(operator[](n), operator[](fcIndex_(n)));
	}

	XFoam_API int edgeDirection(const XFoam_Edge& e) const;

	XFoam_Label nTriangles() const { return size() - 2; }

	static XFoam_API int compare(const XFoam_Face& a, const XFoam_Face& b);
	static XFoam_API bool sameVertices(const XFoam_Face& a, const XFoam_Face& b);

private:
	XFoam_Label fcIndex_(const XFoam_Label i) const
	{
		const XFoam_Label n = size();
		return n ? ((i + 1) % n) : 0;
	}

	XFoam_Label rcIndex_(const XFoam_Label i) const
	{
		const XFoam_Label n = size();
		return n ? ((i - 1 + n) % n) : 0;
	}
};

inline bool operator==(const XFoam_Face& a, const XFoam_Face& b)
{
	return XFoam_Face::compare(a, b) != 0;
}

inline bool operator!=(const XFoam_Face& a, const XFoam_Face& b)
{
	return XFoam_Face::compare(a, b) == 0;
}

XFoam_API XFoam_Label XFoam_longestEdge(const XFoam_Face& f, const XFoam_UList<XFoam_Vector3D>& pts);

/// 棱柱单元中与主面相对的对面：携带主/从面在网格面表中的索引（对齐 OpenFOAM oppositeFace）。
/// oppositeIndex() \< 0 表示未找到或单元非棱柱（与 OpenFOAM opposingFace 失败约定一致）。
class XFoam_OppositeFace : public XFoam_Face
{
	XFoam_Label masterIndex_;
	XFoam_Label oppositeIndex_;

public:
	XFoam_OppositeFace()
		: XFoam_Face()
		, masterIndex_(-1)
		, oppositeIndex_(-1)
	{}

	XFoam_OppositeFace(XFoam_Face f, const XFoam_Label masterI, const XFoam_Label oppI)
		: XFoam_Face(XFoam_move(f))
		, masterIndex_(masterI)
		, oppositeIndex_(oppI)
	{}

	XFoam_Label masterIndex() const { return masterIndex_; }
	XFoam_Label oppositeIndex() const { return oppositeIndex_; }
	bool found() const { return oppositeIndex_ >= 0; }
};

/// 网格单元：存储属于该单元的**面在网格面表中的下标**列表，对齐 OpenFOAM Foam::cell（cell.H / cell.C / oppositeCellFace.C）。
/// 未移植：Istream 构造。
class XFoam_Cell : public XFoam_LabelList
{
public:
	static constexpr const char* typeName = "cell";

	XFoam_Cell() = default;

	explicit XFoam_Cell(const XFoam_Label n)
		: XFoam_LabelList(n, static_cast<XFoam_Label>(-1))
	{}

	explicit XFoam_Cell(const XFoam_UList<XFoam_Label>& lst)
		: XFoam_LabelList(lst.cbegin(), lst.cend())
	{}

	explicit XFoam_Cell(const XFoam_LabelList& lst)
		: XFoam_LabelList(lst)
	{}

	explicit XFoam_Cell(XFoam_LabelList&& lst) noexcept
		: XFoam_LabelList(XFoam_move(lst))
	{}

	XFoam_Cell(std::initializer_list<XFoam_Label> lst)
		: XFoam_LabelList(lst)
	{}

	void operator=(XFoam_LabelList&& l)
	{
		XFoam_LabelList::operator=(XFoam_move(l));
	}

	XFoam_Label nFaces() const { return size(); }

	static XFoam_API XFoam_LabelList labels(const XFoam_Cell& c, const XFoam_UList<XFoam_Face>& meshFaces);
	static XFoam_API XFoam_List<XFoam_Vector3D> points(
		const XFoam_Cell& c,
		const XFoam_UList<XFoam_Face>& meshFaces,
		const XFoam_UList<XFoam_Vector3D>& meshPoints);
	static XFoam_API XFoam_List<XFoam_Edge> edges(
		const XFoam_Cell& c, const XFoam_UList<XFoam_Face>& meshFaces);

	XFoam_LabelList labels(const XFoam_UList<XFoam_Face>& meshFaces) const
	{
		return XFoam_Cell::labels(*this, meshFaces);
	}

	XFoam_List<XFoam_Vector3D> points(
		const XFoam_UList<XFoam_Face>& meshFaces,
		const XFoam_UList<XFoam_Vector3D>& meshPoints) const
	{
		return XFoam_Cell::points(*this, meshFaces, meshPoints);
	}

	XFoam_List<XFoam_Edge> edges(const XFoam_UList<XFoam_Face>& meshFaces) const
	{
		return XFoam_Cell::edges(*this, meshFaces);
	}

	static XFoam_API XFoam_Label opposingFaceLabel(
		const XFoam_Cell& c,
		XFoam_Label masterFaceLabel,
		const XFoam_UList<XFoam_Face>& meshFaces);
	static XFoam_API XFoam_OppositeFace opposingFace(
		const XFoam_Cell& c,
		XFoam_Label masterFaceLabel,
		const XFoam_UList<XFoam_Face>& meshFaces);

	XFoam_Label opposingFaceLabel(
		XFoam_Label masterFaceLabel, const XFoam_UList<XFoam_Face>& meshFaces) const
	{
		return XFoam_Cell::opposingFaceLabel(*this, masterFaceLabel, meshFaces);
	}

	XFoam_OppositeFace opposingFace(
		XFoam_Label masterFaceLabel, const XFoam_UList<XFoam_Face>& meshFaces) const
	{
		return XFoam_Cell::opposingFace(*this, masterFaceLabel, meshFaces);
	}

	/// 面心面积加权估计 + 棱锥分解重心（见 OpenFOAM cell::centre）。
	static XFoam_API XFoam_Vector3D centre(
		const XFoam_Cell& c,
		const XFoam_UList<XFoam_Vector3D>& meshPoints,
		const XFoam_UList<XFoam_Face>& meshFaces);
	/// 面心平均 + 各棱锥体积绝对值之和（见 OpenFOAM cell::mag）。
	static XFoam_API XFoam_Scalar mag(
		const XFoam_Cell& c,
		const XFoam_UList<XFoam_Vector3D>& meshPoints,
		const XFoam_UList<XFoam_Face>& meshFaces);

	XFoam_Vector3D centre(
		const XFoam_UList<XFoam_Vector3D>& meshPoints,
		const XFoam_UList<XFoam_Face>& meshFaces) const
	{
		return XFoam_Cell::centre(*this, meshPoints, meshFaces);
	}

	XFoam_Scalar mag(
		const XFoam_UList<XFoam_Vector3D>& meshPoints,
		const XFoam_UList<XFoam_Face>& meshFaces) const
	{
		return XFoam_Cell::mag(*this, meshPoints, meshFaces);
	}

	static XFoam_API XFoam_BoundBox bb(
		const XFoam_Cell& c,
		const XFoam_UList<XFoam_Vector3D>& meshPoints,
		const XFoam_UList<XFoam_Face>& meshFaces);

	XFoam_BoundBox bb(
		const XFoam_UList<XFoam_Vector3D>& meshPoints,
		const XFoam_UList<XFoam_Face>& meshFaces) const
	{
		return XFoam_Cell::bb(*this, meshPoints, meshFaces);
	}
};

/// 与 OpenFOAM 一致：两侧面下标多重集合相同则相等。
XFoam_API bool operator==(const XFoam_Cell& a, const XFoam_Cell& b);

inline bool operator!=(const XFoam_Cell& a, const XFoam_Cell& b)
{
	return !(a == b);
}

// 体元模型：局部顶点编号下的面/边表 + 由棱锥分解得到的 centre / mag（体积）。
// operator== 与 OpenFOAM 一致：仅当同一对象地址时相等（见 cellModelI.H）。
class XFoam_API XFoam_CellModel
{
	XFoam_String name_;
	XFoam_Label index_;
	XFoam_Label nPoints_;
	XFoam_List<XFoam_LabelList> faces_;
	XFoam_List<XFoam_FixedList<XFoam_Label, 2>> edges_;

public:
	XFoam_CellModel(
		XFoam_String name,
		XFoam_Label index,
		XFoam_Label nPoints,
		XFoam_List<XFoam_LabelList> faces,
		XFoam_List<XFoam_FixedList<XFoam_Label, 2>> edges);

	XFoam_CellModel(const XFoam_CellModel&) = default;
	XFoam_CellModel& operator=(const XFoam_CellModel&) = default;
	XFoam_CellModel(XFoam_CellModel&&) noexcept = default;
	XFoam_CellModel& operator=(XFoam_CellModel&&) noexcept = default;

	const XFoam_String& name() const { return name_; }
	XFoam_Label index() const { return index_; }
	XFoam_Label nPoints() const { return nPoints_; }
	XFoam_Label nEdges() const { return edges_.size(); }
	XFoam_Label nFaces() const { return faces_.size(); }

	const XFoam_List<XFoam_LabelList>& modelFaces() const { return faces_; }

	XFoam_List<XFoam_FixedList<XFoam_Label, 2>> edges(
		const XFoam_UList<XFoam_Label>& pointLabels) const;

	XFoam_List<XFoam_LabelList> faces(
		const XFoam_UList<XFoam_Label>& pointLabels) const;

	XFoam_Vector3D centre(
		const XFoam_UList<XFoam_Label>& pointLabels,
		const XFoam_UList<XFoam_Vector3D>& points) const;

	XFoam_Scalar mag(
		const XFoam_UList<XFoam_Label>& pointLabels,
		const XFoam_UList<XFoam_Vector3D>& points) const;

	XFoam_AutoPtr<XFoam_CellModel> clone() const;

	static const XFoam_CellModel& hex();
};

inline bool operator==(const XFoam_CellModel& a, const XFoam_CellModel& b)
{
	return &a == &b;
}

inline bool operator!=(const XFoam_CellModel& a, const XFoam_CellModel& b)
{
	return &a != &b;
}

/// 移植源码：OpenFOAM src/OpenFOAM/meshes/meshShapes/cellShape/cellShape.H（cellShape 继承 labelList）
/// 命名规范：foam_code.md
/// 移植规范：foam_code.md
/// 顶点全局编号为基类 XFoam_LabelList；cellModel 由指针引用（对标 OpenFOAM 的 cellModel*）。
class XFoam_API XFoam_CellShape : public XFoam_LabelList
{
	const XFoam_CellModel* m_;

public:
	XFoam_CellShape();

	XFoam_CellShape(
		const XFoam_CellModel& model,
		const XFoam_LabelList& labels,
		bool doCollapse = false);

	XFoam_CellShape(
		const XFoam_String& modelName,
		const XFoam_LabelList& labels,
		bool doCollapse = false);

	XFoam_CellShape(const XFoam_String& modelName, const XFoam_FixedList<XFoam_Label, 8>& labels);

	XFoam_CellShape(const XFoam_CellShape&) = default;
	XFoam_CellShape& operator=(const XFoam_CellShape&) = default;
	XFoam_CellShape(XFoam_CellShape&&) noexcept = default;
	XFoam_CellShape& operator=(XFoam_CellShape&&) noexcept = default;

	const XFoam_LabelList& labels() const { return *this; }
	XFoam_LabelList& labels() { return *this; }

	using XFoam_LabelList::size;
	using XFoam_LabelList::operator[];

	XFoam_List<XFoam_Vector3D> points(const XFoam_UList<XFoam_Vector3D>& meshPoints) const;

	const XFoam_CellModel& model() const;

	XFoam_LabelList meshFaces(
		const XFoam_List<XFoam_LabelList>& allFaces,
		const XFoam_LabelList& cellFaceIndices) const;

	XFoam_LabelList meshEdges(
		const XFoam_List<XFoam_FixedList<XFoam_Label, 2>>& allEdges,
		const XFoam_LabelList& cellEdgeIndices) const;

	XFoam_List<XFoam_LabelList> faces() const;
	XFoam_List<XFoam_LabelList> collapsedFaces() const;

	XFoam_Label nFaces() const;
	XFoam_List<XFoam_FixedList<XFoam_Label, 2>> edges() const;
	XFoam_Label nEdges() const;
	XFoam_Label nPoints() const;

	// 与 OpenFOAM cellModel::centre/mag(pointLabels, points) 的 pointLabels 对应。
	const XFoam_UList<XFoam_Label>& pointLabelUList() const { return *this; }

	XFoam_Vector3D centre(const XFoam_UList<XFoam_Vector3D>& meshPoints) const;
	XFoam_Scalar mag(const XFoam_UList<XFoam_Vector3D>& meshPoints) const;

	void collapse();

	XFoam_AutoPtr<XFoam_CellShape> clone() const;

	// blockMesh 辅助：第 facei 个模型面上的全局顶点编号（六面体为四边形）。
	XFoam_FixedList<XFoam_Label, 4> faceVertexLabels(XFoam_Label facei) const;
};

XFoam_API bool operator==(const XFoam_CellShape& a, const XFoam_CellShape& b);

inline bool operator!=(const XFoam_CellShape& a, const XFoam_CellShape& b)
{
	return !(a == b);
}

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_CellModel& m);
XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_CellShape& s);

typedef XFoam_List<XFoam_Edge> XFoam_EdgeList;
typedef XFoam_List<XFoam_Face> XFoam_FaceList;
typedef XFoam_UList<XFoam_Face> XFoam_FaceUList;
typedef XFoam_List<XFoam_Cell> XFoam_CellList;
typedef XFoam_List<XFoam_CellShape> XFoam_CellShapeList;
typedef XFoam_List<XFoam_List<XFoam_Face>> XFoam_FaceListList;

// 头文件包含位置：见 doc/foam_code.md「配套 *_templates.h」例外（须在 XFoam_Face 等声明完成之后）。
#include "XFoam/snap/xfoam_shape_templates.h"

#endif
