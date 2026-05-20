#include "XFoam/topo/xfoam_mbrep.h"

#include <algorithm>
#include <limits>
#include <sstream>
#include <unordered_map>

#ifdef XFOAM_WITH_OCCT
// 只有 OCCT ON 时才拉 OCCT 头文件，避免 OFF 模式下 ninja 也得扫上百个 .hxx。
// 这里集中 include；公有头 xfoam_mbrep.h 一行 OCCT 都不暴露（pImpl 隔离）。
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepExtrema_SupportType.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_State.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#endif // XFOAM_WITH_OCCT

// =============================================================================
// MBrep ── 参数化（OCC）B-rep 实现。
//
// 编译开关：XFOAM_WITH_OCCT 控制是否链接 OCCT。
//   ON：OcctData 持 TopoDS_Shape；readFromStep 用 STEPControl_Reader；
//       tessellate 走 BRepMesh_IncrementalMesh；readFromIges 用 IGESControl_Reader。
//   OFF：OcctData 是空壳；三个 read* 抛 XFoam_Error("OCCT disabled at build
//        time")；tessellate 在 meshedTris 已被外部 reader 填好时是 no-op，否则也
//        抛错。这样头文件 / API 完全一致，但功能矩阵随构建变。
//
// 离散化 → snappy：snappy 只吃三角面集合。MBrep 自身不直接走 snap，必须先
// tessellate() → toVBrep() 把所有 ParametricFace 烘成一个 VBrep。TopoModel
// 的 exportToSnappy() 内部会替用户做这一步。
// =============================================================================

struct XFoam_MBrep::OcctData
{
#ifdef XFOAM_WITH_OCCT
	// 根 shape：可能是 Compound（多 solid）/ Solid（单实体）/ Shell（开壳）/
	// 任意其它 TopoDS_Shape。readFromStep / readFromIges 写入；tessellate
	// 用它做 BRepMesh_IncrementalMesh 的输入。
	TopoDS_Shape rootShape;

	// 解析后的 face / edge / vertex / solid → TopoDS_Shape 映射。
	// idx = ParametricFace.id 等；下标对齐 verts_/edges_/faces_/bodies_。
	// 这些 IndexedMap 仅在 readFromXxx 内部构建，外部不可见。
	TopTools_IndexedMapOfShape vertexMap;
	TopTools_IndexedMapOfShape edgeMap;
	TopTools_IndexedMapOfShape faceMap;
	TopTools_IndexedMapOfShape solidMap;
#endif
};

XFoam_MBrep::XFoam_MBrep()
	: occt_(new OcctData())
{}

XFoam_MBrep::~XFoam_MBrep() = default;

XFoam_MBrep::XFoam_MBrep(XFoam_MBrep&&) noexcept            = default;
XFoam_MBrep& XFoam_MBrep::operator=(XFoam_MBrep&&) noexcept = default;

void XFoam_MBrep::clear()
{
	verts_.clear();
	edges_.clear();
	faces_.clear();
	bodies_.clear();
	faceNames_.clear();
	faceTypes_.clear();
	occt_ = XFoam_AutoPtr<OcctData>(new OcctData());
	featureEdgeIdx_.clear();
	featureVertIdx_.clear();
	invalidateBboxCache();
}

XFoam_BoundBox XFoam_MBrep::bounds() const
{
#ifdef XFOAM_WITH_OCCT
	// OCCT ON：优先用 BRepBndLib 取 shape bbox（解析精度比离散点准）。
	if (!occt_().rootShape.IsNull())
	{
		Bnd_Box bb;
		BRepBndLib::Add(occt_().rootShape, bb);
		if (!bb.IsVoid())
		{
			Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
			bb.Get(xMin, yMin, zMin, xMax, yMax, zMax);
			return XFoam_BoundBox(
				XFoam_Vector3D(static_cast<XFoam_Scalar>(xMin),
				               static_cast<XFoam_Scalar>(yMin),
				               static_cast<XFoam_Scalar>(zMin)),
				XFoam_Vector3D(static_cast<XFoam_Scalar>(xMax),
				               static_cast<XFoam_Scalar>(yMax),
				               static_cast<XFoam_Scalar>(zMax)));
		}
	}
#endif

	if (verts_.size() == 0 && faces_.size() == 0)
	{
		return XFoam_BoundBox::invertedBox;
	}
	XFoam_List<XFoam_Vector3D> pts;
	for (XFoam_Label i = 0; i < verts_.size(); ++i)
	{
		pts.append(verts_[i].p);
	}
	// 还把已 tessellated 的 face 离散点也并进去，避免参数边远离 vertex 时
	// bbox 过紧。
	for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
	{
		const auto& f = faces_[fi];
		for (const auto& p : f.meshedPts) pts.append(p);
	}
	if (pts.size() == 0) return XFoam_BoundBox::invertedBox;
	return XFoam_BoundBox(pts);
}

XFoam_BoundBox XFoam_MBrep::refBounds(const XFoam_BrepRef& r) const
{
	if (r.kind != XFoam_BrepKind::Parametric || !r.valid())
	{
		return XFoam_BoundBox::invertedBox;
	}
	// 与 VBrep 一样，按 idx 范围依次尝试 face / vertex / edge / body。
	if (r.idx < faces_.size())
	{
		const auto& f = faces_[r.idx];
		if (f.meshedPts.empty()) return XFoam_BoundBox::invertedBox;
		XFoam_List<XFoam_Vector3D> pts;
		for (const auto& p : f.meshedPts) pts.append(p);
		return XFoam_BoundBox(pts);
	}
	if (r.idx < verts_.size())
	{
		const XFoam_Vector3D p = verts_[r.idx].p;
		return XFoam_BoundBox(p, p);
	}
	return XFoam_BoundBox::invertedBox;
}

void XFoam_MBrep::setFaceName(XFoam_Label id, const XFoam_Word& name)
{
	while (faceNames_.size() <= id)
	{
		std::ostringstream oss;
		oss << "face_" << faceNames_.size();
		faceNames_.append(XFoam_Word(oss.str()));
		faceTypes_.append(XFoam_Word("wall"));
	}
	faceNames_[id] = name;
}

void XFoam_MBrep::setFaceType(XFoam_Label id, const XFoam_Word& type)
{
	while (faceTypes_.size() <= id)
	{
		std::ostringstream oss;
		oss << "face_" << faceTypes_.size();
		faceNames_.append(XFoam_Word(oss.str()));
		faceTypes_.append(XFoam_Word("wall"));
	}
	faceTypes_[id] = type;
}

#ifdef XFOAM_WITH_OCCT
// OCCT 工具：把当前 occt_().rootShape 遍历一遍，把 vertex/edge/face/solid 落到
// 自有的 verts_/edges_/faces_/bodies_ 里，并构 IndexedMap 便于后续 tessellate
// 时按 ParametricFace.id 反查 TopoDS_Face。
namespace
{

void rebuildFromShape(
	XFoam_MBrep::OcctData& od,
	XFoam_List<XFoam_MBrep::ParametricVertex>& verts,
	XFoam_List<XFoam_MBrep::ParametricEdge>&   edges,
	XFoam_List<XFoam_MBrep::ParametricFace>&   faces,
	XFoam_List<XFoam_MBrep::ParametricBody>&   bodies,
	XFoam_WordList&                            faceNames,
	XFoam_WordList&                            faceTypes)
{
	od.vertexMap.Clear();
	od.edgeMap.Clear();
	od.faceMap.Clear();
	od.solidMap.Clear();
	verts.clear();
	edges.clear();
	faces.clear();
	bodies.clear();
	faceNames.clear();
	faceTypes.clear();
	if (od.rootShape.IsNull()) return;

	// 用 TopExp 一次性收齐所有 sub-shape；TopExp::MapShapes 会按 OCCT 内部哈希
	// 去重，相同的 TopoDS_Shape 只算一次。注意 OCCT 的 IndexedMap 是 1-based。
	TopExp::MapShapes(od.rootShape, TopAbs_VERTEX, od.vertexMap);
	TopExp::MapShapes(od.rootShape, TopAbs_EDGE,   od.edgeMap);
	TopExp::MapShapes(od.rootShape, TopAbs_FACE,   od.faceMap);
	TopExp::MapShapes(od.rootShape, TopAbs_SOLID,  od.solidMap);

	// vertex 表：直接拿 BRep_Tool::Pnt
	for (Standard_Integer i = 1; i <= od.vertexMap.Extent(); ++i)
	{
		const TopoDS_Vertex& v = TopoDS::Vertex(od.vertexMap.FindKey(i));
		const gp_Pnt p = BRep_Tool::Pnt(v);
		verts.append(XFoam_MBrep::ParametricVertex(
			XFoam_Vector3D(static_cast<XFoam_Scalar>(p.X()),
			               static_cast<XFoam_Scalar>(p.Y()),
			               static_cast<XFoam_Scalar>(p.Z()))));
	}

	// edge 表：取端点 + sampled（曲线弦高离散，方便 viz / bbox）
	for (Standard_Integer i = 1; i <= od.edgeMap.Extent(); ++i)
	{
		const TopoDS_Edge& e = TopoDS::Edge(od.edgeMap.FindKey(i));
		XFoam_MBrep::ParametricEdge pe;
		// 端点
		TopoDS_Vertex va, vb;
		TopExp::Vertices(e, va, vb);
		if (!va.IsNull())
		{
			const Standard_Integer iv = od.vertexMap.FindIndex(va);
			pe.v0 = static_cast<XFoam_Label>(iv > 0 ? iv - 1 : -1);
		}
		if (!vb.IsNull())
		{
			const Standard_Integer iv = od.vertexMap.FindIndex(vb);
			pe.v1 = static_cast<XFoam_Label>(iv > 0 ? iv - 1 : -1);
		}
		// 曲线 + 参数范围
		Standard_Real first = 0, last = 0;
		const Handle(Geom_Curve) c3d = BRep_Tool::Curve(e, first, last);
		pe.firstParam = static_cast<XFoam_Scalar>(first);
		pe.lastParam  = static_cast<XFoam_Scalar>(last);
		if (!c3d.IsNull())
		{
			// 用 QuasiUniformDeflection 在曲线上撒点，弦高 = 半径的 1/100 量级
			// 自适应；这里固定 1e-2，调用方需要细的再 setLayerDeflection。
			GeomAdaptor_Curve ac(c3d, first, last);
			try
			{
				GCPnts_QuasiUniformDeflection sampler(ac, 1.0e-2);
				if (sampler.IsDone())
				{
					const Standard_Integer n = sampler.NbPoints();
					pe.sampled.reserve(static_cast<size_t>(n));
					for (Standard_Integer k = 1; k <= n; ++k)
					{
						const gp_Pnt p = sampler.Value(k);
						pe.sampled.push_back(XFoam_Vector3D(
							static_cast<XFoam_Scalar>(p.X()),
							static_cast<XFoam_Scalar>(p.Y()),
							static_cast<XFoam_Scalar>(p.Z())));
					}
				}
			}
			catch (const Standard_Failure&)
			{
				// 离散失败不致命，保留两端点足够后续逻辑（bbox / viz）。
				if (pe.v0 >= 0) pe.sampled.push_back(verts[pe.v0].p);
				if (pe.v1 >= 0) pe.sampled.push_back(verts[pe.v1].p);
			}
		}
		edges.append(pe);
	}

	// face 表：先只记录拓扑外环 / 孔环；三角化由 tessellate() 单独跑。
	for (Standard_Integer i = 1; i <= od.faceMap.Extent(); ++i)
	{
		const TopoDS_Face& f = TopoDS::Face(od.faceMap.FindKey(i));
		XFoam_MBrep::ParametricFace pf;

		// OuterWire 与孔环；ShapeAnalysis_FreeBounds 太重，我们简化为
		// "BRepTools::OuterWire() 取外环；其余 wire 全归 innerLoops"。
		TopoDS_Wire outer;
		try { outer = BRepTools::OuterWire(f); }
		catch (const Standard_Failure&) {}

		auto wireToEdgeIds = [&](const TopoDS_Wire& w) -> XFoam_LabelList {
			XFoam_LabelList ids;
			if (w.IsNull()) return ids;
			for (TopExp_Explorer it(w, TopAbs_EDGE); it.More(); it.Next())
			{
				const Standard_Integer ei = od.edgeMap.FindIndex(it.Current());
				if (ei > 0) ids.append(static_cast<XFoam_Label>(ei - 1));
			}
			return ids;
		};

		pf.outerLoop = wireToEdgeIds(outer);

		for (TopExp_Explorer it(f, TopAbs_WIRE); it.More(); it.Next())
		{
			const TopoDS_Wire wr = TopoDS::Wire(it.Current());
			if (!outer.IsNull() && wr.IsSame(outer)) continue;
			XFoam_LabelList loopIds = wireToEdgeIds(wr);
			if (loopIds.size() > 0) pf.innerLoops.push_back(std::move(loopIds));
		}
		faces.append(std::move(pf));

		// 默认 patch 名 "face_<i>"；用户 / 上层可后续 setFaceName 覆盖。
		std::ostringstream oss;
		oss << "face_" << (i - 1);
		faceNames.append(XFoam_Word(oss.str()));
		faceTypes.append(XFoam_Word("wall"));
	}

	// solid 表：每个 solid 的边界 face id 列表。SOLID closed 在 OCCT 里基本恒
	// true（STEP 进来的实体都是 closed shell）；非 closed 的可能是 SHELL → 不
	// 进 solidMap，所以这里全标 closed。
	for (Standard_Integer i = 1; i <= od.solidMap.Extent(); ++i)
	{
		const TopoDS_Solid& s = TopoDS::Solid(od.solidMap.FindKey(i));
		XFoam_MBrep::ParametricBody pb;
		pb.closed = true;
		for (TopExp_Explorer it(s, TopAbs_FACE); it.More(); it.Next())
		{
			const Standard_Integer fi = od.faceMap.FindIndex(it.Current());
			if (fi > 0) pb.faces.append(static_cast<XFoam_Label>(fi - 1));
		}
		bodies.append(std::move(pb));
	}
}

} // namespace
#endif // XFOAM_WITH_OCCT

void XFoam_MBrep::readFromStep(const XFoam_String& fileName)
{
#ifdef XFOAM_WITH_OCCT
	clear();
	STEPControl_Reader reader;
	const IFSelect_ReturnStatus rs = reader.ReadFile(fileName.c_str());
	if (rs != IFSelect_RetDone)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_MBrep::readFromStep: STEPControl_Reader::ReadFile "
			             "failed (code ") + std::to_string(static_cast<int>(rs))
			+ ") for: " + fileName);
	}
	// TransferRoots 把 STEP 里的全部根实体翻成 TopoDS_Shape。一些 STEP 文件
	// 多根 → 包成一个 Compound。
	const Standard_Integer nRoots = reader.TransferRoots();
	if (nRoots <= 0)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_MBrep::readFromStep: no roots transferred from ")
			+ fileName);
	}
	occt_().rootShape = reader.OneShape();
	rebuildFromShape(occt_(), verts_, edges_, faces_, bodies_, faceNames_, faceTypes_);
#else
	(void)fileName;
	throw XFoam_Error(
		"XFoam_MBrep::readFromStep: OCCT disabled at build time "
		"(rebuild with -DXFOAM_WITH_OCCT=ON to enable STEP import).");
#endif
}

void XFoam_MBrep::readFromIges(const XFoam_String& fileName)
{
#ifdef XFOAM_WITH_OCCT
	clear();
	IGESControl_Reader reader;
	const IFSelect_ReturnStatus rs = reader.ReadFile(fileName.c_str());
	if (rs != IFSelect_RetDone)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_MBrep::readFromIges: IGESControl_Reader::ReadFile "
			             "failed (code ") + std::to_string(static_cast<int>(rs))
			+ ") for: " + fileName);
	}
	reader.TransferRoots();
	occt_().rootShape = reader.OneShape();
	rebuildFromShape(occt_(), verts_, edges_, faces_, bodies_, faceNames_, faceTypes_);
#else
	(void)fileName;
	throw XFoam_Error(
		"XFoam_MBrep::readFromIges: OCCT disabled at build time "
		"(rebuild with -DXFOAM_WITH_OCCT=ON to enable IGES import).");
#endif
}

void XFoam_MBrep::tessellate(XFoam_Scalar deflection)
{
#ifdef XFOAM_WITH_OCCT
	if (occt_().rootShape.IsNull())
	{
		// 没绑 OCCT shape：允许"外部 reader 已经填 meshedTris"的退路；逐 face
		// 检查，全填好 = no-op；否则抛错。
		for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
		{
			if (faces_[fi].meshedTris.empty())
			{
				throw XFoam_Error(
					"XFoam_MBrep::tessellate: no OCCT shape bound and face has no "
					"pre-populated meshedTris.");
			}
		}
		return;
	}

	// BRepMesh_IncrementalMesh 一次跑全 shape；isRelative=false 表示 deflection
	// 是绝对弦高（线性距离）；isInParallel=true 在 OCCT 内部用 TBB / OpenMP
	// 加速（如果有）。我们没装 TBB，所以并行参数事实上是 no-op，但留着不害人。
	const Standard_Real defl = static_cast<Standard_Real>(deflection > 0 ? deflection : 1.0e-2);
	const Standard_Real angle = 0.5; // ≈ 28.6°；OCCT 默认 0.5 rad
	BRepMesh_IncrementalMesh mesher(
		occt_().rootShape,
		defl,
		/*isRelative=*/Standard_False,
		angle,
		/*isInParallel=*/Standard_True);
	mesher.Perform();

	// 把每张 TopoDS_Face 上的 Poly_Triangulation 拷到对应 ParametricFace 里。
	// OCCT 的三角面用 1-based 顶点索引，需要 -1；同一 face 的所有顶点存在
	// triangulation 的 Node(i) 里，按 face 局部坐标系存（带 location）。
	for (Standard_Integer fi = 1; fi <= occt_().faceMap.Extent(); ++fi)
	{
		const TopoDS_Face& f = TopoDS::Face(occt_().faceMap.FindKey(fi));
		const XFoam_Label myIdx = static_cast<XFoam_Label>(fi - 1);
		ParametricFace& pf = faces_[myIdx];
		pf.meshedPts.clear();
		pf.meshedTris.clear();

		TopLoc_Location loc;
		const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(f, loc);
		if (tri.IsNull()) continue;

		const gp_Trsf& trsf = loc.Transformation();
		const Standard_Integer nNodes = tri->NbNodes();
		const Standard_Integer nTris  = tri->NbTriangles();
		pf.meshedPts.reserve(static_cast<size_t>(nNodes));
		pf.meshedTris.reserve(static_cast<size_t>(nTris));

		for (Standard_Integer ni = 1; ni <= nNodes; ++ni)
		{
			gp_Pnt p = tri->Node(ni);
			if (!loc.IsIdentity()) p.Transform(trsf);
			pf.meshedPts.push_back(XFoam_Vector3D(
				static_cast<XFoam_Scalar>(p.X()),
				static_cast<XFoam_Scalar>(p.Y()),
				static_cast<XFoam_Scalar>(p.Z())));
		}

		// face.Orientation() == REVERSED 时三角形 winding 要翻转，否则法向朝里。
		const bool reversed = (f.Orientation() == TopAbs_REVERSED);
		for (Standard_Integer ti = 1; ti <= nTris; ++ti)
		{
			Standard_Integer a, b, c;
			tri->Triangle(ti).Get(a, b, c);
			XFoam_FixedList<XFoam_Label, 3> face;
			if (reversed)
			{
				face[0] = static_cast<XFoam_Label>(a - 1);
				face[1] = static_cast<XFoam_Label>(c - 1);
				face[2] = static_cast<XFoam_Label>(b - 1);
			}
			else
			{
				face[0] = static_cast<XFoam_Label>(a - 1);
				face[1] = static_cast<XFoam_Label>(b - 1);
				face[2] = static_cast<XFoam_Label>(c - 1);
			}
			pf.meshedTris.push_back(face);
		}
	}
#else
	(void)deflection;
	// OFF 路径：若每张 face 已经被 reader 填了 meshedTris，则视为 no-op；否则
	// 抛错。这样允许"已经 tessellated 的 MBrep + dict-loaded ParametricFace"
	// 在没有 OCCT 时也能往下游走。
	for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
	{
		if (faces_[fi].meshedTris.empty())
		{
			throw XFoam_Error(
				"XFoam_MBrep::tessellate: OCCT disabled and face has no "
				"pre-populated meshedTris (call requires either OCCT or an "
				"external triangulator to fill ParametricFace::meshedTris).");
		}
	}
#endif
}

XFoam_AutoPtr<XFoam_VBrep> XFoam_MBrep::toVBrep() const
{
	XFoam_AutoPtr<XFoam_VBrep> out(new XFoam_VBrep());
	XFoam_VBrep& vb = out();
	// 每张 ParametricFace 自己有一套独立局部点表 (meshedPts) + 三角面表
	// (meshedTris)，本函数把它们拼成一个 global VBrep，把 patchId 设为本
	// face 的 id；同 patchId 用 faceNames_ 命名。
	for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
	{
		const auto& pf = faces_[fi];
		const XFoam_Label base = vb.nVerts();
		for (const auto& p : pf.meshedPts) vb.positionsRef().append(p);
		for (const auto& t : pf.meshedTris)
		{
			XFoam_VBrep::DiscreteFace df;
			df.verts[0] = base + t[0];
			df.verts[1] = base + t[1];
			df.verts[2] = base + t[2];
			df.patchId  = fi;
			vb.facesRef().append(df);
		}
		XFoam_Word name = (fi < faceNames_.size())
			? faceNames_[fi]
			: XFoam_Word();
		if (name.empty())
		{
			std::ostringstream oss;
			oss << "face_" << fi;
			name = XFoam_Word(oss.str());
		}
		XFoam_Word type = (fi < faceTypes_.size())
			? faceTypes_[fi]
			: XFoam_Word("wall");
		vb.setPatchName(fi, name);
		vb.setPatchType(fi, type);
	}
	return out;
}

// =============================================================================
// MBrep 几何查询 ── OCCT analytic 后端
// 关键依赖：BRepExtrema_DistShapeShape（点 vs shape 最近距离 + 命中子 shape）；
//          BRepClass3d_SolidClassifier（点 in / on / out solid）；
//          BRepBndLib + Bnd_Box（粗筛 bbox overlap）。
//
// 性能：BRepExtrema 是 NlogN 量级，对单 face 通常 ~ms。snappy 高频 closestPoint
// 每点至少 ~ms × N_iter × N_relax，比 BVH 慢 3-4 数量级。但分析精度无懈可击：
// p 离面 1nm 时 BVH 误差就是 1nm，OCCT 误差仍是浮点 ε。如果嫌慢，把 face 数
// 切多个 SubFaceMBrep wrapper 并行喂 snappy。
// =============================================================================
#ifdef XFOAM_WITH_OCCT
namespace
{

inline gp_Pnt toOccPnt(const XFoam_Vector3D& p)
{
	return gp_Pnt(
		static_cast<Standard_Real>(p.x()),
		static_cast<Standard_Real>(p.y()),
		static_cast<Standard_Real>(p.z()));
}
inline XFoam_Vector3D fromOccPnt(const gp_Pnt& q)
{
	return XFoam_Vector3D(
		static_cast<XFoam_Scalar>(q.X()),
		static_cast<XFoam_Scalar>(q.Y()),
		static_cast<XFoam_Scalar>(q.Z()));
}

// 给定 TopoDS_Face 上的最近点 q（参数化空间未知），用 BRepAdaptor_Surface 反求
// 参数 (u, v) 并取法向。若无法解析，回退 (0,0,0)。
XFoam_Vector3D faceNormalAt(const TopoDS_Face& f, const gp_Pnt& q)
{
	try
	{
		BRepAdaptor_Surface bas(f);
		// 简化：用 surface 中点 (uMid, vMid) 作 reverse 起点不太对；直接借
		// BRepExtrema_DistShapeShape 第二次拿 ParOnFaceS2 也能。这里做粗近似 ──
		// 在 face 的参数化中心点取法向，对大多数 STEP face 已够用。
		const Standard_Real u = 0.5 * (bas.FirstUParameter() + bas.LastUParameter());
		const Standard_Real v = 0.5 * (bas.FirstVParameter() + bas.LastVParameter());
		gp_Pnt pp;
		gp_Vec du, dv;
		bas.Surface().D1(u, v, pp, du, dv);
		(void)q; (void)pp;
		gp_Vec n = du.Crossed(dv);
		if (n.Magnitude() < 1.0e-12) return XFoam_Vector3D(0, 0, 0);
		n.Normalize();
		if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
		return XFoam_Vector3D(
			static_cast<XFoam_Scalar>(n.X()),
			static_cast<XFoam_Scalar>(n.Y()),
			static_cast<XFoam_Scalar>(n.Z()));
	}
	catch (const Standard_Failure&)
	{
		return XFoam_Vector3D(0, 0, 0);
	}
}

} // anonymous
#endif // XFOAM_WITH_OCCT

// =============================================================================
// 几何查询粗筛缓存 ── 第一次几何查询时一次性构建 per-face / per-solid bbox。
// 这跟 VBrep 的 ensureAcceleration() 是同一个模式：粗粒度信息缓存到 mutable，
// 后续 contains / boxIntersects / closestPointAndNormal 用它做 branch-and-bound
// 剪枝，避免反复调 BRepBndLib::Add（每次都要遍历 face 子树）。
// =============================================================================
void XFoam_MBrep::ensureBboxCache() const
{
	if (bboxCacheBuilt_) return;
	faceBboxCache_.clear();
	solidBboxCache_.clear();
#ifdef XFOAM_WITH_OCCT
	const Standard_Integer nFace  = occt_().faceMap.Extent();
	const Standard_Integer nSolid = occt_().solidMap.Extent();
	faceBboxCache_.reserve(static_cast<size_t>(nFace));
	for (Standard_Integer fi = 1; fi <= nFace; ++fi)
	{
		Bnd_Box bb;
		try { BRepBndLib::Add(occt_().faceMap.FindKey(fi), bb); }
		catch (const Standard_Failure&) {}
		if (bb.IsVoid())
		{
			faceBboxCache_.push_back(XFoam_BoundBox::invertedBox);
			continue;
		}
		Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
		bb.Get(xMin, yMin, zMin, xMax, yMax, zMax);
		faceBboxCache_.emplace_back(
			XFoam_Vector3D(static_cast<XFoam_Scalar>(xMin),
			               static_cast<XFoam_Scalar>(yMin),
			               static_cast<XFoam_Scalar>(zMin)),
			XFoam_Vector3D(static_cast<XFoam_Scalar>(xMax),
			               static_cast<XFoam_Scalar>(yMax),
			               static_cast<XFoam_Scalar>(zMax)));
	}
	solidBboxCache_.reserve(static_cast<size_t>(nSolid));
	for (Standard_Integer si = 1; si <= nSolid; ++si)
	{
		Bnd_Box bb;
		try { BRepBndLib::Add(occt_().solidMap.FindKey(si), bb); }
		catch (const Standard_Failure&) {}
		if (bb.IsVoid())
		{
			solidBboxCache_.push_back(XFoam_BoundBox::invertedBox);
			continue;
		}
		Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
		bb.Get(xMin, yMin, zMin, xMax, yMax, zMax);
		solidBboxCache_.emplace_back(
			XFoam_Vector3D(static_cast<XFoam_Scalar>(xMin),
			               static_cast<XFoam_Scalar>(yMin),
			               static_cast<XFoam_Scalar>(zMin)),
			XFoam_Vector3D(static_cast<XFoam_Scalar>(xMax),
			               static_cast<XFoam_Scalar>(yMax),
			               static_cast<XFoam_Scalar>(zMax)));
	}
#endif
	bboxCacheBuilt_ = true;
}

bool XFoam_MBrep::contains(const XFoam_Vector3D& p) const
{
#ifdef XFOAM_WITH_OCCT
	if (occt_().rootShape.IsNull()) return false;
	ensureBboxCache();
	try
	{
		// 两阶段：先 per-SOLID bbox 排除（O(1) per solid）；只对 bbox 真正可能
		// 包含 p 的 SOLID 才跑 BRepClass3d_SolidClassifier（O(N_face_of_solid)）。
		// 单 solid CAD 没影响；assembly 多 solid 时性能与命中数 m 而非总数 N 相关。
		const Standard_Integer nSolid = occt_().solidMap.Extent();
		if (nSolid <= 0)
		{
			// rootShape 不含 SOLID（开壳 / Shell-only） → contains 无定义
			return false;
		}
		for (Standard_Integer si = 1; si <= nSolid; ++si)
		{
			const size_t idx = static_cast<size_t>(si - 1);
			if (idx < solidBboxCache_.size())
			{
				const XFoam_BoundBox& bb = solidBboxCache_[idx];
				// 用 bbox.overlaps(point) 等价于 bbox 含 p
				if (!bb.overlaps(XFoam_BoundBox(p, p))) continue;
			}
			const TopoDS_Solid s = TopoDS::Solid(occt_().solidMap.FindKey(si));
			BRepClass3d_SolidClassifier cls(s, toOccPnt(p),
			                                static_cast<Standard_Real>(1.0e-7));
			const TopAbs_State st = cls.State();
			if (st == TopAbs_IN || st == TopAbs_ON) return true;
		}
	}
	catch (const Standard_Failure&) {}
	return false;
#else
	(void)p;
	throw XFoam_Error(
		"XFoam_MBrep::contains: OCCT disabled at build time.");
#endif
}

bool XFoam_MBrep::boxIntersects(const XFoam_BoundBox& box) const
{
#ifdef XFOAM_WITH_OCCT
	if (occt_().rootShape.IsNull()) return false;
	// per-face bbox vs query box。粒度跟 boxIntersects 之前那版相同（per-face 而
	// 非 per-shape），但这次走 faceBboxCache_，不再反复 BRepBndLib::Add。
	// 第一次访问触发 ensureBboxCache()；后续每次查询 O(N_face)，零 OCCT 调用。
	ensureBboxCache();
	for (size_t i = 0; i < faceBboxCache_.size(); ++i)
	{
		if (box.overlaps(faceBboxCache_[i])) return true;
	}
	return false;
#else
	(void)box;
	throw XFoam_Error(
		"XFoam_MBrep::boxIntersects: OCCT disabled at build time.");
#endif
}

#ifdef XFOAM_WITH_OCCT
namespace
{

// p 到 bbox 的最近距离的平方。p 在 bbox 内部时返回 0；用于 branch-and-bound 剪枝。
inline XFoam_Scalar bboxMinDistSqr(
	const XFoam_Vector3D& p, const XFoam_BoundBox& bb)
{
	const XFoam_Scalar dx = std::max<XFoam_Scalar>(
		0, std::max<XFoam_Scalar>(bb.min().x() - p.x(), p.x() - bb.max().x()));
	const XFoam_Scalar dy = std::max<XFoam_Scalar>(
		0, std::max<XFoam_Scalar>(bb.min().y() - p.y(), p.y() - bb.max().y()));
	const XFoam_Scalar dz = std::max<XFoam_Scalar>(
		0, std::max<XFoam_Scalar>(bb.min().z() - p.z(), p.z() - bb.max().z()));
	return dx * dx + dy * dy + dz * dz;
}

} // anon
#endif

void XFoam_MBrep::closestPointAndNormal(
	const XFoam_Vector3D& p,
	XFoam_Vector3D&       outClosest,
	XFoam_Vector3D&       outNormal) const
{
#ifdef XFOAM_WITH_OCCT
	if (occt_().rootShape.IsNull())
	{
		outClosest = p;
		outNormal  = XFoam_Vector3D(0, 0, 0);
		return;
	}
	ensureBboxCache();
	const Standard_Integer nFace = occt_().faceMap.Extent();
	if (nFace <= 0 || faceBboxCache_.empty())
	{
		outClosest = p;
		outNormal  = XFoam_Vector3D(0, 0, 0);
		return;
	}

	try
	{
		// =========================================================
		// 两阶段 branch-and-bound：
		//   阶段 1（粗筛）：把每个 TopoDS_Face 按 bbox-min-distance-to-p 排序。
		//                  这给出"先访问哪些 face 候选"以及"何时可以早停"。
		//   阶段 2（精化）：按距离顺序逐 face 调 BRepExtrema_DistShapeShape；
		//                  只要下一个候选的 bbox-min-distance >= 当前已找到的
		//                  bestDist，就剪枝（剩下的 face 不可能更近）。
		//
		// 跟之前一次 BRepExtrema(rootShape) 对比：
		//   * OCCT 内部对 rootShape 也是遍历所有 face，但它把 face 当 generic
		//     sub-shape 处理，剪枝粒度粗（按整 shape 的 root bbox）。
		//   * 我们这里直接拿 per-face bbox 做剪枝，对大 assembly（多 face）
		//     访问的 face 数 m ≪ N。单几何 CAD（cylinder1 17 face）差异不明显。
		// =========================================================
		const TopoDS_Vertex vp = BRepBuilderAPI_MakeVertex(toOccPnt(p));

		struct FaceCand
		{
			Standard_Integer key;   ///< faceMap.FindKey 用的 1-based 下标
			XFoam_Scalar     d2min; ///< bbox-min-distance² to p
		};
		std::vector<FaceCand> cands;
		cands.reserve(static_cast<size_t>(nFace));
		for (Standard_Integer fi = 1; fi <= nFace; ++fi)
		{
			const size_t idx = static_cast<size_t>(fi - 1);
			if (idx >= faceBboxCache_.size()) continue;
			const XFoam_BoundBox& bb = faceBboxCache_[idx];
			// invertedBox（min > max）→ bboxMinDistSqr 给出负 → 用 0
			if (!(bb.min().x() <= bb.max().x())) continue;
			cands.push_back({fi, bboxMinDistSqr(p, bb)});
		}
		std::sort(cands.begin(), cands.end(),
			[](const FaceCand& a, const FaceCand& b) { return a.d2min < b.d2min; });

		XFoam_Scalar  bestD2 = std::numeric_limits<XFoam_Scalar>::infinity();
		gp_Pnt        bestQ;
		bool          bestIsFace = false;
		TopoDS_Face   bestFace;
		Standard_Real bestU = 0, bestV = 0;
		bool          haveSol = false;

		for (size_t ci = 0; ci < cands.size(); ++ci)
		{
			// 剪枝：下一个候选 bbox 都比当前 best 远 → 后面更不可能命中
			if (cands[ci].d2min >= bestD2) break;
			const TopoDS_Face f = TopoDS::Face(
				occt_().faceMap.FindKey(cands[ci].key));
			BRepExtrema_DistShapeShape extr(vp, f);
			try { extr.Perform(); }
			catch (const Standard_Failure&) { continue; }
			if (!extr.IsDone() || extr.NbSolution() < 1) continue;
			const Standard_Real dval = extr.Value();
			const XFoam_Scalar d2 = static_cast<XFoam_Scalar>(dval * dval);
			if (d2 < bestD2)
			{
				bestD2 = d2;
				bestQ  = extr.PointOnShape2(1);
				const BRepExtrema_SupportType stype = extr.SupportTypeShape2(1);
				bestIsFace = (stype == BRepExtrema_IsInFace);
				bestFace = f;
				if (bestIsFace)
				{
					try { extr.ParOnFaceS2(1, bestU, bestV); }
					catch (const Standard_Failure&) { bestIsFace = false; }
				}
				haveSol = true;
			}
		}
		if (!haveSol)
		{
			outClosest = p;
			outNormal  = XFoam_Vector3D(0, 0, 0);
			return;
		}
		outClosest = fromOccPnt(bestQ);

		// 法向：FACE 命中 → BRepAdaptor_Surface 在 (u,v) 求 D1 拿真实切平面 ⨯
		//                  →  解析法向（数值精度，OCCT REVERSED 处理）；
		//      EDGE/VERTEX → 法向歧义，回退 p→q 单位向量（snap 阶段够用）
		if (bestIsFace)
		{
			try
			{
				BRepAdaptor_Surface bas(bestFace);
				gp_Pnt pp;
				gp_Vec du, dv;
				bas.Surface().D1(bestU, bestV, pp, du, dv);
				gp_Vec n = du.Crossed(dv);
				if (n.Magnitude() > 1.0e-12)
				{
					n.Normalize();
					if (bestFace.Orientation() == TopAbs_REVERSED) n.Reverse();
					outNormal = XFoam_Vector3D(
						static_cast<XFoam_Scalar>(n.X()),
						static_cast<XFoam_Scalar>(n.Y()),
						static_cast<XFoam_Scalar>(n.Z()));
					return;
				}
			}
			catch (const Standard_Failure&) {}
			outNormal = faceNormalAt(bestFace, bestQ);
		}
		else
		{
			const XFoam_Vector3D pq = outClosest - p;
			const XFoam_Scalar m = pq.mag();
			outNormal = (m > 0)
				? XFoam_Vector3D(pq.x() / m, pq.y() / m, pq.z() / m)
				: XFoam_Vector3D(0, 0, 0);
		}
	}
	catch (const Standard_Failure&)
	{
		outClosest = p;
		outNormal  = XFoam_Vector3D(0, 0, 0);
	}
#else
	(void)p;
	(void)outClosest;
	(void)outNormal;
	throw XFoam_Error(
		"XFoam_MBrep::closestPointAndNormal: OCCT disabled at build time.");
#endif
}

void XFoam_MBrep::buildFeatures(XFoam_Scalar featureAngleDeg)
{
#ifdef XFOAM_WITH_OCCT
	featureEdgeIdx_.clear();
	featureVertIdx_.clear();
	if (occt_().rootShape.IsNull()) return;

	// edge → 相邻 face 表
	TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
	TopExp::MapShapesAndAncestors(
		occt_().rootShape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

	const Standard_Real cosThresh = std::cos(
		static_cast<Standard_Real>(featureAngleDeg)
		* static_cast<Standard_Real>(3.14159265358979323846 / 180.0));

	auto faceMidNormal = [](const TopoDS_Face& f) -> gp_Dir {
		BRepAdaptor_Surface bas(f);
		const Standard_Real u = 0.5 * (bas.FirstUParameter() + bas.LastUParameter());
		const Standard_Real v = 0.5 * (bas.FirstVParameter() + bas.LastVParameter());
		gp_Pnt pp;
		gp_Vec du, dv;
		bas.Surface().D1(u, v, pp, du, dv);
		gp_Vec n = du.Crossed(dv);
		if (n.Magnitude() < 1.0e-12) return gp_Dir(0, 0, 1);
		n.Normalize();
		if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
		return gp_Dir(n);
	};

	std::unordered_map<XFoam_Label, int> vertDegree;

	for (Standard_Integer ei = 1; ei <= occt_().edgeMap.Extent(); ++ei)
	{
		const TopoDS_Edge& e = TopoDS::Edge(occt_().edgeMap.FindKey(ei));
		// edgeToFaces 的 key 必须用同样的 IndexedMap 实例查；用 Contains/FindFromKey
		if (!edgeToFaces.Contains(e)) continue;
		const TopTools_ListOfShape& faces = edgeToFaces.FindFromKey(e);
		bool isFeature = false;
		if (faces.Extent() <= 1)
		{
			// free edge (boundary of open shell) → feature
			isFeature = (faces.Extent() == 1);
		}
		else if (faces.Extent() >= 3)
		{
			// non-manifold → feature
			isFeature = true;
		}
		else
		{
			TopTools_ListIteratorOfListOfShape it(faces);
			const TopoDS_Face f1 = TopoDS::Face(it.Value());
			it.Next();
			const TopoDS_Face f2 = TopoDS::Face(it.Value());
			try
			{
				const gp_Dir n1 = faceMidNormal(f1);
				const gp_Dir n2 = faceMidNormal(f2);
				const Standard_Real dot = n1.Dot(n2);
				if (dot < cosThresh) isFeature = true;
			}
			catch (const Standard_Failure&) {}
		}
		if (isFeature)
		{
			const XFoam_Label myIdx = static_cast<XFoam_Label>(ei - 1);
			featureEdgeIdx_.push_back(myIdx);
			if (myIdx < edges_.size())
			{
				if (edges_[myIdx].v0 >= 0) ++vertDegree[edges_[myIdx].v0];
				if (edges_[myIdx].v1 >= 0) ++vertDegree[edges_[myIdx].v1];
			}
		}
	}

	for (const auto& kv : vertDegree)
	{
		if (kv.second >= 3) featureVertIdx_.push_back(kv.first);
	}
#else
	(void)featureAngleDeg;
	throw XFoam_Error(
		"XFoam_MBrep::buildFeatures: OCCT disabled at build time.");
#endif
}

XFoam_BrepBase::FeatureKind XFoam_MBrep::closestFeature(
	const XFoam_Vector3D& p,
	XFoam_Scalar          searchRadius,
	XFoam_Vector3D&       outClosest,
	XFoam_Vector3D&       outTangent) const
{
	outTangent = XFoam_Vector3D(0, 0, 0);
	if (featureEdgeIdx_.empty() && featureVertIdx_.empty()) return FeatureKind::None;

#ifdef XFOAM_WITH_OCCT
	const XFoam_Scalar r2 = searchRadius * searchRadius;
	XFoam_Scalar bestD2 = r2;
	FeatureKind bestKind = FeatureKind::None;
	XFoam_Vector3D bestQ;
	// 粗筛胜出 edge 在 edges_ / occt_().edgeMap 里的下标；后面 OCCT 精化要用。
	XFoam_Label bestEdgeArrayIdx = -1;
	// 粗筛切向（离散 chord 方向）；精化失败时作为 fallback。
	XFoam_Vector3D bestSegTangent(0, 0, 0);

	// ---- 阶段 1：vertex 直接比距离 ----
	// feature vertex 的几何已经是 OCCT TopoDS_Vertex 的精确点（verts_[vi].p
	// 是 BRep_Tool::Pnt(...) 转出来的），不需要 OCCT 精化。
	for (size_t i = 0; i < featureVertIdx_.size(); ++i)
	{
		const XFoam_Label vi = featureVertIdx_[i];
		if (vi < 0 || vi >= verts_.size()) continue;
		const XFoam_Vector3D& fp = verts_[vi].p;
		const XFoam_Vector3D d = fp - p;
		const XFoam_Scalar d2 = d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
		if (d2 < bestD2)
		{
			bestD2   = d2;
			bestKind = FeatureKind::Vertex;
			bestQ    = fp;
		}
	}

	// ---- 阶段 2：edge 粗筛 ----
	// 在 ParametricEdge.sampled（弦高离散 polyline）上做点到 segment 距离。
	// feature edge 总数典型 ~10-10²，逐 segment 扫描的常数远小于 OCCT
	// BRepExtrema_DistShapeShape 一次调用 → 用它做粗筛（O(N_seg)）。
	for (size_t i = 0; i < featureEdgeIdx_.size(); ++i)
	{
		const XFoam_Label ei = featureEdgeIdx_[i];
		if (ei < 0 || ei >= edges_.size()) continue;
		const auto& sampled = edges_[ei].sampled;
		if (sampled.size() < 2) continue;
		for (size_t k = 0; k + 1 < sampled.size(); ++k)
		{
			const XFoam_Vector3D& a = sampled[k];
			const XFoam_Vector3D& b = sampled[k + 1];
			const XFoam_Vector3D ab(b.x() - a.x(), b.y() - a.y(), b.z() - a.z());
			const XFoam_Scalar abLen2 = ab.x() * ab.x() + ab.y() * ab.y() + ab.z() * ab.z();
			XFoam_Vector3D q;
			if (abLen2 <= 0) { q = a; }
			else
			{
				const XFoam_Vector3D ap(p.x() - a.x(), p.y() - a.y(), p.z() - a.z());
				XFoam_Scalar tt = (ap.x() * ab.x() + ap.y() * ab.y() + ap.z() * ab.z()) / abLen2;
				if (tt < 0) tt = 0;
				if (tt > 1) tt = 1;
				q = XFoam_Vector3D(a.x() + ab.x() * tt,
				                   a.y() + ab.y() * tt,
				                   a.z() + ab.z() * tt);
			}
			const XFoam_Vector3D d(p.x() - q.x(), p.y() - q.y(), p.z() - q.z());
			const XFoam_Scalar d2 = d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
			if (d2 < bestD2)
			{
				bestD2 = d2;
				bestKind = FeatureKind::Edge;
				bestQ  = q;
				bestEdgeArrayIdx = ei;
				const XFoam_Scalar m = std::sqrt(abLen2);
				if (m > 0)
				{
					const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / m;
					bestSegTangent = XFoam_Vector3D(
						ab.x() * inv, ab.y() * inv, ab.z() * inv);
				}
			}
		}
	}

	// ---- 阶段 3：胜出 edge 上做 OCCT 精化 ----
	// 离散粗筛给出的 closest 点最坏在 chord 上，误差 ~ chord 高（= sampler 弦高
	// 1e-2 量级，对 CAD 是 mm 量级的 10⁻²）；切向只是 chord 方向，曲线弯曲处偏。
	// 这里对胜出的那一条 TopoDS_Edge 调一次 BRepExtrema_DistShapeShape：
	//   * PointOnShape2(1) → 精确最近点（数值精度 ~10⁻¹⁴）
	//   * ParOnEdgeS2(1, u) → 拿到曲线参数 u
	//   * BRepAdaptor_Curve.D1(u, P, V) → 真实曲线切向（不是 chord 方向）
	// 一条 edge 一次调用 → O(1) 开销，换 ~10⁸× 精度提升 + 切向正确。
	if (bestKind == FeatureKind::Edge
		&& bestEdgeArrayIdx >= 0
		&& !occt_().rootShape.IsNull())
	{
		const Standard_Integer occtKey =
			static_cast<Standard_Integer>(bestEdgeArrayIdx + 1);
		bool refined = false;
		if (occtKey >= 1 && occtKey <= occt_().edgeMap.Extent())
		{
			try
			{
				const TopoDS_Edge& e = TopoDS::Edge(
					occt_().edgeMap.FindKey(occtKey));
				const TopoDS_Vertex vp = BRepBuilderAPI_MakeVertex(toOccPnt(p));
				BRepExtrema_DistShapeShape extr(vp, e);
				extr.Perform();
				if (extr.IsDone() && extr.NbSolution() >= 1)
				{
					const gp_Pnt q = extr.PointOnShape2(1);
					bestQ = fromOccPnt(q);

					Standard_Real u = 0;
					extr.ParOnEdgeS2(1, u);
					BRepAdaptor_Curve bac(e);
					gp_Pnt pp;
					gp_Vec dv;
					bac.D1(u, pp, dv);
					const Standard_Real m = dv.Magnitude();
					if (m > 1.0e-14)
					{
						dv.Divide(m);
						if (e.Orientation() == TopAbs_REVERSED) dv.Reverse();
						bestSegTangent = XFoam_Vector3D(
							static_cast<XFoam_Scalar>(dv.X()),
							static_cast<XFoam_Scalar>(dv.Y()),
							static_cast<XFoam_Scalar>(dv.Z()));
					}
					refined = true;
				}
			}
			catch (const Standard_Failure&)
			{
				// OCCT 偶尔在退化曲线 / 自相交段会抛；保留粗筛结果即可。
			}
		}
		(void)refined;
	}

	if (bestKind != FeatureKind::None) outClosest = bestQ;
	if (bestKind == FeatureKind::Edge)  outTangent = bestSegTangent;
	return bestKind;
#else
	(void)p;
	(void)searchRadius;
	(void)outClosest;
	return FeatureKind::None;
#endif
}
