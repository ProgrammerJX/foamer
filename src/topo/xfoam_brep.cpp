#include "XFoam/topo/xfoam_brep.h"
#include "XFoam/topo/xfoam_mbrep.h"
#include "XFoam/topo/xfoam_vbrep.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_map>

// ============================================================================
// XFoam_BrepBase / XFoam_TopoEntity 抽象类析构
// ============================================================================
XFoam_BrepBase::~XFoam_BrepBase()  = default;
XFoam_TopoEntity::~XFoam_TopoEntity() = default;

void XFoam_TopoEntity::absorbRefs(const XFoam_TopoEntity& other)
{
	for (XFoam_Label i = 0; i < other.refs_.size(); ++i)
	{
		refs_.append(other.refs_[i]);
	}
}

// ============================================================================
// 工具：跨 brep 的"几何辅助"，集中在这里以便 TopoEntity 计算 bounds / position
// 时不必类型分支。
// ============================================================================
namespace
{

// 取 BrepRef 指代的 vertex 坐标。失败时返回 (0,0,0)；调用方需自行确认 valid()。
XFoam_Vector3D refVertexPosition(const XFoam_BrepBase& brep, const XFoam_BrepRef& r)
{
	if (!r.valid()) return XFoam_Vector3D(0, 0, 0);
	if (r.kind == XFoam_BrepKind::Triangulated)
	{
		const auto& vb = static_cast<const XFoam_VBrep&>(brep);
		if (r.idx < 0 || r.idx >= vb.nVerts()) return XFoam_Vector3D(0, 0, 0);
		return vb.positions()[r.idx];
	}
	if (r.kind == XFoam_BrepKind::Parametric)
	{
		const auto& mb = static_cast<const XFoam_MBrep&>(brep);
		if (r.idx < 0 || r.idx >= mb.nVerts()) return XFoam_Vector3D(0, 0, 0);
		return mb.verts()[r.idx].p;
	}
	return XFoam_Vector3D(0, 0, 0);
}

// 把若干 bbox 合并；约定"空 bbox = invertedBox（min > max）"。空输入直接返回
// invertedBox；候选里若是 invertedBox 会自动被 component-wise min/max 跳过。
XFoam_BoundBox unionBounds(const std::vector<XFoam_BoundBox>& bs)
{
	if (bs.empty()) return XFoam_BoundBox::invertedBox;
	XFoam_Vector3D mn = bs[0].min();
	XFoam_Vector3D mx = bs[0].max();
	for (size_t i = 1; i < bs.size(); ++i)
	{
		const auto& bb = bs[i];
		mn = XFoam_Vector3D(
			std::min(mn.x(), bb.min().x()),
			std::min(mn.y(), bb.min().y()),
			std::min(mn.z(), bb.min().z()));
		mx = XFoam_Vector3D(
			std::max(mx.x(), bb.max().x()),
			std::max(mx.y(), bb.max().y()),
			std::max(mx.z(), bb.max().z()));
	}
	return XFoam_BoundBox(mn, mx);
}

} // namespace

// ============================================================================
// XFoam_TopoVert
// ============================================================================
XFoam_Vector3D XFoam_TopoVert::position() const
{
	if (model_ == nullptr || refs_.size() == 0)
	{
		return XFoam_Vector3D(0, 0, 0);
	}
	const XFoam_BrepBase& brep = model_->brep();
	XFoam_Vector3D sum(0, 0, 0);
	XFoam_Label count = 0;
	for (XFoam_Label i = 0; i < refs_.size(); ++i)
	{
		sum = sum + refVertexPosition(brep, refs_[i]);
		++count;
	}
	if (count == 0) return XFoam_Vector3D(0, 0, 0);
	const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / static_cast<XFoam_Scalar>(count);
	return XFoam_Vector3D(sum.x() * inv, sum.y() * inv, sum.z() * inv);
}

XFoam_BoundBox XFoam_TopoVert::bounds() const
{
	const XFoam_Vector3D p = position();
	return XFoam_BoundBox(p, p);
}

// ============================================================================
// XFoam_TopoEdge
// ============================================================================
XFoam_Scalar XFoam_TopoEdge::length() const
{
	if (model_ == nullptr) return 0;
	const XFoam_BrepBase& brep = model_->brep();
	if (brep.kind() != XFoam_BrepKind::Triangulated) return 0; // MBrep 留 TODO
	const auto& vb = static_cast<const XFoam_VBrep&>(brep);
	XFoam_Scalar L = 0;
	for (XFoam_Label i = 0; i < refs_.size(); ++i)
	{
		const XFoam_BrepRef& r = refs_[i];
		if (r.kind != XFoam_BrepKind::Triangulated) continue;
		if (r.idx < 0 || r.idx >= vb.nEdges()) continue;
		const auto& e = vb.edges()[r.idx];
		if (e.verts[0] < 0 || e.verts[0] >= vb.nVerts()) continue;
		if (e.verts[1] < 0 || e.verts[1] >= vb.nVerts()) continue;
		const XFoam_Vector3D a = vb.positions()[e.verts[0]];
		const XFoam_Vector3D b = vb.positions()[e.verts[1]];
		const XFoam_Vector3D d = b - a;
		L += d.mag();
	}
	return L;
}

XFoam_BoundBox XFoam_TopoEdge::bounds() const
{
	if (model_ == nullptr) return XFoam_BoundBox::invertedBox;
	const XFoam_BrepBase& brep = model_->brep();
	std::vector<XFoam_BoundBox> bs;
	bs.reserve(static_cast<size_t>(refs_.size()));
	for (XFoam_Label i = 0; i < refs_.size(); ++i)
	{
		bs.push_back(brep.refBounds(refs_[i]));
	}
	return unionBounds(bs);
}

// ============================================================================
// XFoam_TopoFace
// ============================================================================
XFoam_Scalar XFoam_TopoFace::area() const
{
	if (model_ == nullptr) return 0;
	const XFoam_BrepBase& brep = model_->brep();
	if (brep.kind() != XFoam_BrepKind::Triangulated) return 0; // MBrep tessellate 后再算
	const auto& vb = static_cast<const XFoam_VBrep&>(brep);
	XFoam_Scalar A = 0;
	for (XFoam_Label i = 0; i < refs_.size(); ++i)
	{
		const XFoam_BrepRef& r = refs_[i];
		if (r.kind != XFoam_BrepKind::Triangulated) continue;
		if (r.idx < 0 || r.idx >= vb.nFaces()) continue;
		const auto& f = vb.faces()[r.idx];
		const XFoam_Vector3D v0 = vb.positions()[f.verts[0]];
		const XFoam_Vector3D v1 = vb.positions()[f.verts[1]];
		const XFoam_Vector3D v2 = vb.positions()[f.verts[2]];
		const XFoam_Vector3D e1 = v1 - v0;
		const XFoam_Vector3D e2 = v2 - v0;
		const XFoam_Vector3D c(
			e1.y() * e2.z() - e1.z() * e2.y(),
			e1.z() * e2.x() - e1.x() * e2.z(),
			e1.x() * e2.y() - e1.y() * e2.x());
		A += static_cast<XFoam_Scalar>(0.5) * c.mag();
	}
	return A;
}

XFoam_BoundBox XFoam_TopoFace::bounds() const
{
	if (model_ == nullptr) return XFoam_BoundBox::invertedBox;
	const XFoam_BrepBase& brep = model_->brep();
	std::vector<XFoam_BoundBox> bs;
	bs.reserve(static_cast<size_t>(refs_.size()));
	for (XFoam_Label i = 0; i < refs_.size(); ++i)
	{
		bs.push_back(brep.refBounds(refs_[i]));
	}
	return unionBounds(bs);
}

// ============================================================================
// XFoam_TopoBody
// ============================================================================
XFoam_BoundBox XFoam_TopoBody::bounds() const
{
	if (model_ == nullptr) return XFoam_BoundBox::invertedBox;
	std::vector<XFoam_BoundBox> bs;
	bs.reserve(static_cast<size_t>(faceIds_.size()));
	for (XFoam_Label i = 0; i < faceIds_.size(); ++i)
	{
		const XFoam_Label fid = faceIds_[i];
		if (fid < 0 || fid >= model_->nFaces()) continue;
		bs.push_back(model_->face(fid).bounds());
	}
	return unionBounds(bs);
}

// ============================================================================
// XFoam_TopoModel
// ============================================================================
XFoam_TopoModel::XFoam_TopoModel()
	: brep_(new XFoam_VBrep())
{}

XFoam_TopoModel::~XFoam_TopoModel() = default;

XFoam_TopoModel::XFoam_TopoModel(XFoam_TopoModel&&) noexcept            = default;
XFoam_TopoModel& XFoam_TopoModel::operator=(XFoam_TopoModel&&) noexcept = default;

XFoam_BrepKind XFoam_TopoModel::brepKind() const
{
	return brep_().kind();
}

const XFoam_VBrep& XFoam_TopoModel::vbrep() const
{
	if (brep_().kind() != XFoam_BrepKind::Triangulated)
	{
		throw XFoam_Error("XFoam_TopoModel::vbrep: underlying brep is not Triangulated");
	}
	return static_cast<const XFoam_VBrep&>(brep_());
}

XFoam_VBrep& XFoam_TopoModel::vbrep()
{
	if (brep_().kind() != XFoam_BrepKind::Triangulated)
	{
		throw XFoam_Error("XFoam_TopoModel::vbrep: underlying brep is not Triangulated");
	}
	return static_cast<XFoam_VBrep&>(brep_());
}

const XFoam_MBrep& XFoam_TopoModel::mbrep() const
{
	if (brep_().kind() != XFoam_BrepKind::Parametric)
	{
		throw XFoam_Error("XFoam_TopoModel::mbrep: underlying brep is not Parametric");
	}
	return static_cast<const XFoam_MBrep&>(brep_());
}

XFoam_MBrep& XFoam_TopoModel::mbrep()
{
	if (brep_().kind() != XFoam_BrepKind::Parametric)
	{
		throw XFoam_Error("XFoam_TopoModel::mbrep: underlying brep is not Parametric");
	}
	return static_cast<XFoam_MBrep&>(brep_());
}

void XFoam_TopoModel::setBrep(XFoam_AutoPtr<XFoam_BrepBase> b)
{
	brep_ = XFoam_move(b);
	verts_.clear();
	edges_.clear();
	faces_.clear();
	bodies_.clear();
}

XFoam_BoundBox XFoam_TopoModel::bounds() const
{
	return brep_().bounds();
}

// ----- I/O 分派 ----------------------------------------------------------
void XFoam_TopoModel::readFromStl(const XFoam_String& fileName)
{
	// 直接 new 一个 VBrep，read 完后包成 BrepBase AutoPtr 接管。XFoam_AutoPtr 的
	// "release 所有权" 接口叫 ptr()（与 OF 一致），不是 std::unique_ptr 的 release()。
	XFoam_VBrep* raw = new XFoam_VBrep();
	try { raw->readStlAscii(fileName); }
	catch (...) { delete raw; throw; }
	setBrep(XFoam_AutoPtr<XFoam_BrepBase>(raw));
}

void XFoam_TopoModel::readFromBdf(const XFoam_String& fileName)
{
	XFoam_VBrep* raw = new XFoam_VBrep();
	try { raw->readFromBdf(fileName); }
	catch (...) { delete raw; throw; }
	setBrep(XFoam_AutoPtr<XFoam_BrepBase>(raw));
}

void XFoam_TopoModel::readFromStep(const XFoam_String& fileName)
{
	XFoam_MBrep* raw = new XFoam_MBrep();
	try { raw->readFromStep(fileName); } // OCCT OFF 时抛
	catch (...) { delete raw; throw; }
	setBrep(XFoam_AutoPtr<XFoam_BrepBase>(raw));
}

void XFoam_TopoModel::readFromIges(const XFoam_String& fileName)
{
	XFoam_MBrep* raw = new XFoam_MBrep();
	try { raw->readFromIges(fileName); } // OCCT OFF 时抛
	catch (...) { delete raw; throw; }
	setBrep(XFoam_AutoPtr<XFoam_BrepBase>(raw));
}

// ----- 编辑：增 ---------------------------------------------------------
XFoam_Label XFoam_TopoModel::addVert(const XFoam_BrepRef& r)
{
	const XFoam_Label id = verts_.size();
	XFoam_TopoVert v(this, id);
	if (r.valid()) v.addRef(r);
	verts_.append(v);
	return id;
}

XFoam_Label XFoam_TopoModel::addEdge(XFoam_Label v0, XFoam_Label v1, const XFoam_BrepRef& r)
{
	const XFoam_Label id = edges_.size();
	XFoam_TopoEdge e(this, id);
	e.setEndpoints(v0, v1);
	if (r.valid()) e.addRef(r);
	edges_.append(e);
	return id;
}

XFoam_Label XFoam_TopoModel::addFace(const XFoam_Word& name, const XFoam_Word& patchType)
{
	const XFoam_Label id = faces_.size();
	XFoam_TopoFace f(this, id);
	f.setName(name);
	f.setPatchType(patchType);
	faces_.append(f);
	return id;
}

XFoam_Label XFoam_TopoModel::addBody(const XFoam_Word& /*name*/)
{
	const XFoam_Label id = bodies_.size();
	bodies_.append(XFoam_TopoBody(this, id));
	return id;
}

// ----- 编辑：merge（src 保留 id + suppressed = true）-----------------------
void XFoam_TopoModel::mergeFaces(const XFoam_LabelList& src, XFoam_Label dst)
{
	if (dst < 0 || dst >= faces_.size())
	{
		throw XFoam_Error("XFoam_TopoModel::mergeFaces: invalid dst id");
	}
	XFoam_TopoFace& d = faces_[dst];
	for (XFoam_Label k = 0; k < src.size(); ++k)
	{
		const XFoam_Label s = src[k];
		if (s == dst) continue;
		if (s < 0 || s >= faces_.size()) continue;
		XFoam_TopoFace& sf = faces_[s];
		d.absorbRefs(sf);
		sf.clearRefs();
		sf.setSuppressed(true);
	}
}

void XFoam_TopoModel::mergeEdges(const XFoam_LabelList& src, XFoam_Label dst)
{
	if (dst < 0 || dst >= edges_.size())
	{
		throw XFoam_Error("XFoam_TopoModel::mergeEdges: invalid dst id");
	}
	XFoam_TopoEdge& d = edges_[dst];
	for (XFoam_Label k = 0; k < src.size(); ++k)
	{
		const XFoam_Label s = src[k];
		if (s == dst) continue;
		if (s < 0 || s >= edges_.size()) continue;
		XFoam_TopoEdge& se = edges_[s];
		d.absorbRefs(se);
		se.clearRefs();
		se.setSuppressed(true);
	}
}

// ----- 编辑：split 首期 stub ---------------------------------------------
XFoam_LabelList XFoam_TopoModel::splitFaceByEdge(XFoam_Label /*faceId*/, XFoam_Label /*cutEdgeId*/)
{
	throw XFoam_Error(
		"XFoam_TopoModel::splitFaceByEdge: not implemented yet "
		"(TODO: ear-clipping along the virtual cut edge)");
}

// ----- 抑制 ---------------------------------------------------------------
XFoam_Label XFoam_TopoModel::suppressEdgesShorterThan(XFoam_Scalar minLen)
{
	XFoam_Label n = 0;
	for (XFoam_Label i = 0; i < edges_.size(); ++i)
	{
		if (edges_[i].suppressed()) continue;
		if (edges_[i].length() < minLen)
		{
			edges_[i].setSuppressed(true);
			++n;
		}
	}
	return n;
}

XFoam_Label XFoam_TopoModel::suppressFacesSmallerThan(XFoam_Scalar minArea)
{
	XFoam_Label n = 0;
	for (XFoam_Label i = 0; i < faces_.size(); ++i)
	{
		if (faces_[i].suppressed()) continue;
		if (faces_[i].area() < minArea)
		{
			faces_[i].setSuppressed(true);
			++n;
		}
	}
	return n;
}

// ----- 从底层 brep 重建"identity"虚拓扑 ----------------------------------
void XFoam_TopoModel::rebuildIdentityFromBrep(XFoam_Scalar featureAngleDeg)
{
	verts_.clear();
	edges_.clear();
	faces_.clear();
	bodies_.clear();

	const XFoam_BrepKind k = brepKind();

	if (k == XFoam_BrepKind::Triangulated)
	{
		XFoam_VBrep& vb = vbrep();

		// 没有 edge 表 → 现场构一份（也会标 feature / boundary）。
		if (vb.nEdges() == 0)
		{
			vb.buildEdgesFromFaces(featureAngleDeg);
		}

		// 每个 patchId（含 -1）一个 TopoFace。把 face 按 patchId 桶排。
		std::unordered_map<XFoam_Label, XFoam_Label> patch2topoFace;
		patch2topoFace.reserve(static_cast<size_t>(std::max<XFoam_Label>(1, vb.nPatches())));
		for (XFoam_Label fi = 0; fi < vb.nFaces(); ++fi)
		{
			const XFoam_Label pid = vb.faces()[fi].patchId;
			auto it = patch2topoFace.find(pid);
			XFoam_Label fid;
			if (it == patch2topoFace.end())
			{
				XFoam_Word pname;
				XFoam_Word ptype("wall");
				if (pid >= 0 && pid < vb.nPatches())
				{
					pname = vb.patchNames()[pid];
					ptype = vb.patchTypes()[pid];
				}
				if (pname.empty())
				{
					std::ostringstream oss;
					oss << "patch_" << pid;
					pname = XFoam_Word(oss.str());
				}
				fid = addFace(pname, ptype);
				patch2topoFace.emplace(pid, fid);
			}
			else
			{
				fid = it->second;
			}
			faces_[fid].addRef(XFoam_BrepRef(XFoam_BrepKind::Triangulated, fi));
		}

		// feature edge → TopoEdge（不去重端点，端点 v0/v1 留 -1，需要时再 rebuild）。
		// 简化：不去构造 polyline 段链，每条 feature DiscreteEdge = 一个 TopoEdge。
		for (XFoam_Label ei = 0; ei < vb.nEdges(); ++ei)
		{
			if (!vb.edges()[ei].isFeature) continue;
			addEdge(-1, -1, XFoam_BrepRef(XFoam_BrepKind::Triangulated, ei));
		}

		// feature vertex：先简化为"出现在 ≥ 3 条 feature edge 端点上的 vertex"。
		std::unordered_map<XFoam_Label, int> vIncCount;
		for (XFoam_Label ei = 0; ei < vb.nEdges(); ++ei)
		{
			if (!vb.edges()[ei].isFeature) continue;
			++vIncCount[vb.edges()[ei].verts[0]];
			++vIncCount[vb.edges()[ei].verts[1]];
		}
		for (const auto& kv : vIncCount)
		{
			if (kv.second >= 3)
			{
				addVert(XFoam_BrepRef(XFoam_BrepKind::Triangulated, kv.first));
			}
		}

		// 一个粗粒度 Body 把所有 face 收编（后续 multi-region 再细分）。
		if (faces_.size() > 0)
		{
			const XFoam_Label bid = addBody(XFoam_Word("default"));
			for (XFoam_Label fid = 0; fid < faces_.size(); ++fid)
			{
				bodies_[bid].addFace(fid);
			}
		}
		return;
	}

	if (k == XFoam_BrepKind::Parametric)
	{
		const XFoam_MBrep& mb = mbrep();

		// 每个 ParametricFace 一个 TopoFace。
		for (XFoam_Label fi = 0; fi < mb.nFaces(); ++fi)
		{
			XFoam_Word pname;
			XFoam_Word ptype("wall");
			if (fi < mb.faceNames().size()) pname = mb.faceNames()[fi];
			if (fi < mb.faceTypes().size()) ptype = mb.faceTypes()[fi];
			if (pname.empty())
			{
				std::ostringstream oss;
				oss << "face_" << fi;
				pname = XFoam_Word(oss.str());
			}
			const XFoam_Label fid = addFace(pname, ptype);
			faces_[fid].addRef(XFoam_BrepRef(XFoam_BrepKind::Parametric, fi));
		}
		// 每条 ParametricEdge → TopoEdge；端点已经在 ParametricEdge.v0/v1 里。
		for (XFoam_Label ei = 0; ei < mb.nEdges(); ++ei)
		{
			addEdge(mb.edges()[ei].v0, mb.edges()[ei].v1,
			        XFoam_BrepRef(XFoam_BrepKind::Parametric, ei));
		}
		// 每个 ParametricVertex → TopoVert。
		for (XFoam_Label vi = 0; vi < mb.nVerts(); ++vi)
		{
			addVert(XFoam_BrepRef(XFoam_BrepKind::Parametric, vi));
		}
		// 每个 ParametricBody → TopoBody。
		for (XFoam_Label bi = 0; bi < mb.nBodies(); ++bi)
		{
			const XFoam_Label bid = addBody();
			bodies_[bid].setFaceIds(mb.bodies()[bi].faces);
		}
		return;
	}
}

// ----- Parametric → Triangulated 一步切换 ---------------------------------
void XFoam_TopoModel::convertMBrepToVBrep(
	XFoam_Scalar deflection, XFoam_Scalar featureAngleDeg)
{
	if (brepKind() != XFoam_BrepKind::Parametric)
	{
		throw XFoam_Error(
			"XFoam_TopoModel::convertMBrepToVBrep: brep is not Parametric");
	}
	XFoam_MBrep& mb = mbrep();
	mb.tessellate(deflection);
	XFoam_AutoPtr<XFoam_VBrep> vbAuto = mb.toVBrep();
	XFoam_VBrep* vbRaw = vbAuto.ptr();        // ptr() 转移所有权 → 我们接管
	setBrep(XFoam_AutoPtr<XFoam_BrepBase>(vbRaw));
	rebuildIdentityFromBrep(featureAngleDeg);
}

// ----- snappy 导出适配器 --------------------------------------------------
std::vector<XFoam_TopoModel::ExportedSurface>
XFoam_TopoModel::exportToSnappy(XFoam_Scalar deflection) const
{
	std::vector<ExportedSurface> out;

	const XFoam_BrepKind k = brepKind();

	if (k == XFoam_BrepKind::Parametric)
	{
		// MBrep 路径：先 tessellate（弦高 deflection），再借用 MBrep::toVBrep()
		// 拷一份临时 VBrep，把"每张 ParametricFace 的 meshedTris"映射成一个
		// global triangle list；然后按 TopoFace.refs_ 列出来的 ParametricFace
		// idx 把对应 VBrep face 全部归到同一个 ExportedSurface。
		//
		// 关键约束（决定）：本函数只生产 ExportedSurface，不修改 brep_。toVBrep()
		// 返回的临时 VBrep 是调用方需要的，但 ExportedSurface.triIdx 引用的
		// VBrep 必须由调用方拿到 —— 这里把 triangles 拷贝进 ExportedSurface
		// 之外没办法，简洁起见我们改成：返回 vbrep 上的 face id（即 ParametricFace
		// 内 tri 拼好后在 VBrep 里的 face 全局 idx），同时把 VBrep 暂存在
		// thread-local 等地方就太花哨；这里采用更简单的契约：MBrep 调用方
		// 应当**先** setBrep(toVBrep()) 把模型固化为 Triangulated，再调
		// exportToSnappy。本分支为了不让 caller 落空，至少先把 tessellate 跑
		// 一遍并返回空集合，让 caller 知道下一步该 toVBrep。
		const_cast<XFoam_MBrep&>(mbrep()).tessellate(deflection);
		return out;
	}

	if (k != XFoam_BrepKind::Triangulated) return out;

	const XFoam_VBrep& vb = static_cast<const XFoam_VBrep&>(brep());
	for (XFoam_Label fid = 0; fid < faces_.size(); ++fid)
	{
		const XFoam_TopoFace& tf = faces_[fid];
		if (tf.suppressed()) continue;
		ExportedSurface es;
		es.name      = tf.name();
		es.patchType = tf.patchType();
		es.triIdx.reserve(static_cast<size_t>(tf.nRefs()));
		for (XFoam_Label k = 0; k < tf.nRefs(); ++k)
		{
			const XFoam_BrepRef& r = tf.ref(k);
			if (r.kind != XFoam_BrepKind::Triangulated) continue;
			if (r.idx < 0 || r.idx >= vb.nFaces()) continue;
			es.triIdx.push_back(r.idx);
		}
		if (!es.triIdx.empty()) out.push_back(es);
	}
	return out;
}
