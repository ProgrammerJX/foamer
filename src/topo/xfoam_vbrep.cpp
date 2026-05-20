#include "XFoam/topo/xfoam_vbrep.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

// =============================================================================
// VBrep ── 离散（三角化）B-rep 实现。
//
// 本文件汇集所有"已经离散为三角面"的输入路径：STL、Nastran BDF 等。原 MBrep
// 的 BDF 读取（CTRIA3 / CQUAD4 / $HMMOVE）整体迁过来挂在 readFromBdf 上。
//
// edge 表 / dihedral 标定 / stitch / unifySameDomain 当前为最小实现（buildEdges
// 已能算 dihedralCos 与 isFeature；stitch / unify 留 stub 待后续）。
// =============================================================================
namespace
{

void trimString(XFoam_String& s);

// ---------- Nastran BDF parsing helpers ----------
struct HmMoveBlock
{
	int compId = -1;
	std::vector<int> elemIds;
};

void parseHmMoveLine(const XFoam_String& line, std::vector<XFoam_String>& strs)
{
	if (line.empty())
	{
		strs.clear();
		return;
	}
	const auto num = line.length() / 8u;
	strs.resize(num);
	for (XFoam_Size i = 0; i < num; ++i)
	{
		strs[i].clear();
		for (unsigned j = 0; j < 8u; ++j)
		{
			const XFoam_Size k = i * 8u + j;
			if (k < line.size())
			{
				strs[i].push_back(line[k]);
			}
		}
		trimString(strs[i]);
	}
}

// 读取 $HMMOVE 块；语义参考 XData bdf_readcomps.cpp BDF_readHMMove。
void readHmMoveBlock(
	std::ifstream& ifs,
	const XFoam_String& firstLine,
	XFoam_String& pendingOut,
	HmMoveBlock& out)
{
	out.compId = -1;
	out.elemIds.clear();

	std::vector<XFoam_String> strs;
	parseHmMoveLine(firstLine, strs);
	if (strs.size() < 2u)
	{
		return;
	}
	out.compId = std::atoi(strs[1].c_str());

	std::vector<int> ints;
	XFoam_String line;
	while (std::getline(ifs, line))
	{
		if (!(line.size() >= 2u && line[0] == '$'
		      && (line[1] == ' ' || line[1] == '\t')))
		{
			pendingOut = line;
			break;
		}
		parseHmMoveLine(line, strs);
		for (auto& str3 : strs)
		{
			trimString(str3);
			if (str3.empty() || str3[0] == '$')
			{
				continue;
			}
			if (str3.size() >= 4u && str3.compare(0, 4, "THRU") == 0)
			{
				ints.push_back(-1);
				continue;
			}
			ints.push_back(std::atoi(str3.c_str()));
		}
	}

	for (XFoam_Size i = 0; i < ints.size();)
	{
		if (ints.size() > i + 2u && ints[i + 1] < 0)
		{
			for (int j = ints[i]; j <= ints[i + 2]; ++j)
			{
				out.elemIds.push_back(j);
			}
			i += 3;
			continue;
		}
		out.elemIds.push_back(ints[static_cast<int>(i)]);
		++i;
	}
}

double bdfAtof(const char* str1)
{
	XFoam_String str(str1 ? str1 : "");
	const XFoam_Size len = str.size();
	for (XFoam_Size i = 0; i < len; ++i)
	{
		if (str[i] == 'E' || str[i] == 'e')
		{
			return std::atof(str.c_str());
		}
		if (str[i] == 'D' || str[i] == 'd')
		{
			str[i] = 'E';
			return std::atof(str.c_str());
		}
	}
	char tmp[128];
	int j = 0;
	int leadingMinus = 1;
	for (XFoam_Size i = 0; i < len; ++i)
	{
		if (leadingMinus && str[i] != ' ' && str[i] != '-')
		{
			leadingMinus = 0;
		}
		if (!leadingMinus && str[i] == '-')
		{
			tmp[j++] = 'E';
		}
		if (str[i] == '+')
		{
			tmp[j++] = 'E';
		}
		tmp[j++] = str[i];
	}
	tmp[j] = '\0';
	return std::atof(tmp);
}

XFoam_String field8(const XFoam_String& line, unsigned fieldIndex)
{
	const XFoam_Size pos = static_cast<XFoam_Size>(fieldIndex) * 8u;
	if (pos >= line.size())
	{
		return {};
	}
	const XFoam_Size n = line.size() - pos;
	return line.substr(pos, n < 8u ? n : 8u);
}

void trimString(XFoam_String& s)
{
	while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
	{
		s.pop_back();
	}
	const auto p = s.find_first_not_of(" \t\r");
	if (p != XFoam_String::npos)
	{
		s.erase(0, p);
	}
}

bool lineStartsWith(const XFoam_String& line, const char* keyword)
{
	const auto p = line.find_first_not_of(" \t\r");
	if (p == XFoam_String::npos)
	{
		return false;
	}
	const XFoam_Size klen = static_cast<XFoam_Size>(::strlen(keyword));
	if (line.size() < p + klen)
	{
		return false;
	}
	return line.compare(p, klen, keyword) == 0;
}

void readGridOneLine(
	const XFoam_String& str,
	std::unordered_map<int, XFoam_Label>& idToIdx,
	XFoam_List<XFoam_Vector3D>& positions)
{
	if (str.size() < 48u)
	{
		return;
	}
	XFoam_String f1 = field8(str, 1);
	trimString(f1);
	const int id = std::atoi(f1.c_str());
	XFoam_String fx = field8(str, 3);
	XFoam_String fy = field8(str, 4);
	XFoam_String fz = field8(str, 5);
	trimString(fx);
	trimString(fy);
	trimString(fz);
	const XFoam_Scalar x = static_cast<XFoam_Scalar>(bdfAtof(fx.c_str()));
	const XFoam_Scalar y = static_cast<XFoam_Scalar>(bdfAtof(fy.c_str()));
	const XFoam_Scalar z = static_cast<XFoam_Scalar>(bdfAtof(fz.c_str()));

	const XFoam_Label idx = positions.size();
	idToIdx[id] = idx;
	positions.append(XFoam_Vector3D(x, y, z));
}

void readGridStar(
	const XFoam_String& str,
	const XFoam_String& str22,
	std::unordered_map<int, XFoam_Label>& idToIdx,
	XFoam_List<XFoam_Vector3D>& positions)
{
	if (str.size() < 8 + 16 + 16 + 16 || str22.size() < 8 + 1)
	{
		return;
	}
	const XFoam_String str0 = str.substr(8, 16);
	const XFoam_String str1 = str.substr(8 + 16 + 16, 16);
	const XFoam_String str2 = str.substr(8 + 16 + 16 + 16);
	XFoam_String str3 = str22.substr(8);
	trimString(str3);
	XFoam_String t0 = str0;
	XFoam_String t1 = str1;
	XFoam_String t2 = str2;
	trimString(t0);
	trimString(t1);
	trimString(t2);
	const int id = std::atoi(t0.c_str());
	const XFoam_Scalar x = static_cast<XFoam_Scalar>(bdfAtof(t1.c_str()));
	const XFoam_Scalar y = static_cast<XFoam_Scalar>(bdfAtof(t2.c_str()));
	const XFoam_Scalar z = static_cast<XFoam_Scalar>(bdfAtof(str3.c_str()));
	const XFoam_Label idx = positions.size();
	idToIdx[id] = idx;
	positions.append(XFoam_Vector3D(x, y, z));
}

XFoam_Label nodeIndex(
	const std::unordered_map<int, XFoam_Label>& idToIdx,
	int nid,
	const char* ctx)
{
	const auto it = idToIdx.find(nid);
	if (it == idToIdx.end())
	{
		throw XFoam_Error(
			XFoam_String(ctx) + ": unknown GRID id " + std::to_string(nid));
	}
	return it->second;
}

// ---------- ASCII STL parsing helpers ----------
bool startsWithIgnoreCase(const std::string& s, const char* prefix)
{
	const size_t n = std::strlen(prefix);
	size_t i = 0;
	while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
	if (s.size() - i < n) return false;
	for (size_t k = 0; k < n; ++k)
	{
		if (std::tolower(static_cast<unsigned char>(s[i + k])) !=
		    std::tolower(static_cast<unsigned char>(prefix[k])))
		{
			return false;
		}
	}
	return true;
}

} // namespace

// ============================================================================
// XFoam_VBrep
// ============================================================================
XFoam_VBrep::XFoam_VBrep()  = default;
XFoam_VBrep::~XFoam_VBrep() = default;

void XFoam_VBrep::clear()
{
	positions_.clear();
	edges_.clear();
	faces_.clear();
	patchNames_.clear();
	patchTypes_.clear();
	invalidateAcceleration();
	featuresBuilt_ = false;
	featureEdges_.clear();
	featureVerts_.clear();
}

XFoam_BoundBox XFoam_VBrep::bounds() const
{
	if (positions_.size() == 0) return XFoam_BoundBox::invertedBox;
	return XFoam_BoundBox(positions_);
}

XFoam_BoundBox XFoam_VBrep::refBounds(const XFoam_BrepRef& r) const
{
	if (r.kind != XFoam_BrepKind::Triangulated || !r.valid())
	{
		return XFoam_BoundBox::invertedBox;
	}
	// 这里允许 idx 同时指 vertex / edge / face；我们用 sub 作为粗类别提示，
	// 但更稳的是按 idx 范围尝试。当前调用方仅 face / vertex / edge 三种，
	// 先按 face 优先（snappy 路径上最常用）。
	if (r.idx < nFaces())
	{
		const auto& f = faces_[r.idx];
		const XFoam_Vector3D v0 = positions_[f.verts[0]];
		const XFoam_Vector3D v1 = positions_[f.verts[1]];
		const XFoam_Vector3D v2 = positions_[f.verts[2]];
		const XFoam_Vector3D mn(
			std::min(std::min(v0.x(), v1.x()), v2.x()),
			std::min(std::min(v0.y(), v1.y()), v2.y()),
			std::min(std::min(v0.z(), v1.z()), v2.z()));
		const XFoam_Vector3D mx(
			std::max(std::max(v0.x(), v1.x()), v2.x()),
			std::max(std::max(v0.y(), v1.y()), v2.y()),
			std::max(std::max(v0.z(), v1.z()), v2.z()));
		return XFoam_BoundBox(mn, mx);
	}
	if (r.idx < nVerts())
	{
		const XFoam_Vector3D p = positions_[r.idx];
		return XFoam_BoundBox(p, p);
	}
	return XFoam_BoundBox::invertedBox;
}

void XFoam_VBrep::ensurePatches(XFoam_Label nPatches)
{
	while (patchNames_.size() < nPatches)
	{
		std::ostringstream oss;
		oss << "patch_" << patchNames_.size();
		patchNames_.append(XFoam_Word(oss.str()));
		patchTypes_.append(XFoam_Word("wall"));
	}
}

void XFoam_VBrep::setPatchName(XFoam_Label id, const XFoam_Word& name)
{
	ensurePatches(id + 1);
	patchNames_[id] = name;
}

void XFoam_VBrep::setPatchType(XFoam_Label id, const XFoam_Word& type)
{
	ensurePatches(id + 1);
	patchTypes_[id] = type;
}

// ----- I/O ---------------------------------------------------------------
void XFoam_VBrep::readFromBdf(const XFoam_String& fileName)
{
	clear();
	std::ifstream ifs(fileName.c_str());
	if (!ifs)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_VBrep::readFromBdf: cannot open ") + fileName);
	}

	std::unordered_map<int, XFoam_Label> idToIdx;
	idToIdx.reserve(4096);
	std::unordered_map<int, XFoam_Label> elemIdToIx;
	elemIdToIx.reserve(4096);
	std::vector<HmMoveBlock> hmMoves;
	hmMoves.reserve(16);

	// 中间表：每个 element 是 3 或 4 节点；patchId 一开始全 -1，最后用
	// $HMMOVE 信息回填。先按 (i0,i1,i2[,i3]) 缓存，BDF 全读完后再做三角化
	// （quad → 2 tri）；这样 patchId 回填 + 三角化 + face 写入 faces_ 一次完成。
	struct StagedElem
	{
		XFoam_FixedList<XFoam_Label, 4> n;
		bool isQuad = false; // false → 用前 3 个；true → 用 4 个，拆 2 tri
	};
	std::vector<StagedElem> staged;
	staged.reserve(4096);

	XFoam_String pending;
	XFoam_String line;

	while (true)
	{
		if (!pending.empty())
		{
			line = pending;
			pending.clear();
		}
		else if (!std::getline(ifs, line))
		{
			break;
		}

		if (lineStartsWith(line, "$HMMOVE"))
		{
			HmMoveBlock mv;
			readHmMoveBlock(ifs, line, pending, mv);
			hmMoves.push_back(XFoam_move(mv));
			continue;
		}

		if (lineStartsWith(line, "GRID*"))
		{
			XFoam_String cont;
			if (!std::getline(ifs, cont))
			{
				break;
			}
			readGridStar(line, cont, idToIdx, positions_);
		}
		else if (lineStartsWith(line, "GRID"))
		{
			readGridOneLine(line, idToIdx, positions_);
		}
		else if (lineStartsWith(line, "CTRIA3"))
		{
			if (line.size() < 48u) continue;
			XFoam_String s1 = field8(line, 1);
			XFoam_String s3 = field8(line, 3);
			XFoam_String s4 = field8(line, 4);
			XFoam_String s5 = field8(line, 5);
			trimString(s1); trimString(s3); trimString(s4); trimString(s5);
			const int elemId = std::atoi(s1.c_str());
			const int g1 = std::atoi(s3.c_str());
			const int g2 = std::atoi(s4.c_str());
			const int g3 = std::atoi(s5.c_str());
			StagedElem se;
			se.n[0] = nodeIndex(idToIdx, g1, "CTRIA3");
			se.n[1] = nodeIndex(idToIdx, g2, "CTRIA3");
			se.n[2] = nodeIndex(idToIdx, g3, "CTRIA3");
			se.n[3] = se.n[2];
			se.isQuad = false;
			elemIdToIx[elemId] = static_cast<XFoam_Label>(staged.size());
			staged.push_back(se);
		}
		else if (lineStartsWith(line, "CQUAD4"))
		{
			if (line.size() < 56u) continue;
			XFoam_String s1 = field8(line, 1);
			XFoam_String s3 = field8(line, 3);
			XFoam_String s4 = field8(line, 4);
			XFoam_String s5 = field8(line, 5);
			XFoam_String s6 = field8(line, 6);
			trimString(s1); trimString(s3); trimString(s4); trimString(s5); trimString(s6);
			const int elemId = std::atoi(s1.c_str());
			const int g1 = std::atoi(s3.c_str());
			const int g2 = std::atoi(s4.c_str());
			const int g3 = std::atoi(s5.c_str());
			const int g4 = std::atoi(s6.c_str());
			StagedElem se;
			se.n[0] = nodeIndex(idToIdx, g1, "CQUAD4");
			se.n[1] = nodeIndex(idToIdx, g2, "CQUAD4");
			se.n[2] = nodeIndex(idToIdx, g3, "CQUAD4");
			se.n[3] = nodeIndex(idToIdx, g4, "CQUAD4");
			se.isQuad = true;
			elemIdToIx[elemId] = static_cast<XFoam_Label>(staged.size());
			staged.push_back(se);
		}
	}

	// 把 $HMMOVE 列表 → "elemId → patchId" 映射；patchId 按 HMMOVE 出现顺序编号。
	std::unordered_map<int, XFoam_Label> elemId2Patch;
	elemId2Patch.reserve(staged.size());
	XFoam_Label nextPatch = 0;
	for (const HmMoveBlock& mv : hmMoves)
	{
		if (mv.compId < 0 || mv.elemIds.empty()) continue;
		bool allShell = true;
		for (int eid : mv.elemIds)
		{
			if (elemIdToIx.find(eid) == elemIdToIx.end())
			{
				allShell = false;
				break;
			}
		}
		if (!allShell) continue;
		std::ostringstream oss;
		oss << "comp_" << mv.compId;
		setPatchName(nextPatch, XFoam_Word(oss.str()));
		setPatchType(nextPatch, XFoam_Word("wall"));
		for (int eid : mv.elemIds)
		{
			elemId2Patch[eid] = nextPatch;
		}
		++nextPatch;
	}

	// 三角化 + 写入 faces_。quad → (0,1,2)+(0,2,3)。patchId 查 elemId 表，
	// 未命中保持 -1（即 default patch；rebuildIdentity 会落到 "patch_-1"）。
	for (const auto& kv : elemIdToIx)
	{
		const int elemId = kv.first;
		const XFoam_Label sidx = kv.second;
		const StagedElem& se = staged[sidx];
		XFoam_Label pid = -1;
		auto itp = elemId2Patch.find(elemId);
		if (itp != elemId2Patch.end()) pid = itp->second;

		DiscreteFace f0;
		f0.verts[0] = se.n[0];
		f0.verts[1] = se.n[1];
		f0.verts[2] = se.n[2];
		f0.patchId  = pid;
		faces_.append(f0);

		if (se.isQuad)
		{
			DiscreteFace f1;
			f1.verts[0] = se.n[0];
			f1.verts[1] = se.n[2];
			f1.verts[2] = se.n[3];
			f1.patchId  = pid;
			faces_.append(f1);
		}
	}
}

void XFoam_VBrep::readStlAscii(const XFoam_String& fileName)
{
	clear();
	std::ifstream ifs(fileName.c_str());
	if (!ifs)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_VBrep::readStlAscii: cannot open ") + fileName);
	}

	// 简化版 ASCII STL parser：按 solid / endsolid 切 patch；facet normal 行
	// 忽略；只用 vertex 行。同一 vertex 不去重（snappy 后续会按需 dedup）。
	XFoam_Label curPatch = -1;
	std::string line;
	std::vector<XFoam_Vector3D> triBuf; triBuf.reserve(3);
	while (std::getline(ifs, line))
	{
		if (startsWithIgnoreCase(line, "solid"))
		{
			// 解析 solid 名字 → 新 patch
			std::string name;
			std::istringstream iss(line);
			std::string kw; iss >> kw >> name;
			if (name.empty()) name = "solid";
			curPatch = nPatches();
			setPatchName(curPatch, XFoam_Word(name));
			setPatchType(curPatch, XFoam_Word("wall"));
			continue;
		}
		if (startsWithIgnoreCase(line, "endsolid"))
		{
			curPatch = -1;
			continue;
		}
		if (startsWithIgnoreCase(line, "vertex"))
		{
			std::istringstream iss(line);
			std::string kw;
			double x, y, z;
			if (iss >> kw >> x >> y >> z)
			{
				triBuf.push_back(XFoam_Vector3D(
					static_cast<XFoam_Scalar>(x),
					static_cast<XFoam_Scalar>(y),
					static_cast<XFoam_Scalar>(z)));
			}
			if (triBuf.size() == 3)
			{
				const XFoam_Label base = positions_.size();
				positions_.append(triBuf[0]);
				positions_.append(triBuf[1]);
				positions_.append(triBuf[2]);
				DiscreteFace f;
				f.verts[0] = base;
				f.verts[1] = base + 1;
				f.verts[2] = base + 2;
				f.patchId  = (curPatch >= 0) ? curPatch : 0;
				faces_.append(f);
				triBuf.clear();
			}
		}
	}
}

void XFoam_VBrep::readStlBinary(const XFoam_String& fileName)
{
	clear();
	std::ifstream f(fileName.c_str(), std::ios::binary);
	if (!f)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_VBrep::readStlBinary: cannot open ") + fileName);
	}
	char header[80];
	f.read(header, 80);
	std::uint32_t nTri = 0;
	f.read(reinterpret_cast<char*>(&nTri), sizeof(nTri));
	if (!f)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_VBrep::readStlBinary: truncated header in ") + fileName);
	}
	ensurePatches(1);
	setPatchName(0, XFoam_Word("solid"));
	setPatchType(0, XFoam_Word("wall"));
	for (std::uint32_t i = 0; i < nTri; ++i)
	{
		float vals[12];
		std::uint16_t attr = 0;
		f.read(reinterpret_cast<char*>(vals), sizeof(vals));
		f.read(reinterpret_cast<char*>(&attr), sizeof(attr));
		if (!f) break;
		// binary STL 的 attr 字段在工业里偶尔被滥用做 patchId / colour，但同一文件
		// 内多 patch 的 STL 极少见；为简化起见这里一律落到 patchId 0，保留旧
		// XFoam_TriSurface 的行为。如需多 patch 走 ASCII。
		const XFoam_Label base = positions_.size();
		positions_.append(XFoam_Vector3D(
			static_cast<XFoam_Scalar>(vals[3]),
			static_cast<XFoam_Scalar>(vals[4]),
			static_cast<XFoam_Scalar>(vals[5])));
		positions_.append(XFoam_Vector3D(
			static_cast<XFoam_Scalar>(vals[6]),
			static_cast<XFoam_Scalar>(vals[7]),
			static_cast<XFoam_Scalar>(vals[8])));
		positions_.append(XFoam_Vector3D(
			static_cast<XFoam_Scalar>(vals[9]),
			static_cast<XFoam_Scalar>(vals[10]),
			static_cast<XFoam_Scalar>(vals[11])));
		DiscreteFace fc;
		fc.verts[0] = base;
		fc.verts[1] = base + 1;
		fc.verts[2] = base + 2;
		fc.patchId  = 0;
		faces_.append(fc);
	}
}

bool XFoam_VBrep::read(const std::string& path)
{
	// 仿 XFoam_TriSurface::read：先看前 5 字节判断 ASCII / binary，仍模糊就先 ascii
	// 失败回退 binary。读完后 faces_ 仍空 → 视为失败。
	std::ifstream sniff(path.c_str(), std::ios::binary);
	if (!sniff) return false;
	char head[5] = {0};
	sniff.read(head, 5);
	const bool startsSolid =
		(head[0] == 's' || head[0] == 'S') &&
		(head[1] == 'o' || head[1] == 'O') &&
		(head[2] == 'l' || head[2] == 'L') &&
		(head[3] == 'i' || head[3] == 'I') &&
		(head[4] == 'd' || head[4] == 'D');
	sniff.seekg(0, std::ios::end);
	const std::streamoff fileSize = sniff.tellg();
	sniff.close();
	auto tryAscii = [&]() -> bool {
		try { readStlAscii(XFoam_String(path)); return nFaces() > 0; }
		catch (...) { return false; }
	};
	auto tryBinary = [&]() -> bool {
		try { readStlBinary(XFoam_String(path)); return nFaces() > 0; }
		catch (...) { return false; }
	};
	if (startsSolid && tryAscii()) return true;
	if (fileSize >= 84) return tryBinary();
	return false;
}

void XFoam_VBrep::writeStl(const XFoam_String& fileName) const
{
	std::ofstream ofs(fileName.c_str());
	if (!ofs)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_VBrep::writeStl: cannot open ") + fileName);
	}
	// 按 patchId 分块；同一 patchId 的三角面写成一个 solid 块。
	// 简化：直接遍历 faces_，每次发现新 patchId 就开新 solid。要求 faces_ 已按
	// patchId 大致连续；若不连续会得到多个同名 solid，但仍是合法 STL。
	XFoam_Label cur = std::numeric_limits<XFoam_Label>::min();
	for (XFoam_Label fi = 0; fi < nFaces(); ++fi)
	{
		const auto& f = faces_[fi];
		if (f.patchId != cur)
		{
			if (cur != std::numeric_limits<XFoam_Label>::min())
			{
				ofs << "endsolid\n";
			}
			XFoam_Word name = (f.patchId >= 0 && f.patchId < patchNames_.size())
				? patchNames_[f.patchId]
				: XFoam_Word("solid");
			ofs << "solid " << static_cast<const XFoam_String&>(name) << '\n';
			cur = f.patchId;
		}
		const XFoam_Vector3D v0 = positions_[f.verts[0]];
		const XFoam_Vector3D v1 = positions_[f.verts[1]];
		const XFoam_Vector3D v2 = positions_[f.verts[2]];
		const XFoam_Vector3D e1 = v1 - v0;
		const XFoam_Vector3D e2 = v2 - v0;
		XFoam_Vector3D n(
			e1.y() * e2.z() - e1.z() * e2.y(),
			e1.z() * e2.x() - e1.x() * e2.z(),
			e1.x() * e2.y() - e1.y() * e2.x());
		const XFoam_Scalar m = n.mag();
		if (m > 0) n = XFoam_Vector3D(n.x() / m, n.y() / m, n.z() / m);
		ofs << "  facet normal " << n.x() << ' ' << n.y() << ' ' << n.z() << '\n';
		ofs << "    outer loop\n";
		ofs << "      vertex " << v0.x() << ' ' << v0.y() << ' ' << v0.z() << '\n';
		ofs << "      vertex " << v1.x() << ' ' << v1.y() << ' ' << v1.z() << '\n';
		ofs << "      vertex " << v2.x() << ' ' << v2.y() << ' ' << v2.z() << '\n';
		ofs << "    endloop\n";
		ofs << "  endfacet\n";
	}
	if (cur != std::numeric_limits<XFoam_Label>::min())
	{
		ofs << "endsolid\n";
	}
}

// ----- 构建 / 修补 -----------------------------------------------------
void XFoam_VBrep::buildEdgesFromFaces(XFoam_Scalar featureAngleDeg)
{
	edges_.clear();
	if (faces_.size() == 0) return;

	const XFoam_Scalar cosThresh = static_cast<XFoam_Scalar>(
		std::cos(static_cast<double>(featureAngleDeg) * 3.14159265358979323846 / 180.0));

	// (vMin, vMax) → 已收录 edge 在 edges_ 里的下标
	struct EdgeKey { XFoam_Label a, b; };
	struct EdgeKeyHash
	{
		size_t operator()(const EdgeKey& k) const noexcept
		{
			return std::hash<XFoam_Label>()(k.a) * 0x9E3779B97F4A7C15ULL ^
			       std::hash<XFoam_Label>()(k.b);
		}
	};
	struct EdgeKeyEq
	{
		bool operator()(const EdgeKey& l, const EdgeKey& r) const noexcept
		{
			return l.a == r.a && l.b == r.b;
		}
	};
	std::unordered_map<EdgeKey, XFoam_Label, EdgeKeyHash, EdgeKeyEq> idx;
	idx.reserve(faces_.size() * 3);

	auto faceNormal = [&](const DiscreteFace& f) -> XFoam_Vector3D {
		const XFoam_Vector3D v0 = positions_[f.verts[0]];
		const XFoam_Vector3D v1 = positions_[f.verts[1]];
		const XFoam_Vector3D v2 = positions_[f.verts[2]];
		const XFoam_Vector3D e1 = v1 - v0;
		const XFoam_Vector3D e2 = v2 - v0;
		XFoam_Vector3D n(
			e1.y() * e2.z() - e1.z() * e2.y(),
			e1.z() * e2.x() - e1.x() * e2.z(),
			e1.x() * e2.y() - e1.y() * e2.x());
		const XFoam_Scalar m = n.mag();
		if (m > 0) n = XFoam_Vector3D(n.x() / m, n.y() / m, n.z() / m);
		return n;
	};

	for (XFoam_Label fi = 0; fi < faces_.size(); ++fi)
	{
		const DiscreteFace& f = faces_[fi];
		for (int s = 0; s < 3; ++s)
		{
			XFoam_Label va = f.verts[s];
			XFoam_Label vb = f.verts[(s + 1) % 3];
			EdgeKey key{std::min(va, vb), std::max(va, vb)};
			auto it = idx.find(key);
			if (it == idx.end())
			{
				DiscreteEdge e;
				e.verts[0] = key.a;
				e.verts[1] = key.b;
				e.faceL = fi;
				e.faceR = -1;
				e.dihedralCos = 1;
				e.isFeature   = false;
				e.isBoundary  = true; // 暂定 boundary，找到第二邻面时清掉
				idx.emplace(key, edges_.size());
				edges_.append(e);
			}
			else
			{
				DiscreteEdge& e = edges_[it->second];
				if (e.faceR == -1)
				{
					e.faceR = fi;
					e.isBoundary = false;
					const XFoam_Vector3D nL = faceNormal(faces_[e.faceL]);
					const XFoam_Vector3D nR = faceNormal(faces_[e.faceR]);
					e.dihedralCos = nL.x() * nR.x() + nL.y() * nR.y() + nL.z() * nR.z();
					e.isFeature = (e.dihedralCos < cosThresh);
				}
				// 第 3+ 个面共享同一边 → 非流形；保留首两个邻面，标 feature。
				else
				{
					e.isFeature = true;
				}
			}
		}
	}

	// boundary edge 也算 feature（snappy 习惯）。
	for (XFoam_Label ei = 0; ei < edges_.size(); ++ei)
	{
		if (edges_[ei].isBoundary) edges_[ei].isFeature = true;
	}
}

XFoam_Label XFoam_VBrep::stitchBorders(XFoam_Scalar /*tol*/)
{
	// TODO: KD-tree 搜近邻 vertex 合并；首期 no-op。
	return 0;
}

XFoam_Label XFoam_VBrep::unifySameDomain(XFoam_Scalar /*coplanarTolDeg*/)
{
	// TODO: union-find 跨非 feature edge 把三角面归一个 patchId；首期 no-op。
	return 0;
}

// =============================================================================
// 几何查询加速 / 几何谓词 ── 全部从原 XFoam_TriSurface 迁过来。
// 设计要点：
//   * triCache_ / bvhNodes_ / bvhOrder_ / featureEdges_ / featureVerts_ 都是
//     mutable cache，正常 const 访问；外部改 positions_/faces_ 后必须
//     invalidateAcceleration()。
//   * ensureAcceleration 在第一次查询时一次性建好 triCache + BVH + bounds，
//     之后查询直接走 BVH 剪枝。
// =============================================================================
namespace
{

// Christer Ericson, Real-Time Collision Detection §5.1.5：点到三角形最近点。
XFoam_Vector3D closestPointOnTriangle(
	const XFoam_Vector3D& p,
	const XFoam_Vector3D& a,
	const XFoam_Vector3D& b,
	const XFoam_Vector3D& c,
	XFoam_Scalar& dist2)
{
	const XFoam_Vector3D ab = b - a;
	const XFoam_Vector3D ac = c - a;
	const XFoam_Vector3D ap = p - a;
	const XFoam_Scalar d1 = ab & ap;
	const XFoam_Scalar d2 = ac & ap;
	if (d1 <= 0 && d2 <= 0)
	{
		dist2 = (p - a).magSqr();
		return a;
	}
	const XFoam_Vector3D bp = p - b;
	const XFoam_Scalar d3 = ab & bp;
	const XFoam_Scalar d4 = ac & bp;
	if (d3 >= 0 && d4 <= d3)
	{
		dist2 = (p - b).magSqr();
		return b;
	}
	const XFoam_Scalar vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0)
	{
		const XFoam_Scalar v = d1 / (d1 - d3);
		const XFoam_Vector3D q = a + ab * v;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Vector3D cp = p - c;
	const XFoam_Scalar d5 = ab & cp;
	const XFoam_Scalar d6 = ac & cp;
	if (d6 >= 0 && d5 <= d6)
	{
		dist2 = (p - c).magSqr();
		return c;
	}
	const XFoam_Scalar vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0)
	{
		const XFoam_Scalar w = d2 / (d2 - d6);
		const XFoam_Vector3D q = a + ac * w;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Scalar va = d3 * d6 - d5 * d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
	{
		const XFoam_Scalar w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		const XFoam_Vector3D q = b + (c - b) * w;
		dist2 = (p - q).magSqr();
		return q;
	}
	const XFoam_Scalar denom = static_cast<XFoam_Scalar>(1) / (va + vb + vc);
	const XFoam_Scalar v = vb * denom;
	const XFoam_Scalar w = vc * denom;
	const XFoam_Vector3D q = a + ab * v + ac * w;
	dist2 = (p - q).magSqr();
	return q;
}

// Möller-Trumbore：dir = (1,0,0)，命中 t > 0 计 1 次（用于内外射线计数）。
bool rayHitsTrianglePlusX(
	const XFoam_Vector3D& orig,
	const XFoam_Vector3D& a,
	const XFoam_Vector3D& b,
	const XFoam_Vector3D& c)
{
	const XFoam_Vector3D edge1 = b - a;
	const XFoam_Vector3D edge2 = c - a;
	const XFoam_Vector3D pvec(0, -edge2.z(), edge2.y());
	const XFoam_Scalar det = edge1 & pvec;
	const XFoam_Scalar eps = static_cast<XFoam_Scalar>(1e-12);
	if (det > -eps && det < eps) return false;
	const XFoam_Scalar invDet = static_cast<XFoam_Scalar>(1) / det;
	const XFoam_Vector3D tvec = orig - a;
	const XFoam_Scalar u = (tvec & pvec) * invDet;
	if (u < 0 || u > 1) return false;
	const XFoam_Vector3D qvec = tvec ^ edge1;
	const XFoam_Scalar v = qvec.x() * invDet;
	if (v < 0 || u + v > 1) return false;
	const XFoam_Scalar t = (edge2 & qvec) * invDet;
	return t > eps;
}

inline XFoam_Scalar pointSegmentDistSqr(
	const XFoam_Vector3D& p,
	const XFoam_Vector3D& a,
	const XFoam_Vector3D& b,
	XFoam_Vector3D& outClosest)
{
	const XFoam_Vector3D ab = b - a;
	const XFoam_Scalar abLen2 = ab.x() * ab.x() + ab.y() * ab.y() + ab.z() * ab.z();
	if (abLen2 <= 0)
	{
		outClosest = a;
		const XFoam_Vector3D d = p - a;
		return d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
	}
	const XFoam_Vector3D ap = p - a;
	XFoam_Scalar t = (ap.x() * ab.x() + ap.y() * ab.y() + ap.z() * ab.z()) / abLen2;
	if (t < 0) t = 0;
	if (t > 1) t = 1;
	outClosest = a + ab * t;
	const XFoam_Vector3D d = p - outClosest;
	return d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
}

} // namespace

void XFoam_VBrep::ensureAcceleration() const
{
	if (accelBuilt_) return;
	triCache_.clear();
	bvhNodes_.clear();
	bvhOrder_.clear();
	const size_t nF = static_cast<size_t>(faces_.size());
	triCache_.reserve(nF);
	for (XFoam_Label i = 0; i < faces_.size(); ++i)
	{
		const DiscreteFace& f = faces_[i];
		const XFoam_Vector3D& a = positions_[f.verts[0]];
		const XFoam_Vector3D& b = positions_[f.verts[1]];
		const XFoam_Vector3D& c = positions_[f.verts[2]];
		TriCache t;
		t.v0 = a; t.v1 = b; t.v2 = c;
		t.normal = XFoam_normalised((b - a) ^ (c - a));
		t.bbox = XFoam_BoundBox(
			XFoam_Vector3D(
				std::min({a.x(), b.x(), c.x()}),
				std::min({a.y(), b.y(), c.y()}),
				std::min({a.z(), b.z(), c.z()})),
			XFoam_Vector3D(
				std::max({a.x(), b.x(), c.x()}),
				std::max({a.y(), b.y(), c.y()}),
				std::max({a.z(), b.z(), c.z()})));
		triCache_.push_back(t);
	}
	// 重算 cached bounds（snap.contains 用它做抖动尺度）
	if (triCache_.empty())
	{
		cachedBounds_ = XFoam_BoundBox::invertedBox;
	}
	else
	{
		XFoam_Vector3D mn = triCache_[0].bbox.min();
		XFoam_Vector3D mx = triCache_[0].bbox.max();
		for (size_t i = 1; i < triCache_.size(); ++i)
		{
			const XFoam_BoundBox& bb = triCache_[i].bbox;
			mn.x() = std::min(mn.x(), bb.min().x());
			mn.y() = std::min(mn.y(), bb.min().y());
			mn.z() = std::min(mn.z(), bb.min().z());
			mx.x() = std::max(mx.x(), bb.max().x());
			mx.y() = std::max(mx.y(), bb.max().y());
			mx.z() = std::max(mx.z(), bb.max().z());
		}
		cachedBounds_ = XFoam_BoundBox(mn, mx);
	}
	buildBvh();
	accelBuilt_ = true;
}

void XFoam_VBrep::buildBvh() const
{
	bvhNodes_.clear();
	bvhOrder_.clear();
	if (triCache_.empty()) return;
	bvhOrder_.resize(triCache_.size());
	for (size_t i = 0; i < triCache_.size(); ++i)
	{
		bvhOrder_[i] = static_cast<XFoam_Label>(i);
	}
	bvhNodes_.reserve(2 * triCache_.size());
	buildBvhRecursive(0, static_cast<int>(triCache_.size()));
}

int XFoam_VBrep::buildBvhRecursive(int lo, int hi) const
{
	bvhNodes_.emplace_back();
	const int nodeIdx = static_cast<int>(bvhNodes_.size() - 1);

	XFoam_BoundBox bb = triCache_[bvhOrder_[lo]].bbox;
	for (int i = lo + 1; i < hi; ++i)
	{
		const XFoam_BoundBox& tb = triCache_[bvhOrder_[i]].bbox;
		bb.min().x() = std::min(bb.min().x(), tb.min().x());
		bb.min().y() = std::min(bb.min().y(), tb.min().y());
		bb.min().z() = std::min(bb.min().z(), tb.min().z());
		bb.max().x() = std::max(bb.max().x(), tb.max().x());
		bb.max().y() = std::max(bb.max().y(), tb.max().y());
		bb.max().z() = std::max(bb.max().z(), tb.max().z());
	}

	const int count = hi - lo;
	if (count <= kBvhLeafLimit)
	{
		bvhNodes_[nodeIdx].bbox     = bb;
		bvhNodes_[nodeIdx].firstTri = lo;
		bvhNodes_[nodeIdx].triCount = count;
		bvhNodes_[nodeIdx].leftIdx  = -1;
		bvhNodes_[nodeIdx].rightIdx = -1;
		return nodeIdx;
	}

	const XFoam_Scalar ex = bb.max().x() - bb.min().x();
	const XFoam_Scalar ey = bb.max().y() - bb.min().y();
	const XFoam_Scalar ez = bb.max().z() - bb.min().z();
	int axis = 0;
	if (ey > ex) axis = 1;
	if (axis == 0 ? (ez > ex) : (ez > ey)) axis = 2;

	auto centroidOnAxis = [&](XFoam_Label triIdx) -> XFoam_Scalar {
		const XFoam_BoundBox& tb = triCache_[triIdx].bbox;
		switch (axis)
		{
		case 0:  return static_cast<XFoam_Scalar>(0.5) * (tb.min().x() + tb.max().x());
		case 1:  return static_cast<XFoam_Scalar>(0.5) * (tb.min().y() + tb.max().y());
		default: return static_cast<XFoam_Scalar>(0.5) * (tb.min().z() + tb.max().z());
		}
	};
	std::sort(bvhOrder_.begin() + lo, bvhOrder_.begin() + hi,
		[&](XFoam_Label a, XFoam_Label b) {
			return centroidOnAxis(a) < centroidOnAxis(b);
		});

	const int mid       = (lo + hi) / 2;
	const int leftIdx   = buildBvhRecursive(lo, mid);
	const int rightIdx  = buildBvhRecursive(mid, hi);
	bvhNodes_[nodeIdx].bbox     = bb;
	bvhNodes_[nodeIdx].firstTri = -1;
	bvhNodes_[nodeIdx].triCount = 0;
	bvhNodes_[nodeIdx].leftIdx  = leftIdx;
	bvhNodes_[nodeIdx].rightIdx = rightIdx;
	return nodeIdx;
}

XFoam_Scalar XFoam_VBrep::bboxMinDistSqr(
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

void XFoam_VBrep::bvhClosestPoint(
	const XFoam_Vector3D& p,
	XFoam_Scalar&         bestD2,
	XFoam_Vector3D&       bestQ,
	XFoam_Label&          bestTri,
	int                   nodeIdx) const
{
	const BvhNode& n = bvhNodes_[nodeIdx];
	if (bboxMinDistSqr(p, n.bbox) >= bestD2) return;

	if (n.triCount > 0)
	{
		for (int i = 0; i < n.triCount; ++i)
		{
			const XFoam_Label triIdx = bvhOrder_[n.firstTri + i];
			const TriCache& t = triCache_[triIdx];
			XFoam_Scalar d2 = 0;
			const XFoam_Vector3D q = closestPointOnTriangle(p, t.v0, t.v1, t.v2, d2);
			if (d2 < bestD2)
			{
				bestD2  = d2;
				bestQ   = q;
				bestTri = triIdx;
			}
		}
		return;
	}

	const XFoam_Scalar lD2 = bboxMinDistSqr(p, bvhNodes_[n.leftIdx].bbox);
	const XFoam_Scalar rD2 = bboxMinDistSqr(p, bvhNodes_[n.rightIdx].bbox);
	int firstChild, secondChild;
	XFoam_Scalar firstD2, secondD2;
	if (lD2 <= rD2)
	{
		firstChild  = n.leftIdx;  firstD2  = lD2;
		secondChild = n.rightIdx; secondD2 = rD2;
	}
	else
	{
		firstChild  = n.rightIdx; firstD2  = rD2;
		secondChild = n.leftIdx;  secondD2 = lD2;
	}
	if (firstD2  < bestD2) bvhClosestPoint(p, bestD2, bestQ, bestTri, firstChild);
	if (secondD2 < bestD2) bvhClosestPoint(p, bestD2, bestQ, bestTri, secondChild);
}

int XFoam_VBrep::bvhRayCountPlusX(const XFoam_Vector3D& p, int nodeIdx) const
{
	const BvhNode& n = bvhNodes_[nodeIdx];
	if (n.bbox.max().x() < p.x()) return 0;
	if (p.y() < n.bbox.min().y() || p.y() > n.bbox.max().y()) return 0;
	if (p.z() < n.bbox.min().z() || p.z() > n.bbox.max().z()) return 0;

	if (n.triCount > 0)
	{
		int count = 0;
		for (int i = 0; i < n.triCount; ++i)
		{
			const XFoam_Label triIdx = bvhOrder_[n.firstTri + i];
			const TriCache& t = triCache_[triIdx];
			if (p.y() < t.bbox.min().y() || p.y() > t.bbox.max().y()) continue;
			if (p.z() < t.bbox.min().z() || p.z() > t.bbox.max().z()) continue;
			if (t.bbox.max().x() < p.x()) continue;
			if (rayHitsTrianglePlusX(p, t.v0, t.v1, t.v2)) ++count;
		}
		return count;
	}
	return bvhRayCountPlusX(p, n.leftIdx) + bvhRayCountPlusX(p, n.rightIdx);
}

bool XFoam_VBrep::bvhBoxIntersects(const XFoam_BoundBox& q, int nodeIdx) const
{
	const BvhNode& n = bvhNodes_[nodeIdx];
	if (!q.overlaps(n.bbox)) return false;
	if (n.triCount > 0)
	{
		for (int i = 0; i < n.triCount; ++i)
		{
			const XFoam_Label triIdx = bvhOrder_[n.firstTri + i];
			if (q.overlaps(triCache_[triIdx].bbox)) return true;
		}
		return false;
	}
	if (bvhBoxIntersects(q, n.leftIdx)) return true;
	return bvhBoxIntersects(q, n.rightIdx);
}

int XFoam_VBrep::rayCountPlusX(const XFoam_Vector3D& p) const
{
	ensureAcceleration();
	if (!bvhNodes_.empty()) return bvhRayCountPlusX(p, 0);
	int count = 0;
	for (size_t i = 0; i < triCache_.size(); ++i)
	{
		const TriCache& t = triCache_[i];
		if (p.y() < t.bbox.min().y() || p.y() > t.bbox.max().y()) continue;
		if (p.z() < t.bbox.min().z() || p.z() > t.bbox.max().z()) continue;
		if (p.x() > t.bbox.max().x()) continue;
		if (rayHitsTrianglePlusX(p, t.v0, t.v1, t.v2)) ++count;
	}
	return count;
}

bool XFoam_VBrep::contains(const XFoam_Vector3D& p) const
{
	ensureAcceleration();
	if (triCache_.empty()) return false;
	// 在 y/z 上加小抖动，避开 +X 射线穿过三角面顶点 / 棱时奇偶判定退化。
	const XFoam_Scalar sp  = static_cast<XFoam_Scalar>(cachedBounds_.mag());
	const XFoam_Scalar eps = std::max(static_cast<XFoam_Scalar>(1e-9),
	                                  sp * static_cast<XFoam_Scalar>(1e-7));
	const XFoam_Vector3D pp(
		p.x(),
		p.y() + eps * static_cast<XFoam_Scalar>(0.7314159),
		p.z() + eps * static_cast<XFoam_Scalar>(0.4142135));
	return (rayCountPlusX(pp) & 1) != 0;
}

bool XFoam_VBrep::boxIntersects(const XFoam_BoundBox& box) const
{
	ensureAcceleration();
	if (triCache_.empty()) return false;
	if (!bvhNodes_.empty()) return bvhBoxIntersects(box, 0);
	for (size_t i = 0; i < triCache_.size(); ++i)
	{
		if (box.overlaps(triCache_[i].bbox)) return true;
	}
	return false;
}

void XFoam_VBrep::closestPointAndNormal(
	const XFoam_Vector3D& p,
	XFoam_Vector3D&       outClosest,
	XFoam_Vector3D&       outNormal) const
{
	ensureAcceleration();
	if (triCache_.empty())
	{
		outClosest = p;
		outNormal  = XFoam_Vector3D(0, 0, 0);
		return;
	}
	XFoam_Scalar bestD2 = std::numeric_limits<XFoam_Scalar>::infinity();
	XFoam_Vector3D bestQ = triCache_[0].v0;
	XFoam_Label bestTri  = 0;
	if (!bvhNodes_.empty())
	{
		bvhClosestPoint(p, bestD2, bestQ, bestTri, 0);
	}
	else
	{
		for (size_t i = 0; i < triCache_.size(); ++i)
		{
			XFoam_Scalar d2 = 0;
			const XFoam_Vector3D q = closestPointOnTriangle(
				p, triCache_[i].v0, triCache_[i].v1, triCache_[i].v2, d2);
			if (d2 < bestD2)
			{
				bestD2  = d2;
				bestTri = static_cast<XFoam_Label>(i);
				bestQ   = q;
			}
		}
	}
	outClosest = bestQ;
	outNormal  = triCache_[bestTri].normal;
}

void XFoam_VBrep::rebuildFeaturesGeomFromEdges(XFoam_Scalar /*featureAngleDeg*/) const
{
	featureEdges_.clear();
	featureVerts_.clear();

	// 由 buildEdgesFromFaces 已经填好的 edges_.isFeature / .isBoundary → 物化点对
	// + 统计每个 vertex 入射的 feature edge 数。
	std::unordered_map<XFoam_Label, int> degree;
	degree.reserve(static_cast<size_t>(edges_.size() * 2));
	for (XFoam_Label ei = 0; ei < edges_.size(); ++ei)
	{
		const DiscreteEdge& e = edges_[ei];
		if (!e.isFeature) continue;
		const XFoam_Label va = e.verts[0];
		const XFoam_Label vb = e.verts[1];
		if (va < 0 || vb < 0
			|| va >= positions_.size() || vb >= positions_.size()) continue;
		FeatureEdgeGeom fe;
		fe.p1 = positions_[va];
		fe.p2 = positions_[vb];
		featureEdges_.push_back(fe);
		++degree[va];
		++degree[vb];
	}
	for (const auto& kv : degree)
	{
		if (kv.second >= 3)
		{
			FeatureVertGeom fv;
			fv.p = positions_[kv.first];
			featureVerts_.push_back(fv);
		}
	}
}

void XFoam_VBrep::buildFeatures(XFoam_Scalar featureAngleDeg)
{
	// buildEdgesFromFaces 是真正的算 + 标 logic；它会填 edges_ + 设 isFeature。
	// 之后把 isFeature edges 物化到 featureEdges_/featureVerts_ 缓存。
	buildEdgesFromFaces(featureAngleDeg);
	featureAngleDegCached_ = featureAngleDeg;
	rebuildFeaturesGeomFromEdges(featureAngleDeg);
	featuresBuilt_ = true;
	// edge 列表变了 → BVH 不受影响（只看 faces），但保守起见也标重建。
	invalidateAcceleration();
}

XFoam_BrepBase::FeatureKind XFoam_VBrep::closestFeature(
	const XFoam_Vector3D& p,
	XFoam_Scalar          searchRadius,
	XFoam_Vector3D&       outClosest,
	XFoam_Vector3D&       outTangent) const
{
	outTangent = XFoam_Vector3D(0, 0, 0);
	if (featureEdges_.empty() && featureVerts_.empty()) return FeatureKind::None;
	const XFoam_Scalar r2 = searchRadius * searchRadius;
	XFoam_Scalar bestD2 = r2;
	FeatureKind bestKind = FeatureKind::None;
	XFoam_Vector3D bestQ;
	size_t bestEdgeIdx = static_cast<size_t>(-1);

	// vertex 优先（snap 到尖角更稳）
	for (size_t i = 0; i < featureVerts_.size(); ++i)
	{
		const XFoam_Vector3D& fp = featureVerts_[i].p;
		const XFoam_Vector3D d = fp - p;
		const XFoam_Scalar d2 = d.x() * d.x() + d.y() * d.y() + d.z() * d.z();
		if (d2 < bestD2)
		{
			bestD2   = d2;
			bestKind = FeatureKind::Vertex;
			bestQ    = fp;
		}
	}
	for (size_t i = 0; i < featureEdges_.size(); ++i)
	{
		XFoam_Vector3D q;
		const XFoam_Scalar d2 = pointSegmentDistSqr(
			p, featureEdges_[i].p1, featureEdges_[i].p2, q);
		if (d2 < bestD2)
		{
			bestD2       = d2;
			bestKind     = FeatureKind::Edge;
			bestQ        = q;
			bestEdgeIdx  = i;
		}
	}
	if (bestKind != FeatureKind::None) outClosest = bestQ;
	if (bestKind == FeatureKind::Edge)
	{
		const FeatureEdgeGeom& fe = featureEdges_[bestEdgeIdx];
		XFoam_Vector3D t(fe.p2.x() - fe.p1.x(),
		                 fe.p2.y() - fe.p1.y(),
		                 fe.p2.z() - fe.p1.z());
		const XFoam_Scalar m = t.mag();
		if (m > 0)
		{
			const XFoam_Scalar inv = static_cast<XFoam_Scalar>(1) / m;
			outTangent = XFoam_Vector3D(t.x() * inv, t.y() * inv, t.z() * inv);
		}
	}
	return bestKind;
}
