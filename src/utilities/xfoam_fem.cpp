#include "XFoam/utilities/xfoam_fem.h"
#include "XFoam/utilities/xfoam_error.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace
{

XFoam_String XF_trimCopy(const XFoam_String& s)
{
	XFoam_Size a = 0;
	XFoam_Size b = static_cast<XFoam_Size>(s.size());
	while (a < b && std::isspace(static_cast<unsigned char>(s[static_cast<XFoam_Size>(a)])))
	{
		++a;
	}
	while (b > a && std::isspace(static_cast<unsigned char>(s[static_cast<XFoam_Size>(b - 1)])))
	{
		--b;
	}
	return s.substr(static_cast<XFoam_Size>(a), static_cast<XFoam_Size>(b - a));
}

XFoam_String XF_bulkField8(const XFoam_String& line, XFoam_Label fieldIndex)
{
	const size_t off = static_cast<size_t>(fieldIndex) * 8u;
	if (off >= line.size())
	{
		return XFoam_String();
	}
	const size_t n = std::min<size_t>(8u, line.size() - off);
	return line.substr(off, n);
}

double XF_parseNastranDouble(const XFoam_String& rawIn)
{
	XFoam_String raw = XF_trimCopy(rawIn);
	if (raw.empty())
	{
		return 0.0;
	}
	for (size_t i = raw.size(); i-- > 1;)
	{
		const char c = raw[i];
		if ((c == '+' || c == '-') && i > 0 && std::isdigit(static_cast<unsigned char>(raw[i - 1]))
			&& i + 1 < raw.size() && std::isdigit(static_cast<unsigned char>(raw[i + 1])))
		{
			const XFoam_String mant = raw.substr(0, i);
			const XFoam_String exp = raw.substr(i);
			return std::stod(mant + "e" + exp);
		}
	}
	return std::stod(raw);
}

double XF_readNastranRealFrom8Fields(const XFoam_String& line, XFoam_Label& fieldIndex)
{
	XFoam_String acc = XF_trimCopy(XF_bulkField8(line, fieldIndex));
	++fieldIndex;
	if (acc.empty())
	{
		return 0.0;
	}
	for (;;)
	{
		char* endp = nullptr;
		const double v = std::strtod(acc.c_str(), &endp);
		if (static_cast<size_t>(endp - acc.c_str()) == acc.size())
		{
			return v;
		}
		if (static_cast<size_t>(fieldIndex) * 8u >= line.size())
		{
			return XF_parseNastranDouble(acc);
		}
		XFoam_String nxt = XF_trimCopy(XF_bulkField8(line, fieldIndex));
		++fieldIndex;
		if (nxt.empty())
		{
			return XF_parseNastranDouble(acc);
		}
		if (!nxt.empty() && nxt[0] == '.')
		{
			acc += nxt.substr(1);
		}
		else
		{
			acc += nxt;
		}
	}
}

XFoam_String XF_pad8Field(const XFoam_String& in)
{
	XFoam_String t = XF_trimCopy(in);
	if (t.size() > 8u)
	{
		t = t.substr(0, 8);
	}
	return XFoam_String(8u - t.size(), ' ') + t;
}

bool XF_hasExtensionCi(const XFoam_FileName& fn, const char* extLower)
{
	const XFoam_String s(static_cast<const XFoam_String&>(fn));
	const auto dot = s.find_last_of('.');
	if (dot == XFoam_String::npos)
	{
		return false;
	}
	XFoam_String suf = s.substr(dot);
	for (char& c : suf)
	{
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	return suf == extLower;
}

bool XF_ciStartsWith(const XFoam_String& s, const char* pfx)
{
	const XFoam_String t = XF_trimCopy(s);
	const size_t n = std::strlen(pfx);
	if (t.size() < n)
	{
		return false;
	}
	for (size_t i = 0; i < n; ++i)
	{
		const char a = static_cast<char>(std::tolower(static_cast<unsigned char>(t[i])));
		const char b = static_cast<char>(std::tolower(static_cast<unsigned char>(pfx[i])));
		if (a != b)
		{
			return false;
		}
	}
	return true;
}

struct XfKeyPendingShell
{
	int eid;
	int n1o;
	int n2o;
	int n3o;
};

} // namespace

XFoam_FEM::XFoam_FEM() = default;

XFoam_FEM::~XFoam_FEM() = default;

void XFoam_FEM::clear()
{
	nodes_.clear();
	elements_.clear();
	sets_.clear();
}

XFoam_Label XFoam_FEM::findNodeInternalByOrigin_(int originId) const
{
	for (XFoam_Label i = 0; i < nodes_.size(); ++i)
	{
		if (nodes_[i].originId == originId)
		{
			return i;
		}
	}
	return -1;
}

void XFoam_FEM::addNode(const XFoam_Vector3D& position, int originId)
{
	FemNode n;
	n.originId = originId;
	n.position = position;
	nodes_.append(n);
}

void XFoam_FEM::addLine(int* nodes, int originId)
{
	FemElement e;
	e.originId = originId;
	e.type = eFemElementTypeLine;
	for (int k = 0; k < 20; ++k)
	{
		e.nodes[k] = 0;
	}
	e.nodes[0] = nodes[0];
	e.nodes[1] = nodes[1];
	elements_.append(e);
}

void XFoam_FEM::addTriangle(int* nodes, int originId)
{
	FemElement e;
	e.originId = originId;
	e.type = eFemElementTypeTriangle;
	for (int k = 0; k < 20; ++k)
	{
		e.nodes[k] = 0;
	}
	e.nodes[0] = nodes[0];
	e.nodes[1] = nodes[1];
	e.nodes[2] = nodes[2];
	elements_.append(e);
}

void XFoam_FEM::addQuad(int* nodes, int originId)
{
    // 四个点
    FemElement e;
    e.originId = originId;
    e.type = eFemElementTypeQuad;
    for (int k = 0; k < 20; ++k)
    {
        e.nodes[k] = 0;
    }
    e.nodes[0] = nodes[0];
    e.nodes[1] = nodes[1];
    e.nodes[2] = nodes[2];
    e.nodes[3] = nodes[3];
    elements_.append(e);
}

void XFoam_FEM::addTetra(int* nodes, int originId)
{
    // 四个点
    FemElement e;
    e.originId = originId;
    e.type = eFemElementTypeTetra;
    for (int k = 0; k < 20; ++k)
    {
        e.nodes[k] = 0;
    }
    e.nodes[0] = nodes[0];
    e.nodes[1] = nodes[1];
    e.nodes[2] = nodes[2];
    e.nodes[3] = nodes[3];
    elements_.append(e);
}

void XFoam_FEM::addHex(int* nodes, int originId)
{
    // 八个点
    FemElement e;
    e.originId = originId;
    e.type = eFemElementTypeHex;
    for (int k = 0; k < 20; ++k)
    {
        e.nodes[k] = 0;
    }
    e.nodes[0] = nodes[0];
    e.nodes[1] = nodes[1];
    e.nodes[2] = nodes[2];
    e.nodes[3] = nodes[3];
    e.nodes[4] = nodes[4];
    e.nodes[5] = nodes[5];
    e.nodes[6] = nodes[6];
    e.nodes[7] = nodes[7];
    elements_.append(e);
}

void XFoam_FEM::addElementToSet(int elementId, int setId)
{
	const XFoam_Label k = static_cast<XFoam_Label>(setId);
	if (!sets_.found(k))
	{
		FemSet s;
		s.originId = setId;
		s.elements.clear();
		sets_.set(k, s);
	}
	XFoam_Map<FemSet>::iterator it = sets_.find(k);
	(*it).elements.append(elementId);
}

XFoam_Vector3D XFoam_FEM::getNodePosition(int id) const
{
	if (id < 0 || id >= nodes_.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_FEM::getNodePosition: id out of range"));
	}
	return nodes_[id].position;
}

XFoam_FEM::FemElement XFoam_FEM::getElement(int id) const
{
	if (id < 0 || id >= elements_.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_FEM::getElement: id out of range"));
	}
	return elements_[id];
}

void XFoam_FEM::readKey(const XFoam_FileName& fileName)
{
	// 移植源码: LS-DYNA keyword 子集（*ELEMENT_SHELL + *NODE，与 data/key/cylinder.key 对齐）；完整 key 未移植。
	// 命名规范: foam_code.md
	// 移植规范: foam_code.md
	clear();
	std::vector<XfKeyPendingShell> pending;
	bool inShell = false;
	bool inNode = false;
	const XFoam_String path(static_cast<const XFoam_String&>(fileName));
	std::ifstream in(path.c_str(), std::ios::in);
	if (!in)
	{
		throw XFoam_Error(XFoam_String("XFoam_FEM::readKey: cannot open file"));
	}
	XFoam_String line;
	while (std::getline(in, line))
	{
		const XFoam_String t = XF_trimCopy(line);
		if (t.empty())
		{
			continue;
		}
		if (t[0] == '$')
		{
			continue;
		}
		if (t[0] == '*')
		{
			if (XF_ciStartsWith(t, "*END"))
			{
				break;
			}
			inShell = XF_ciStartsWith(t, "*ELEMENT_SHELL");
			inNode = XF_ciStartsWith(t, "*NODE");
			continue;
		}
		if (inShell)
		{
			std::istringstream iss(t);
			long long eid = 0;
			long long pid = 0;
			long long n1 = 0;
			long long n2 = 0;
			long long n3 = 0;
			long long n4 = 0;
			long long n5 = 0;
			long long n6 = 0;
			long long n7 = 0;
			long long n8 = 0;
			iss >> eid >> pid >> n1 >> n2 >> n3 >> n4 >> n5 >> n6 >> n7 >> n8;
			if (!iss)
			{
				continue;
			}
			(void)n4;
			(void)n5;
			(void)n6;
			(void)n7;
			(void)n8;
			XfKeyPendingShell p;
			p.eid = static_cast<int>(eid);
			p.n1o = static_cast<int>(n1);
			p.n2o = static_cast<int>(n2);
			p.n3o = static_cast<int>(n3);
			pending.push_back(p);
		}
		else if (inNode)
		{
			std::istringstream iss(t);
			long long nid = 0;
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			long long tc = 0;
			long long rc = 0;
			iss >> nid >> x >> y >> z >> tc >> rc;
			if (!iss)
			{
				continue;
			}
			(void)tc;
			(void)rc;
			addNode(XFoam_Vector3D(x, y, z), static_cast<int>(nid));
		}
	}
	for (const XfKeyPendingShell& p : pending)
	{
		const XFoam_Label i1 = findNodeInternalByOrigin_(p.n1o);
		const XFoam_Label i2 = findNodeInternalByOrigin_(p.n2o);
		const XFoam_Label i3 = findNodeInternalByOrigin_(p.n3o);
		if (i1 < 0 || i2 < 0 || i3 < 0)
		{
			throw XFoam_Error(XFoam_String("XFoam_FEM::readKey: shell element references unknown node id"));
		}
		int nv[3] = {static_cast<int>(i1), static_cast<int>(i2), static_cast<int>(i3)};
		addTriangle(nv, p.eid);
	}
}

void XFoam_FEM::writeKey(const XFoam_FileName& fileName)
{
	// 命名规范: foam_code.md
	// 移植规范: foam_code.md
	const XFoam_String path(static_cast<const XFoam_String&>(fileName));
	std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
	if (!out)
	{
		throw XFoam_Error(XFoam_String("XFoam_FEM::writeKey: cannot open file for write"));
	}
	out << "$# XFoam FEM export (LS-DYNA keyword subset)\n";
	out << "*KEYWORD\n";
	out << "*TITLE\n";
	out << "$#                                                                         title\n";
	out << "XFoam export\n";
	out << "*ELEMENT_SHELL\n";
	out << "$#   eid     pid      n1      n2      n3      n4      n5      n6      n7      n8\n";
	for (XFoam_Label ei = 0; ei < elements_.size(); ++ei)
	{
		const FemElement& e = elements_[ei];
		if (e.type != eFemElementTypeTriangle)
		{
			continue;
		}
		const int a = e.nodes[0];
		const int b = e.nodes[1];
		const int c = e.nodes[2];
		const int oa = nodes_[a].originId;
		const int ob = nodes_[b].originId;
		const int oc = nodes_[c].originId;
		out << std::setw(10) << e.originId << std::setw(10) << 1 << std::setw(10) << oa << std::setw(10) << ob
			<< std::setw(10) << oc << std::setw(10) << oc << std::setw(10) << 0 << std::setw(10) << 0 << std::setw(10) << 0
			<< std::setw(10) << 0 << '\n';
	}
	out << "*NODE\n";
	out << "$#   nid               x               y               z      tc      rc\n";
	out << std::fixed;
	for (XFoam_Label ni = 0; ni < nodes_.size(); ++ni)
	{
		const FemNode& n = nodes_[ni];
		out << std::setw(10) << n.originId << std::setw(20) << std::setprecision(15) << n.position.x()
			<< std::setw(20) << std::setprecision(15) << n.position.y() << std::setw(20) << std::setprecision(15)
			<< n.position.z() << std::setw(8) << 0 << std::setw(8) << 0 << '\n';
	}
	out << "*END\n";
}

void XFoam_FEM::read(const XFoam_FileName& fileName)
{
	if (XF_hasExtensionCi(fileName, ".bdf"))
	{
		readBdf(fileName);
		return;
	}
	if (XF_hasExtensionCi(fileName, ".key"))
	{
		readKey(fileName);
		return;
	}
	throw XFoam_Error(XFoam_String("XFoam_FEM::read: unsupported file extension"));
}

void XFoam_FEM::write(const XFoam_FileName& fileName)
{
	if (XF_hasExtensionCi(fileName, ".bdf"))
	{
		writeBdf(fileName);
		return;
	}
	if (XF_hasExtensionCi(fileName, ".key"))
	{
		writeKey(fileName);
		return;
	}
	throw XFoam_Error(XFoam_String("XFoam_FEM::write: unsupported file extension"));
}

void XFoam_FEM::readBdf(const XFoam_FileName& fileName)
{
	// 移植源码: NASTRAN NX/HyperMesh bulk GRID + CTRIA3 小字段（8 字符）子集；完整 BDF 未移植。
	// 命名规范: foam_code.md
	// 移植规范: foam_code.md
	clear();
	const XFoam_String path(static_cast<const XFoam_String&>(fileName));
	std::ifstream in(path.c_str(), std::ios::in);
	if (!in)
	{
		throw XFoam_Error(XFoam_String("XFoam_FEM::readBdf: cannot open file"));
	}
	XFoam_String line;
	while (std::getline(in, line))
	{
		if (line.empty())
		{
			continue;
		}
		const XFoam_String t = XF_trimCopy(line);
		if (t.empty() || t[0] == '$')
		{
			continue;
		}
		const XFoam_String pad = line.size() % 8u == 0 ? line : (line + XFoam_String(8u - line.size() % 8u, ' '));
		const XFoam_String key = XF_trimCopy(XF_bulkField8(pad, 0));
		if (key.rfind("GRID", 0) == 0)
		{
			const int nid = static_cast<int>(std::stoi(XF_trimCopy(XF_bulkField8(pad, 1))));
			const XFoam_String cp = XF_trimCopy(XF_bulkField8(pad, 2));
			if (!cp.empty())
			{
				(void)std::stoi(cp);
			}
			XFoam_Label fi = 3;
			const double x = XF_readNastranRealFrom8Fields(pad, fi);
			const double y = XF_readNastranRealFrom8Fields(pad, fi);
			const double z = XF_readNastranRealFrom8Fields(pad, fi);
			addNode(XFoam_Vector3D(x, y, z), nid);
		}
		else if (key.rfind("CTRIA3", 0) == 0)
		{
			XFoam_Label fi = 1;
			const int eid = static_cast<int>(std::stoi(XF_trimCopy(XF_bulkField8(pad, fi++))));
			(void)std::stoi(XF_trimCopy(XF_bulkField8(pad, fi++))); // property
			const int n1o = static_cast<int>(std::stoi(XF_trimCopy(XF_bulkField8(pad, fi++))));
			const int n2o = static_cast<int>(std::stoi(XF_trimCopy(XF_bulkField8(pad, fi++))));
			const int n3o = static_cast<int>(std::stoi(XF_trimCopy(XF_bulkField8(pad, fi++))));
			const XFoam_Label i1 = findNodeInternalByOrigin_(n1o);
			const XFoam_Label i2 = findNodeInternalByOrigin_(n2o);
			const XFoam_Label i3 = findNodeInternalByOrigin_(n3o);
			if (i1 < 0 || i2 < 0 || i3 < 0)
			{
				throw XFoam_Error(XFoam_String("XFoam_FEM::readBdf: CTRIA3 references unknown GRID id"));
			}
			int nv[3] = {static_cast<int>(i1), static_cast<int>(i2), static_cast<int>(i3)};
			addTriangle(nv, eid);
		}
	}
}

void XFoam_FEM::writeBdf(const XFoam_FileName& fileName)
{
	// 命名规范: foam_code.md
	// 移植规范: foam_code.md
	const XFoam_String path(static_cast<const XFoam_String&>(fileName));
	std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
	if (!out)
	{
		throw XFoam_Error(XFoam_String("XFoam_FEM::writeBdf: cannot open file for write"));
	}
	out << "CEND\n";
	out << "BEGIN BULK\n";
	for (XFoam_Label ni = 0; ni < nodes_.size(); ++ni)
	{
		const FemNode& n = nodes_[ni];
		std::ostringstream os;
		os << n.position.x();
		const XFoam_String sx = XF_pad8Field(os.str());
		os.str("");
		os << n.position.y();
		const XFoam_String sy = XF_pad8Field(os.str());
		os.str("");
		os << n.position.z();
		const XFoam_String sz = XF_pad8Field(os.str());
		out << XF_pad8Field("GRID")
			<< XF_pad8Field(std::to_string(n.originId))
			<< XF_pad8Field("") << sx << sy << sz << "\n";
	}
	for (XFoam_Label ei = 0; ei < elements_.size(); ++ei)
	{
		const FemElement& e = elements_[ei];
		if (e.type == eFemElementTypeTriangle)
		{
			const int a = e.nodes[0];
			const int b = e.nodes[1];
			const int c = e.nodes[2];
			out << XF_pad8Field("CTRIA3") << XF_pad8Field(std::to_string(e.originId)) << XF_pad8Field("0")
				<< XF_pad8Field(std::to_string(nodes_[a].originId))
				<< XF_pad8Field(std::to_string(nodes_[b].originId))
				<< XF_pad8Field(std::to_string(nodes_[c].originId)) << "\n";
		}
		else if (e.type == eFemElementTypeQuad)
		{
			const int a = e.nodes[0];
			const int b = e.nodes[1];
			const int c = e.nodes[2];
			const int d = e.nodes[3];
			out << XF_pad8Field("CQUAD4") << XF_pad8Field(std::to_string(e.originId)) << XF_pad8Field("0")
				<< XF_pad8Field(std::to_string(nodes_[a].originId))
				<< XF_pad8Field(std::to_string(nodes_[b].originId))
				<< XF_pad8Field(std::to_string(nodes_[c].originId))
				<< XF_pad8Field(std::to_string(nodes_[d].originId)) << "\n";
		}
		else if (e.type == eFemElementTypeLine)
		{
			const int a = e.nodes[0];
			const int b = e.nodes[1];
			out << XF_pad8Field("CROD") << XF_pad8Field(std::to_string(e.originId)) << XF_pad8Field("0")
				<< XF_pad8Field(std::to_string(nodes_[a].originId))
				<< XF_pad8Field(std::to_string(nodes_[b].originId)) << "\n";
		}
	}
	out << "ENDDATA\n";
}
