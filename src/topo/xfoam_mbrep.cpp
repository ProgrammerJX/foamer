#include "XFoam/topo/xfoam_mbrep.h"
#include <cstring>

namespace
{

void trimString(XFoam_String& s);

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

// 读取 $HMMOVE 块；参考 XData bdf_readcomps.cpp BDF_readHMMove。
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

void buildFacePatchesFromHmMoves(
	const std::vector<HmMoveBlock>& moves,
	const std::unordered_map<int, XFoam_Label>& elemIdToIx,
	XFoam_List<XFoam_LabelList>& facesOut)
{
	for (const HmMoveBlock& mv : moves)
	{
		if (mv.compId < 0 || mv.elemIds.empty())
		{
			continue;
		}
		bool allShell = true;
		for (int eid : mv.elemIds)
		{
			if (elemIdToIx.find(eid) == elemIdToIx.end())
			{
				allShell = false;
				break;
			}
		}
		if (!allShell)
		{
			continue;
		}
		XFoam_LabelList patch;
		for (int eid : mv.elemIds)
		{
			patch.append(elemIdToIx.find(eid)->second);
		}
		facesOut.append(patch);
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

} // namespace

XFoam_MBrep::XFoam_MBrep() = default;

XFoam_MBrep::~XFoam_MBrep() = default;

void XFoam_MBrep::clear()
{
	XFoam_VBrep::clear();
	positions_.clear();
	elems_.clear();
	faces_.clear();
	segments_.clear();
	edges_.clear();
}

XFoam_BoundBox XFoam_MBrep::bounds() const
{
	return XFoam_BoundBox(positions_);
}

void XFoam_MBrep::readFromBdf(const XFoam_String& fileName)
{
	clear();
	std::ifstream ifs(fileName.c_str());
	if (!ifs)
	{
		throw XFoam_Error(
			XFoam_String("XFoam_MBrep::readFromBdf: cannot open ") + fileName);
	}

	std::unordered_map<int, XFoam_Label> idToIdx;
	idToIdx.reserve(4096);
	std::unordered_map<int, XFoam_Label> elemIdToIx;
	elemIdToIx.reserve(4096);
	std::vector<HmMoveBlock> hmMoves;
	hmMoves.reserve(16);

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
			if (line.size() < 48u)
			{
				continue;
			}
			XFoam_String s1 = field8(line, 1);
			XFoam_String s3 = field8(line, 3);
			XFoam_String s4 = field8(line, 4);
			XFoam_String s5 = field8(line, 5);
			trimString(s1);
			trimString(s3);
			trimString(s4);
			trimString(s5);
			const int elemId = std::atoi(s1.c_str());
			const int g1 = std::atoi(s3.c_str());
			const int g2 = std::atoi(s4.c_str());
			const int g3 = std::atoi(s5.c_str());
			const XFoam_Label i0 = nodeIndex(idToIdx, g1, "CTRIA3");
			const XFoam_Label i1 = nodeIndex(idToIdx, g2, "CTRIA3");
			const XFoam_Label i2 = nodeIndex(idToIdx, g3, "CTRIA3");
			XFoam_FixedList<XFoam_Label, 4> el;
			el[0] = i0;
			el[1] = i1;
			el[2] = i2;
			el[3] = i2;
			elems_.append(el);
			elemIdToIx[elemId] = elems_.size() - 1;
		}
		else if (lineStartsWith(line, "CQUAD4"))
		{
			if (line.size() < 56u)
			{
				continue;
			}
			XFoam_String s1 = field8(line, 1);
			XFoam_String s3 = field8(line, 3);
			XFoam_String s4 = field8(line, 4);
			XFoam_String s5 = field8(line, 5);
			XFoam_String s6 = field8(line, 6);
			trimString(s1);
			trimString(s3);
			trimString(s4);
			trimString(s5);
			trimString(s6);
			const int elemId = std::atoi(s1.c_str());
			const int g1 = std::atoi(s3.c_str());
			const int g2 = std::atoi(s4.c_str());
			const int g3 = std::atoi(s5.c_str());
			const int g4 = std::atoi(s6.c_str());
			const XFoam_Label i0 = nodeIndex(idToIdx, g1, "CQUAD4");
			const XFoam_Label i1 = nodeIndex(idToIdx, g2, "CQUAD4");
			const XFoam_Label i2 = nodeIndex(idToIdx, g3, "CQUAD4");
			const XFoam_Label i3 = nodeIndex(idToIdx, g4, "CQUAD4");
			XFoam_FixedList<XFoam_Label, 4> el;
			el[0] = i0;
			el[1] = i1;
			el[2] = i2;
			el[3] = i3;
			elems_.append(el);
			elemIdToIx[elemId] = elems_.size() - 1;
		}
	}

	buildFacePatchesFromHmMoves(hmMoves, elemIdToIx, faces_);
}
