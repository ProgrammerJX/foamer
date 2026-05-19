#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_error.h"
#include "XFoam/utilities/xfoam_stream.h"

#include <boost/filesystem.hpp>

const XFoam_Word XFoam_Word::null{};

// * * * * * * * * * * * * * * XFoam_Variable * * * * * * * * * * * * * * //

const char* const XFoam_Variable::typeName = "variable";
int XFoam_Variable::debug = 0;
const XFoam_Variable XFoam_Variable::null;

void XFoam_Variable::stripInvalid()
{
	if (debug && false)
	{
		// XFoam 未实现：Foam::string::stripInvalid<variable>
	}
}

XFoam_Variable::XFoam_Variable() = default;

XFoam_Variable::XFoam_Variable(const XFoam_Variable& v) = default;

XFoam_Variable::XFoam_Variable(const XFoam_Word& w)
	: XFoam_Word(w)
{}

XFoam_Variable::XFoam_Variable(const std::string& s, const bool doStripInvalid)
	: XFoam_Word(s)
{
	if (doStripInvalid)
	{
		stripInvalid();
	}
}

XFoam_Variable::XFoam_Variable(const char* s, const bool doStripInvalid)
	: XFoam_Word(s ? s : "")
{
	if (doStripInvalid)
	{
		stripInvalid();
	}
}

XFoam_Variable::XFoam_Variable(std::istream& is)
	: XFoam_Word()
{
	is >> *this;
}

bool XFoam_Variable::valid(char c)
{
	return !std::isspace(static_cast<unsigned char>(c)) && c != '"' && c != '\'' && c != ';' && c != '{' && c != '}';
}

void XFoam_Variable::operator=(const XFoam_Variable& s)
{
	static_cast<std::string&>(*this) = static_cast<const std::string&>(s);
}

void XFoam_Variable::operator=(const XFoam_Word& s)
{
	static_cast<std::string&>(*this) = static_cast<const std::string&>(s);
}

void XFoam_Variable::operator=(const std::string& s)
{
	static_cast<std::string&>(*this) = s;
	stripInvalid();
}

void XFoam_Variable::operator=(const char* s)
{
	static_cast<std::string&>(*this) = s ? s : "";
	stripInvalid();
}

std::istream& operator>>(std::istream& is, XFoam_Variable& v)
{
	XFoam_String s;
	is >> s;
	v = XFoam_Variable(s);
	return is;
}

std::ostream& operator<<(std::ostream& os, const XFoam_Variable& v)
{
	return os << static_cast<const std::string&>(v);
}

void XFoam_writeEntry(std::ostream& os, const XFoam_Variable& value)
{
	os << value;
}

// * * * * * * * * * * * * * * XFoam_FunctionName * * * * * * * * * * * * * //

const char* const XFoam_FunctionName::typeName = "functionName";
int XFoam_FunctionName::debug = 0;
const XFoam_FunctionName XFoam_FunctionName::null;

XFoam_FunctionName::XFoam_FunctionName() = default;

XFoam_FunctionName::XFoam_FunctionName(const XFoam_FunctionName&) = default;

XFoam_FunctionName::XFoam_FunctionName(const XFoam_Word& w)
	: XFoam_Word(w)
{}

XFoam_FunctionName::XFoam_FunctionName(const std::string& s, const bool doStripInvalid)
	: XFoam_Word(s)
{
	(void)doStripInvalid;
}

XFoam_FunctionName::XFoam_FunctionName(const char* s, const bool doStripInvalid)
	: XFoam_Word(s ? s : "")
{
	(void)doStripInvalid;
}

XFoam_FunctionName::XFoam_FunctionName(std::istream& is)
	: XFoam_Word()
{
	is >> *this;
}

bool XFoam_FunctionName::valid(char c)
{
	return XFoam_Variable::valid(c) || c == '#';
}

void XFoam_FunctionName::operator=(const XFoam_FunctionName& s)
{
	XFoam_Word::operator=(s);
}

void XFoam_FunctionName::operator=(const XFoam_Word& s)
{
	XFoam_Word::operator=(s);
}

void XFoam_FunctionName::operator=(const std::string& s)
{
	XFoam_Word::operator=(s);
}

void XFoam_FunctionName::operator=(const char* s)
{
	static_cast<std::string&>(*this) = s ? s : "";
}

std::istream& operator>>(std::istream& is, XFoam_FunctionName& v)
{
	XFoam_String s;
	is >> s;
	v = XFoam_FunctionName(s);
	return is;
}

std::ostream& operator<<(std::ostream& os, const XFoam_FunctionName& v)
{
	return os << static_cast<const std::string&>(v);
}

void XFoam_writeEntry(std::ostream& os, const XFoam_FunctionName& value)
{
	os << value;
}

// * * * * * * * * * * * * * * XFoam_Switch * * * * * * * * * * * * * * //

const char* XFoam_Switch::names[nSwitchType] = {
	"false",
	"true",
	"off",
	"on",
	"no",
	"yes",
	"n",
	"y",
	"f",
	"t",
	"none",
	"any",
	"invalid"};

XFoam_Switch::SwitchType XFoam_Switch::asEnum(const std::string& str, const bool allowInvalid)
{
	for (SwitchType sw = SwitchType::False; sw < SwitchType::invalid; ++sw)
	{
		if (str == names[toInt(sw)])
		{
			switch (sw)
			{
			case SwitchType::n:
			case SwitchType::none:
				return SwitchType::no;
			case SwitchType::y:
			case SwitchType::any:
				return SwitchType::yes;
			case SwitchType::f:
				return SwitchType::False;
			case SwitchType::t:
				return SwitchType::True;
			default:
				return sw;
			}
		}
	}

	if (!allowInvalid)
	{
		XFoam_FatalErrorInFunction << "unknown switch word " << str.c_str() << '\n' << XFoam_abort(XFoam_FatalError);
	}

	return SwitchType::invalid;
}

bool XFoam_Switch::valid() const
{
	return switch_ <= SwitchType::none;
}

const char* XFoam_Switch::asText() const
{
	return names[toInt(switch_)];
}

bool XFoam_Switch::readIfPresent(const XFoam_Word& name, const XFoam_Dictionary& dict)
{
	if (!dict.found(name))
	{
		return false;
	}
	XFoam_ITstream& is = dict.lookup(name);
	static_cast<XFoam_IStream&>(is) >> *this;
	return true;
}

XFoam_Switch::XFoam_Switch(XFoam_IStream& is)
	: switch_(SwitchType::False)
{
	is >> *this;
}

XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Switch& s)
{
	XFoam_Token t(is);

	if (!t.good())
	{
		is.setBad();
		return is;
	}

	if (t.isLabel())
	{
		s = static_cast<bool>(t.labelToken());
	}
	else if (t.isWord())
	{
		XFoam_Switch sw(XFoam_String(t.wordToken()), true);

		if (sw.valid())
		{
			s = sw;
		}
		else
		{
			is.setBad();
			XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(XFoam_String(static_cast<const std::string&>(is.name()))))
				<< "expected 'true/false', 'on/off' ... found " << t.wordToken() << XFoam_exit(XFoam_FatalIOError, 1);
			return is;
		}
	}
	else
	{
		is.setBad();
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(XFoam_String(static_cast<const std::string&>(is.name()))))
			<< "wrong token type - expected bool, found " << t << XFoam_exit(XFoam_FatalIOError, 1);
		return is;
	}

	(void)is.check("XFoam_IStream& operator>>(XFoam_IStream&, XFoam_Switch&)");
	return is;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Switch& s)
{
	os << s.asText();
	(void)os.check("XFoam_OStream& operator<<(XFoam_OStream&, const XFoam_Switch&)");
	return os;
}

// * * * * * * * * * * * * * * XFoam_FileName * * * * * * * * * * * * * * //

const char* const XFoam_FileName::typeName = "fileName";
int XFoam_FileName::debug = 0;
const XFoam_FileName XFoam_FileName::null{};

XFoam_FileName::XFoam_FileName(const XFoam_List<XFoam_Word>& lst)
	: XFoam_String()
{
	for (XFoam_Label i = 0; i < lst.size(); ++i)
	{
		if (i)
		{
			push_back('/');
		}
		append(static_cast<const XFoam_String&>(lst[i]));
	}
	stripInvalid();
}

XFoam_FileName::XFoam_FileName(std::istream& is)
	: XFoam_String()
{
	XFoam_String s;
	is >> s;
	*this = XFoam_FileName(s);
}

bool XFoam_FileName::clean()
{
	const XFoam_String before = static_cast<const XFoam_String&>(*this);
	XFoam_String s = before;
	for (;;)
	{
		const XFoam_String::size_type p = s.find("//");
		if (p == XFoam_String::npos)
		{
			break;
		}
		s.erase(p, 1);
	}
	while (s.size() > 1 && s.back() == '/')
	{
		s.pop_back();
	}
	if (s != before)
	{
		static_cast<XFoam_String&>(*this) = s;
		return true;
	}
	return false;
}

XFoam_FileName XFoam_FileName::clean() const
{
	XFoam_FileName c(*this);
	(void)c.clean();
	return c;
}

XFoam_FileType XFoam_FileName::type(const bool /*checkVariants*/, const bool followLink) const
{
	if (empty())
	{
		return XFoam_FileType::undefined;
	}
	namespace fs = boost::filesystem;
	const fs::path p(static_cast<const std::string&>(*this));
	boost::system::error_code ec;
	const fs::file_status st = followLink ? fs::status(p, ec) : fs::symlink_status(p, ec);
	if (ec)
	{
		return XFoam_FileType::undefined;
	}
	if (fs::is_directory(st))
	{
		return XFoam_FileType::directory;
	}
	if (fs::is_regular_file(st))
	{
		return XFoam_FileType::file;
	}
	if (fs::is_symlink(st))
	{
		return XFoam_FileType::link;
	}
	return XFoam_FileType::undefined;
}

XFoam_FileName& XFoam_FileName::toAbsolute()
{
	if (empty())
	{
		return *this;
	}
	namespace fs = boost::filesystem;
	boost::system::error_code ec;
	const fs::path abs = fs::absolute(fs::path(static_cast<const std::string&>(*this)), ec);
	if (!ec)
	{
		static_cast<XFoam_String&>(*this) = abs.generic_string();
		stripInvalid();
	}
	return *this;
}

XFoam_String XFoam_FileName::caseName() const
{
	return XFoam_String(static_cast<const XFoam_String&>(name()));
}

XFoam_List<XFoam_Word> XFoam_FileName::components(const char delimiter) const
{
	XFoam_List<XFoam_Word> out;
	if (empty())
	{
		return out;
	}
	XFoam_String::size_type beg = 0;
	while (beg < size())
	{
		XFoam_String::size_type end = find(delimiter, beg);
		if (end == npos)
		{
			out.append(XFoam_Word(substr(beg)));
			break;
		}
		if (end > beg)
		{
			out.append(XFoam_Word(substr(beg, end - beg)));
		}
		beg = end + 1;
	}
	return out;
}

XFoam_Word XFoam_FileName::component(const size_type cmpt, const char delimiter) const
{
	const XFoam_List<XFoam_Word> cmpts = components(delimiter);
	if (cmpt >= static_cast<size_type>(cmpts.size()))
	{
		return XFoam_Word::null;
	}
	return cmpts[static_cast<XFoam_Label>(cmpt)];
}

void XFoam_FileName::operator=(const XFoam_FileName& str)
{
	static_cast<XFoam_String&>(*this) = static_cast<const XFoam_String&>(str);
}

void XFoam_FileName::operator=(XFoam_FileName&& str)
{
	static_cast<XFoam_String&>(*this) = std::move(static_cast<XFoam_String&&>(str));
}

void XFoam_FileName::operator=(const XFoam_String& str)
{
	static_cast<XFoam_String&>(*this) = str;
	stripInvalid();
}

void XFoam_FileName::operator=(const char* str)
{
	static_cast<XFoam_String&>(*this) = str ? str : "";
	stripInvalid();
}

void XFoam_FileName::operator/=(const XFoam_String& tail)
{
	if (tail.empty())
	{
		return;
	}
	if (!empty() && back() != '/' && tail.front() != '/')
	{
		push_back('/');
	}
	append(tail);
	stripInvalid();
}

std::istream& operator>>(std::istream& is, XFoam_FileName& fn)
{
	XFoam_String s;
	is >> s;
	fn = XFoam_FileName(s);
	return is;
}

std::ostream& operator<<(std::ostream& os, const XFoam_FileName& fn)
{
	return os << static_cast<const std::string&>(fn);
}

XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_FileName& fn)
{
	XFoam_Token t(is);
	if (!t.good() || t.error())
	{
		is.setBad();
		return is;
	}
	if (t.isWord())
	{
		fn = static_cast<const XFoam_String&>(t.wordToken());
	}
	else if (t.isString())
	{
		fn = t.stringToken();
	}
	else
	{
		is.setBad();
	}
	return is;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_FileName& fn)
{
	os << static_cast<const XFoam_String&>(fn);
	(void)os.check("XFoam_OStream& operator<<(XFoam_OStream&, const XFoam_FileName&)");
	return os;
}
