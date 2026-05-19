#include "XFoam/utilities/xfoam_regexp.h"
#include "XFoam/utilities/xfoam_dictionary.h"
#include "XFoam/utilities/xfoam_stream.h"
#include "XFoam/utilities/xfoam_error.h"
#include <cctype>
#include <cstring>
#if XFOAM_REGEXP_IMPL_STD
#include <sstream>
#endif

// * * * * * * * * * * * * * Private Member Functions  * * * * * * * * * * * //

template<class StringType>
bool XFoam_RegExp::matchGrouping(const std::string& str, XFoam_List<StringType>& groups) const
{
	if (preg_ && str.size())
	{
#if XFOAM_REGEXP_IMPL_STD
		std::smatch m;
		if (std::regex_match(str, m, *preg_))
		{
			groups.setSize(ngroups());
			for (int groupI = 0; groupI < ngroups(); ++groupI)
			{
				const std::ssub_match& sm = m[static_cast<std::size_t>(groupI + 1)];
				if (sm.matched)
				{
					groups[groupI] = StringType(sm.str());
				}
				else
				{
					groups[groupI].clear();
				}
			}
			return true;
		}
#else
		const std::size_t nmatch = static_cast<std::size_t>(ngroups() + 1);
		XFoam_List<regmatch_t> pmatch(static_cast<XFoam_Label>(nmatch));

		if (
			regexec(preg_, str.c_str(), nmatch, &pmatch[0], 0) == 0
			&& (pmatch[0].rm_so == 0 && pmatch[0].rm_eo == static_cast<regoff_t>(str.size())))
		{
			groups.setSize(ngroups());
			XFoam_Label groupI = 0;
			for (std::size_t matchI = 1; matchI < nmatch; matchI++)
			{
				if (pmatch[static_cast<XFoam_Label>(matchI)].rm_so != -1 && pmatch[static_cast<XFoam_Label>(matchI)].rm_eo != -1)
				{
					groups[groupI] = StringType(str.substr(
						static_cast<std::size_t>(pmatch[static_cast<XFoam_Label>(matchI)].rm_so),
						static_cast<std::size_t>(
							pmatch[static_cast<XFoam_Label>(matchI)].rm_eo - pmatch[static_cast<XFoam_Label>(matchI)].rm_so)));
				}
				else
				{
					groups[groupI].clear();
				}
				groupI++;
			}
			return true;
		}
#endif
	}
	groups.clear();
	return false;
}

template bool XFoam_RegExp::matchGrouping<std::string>(const std::string&, XFoam_List<std::string>&) const;

// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

XFoam_RegExp::XFoam_RegExp()
	: preg_(nullptr)
{}

XFoam_RegExp::XFoam_RegExp(const char* pattern, const bool ignoreCase)
	: preg_(nullptr)
{
	set(pattern, ignoreCase);
}

XFoam_RegExp::XFoam_RegExp(const std::string& pattern, const bool ignoreCase)
	: preg_(nullptr)
{
	set(pattern.c_str(), ignoreCase);
}

XFoam_RegExp::~XFoam_RegExp()
{
	clear();
}

// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void XFoam_RegExp::set(const char* pattern, const bool ignoreCase) const
{
	clear();

	if (pattern && *pattern)
	{
#if XFOAM_REGEXP_IMPL_STD
		bool ic = ignoreCase;
		const char* pat = pattern;
		if (!strncmp(pattern, "(?i)", 4))
		{
			ic = true;
			pat += 4;
			if (!*pat)
			{
				return;
			}
		}
		try
		{
			auto flags = std::regex::ECMAScript;
			if (ic)
			{
				flags |= std::regex_constants::icase;
			}
			preg_ = new std::regex(std::string(pat), flags);
		}
		catch (const std::regex_error& e)
		{
			std::ostringstream oss;
			oss << e.what();
			XFoam_FatalErrorInFunction << "Failed to compile regular expression '" << pattern << "'" << '\n' << oss.str()
									   << XFoam_exit(XFoam_FatalError, 1);
		}
#else
		int cflags = REG_EXTENDED;
		if (ignoreCase)
		{
			cflags |= REG_ICASE;
		}

		const char* pat = pattern;

		if (!strncmp(pattern, "(?i)", 4))
		{
			cflags |= REG_ICASE;
			pat += 4;

			if (!*pat)
			{
				return;
			}
		}

		preg_ = new regex_t;
		const int err = regcomp(preg_, pat, cflags);

		if (err != 0)
		{
			char errbuf[200];
			regerror(err, preg_, errbuf, sizeof(errbuf));

			XFoam_FatalErrorInFunction << "Failed to compile regular expression '" << pattern << "'" << '\n' << errbuf
									   << XFoam_exit(XFoam_FatalError, 1);
		}
#endif
	}
}

void XFoam_RegExp::set(const std::string& pattern, const bool ignoreCase) const
{
	set(pattern.c_str(), ignoreCase);
}

bool XFoam_RegExp::clear() const
{
	if (preg_)
	{
#if XFOAM_REGEXP_IMPL_STD
		delete preg_;
#else
		regfree(preg_);
		delete preg_;
#endif
		preg_ = nullptr;
		return true;
	}
	return false;
}

std::string::size_type XFoam_RegExp::find(const std::string& str) const
{
	if (preg_ && str.size())
	{
#if XFOAM_REGEXP_IMPL_STD
		std::smatch m;
		if (std::regex_search(str, m, *preg_))
		{
			return static_cast<std::string::size_type>(m.position(0));
		}
#else
		const std::size_t nmatch = 1;
		regmatch_t pmatch[1];

		if (regexec(preg_, str.c_str(), nmatch, pmatch, 0) == 0)
		{
			return static_cast<std::string::size_type>(pmatch[0].rm_so);
		}
#endif
	}
	return std::string::npos;
}

bool XFoam_RegExp::match(const std::string& str) const
{
	if (preg_ && str.size())
	{
#if XFOAM_REGEXP_IMPL_STD
		return std::regex_match(str, *preg_);
#else
		const std::size_t nmatch = 1;
		regmatch_t pmatch[1];

		if (
			regexec(preg_, str.c_str(), nmatch, pmatch, 0) == 0
			&& (pmatch[0].rm_so == 0 && pmatch[0].rm_eo == static_cast<regoff_t>(str.size())))
		{
			return true;
		}
#endif
	}
	return false;
}

bool XFoam_RegExp::match(const std::string& str, XFoam_List<std::string>& groups) const
{
	return matchGrouping(str, groups);
}

void XFoam_RegExp::operator=(const char* pat)
{
	set(pat);
}

void XFoam_RegExp::operator=(const std::string& pat)
{
	set(pat);
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

const XFoam_WordRe XFoam_WordRe::null{};

bool XFoam_WordRe::isPattern(const XFoam_String& str)
{
	const std::string& s = static_cast<const std::string&>(str);
	for (std::size_t i = 0; i < s.size(); ++i)
	{
		if (XFoam_RegExp::meta(s[i]))
		{
			return true;
		}
	}
	for (std::size_t i = 0; i < s.size(); ++i)
	{
		const unsigned char c = static_cast<unsigned char>(s[i]);
		if (!(std::isalnum(c) || s[i] == '_' || s[i] == ':' || s[i] == '.' || s[i] == '-' || s[i] == '+'))
		{
			return true;
		}
	}
	return false;
}

XFoam_WordRe::XFoam_WordRe()
	: XFoam_Word()
	, re_()
{
}

XFoam_WordRe::XFoam_WordRe(const XFoam_WordRe& str)
	: XFoam_Word(static_cast<const XFoam_Word&>(str))
	, re_()
{
	if (str.isPattern())
	{
		(void)compile();
	}
}

XFoam_WordRe::XFoam_WordRe(const XFoam_KeyType& str)
	: XFoam_Word(static_cast<const XFoam_Word&>(str))
	, re_()
{
	if (str.isPattern())
	{
		(void)compile();
	}
}

XFoam_WordRe::XFoam_WordRe(const XFoam_KeyType& str, const compOption opt)
	: XFoam_Word(static_cast<const XFoam_Word&>(str))
	, re_()
{
	if (str.isPattern())
	{
		(void)compile(opt);
	}
}

XFoam_WordRe::XFoam_WordRe(const XFoam_Word& str)
	: XFoam_Word(str)
	, re_()
{
}

XFoam_WordRe::XFoam_WordRe(const char* str, const compOption opt)
	: XFoam_Word(str ? str : "")
	, re_()
{
	(void)compile(opt);
}

XFoam_WordRe::XFoam_WordRe(const XFoam_String& str, const compOption opt)
	: XFoam_Word(static_cast<const std::string&>(str))
	, re_()
{
	(void)compile(opt);
}

XFoam_WordRe::XFoam_WordRe(XFoam_IStream& is)
	: XFoam_Word()
	, re_()
{
	is >> *this;
}

bool XFoam_WordRe::isPattern() const
{
	return re_.exists();
}

bool XFoam_WordRe::compile() const
{
	re_ = static_cast<const std::string&>(*this);
	return re_.exists();
}

bool XFoam_WordRe::compile(const compOption opt) const
{
	bool doCompile = false;
	if (opt & compOption::regExp)
	{
		doCompile = true;
	}
	else if (opt & compOption::detect)
	{
		if (isPattern(static_cast<const XFoam_String&>(*this)))
		{
			doCompile = true;
		}
	}
	else if (opt & compOption::noCase)
	{
		doCompile = true;
	}
	if (doCompile)
	{
		const bool ign = (static_cast<int>(opt) & static_cast<int>(compOption::noCase)) != 0;
		re_.set(static_cast<const std::string&>(*this), ign);
	}
	else
	{
		(void)re_.clear();
	}
	return re_.exists();
}

bool XFoam_WordRe::recompile() const
{
	if (re_.exists())
	{
		re_ = static_cast<const std::string&>(*this);
	}
	return re_.exists();
}

void XFoam_WordRe::uncompile(const bool doStripInvalid) const
{
	if (re_.clear())
	{
		(void)doStripInvalid;
		// 未移植：Foam::string::stripInvalid<word> 与 word::debug 联动
	}
}

void XFoam_WordRe::clear()
{
	XFoam_Word::clear();
	(void)re_.clear();
}

void XFoam_WordRe::set(const std::string& str, const compOption opt)
{
	XFoam_String::operator=(str);
	(void)compile(opt);
}

void XFoam_WordRe::set(const char* str, const compOption opt)
{
	XFoam_String::operator=(str ? str : "");
	(void)compile(opt);
}

bool XFoam_WordRe::match(const std::string& str, const bool literalMatch) const
{
	if (literalMatch || !re_.exists())
	{
		return str == static_cast<const std::string&>(*this);
	}
	return re_.match(str);
}

XFoam_String XFoam_WordRe::quotemeta() const
{
	std::string out;
	const std::string& s = static_cast<const std::string&>(*this);
	for (std::size_t i = 0; i < s.size(); ++i)
	{
		const char c = s[i];
		if (XFoam_RegExp::meta(c))
		{
			out += '\\';
		}
		out += c;
	}
	return XFoam_String(out);
}

XFoam_OStream& XFoam_WordRe::info(XFoam_OStream& os) const
{
	if (isPattern())
	{
		os << "wordRe(regex) " << static_cast<const std::string&>(*this);
	}
	else
	{
		os << "wordRe(plain) \"" << static_cast<const std::string&>(*this) << '"';
	}
	os.flush();
	return os;
}

void XFoam_WordRe::operator=(const XFoam_WordRe& str)
{
	XFoam_Word::operator=(static_cast<const XFoam_Word&>(str));
	if (str.isPattern())
	{
		(void)compile();
	}
	else
	{
		(void)re_.clear();
	}
}

void XFoam_WordRe::operator=(const XFoam_Word& str)
{
	XFoam_Word::operator=(str);
	(void)re_.clear();
}

void XFoam_WordRe::operator=(const XFoam_KeyType& str)
{
	XFoam_Word::operator=(static_cast<const XFoam_Word&>(str));
	if (str.isPattern())
	{
		(void)compile();
	}
	else
	{
		(void)re_.clear();
	}
}

void XFoam_WordRe::operator=(const XFoam_String& str)
{
	XFoam_Word::operator=(static_cast<const XFoam_String&>(str));
	(void)compile(compOption::detect);
}

void XFoam_WordRe::operator=(const char* str)
{
	XFoam_Word::operator=(XFoam_Word(str ? str : ""));
	(void)compile(compOption::detect);
}

XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_WordRe& w)
{
	XFoam_Token t(is);
	if (!t.good())
	{
		is.setFail();
		return is;
	}
	if (t.isWord())
	{
		w = t.wordToken();
	}
	else if (t.isString())
	{
		w = t.stringToken();
		if (w.empty())
		{
			is.setFail();
			throw XFoam_Error(XFoam_String("empty word/expression in wordRe stream read"));
		}
	}
	else
	{
		is.setFail();
		throw XFoam_Error(XFoam_String("wrong token type for wordRe"));
	}
	return is;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_WordRe& w)
{
	const bool q = w.isPattern();
	if (q)
	{
		os << static_cast<const std::string&>(w);
	}
	else
	{
		os << '"';
		const std::string& s = static_cast<const std::string&>(w);
		for (std::size_t i = 0; i < s.size(); ++i)
		{
			const char c = s[i];
			if (c == '"')
			{
				os << '\\' << '"';
			}
			else if (c == '\\')
			{
				os << '\\' << '\\';
			}
			else
			{
				os << c;
			}
		}
		os << '"';
	}
	return os;
}
