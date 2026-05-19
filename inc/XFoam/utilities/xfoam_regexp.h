#ifndef XFoam_RegExp_H_
#define XFoam_RegExp_H_

// 对标 OpenFOAM OSspecific/POSIX/regExp.H。
// Windows/MSVC 无 POSIX <regex.h>：preg_ 为 std::regex*，语义按 regExp.C 近似（非 POSIX 扩展正则完全一致处见实现注释）。
// 非 MSVC 平台使用 POSIX regcomp/regexec，逻辑照抄 regExp.C。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_list.h"
#include <string>

#if defined(_MSC_VER)
#include <regex>
#define XFOAM_REGEXP_IMPL_STD 1
#else
#include <regex.h>
#define XFOAM_REGEXP_IMPL_STD 0
#endif

/*---------------------------------------------------------------------------*\
                           Class XFoam_RegExp Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_RegExp
{
#if XFOAM_REGEXP_IMPL_STD
	mutable std::regex* preg_; // mutable：与 OF regExp::set/clear 的 const 限定一致
#else
	mutable regex_t* preg_;
#endif

	template<class StringType>
	bool matchGrouping(const std::string&, XFoam_List<StringType>& groups) const;

public:
	inline static bool meta(char c)
	{
		return (
			(c == '.') || (c == '*' || c == '+' || c == '?') || (c == '(' || c == ')' || c == '|') || (c == '[' || c == ']'));
	}

	XFoam_RegExp();
	XFoam_RegExp(const char*, const bool ignoreCase = false);
	XFoam_RegExp(const std::string&, const bool ignoreCase = false);
	XFoam_RegExp(const XFoam_RegExp&) = delete;
	~XFoam_RegExp();

	inline bool empty() const { return !preg_; }
	inline bool exists() const { return preg_ ? true : false; }
	inline int ngroups() const
	{
#if XFOAM_REGEXP_IMPL_STD
		return int(preg_ ? static_cast<int>(preg_->mark_count()) : 0);
#else
		return int(preg_ ? preg_->re_nsub : 0);
#endif
	}

	void set(const char*, const bool ignoreCase = false) const;
	void set(const std::string&, const bool ignoreCase = false) const;
	bool clear() const;

	std::string::size_type find(const std::string& str) const;
	bool match(const std::string&) const;
	bool match(const std::string&, XFoam_List<std::string>& groups) const;
	bool search(const std::string& str) const { return std::string::npos != find(str); }

	void operator=(const XFoam_RegExp&) = delete;
	void operator=(const char*);
	void operator=(const std::string&);
};

class XFoam_KeyType;

/*---------------------------------------------------------------------------*\
                           Class XFoam_WordRe Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_WordRe
	: public XFoam_Word
{
	mutable XFoam_RegExp re_;

public:
	static const XFoam_WordRe null;

	enum class compOption
	{
		literal = 0,
		detect = 1,
		regExp = 2,
		noCase = 4,
		detectNoCase = detect | noCase,
		regExpNoCase = regExp | noCase
	};

	static bool meta(char c) { return XFoam_RegExp::meta(c); }

	static bool isPattern(const XFoam_String& str);

	XFoam_WordRe();

	XFoam_WordRe(const XFoam_WordRe& str);

	XFoam_WordRe(const XFoam_KeyType& str);

	XFoam_WordRe(const XFoam_KeyType& str, compOption opt);

	XFoam_WordRe(const XFoam_Word& str);

	explicit XFoam_WordRe(const char* str, compOption opt = compOption::literal);

	explicit XFoam_WordRe(const XFoam_String& str, compOption opt = compOption::literal);
	// OpenFOAM 另有 wordRe(const std::string&, compOption)；XFoam 中 XFoam_String 即 std::string，不能重载两份。

	explicit XFoam_WordRe(XFoam_IStream& is);

	bool isPattern() const;

	bool compile() const;

	bool compile(compOption opt) const;

	bool recompile() const;

	void uncompile(bool doStripInvalid = false) const;

	void clear();

	void set(const std::string& str, compOption opt = compOption::detect);

	void set(const char* str, compOption opt = compOption::detect);

	bool match(const std::string& str, bool literalMatch = false) const;

	XFoam_String quotemeta() const;

	XFoam_OStream& info(XFoam_OStream& os) const;

	void operator=(const XFoam_WordRe& str);

	void operator=(const XFoam_Word& str);

	void operator=(const XFoam_KeyType& str);

	void operator=(const XFoam_String& str);
	// OpenFOAM 另有 operator=(const std::string&)；XFoam_String 同 std::string，此处合并为一份。

	void operator=(const char* str);

	friend XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_WordRe& w);

	friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_WordRe& w);
};

inline int operator&(const XFoam_WordRe::compOption co1, const XFoam_WordRe::compOption co2)
{
	return int(co1) & int(co2);
}

typedef XFoam_List<XFoam_WordRe> XFoam_WordReList;

#endif
