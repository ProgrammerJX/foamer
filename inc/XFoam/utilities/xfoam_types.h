#ifndef XFoam_Utilities_Types_H_
#define XFoam_Utilities_Types_H_

// XFoam 模块内标准库头文件统一由此文件引入；其它 XFoam 头文件请勿再直接 #include <...>。
// 例外：仅 C++17 可用头文件（如 <filesystem>）、平台头（<endian.h>、<intrin.h>）可在对应 .cpp 中单独包含。
// XFoam_API 宏与此处定义。
// （工程内文件名：xfoam_types.h，与口语中的 XFoam_Types.h 同义。）

#ifdef X_BUILD_STATIC
#define XFoam_API
#else
#ifdef __unix
#define XFoam_API
#else
#ifdef XFoam_Export
#define XFoam_API __declspec(dllexport)
#else
#define XFoam_API __declspec(dllimport)
#endif
#endif
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <iostream>
#include <vector>

typedef int XFoam_Label;
typedef std::string XFoam_String;
// 区别于 OpenFOAM：Foam::scalar 由编译选项在 float / double / long double 之间切换（如 SP/DP/LDP）。
// XFoam 将 XFoam_Scalar 强制 typedef 为 double，与 CMake 中 XFOAM_SCALAR_* 宏是否仍存在无关；模块内标量请统一用本类型。
typedef double XFoam_Scalar;
typedef std::size_t XFoam_Size;

// 固定宽度整数、流字节计数与字符缓冲：与 std 底层相同，模块内统一用 XFoam_* 命名，减少对裸 std::uint32_t 等的直接依赖。
typedef std::uint8_t XFoam_UInt8;
typedef std::uint16_t XFoam_UInt16;
typedef std::uint32_t XFoam_UInt32;
typedef std::uint64_t XFoam_UInt64;
typedef std::int8_t XFoam_Int8;
typedef std::int16_t XFoam_Int16;
typedef std::int32_t XFoam_Int32;
typedef std::int64_t XFoam_Int64;
typedef std::streamsize XFoam_StreamSize;
typedef std::vector<char> XFoam_CharBuffer;

class XFoam_IStream;
class XFoam_OStream;
class XFoam_Dictionary;

/// 串行占位：与 OpenFOAM PstreamBuffers 形参位置一致，无 MPI 缓冲逻辑；供 polyPatch、polyBoundaryMesh 等接口共用。
class XFoam_API XFoam_PstreamBuffers
{};

#define XFoam_swap  (std::swap)
#define XFoam_move  (std::move)
#define XFoam_debug  1


// 与 OpenFOAM 一致：word 与 string 为不同类型，以便 dictionary::add 等重载可区分。
class XFoam_Word
	: public XFoam_String
{
public:
	using std::string::string;

	XFoam_Word() = default;

	XFoam_Word(const char* s)
		: std::string(s ? s : "")
	{}

	XFoam_Word(const std::string& s)
		: std::string(s)
	{}

	XFoam_Word(std::string&& s) noexcept
		: std::string(std::move(s))
	{}

	explicit XFoam_Word(std::istream& is)
	{
		std::string t;
		is >> t;
		static_cast<std::string&>(*this) = std::move(t);
	}

	static const XFoam_Word null;

	// 与 OpenFOAM keyType / word 的 isPattern 对齐的简化实现（用于 dictionary 模式表）。
	bool isPattern() const noexcept
	{
		return !empty() && front() == '~';
	}

	// Foam::word::valid
	inline static bool valid(char c)
	{
		return (!std::isspace(static_cast<unsigned char>(c)) && c != '"' && c != '\'' && c != '/' && c != '$'
				&& c != ';' && c != '{' && c != '}');
	}
};

template<class T>
class XFoam_List;

class XFoam_FileName;

XFoam_IStream& operator>>(XFoam_IStream&, XFoam_FileName&);
XFoam_OStream& operator<<(XFoam_OStream&, const XFoam_FileName&);

enum class XFoam_FileType
{
	undefined,
	file,
	directory,
	link
};

class XFoam_API XFoam_FileName
	: public XFoam_String
{
private:
	inline void stripInvalid();

public:
	static const char* const typeName;
	static int debug;
	static const XFoam_FileName null;

	inline XFoam_FileName()
		: XFoam_String()
	{
	}

	inline XFoam_FileName(const XFoam_FileName& fn)
		: XFoam_String(fn)
	{
	}

	inline XFoam_FileName(XFoam_FileName&& fn) noexcept
		: XFoam_String(std::move(fn))
	{
	}

	inline XFoam_FileName(const XFoam_Word& w)
		: XFoam_String(static_cast<const XFoam_String&>(w))
	{
	}

	inline XFoam_FileName(const XFoam_String& str)
		: XFoam_String(str)
	{
		stripInvalid();
	}

	// XFoam 注：Foam::string 与 std::string 在 OF 中为不同类型；此处 XFoam_String 即 std::string，故不另写 std::string 重载。

	inline XFoam_FileName(const char* str)
		: XFoam_String(str ? str : "")
	{
		stripInvalid();
	}

	explicit XFoam_FileName(const XFoam_List<XFoam_Word>& lst);

	// 未移植：OF fileName(Istream) 与 token/字典链；不从字典构造。此处仅占位，体为空。
	explicit XFoam_FileName(std::istream& is);

	inline static bool valid(char c)
	{
		return !std::isspace(static_cast<unsigned char>(c)) && c != '"' && c != '\'';
	}

	bool clean();
	XFoam_FileName clean() const;

	XFoam_FileType type(bool checkVariants = true, bool followLink = true) const;

	inline bool isName() const { return find_first_of("/\\") == XFoam_String::npos; }

	inline bool hasPath() const { return find_first_of("/\\") != XFoam_String::npos; }

	inline bool isAbsolute() const
	{
		if (empty())
		{
			return false;
		}
		// POSIX
		if (operator[](0) == '/')
		{
			return true;
		}
		// Windows: D:/... 或 D:\...（仅判首字符为 '/' 会把盘符路径当相对路径，进而 system/ 错误拼接）
		if (size() >= 2 && operator[](1) == ':')
		{
			const unsigned char d = static_cast<unsigned char>(operator[](0));
			if (std::isalpha(d))
			{
				return true;
			}
		}
		// Windows UNC \\server\share\...
		if (size() >= 2 && operator[](0) == '\\' && operator[](1) == '\\')
		{
			return true;
		}
		return false;
	}

	XFoam_FileName& toAbsolute();

	inline XFoam_Word name() const
	{
		const size_type i = find_last_of("/\\");
		if (i == npos)
		{
			return XFoam_Word(static_cast<const XFoam_String&>(*this));
		}
		return XFoam_Word(substr(i + 1));
	}

	XFoam_String caseName() const;

	// 未移植：OF 相对 FOAM_CASE 的路径归一化；返回空字符串。
	inline XFoam_String relativePath() const { return XFoam_String(); }

	inline XFoam_Word name(const bool noExt) const
	{
		if (noExt)
		{
			size_type beg = find_last_of("/\\");
			if (beg == npos)
			{
				beg = 0;
			}
			else
			{
				++beg;
			}

			size_type dot = rfind('.');
			if (dot != npos && dot <= beg)
			{
				dot = npos;
			}

			if (dot == npos)
			{
				return XFoam_Word(substr(beg));
			}
			return XFoam_Word(substr(beg, dot - beg));
		}
		return name();
	}

	inline XFoam_FileName path() const
	{
		const size_type i = find_last_of("/\\");
		if (i == npos)
		{
			return XFoam_FileName(".");
		}
		if (i == 0)
		{
			return XFoam_FileName(substr(0, 1));
		}
		return XFoam_FileName(substr(0, i));
	}

	inline XFoam_FileName lessExt() const
	{
		const size_type i = find_last_of("./");
		if (i == npos || i == 0 || operator[](i) == '/')
		{
			return *this;
		}
		return XFoam_FileName(substr(0, i));
	}

	inline XFoam_Word ext() const
	{
		const size_type i = find_last_of("./");
		if (i == npos || i == 0 || operator[](i) == '/')
		{
			return XFoam_Word::null;
		}
		return XFoam_Word(substr(i + 1));
	}

	XFoam_List<XFoam_Word> components(const char delimiter = '/') const;

	XFoam_Word component(const size_type cmpt, const char delimiter = '/') const;

	void operator=(const XFoam_FileName& str);
	void operator=(XFoam_FileName&& str);
	inline void operator=(const XFoam_Word& str) { XFoam_String::operator=(static_cast<const XFoam_String&>(str)); }

	void operator=(const XFoam_String& str);
	void operator=(const char* str);

	void operator/=(const XFoam_String&);

	friend std::istream& operator>>(std::istream&, XFoam_FileName&);
	friend std::ostream& operator<<(std::ostream&, const XFoam_FileName&);
};

inline XFoam_FileName operator/(const XFoam_String& a, const XFoam_String& b)
{
	if (a.size())
	{
		if (b.size())
		{
			return XFoam_FileName(a + '/' + b);
		}
		return XFoam_FileName(a);
	}
	if (b.size())
	{
		return XFoam_FileName(b);
	}
	return XFoam_FileName();
}

inline void XFoam_FileName::stripInvalid()
{
	for (size_type i = 0; i < size();)
	{
		if (!valid(static_cast<char>(operator[](i))))
		{
			erase(i, 1);
		}
		else
		{
			++i;
		}
	}
}

/*---------------------------------------------------------------------------*\
                        Class XFoam_Variable Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_Variable
	: public XFoam_Word
{
	void stripInvalid();

public:
	// XFoam_String 与 std::string 为同一类型；仅保留 std::string 重载。
	static const char* const typeName;
	static int debug;
	static const XFoam_Variable null;

	XFoam_Variable();
	XFoam_Variable(const XFoam_Variable&);
	explicit XFoam_Variable(const XFoam_Word&);
	explicit XFoam_Variable(const std::string&, const bool doStripInvalid = true);
	explicit XFoam_Variable(const char*, const bool doStripInvalid = true);
	explicit XFoam_Variable(std::istream&);

	static bool valid(char c);

	void clear() { std::string::clear(); }

	void operator=(const XFoam_Variable&);
	void operator=(const XFoam_Word&);
	void operator=(const std::string&);
	void operator=(const char*);

	friend XFoam_API std::istream& operator>>(std::istream&, XFoam_Variable&);
	friend XFoam_API std::ostream& operator<<(std::ostream&, const XFoam_Variable&);
};

XFoam_API void XFoam_writeEntry(std::ostream& os, const XFoam_Variable& value);

/*---------------------------------------------------------------------------*\
                        Class XFoam_FunctionName Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_FunctionName
	: public XFoam_Word
{
public:
	static const char* const typeName;
	static int debug;
	static const XFoam_FunctionName null;

	XFoam_FunctionName();
	XFoam_FunctionName(const XFoam_FunctionName&);
	explicit XFoam_FunctionName(const XFoam_Word&);
	explicit XFoam_FunctionName(const std::string&, const bool doStripInvalid = true);
	explicit XFoam_FunctionName(const char*, const bool doStripInvalid = true);
	explicit XFoam_FunctionName(std::istream&);

	static bool valid(char c);

	void operator=(const XFoam_FunctionName&);
	void operator=(const XFoam_Word&);
	void operator=(const std::string&);
	void operator=(const char*);

	friend XFoam_API std::istream& operator>>(std::istream&, XFoam_FunctionName&);
	friend XFoam_API std::ostream& operator<<(std::ostream&, const XFoam_FunctionName&);
};

XFoam_API void XFoam_writeEntry(XFoam_OStream& os, const XFoam_FunctionName& value);

/*---------------------------------------------------------------------------*\
                        Class XFoam_Switch Declaration
	对标 OpenFOAM primitives/bools/Switch/Switch.H（bool 包装，可按词读入 true/false、on/off 等）。
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_Switch
{
public:
	// 类内枚举：PascalCase（见 doc/foam_code.md）；与 OF Foam::Switch::switchType 逐项对应。
	enum class SwitchType : unsigned char
	{
		False = 0,
		True = 1,
		off = 2,
		on = 3,
		no = 4,
		yes = 5,
		n = 6,
		y = 7,
		f = 8,
		t = 9,
		none = 10,
		any = 11,
		invalid
	};

	static constexpr unsigned char nSwitchType = static_cast<unsigned char>(SwitchType::invalid) + 1;

	static unsigned char toInt(const SwitchType x) { return static_cast<unsigned char>(x); }

	inline friend SwitchType operator++(SwitchType& x)
	{
		x = SwitchType(static_cast<unsigned char>(toInt(x) + 1));
		return x;
	}

private:
	SwitchType switch_;

	static const char* names[nSwitchType];

	static SwitchType asEnum(const std::string& str, const bool allowInvalid);

public:
	XFoam_Switch()
		: switch_(SwitchType::False)
	{
	}

	explicit XFoam_Switch(const SwitchType sw)
		: switch_(sw)
	{
	}

	XFoam_Switch(const bool b)
		: switch_(b ? SwitchType::True : SwitchType::False)
	{
	}

	XFoam_Switch(const int i)
		: switch_(i ? SwitchType::True : SwitchType::False)
	{
	}

	XFoam_Switch(const std::string& str, const bool allowInvalid = false)
		: switch_(asEnum(str, allowInvalid))
	{
	}

	XFoam_Switch(const char* str, const bool allowInvalid = false)
		: switch_(asEnum(std::string(str ? str : ""), allowInvalid))
	{
	}

	explicit XFoam_Switch(XFoam_IStream& is);

	bool valid() const;

	const char* asText() const;

	bool readIfPresent(const XFoam_Word& name, const XFoam_Dictionary& dict);

	operator bool() const { return (toInt(switch_) & 0x1) != 0; }

	void operator=(const SwitchType sw) { switch_ = sw; }

	void operator=(const bool b) { switch_ = (b ? SwitchType::True : SwitchType::False); }

	friend XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Switch& s);
	friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Switch& s);
};

// 对标 OpenFOAM primitives/Scalar/doubleScalar/doubleScalar.H 中 doubleScalar* 常量。
constexpr XFoam_Scalar XFoam_vGreat = static_cast<XFoam_Scalar>(
	std::numeric_limits<double>::max() / 10.0);
constexpr XFoam_Scalar XFoam_vSmall =
	static_cast<XFoam_Scalar>(std::numeric_limits<double>::min());
constexpr XFoam_Scalar XFoam_small =
	static_cast<XFoam_Scalar>(std::numeric_limits<double>::epsilon());
constexpr XFoam_Scalar XFoam_great =
	static_cast<XFoam_Scalar>(1.0 / static_cast<double>(XFoam_small));

// C++11：非 constexpr 初始化用 static const，头文件内每 TU 一份，避免 C++17 的 inline 变量。
static const XFoam_Scalar XFoam_rootVGreat = std::sqrt(XFoam_vGreat);
static const XFoam_Scalar XFoam_rootVSmall = std::sqrt(XFoam_vSmall);
static const XFoam_Scalar XFoam_rootGreat = std::sqrt(XFoam_great);
static const XFoam_Scalar XFoam_rootSmall = std::sqrt(XFoam_small);

inline XFoam_Scalar XFoam_sqrt(XFoam_Scalar x)
{
	return std::sqrt(x);
}
inline XFoam_Scalar XFoam_mag(XFoam_Scalar x)
{
	return std::fabs(x);
}
inline XFoam_Scalar XFoam_pow(XFoam_Scalar x, XFoam_Scalar y)
{
	return std::pow(x, y);
}
inline XFoam_Scalar XFoam_sqr(XFoam_Scalar x)
{
	return x * x;
}

// C++11：constexpr 成员函数体只能实质为一条 return（C++14 起才允许多语句）；此处保持单 return。
// C++11 下两参 min/max 不作 constexpr：编译期常量表达式对浮点等待遇到 C++14 才与实现一致。
// 仅算术类型，避免与 XFoam_Field 等类型的二参 min/max（xfoam_field.h）在重载解析中混淆。
template<typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value, T>::type XFoam_max(
	T a,
	T b)
{
	return (a < b) ? b : a;
}

template<typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value, T>::type XFoam_min(
	T a,
	T b)
{
	return (a < b) ? a : b;
}

// 原始类型特征（Foam::pTraits — primitives/pTraits/pTraits.H）。类类型可沿用主模板；标量 typedef 见下特化。
template<class PrimitiveType>
class XFoam_pTraits
	: public PrimitiveType
{
public:
	typedef PrimitiveType cmptType;

	XFoam_pTraits(const PrimitiveType& p)
		: PrimitiveType(p)
	{}

	XFoam_pTraits(std::istream& is)
		: PrimitiveType(is)
	{}
};

template<>
class XFoam_pTraits<XFoam_Scalar>
{
	XFoam_Scalar p_;

public:
	typedef XFoam_Scalar cmptType;

	XFoam_pTraits(const XFoam_Scalar& p)
		: p_(p)
	{}

	XFoam_pTraits(std::istream& is)
	{
		is >> p_;
	}

	operator XFoam_Scalar() const { return p_; }
};

template<>
class XFoam_pTraits<XFoam_Label>
{
	XFoam_Label p_;

public:
	typedef XFoam_Label cmptType;

	XFoam_pTraits(const XFoam_Label& p)
		: p_(p)
	{}

	XFoam_pTraits(std::istream& is)
	{
		is >> p_;
	}

	operator XFoam_Label() const { return p_; }
};

template<typename Type>
inline Type XFoam_read(std::istream& is)
{
	return XFoam_pTraits<Type>(is);
}

// 显式零初始化标签（如 XFoam_Vector(XFoam_Zero_v)）。
struct XFoam_Zero_Tag
{};
constexpr XFoam_Zero_Tag XFoam_Zero_v{};

namespace std
{
template<>
struct hash<XFoam_Word>
{
	size_t operator()(const XFoam_Word& w) const noexcept
	{
		return hash<std::string>()(static_cast<const std::string&>(w));
	}
};
} // namespace std

#endif
