#ifndef XFoam_Tuple2_H_
#define XFoam_Tuple2_H_

// 二元组，对齐 OpenFOAM Foam::Tuple2（Tuple2.H）：异型 first/second、相等、reverse、流与 Hash。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_stream.h"

template<class Type1, class Type2>
class XFoam_Tuple2;

template<class Type1, class Type2>
inline void XFoam_writeEntry(XFoam_OStream& os, const XFoam_Tuple2<Type1, Type2>& t);

template<class Type1, class Type2>
inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Tuple2<Type1, Type2>& t);

template<class Type1, class Type2>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Tuple2<Type1, Type2>& t);

template<class Type1, class Type2>
class XFoam_Tuple2
{
	Type1 f_;
	Type2 s_;

public:
	// 用于无序容器等；OpenFOAM Tuple2::Hash 为增量 seed 形式，此处提供 XFoam_Size 单参哈希（组合 std::hash）。
	struct Hash
	{
		XFoam_Size operator()(const XFoam_Tuple2<Type1, Type2>& t) const noexcept
		{
			const XFoam_Size h1 = std::hash<Type1>{}(t.first());
			const XFoam_Size h2 = std::hash<Type2>{}(t.second());
			return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
		}
	};

	static constexpr const char* typeName = "Tuple2";

	XFoam_Tuple2() = default;

	XFoam_Tuple2(const Type1& f, const Type2& s)
		: f_(f)
		, s_(s)
	{}

	explicit XFoam_Tuple2(XFoam_IStream& is) { is >> *this; }

	const Type1& first() const { return f_; }
	Type1& first() { return f_; }
	const Type2& second() const { return s_; }
	Type2& second() { return s_; }
};

template<class Type1, class Type2>
inline XFoam_Tuple2<Type2, Type1> reverse(const XFoam_Tuple2<Type1, Type2>& t)
{
	return XFoam_Tuple2<Type2, Type1>(t.second(), t.first());
}

template<class Type1, class Type2>
inline bool operator==(const XFoam_Tuple2<Type1, Type2>& a, const XFoam_Tuple2<Type1, Type2>& b)
{
	return (a.first() == b.first() && a.second() == b.second());
}

template<class Type1, class Type2>
inline bool operator!=(const XFoam_Tuple2<Type1, Type2>& a, const XFoam_Tuple2<Type1, Type2>& b)
{
	return !(a == b);
}

template<class Type1, class Type2>
inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Tuple2<Type1, Type2>& t)
{
	char ch = 0;
	if (!(is >> ch))
	{
		return is;
	}
	if (ch == '(')
	{
		is >> t.first() >> t.second() >> ch;
		if (ch != ')')
		{
			is.setFail();
		}
	}
	else
	{
		is.putback(ch);
		is >> t.first() >> t.second();
	}
	return is;
}

template<class Type1, class Type2>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Tuple2<Type1, Type2>& t)
{
	os << '(' << t.first() << ' ' << t.second() << ')';
	return os;
}

template<class Type1, class Type2>
inline void XFoam_writeEntry(XFoam_OStream& os, const XFoam_Tuple2<Type1, Type2>& t)
{
	os << t;
}

template<class T1, class T2 = T1>
class XFoam_Pair
	: public XFoam_Tuple2<T1, T2>
{
public:
	using XFoam_Tuple2<T1, T2>::XFoam_Tuple2;
};

template<class Type1, class Type2, class Type3>
class XFoam_Tuple3
{
	Type1 f_;
	Type2 s_;
	Type3 t_;

public:
	static constexpr const char* typeName = "Tuple3";

	XFoam_Tuple3() = default;

	XFoam_Tuple3(const Type1& f, const Type2& s, const Type3& t)
		: f_(f)
		, s_(s)
		, t_(t)
	{}

	const Type1& first() const { return f_; }
	Type1& first() { return f_; }
	const Type2& second() const { return s_; }
	Type2& second() { return s_; }
	const Type3& third() const { return t_; }
	Type3& third() { return t_; }
};

template<class Type1, class Type2, class Type3>
inline bool operator==(
	const XFoam_Tuple3<Type1, Type2, Type3>& a,
	const XFoam_Tuple3<Type1, Type2, Type3>& b)
{
	return (a.first() == b.first() && a.second() == b.second() && a.third() == b.third());
}

template<class Type1, class Type2, class Type3>
inline bool operator!=(
	const XFoam_Tuple3<Type1, Type2, Type3>& a,
	const XFoam_Tuple3<Type1, Type2, Type3>& b)
{
	return !(a == b);
}

#endif
