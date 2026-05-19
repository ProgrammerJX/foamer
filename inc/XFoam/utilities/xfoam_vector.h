#ifndef XFoam_Vector_H_
#define XFoam_Vector_H_

// 三维向量，对齐 OpenFOAM Foam::Vector（Vector.H / VectorI.H）：分量访问 x/y/z、点积 &、叉积 ^。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_stream.h"

template<class T>
class XFoam_Vector
{
public:
	using labelType = XFoam_Vector<XFoam_Label>;

	static constexpr unsigned char rank = 1;

	enum components : unsigned char
	{
		X,
		Y,
		Z
	};

	T v_[3];

	XFoam_Vector()
		: v_{}
	{}

	explicit XFoam_Vector(XFoam_Zero_Tag)
		: v_{}
	{}

	XFoam_Vector(const T& vx, const T& vy, const T& vz)
	{
		v_[X] = vx;
		v_[Y] = vy;
		v_[Z] = vz;
	}

	const T& x() const { return v_[X]; }
	const T& y() const { return v_[Y]; }
	const T& z() const { return v_[Z]; }
	T& x() { return v_[X]; }
	T& y() { return v_[Y]; }
	T& z() { return v_[Z]; }

	const T& operator[](XFoam_Size i) const { return v_[i]; }
	T& operator[](XFoam_Size i) { return v_[i]; }

	T magSqr() const
	{
		return static_cast<T>(v_[X] * v_[X] + v_[Y] * v_[Y] + v_[Z] * v_[Z]);
	}

	double mag() const { return std::sqrt(static_cast<double>(magSqr())); }

	template<class ListLike>
	const XFoam_Vector& centre(const ListLike&) const
	{
		return *this;
	}

	const XFoam_Vector& centre() const { return *this; }

	XFoam_Vector& operator+=(const XFoam_Vector& b)
	{
		v_[X] += b.v_[X];
		v_[Y] += b.v_[Y];
		v_[Z] += b.v_[Z];
		return *this;
	}

	XFoam_Vector& operator-=(const XFoam_Vector& b)
	{
		v_[X] -= b.v_[X];
		v_[Y] -= b.v_[Y];
		v_[Z] -= b.v_[Z];
		return *this;
	}

	XFoam_Vector& operator*=(const T& s)
	{
		v_[X] *= s;
		v_[Y] *= s;
		v_[Z] *= s;
		return *this;
	}

	XFoam_Vector& operator/=(const T& s)
	{
		v_[X] /= s;
		v_[Y] /= s;
		v_[Z] /= s;
		return *this;
	}

	XFoam_Vector operator-() const { return XFoam_Vector(-v_[X], -v_[Y], -v_[Z]); }
};

typedef XFoam_Vector<XFoam_Scalar> XFoam_Vector3D;
typedef XFoam_Vector<XFoam_Scalar> XFoam_Point3D;

template<class T>
inline XFoam_Vector<T> operator+(
	const XFoam_Vector<T>& a,
	const XFoam_Vector<T>& b)
{
	return XFoam_Vector<T>(
		a.x() + b.x(),
		a.y() + b.y(),
		a.z() + b.z());
}

template<class T>
inline XFoam_Vector<T> operator-(
	const XFoam_Vector<T>& a,
	const XFoam_Vector<T>& b)
{
	return XFoam_Vector<T>(
		a.x() - b.x(),
		a.y() - b.y(),
		a.z() - b.z());
}

template<class T>
inline XFoam_Vector<T> operator*(const XFoam_Vector<T>& a, const T& s)
{
	return XFoam_Vector<T>(a.x() * s, a.y() * s, a.z() * s);
}

template<class T>
inline XFoam_Vector<T> operator*(const T& s, const XFoam_Vector<T>& a)
{
	return a * s;
}

/// 分量乘积（对标 OpenFOAM cmptMultiply / 向量 Field 的 *）。
template<class T>
inline XFoam_Vector<T> operator*(
	const XFoam_Vector<T>& a, const XFoam_Vector<T>& b)
{
	return XFoam_Vector<T>(
		static_cast<T>(a.x() * b.x()),
		static_cast<T>(a.y() * b.y()),
		static_cast<T>(a.z() * b.z()));
}

template<class T>
inline XFoam_Vector<T> operator/(const XFoam_Vector<T>& a, const T& s)
{
	return XFoam_Vector<T>(a.x() / s, a.y() / s, a.z() / s);
}

// 与 OpenFOAM normalised 等价：|v|>rootSmall 时返回 v/|v|，否则零向量。
template<class T>
inline XFoam_Vector<T> XFoam_normalised(const XFoam_Vector<T>& v)
{
	const XFoam_Scalar m = static_cast<XFoam_Scalar>(v.mag());
	return m > XFoam_rootSmall ? v / static_cast<T>(m) : XFoam_Vector<T>(XFoam_Zero_v);
}

template<class T>
inline T operator&(const XFoam_Vector<T>& a, const XFoam_Vector<T>& b)
{
	return static_cast<T>(a.x() * b.x() + a.y() * b.y() + a.z() * b.z());
}

template<class T>
inline XFoam_Vector<T> operator^(
	const XFoam_Vector<T>& a,
	const XFoam_Vector<T>& b)
{
	return XFoam_Vector<T>(
		static_cast<T>(a.y() * b.z() - a.z() * b.y()),
		static_cast<T>(a.z() * b.x() - a.x() * b.z()),
		static_cast<T>(a.x() * b.y() - a.y() * b.x()));
}

template<class T>
inline bool operator==(const XFoam_Vector<T>& a, const XFoam_Vector<T>& b)
{
	return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
}

template<class T>
inline bool operator!=(const XFoam_Vector<T>& a, const XFoam_Vector<T>& b)
{
	return !(a == b);
}

template<class T>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Vector<T>& v)
{
	os << '(' << v.x() << ' ' << v.y() << ' ' << v.z() << ')';
	return os;
}


inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_Vector3D& v)
{
	char ch = 0;
	if (!(is >> ch))
	{
		return is;
	}
	if (ch == '(')
	{
		is >> v.x() >> v.y() >> v.z() >> ch;
		if (ch != ')')
		{
			is.setFail();
		}
	}
	else
	{
		is.putback(ch);
		is >> v.x() >> v.y() >> v.z();
	}
	return is;
}

#endif
