#ifndef XFoam_Tensor_H_
#define XFoam_Tensor_H_

// 三维二阶张量，对齐 OpenFOAM Foam::Tensor（Tensor.H / TensorI.H）的主要接口与运算。
// 未移植：SphericalTensor / SymmTensor / DiagTensor 转换、Istream 构造、eigenValues 等（见 tensor.C）。

#include "XFoam/utilities/xfoam_vector.h"

template<class Cmpt>
class XFoam_Tensor
{
public:
	using labelType = XFoam_Tensor<XFoam_Label>;

	static constexpr unsigned char rank = 2;

	enum components : unsigned char
	{
		XX,
		XY,
		XZ,
		YX,
		YY,
		YZ,
		ZX,
		ZY,
		ZZ
	};

	Cmpt v_[9];

	XFoam_Tensor()
		: v_{}
	{}

	explicit XFoam_Tensor(XFoam_Zero_Tag)
		: v_{}
	{}

	XFoam_Tensor(
		const Cmpt& txx,
		const Cmpt& txy,
		const Cmpt& txz,
		const Cmpt& tyx,
		const Cmpt& tyy,
		const Cmpt& tyz,
		const Cmpt& tzx,
		const Cmpt& tzy,
		const Cmpt& tzz)
	{
		v_[XX] = txx;
		v_[XY] = txy;
		v_[XZ] = txz;
		v_[YX] = tyx;
		v_[YY] = tyy;
		v_[YZ] = tyz;
		v_[ZX] = tzx;
		v_[ZY] = tzy;
		v_[ZZ] = tzz;
	}

	XFoam_Tensor(
		const XFoam_Vector<Cmpt>& vx,
		const XFoam_Vector<Cmpt>& vy,
		const XFoam_Vector<Cmpt>& vz)
	{
		v_[XX] = vx.x();
		v_[XY] = vx.y();
		v_[XZ] = vx.z();
		v_[YX] = vy.x();
		v_[YY] = vy.y();
		v_[YZ] = vy.z();
		v_[ZX] = vz.x();
		v_[ZY] = vz.y();
		v_[ZZ] = vz.z();
	}

	static XFoam_Tensor I()
	{
		return XFoam_Tensor(
			Cmpt(1),
			Cmpt(0),
			Cmpt(0),
			Cmpt(0),
			Cmpt(1),
			Cmpt(0),
			Cmpt(0),
			Cmpt(0),
			Cmpt(1));
	}

	const Cmpt& xx() const { return v_[XX]; }
	const Cmpt& xy() const { return v_[XY]; }
	const Cmpt& xz() const { return v_[XZ]; }
	const Cmpt& yx() const { return v_[YX]; }
	const Cmpt& yy() const { return v_[YY]; }
	const Cmpt& yz() const { return v_[YZ]; }
	const Cmpt& zx() const { return v_[ZX]; }
	const Cmpt& zy() const { return v_[ZY]; }
	const Cmpt& zz() const { return v_[ZZ]; }

	Cmpt& xx() { return v_[XX]; }
	Cmpt& xy() { return v_[XY]; }
	Cmpt& xz() { return v_[XZ]; }
	Cmpt& yx() { return v_[YX]; }
	Cmpt& yy() { return v_[YY]; }
	Cmpt& yz() { return v_[YZ]; }
	Cmpt& zx() { return v_[ZX]; }
	Cmpt& zy() { return v_[ZY]; }
	Cmpt& zz() { return v_[ZZ]; }

	const Cmpt& operator[](XFoam_Size i) const { return v_[i]; }
	Cmpt& operator[](XFoam_Size i) { return v_[i]; }

	XFoam_Vector<Cmpt> x() const
	{
		return XFoam_Vector<Cmpt>(v_[XX], v_[XY], v_[XZ]);
	}

	XFoam_Vector<Cmpt> y() const
	{
		return XFoam_Vector<Cmpt>(v_[YX], v_[YY], v_[YZ]);
	}

	XFoam_Vector<Cmpt> z() const
	{
		return XFoam_Vector<Cmpt>(v_[ZX], v_[ZY], v_[ZZ]);
	}

	XFoam_Vector<Cmpt> vectorComponent(int cmpt) const
	{
		switch (cmpt)
		{
		case 0:
			return x();
		case 1:
			return y();
		case 2:
			return z();
		default:
			return XFoam_Vector<Cmpt>(XFoam_Zero_v);
		}
	}

	XFoam_Tensor T() const
	{
		return XFoam_Tensor(
			xx(),
			yx(),
			zx(),
			xy(),
			yy(),
			zy(),
			xz(),
			yz(),
			zz());
	}

	XFoam_Tensor inv() const;

	void operator&=(const XFoam_Tensor& t);

	XFoam_Tensor& operator+=(const XFoam_Tensor& b)
	{
		for (int i = 0; i < 9; ++i)
		{
			v_[static_cast<XFoam_Size>(i)] += b.v_[static_cast<XFoam_Size>(i)];
		}
		return *this;
	}

	XFoam_Tensor& operator-=(const XFoam_Tensor& b)
	{
		for (int i = 0; i < 9; ++i)
		{
			v_[static_cast<XFoam_Size>(i)] -= b.v_[static_cast<XFoam_Size>(i)];
		}
		return *this;
	}

	XFoam_Tensor& operator*=(const Cmpt& s)
	{
		for (int i = 0; i < 9; ++i)
		{
			v_[static_cast<XFoam_Size>(i)] *= s;
		}
		return *this;
	}

	XFoam_Tensor& operator/=(const Cmpt& s)
	{
		for (int i = 0; i < 9; ++i)
		{
			v_[static_cast<XFoam_Size>(i)] /= s;
		}
		return *this;
	}
};

typedef XFoam_Tensor<double> XFoam_TensorD;

// * * * * * * * * * * * * * * * Free functions (TensorI.H) * * * * * * * * * * * * * //

template<class Cmpt>
inline Cmpt XFoam_tr(const XFoam_Tensor<Cmpt>& t)
{
	return static_cast<Cmpt>(t.xx() + t.yy() + t.zz());
}

template<class Cmpt>
inline Cmpt XFoam_det(const XFoam_Tensor<Cmpt>& t)
{
	return static_cast<Cmpt>(
		t.xx() * t.yy() * t.zz() + t.xy() * t.yz() * t.zx() + t.xz() * t.yx() * t.zy()
		- t.xx() * t.yz() * t.zy() - t.xy() * t.yx() * t.zz() - t.xz() * t.yy() * t.zx());
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> XFoam_cof(const XFoam_Tensor<Cmpt>& t)
{
	return XFoam_Tensor<Cmpt>(
		static_cast<Cmpt>(t.yy() * t.zz() - t.zy() * t.yz()),
		static_cast<Cmpt>(t.zx() * t.yz() - t.yx() * t.zz()),
		static_cast<Cmpt>(t.yx() * t.zy() - t.yy() * t.zx()),
		static_cast<Cmpt>(t.xz() * t.zy() - t.xy() * t.zz()),
		static_cast<Cmpt>(t.xx() * t.zz() - t.xz() * t.zx()),
		static_cast<Cmpt>(t.xy() * t.zx() - t.xx() * t.zy()),
		static_cast<Cmpt>(t.xy() * t.yz() - t.xz() * t.yy()),
		static_cast<Cmpt>(t.yx() * t.xz() - t.xx() * t.yz()),
		static_cast<Cmpt>(t.xx() * t.yy() - t.yx() * t.xy()));
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> XFoam_inv(const XFoam_Tensor<Cmpt>& t, const Cmpt& dett)
{
	// 伴随矩阵 / det（与 OpenFOAM inv(Tensor, det) 一致；非 cof/det）
	return XFoam_Tensor<Cmpt>(
		static_cast<Cmpt>(t.yy() * t.zz() - t.zy() * t.yz()),
		static_cast<Cmpt>(t.xz() * t.zy() - t.xy() * t.zz()),
		static_cast<Cmpt>(t.xy() * t.yz() - t.xz() * t.yy()),
		static_cast<Cmpt>(t.zx() * t.yz() - t.yx() * t.zz()),
		static_cast<Cmpt>(t.xx() * t.zz() - t.xz() * t.zx()),
		static_cast<Cmpt>(t.yx() * t.xz() - t.xx() * t.yz()),
		static_cast<Cmpt>(t.yx() * t.zy() - t.yy() * t.zx()),
		static_cast<Cmpt>(t.xy() * t.zx() - t.xx() * t.zy()),
		static_cast<Cmpt>(t.xx() * t.yy() - t.yx() * t.xy())) / dett;
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> XFoam_inv(const XFoam_Tensor<Cmpt>& t)
{
	return XFoam_inv(t, XFoam_det(t));
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> XFoam_Tensor<Cmpt>::inv() const
{
	return XFoam_inv(*this);
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator+(XFoam_Tensor<Cmpt> a, const XFoam_Tensor<Cmpt>& b)
{
	a += b;
	return a;
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator-(XFoam_Tensor<Cmpt> a, const XFoam_Tensor<Cmpt>& b)
{
	a -= b;
	return a;
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator-(const XFoam_Tensor<Cmpt>& a)
{
	XFoam_Tensor<Cmpt> r(XFoam_Zero_v);
	for (int i = 0; i < 9; ++i)
	{
		r.v_[static_cast<XFoam_Size>(i)] = static_cast<Cmpt>(-a.v_[static_cast<XFoam_Size>(i)]);
	}
	return r;
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator*(XFoam_Tensor<Cmpt> a, const Cmpt& s)
{
	a *= s;
	return a;
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator*(const Cmpt& s, XFoam_Tensor<Cmpt> a)
{
	a *= s;
	return a;
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator/(XFoam_Tensor<Cmpt> a, const Cmpt& s)
{
	a /= s;
	return a;
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator&(const XFoam_Tensor<Cmpt>& t1, const XFoam_Tensor<Cmpt>& t2)
{
	return XFoam_Tensor<Cmpt>(
		static_cast<Cmpt>(t1.xx() * t2.xx() + t1.xy() * t2.yx() + t1.xz() * t2.zx()),
		static_cast<Cmpt>(t1.xx() * t2.xy() + t1.xy() * t2.yy() + t1.xz() * t2.zy()),
		static_cast<Cmpt>(t1.xx() * t2.xz() + t1.xy() * t2.yz() + t1.xz() * t2.zz()),
		static_cast<Cmpt>(t1.yx() * t2.xx() + t1.yy() * t2.yx() + t1.yz() * t2.zx()),
		static_cast<Cmpt>(t1.yx() * t2.xy() + t1.yy() * t2.yy() + t1.yz() * t2.zy()),
		static_cast<Cmpt>(t1.yx() * t2.xz() + t1.yy() * t2.yz() + t1.yz() * t2.zz()),
		static_cast<Cmpt>(t1.zx() * t2.xx() + t1.zy() * t2.yx() + t1.zz() * t2.zx()),
		static_cast<Cmpt>(t1.zx() * t2.xy() + t1.zy() * t2.yy() + t1.zz() * t2.zy()),
		static_cast<Cmpt>(t1.zx() * t2.xz() + t1.zy() * t2.yz() + t1.zz() * t2.zz()));
}

template<class Cmpt>
inline void XFoam_Tensor<Cmpt>::operator&=(const XFoam_Tensor<Cmpt>& t)
{
	*this = (*this) & t;
}

template<class Cmpt>
inline XFoam_Vector<Cmpt> operator&(const XFoam_Tensor<Cmpt>& t, const XFoam_Vector<Cmpt>& v)
{
	return XFoam_Vector<Cmpt>(
		static_cast<Cmpt>(t.xx() * v.x() + t.xy() * v.y() + t.xz() * v.z()),
		static_cast<Cmpt>(t.yx() * v.x() + t.yy() * v.y() + t.yz() * v.z()),
		static_cast<Cmpt>(t.zx() * v.x() + t.zy() * v.y() + t.zz() * v.z()));
}

template<class Cmpt>
inline XFoam_Vector<Cmpt> operator&(const XFoam_Vector<Cmpt>& v, const XFoam_Tensor<Cmpt>& t)
{
	return XFoam_Vector<Cmpt>(
		static_cast<Cmpt>(v.x() * t.xx() + v.y() * t.yx() + v.z() * t.zx()),
		static_cast<Cmpt>(v.x() * t.xy() + v.y() * t.yy() + v.z() * t.zy()),
		static_cast<Cmpt>(v.x() * t.xz() + v.y() * t.yz() + v.z() * t.zz()));
}

template<class Cmpt>
inline XFoam_Tensor<Cmpt> operator*(const XFoam_Vector<Cmpt>& v1, const XFoam_Vector<Cmpt>& v2)
{
	return XFoam_Tensor<Cmpt>(
		static_cast<Cmpt>(v1.x() * v2.x()),
		static_cast<Cmpt>(v1.x() * v2.y()),
		static_cast<Cmpt>(v1.x() * v2.z()),
		static_cast<Cmpt>(v1.y() * v2.x()),
		static_cast<Cmpt>(v1.y() * v2.y()),
		static_cast<Cmpt>(v1.y() * v2.z()),
		static_cast<Cmpt>(v1.z() * v2.x()),
		static_cast<Cmpt>(v1.z() * v2.y()),
		static_cast<Cmpt>(v1.z() * v2.z()));
}

template<class Cmpt>
inline XFoam_Vector<Cmpt> operator/(const XFoam_Vector<Cmpt>& v, const XFoam_Tensor<Cmpt>& t)
{
	return XFoam_inv(t) & v;
}

template<class Cmpt>
inline bool operator==(const XFoam_Tensor<Cmpt>& a, const XFoam_Tensor<Cmpt>& b)
{
	for (int i = 0; i < 9; ++i)
	{
		if (a.v_[static_cast<XFoam_Size>(i)] != b.v_[static_cast<XFoam_Size>(i)])
		{
			return false;
		}
	}
	return true;
}

template<class Cmpt>
inline bool operator!=(const XFoam_Tensor<Cmpt>& a, const XFoam_Tensor<Cmpt>& b)
{
	return !(a == b);
}

template<class Cmpt>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_Tensor<Cmpt>& t)
{
	os << '(' << t.xx() << ' ' << t.xy() << ' ' << t.xz() << ' ' << t.yx() << ' ' << t.yy() << ' '
		<< t.yz() << ' ' << t.zx() << ' ' << t.zy() << ' ' << t.zz() << ')';
	return os;
}

#endif
