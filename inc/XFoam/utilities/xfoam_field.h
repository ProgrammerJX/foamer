#ifndef XFoam_Field_H_
#define XFoam_Field_H_

// XFoam_Field / XFoam_SubField：对齐 OpenFOAM Foam::Field、Foam::SubField（Field/Field.H、SubField/SubField.H）。
// XFoam_Field 继承顺序与 OpenFOAM 一致：tmp<Field>::refCount、List<Type>；SubField 仍继承 refCount + UList（见 SubField.H）。
//标量/向量场运算：对齐 OpenFOAM fields/Fields/Field/FieldFunctions.H（元素级与归约），全局函数均加 XFoam_ 前缀。

#include "XFoam/utilities/xfoam_list.h"
#include "XFoam/utilities/xfoam_vector.h"

template<class Type>
class XFoam_SubField;

template<class Type>
class XFoam_Field
	: public XFoam_RefCount
	, public XFoam_List<Type>
{
public:
	typedef Type value_type;

	typedef typename XFoam_pTraits<Type>::cmptType cmptType;

	//- SubField 类型（对标 Field::subField）
	typedef XFoam_SubField<Type> subField;

//- 类名字符串（对标 Field::typeName，OF 为 "Field"）
	static constexpr const char* typeName = "XFoam_Field";

	using XFoam_List<Type>::XFoam_List;

	XFoam_Field()
		: XFoam_RefCount()
		, XFoam_List<Type>()
	{
	}

	//- 对标 Field(const UList<Type>&)
	explicit XFoam_Field(const XFoam_UList<Type>& lst)
		: XFoam_RefCount()
		, XFoam_List<Type>(lst.size())
	{
		for (XFoam_Label i = 0; i < lst.size(); ++i)
		{
			(*this)[i] = lst[i];
		}
	}

	//- 对标 Field(List<Type>&&)
	explicit XFoam_Field(XFoam_List<Type>&& lst)
		: XFoam_RefCount()
		, XFoam_List<Type>(XFoam_move(lst))
	{
	}

	XFoam_Field(const XFoam_Field& other)
		: XFoam_RefCount()
		, XFoam_List<Type>(static_cast<const XFoam_List<Type>&>(other))
	{
	}

	XFoam_Field& operator=(const XFoam_Field& other)
	{
		if (this != &other)
		{
			XFoam_List<Type>::operator=(other);
		}
		return *this;
	}

	XFoam_Field(XFoam_Field&& other) noexcept
		: XFoam_RefCount()
		, XFoam_List<Type>(XFoam_move(other))
	{
	}

	XFoam_Field& operator=(XFoam_Field&& other) noexcept
	{
		if (this != &other)
		{
			XFoam_List<Type>::operator=(XFoam_move(other));
		}
		return *this;
	}

	//- 对标 Field::null()
	static const XFoam_Field<Type>& null()
	{
		static const XFoam_Field<Type> nullInst;
		return nullInst;
	}

	//- 对标 Field::clone() 返回 tmp<Field>
	XFoam_Tmp<XFoam_Field<Type>> clone() const
	{
		return XFoam_Tmp<XFoam_Field<Type>>(new XFoam_Field<Type>(*this));
	}
};

//对标 OpenFOAM SubList：UList 上的非拥有连续子区间 [startIndex, startIndex + subSize)。
template<class T>
class XFoam_SubList
	: public XFoam_UList<T>
{
public:
	XFoam_SubList()
		: XFoam_UList<T>()
	{
	}

	XFoam_SubList(const XFoam_SubList<T>& sl)
		: XFoam_UList<T>()
	{
		this->shallowCopy(static_cast<const XFoam_UList<T>&>(sl));
	}

	explicit XFoam_SubList(const XFoam_UList<T>& list)
		: XFoam_SubList(list, list.size(), 0)
	{
	}

	XFoam_SubList(const XFoam_UList<T>& list, XFoam_Label subSize)
		: XFoam_SubList(list, subSize, 0)
	{
	}

	XFoam_SubList(const XFoam_UList<T>& list, XFoam_Label subSize, XFoam_Label startIndex)
		: XFoam_UList<T>()
	{
		if (subSize < 0 || startIndex < 0 || startIndex + subSize > list.size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_SubList: length or start out of range"));
		}
		this->shallowCopy(XFoam_UList<T>(
			const_cast<T*>(list.data()) + startIndex, subSize));
	}

	void operator=(const XFoam_SubList<T>& rhs)
	{
		this->operator=(static_cast<const XFoam_UList<T>&>(rhs));
	}

	void operator=(const XFoam_UList<T>& rhs)
	{
		if (rhs.size() != this->size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_SubList::operator=: size mismatch"));
		}
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] = rhs[i];
		}
	}

	void operator=(const T& rhs)
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] = rhs;
		}
	}
};

//对标 SubField.H：tmp<SubField>::refCount + SubList<Type>。
template<class Type>
class XFoam_SubField
	: public XFoam_RefCount
	, public XFoam_SubList<Type>
{
public:
	typedef typename XFoam_Field<Type>::cmptType cmptType;

	XFoam_SubField(const XFoam_SubList<Type>& list)
		: XFoam_RefCount()
		, XFoam_SubList<Type>(list)
	{
	}

	explicit XFoam_SubField(const XFoam_UList<Type>& list)
		: XFoam_RefCount()
		, XFoam_SubList<Type>(list)
	{
	}

	XFoam_SubField(const XFoam_UList<Type>& list, XFoam_Label subSize)
		: XFoam_RefCount()
		, XFoam_SubList<Type>(list, subSize)
	{
	}

	XFoam_SubField(
		const XFoam_UList<Type>& list,
		XFoam_Label subSize,
		XFoam_Label startIndex)
		: XFoam_RefCount()
		, XFoam_SubList<Type>(list, subSize, startIndex)
	{
	}

	XFoam_SubField(const XFoam_SubField& sfield)
		: XFoam_RefCount()
		, XFoam_SubList<Type>(static_cast<const XFoam_SubList<Type>&>(sfield))
	{
	}

	XFoam_SubField(XFoam_SubField& sfield, bool /*reuse*/)
		: XFoam_RefCount()
		, XFoam_SubList<Type>(static_cast<const XFoam_SubList<Type>&>(sfield))
	{
	}

	void operator=(const XFoam_SubField<Type>& rhs)
	{
		XFoam_SubList<Type>::operator=(rhs);
	}

	void operator=(const XFoam_UList<Type>& rhs)
	{
		XFoam_SubList<Type>::operator=(rhs);
	}

	void operator=(const XFoam_Tmp<XFoam_Field<Type>>& rhs)
	{
		XFoam_SubList<Type>::operator=(rhs());
		rhs.clear();
	}

	void operator=(const Type& rhs)
	{
		XFoam_SubList<Type>::operator=(rhs);
	}

	void operator=(XFoam_Zero_Tag)
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] = Type{};
		}
	}

	void operator+=(const XFoam_UList<Type>& rhs)
	{
		if (rhs.size() != this->size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_SubField::operator+=: size mismatch"));
		}
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] += rhs[i];
		}
	}

	void operator+=(const XFoam_Tmp<XFoam_Field<Type>>& tf)
	{
		this->operator+=(static_cast<const XFoam_UList<Type>&>(tf()));
		tf.clear();
	}

	void operator-=(const XFoam_UList<Type>& rhs)
	{
		if (rhs.size() != this->size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_SubField::operator-=: size mismatch"));
		}
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] -= rhs[i];
		}
	}

	void operator-=(const XFoam_Tmp<XFoam_Field<Type>>& tf)
	{
		this->operator-=(static_cast<const XFoam_UList<Type>&>(tf()));
		tf.clear();
	}

	void operator*=(const XFoam_UList<XFoam_Scalar>& rhs)
	{
		if (rhs.size() != this->size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_SubField::operator*=: size mismatch"));
		}
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] *= rhs[i];
		}
	}

	void operator*=(const XFoam_Tmp<XFoam_Field<XFoam_Scalar>>& tf)
	{
		this->operator*=(static_cast<const XFoam_UList<XFoam_Scalar>&>(tf()));
		tf.clear();
	}

	void operator/=(const XFoam_UList<XFoam_Scalar>& rhs)
	{
		if (rhs.size() != this->size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_SubField::operator/=: size mismatch"));
		}
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] /= rhs[i];
		}
	}

	void operator/=(const XFoam_Tmp<XFoam_Field<XFoam_Scalar>>& tf)
	{
		this->operator/=(static_cast<const XFoam_UList<XFoam_Scalar>&>(tf()));
		tf.clear();
	}

	void operator+=(const Type& rhs)
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] += rhs;
		}
	}

	void operator-=(const Type& rhs)
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] -= rhs;
		}
	}

	void operator*=(const XFoam_Scalar& rhs)
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] *= rhs;
		}
	}

	void operator/=(const XFoam_Scalar& rhs)
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			(*this)[i] /= rhs;
		}
	}

	static const XFoam_SubField<Type>& null()
	{
		static const XFoam_SubField<Type> nullRef(
			static_cast<const XFoam_UList<Type>&>(XFoam_UList<Type>::null()));
		return nullRef;
	}

	XFoam_Tmp<XFoam_SubField<Type>> clone() const
	{
		return XFoam_Tmp<XFoam_SubField<Type>>(new XFoam_SubField<Type>(*this));
	}
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// 对标 OpenFOAM FieldFunctions：直接以全局 XFoam_* / 运算符实现（无中间 struct/命名空间）。

template<class Type>
inline Type XFoam_sum(const XFoam_UList<Type>& f)
{
	Type s = Type();
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		s += f[i];
	}
	return s;
}

template<class Type>
inline Type XFoam_max(const XFoam_UList<Type>& f)
{
	if (f.empty())
	{
		throw XFoam_Error(XFoam_String("XFoam_max: empty field"));
	}
	Type m = f[0];
	for (XFoam_Label i = 1; i < f.size(); ++i)
	{
		if (f[i] > m)
		{
			m = f[i];
		}
	}
	return m;
}

template<class Type>
inline Type XFoam_min(const XFoam_UList<Type>& f)
{
	if (f.empty())
	{
		throw XFoam_Error(XFoam_String("XFoam_min: empty field"));
	}
	Type m = f[0];
	for (XFoam_Label i = 1; i < f.size(); ++i)
	{
		if (f[i] < m)
		{
			m = f[i];
		}
	}
	return m;
}

template<class Type>
inline Type XFoam_average(const XFoam_UList<Type>& f)
{
	if (f.empty())
	{
		throw XFoam_Error(XFoam_String("XFoam_average: empty field"));
	}
	return XFoam_sum(f) / static_cast<XFoam_Scalar>(f.size());
}

inline XFoam_Scalar XFoam_sumMag(const XFoam_UList<XFoam_Scalar>& f)
{
	XFoam_Scalar s = 0;
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		s += std::fabs(f[i]);
	}
	return s;
}

inline XFoam_Scalar XFoam_maxMag(const XFoam_UList<XFoam_Scalar>& f)
{
	if (f.empty())
	{
		throw XFoam_Error(XFoam_String("XFoam_maxMag: empty field"));
	}
	XFoam_Scalar m = std::fabs(f[0]);
	for (XFoam_Label i = 1; i < f.size(); ++i)
	{
		const XFoam_Scalar mm = std::fabs(f[i]);
		if (mm > m)
		{
			m = mm;
		}
	}
	return m;
}

inline XFoam_Scalar XFoam_sumMag(const XFoam_UList<XFoam_Vector3D>& f)
{
	XFoam_Scalar s = 0;
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		s += static_cast<XFoam_Scalar>(f[i].mag());
	}
	return s;
}

inline XFoam_Scalar XFoam_maxMag(const XFoam_UList<XFoam_Vector3D>& f)
{
	if (f.empty())
	{
		throw XFoam_Error(XFoam_String("XFoam_maxMag: empty field"));
	}
	XFoam_Scalar m = static_cast<XFoam_Scalar>(f[0].mag());
	for (XFoam_Label i = 1; i < f.size(); ++i)
	{
		const XFoam_Scalar mm = static_cast<XFoam_Scalar>(f[i].mag());
		if (mm > m)
		{
			m = mm;
		}
	}
	return m;
}

inline XFoam_Field<XFoam_Scalar> XFoam_mag(const XFoam_UList<XFoam_Scalar>& f)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = std::fabs(f[i]);
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> XFoam_magSqr(const XFoam_UList<XFoam_Scalar>& f)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = f[i] * f[i];
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> XFoam_mag(const XFoam_UList<XFoam_Vector3D>& f)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = static_cast<XFoam_Scalar>(f[i].mag());
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> XFoam_magSqr(const XFoam_UList<XFoam_Vector3D>& f)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = static_cast<XFoam_Scalar>(f[i].magSqr());
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> XFoam_neg(const XFoam_UList<Type>& f)
{
	XFoam_Field<Type> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = -f[i];
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> operator-(const XFoam_UList<Type>& f)
{
	return XFoam_neg(f);
}

inline XFoam_Field<XFoam_Scalar> XFoam_sqr(const XFoam_UList<XFoam_Scalar>& f)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = f[i] * f[i];
	}
	return r;
}

inline XFoam_Field<XFoam_Vector3D> XFoam_sqr(const XFoam_UList<XFoam_Vector3D>& f)
{
	XFoam_Field<XFoam_Vector3D> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		const XFoam_Vector3D& v = f[i];
		// 显式分量平方，避免 vector×vector 与 tensor.h 中外积 operator* 二义（见 xfoam_common.h 全量包含顺序）。
		r[i] = XFoam_Vector3D(v.x() * v.x(), v.y() * v.y(), v.z() * v.z());
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> XFoam_sqrt(const XFoam_UList<XFoam_Scalar>& f)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = static_cast<XFoam_Scalar>(std::sqrt(static_cast<double>(f[i])));
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> XFoam_pow(
	const XFoam_UList<XFoam_Scalar>& f,
	const XFoam_Scalar& e)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		r[i] = static_cast<XFoam_Scalar>(
			std::pow(static_cast<double>(f[i]), static_cast<double>(e)));
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> XFoam_pow3(const XFoam_UList<XFoam_Scalar>& f)
{
	XFoam_Field<XFoam_Scalar> r(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		const XFoam_Scalar x = f[i];
		r[i] = x * x * x;
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> XFoam_pow4(const XFoam_UList<XFoam_Scalar>& f)
{
	return XFoam_sqr(XFoam_sqr(f));
}

template<class Type>
inline XFoam_Field<Type> XFoam_add(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_add: size mismatch"));
	}
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] + b[i];
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> XFoam_subtract(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_subtract: size mismatch"));
	}
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] - b[i];
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> XFoam_multiply(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_multiply: size mismatch"));
	}
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] * b[i];
	}
	return r;
}

template<>
inline XFoam_Field<XFoam_Vector3D> XFoam_multiply(
	const XFoam_UList<XFoam_Vector3D>& a,
	const XFoam_UList<XFoam_Vector3D>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_multiply: size mismatch"));
	}
	XFoam_Field<XFoam_Vector3D> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		const XFoam_Vector3D& u = a[i];
		const XFoam_Vector3D& v = b[i];
		r[i] = XFoam_Vector3D(u.x() * v.x(), u.y() * v.y(), u.z() * v.z());
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> XFoam_divide(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_divide: size mismatch"));
	}
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] / b[i];
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> operator+(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	return XFoam_add(a, b);
}

template<class Type>
inline XFoam_Field<Type> operator-(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	return XFoam_subtract(a, b);
}

template<class Type>
inline XFoam_Field<Type> operator*(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	return XFoam_multiply(a, b);
}

template<class Type>
inline XFoam_Field<Type> operator/(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	return XFoam_divide(a, b);
}

template<class Type>
inline XFoam_Field<Type> operator+(
	const XFoam_UList<Type>& a,
	const Type& s)
{
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] + s;
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> operator+(
	const Type& s,
	const XFoam_UList<Type>& a)
{
	return a + s;
}

template<class Type>
inline XFoam_Field<Type> operator-(
	const XFoam_UList<Type>& a,
	const Type& s)
{
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] - s;
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> operator-(
	const Type& s,
	const XFoam_UList<Type>& a)
{
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = s - a[i];
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> operator*(
	const XFoam_UList<Type>& a,
	const Type& s)
{
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] * s;
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> operator*(
	const Type& s,
	const XFoam_UList<Type>& a)
{
	return a * s;
}

template<class Type>
inline XFoam_Field<Type> operator/(
	const XFoam_UList<Type>& a,
	const Type& s)
{
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] / s;
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> operator/(
	const Type& s,
	const XFoam_UList<Type>& a)
{
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = s / a[i];
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> XFoam_min(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_min: size mismatch"));
	}
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = (b[i] < a[i]) ? b[i] : a[i];
	}
	return r;
}

template<class Type>
inline XFoam_Field<Type> XFoam_max(
	const XFoam_UList<Type>& a,
	const XFoam_UList<Type>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_max: size mismatch"));
	}
	XFoam_Field<Type> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = (a[i] > b[i]) ? a[i] : b[i];
	}
	return r;
}

inline XFoam_Field<XFoam_Vector3D> operator^(
	const XFoam_UList<XFoam_Vector3D>& a,
	const XFoam_UList<XFoam_Vector3D>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("operator^: size mismatch"));
	}
	XFoam_Field<XFoam_Vector3D> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = a[i] ^ b[i];
	}
	return r;
}

inline XFoam_Field<XFoam_Scalar> operator&(
	const XFoam_UList<XFoam_Vector3D>& a,
	const XFoam_UList<XFoam_Vector3D>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("operator&: size mismatch"));
	}
	XFoam_Field<XFoam_Scalar> r(a.size());
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		r[i] = static_cast<XFoam_Scalar>(a[i] & b[i]);
	}
	return r;
}

inline XFoam_Scalar XFoam_sumProd(
	const XFoam_UList<XFoam_Vector3D>& a,
	const XFoam_UList<XFoam_Vector3D>& b)
{
	if (a.size() != b.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_sumProd: size mismatch"));
	}
	XFoam_Scalar s = 0;
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		s += static_cast<XFoam_Scalar>(a[i] & b[i]);
	}
	return s;
}

inline void XFoam_component(
	XFoam_Field<XFoam_Scalar>& res,
	const XFoam_UList<XFoam_Vector3D>& f,
	const XFoam_Label cmpt)
{
	res.setSize(f.size());
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		res[i] = f[i][cmpt];
	}
}

inline void XFoam_replaceComponent(
	XFoam_Field<XFoam_Vector3D>& f,
	const XFoam_UList<XFoam_Scalar>& comp,
	const XFoam_Label cmpt)
{
	if (f.size() != comp.size())
	{
		throw XFoam_Error(XFoam_String("XFoam_replaceComponent: size mismatch"));
	}
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		f[i][cmpt] = comp[i];
	}
}

typedef XFoam_Field<XFoam_Label> XFoam_LabelField;
typedef XFoam_Field<XFoam_Scalar> XFoam_ScalarField;
typedef XFoam_Field<XFoam_Vector3D> XFoam_VectorField;
typedef XFoam_VectorField XFoam_PointField;
typedef XFoam_UList<XFoam_Vector3D> XFoam_Vector3DUList;
typedef XFoam_List<XFoam_Vector3D> XFoam_Vector3DList;

#endif
