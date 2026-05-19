#ifndef XFoam_AutoPtr_H_
#define XFoam_AutoPtr_H_

// XFoam_AutoPtr<T>: 仿照 OpenFOAM Foam::autoPtr（tmp/.../memory/autoPtr/autoPtr.H + autoPtrI.H）
// - 与 std::unique_ptr 不同：可从 const autoPtr& 转移所有权（源指针置空），与 OF 一致
// - 未分配时 operator()/operator* 等走 XFoam_FatalError 并 XFoam_abort（与 OF FatalError 一致）
// XFoam_RefCount: 仿照 OpenFOAM Foam::refCount（memory/refCount/refCount.H），供 tmp 等复用。
// XFoam_Tmp<T>: 仿照 OpenFOAM Foam::tmp（memory/tmp/tmp.H + tmpI.H）；T 须公有继承 XFoam_RefCount。
// 不在类体对 T 做 static_assert：MSVC 在 PtrList 等 SFINAE 场景仍会实例化 XFoam_Tmp<T>（如 T 无 RefCount）。
// 标准库由 xfoam_types.h 统一引入。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_error.h"

/// 引用计数基类，接口与 Foam::refCount 一致（含 unique() 表示 count==0 的语义）。
class XFoam_RefCount
{
	int count_;

protected:
	XFoam_RefCount()
		: count_(0)
	{
	}

	XFoam_RefCount(const XFoam_RefCount&) = delete;

public:
	int count() const { return count_; }

	bool unique() const { return count_ == 0; }

	void operator++() { ++count_; }

	void operator++(int) { ++count_; }

	void operator--() { --count_; }

	void operator--(int) { --count_; }

	void operator=(const XFoam_RefCount&) = delete;
};

template<class T>
class XFoam_AutoPtr
{
	mutable T* ptr_;

public:
	typedef T Type;

	explicit XFoam_AutoPtr(T* p = nullptr)
		: ptr_(p)
	{
	}

	XFoam_AutoPtr(const XFoam_AutoPtr<T>& ap)
		: ptr_(ap.ptr_)
	{
		ap.ptr_ = nullptr;
	}

	// reuse == true: 转移指针；reuse == false: 调用 ap()->clone() 复制（要求 T 提供 clone）
	XFoam_AutoPtr(const XFoam_AutoPtr<T>& ap, bool reuse)
		: ptr_(nullptr)
	{
		if (reuse)
		{
			ptr_ = ap.ptr_;
			ap.ptr_ = nullptr;
		}
		else if (ap.valid())
		{
			ptr_ = cloneOrThrow_(
				ap,
				std::integral_constant<bool, HasMemberClone_::value>());
		}
		else
		{
			ptr_ = nullptr;
		}
	}

	XFoam_AutoPtr(XFoam_AutoPtr<T>&& ap) noexcept
		: ptr_(ap.ptr_)
	{
		ap.ptr_ = nullptr;
	}

	~XFoam_AutoPtr()
	{
		clear();
	}

	bool empty() const { return !ptr_; }
	bool valid() const { return ptr_ != nullptr; }

	T* ptr()
	{
		T* p = ptr_;
		ptr_ = nullptr;
		return p;
	}

	void set(T* p)
	{
		if (ptr_)
		{
			XFoam_FatalErrorIn("XFoam_AutoPtr::set")
				<< " (type " << typeid(T).name() << " already allocated)"
				<< XFoam_abort(XFoam_FatalError);
		}
		ptr_ = p;
	}

	void reset(T* p = nullptr)
	{
		if (ptr_)
		{
			delete ptr_;
		}
		ptr_ = p;
	}

	void clear()
	{
		reset(nullptr);
	}

	T& operator()()
	{
		if (!ptr_)
		{
			XFoam_FatalErrorIn("XFoam_AutoPtr::operator()")
				<< " (type " << typeid(T).name() << " is not allocated)"
				<< XFoam_abort(XFoam_FatalError);
		}
		return *ptr_;
	}

	const T& operator()() const
	{
		if (!ptr_)
		{
			XFoam_FatalErrorIn("XFoam_AutoPtr::operator() const")
				<< " (type " << typeid(T).name() << " is not allocated)"
				<< XFoam_abort(XFoam_FatalError);
		}
		return *ptr_;
	}

	T& operator*()
	{
		if (!ptr_)
		{
			XFoam_FatalErrorIn("XFoam_AutoPtr::operator*")
				<< " (type " << typeid(T).name() << " is not allocated)"
				<< XFoam_abort(XFoam_FatalError);
		}
		return *ptr_;
	}

	const T& operator*() const
	{
		if (!ptr_)
		{
			XFoam_FatalErrorIn("XFoam_AutoPtr::operator* const")
				<< " (type " << typeid(T).name() << " is not allocated)"
				<< XFoam_abort(XFoam_FatalError);
		}
		return *ptr_;
	}

	operator const T&() const
	{
		return operator()();
	}

	T* operator->()
	{
		if (!ptr_)
		{
			XFoam_FatalErrorIn("XFoam_AutoPtr::operator->")
				<< " (type " << typeid(T).name() << " is not allocated)"
				<< XFoam_abort(XFoam_FatalError);
		}
		return ptr_;
	}

	const T* operator->() const
	{
		return const_cast<XFoam_AutoPtr<T>*>(this)->operator->();
	}

	void operator=(T* p)
	{
		reset(p);
	}

	void operator=(const XFoam_AutoPtr<T>& ap)
	{
		if (this != &ap)
		{
			reset(const_cast<XFoam_AutoPtr<T>&>(ap).ptr());
		}
	}

	XFoam_AutoPtr& operator=(XFoam_AutoPtr<T>&& ap) noexcept
	{
		if (this != &ap)
		{
			reset(ap.ptr_);
			ap.ptr_ = nullptr;
		}
		return *this;
	}

private:
	// reuse==false 且 T 无 clone() 时不可编译 ap().clone()，用 SFINAE 分派。
	struct HasMemberClone_
	{
		template<class U, class V = decltype(std::declval<const U&>().clone())>
		static std::true_type test(int);
		template<class U>
		static std::false_type test(...);
		static const bool value =
			std::is_same<decltype(test<T>(0)), std::true_type>::value;
	};

	static T* cloneOrThrow_(const XFoam_AutoPtr<T>& ap, std::true_type)
	{
		return ap().clone().ptr();
	}

	static T* cloneOrThrow_(const XFoam_AutoPtr<T>& ap, std::false_type)
	{
		(void)ap;
		throw XFoam_Error(
			XFoam_String("XFoam_AutoPtr: copy with reuse=false requires T::clone()"));
	}
};

template<class T>
class XFoam_Tmp
{
	// 不在类体用 static_assert(std::is_base_of<...>)：MSVC 在 PtrList 等 SFINAE 重载仍会实例化
	// XFoam_Tmp<T>（如 T=XFoam_Block），此时若 T 非 RefCount 会误触发。语义仍要求 T 公有派生
	// XFoam_RefCount；仅应对 Field 等合法 T 使用 XFoam_Tmp（见 xfoam_dictionary.h 注释）。

public:
	enum class Kind
	{
		Reusable,
		NonReusable,
		ConstRef
	};

private:
	Kind kind_;
	mutable T* ptr_;

	bool isAnyTmp() const
	{
		return kind_ == Kind::Reusable || kind_ == Kind::NonReusable;
	}

	void incrRef_() const
	{
		ptr_->operator++();
		if (ptr_->count() > 1)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp: more than 2 tmp referring to same object of type ")
				+ typeName());
		}
	}

	static XFoam_String typeNameFor_()
	{
		return XFoam_String("tmp<") + typeid(T).name() + ">";
	}

public:
	typedef T Type;
	typedef XFoam_RefCount refCount;

	explicit XFoam_Tmp(T* tPtr = nullptr, bool nonReusable = false)
		: kind_(nonReusable ? Kind::NonReusable : Kind::Reusable)
		, ptr_(tPtr)
	{
		if (tPtr && !tPtr->unique())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp: construction from non-unique pointer for ")
				+ typeNameFor_());
		}
	}

	XFoam_Tmp(const T& tRef)
		: kind_(Kind::ConstRef)
		, ptr_(const_cast<T*>(&tRef))
	{
	}

	XFoam_Tmp(const XFoam_Tmp<T>& t)
		: kind_(t.kind_)
		, ptr_(t.ptr_)
	{
		if (isAnyTmp())
		{
			if (ptr_)
			{
				incrRef_();
			}
			else
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp: copy of deallocated ") + typeNameFor_());
			}
		}
	}

	XFoam_Tmp(XFoam_Tmp<T>&& t) noexcept
		: kind_(t.kind_)
		, ptr_(t.ptr_)
	{
		if (isAnyTmp())
		{
			t.ptr_ = nullptr;
		}
	}

	XFoam_Tmp(const XFoam_Tmp<T>&& t) noexcept
		: kind_(t.kind_)
		, ptr_(t.ptr_)
	{
		if (isAnyTmp())
		{
			t.ptr_ = nullptr;
		}
	}

	XFoam_Tmp(const XFoam_Tmp<T>& t, bool allowTransfer)
		: kind_(t.kind_)
		, ptr_(t.ptr_)
	{
		if (isAnyTmp())
		{
			if (ptr_)
			{
				if (allowTransfer && kind_ == Kind::Reusable)
				{
					t.ptr_ = nullptr;
				}
				else
				{
					incrRef_();
				}
			}
			else
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp: copy of deallocated ") + typeNameFor_());
			}
		}
	}

	~XFoam_Tmp() { clear(); }

	bool isTmp() const { return kind_ == Kind::Reusable; }

	bool empty() const { return isAnyTmp() && !ptr_; }

	bool valid() const { return !isAnyTmp() || (isAnyTmp() && ptr_); }

	XFoam_String typeName() const { return typeNameFor_(); }

	T& ref() const
	{
		if (isAnyTmp())
		{
			if (!ptr_)
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp::ref: deallocated ") + typeNameFor_());
			}
		}
		else
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp::ref: non-const access from const-reference ")
				+ typeNameFor_());
		}
		return *ptr_;
	}

	T* ptr() const
	{
		if (isTmp())
		{
			if (!ptr_)
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp::ptr: deallocated ") + typeNameFor_());
			}
			if (!ptr_->unique())
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp::ptr: object referred to by multiple ")
					+ typeNameFor_());
			}
			T* p = ptr_;
			ptr_ = nullptr;
			return p;
		}
		return ptr_->clone().ptr();
	}

	void clear() const
	{
		if (isAnyTmp() && ptr_)
		{
			if (ptr_->unique())
			{
				delete ptr_;
				ptr_ = nullptr;
			}
			else
			{
				ptr_->operator--();
				ptr_ = nullptr;
			}
		}
	}

	const T& operator()() const
	{
		if (isAnyTmp())
		{
			if (!ptr_)
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp::operator(): deallocated ") + typeNameFor_());
			}
		}
		return *ptr_;
	}

	operator const T&() const { return operator()(); }

	T* operator->()
	{
		if (isAnyTmp())
		{
			if (!ptr_)
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp::operator->: deallocated ") + typeNameFor_());
			}
		}
		else
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp::operator->: const object for ") + typeNameFor_());
		}
		return ptr_;
	}

	const T* operator->() const
	{
		if (isAnyTmp() && !ptr_)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp::operator-> const: deallocated ") + typeNameFor_());
		}
		return ptr_;
	}

	void operator=(T* tPtr)
	{
		clear();
		if (!tPtr)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp::operator=: null pointer for ") + typeNameFor_());
		}
		if (!tPtr->unique())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp::operator=: non-unique pointer for ")
				+ typeNameFor_());
		}
		kind_ = Kind::Reusable;
		ptr_ = tPtr;
	}

	void operator=(const XFoam_Tmp<T>& t)
	{
		clear();
		if (t.isAnyTmp())
		{
			if (!t.ptr_)
			{
				throw XFoam_Error(
					XFoam_String("XFoam_Tmp::operator=: deallocated rhs ") + typeNameFor_());
			}
			kind_ = t.kind_;
			ptr_ = t.ptr_;
			t.ptr_ = nullptr;
		}
		else
		{
			throw XFoam_Error(
				XFoam_String("XFoam_Tmp::operator=: cannot assign from const-ref tmp to type ")
				+ typeid(T).name());
		}
	}

	void operator=(XFoam_Tmp<T>&& t)
	{
		clear();
		kind_ = t.kind_;
		ptr_ = t.ptr_;
		if (isAnyTmp())
		{
			t.ptr_ = nullptr;
		}
	}

	void operator=(const XFoam_Tmp<T>&& t)
	{
		clear();
		kind_ = t.kind_;
		ptr_ = t.ptr_;
		if (isAnyTmp())
		{
			t.ptr_ = nullptr;
		}
	}
};

#endif
