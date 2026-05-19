#ifndef XFoam_List_H_
#define XFoam_List_H_

// XFoam_List<T>: 仿照 OpenFOAM Foam::List（见 tmp/.../List/List.H），公有派生自 XFoam_UList<T>。
// 底层用 std::vector 拥有元素；通过 shallowCopy 将 UList 视图指向 vector 连续存储。
// 标准库头文件统一由 xfoam_types.h 引入。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_autoptr.h"
#include "XFoam/utilities/xfoam_stream.h"
#include "XFoam/utilities/xfoam_hash.h"

class XFoam_IStream;
class XFoam_OStream;

template<class T>
class XFoam_List;

template<class T, unsigned SizeInc, unsigned SizeMult, unsigned SizeDiv>
class XFoam_DynamicList;

// XFoam_UList<T>: 仿照 OpenFOAM Foam::UList（tmp/.../UList/UList.H）
// 非拥有视图：仅保存指针与长度，析构不释放存储；赋值 UList 被删除，用 shallowCopy / deepCopy。
template<class T>
class XFoam_UList
{
private:
	XFoam_Label size_;
	T* v_;

public:
	friend class XFoam_List<T>;

	typedef T value_type;
	typedef T& reference;
	typedef const T& const_reference;
	typedef XFoam_Label difference_type;
	typedef XFoam_Label size_type;
	typedef T* iterator;
	typedef const T* const_iterator;
	typedef std::reverse_iterator<iterator> reverse_iterator;
	typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

	class less
	{
		const XFoam_UList<T>& values_;

	public:
		explicit less(const XFoam_UList<T>& values)
			: values_(values)
		{
		}

		bool operator()(XFoam_Label a, XFoam_Label b) const
		{
			return values_[a] < values_[b];
		}
	};

	class greater
	{
		const XFoam_UList<T>& values_;

	public:
		explicit greater(const XFoam_UList<T>& values)
			: values_(values)
		{
		}

		bool operator()(XFoam_Label a, XFoam_Label b) const
		{
			return values_[a] > values_[b];
		}
	};

	void operator=(const XFoam_UList<T>&) = delete;

	static const XFoam_UList<T>& null()
	{
		static const XFoam_UList<T> nullRef(nullptr, 0);
		return nullRef;
	}

	XFoam_UList()
		: size_(0)
		, v_(nullptr)
	{
	}

	XFoam_UList(T* v, XFoam_Label n)
		: size_(n)
		, v_(v)
	{
		if (n < 0)
		{
			throw XFoam_Error(XFoam_String("XFoam_UList: negative size"));
		}
	}

	XFoam_UList(const XFoam_UList<T>&) = default;

	XFoam_Label size() const { return size_; }

	XFoam_Label max_size() const
	{
		return std::numeric_limits<XFoam_Label>::max() / 2;
	}

	bool empty() const { return size_ == 0; }

	XFoam_Label fcIndex(XFoam_Label i) const
	{
		return (i == size() - 1 ? 0 : i + 1);
	}

	XFoam_Label rcIndex(XFoam_Label i) const
	{
		return (i ? i - 1 : size() - 1);
	}

	void checkStart(XFoam_Label start) const
	{
		if (start < 0 || (start != 0 && start >= size_))
		{
			throw XFoam_Error(
				XFoam_String("XFoam_UList::checkStart: start out of range"));
		}
	}

	void checkSize(XFoam_Label n) const
	{
		if (n < 0 || n > size_)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_UList::checkSize: size out of range"));
		}
	}

	void checkIndex(XFoam_Label i) const
	{
		if (size_ == 0)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_UList::checkIndex: zero-sized list"));
		}
		if (i < 0 || i >= size_)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_UList::checkIndex: index out of range"));
		}
	}

	const T* cdata() const { return v_; }

	T* data() { return v_; }

	const T* data() const { return v_; }

	T& first() { return (*this)[0]; }

	const T& first() const { return (*this)[0]; }

	T& last() { return (*this)[size() - 1]; }

	const T& last() const { return (*this)[size() - 1]; }

	void shallowCopy(const XFoam_UList<T>& a)
	{
		size_ = a.size_;
		v_ = a.v_;
	}

	void deepCopy(const XFoam_UList<T>& a)
	{
		if (a.size_ != size_)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_UList::deepCopy: size mismatch"));
		}
		for (XFoam_Label i = 0; i < size_; ++i)
		{
			v_[i] = a.v_[i];
		}
	}

	T& operator[](XFoam_Label i) { return v_[i]; }

	const T& operator[](XFoam_Label i) const { return v_[i]; }

	void operator=(const T& t)
	{
		for (XFoam_Label i = 0; i < size_; ++i)
		{
			v_[i] = t;
		}
	}

	iterator begin() { return v_; }
	iterator end() { return v_ + size_; }

	const_iterator cbegin() const { return v_; }
	const_iterator cend() const { return v_ + size_; }

	const_iterator begin() const { return v_; }
	const_iterator end() const { return v_ + size_; }

	reverse_iterator rbegin() { return reverse_iterator(end()); }
	reverse_iterator rend() { return reverse_iterator(begin()); }

	const_reverse_iterator crbegin() const
	{
		return const_reverse_iterator(cend());
	}

	const_reverse_iterator crend() const
	{
		return const_reverse_iterator(cbegin());
	}

	const_reverse_iterator rbegin() const
	{
		return const_reverse_iterator(cend());
	}

	const_reverse_iterator rend() const
	{
		return const_reverse_iterator(cbegin());
	}
};

typedef XFoam_UList<XFoam_Label> XFoam_LabelUList;

// XFoam_FixedList<T, Size>: 仿照 OpenFOAM Foam::FixedList（tmp/.../FixedList/FixedList.H + FixedListI.H）
// - 定长 Size>0，栈上 T v_[Size]；无 Istream/Ostream、SLList、Foam::Hash 特化（可按需扩展）
template<class T, unsigned Size>
class XFoam_FixedList
{
	static_assert(
		Size
			&& Size
				<= static_cast<unsigned>(std::numeric_limits<int>::max()),
		"Size must be positive (non-zero) and fit in signed label range");

	T v_[Size];

public:
	static const XFoam_FixedList<T, Size>& null()
	{
		static const XFoam_FixedList<T, Size> nullRef;
		return nullRef;
	}

	XFoam_FixedList() = default;

	explicit XFoam_FixedList(const T& t)
	{
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = t;
		}
	}

	explicit XFoam_FixedList(const T (&src)[Size])
	{
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = src[i];
		}
	}

	template<class InputIterator>
	XFoam_FixedList(InputIterator first, InputIterator last)
	{
		checkSize(static_cast<XFoam_Label>(std::distance(first, last)));
		InputIterator iter = first;
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = *iter++;
		}
	}

	XFoam_FixedList(std::initializer_list<T> lst)
		: XFoam_FixedList(lst.begin(), lst.end())
	{
	}

	explicit XFoam_FixedList(const XFoam_UList<T>& lst)
	{
		checkSize(lst.size());
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = lst[static_cast<XFoam_Label>(i)];
		}
	}

	XFoam_FixedList(const XFoam_FixedList<T, Size>&) = default;

	XFoam_FixedList(XFoam_FixedList<T, Size>&&) noexcept = default;

	XFoam_AutoPtr<XFoam_FixedList<T, Size>> clone() const
	{
		return XFoam_AutoPtr<XFoam_FixedList<T, Size>>(
			new XFoam_FixedList<T, Size>(*this));
	}

	XFoam_Label fcIndex(XFoam_Label i) const
	{
		return (i == static_cast<XFoam_Label>(Size) - 1 ? 0 : i + 1);
	}

	XFoam_Label rcIndex(XFoam_Label i) const
	{
		return (i ? i - 1 : static_cast<XFoam_Label>(Size) - 1);
	}

	const T* cdata() const { return v_; }

	T* data() { return v_; }

	const T* data() const { return v_; }

	T& first() { return v_[0]; }

	const T& first() const { return v_[0]; }

	T& last() { return v_[Size - 1]; }

	const T& last() const { return v_[Size - 1]; }

	void checkStart(XFoam_Label start) const
	{
		if (start < 0
			|| (start != 0 && static_cast<unsigned>(start) >= Size))
		{
			throw XFoam_Error(
				XFoam_String("XFoam_FixedList::checkStart: start out of range"));
		}
	}

	void checkSize(XFoam_Label n) const
	{
		if (static_cast<unsigned>(n) != Size)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_FixedList::checkSize: size mismatch"));
		}
	}

	void checkIndex(XFoam_Label i) const
	{
		if (i < 0 || static_cast<unsigned>(i) >= Size)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_FixedList::checkIndex: index out of range"));
		}
	}

	void resize(XFoam_Label s) { (void)s; }

	void setSize(XFoam_Label s) { (void)s; }

	void transfer(const XFoam_FixedList<T, Size>& lst)
	{
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = lst[static_cast<XFoam_Label>(i)];
		}
	}

	T& operator[](XFoam_Label i) { return v_[i]; }

	const T& operator[](XFoam_Label i) const { return v_[i]; }

	void operator=(const T (&src)[Size])
	{
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = src[i];
		}
	}

	void operator=(const XFoam_UList<T>& lst)
	{
		checkSize(lst.size());
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = lst[static_cast<XFoam_Label>(i)];
		}
	}

	void operator=(std::initializer_list<T> lst)
	{
		checkSize(static_cast<XFoam_Label>(lst.size()));
		auto iter = lst.begin();
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = *iter++;
		}
	}

	void operator=(const T& t)
	{
		for (unsigned i = 0; i < Size; ++i)
		{
			v_[i] = t;
		}
	}

	XFoam_FixedList<T, Size>& operator=(const XFoam_FixedList<T, Size>& other)
	{
		if (this != &other)
		{
			for (unsigned i = 0; i < Size; ++i)
			{
				v_[i] = other.v_[i];
			}
		}
		return *this;
	}

	XFoam_FixedList<T, Size>& operator=(XFoam_FixedList<T, Size>&& other) noexcept
	{
		if (this != &other)
		{
			for (unsigned i = 0; i < Size; ++i)
			{
				v_[i] = XFoam_move(other.v_[i]);
			}
		}
		return *this;
	}

	typedef T value_type;
	typedef T& reference;
	typedef const T& const_reference;
	typedef XFoam_Label difference_type;
	typedef XFoam_Label size_type;
	typedef T* iterator;
	typedef const T* const_iterator;
	typedef std::reverse_iterator<iterator> reverse_iterator;
	typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

	iterator begin() { return v_; }

	iterator end() { return v_ + Size; }

	const_iterator cbegin() const { return v_; }

	const_iterator cend() const { return v_ + Size; }

	const_iterator begin() const { return v_; }

	const_iterator end() const { return v_ + Size; }

	reverse_iterator rbegin() { return reverse_iterator(end()); }

	reverse_iterator rend() { return reverse_iterator(begin()); }

	const_reverse_iterator crbegin() const
	{
		return const_reverse_iterator(cend());
	}

	const_reverse_iterator crend() const
	{
		return const_reverse_iterator(cbegin());
	}

	const_reverse_iterator rbegin() const
	{
		return const_reverse_iterator(cend());
	}

	const_reverse_iterator rend() const
	{
		return const_reverse_iterator(cbegin());
	}

	XFoam_Label size() const { return static_cast<XFoam_Label>(Size); }

	XFoam_Label max_size() const { return static_cast<XFoam_Label>(Size); }

	bool empty() const { return false; }

	void swap(XFoam_FixedList<T, Size>& a)
	{
		using std::swap;
		for (unsigned i = 0; i < Size; ++i)
		{
			swap(v_[i], a.v_[i]);
		}
	}

	bool operator==(const XFoam_FixedList<T, Size>& a) const
	{
		for (unsigned i = 0; i < Size; ++i)
		{
			if (!(v_[i] == a.v_[i]))
			{
				return false;
			}
		}
		return true;
	}

	bool operator!=(const XFoam_FixedList<T, Size>& a) const
	{
		return !(*this == a);
	}

	bool operator<(const XFoam_FixedList<T, Size>& a) const
	{
		for (unsigned i = 0; i < Size; ++i)
		{
			if (v_[i] < a.v_[i])
			{
				return true;
			}
			if (a.v_[i] < v_[i])
			{
				return false;
			}
		}
		return false;
	}

	bool operator>(const XFoam_FixedList<T, Size>& a) const
	{
		return a < *this;
	}

	bool operator<=(const XFoam_FixedList<T, Size>& a) const
	{
		return !(*this > a);
	}

	bool operator>=(const XFoam_FixedList<T, Size>& a) const
	{
		return !(*this < a);
	}
};

template<class T>
class XFoam_List
	: public XFoam_UList<T>
{
private:
	template<class U, unsigned A, unsigned B, unsigned C>
	friend class XFoam_DynamicList;

	std::vector<T> store_;

	void syncUList_()
	{
		if (store_.empty())
		{
			this->shallowCopy(XFoam_UList<T>(nullptr, 0));
		}
		else
		{
			this->shallowCopy(XFoam_UList<T>(
				store_.data(), static_cast<XFoam_Label>(store_.size())));
		}
	}

public:
	XFoam_List()
		: XFoam_UList<T>()
		, store_()
	{
		syncUList_();
	}

	explicit XFoam_List(XFoam_Label n)
		: XFoam_UList<T>()
		, store_()
	{
		setSize(n);
	}

	XFoam_List(XFoam_Label n, const T& val)
		: XFoam_UList<T>()
		, store_()
	{
		setSize(n, val);
	}

	XFoam_List(const XFoam_List<T>& other)
		: XFoam_UList<T>()
		, store_(other.store_)
	{
		syncUList_();
	}

	XFoam_List(XFoam_List<T>&& other) noexcept
		: XFoam_UList<T>()
		, store_(XFoam_move(other.store_))
	{
		syncUList_();
		other.syncUList_();
	}

	XFoam_List(std::initializer_list<T> lst)
		: XFoam_UList<T>()
		, store_(lst)
	{
		syncUList_();
	}

	template<class InputIterator>
	XFoam_List(InputIterator first, InputIterator last)
		: XFoam_UList<T>()
		, store_(first, last)
	{
		syncUList_();
	}

	static const XFoam_List<T>& null()
	{
		static const XFoam_List<T> nullRef;
		return nullRef;
	}

	void resize(XFoam_Label n) { setSize(n); }

	void resize(XFoam_Label n, const T& a) { setSize(n, a); }

	void setSize(XFoam_Label n)
	{
		if (n < 0)
		{
			n = 0;
		}
		store_.resize(static_cast<XFoam_Size>(n));
		syncUList_();
	}

	void setSize(XFoam_Label n, const T& a)
	{
		if (n < 0)
		{
			n = 0;
		}
		const XFoam_Size sz = static_cast<XFoam_Size>(n);
		const XFoam_Size old = store_.size();
		store_.resize(sz);
		for (XFoam_Size i = old; i < sz; ++i)
		{
			store_[i] = a;
		}
		syncUList_();
	}

	void clear()
	{
		store_.clear();
		syncUList_();
	}

	void append(const T& t)
	{
		setSize(this->size() + 1, t);
	}

	void append(const XFoam_List<T>& lst)
	{
		if (this == &lst)
		{
			return;
		}
		const XFoam_Label nextFree = this->size();
		setSize(nextFree + lst.size());
		for (XFoam_Label i = 0; i < lst.size(); ++i)
		{
			store_[static_cast<XFoam_Size>(nextFree + i)] = lst[i];
		}
	}

	T& newElmt(XFoam_Label i)
	{
		if (i < 0)
		{
			i = 0;
		}
		if (i >= this->size())
		{
			XFoam_Label ns = this->size() ? (2 * this->size()) : (i + 1);
			if (ns <= i)
			{
				ns = i + 1;
			}
			setSize(ns);
		}
		return store_[static_cast<XFoam_Size>(i)];
	}

	T& at(XFoam_Label i) { return store_.at(static_cast<XFoam_Size>(i)); }

	const T& at(XFoam_Label i) const { return store_.at(static_cast<XFoam_Size>(i)); }

	using XFoam_UList<T>::operator[];
	using XFoam_UList<T>::data;
	using XFoam_UList<T>::cdata;
	using XFoam_UList<T>::size;
	using XFoam_UList<T>::empty;
	using XFoam_UList<T>::begin;
	using XFoam_UList<T>::end;
	using XFoam_UList<T>::cbegin;
	using XFoam_UList<T>::cend;

	void operator=(const XFoam_List<T>& other)
	{
		if (this != &other)
		{
			store_ = other.store_;
			syncUList_();
		}
	}

	void operator=(XFoam_List<T>&& other) noexcept
	{
		if (this != &other)
		{
			store_ = XFoam_move(other.store_);
			syncUList_();
			other.syncUList_();
		}
	}

	void operator=(std::initializer_list<T> lst)
	{
		store_.assign(lst);
		syncUList_();
	}

	void operator=(const T& val)
	{
		syncUList_();
		XFoam_UList<T>::operator=(val);
	}

	void swap(XFoam_List<T>& other) noexcept
	{
		store_.swap(other.store_);
		syncUList_();
		other.syncUList_();
	}

	// 与 XFoam_DynamicList::transfer(List&) 语义一致：交换存储后清空对方（UPtrList/PtrList 的 reorder/shuffle 依赖）。
	void transfer(XFoam_List<T>& lst)
	{
		store_.swap(lst.store_);
		syncUList_();
		lst.store_.clear();
		lst.syncUList_();
	}
};

typedef XFoam_List<XFoam_Label> XFoam_LabelList;
typedef XFoam_List<XFoam_Scalar> XFoam_ScalarList;
typedef XFoam_List<XFoam_String> XFoam_StringList;
typedef XFoam_List<XFoam_LabelList> XFoam_LabelListList;
typedef XFoam_List<XFoam_Token> XFoam_TokenList;

/// 移植源码: OpenFOAM src/OpenFOAM/primitives/bools/lists/boolList.H（boolUList / boolList / boolListList 为 typedef）
/// 命名规范: foam_code.md
/// 移植规范: foam_code.md
typedef XFoam_UList<bool> XFoam_BoolUList;
typedef XFoam_List<bool> XFoam_BoolList;
typedef XFoam_List<XFoam_List<bool>> XFoam_BoolListList;

// ---------- UIndirectList / IndirectList（tmp/.../UIndirectList/UIndirectList.H、
//          tmp/.../IndirectList/IndirectList.H + *I.H）----------

template<class T>
class XFoam_UIndirectList
{
	XFoam_UList<T>& completeList_;
	const XFoam_UList<XFoam_Label>& addressing_;

public:
	XFoam_UIndirectList(
		const XFoam_UList<T>& cl, const XFoam_UList<XFoam_Label>& addr)
		: completeList_(const_cast<XFoam_UList<T>&>(cl))
		, addressing_(addr)
	{
	}

	XFoam_UIndirectList(const XFoam_UIndirectList<T>&) = default;

	XFoam_Label size() const { return addressing_.size(); }

	bool empty() const { return addressing_.empty(); }

	T& first() { return completeList_[addressing_.first()]; }

	const T& first() const { return completeList_[addressing_.first()]; }

	T& last() { return completeList_[addressing_.last()]; }

	const T& last() const { return completeList_[addressing_.last()]; }

	XFoam_Label fcIndex(XFoam_Label i) const { return addressing_.fcIndex(i); }

	XFoam_Label rcIndex(XFoam_Label i) const { return addressing_.rcIndex(i); }

	const XFoam_UList<T>& completeList() const { return completeList_; }

	const XFoam_UList<XFoam_Label>& addressing() const { return addressing_; }

	XFoam_List<T> operator()() const
	{
		XFoam_List<T> result(size());
		for (XFoam_Label i = 0; i < size(); ++i)
		{
			result[i] = (*this)[i];
		}
		return result;
	}

	T& operator[](XFoam_Label i) { return completeList_[addressing_[i]]; }

	const T& operator[](XFoam_Label i) const
	{
		return completeList_[addressing_[i]];
	}

	void operator=(const XFoam_UList<T>& ae)
	{
		if (addressing_.size() != ae.size())
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_UIndirectList::operator=: size mismatch"));
		}
		for (XFoam_Label i = 0; i < addressing_.size(); ++i)
		{
			completeList_[addressing_[i]] = ae[i];
		}
	}

	void operator=(const XFoam_UIndirectList<T>& ae)
	{
		if (addressing_.size() != ae.size())
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_UIndirectList::operator=: size mismatch"));
		}
		for (XFoam_Label i = 0; i < addressing_.size(); ++i)
		{
			completeList_[addressing_[i]] = ae[i];
		}
	}

	void operator=(const T& t)
	{
		for (XFoam_Label i = 0; i < addressing_.size(); ++i)
		{
			completeList_[addressing_[i]] = t;
		}
	}

	bool operator==(const XFoam_UIndirectList<T>& ae) const
	{
		if (addressing_.size() != ae.size())
		{
			return false;
		}
		for (XFoam_Label i = 0; i < addressing_.size(); ++i)
		{
			if (completeList_[addressing_[i]] != ae[i])
			{
				return false;
			}
		}
		return true;
	}

	bool operator!=(const XFoam_UIndirectList<T>& ae) const
	{
		return !(*this == ae);
	}

	typedef T value_type;
	typedef T& reference;
	typedef const T& const_reference;
	typedef XFoam_Label difference_type;
	typedef XFoam_Label size_type;
};


class XFoam_IndirectListAddressing
{
	XFoam_LabelList addressing_;

protected:
	explicit XFoam_IndirectListAddressing(const XFoam_UList<XFoam_Label>& addr)
		: addressing_(addr.size())
	{
		for (XFoam_Label i = 0; i < addr.size(); ++i)
		{
			addressing_[i] = addr[i];
		}
	}

	explicit XFoam_IndirectListAddressing(XFoam_LabelList&& addr)
		: addressing_(XFoam_move(addr))
	{
	}

	XFoam_IndirectListAddressing(const XFoam_IndirectListAddressing&) = delete;

	void operator=(const XFoam_IndirectListAddressing&) = delete;

	const XFoam_LabelList& addressing() const { return addressing_; }

	void resetAddressing(const XFoam_UList<XFoam_Label>& addr)
	{
		addressing_.setSize(addr.size());
		for (XFoam_Label i = 0; i < addr.size(); ++i)
		{
			addressing_[i] = addr[i];
		}
	}
};


template<class T>
class XFoam_IndirectList
	: private XFoam_IndirectListAddressing
	, public XFoam_UIndirectList<T>
{
public:
	XFoam_IndirectList(
		const XFoam_UList<T>& cl, const XFoam_UList<XFoam_Label>& addr)
		: XFoam_IndirectListAddressing(addr)
		, XFoam_UIndirectList<T>(cl, XFoam_IndirectListAddressing::addressing())
	{
	}

	XFoam_IndirectList(const XFoam_UList<T>& cl, XFoam_LabelList&& addr)
		: XFoam_IndirectListAddressing(XFoam_move(addr))
		, XFoam_UIndirectList<T>(cl, XFoam_IndirectListAddressing::addressing())
	{
	}

	XFoam_IndirectList(const XFoam_IndirectList<T>& lst)
		: XFoam_IndirectListAddressing(lst.XFoam_UIndirectList<T>::addressing())
		, XFoam_UIndirectList<T>(
			lst.completeList(), XFoam_IndirectListAddressing::addressing())
	{
	}

	explicit XFoam_IndirectList(const XFoam_UIndirectList<T>& lst)
		: XFoam_IndirectListAddressing(lst.addressing())
		, XFoam_UIndirectList<T>(
			lst.completeList(), XFoam_IndirectListAddressing::addressing())
	{
	}

	using XFoam_UIndirectList<T>::addressing;
	using XFoam_IndirectListAddressing::resetAddressing;

	using XFoam_UIndirectList<T>::operator=;

	void operator=(const XFoam_IndirectList<T>&) = delete;

	void operator=(const XFoam_UIndirectList<T>&) = delete;
};

// XFoam_DynamicList<T, SizeInc, SizeMult, SizeDiv>: 仿照 OpenFOAM Foam::DynamicList
//（tmp/.../DynamicList/DynamicList.H + DynamicListI.H）
// - 公有派生 XFoam_UList<T>；std::vector 为分配容量，sizeAddr_ 为逻辑长度（与 OF addressed size 对应）
// - 无 Istream/Ostream；append(UList) 对 this 指针与 &lst 相同时抛错（与 OF 一致）
template<class T, unsigned SizeInc = 0, unsigned SizeMult = 2, unsigned SizeDiv = 1>
class XFoam_DynamicList
	: public XFoam_UList<T>
{
	static_assert(
		(SizeInc || SizeMult) && SizeDiv,
		"Invalid sizing parameters");

	std::vector<T> store_;
	XFoam_Label sizeAddr_{0};

	void syncUList_()
	{
		if (sizeAddr_ <= 0)
		{
			this->shallowCopy(XFoam_UList<T>(nullptr, 0));
		}
		else
		{
			this->shallowCopy(XFoam_UList<T>(store_.data(), sizeAddr_));
		}
	}

	static XFoam_Label nextCapacity_(XFoam_Label curCap, XFoam_Label needElem)
	{
		const XFoam_Label grow =
			static_cast<XFoam_Label>(SizeInc)
			+ (static_cast<XFoam_Label>(SizeMult) * curCap)
				/ static_cast<XFoam_Label>(SizeDiv);
		return std::max(needElem, grow);
	}

public:
	XFoam_DynamicList()
		: XFoam_UList<T>()
		, store_()
		, sizeAddr_(0)
	{
		syncUList_();
	}

	explicit XFoam_DynamicList(XFoam_Label nElem)
		: XFoam_UList<T>()
		, store_()
		, sizeAddr_(0)
	{
		if (nElem < 0)
		{
			nElem = 0;
		}
		store_.resize(static_cast<XFoam_Size>(nElem));
		syncUList_();
	}

	XFoam_DynamicList(XFoam_Label nElem, const T& a)
		: XFoam_UList<T>()
		, store_()
	{
		if (nElem < 0)
		{
			nElem = 0;
		}
		store_.assign(static_cast<XFoam_Size>(nElem), a);
		sizeAddr_ = nElem;
		syncUList_();
	}

	XFoam_DynamicList(const XFoam_DynamicList& other)
		: XFoam_UList<T>()
		, store_(
			other.store_.begin(),
			other.store_.begin()
				+ static_cast<std::ptrdiff_t>(other.sizeAddr_))
		, sizeAddr_(other.sizeAddr_)
	{
		syncUList_();
	}

	XFoam_DynamicList(XFoam_DynamicList&& other) noexcept
		: XFoam_UList<T>()
		, store_(XFoam_move(other.store_))
		, sizeAddr_(other.sizeAddr_)
	{
		other.sizeAddr_ = 0;
		other.syncUList_();
		syncUList_();
	}

	explicit XFoam_DynamicList(const XFoam_UList<T>& lst)
		: XFoam_UList<T>()
		, store_()
	{
		sizeAddr_ = lst.size();
		if (sizeAddr_ < 0)
		{
			sizeAddr_ = 0;
		}
		store_.resize(static_cast<XFoam_Size>(sizeAddr_));
		for (XFoam_Label i = 0; i < sizeAddr_; ++i)
		{
			store_[static_cast<XFoam_Size>(i)] = lst[i];
		}
		syncUList_();
	}

	explicit XFoam_DynamicList(XFoam_List<T>&& lst)
		: XFoam_UList<T>()
		, store_(XFoam_move(lst.store_))
		, sizeAddr_(static_cast<XFoam_Label>(store_.size()))
	{
		lst.syncUList_();
		syncUList_();
	}

	XFoam_Label capacity() const
	{
		return static_cast<XFoam_Label>(store_.size());
	}

	void setCapacity(XFoam_Label nElem)
	{
		if (nElem < 0)
		{
			nElem = 0;
		}
		XFoam_Label nextFree = sizeAddr_;
		if (nextFree > nElem)
		{
			nextFree = nElem;
		}
		store_.resize(static_cast<XFoam_Size>(nElem));
		sizeAddr_ = nextFree;
		syncUList_();
	}

	void reserve(XFoam_Label nElem)
	{
		if (nElem < 0)
		{
			nElem = 0;
		}
		if (nElem > static_cast<XFoam_Label>(store_.size()))
		{
			const XFoam_Label newCap = nextCapacity_(
				static_cast<XFoam_Label>(store_.size()), nElem);
			store_.resize(static_cast<XFoam_Size>(newCap));
		}
		syncUList_();
	}

	void setSize(XFoam_Label nElem)
	{
		if (nElem < 0)
		{
			nElem = 0;
		}
		if (nElem > static_cast<XFoam_Label>(store_.size()))
		{
			const XFoam_Label newCap = nextCapacity_(
				static_cast<XFoam_Label>(store_.size()), nElem);
			store_.resize(static_cast<XFoam_Size>(newCap));
		}
		sizeAddr_ = nElem;
		syncUList_();
	}

	void setSize(XFoam_Label nElem, const T& t)
	{
		const XFoam_Label nextFree = sizeAddr_;
		setSize(nElem);
		for (XFoam_Label i = nextFree; i < sizeAddr_; ++i)
		{
			store_[static_cast<XFoam_Size>(i)] = t;
		}
	}

	void resize(XFoam_Label nElem) { setSize(nElem); }

	void resize(XFoam_Label nElem, const T& t) { setSize(nElem, t); }

	void clear()
	{
		sizeAddr_ = 0;
		syncUList_();
	}

	void clearStorage()
	{
		store_.clear();
		sizeAddr_ = 0;
		syncUList_();
	}

	XFoam_DynamicList& shrink()
	{
		if (static_cast<XFoam_Label>(store_.size()) > sizeAddr_)
		{
			store_.resize(static_cast<XFoam_Size>(sizeAddr_));
		}
		syncUList_();
		return *this;
	}

	void transfer(XFoam_DynamicList& other)
	{
		store_.swap(other.store_);
		std::swap(sizeAddr_, other.sizeAddr_);
		syncUList_();
		other.syncUList_();
	}

	void transfer(XFoam_List<T>& lst)
	{
		store_.swap(lst.store_);
		sizeAddr_ = static_cast<XFoam_Label>(store_.size());
		lst.store_.clear();
		lst.syncUList_();
		syncUList_();
	}

	XFoam_DynamicList& append(const T& t)
	{
		const XFoam_Label elemI = sizeAddr_;
		setSize(elemI + 1);
		store_[static_cast<XFoam_Size>(elemI)] = t;
		return *this;
	}

	XFoam_DynamicList& append(const XFoam_UList<T>& lst)
	{
		if (static_cast<const void*>(this) == static_cast<const void*>(&lst))
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_DynamicList::append: attempted appending to self"));
		}
		const XFoam_Label nextFree = sizeAddr_;
		setSize(nextFree + lst.size());
		for (XFoam_Label i = 0; i < lst.size(); ++i)
		{
			store_[static_cast<XFoam_Size>(nextFree + i)] = lst[i];
		}
		return *this;
	}

	T remove()
	{
		if (sizeAddr_ <= 0)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_DynamicList::remove: list is empty"));
		}
		const XFoam_Label elemI = sizeAddr_ - 1;
		T val(XFoam_move(store_[static_cast<XFoam_Size>(elemI)]));
		sizeAddr_ = elemI;
		syncUList_();
		return val;
	}

	T& operator()(XFoam_Label elemI)
	{
		if (elemI >= sizeAddr_)
		{
			setSize(elemI + 1);
		}
		return store_[static_cast<XFoam_Size>(elemI)];
	}

	void operator=(const T& t)
	{
		syncUList_();
		XFoam_UList<T>::operator=(t);
	}

	void operator=(const XFoam_DynamicList& lst)
	{
		if (this == &lst)
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_DynamicList::operator=: assignment to self"));
		}
		if (static_cast<XFoam_Label>(store_.size()) >= lst.sizeAddr_)
		{
			sizeAddr_ = lst.sizeAddr_;
			for (XFoam_Label i = 0; i < sizeAddr_; ++i)
			{
				store_[static_cast<XFoam_Size>(i)] =
					lst.store_[static_cast<XFoam_Size>(i)];
			}
		}
		else
		{
			store_.resize(static_cast<XFoam_Size>(lst.sizeAddr_));
			sizeAddr_ = lst.sizeAddr_;
			for (XFoam_Label i = 0; i < sizeAddr_; ++i)
			{
				store_[static_cast<XFoam_Size>(i)] =
					lst.store_[static_cast<XFoam_Size>(i)];
			}
		}
		syncUList_();
	}

	void operator=(XFoam_DynamicList&& lst)
	{
		if (this == &lst)
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_DynamicList::operator=: assignment to self"));
		}
		store_ = XFoam_move(lst.store_);
		sizeAddr_ = lst.sizeAddr_;
		lst.sizeAddr_ = 0;
		lst.syncUList_();
		syncUList_();
	}

	void operator=(XFoam_List<T>&& lst)
	{
		if (static_cast<void*>(this) == static_cast<void*>(&lst))
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_DynamicList::operator=: assignment to self"));
		}
		store_ = XFoam_move(lst.store_);
		sizeAddr_ = static_cast<XFoam_Label>(store_.size());
		lst.syncUList_();
		syncUList_();
	}

	void operator=(const XFoam_UList<T>& lst)
	{
		if (static_cast<const void*>(this) == static_cast<const void*>(&lst))
		{
			throw XFoam_Error(XFoam_String(
				"XFoam_DynamicList::operator=: assignment to self"));
		}
		if (static_cast<XFoam_Label>(store_.size()) >= lst.size())
		{
			sizeAddr_ = lst.size();
			for (XFoam_Label i = 0; i < sizeAddr_; ++i)
			{
				store_[static_cast<XFoam_Size>(i)] = lst[i];
			}
		}
		else
		{
			store_.resize(static_cast<XFoam_Size>(lst.size()));
			sizeAddr_ = lst.size();
			for (XFoam_Label i = 0; i < sizeAddr_; ++i)
			{
				store_[static_cast<XFoam_Size>(i)] = lst[i];
			}
		}
		syncUList_();
	}

	typename XFoam_UList<T>::iterator erase(typename XFoam_UList<T>::iterator curIter)
	{
		typename XFoam_UList<T>::iterator iter = curIter;
		typename XFoam_UList<T>::iterator nextIter = curIter;
		if (iter != this->end())
		{
			++iter;
			while (iter != this->end())
			{
				*nextIter++ = *iter++;
			}
			setSize(sizeAddr_ - 1);
		}
		return curIter;
	}

	using XFoam_UList<T>::operator[];
	using XFoam_UList<T>::data;
	using XFoam_UList<T>::cdata;
	using XFoam_UList<T>::size;
	using XFoam_UList<T>::empty;
	using XFoam_UList<T>::begin;
	using XFoam_UList<T>::end;
	using XFoam_UList<T>::cbegin;
	using XFoam_UList<T>::cend;
};

// XFoam_CompactListList<T>: 仿照 OpenFOAM Foam::CompactListList / UCompactListList
//（tmp/.../CompactListList/CompactListList.H 与 UCompactListList.H）
// - offsets_.size() == 行数 + 1；offsets_[i] 为第 i 行在 m_ 中的起始下标；
//   offsets_[i+1]-offsets_[i] 为第 i 行长度；offsets_[0]==0 且 offsets_.last()==m_.size()
// - 类型使用 xfoam_types.h 中的 XFoam_Label 等；非法构造抛出 XFoam_Error

template<class T>
class XFoam_CompactListList
{
private:
	XFoam_LabelList offsets_;
	XFoam_List<T> m_;

public:
	// operator[] 返回 XFoam_UList<T> / XFoam_UList<const T> 行视图（与 OpenFOAM UList 一致，列下标不检查）

	static const XFoam_CompactListList<T>& null()
	{
		static const XFoam_CompactListList<T> nullRef;
		return nullRef;
	}

	XFoam_CompactListList()
	{
		offsets_.setSize(1);
		offsets_[0] = 0;
	}

	XFoam_CompactListList(const XFoam_LabelList& offsets, const XFoam_List<T>& m)
		: offsets_(offsets)
		, m_(m)
	{
		validateOffsets_("XFoam_CompactListList(offsets,m)");
	}

	// 与 OpenFOAM 一致：仅分配 offsets_(mRows+1,0) 与 m_(nData,val)，不保证 offset 表与 m 一致
	//（完整拓扑需后续 setSize / 由其它构造传入 offsets）。
	XFoam_CompactListList(XFoam_Label mRows, XFoam_Label nData, const T& val)
		: offsets_(mRows + 1, XFoam_Label(0))
		, m_(nData, val)
	{
		if (mRows < 0 || nData < 0)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_CompactListList(mRows,nData): negative size"));
		}
	}

	// 由每行长度 rowSizes 构造 offsets，并将 m_ 填充为 val
	XFoam_CompactListList(const XFoam_LabelList& rowSizes, const T& val)
	{
		offsets_.setSize(rowSizes.size() + 1);
		offsets_[0] = 0;
		for (XFoam_Label i = 0; i < rowSizes.size(); ++i)
		{
			if (rowSizes[i] < 0)
			{
				throw XFoam_Error(
					XFoam_String("XFoam_CompactListList(rowSizes): negative row size"));
			}
			offsets_[i + 1] = offsets_[i] + rowSizes[i];
		}
		m_.setSize(offsets_.last(), val);
		validateOffsets_("XFoam_CompactListList(rowSizes,val)");
	}

	XFoam_CompactListList(const XFoam_CompactListList<T>& other)
		: offsets_(other.offsets_)
		, m_(other.m_)
	{
	}

	XFoam_CompactListList(XFoam_CompactListList<T>&& other) noexcept
		: offsets_(XFoam_move(other.offsets_))
		, m_(XFoam_move(other.m_))
	{
		other.clear();
	}

	~XFoam_CompactListList() = default;

	XFoam_AutoPtr<XFoam_CompactListList<T>> clone() const
	{
		return XFoam_AutoPtr<XFoam_CompactListList<T>>(
			new XFoam_CompactListList<T>(*this));
	}

	XFoam_Label size() const { return offsets_.size() - 1; }

	bool empty() const { return size() <= 0; }

	const XFoam_LabelList& offsets() const { return offsets_; }
	XFoam_LabelList& offsets() { return offsets_; }

	const XFoam_List<T>& m() const { return m_; }
	XFoam_List<T>& m() { return m_; }

	void clear()
	{
		offsets_.setSize(1);
		offsets_[0] = 0;
		m_.clear();
	}

	void transfer(XFoam_CompactListList<T>& other)
	{
		offsets_.swap(other.offsets_);
		m_.swap(other.m_);
		other.clear();
	}

	XFoam_Label index(XFoam_Label row, XFoam_Label col) const
	{
		checkRowCol_(row, col, "index");
		return offsets_[row] + col;
	}

	XFoam_Label whichRow(XFoam_Label flatIndex) const
	{
		if (flatIndex < 0 || flatIndex >= m_.size())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_CompactListList::whichRow: flat index out of range"));
		}
		// 行 r 满足 offsets_[r] <= flatIndex < offsets_[r+1]：第一个 > flatIndex 的 offset 再回退一格
		auto it = std::upper_bound(offsets_.begin(), offsets_.end(), flatIndex);
		if (it == offsets_.begin())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_CompactListList::whichRow: invalid offsets table"));
		}
		--it;
		return static_cast<XFoam_Label>(std::distance(offsets_.begin(), it));
	}

	XFoam_Label whichColumn(XFoam_Label row, XFoam_Label flatIndex) const
	{
		return flatIndex - offsets_[row];
	}

	XFoam_LabelList sizes() const
	{
		XFoam_LabelList s(size());
		for (XFoam_Label i = 0; i < size(); ++i)
		{
			s[i] = offsets_[i + 1] - offsets_[i];
		}
		return s;
	}

	XFoam_UList<T> operator[](XFoam_Label row)
	{
		checkRow_(row, "operator[]");
		const XFoam_Label start = offsets_[row];
		const XFoam_Label len = offsets_[row + 1] - start;
		return XFoam_UList<T>(m_.data() + start, len);
	}

	XFoam_UList<const T> operator[](XFoam_Label row) const
	{
		checkRow_(row, "operator[] const");
		const XFoam_Label start = offsets_[row];
		const XFoam_Label len = offsets_[row + 1] - start;
		return XFoam_UList<const T>(m_.data() + start, len);
	}

	T& operator()(XFoam_Label row, XFoam_Label col)
	{
		return m_[index(row, col)];
	}

	const T& operator()(XFoam_Label row, XFoam_Label col) const
	{
		return m_[index(row, col)];
	}

	void operator=(const T& val)
	{
		for (XFoam_Label i = 0; i < m_.size(); ++i)
		{
			m_[i] = val;
		}
	}

	XFoam_CompactListList<T>& operator=(const XFoam_CompactListList<T>& other)
	{
		if (this != &other)
		{
			offsets_ = other.offsets_;
			m_ = other.m_;
		}
		return *this;
	}

	XFoam_CompactListList<T>& operator=(XFoam_CompactListList<T>&& other) noexcept
	{
		if (this != &other)
		{
			offsets_ = XFoam_move(other.offsets_);
			m_ = XFoam_move(other.m_);
			other.clear();
		}
		return *this;
	}

private:
	void validateOffsets_(const char* ctx) const
	{
		if (offsets_.empty())
		{
			throw XFoam_Error(XFoam_String(ctx) + ": offsets empty");
		}
		if (offsets_[0] != 0)
		{
			throw XFoam_Error(XFoam_String(ctx) + ": offsets[0] must be 0");
		}
		if (offsets_.last() != m_.size())
		{
			throw XFoam_Error(XFoam_String(ctx) + ": offsets.last() must equal m().size()");
		}
		for (XFoam_Label i = 1; i < offsets_.size(); ++i)
		{
			if (offsets_[i] < offsets_[i - 1])
			{
				throw XFoam_Error(XFoam_String(ctx) + ": offsets must be non-decreasing");
			}
		}
	}

	void checkRow_(XFoam_Label row, const char* ctx) const
	{
		if (row < 0 || row >= size())
		{
			throw XFoam_Error(
				XFoam_String(ctx) + ": row out of range");
		}
	}

	void checkRowCol_(XFoam_Label row, XFoam_Label col, const char* ctx) const
	{
		checkRow_(row, ctx);
		const XFoam_Label nrow = offsets_[row + 1] - offsets_[row];
		if (col < 0 || col >= nrow)
		{
			throw XFoam_Error(
				XFoam_String(ctx) + ": column out of range");
		}
	}
};

// 对标 OpenFOAM-8
//   src/OpenFOAM/containers/LinkedLists/linkTypes/DLListBase
//   src/OpenFOAM/containers/LinkedLists/linkTypes/SLListBase
//   src/OpenFOAM/containers/LinkedLists/accessTypes/LList（XFoam_LList）
// XFoam_UILList / XFoam_ILList 与 OF 一致；基类为侵入式 XFoam_DLListBase（及 XFoam_SLListBase）。
// 典型用法：XFoam_UILList<XFoam_DLListBase, MyType>，MyType 公有派生自 XFoam_DLListBase::link。
// 静态 end 迭代器与 insert/append/remove 等非内联成员定义见 xfoam_list.cpp。

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// XFoam_DLListBase
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

class XFoam_API XFoam_DLListBase
{
public:
	struct link
	{
		link* prev_;
		link* next_;

		inline link()
			: prev_(nullptr)
			, next_(nullptr)
		{
		}

		inline bool registered() const { return prev_ != nullptr && next_ != nullptr; }

		inline void deregister()
		{
			prev_ = nullptr;
			next_ = nullptr;
		}
	};

private:
	link* first_;
	link* last_;
	XFoam_Label nElmts_;

public:
	class iterator;
	friend class iterator;
	class const_iterator;
	friend class const_iterator;
	class const_reverse_iterator;
	friend class const_reverse_iterator;

	inline XFoam_DLListBase()
		: first_(nullptr)
		, last_(nullptr)
		, nElmts_(0)
	{
	}

	inline explicit XFoam_DLListBase(link* a)
		: first_(a)
		, last_(a)
		, nElmts_(1)
	{
		a->prev_ = a;
		a->next_ = a;
	}

	XFoam_DLListBase(const XFoam_DLListBase&) = delete;

	inline ~XFoam_DLListBase() = default;

	inline XFoam_Label size() const { return nElmts_; }

	inline bool empty() const { return !nElmts_; }

	inline link* first()
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_DLListBase::first: list is empty"));
		}
		return first_;
	}

	inline const link* first() const
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_DLListBase::first: list is empty"));
		}
		return first_;
	}

	inline link* last()
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_DLListBase::last: list is empty"));
		}
		return last_;
	}

	inline const link* last() const
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_DLListBase::last: list is empty"));
		}
		return last_;
	}

	void insert(link*);
	void append(link*);
	bool swapUp(link*);
	bool swapDown(link*);
	link* removeHead();
	link* remove(link*);

	link* replace(link* oldLink, link* newLink);

	inline void clear()
	{
		first_ = nullptr;
		last_ = nullptr;
		nElmts_ = 0;
	}

	inline void transfer(XFoam_DLListBase& lst)
	{
		first_ = lst.first_;
		last_ = lst.last_;
		nElmts_ = lst.nElmts_;
		lst.clear();
	}

	void operator=(const XFoam_DLListBase&) = delete;

	class iterator
	{
		friend class XFoam_DLListBase;
		friend class const_iterator;

		XFoam_DLListBase& curList_;
		link* curElmt_;
		link curLink_;

		inline iterator(XFoam_DLListBase& s)
			: curList_(s)
			, curElmt_(nullptr)
			, curLink_()
		{
		}

	public:
		inline iterator(XFoam_DLListBase& s, link* elmt)
			: curList_(s)
			, curElmt_(elmt)
			, curLink_(*curElmt_)
		{
		}

		inline iterator(const iterator&) = default;

		inline void operator=(const iterator& iter)
		{
			curElmt_ = iter.curElmt_;
			curLink_ = iter.curLink_;
		}

		inline bool operator==(const iterator& iter) const { return curElmt_ == iter.curElmt_; }

		inline bool operator!=(const iterator& iter) const { return curElmt_ != iter.curElmt_; }

		inline link& operator*() { return *curElmt_; }

		inline iterator& operator++()
		{
			if (curLink_.next_ == curElmt_ || curList_.last_ == nullptr)
			{
				curElmt_ = nullptr;
			}
			else
			{
				curElmt_ = curLink_.next_;
				curLink_ = *curElmt_;
			}
			return *this;
		}

		inline iterator operator++(int)
		{
			iterator tmp = *this;
			++*this;
			return tmp;
		}
	};

	inline link* remove(iterator& it) { return remove(it.curElmt_); }

	inline link* replace(iterator& oldIter, link* newLink) { return replace(oldIter.curElmt_, newLink); }

	inline iterator begin()
	{
		if (size())
		{
			return iterator(*this, first());
		}
		return endIter_;
	}

	inline const iterator& end() { return endIter_; }

	class const_iterator
	{
		const XFoam_DLListBase& curList_;
		const link* curElmt_;

	public:
		inline const_iterator(const XFoam_DLListBase& s, const link* elmt)
			: curList_(s)
			, curElmt_(elmt)
		{
		}

		inline const_iterator(const iterator& iter)
			: curList_(iter.curList_)
			, curElmt_(iter.curElmt_)
		{
		}

		inline const_iterator(const const_iterator&) = default;

		inline void operator=(const const_iterator& iter) { curElmt_ = iter.curElmt_; }

		inline bool operator==(const const_iterator& iter) const { return curElmt_ == iter.curElmt_; }

		inline bool operator!=(const const_iterator& iter) const { return curElmt_ != iter.curElmt_; }

		inline const link& operator*() const { return *curElmt_; }

		inline const_iterator& operator++()
		{
			if (curElmt_ == curList_.last_)
			{
				curElmt_ = nullptr;
			}
			else
			{
				curElmt_ = curElmt_->next_;
			}
			return *this;
		}

		inline const_iterator operator++(int)
		{
			const_iterator tmp = *this;
			++*this;
			return tmp;
		}
	};

	inline const_iterator cbegin() const
	{
		if (size())
		{
			return const_iterator(*this, first());
		}
		return endConstIter_;
	}

	inline const const_iterator& cend() const { return endConstIter_; }

	inline const_iterator begin() const { return cbegin(); }

	inline const const_iterator& end() const { return endConstIter_; }

	class const_reverse_iterator
	{
		const XFoam_DLListBase& curList_;
		const link* curElmt_;

	public:
		inline const_reverse_iterator(const XFoam_DLListBase& s, const link* elmt)
			: curList_(s)
			, curElmt_(elmt)
		{
		}

		inline const_reverse_iterator(const const_reverse_iterator&) = default;

		inline void operator=(const const_reverse_iterator& iter) { curElmt_ = iter.curElmt_; }

		inline bool operator==(const const_reverse_iterator& iter) const
		{
			return curElmt_ == iter.curElmt_;
		}

		inline bool operator!=(const const_reverse_iterator& iter) const
		{
			return curElmt_ != iter.curElmt_;
		}

		inline const link& operator*() const { return *curElmt_; }

		inline const_reverse_iterator& operator++()
		{
			if (curElmt_ == curList_.first_)
			{
				curElmt_ = nullptr;
			}
			else
			{
				curElmt_ = curElmt_->prev_;
			}
			return *this;
		}

		inline const_reverse_iterator operator++(int)
		{
			const_reverse_iterator tmp = *this;
			++*this;
			return tmp;
		}
	};

	inline const_reverse_iterator crbegin() const
	{
		if (size())
		{
			return const_reverse_iterator(*this, last());
		}
		return endConstRevIter_;
	}

	inline const const_reverse_iterator& crend() const { return endConstRevIter_; }

	inline const_reverse_iterator rbegin() const { return crbegin(); }

	inline const const_reverse_iterator& rend() const { return endConstRevIter_; }

private:
	static iterator endIter_;
	static const_iterator endConstIter_;
	static const_reverse_iterator endConstRevIter_;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// XFoam_LList（对标 OpenFOAM Foam::LList，LList.H / LList.C / LListIO.C）
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

template<class LListBase, class T>
class XFoam_LList
	: public LListBase
{
public:
	class iterator;
	friend class iterator;
	class const_iterator;
	friend class const_iterator;

	struct link
		: public LListBase::link
	{
		T obj_;

		link(T a)
			: LListBase::link()
			, obj_(XFoam_move(a))
		{}
	};

	XFoam_LList() {}

	XFoam_LList(T a)
		: LListBase(new link(XFoam_move(a)))
	{}

	// 未移植：Foam::LList(Istream&) 调用 operator>>（LListIO.C）；此处不读入。
	XFoam_LList(XFoam_IStream& is)
	{
		(void)is;
	}

	XFoam_LList(const XFoam_LList& lst)
		: LListBase()
	{
		for (const T& val : lst)
		{
			this->append(val);
		}
	}

	XFoam_LList(XFoam_LList&& lst)
		: LListBase()
	{
		this->transfer(lst);
	}

	XFoam_LList(std::initializer_list<T> lst)
		: LListBase()
	{
		for (const T& val : lst)
		{
			this->append(val);
		}
	}

	~XFoam_LList() { this->clear(); }

	T& first()
	{
		return static_cast<link*>(LListBase::first())->obj_;
	}

	const T& first() const
	{
		return static_cast<const link*>(LListBase::first())->obj_;
	}

	T& last()
	{
		return static_cast<link*>(LListBase::last())->obj_;
	}

	const T& last() const
	{
		return static_cast<const link*>(LListBase::last())->obj_;
	}

	void insert(const T& a) { LListBase::insert(new link(T(a))); }

	void append(const T& a) { LListBase::append(new link(T(a))); }

	T removeHead()
	{
		link* elmtPtr = static_cast<link*>(LListBase::removeHead());
		T data = XFoam_move(elmtPtr->obj_);
		delete elmtPtr;
		return data;
	}

	T remove(link* l)
	{
		link* elmtPtr = static_cast<link*>(LListBase::remove(l));
		T data = XFoam_move(elmtPtr->obj_);
		delete elmtPtr;
		return data;
	}

	T remove(iterator& it)
	{
		link* elmtPtr = static_cast<link*>(LListBase::remove(it));
		T data = XFoam_move(elmtPtr->obj_);
		delete elmtPtr;
		return data;
	}

	void clear()
	{
		const XFoam_Label oldSize = this->size();
		for (XFoam_Label i = 0; i < oldSize; ++i)
		{
			(void)this->removeHead();
		}
		LListBase::clear();
	}

	void transfer(XFoam_LList& lst)
	{
		this->clear();
		LListBase::transfer(lst);
	}

	void operator=(const XFoam_LList& lst)
	{
		this->clear();
		for (const T& val : lst)
		{
			this->append(val);
		}
	}

	void operator=(XFoam_LList&& lst) { this->transfer(lst); }

	void operator=(std::initializer_list<T> lst)
	{
		this->clear();
		for (const T& val : lst)
		{
			this->append(val);
		}
	}

	typedef T value_type;
	typedef T& reference;
	typedef const T& const_reference;
	typedef XFoam_Label size_type;

	typedef typename LListBase::iterator LListBase_iterator;

	class iterator
		: public LListBase_iterator
	{
	public:
		explicit iterator(LListBase_iterator baseIter)
			: LListBase_iterator(baseIter)
		{}

		T& operator*()
		{
			return static_cast<link&>(LListBase_iterator::operator*()).obj_;
		}

		T& operator()() { return operator*(); }

		iterator& operator++()
		{
			LListBase_iterator::operator++();
			return *this;
		}
	};

	inline iterator begin() { return iterator(LListBase::begin()); }

	inline const iterator& end() { return static_cast<const iterator&>(LListBase::end()); }

	typedef typename LListBase::const_iterator LListBase_const_iterator;

	class const_iterator
		: public LListBase_const_iterator
	{
	public:
		explicit const_iterator(LListBase_const_iterator baseIter)
			: LListBase_const_iterator(baseIter)
		{}

		explicit const_iterator(LListBase_iterator baseIter)
			: LListBase_const_iterator(baseIter)
		{}

		const T& operator*() const
		{
			return static_cast<const link&>(LListBase_const_iterator::operator*()).obj_;
		}

		const T& operator()() const { return operator*(); }

		const_iterator& operator++()
		{
			LListBase_const_iterator::operator++();
			return *this;
		}
	};

	inline const_iterator cbegin() const { return const_iterator(LListBase::cbegin()); }

	inline const const_iterator& cend() const
	{
		return static_cast<const const_iterator&>(LListBase::cend());
	}

	inline const_iterator begin() const { return const_iterator(LListBase::begin()); }

	inline const const_iterator& end() const
	{
		return static_cast<const const_iterator&>(LListBase::end());
	}
};

template<class LListBase, class T>
inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_LList<LListBase, T>& L)
{
	L.clear();
	(void)is;
	// 未移植：Foam::operator>>(Istream&, LList&)（LListIO.C）
	return is;
}

template<class LListBase, class T>
inline XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_LList<LListBase, T>& lst)
{
	// 未移植：Foam::operator<<(Ostream&, LList&) 的 token::BEGIN_LIST / END_LIST；仅输出规模与元素占位。
	os << '\n' << lst.size() << '\n';
	for (const T& val : lst)
	{
		os << val << '\n';
	}
	return os;
}

template<class T>
using XFoam_DLList = XFoam_LList<XFoam_DLListBase, T>;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// XFoam_SLListBase
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

class XFoam_API XFoam_SLListBase
{
public:
	struct link
	{
		link* next_;

		inline link()
			: next_(nullptr)
		{
		}

		inline explicit link(link* p)
			: next_(p)
		{
		}
	};

private:
	link* last_;
	XFoam_Label nElmts_;

public:
	class iterator;
	friend class iterator;
	class const_iterator;
	friend class const_iterator;

	inline XFoam_SLListBase()
		: last_(nullptr)
		, nElmts_(0)
	{
	}

	inline explicit XFoam_SLListBase(link* a)
		: last_(a->next_ = a)
		, nElmts_(1)
	{
	}

	XFoam_SLListBase(const XFoam_SLListBase&) = delete;

	inline ~XFoam_SLListBase() = default;

	inline XFoam_Label size() const { return nElmts_; }

	inline bool empty() const { return !nElmts_; }

	inline link* first()
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_SLListBase::first: list is empty"));
		}
		return last_->next_;
	}

	inline const link* first() const
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_SLListBase::first: list is empty"));
		}
		return last_->next_;
	}

	inline link* last()
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_SLListBase::last: list is empty"));
		}
		return last_;
	}

	inline const link* last() const
	{
		if (!nElmts_)
		{
			throw XFoam_Error(XFoam_String("XFoam_SLListBase::last: list is empty"));
		}
		return last_;
	}

	void insert(link*);
	void append(link*);
	link* removeHead();
	link* remove(link*);

	inline void clear()
	{
		last_ = nullptr;
		nElmts_ = 0;
	}

	inline void transfer(XFoam_SLListBase& lst)
	{
		last_ = lst.last_;
		nElmts_ = lst.nElmts_;
		lst.clear();
	}

	void operator=(const XFoam_SLListBase&) = delete;

	class iterator
	{
		friend class XFoam_SLListBase;
		friend class const_iterator;

		XFoam_SLListBase& curList_;
		link* curElmt_;
		link curLink_;

		inline iterator(XFoam_SLListBase& s)
			: curList_(s)
			, curElmt_(nullptr)
			, curLink_()
		{
		}

	public:
		inline iterator(XFoam_SLListBase& s, link* elmt)
			: curList_(s)
			, curElmt_(elmt)
			, curLink_(*curElmt_)
		{
		}

		inline iterator(const iterator&) = default;

		inline void operator=(const iterator& iter)
		{
			curElmt_ = iter.curElmt_;
			curLink_ = iter.curLink_;
		}

		inline bool operator==(const iterator& iter) const { return curElmt_ == iter.curElmt_; }

		inline bool operator!=(const iterator& iter) const { return curElmt_ != iter.curElmt_; }

		inline link& operator*() { return *curElmt_; }

		inline iterator& operator++()
		{
			if (curElmt_ == curList_.last_ || curList_.last_ == nullptr)
			{
				curElmt_ = nullptr;
			}
			else
			{
				curElmt_ = curLink_.next_;
				curLink_ = *curElmt_;
			}
			return *this;
		}

		inline iterator operator++(int)
		{
			iterator tmp = *this;
			++*this;
			return tmp;
		}
	};

	inline link* remove(iterator& it) { return remove(it.curElmt_); }

	inline iterator begin()
	{
		if (size())
		{
			return iterator(*this, first());
		}
		return endIter_;
	}

	inline const iterator& end() { return endIter_; }

	class const_iterator
	{
		const XFoam_SLListBase& curList_;
		const link* curElmt_;

	public:
		inline const_iterator(const XFoam_SLListBase& s, const link* elmt)
			: curList_(s)
			, curElmt_(elmt)
		{
		}

		inline const_iterator(const iterator& iter)
			: curList_(iter.curList_)
			, curElmt_(iter.curElmt_)
		{
		}

		inline const_iterator(const const_iterator&) = default;

		inline void operator=(const const_iterator& iter) { curElmt_ = iter.curElmt_; }

		inline bool operator==(const const_iterator& iter) const { return curElmt_ == iter.curElmt_; }

		inline bool operator!=(const const_iterator& iter) const { return curElmt_ != iter.curElmt_; }

		inline const link& operator*() const { return *curElmt_; }

		inline const_iterator& operator++()
		{
			if (curElmt_ == curList_.last_)
			{
				curElmt_ = nullptr;
			}
			else
			{
				curElmt_ = curElmt_->next_;
			}
			return *this;
		}

		inline const_iterator operator++(int)
		{
			const_iterator tmp = *this;
			++*this;
			return tmp;
		}
	};

	inline const_iterator cbegin() const
	{
		if (size())
		{
			return const_iterator(*this, first());
		}
		return endConstIter_;
	}

	inline const const_iterator& cend() const { return endConstIter_; }

	inline const_iterator begin() const { return cbegin(); }

	inline const const_iterator& end() const { return endConstIter_; }

private:
	static iterator endIter_;
	static const_iterator endConstIter_;
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// XFoam_UILList
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

template<class LListBase, class T>
class XFoam_UILList
	: public LListBase
{
public:
	class iterator;
	friend class iterator;
	class const_iterator;
	friend class const_iterator;
	class const_reverse_iterator;
	friend class const_reverse_iterator;

	XFoam_UILList() {}

	// 未移植：与 OF UIDLList / DictionaryBase 的 Istream 构造兼容；不读入任何元素。
	explicit XFoam_UILList(XFoam_IStream& is)
	{
		(void)is;
	}

	template<class INew>
	XFoam_UILList(XFoam_IStream& is, const INew& iNew)
	{
		(void)is;
		(void)iNew;
	}

	explicit XFoam_UILList(T* a)
		: LListBase(a)
	{
	}

	XFoam_UILList(const XFoam_UILList& lst)
	{
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			this->append(const_cast<typename LListBase::link*>(reinterpret_cast<const typename LListBase::link*>(
				std::addressof(iter()))));
		}
	}

	XFoam_UILList(XFoam_UILList&& lst) { this->transfer(lst); }

	T* first() { return static_cast<T*>(LListBase::first()); }

	const T* first() const { return static_cast<const T*>(LListBase::first()); }

	T* last() { return static_cast<T*>(LListBase::last()); }

	const T* last() const { return static_cast<const T*>(LListBase::last()); }

	T* removeHead() { return static_cast<T*>(LListBase::removeHead()); }

	T* remove(T* p) { return static_cast<T*>(LListBase::remove(p)); }

	T* remove(iterator& it)
	{
		return static_cast<T*>(LListBase::remove(
			static_cast<typename LListBase::iterator&>(it)));
	}

	void operator=(const XFoam_UILList& rhs)
	{
		LListBase::clear();
		for (const_iterator iter = rhs.begin(); iter != rhs.end(); ++iter)
		{
			this->append(const_cast<typename LListBase::link*>(reinterpret_cast<const typename LListBase::link*>(
				std::addressof(iter()))));
		}
	}

	void operator=(XFoam_UILList&& rhs) { this->transfer(rhs); }

	typedef T value_type;
	typedef T& reference;
	typedef const T& const_reference;
	typedef XFoam_Label size_type;

	typedef typename LListBase::iterator LListBase_iterator;

	class iterator
		: public LListBase_iterator
	{
	public:
		explicit iterator(LListBase_iterator baseIter)
			: LListBase_iterator(baseIter)
		{
		}

		T& operator*() { return static_cast<T&>(LListBase_iterator::operator*()); }

		T& operator()() { return operator*(); }

		iterator& operator++()
		{
			LListBase_iterator::operator++();
			return *this;
		}
	};

	inline iterator begin() { return iterator(LListBase::begin()); }

	inline const iterator& end() { return static_cast<const iterator&>(LListBase::end()); }

	typedef typename LListBase::const_iterator LListBase_const_iterator;

	class const_iterator
		: public LListBase_const_iterator
	{
	public:
		explicit const_iterator(LListBase_const_iterator baseIter)
			: LListBase_const_iterator(baseIter)
		{
		}

		const_iterator(LListBase_iterator baseIter)
			: LListBase_const_iterator(baseIter)
		{
		}

		const T& operator*() const
		{
			return static_cast<const T&>(LListBase_const_iterator::operator*());
		}

		const T& operator()() const { return operator*(); }

		const_iterator& operator++()
		{
			LListBase_const_iterator::operator++();
			return *this;
		}
	};

	inline const_iterator cbegin() const { return const_iterator(LListBase::cbegin()); }

	inline const const_iterator& cend() const
	{
		return static_cast<const const_iterator&>(LListBase::cend());
	}

	inline const_iterator begin() const { return const_iterator(LListBase::begin()); }

	inline const const_iterator& end() const
	{
		return static_cast<const const_iterator&>(LListBase::end());
	}

	class const_reverse_iterator
		: public LListBase::const_reverse_iterator
	{
	public:
		explicit const_reverse_iterator(typename LListBase::const_reverse_iterator baseIter)
			: LListBase::const_reverse_iterator(baseIter)
		{
		}

		const T& operator*() const
		{
			return static_cast<const T&>(LListBase::const_reverse_iterator::operator*());
		}

		const T& operator()() const { return operator*(); }

		const_reverse_iterator& operator++()
		{
			LListBase::const_reverse_iterator::operator++();
			return *this;
		}
	};

	inline const_reverse_iterator crbegin() const
	{
		return const_reverse_iterator(LListBase::crbegin());
	}

	inline const const_reverse_iterator& crend() const
	{
		return static_cast<const const_reverse_iterator&>(LListBase::crend());
	}

	inline const_reverse_iterator rbegin() const
	{
		return const_reverse_iterator(LListBase::rbegin());
	}

	inline const const_reverse_iterator& rend() const
	{
		return static_cast<const const_reverse_iterator&>(LListBase::rend());
	}

	bool operator==(const XFoam_UILList& rhs) const
	{
		if (this->size() != rhs.size())
		{
			return false;
		}
		bool equal = true;
		const_iterator iter1 = this->begin();
		const_iterator iter2 = rhs.begin();
		for (; iter1 != this->end(); ++iter1, ++iter2)
		{
			equal = equal && (iter1() == iter2());
		}
		return equal;
	}

	bool operator!=(const XFoam_UILList& rhs) const { return !(*this == rhs); }

	// 未移植：OF 的 token::BEGIN_LIST / END_LIST 与 check；此处仅写 size 与换行占位。
	friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_UILList& lst)
	{
		os << '\n' << lst.size() << '\n';
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			os << iter() << '\n';
		}
		return os;
	}
};

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// XFoam_ILList
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

template<class LListBase, class T>
class XFoam_ILList
	: public XFoam_UILList<LListBase, T>
{
	template<class INew>
	void read(XFoam_IStream&, const INew&)
	{
		// 未移植：OF ILListIO.C 的 token/列表语法；无操作。
	}

public:
	XFoam_ILList() {}

	explicit XFoam_ILList(T* a)
		: XFoam_UILList<LListBase, T>(a)
	{
	}

	// 未移植：XFoam 无 OF Istream/token；不读入任何元素，保持空表。
	explicit XFoam_ILList(XFoam_IStream& is)
	{
		(void)is;
	}

	XFoam_ILList(const XFoam_ILList& lst)
		: XFoam_UILList<LListBase, T>()
	{
		for (typename XFoam_UILList<LListBase, T>::const_iterator iter = lst.begin(); iter != lst.end();
			 ++iter)
		{
			this->append(iter().clone().ptr());
		}
	}

	XFoam_ILList(XFoam_ILList&& lst)
		: XFoam_UILList<LListBase, T>()
	{
		this->transfer(lst);
	}

	template<class CloneArg>
	XFoam_ILList(const XFoam_ILList& lst, const CloneArg& cloneArg)
		: XFoam_UILList<LListBase, T>()
	{
		for (typename XFoam_UILList<LListBase, T>::const_iterator iter = lst.begin(); iter != lst.end();
			 ++iter)
		{
			this->append(iter().clone(cloneArg).ptr());
		}
	}

	template<class INew>
	XFoam_ILList(XFoam_IStream& is, const INew& iNew)
	{
		this->read(is, iNew);
	}

	~XFoam_ILList() { this->clear(); }

	bool eraseHead()
	{
		T* tPtr;
		if ((tPtr = this->removeHead()))
		{
			delete tPtr;
			return true;
		}
		return false;
	}

	bool erase(T* p)
	{
		T* tPtr;
		if ((tPtr = this->remove(p)))
		{
			delete tPtr;
			return true;
		}
		return false;
	}

	void clear()
	{
		const XFoam_Label oldSize = this->size();
		for (XFoam_Label i = 0; i < oldSize; ++i)
		{
			this->eraseHead();
		}
	}

	void transfer(XFoam_ILList& lst)
	{
		this->clear();
		LListBase::transfer(lst);
	}

	void operator=(const XFoam_ILList& lst)
	{
		this->clear();
		for (typename XFoam_UILList<LListBase, T>::const_iterator iter = lst.begin(); iter != lst.end();
			 ++iter)
		{
			this->append(iter().clone().ptr());
		}
	}

	void operator=(XFoam_ILList&& lst) { this->transfer(lst); }

	friend XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_ILList& L)
	{
		L.clear();
		(void)is;
		// 未移植：OF 中 L.read(is, INew<T>())；此处等价于仅 discard 原有内容。
		return is;
	}
};

template<class T>
using XFoam_IDLList = XFoam_ILList<XFoam_DLListBase, T>;

template<class T>
using XFoam_UIDLList = XFoam_UILList<XFoam_DLListBase, T>;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// XFoam_LPtrList / XFoam_SLPtrList（对标 OF-9 LPtrList.H / SLPtrList.H）、
// XFoam_ULPtrList（对标 OF-dev ULPtrList.H+ULPtrList.C+ULPtrListIO.C）、
// XFoam_UPtrList / XFoam_PtrList（对标 UPtrList.H+UPtrList.C / PtrList.H+PtrList.C）。
// UPtrList 的 operator>> 未移植；PtrList::read(Istream&, INew) 对标 PtrListIO.C。
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

template<class LListBase, class T>
class XFoam_LPtrList;

template<class LListBase, class T>
class XFoam_ULPtrList;

template<class T>
class XFoam_PtrList;

template<class T>
class XFoam_UPtrList;

template<class LListBase, class T>
class XFoam_LPtrList
	: public XFoam_LList<LListBase, T*>
{
	typedef XFoam_LList<LListBase, T*> Base;

	template<class INew>
	void read(XFoam_IStream&, const INew&);

public:
	typedef typename Base::iterator Base_iterator;
	typedef typename Base::const_iterator Base_const_iterator;

	class iterator;
	friend class iterator;
	class const_iterator;
	friend class const_iterator;

	class iterator
		: public Base::iterator
	{
	public:
		explicit iterator(Base_iterator baseIter)
			: Base::iterator(baseIter)
		{
		}

		T& operator*() { return *(Base::iterator::operator*()); }

		T& operator()() { return operator*(); }
	};

	class const_iterator
		: public Base::const_iterator
	{
	public:
		explicit const_iterator(Base_const_iterator baseIter)
			: Base::const_iterator(baseIter)
		{
		}

		explicit const_iterator(Base_iterator baseIter)
			: Base::const_iterator(baseIter)
		{
		}

		const T& operator*() const { return *(Base::const_iterator::operator*()); }

		const T& operator()() const { return operator*(); }
	};

	XFoam_LPtrList() {}

	explicit XFoam_LPtrList(T* a)
		: Base(a)
	{
	}

	explicit XFoam_LPtrList(XFoam_IStream& is)
	{
		(void)is;
		// 未移植：Foam::LPtrList(Istream&)（LPtrListIO.C）。
	}

	template<class INew>
	XFoam_LPtrList(XFoam_IStream& is, const INew&)
	{
		(void)is;
		// 未移植：Foam::LPtrList(Istream&, INew)（LPtrListIO.C）。
	}

	XFoam_LPtrList(const XFoam_LPtrList& lst)
		: Base()
	{
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			this->append(iter().clone().ptr());
		}
	}

	XFoam_LPtrList(XFoam_LPtrList&& lst)
		: Base()
	{
		this->transfer(lst);
	}

	~XFoam_LPtrList() { this->clear(); }

	T& first() { return *Base::first(); }

	const T& first() const { return *Base::first(); }

	T& last() { return *Base::last(); }

	const T& last() const { return *Base::last(); }

	bool eraseHead()
	{
		T* tPtr = this->removeHead();
		if (tPtr)
		{
			delete tPtr;
			return true;
		}
		return false;
	}

	void clear()
	{
		const XFoam_Label oldSize = this->size();
		for (XFoam_Label i = 0; i < oldSize; ++i)
		{
			(void)eraseHead();
		}
	}

	void transfer(XFoam_LPtrList& lst)
	{
		this->clear();
		Base::transfer(lst);
	}

	void operator=(const XFoam_LPtrList& lst)
	{
		this->clear();
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			this->append(iter().clone().ptr());
		}
	}

	void operator=(XFoam_LPtrList&& lst) { this->transfer(lst); }

	typedef T& reference;
	typedef T& const_reference;

	iterator begin() { return iterator(Base::begin()); }

	iterator end()
	{
		Base_iterator it = Base::begin();
		const XFoam_Label n = this->Base::size();
		for (XFoam_Label i = 0; i < n; ++i)
		{
			++it;
		}
		return iterator(it);
	}

	const_iterator begin() const { return const_iterator(Base::cbegin()); }

	const_iterator end() const
	{
		Base_const_iterator it = Base::cbegin();
		const XFoam_Label n = this->Base::size();
		for (XFoam_Label i = 0; i < n; ++i)
		{
			++it;
		}
		return const_iterator(it);
	}

	const_iterator cbegin() const { return const_iterator(Base::cbegin()); }

	const_iterator cend() const { return end(); }

	friend XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_LPtrList& L)
	{
		L.clear();
		(void)is;
		// 未移植：Foam::operator>>(Istream&, LPtrList&)（LPtrListIO.C）。
		return is;
	}

	friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_LPtrList& lst)
	{
		os << '\n' << lst.size() << '\n';
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			os << iter() << '\n';
		}
		return os;
	}
};

template<class LListBase, class T>
class XFoam_ULPtrList
	: public XFoam_LList<LListBase, T*>
{
	typedef XFoam_LList<LListBase, T*> Base;

public:
	typedef typename Base::iterator Base_iterator;
	typedef typename Base::const_iterator Base_const_iterator;

	class iterator;
	friend class iterator;
	class const_iterator;
	friend class const_iterator;

	class iterator
		: public Base::iterator
	{
	public:
		explicit iterator(Base_iterator baseIter)
			: Base::iterator(baseIter)
		{
		}

		T& operator*() const { return *(Base::iterator::operator*()); }

		T& operator()() const { return operator*(); }
	};

	class const_iterator
		: public Base::const_iterator
	{
	public:
		explicit const_iterator(Base_const_iterator baseIter)
			: Base::const_iterator(baseIter)
		{
		}

		explicit const_iterator(Base_iterator baseIter)
			: Base::const_iterator(baseIter)
		{
		}

		const T& operator*() const { return *(Base::const_iterator::operator*()); }

		const T& operator()() const { return operator*(); }
	};

	XFoam_ULPtrList() {}

	explicit XFoam_ULPtrList(T* a)
		: Base(a)
	{
	}

	XFoam_ULPtrList(const XFoam_ULPtrList& lst)
		: Base()
	{
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			this->append(const_cast<T*>(&iter()));
		}
	}

	XFoam_ULPtrList(XFoam_ULPtrList&& lst)
		: Base()
	{
		Base::transfer(lst);
	}

	T& first() { return *Base::first(); }

	const T& first() const { return *Base::first(); }

	T& last() { return *Base::last(); }

	const T& last() const { return *Base::last(); }

	void operator=(const XFoam_ULPtrList& lst)
	{
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			this->append(const_cast<T*>(&iter()));
		}
	}

	void operator=(XFoam_ULPtrList&& lst) { Base::transfer(lst); }

	typedef T& reference;
	typedef T& const_reference;

	iterator begin() { return iterator(Base::begin()); }

	iterator end()
	{
		Base_iterator it = Base::begin();
		const XFoam_Label n = this->Base::size();
		for (XFoam_Label i = 0; i < n; ++i)
		{
			++it;
		}
		return iterator(it);
	}

	const_iterator begin() const { return const_iterator(Base::cbegin()); }

	const_iterator end() const
	{
		Base_const_iterator it = Base::cbegin();
		const XFoam_Label n = this->Base::size();
		for (XFoam_Label i = 0; i < n; ++i)
		{
			++it;
		}
		return const_iterator(it);
	}

	const_iterator cbegin() const { return const_iterator(Base::cbegin()); }

	const_iterator cend() const { return end(); }

	friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_ULPtrList& lst)
	{
		os << '\n' << lst.size();
		os << '\n' << static_cast<char>(XFoam_Token::BEGIN_LIST) << '\n';
		for (const_iterator iter = lst.begin(); iter != lst.end(); ++iter)
		{
			os << iter() << '\n';
		}
		os << static_cast<char>(XFoam_Token::END_LIST);
		return os;
	}
};

template<class T>
using XFoam_SLPtrList = XFoam_LPtrList<XFoam_SLListBase, T>;

template<class T>
class XFoam_UPtrList
{
	friend class XFoam_PtrList<T>;

	XFoam_List<T*> ptrs_;

public:
	XFoam_UPtrList()
		: ptrs_()
	{
	}

	explicit XFoam_UPtrList(const XFoam_Label s)
		: ptrs_(s, reinterpret_cast<T*>(0))
	{
	}

	XFoam_UPtrList(XFoam_UPtrList& a, bool reuse)
		: ptrs_()
	{
		if (reuse)
		{
			ptrs_.transfer(a.ptrs_);
		}
		else
		{
			ptrs_ = a.ptrs_;
		}
	}

	XFoam_Label size() const { return ptrs_.size(); }

	bool empty() const { return ptrs_.empty(); }

	T& first() { return this->operator[](0); }

	const T& first() const { return this->operator[](0); }

	T& last() { return this->operator[](this->size() - 1); }

	const T& last() const { return this->operator[](this->size() - 1); }

	void setSize(const XFoam_Label newSize)
	{
		const XFoam_Label oldSize = size();

		if (newSize <= 0)
		{
			clear();
		}
		else if (newSize < oldSize)
		{
			ptrs_.setSize(newSize);
		}
		else if (newSize > oldSize)
		{
			ptrs_.setSize(newSize);
			for (XFoam_Label i = oldSize; i < newSize; ++i)
			{
				ptrs_[i] = nullptr;
			}
		}
	}

	void resize(const XFoam_Label newSize) { setSize(newSize); }

	void clear() { ptrs_.clear(); }

	void transfer(XFoam_UPtrList& a) { ptrs_.transfer(a.ptrs_); }

	bool set(const XFoam_Label i) const { return ptrs_[i] != nullptr; }

	T* set(const XFoam_Label i, T* ptr)
	{
		T* old = ptrs_[i];
		ptrs_[i] = ptr;
		return old;
	}

	void reorder(const XFoam_UList<XFoam_Label>& oldToNew)
	{
		if (oldToNew.size() != size())
		{
			throw XFoam_Error(XFoam_String("XFoam_UPtrList::reorder: size of map not equal to list size"));
		}

		XFoam_List<T*> newPtrs_(ptrs_.size(), reinterpret_cast<T*>(0));

		for (XFoam_Label i = 0; i < size(); ++i)
		{
			const XFoam_Label newI = oldToNew[i];

			if (newI < 0 || newI >= size())
			{
				throw XFoam_Error(XFoam_String("XFoam_UPtrList::reorder: illegal index"));
			}

			if (newPtrs_[newI])
			{
				throw XFoam_Error(XFoam_String("XFoam_UPtrList::reorder: reorder map is not unique"));
			}
			newPtrs_[newI] = ptrs_[i];
		}

		for (XFoam_Label i = 0; i < newPtrs_.size(); ++i)
		{
			if (!newPtrs_[i])
			{
				throw XFoam_Error(XFoam_String("XFoam_UPtrList::reorder: element not set after reordering"));
			}
		}

		ptrs_.transfer(newPtrs_);
	}

	void shuffle(const XFoam_UList<XFoam_Label>& newToOld)
	{
		XFoam_List<T*> newPtrs_(newToOld.size(), reinterpret_cast<T*>(0));

		for (XFoam_Label newI = 0; newI < newToOld.size(); ++newI)
		{
			const XFoam_Label oldI = newToOld[newI];

			if (oldI >= 0 && oldI < size())
			{
				newPtrs_[newI] = ptrs_[oldI];
				ptrs_[oldI] = nullptr;
			}
		}

		ptrs_.transfer(newPtrs_);
	}

	const T& operator[](const XFoam_Label i) const
	{
		if (!ptrs_[i])
		{
			throw XFoam_Error(XFoam_String("XFoam_UPtrList::operator[] const: hanging pointer"));
		}
		return *ptrs_[i];
	}

	T& operator[](const XFoam_Label i)
	{
		if (!ptrs_[i])
		{
			throw XFoam_Error(XFoam_String("XFoam_UPtrList::operator[]: hanging pointer"));
		}
		return *ptrs_[i];
	}

	const T* operator()(const XFoam_Label i) const { return ptrs_[i]; }

	typedef T value_type;
	typedef T& reference;
	typedef const T& const_reference;

	friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_UPtrList& L)
	{
		os << '\n'
		   << '\t' << L.size() << '\n'
		   << '\t' << '(';
		for (XFoam_Label i = 0; i < L.size(); ++i)
		{
			os << '\n'
			   << '\t' << L[i];
		}
		os << '\n'
		   << '\t' << ')' << '\n';
		return os;
	}
};

template<class T>
XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_UPtrList<T>& L)
{
	(void)L;
	(void)is;
	// 未移植：Foam::operator>>(Istream&, UPtrList&)（UPtrListIO.C）。
	return is;
}

template<class T>
void XFoam_writeEntry(XFoam_OStream& os, const XFoam_UPtrList<T>& l)
{
	os << l;
}

template<class T>
struct XFoam_INew
{
	template<class U = T>
	typename std::enable_if<std::is_constructible<U, XFoam_IStream&>::value, XFoam_AutoPtr<T>>::type
	operator()(XFoam_IStream& is) const
	{
		return XFoam_AutoPtr<T>(new T(is));
	}
};

template<class T>
class XFoam_PtrList
	: public XFoam_UPtrList<T>
{
	typedef XFoam_UPtrList<T> Base;

public:
	XFoam_PtrList()
		: Base()
	{
	}

	explicit XFoam_PtrList(const XFoam_Label s)
		: Base(s)
	{
	}

	XFoam_PtrList(const XFoam_PtrList& a)
		: Base(a.size())
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			this->ptrs_[i] = (a[i]).clone().ptr();
		}
	}

	template<class CloneArg>
	XFoam_PtrList(const XFoam_PtrList& a, const CloneArg& cloneArg)
		: Base(a.size())
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			this->ptrs_[i] = (a[i]).clone(cloneArg).ptr();
		}
	}

	XFoam_PtrList(XFoam_PtrList&& lst) { this->transfer(lst); }

	XFoam_PtrList(XFoam_PtrList& a, bool reuse)
		: Base(a, reuse)
	{
		if (!reuse)
		{
			for (XFoam_Label i = 0; i < this->size(); ++i)
			{
				this->ptrs_[i] = (a[i]).clone().ptr();
			}
		}
	}

	explicit XFoam_PtrList(const XFoam_SLPtrList<T>& sll)
		: Base(sll.size())
	{
		if (sll.size())
		{
			XFoam_Label i = 0;
			for (typename XFoam_SLPtrList<T>::const_iterator iter = sll.begin(); iter != sll.end(); ++iter)
			{
				this->ptrs_[i++] = (iter()).clone().ptr();
			}
		}
	}

	template<class INew>
	XFoam_PtrList(XFoam_IStream& is, const INew& inewt)
	{
		read(is, inewt);
	}

	template<class U = T>
	explicit XFoam_PtrList(
		XFoam_IStream& is,
		typename std::enable_if<std::is_constructible<U, XFoam_IStream&>::value, int>::type = 0)
	{
		read(is, XFoam_INew<T>());
	}

	template<class INew>
	void read(XFoam_IStream& is, const INew& inewt);

	~XFoam_PtrList()
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			if (this->ptrs_[i])
			{
				delete this->ptrs_[i];
				this->ptrs_[i] = nullptr;
			}
		}
		this->ptrs_.clear();
	}

	void setSize(const XFoam_Label newSize)
	{
		if (newSize < 0)
		{
			throw XFoam_Error(XFoam_String("XFoam_PtrList::setSize: bad size"));
		}

		const XFoam_Label oldSize = this->size();

		if (newSize == 0)
		{
			clear();
		}
		else if (newSize < oldSize)
		{
			for (XFoam_Label i = newSize; i < oldSize; ++i)
			{
				if (this->ptrs_[i])
				{
					delete this->ptrs_[i];
				}
			}
			this->ptrs_.setSize(newSize);
		}
		else
		{
			this->ptrs_.setSize(newSize);
			for (XFoam_Label i = oldSize; i < newSize; ++i)
			{
				this->ptrs_[i] = nullptr;
			}
		}
	}

	void resize(const XFoam_Label newSize) { setSize(newSize); }

	void clear()
	{
		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			if (this->ptrs_[i])
			{
				delete this->ptrs_[i];
			}
		}
		this->ptrs_.clear();
	}

	void append(T* ptr)
	{
		const XFoam_Label sz = this->size();
		setSize(sz + 1);
		this->ptrs_[sz] = ptr;
	}

	// 对标 OpenFOAM PtrList / DLList：在表头插入（DictionaryBase::insert）。
	void insert(T* ptr)
	{
		const XFoam_Label n = this->size();
		setSize(n + 1);
		for (XFoam_Label j = n; j > 0; --j)
		{
			this->ptrs_[static_cast<XFoam_Size>(j)] = this->ptrs_[static_cast<XFoam_Size>(j - 1)];
		}
		this->ptrs_[0] = ptr;
	}

	void insert(const XFoam_AutoPtr<T>& aptr)
	{
		insert(const_cast<XFoam_AutoPtr<T>&>(aptr).ptr());
	}

	template<class U = T>
	typename std::enable_if<std::is_base_of<XFoam_RefCount, U>::value>::type insert(
		const XFoam_Tmp<T>& t)
	{
		insert(const_cast<XFoam_Tmp<T>&>(t).ptr());
	}

	// 按索引移除并压缩存储；返回对象所有权（与 OF PtrList::set 移除语义配合 DictionaryBase::remove）。
	XFoam_AutoPtr<T> removeAt(const XFoam_Label i)
	{
		if (i < 0 || i >= this->size())
		{
			return XFoam_AutoPtr<T>();
		}
		T* const raw = this->ptrs_[static_cast<XFoam_Size>(i)];
		this->ptrs_[static_cast<XFoam_Size>(i)] = nullptr;
		const XFoam_Label n = this->size();
		for (XFoam_Label j = i; j < n - 1; ++j)
		{
			this->ptrs_[static_cast<XFoam_Size>(j)] = this->ptrs_[static_cast<XFoam_Size>(j + 1)];
		}
		this->ptrs_.setSize(n - 1);
		return XFoam_AutoPtr<T>(raw);
	}

	// 对标 OpenFOAM PtrList::shrink：去掉尾部空指针槽位。
	void shrink()
	{
		while (this->size() > 0 && !this->ptrs_[static_cast<XFoam_Size>(this->size() - 1)])
		{
			this->ptrs_.setSize(this->size() - 1);
		}
	}

	void append(const XFoam_AutoPtr<T>& aptr)
	{
		append(const_cast<XFoam_AutoPtr<T>&>(aptr).ptr());
	}

	template<class U = T>
	typename std::enable_if<std::is_base_of<XFoam_RefCount, U>::value>::type append(
		const XFoam_Tmp<T>& t)
	{
		append(const_cast<XFoam_Tmp<T>&>(t).ptr());
	}

	void transfer(XFoam_PtrList& a)
	{
		clear();
		this->ptrs_.transfer(a.ptrs_);
	}

	bool set(const XFoam_Label i) const { return this->ptrs_[i] != nullptr; }

	XFoam_AutoPtr<T> set(const XFoam_Label i, T* ptr)
	{
		XFoam_AutoPtr<T> old(this->ptrs_[i]);
		this->ptrs_[i] = ptr;
		return old;
	}

	XFoam_AutoPtr<T> set(const XFoam_Label i, const XFoam_AutoPtr<T>& aptr)
	{
		return set(i, const_cast<XFoam_AutoPtr<T>&>(aptr).ptr());
	}

	template<class U = T>
	typename std::enable_if<std::is_base_of<XFoam_RefCount, U>::value, XFoam_AutoPtr<T>>::type set(
		const XFoam_Label i,
		const XFoam_Tmp<T>& t)
	{
		return set(i, const_cast<XFoam_Tmp<T>&>(t).ptr());
	}

	void reorder(const XFoam_UList<XFoam_Label>& oldToNew)
	{
		if (oldToNew.size() != this->size())
		{
			throw XFoam_Error(XFoam_String("XFoam_PtrList::reorder: size mismatch"));
		}

		XFoam_List<T*> newPtrs_(this->ptrs_.size(), reinterpret_cast<T*>(0));

		for (XFoam_Label i = 0; i < this->size(); ++i)
		{
			const XFoam_Label newI = oldToNew[i];

			if (newI < 0 || newI >= this->size())
			{
				throw XFoam_Error(XFoam_String("XFoam_PtrList::reorder: illegal index"));
			}

			if (newPtrs_[newI])
			{
				throw XFoam_Error(XFoam_String("XFoam_PtrList::reorder: map not unique"));
			}
			newPtrs_[newI] = this->ptrs_[i];
		}

		for (XFoam_Label i = 0; i < newPtrs_.size(); ++i)
		{
			if (!newPtrs_[i])
			{
				throw XFoam_Error(XFoam_String("XFoam_PtrList::reorder: element not set"));
			}
		}

		this->ptrs_.transfer(newPtrs_);
	}

	void shuffle(const XFoam_UList<XFoam_Label>& newToOld)
	{
		XFoam_List<T*> newPtrs_(newToOld.size(), reinterpret_cast<T*>(0));

		for (XFoam_Label newI = 0; newI < newToOld.size(); ++newI)
		{
			const XFoam_Label oldI = newToOld[newI];

			if (oldI >= 0 && oldI < this->size())
			{
				newPtrs_[newI] = this->ptrs_[oldI];
				this->ptrs_[oldI] = nullptr;
			}
		}

		clear();
		this->ptrs_.transfer(newPtrs_);
	}

	void operator=(const XFoam_PtrList& a)
	{
		if (this == &a)
		{
			throw XFoam_Error(XFoam_String("XFoam_PtrList::operator=: attempted assignment to self"));
		}

		if (this->size() == 0)
		{
			setSize(a.size());
			for (XFoam_Label i = 0; i < this->size(); ++i)
			{
				this->ptrs_[i] = (a[i]).clone().ptr();
			}
		}
		else if (a.size() == this->size())
		{
			// 不用 T 的赋值运算符（如 XFoam_Block 为 delete）；clone 后换指针。
			for (XFoam_Label i = 0; i < this->size(); ++i)
			{
				XFoam_AutoPtr<T> ap((a[i]).clone());
				T* const raw = ap.ptr();
				if (this->ptrs_[i])
				{
					delete this->ptrs_[i];
				}
				this->ptrs_[i] = raw;
			}
		}
		else
		{
			throw XFoam_Error(XFoam_String("XFoam_PtrList::operator=: bad size"));
		}
	}

	void operator=(XFoam_PtrList&& a)
	{
		if (this == &a)
		{
			throw XFoam_Error(XFoam_String("XFoam_PtrList::operator=(move): attempted assignment to self"));
		}
		transfer(a);
	}

	friend XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_PtrList& L);
};

template<class T>
template<class INew>
void XFoam_PtrList<T>::read(XFoam_IStream& is, const INew& inewt)
{
	this->clear();
	if (!is.good())
	{
		return;
	}
	XFoam_Token firstTok(is);
	if (firstTok.good() && firstTok.isPunctuation() && firstTok.pToken() == XFoam_Token::BEGIN_LIST)
	{
		// 已消费 leading '('
	}
	else if (firstTok.good())
	{
		is.putBack(firstTok);
	}

	while (is.good())
	{
		XFoam_Token t(is);
		if (!t.good() || t.error())
		{
			break;
		}
		if (t.isPunctuation() && t.pToken() == XFoam_Token::END_LIST)
		{
			break;
		}
		is.putBack(t);
		XFoam_AutoPtr<T> ap(inewt(is));
		if (!ap.valid())
		{
			break;
		}
		// 不调用 append(ap)：MSVC 对重载集可能实例化 append(Tmp)（AutoPtr→const T&→Tmp）。
		{
			T* const raw = ap.ptr();
			const XFoam_Label sz = this->size();
			this->setSize(sz + 1);
			this->ptrs_[sz] = raw;
		}
	}
}

template<class T>
typename std::enable_if<std::is_constructible<T, XFoam_IStream&>::value, XFoam_IStream&>::type
operator>>(XFoam_IStream& is, XFoam_PtrList<T>& L)
{
	L.clear();
	L.read(is, XFoam_INew<T>());
	return is;
}

/*---------------------------------------------------------------------------*\
                        Class XFoam_NamedEnum Declaration
\*---------------------------------------------------------------------------*/

template<class Enum, int nEnum>
class XFoam_NamedEnum
	: public XFoam_HashTable<XFoam_Label, XFoam_Word>
{
	XFoam_NamedEnum(const XFoam_NamedEnum&) = delete;

	void operator=(const XFoam_NamedEnum&) = delete;

public:
	static const char* names[nEnum];

	XFoam_NamedEnum();

	Enum read(XFoam_IStream& is) const;

	void write(const Enum e, XFoam_OStream& os) const;

	static XFoam_List<XFoam_String> strings();

	static XFoam_List<XFoam_Word> words();

	const Enum operator[](const char* name) const
	{
		return Enum(XFoam_HashTable<XFoam_Label, XFoam_Word>::operator[](XFoam_Word(name)));
	}

	const Enum operator[](const XFoam_Word& name) const
	{
		return Enum(XFoam_HashTable<XFoam_Label, XFoam_Word>::operator[](name));
	}

	const char* operator[](const Enum e) const;
};

template<class Enum, int nEnum>
XFoam_NamedEnum<Enum, nEnum>::XFoam_NamedEnum()
	: XFoam_HashTable<XFoam_Label, XFoam_Word>(2 * nEnum)
{
	for (unsigned int enumI = 0; enumI < static_cast<unsigned>(nEnum); ++enumI)
	{
		if (!names[enumI] || names[enumI][0] == '\0')
		{
			XFoam_FatalErrorInFunction << "Illegal enumeration name at position " << enumI << '\n'
									   << "Possibly your XFoam_NamedEnum::names array"
									   << " is not of size " << nEnum << '\n'
									   << XFoam_exit(XFoam_FatalError, 1);
		}
		insert(XFoam_Word(names[enumI]), static_cast<XFoam_Label>(enumI));
	}
}

template<class Enum, int nEnum>
Enum XFoam_NamedEnum<Enum, nEnum>::read(XFoam_IStream& is) const
{
	XFoam_Word name;
	is >> name;
	const auto iter = find(name);
	if (iter == end())
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(XFoam_String()))
			<< name << " is not in enumeration: " << sortedToc() << XFoam_exit(XFoam_FatalIOError, 1);
	}
	return Enum((*iter)());
}

template<class Enum, int nEnum>
void XFoam_NamedEnum<Enum, nEnum>::write(const Enum e, XFoam_OStream& os) const
{
	os << operator[](e);
}

template<class Enum, int nEnum>
XFoam_List<XFoam_String> XFoam_NamedEnum<Enum, nEnum>::strings()
{
	XFoam_List<XFoam_String> lst(nEnum);

	XFoam_Label nElem = 0;
	for (unsigned int enumI = 0; enumI < static_cast<unsigned>(nEnum); ++enumI)
	{
		if (names[enumI] && names[enumI][0])
		{
			lst[nElem++] = names[enumI];
		}
	}

	lst.setSize(nElem);
	return lst;
}

template<class Enum, int nEnum>
XFoam_List<XFoam_Word> XFoam_NamedEnum<Enum, nEnum>::words()
{
	XFoam_List<XFoam_Word> lst(nEnum);

	XFoam_Label nElem = 0;
	for (unsigned int enumI = 0; enumI < static_cast<unsigned>(nEnum); ++enumI)
	{
		if (names[enumI] && names[enumI][0])
		{
			lst[nElem++] = XFoam_Word(names[enumI]);
		}
	}

	lst.setSize(nElem);
	return lst;
}

template<class Enum, int nEnum>
const char* XFoam_NamedEnum<Enum, nEnum>::operator[](const Enum e) const
{
	const unsigned ue = static_cast<unsigned>(e);

	if (ue < static_cast<unsigned>(nEnum))
	{
		return names[ue];
	}
	XFoam_FatalErrorInFunction << "names array index " << ue << " out of range 0-" << (nEnum - 1) << '\n'
							   << XFoam_exit(XFoam_FatalError, 1);
	return names[0];
}

typedef XFoam_List<XFoam_Word> XFoam_WordList;

/// 移植源码：D:\git\simulation\tmp\src\OpenFOAM\containers\Lists\UList\UList.H
/// 中的四种forAll实现，移植到XFoam中
/// 命名规范：foam_code.md
/// 移植规范：foam_code.md
#define XFoam_forAll(list, i) for (XFoam_Label i = 0; i < list.size(); ++i)
#define XFoam_forAllReverse(list, i) for (XFoam_Label i = list.size() - 1; i >= 0; --i)
#define XFoam_forAllIter(list, iter) for (auto iter = list.begin(); iter != list.end(); ++iter)
#define XFoam_forAllReverseIter(list, iter) for (auto iter = list.rbegin(); iter != list.rend(); --iter)

#endif
