#ifndef XFoam_Hash_H_
#define XFoam_Hash_H_

// Bob Jenkins lookup3（实现见 src/utilities/xfoam_hash.cpp）
// - 函数：XFoam_hashBytes、XFoam_hashWords、XFoam_hashWordsDual
//
// XFoam_HashTable：参考 OpenFOAM Foam::HashTable（HashTable.H），底层为 std::unordered_map。
// XFoam_HashSet：参考 OpenFOAM Foam::HashSet（HashSet.H），底层为 std::unordered_set。
// XFoam_Map：参考 OpenFOAM Foam::Map（Map.H），即 XFoam_Label 键的 HashTable。
// 迭代器语义接近 OF（operator* / operator() 为 T&，key() 为键）。

#include "XFoam/utilities/xfoam_types.h"

XFoam_API unsigned XFoam_hashBytes(const void* data, XFoam_Size len, unsigned seed = 0);

XFoam_API unsigned XFoam_hashWords(
	const XFoam_UInt32* data, XFoam_Size nWords, unsigned seed = 0);

XFoam_API unsigned XFoam_hashWordsDual(
	const XFoam_UInt32* data,
	XFoam_Size nWords,
	unsigned& hash1,
	unsigned& hash2);

template<class T, class Key = XFoam_String, class Hash = std::hash<Key>>
class XFoam_HashTable
{
private:
	using map_type = std::unordered_map<Key, T, Hash>;
	map_type map_;

public:
	using key_type = Key;
	using mapped_type = T;
	using hasher = Hash;
	using size_type = XFoam_Label;

	class const_iterator;

	class iterator
	{
		friend class XFoam_HashTable;
		friend class const_iterator;

	private:
		typename map_type::iterator it_;

		explicit iterator(typename map_type::iterator i)
			: it_(i)
		{}

	public:
		T& operator*() const { return it_->second; }
		T& operator()() const { return it_->second; }
		T* operator->() const { return &it_->second; }
		const Key& key() const { return it_->first; }
		iterator& operator++()
		{
			++it_;
			return *this;
		}
		iterator operator++(int)
		{
			iterator old(*this);
			++*this;
			return old;
		}
		bool operator==(const iterator& o) const { return it_ == o.it_; }
		bool operator!=(const iterator& o) const { return it_ != o.it_; }
	};

	class const_iterator
	{
		friend class XFoam_HashTable;

	private:
		typename map_type::const_iterator it_;

		explicit const_iterator(typename map_type::const_iterator i)
			: it_(i)
		{}

	public:
		const_iterator() = default;
		const_iterator(iterator i)
			: it_(i.it_)
		{}
		const T& operator*() const { return it_->second; }
		const T& operator()() const { return it_->second; }
		const T* operator->() const { return &it_->second; }
		const Key& key() const { return it_->first; }
		const_iterator& operator++()
		{
			++it_;
			return *this;
		}
		const_iterator operator++(int)
		{
			const_iterator old(*this);
			++*this;
			return old;
		}
		bool operator==(const const_iterator& o) const { return it_ == o.it_; }
		bool operator!=(const const_iterator& o) const { return it_ != o.it_; }
	};

	XFoam_HashTable() = default;

	explicit XFoam_HashTable(XFoam_Label bucketHint)
	{
		if (bucketHint > 0)
		{
			map_.reserve(static_cast<XFoam_Size>(bucketHint));
		}
	}

	XFoam_HashTable(std::initializer_list<std::pair<const Key, T>> init)
		: map_(init)
	{}

	XFoam_HashTable(const XFoam_HashTable&) = default;
	XFoam_HashTable(XFoam_HashTable&&) noexcept = default;
	~XFoam_HashTable() = default;

	XFoam_HashTable& operator=(const XFoam_HashTable& rhs)
	{
		if (this == &rhs)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_HashTable: attempted assignment to self"));
		}
		map_ = rhs.map_;
		return *this;
	}

	XFoam_HashTable& operator=(XFoam_HashTable&& rhs) noexcept
	{
		if (this != &rhs)
		{
			map_ = XFoam_move(rhs.map_);
		}
		return *this;
	}

	void operator=(std::initializer_list<std::pair<const Key, T>> lst)
	{
		map_ = map_type(lst);
	}

	size_type capacity() const
	{
		return static_cast<size_type>(map_.bucket_count());
	}

	size_type size() const { return static_cast<size_type>(map_.size()); }

	bool empty() const { return map_.empty(); }

	bool found(const Key& k) const { return map_.find(k) != map_.end(); }

	iterator find(const Key& k) { return iterator(map_.find(k)); }

	const_iterator find(const Key& k) const
	{
		return const_iterator(map_.find(k));
	}

	std::vector<Key> toc() const
	{
		std::vector<Key> out;
		out.reserve(map_.size());
		for (const auto& e : map_)
		{
			out.push_back(e.first);
		}
		return out;
	}

	std::vector<Key> sortedToc() const
	{
		std::vector<Key> out = toc();
		std::sort(out.begin(), out.end());
		return out;
	}

	bool insert(const Key& k, const T& v) { return map_.try_emplace(k, v).second; }

	void insert(const XFoam_HashTable& o)
	{
		for (const_iterator it = o.cbegin(); it != o.cend(); ++it)
		{
			insert(it.key(), *it);
		}
	}

	bool set(const Key& k, const T& v)
	{
		map_.insert_or_assign(k, v);
		return true;
	}

	void set(const XFoam_HashTable& o)
	{
		for (const_iterator it = o.cbegin(); it != o.cend(); ++it)
		{
			set(it.key(), *it);
		}
	}

	bool erase(iterator it)
	{
		if (it == end())
		{
			return false;
		}
		map_.erase(it.it_);
		return true;
	}

	bool erase(const Key& k) { return map_.erase(k) != 0u; }

	void resize(XFoam_Label newSize)
	{
		if (newSize > 0)
		{
			map_.rehash(static_cast<XFoam_Size>(newSize));
		}
	}

	void clear() { map_.clear(); }

	void clearStorage()
	{
		clear();
		map_type empty;
		map_.swap(empty);
	}

	void shrink()
	{
		map_type compact;
		compact.reserve(map_.size());
		compact.insert(map_.begin(), map_.end());
		map_.swap(compact);
	}

	void transfer(XFoam_HashTable& src)
	{
		clear();
		map_.swap(src.map_);
		src.clear();
	}

	T& operator[](const Key& k)
	{
		iterator it = find(k);
		if (it == end())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_HashTable: key not found (operator[])"));
		}
		return *it;
	}

	const T& operator[](const Key& k) const
	{
		const_iterator it = find(k);
		if (it == cend())
		{
			throw XFoam_Error(
				XFoam_String("XFoam_HashTable: key not found (const operator[])"));
		}
		return *it;
	}

	T& operator()(const Key& k) { return map_.try_emplace(k, T()).first->second; }

	bool operator==(const XFoam_HashTable& o) const
	{
		if (map_.size() != o.map_.size())
		{
			return false;
		}
		for (const auto& e : map_)
		{
			const auto it = o.map_.find(e.first);
			if (it == o.map_.end() || !(it->second == e.second))
			{
				return false;
			}
		}
		return true;
	}

	bool operator!=(const XFoam_HashTable& o) const { return !(*this == o); }

	iterator begin() { return iterator(map_.begin()); }
	iterator end() { return iterator(map_.end()); }
	const_iterator begin() const { return const_iterator(map_.cbegin()); }
	const_iterator end() const { return const_iterator(map_.cend()); }
	const_iterator cbegin() const { return const_iterator(map_.cbegin()); }
	const_iterator cend() const { return const_iterator(map_.cend()); }
};

template<class Key = XFoam_String, class Hash = std::hash<Key>>
class XFoam_HashSet
{
private:
	using set_type = std::unordered_set<Key, Hash>;
	set_type set_;

public:
	using key_type = Key;
	using hasher = Hash;
	using size_type = XFoam_Label;

	class const_iterator;

	class iterator
	{
		friend class XFoam_HashSet;
		friend class const_iterator;

	private:
		typename set_type::iterator it_;

		explicit iterator(typename set_type::iterator i)
			: it_(i)
		{}

	public:
		const Key& operator*() const { return *it_; }
		const Key& key() const { return *it_; }
		iterator& operator++()
		{
			++it_;
			return *this;
		}
		iterator operator++(int)
		{
			iterator old(*this);
			++*this;
			return old;
		}
		bool operator==(const iterator& o) const { return it_ == o.it_; }
		bool operator!=(const iterator& o) const { return it_ != o.it_; }
	};

	class const_iterator
	{
		friend class XFoam_HashSet;

	private:
		typename set_type::const_iterator it_;

		explicit const_iterator(typename set_type::const_iterator i)
			: it_(i)
		{}

	public:
		const_iterator() = default;
		const_iterator(iterator i)
			: it_(i.it_)
		{}
		const Key& operator*() const { return *it_; }
		const Key& key() const { return *it_; }
		const_iterator& operator++()
		{
			++it_;
			return *this;
		}
		const_iterator operator++(int)
		{
			const_iterator old(*this);
			++*this;
			return old;
		}
		bool operator==(const const_iterator& o) const { return it_ == o.it_; }
		bool operator!=(const const_iterator& o) const { return it_ != o.it_; }
	};

	XFoam_HashSet() = default;

	explicit XFoam_HashSet(XFoam_Label bucketHint)
	{
		if (bucketHint > 0)
		{
			set_.reserve(static_cast<XFoam_Size>(bucketHint));
		}
	}

	XFoam_HashSet(std::initializer_list<Key> init)
		: set_(init)
	{}

	template<class T, class K2, class H2>
	explicit XFoam_HashSet(const XFoam_HashTable<T, K2, H2>& ht)
	{
		for (auto it = ht.cbegin(); it != ht.cend(); ++it)
		{
			set_.insert(it.key());
		}
	}

	XFoam_HashSet(const XFoam_HashSet&) = default;
	XFoam_HashSet(XFoam_HashSet&&) noexcept = default;
	~XFoam_HashSet() = default;

	XFoam_HashSet& operator=(const XFoam_HashSet& rhs)
	{
		if (this == &rhs)
		{
			throw XFoam_Error(
				XFoam_String("XFoam_HashSet: attempted assignment to self"));
		}
		set_ = rhs.set_;
		return *this;
	}

	XFoam_HashSet& operator=(XFoam_HashSet&& rhs) noexcept
	{
		if (this != &rhs)
		{
			set_ = XFoam_move(rhs.set_);
		}
		return *this;
	}

	void operator=(std::initializer_list<Key> lst) { set_ = set_type(lst); }

	size_type capacity() const
	{
		return static_cast<size_type>(set_.bucket_count());
	}

	size_type size() const { return static_cast<size_type>(set_.size()); }

	bool empty() const { return set_.empty(); }

	bool found(const Key& k) const { return set_.find(k) != set_.end(); }

	bool operator[](const Key& k) const { return found(k); }

	iterator find(const Key& k) { return iterator(set_.find(k)); }

	const_iterator find(const Key& k) const
	{
		return const_iterator(set_.find(k));
	}

	bool insert(const Key& k) { return set_.insert(k).second; }

	size_type insert(const XFoam_HashSet& o)
	{
		size_type n = 0;
		for (const_iterator it = o.cbegin(); it != o.cend(); ++it)
		{
			if (insert(it.key()))
			{
				++n;
			}
		}
		return n;
	}

	bool set(const Key& k) { return insert(k); }

	size_type set(const XFoam_HashSet& o) { return insert(o); }

	bool unset(const Key& k) { return set_.erase(k) != 0u; }

	bool erase(const Key& k) { return unset(k); }

	bool erase(iterator it)
	{
		if (it == end())
		{
			return false;
		}
		set_.erase(it.it_);
		return true;
	}

	void resize(XFoam_Label newSize)
	{
		if (newSize > 0)
		{
			set_.rehash(static_cast<XFoam_Size>(newSize));
		}
	}

	void clear() { set_.clear(); }

	void clearStorage()
	{
		clear();
		set_type empty;
		set_.swap(empty);
	}

	void shrink()
	{
		set_type compact;
		compact.reserve(set_.size());
		compact.insert(set_.begin(), set_.end());
		set_.swap(compact);
	}

	void transfer(XFoam_HashSet& src)
	{
		clear();
		set_.swap(src.set_);
		src.clear();
	}

	void operator|=(const XFoam_HashSet& o)
	{
		set_.insert(o.set_.begin(), o.set_.end());
	}

	void operator&=(const XFoam_HashSet& o)
	{
		set_type out;
		for (const Key& k : set_)
		{
			if (o.found(k))
			{
				out.insert(k);
			}
		}
		set_.swap(out);
	}

	void operator^=(const XFoam_HashSet& o)
	{
		set_type out;
		for (const Key& k : set_)
		{
			if (!o.found(k))
			{
				out.insert(k);
			}
		}
		for (const Key& k : o.set_)
		{
			if (!found(k))
			{
				out.insert(k);
			}
		}
		set_.swap(out);
	}

	void operator-=(const XFoam_HashSet& o)
	{
		for (const Key& k : o.set_)
		{
			set_.erase(k);
		}
	}

	void operator+=(const XFoam_HashSet& rhs) { operator|=(rhs); }

	bool operator==(const XFoam_HashSet& o) const { return set_ == o.set_; }

	bool operator!=(const XFoam_HashSet& o) const { return set_ != o.set_; }

	iterator begin() { return iterator(set_.begin()); }
	iterator end() { return iterator(set_.end()); }
	const_iterator begin() const { return const_iterator(set_.cbegin()); }
	const_iterator end() const { return const_iterator(set_.cend()); }
	const_iterator cbegin() const { return const_iterator(set_.cbegin()); }
	const_iterator cend() const { return const_iterator(set_.cend()); }
};

template<class Key, class Hash>
XFoam_HashSet<Key, Hash> operator|(
	const XFoam_HashSet<Key, Hash>& a,
	const XFoam_HashSet<Key, Hash>& b)
{
	XFoam_HashSet<Key, Hash> r(a);
	r |= b;
	return r;
}

template<class Key, class Hash>
XFoam_HashSet<Key, Hash> operator&(
	const XFoam_HashSet<Key, Hash>& a,
	const XFoam_HashSet<Key, Hash>& b)
{
	XFoam_HashSet<Key, Hash> r(a);
	r &= b;
	return r;
}

template<class Key, class Hash>
XFoam_HashSet<Key, Hash> operator^(
	const XFoam_HashSet<Key, Hash>& a,
	const XFoam_HashSet<Key, Hash>& b)
{
	XFoam_HashSet<Key, Hash> r(a);
	r ^= b;
	return r;
}

using XFoam_WordHashSet = XFoam_HashSet<>;
using XFoam_LabelHashSet = XFoam_HashSet<XFoam_Label, std::hash<XFoam_Label>>;

template<class T>
class XFoam_Map : public XFoam_HashTable<T, XFoam_Label, std::hash<XFoam_Label>>
{
public:
	using XFoam_HashTable<T, XFoam_Label, std::hash<XFoam_Label>>::XFoam_HashTable;
};

#endif
