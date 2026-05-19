#ifndef XFoam_Circulator_H_
#define XFoam_Circulator_H_

// 环形遍历容器，对齐 OpenFOAM CirculatorBase / Circulator / ConstCirculator
// （tmp/.../containers/Circulators）。
// ContainerType 须具备：value_type, size_type, difference_type,
// iterator / const_iterator, reference / const_reference，以及 begin()/end()。

/// 环形迭代器公共基类。
class XFoam_CirculatorBase
{
public:
	enum class direction
	{
		none,
		clockwise,
		anticlockwise
	};

	XFoam_CirculatorBase() = default;
};

/// 可修改元素的环形迭代器。
template<class ContainerType>
class XFoam_Circulator : public XFoam_CirculatorBase
{
protected:
	typename ContainerType::iterator begin_;
	typename ContainerType::iterator end_;
	typename ContainerType::iterator iter_;
	typename ContainerType::iterator fulcrum_;

public:
	typedef typename ContainerType::value_type value_type;
	typedef typename ContainerType::size_type size_type;
	typedef typename ContainerType::difference_type difference_type;
	typedef typename ContainerType::iterator iterator;
	typedef typename ContainerType::reference reference;

	XFoam_Circulator()
		: begin_{}
		, end_{}
		, iter_{}
		, fulcrum_{}
	{}

	explicit XFoam_Circulator(ContainerType& container)
		: begin_(container.begin())
		, end_(container.end())
		, iter_(begin_)
		, fulcrum_(begin_)
	{}

	XFoam_Circulator(const iterator& begin, const iterator& end)
		: begin_(begin)
		, end_(end)
		, iter_(begin)
		, fulcrum_(begin)
	{}

	XFoam_Circulator(const XFoam_Circulator& rhs) = default;

	~XFoam_Circulator() = default;

	size_type size() const { return static_cast<size_type>(end_ - begin_); }

	bool circulate(const XFoam_CirculatorBase::direction dir = XFoam_CirculatorBase::direction::none)
	{
		if (dir == XFoam_CirculatorBase::direction::clockwise)
		{
			operator++();
		}
		else if (dir == XFoam_CirculatorBase::direction::anticlockwise)
		{
			operator--();
		}
		return !(iter_ == fulcrum_);
	}

	void setFulcrumToIterator() { fulcrum_ = iter_; }

	void setIteratorToFulcrum() { iter_ = fulcrum_; }

	difference_type nRotations() const { return iter_ - fulcrum_; }

	reference next() const
	{
		if (iter_ == end_ - 1)
		{
			return *begin_;
		}
		return *(iter_ + 1);
	}

	reference prev() const
	{
		if (iter_ == begin_)
		{
			return *(end_ - 1);
		}
		return *(iter_ - 1);
	}

	void operator=(const XFoam_Circulator& rhs)
	{
		if (this == &rhs)
		{
			return;
		}
		begin_ = rhs.begin_;
		end_ = rhs.end_;
		iter_ = rhs.iter_;
		fulcrum_ = rhs.fulcrum_;
	}

	XFoam_Circulator& operator++()
	{
		++iter_;
		if (iter_ == end_)
		{
			iter_ = begin_;
		}
		return *this;
	}

	XFoam_Circulator operator++(int)
	{
		XFoam_Circulator tmp = *this;
		++(*this);
		return tmp;
	}

	XFoam_Circulator& operator--()
	{
		if (iter_ == begin_)
		{
			iter_ = end_;
		}
		--iter_;
		return *this;
	}

	XFoam_Circulator operator--(int)
	{
		XFoam_Circulator tmp = *this;
		--(*this);
		return tmp;
	}

	bool operator==(const XFoam_Circulator& c) const
	{
		return begin_ == c.begin_ && end_ == c.end_ && iter_ == c.iter_ && fulcrum_ == c.fulcrum_;
	}

	bool operator!=(const XFoam_Circulator& c) const { return !(*this == c); }

	reference operator*() const { return *iter_; }

	reference operator()() const { return operator*(); }

	difference_type operator-(const XFoam_Circulator& c) const { return iter_ - c.iter_; }
};

/// 只读环形迭代器。
template<class ContainerType>
class XFoam_ConstCirculator : public XFoam_CirculatorBase
{
protected:
	typename ContainerType::const_iterator begin_;
	typename ContainerType::const_iterator end_;
	typename ContainerType::const_iterator iter_;
	typename ContainerType::const_iterator fulcrum_;

public:
	typedef typename ContainerType::value_type value_type;
	typedef typename ContainerType::size_type size_type;
	typedef typename ContainerType::difference_type difference_type;
	typedef typename ContainerType::const_iterator const_iterator;
	typedef typename ContainerType::const_reference const_reference;

	XFoam_ConstCirculator()
		: begin_{}
		, end_{}
		, iter_{}
		, fulcrum_{}
	{}

	explicit XFoam_ConstCirculator(const ContainerType& container)
		: begin_(container.begin())
		, end_(container.end())
		, iter_(begin_)
		, fulcrum_(begin_)
	{}

	XFoam_ConstCirculator(const const_iterator& begin, const const_iterator& end)
		: begin_(begin)
		, end_(end)
		, iter_(begin)
		, fulcrum_(begin)
	{}

	XFoam_ConstCirculator(const XFoam_ConstCirculator& rhs) = default;

	~XFoam_ConstCirculator() = default;

	size_type size() const { return static_cast<size_type>(end_ - begin_); }

	bool circulate(const XFoam_CirculatorBase::direction dir = XFoam_CirculatorBase::direction::none)
	{
		if (dir == XFoam_CirculatorBase::direction::clockwise)
		{
			operator++();
		}
		else if (dir == XFoam_CirculatorBase::direction::anticlockwise)
		{
			operator--();
		}
		return !(iter_ == fulcrum_);
	}

	void setFulcrumToIterator() { fulcrum_ = iter_; }

	void setIteratorToFulcrum() { iter_ = fulcrum_; }

	difference_type nRotations() const { return iter_ - fulcrum_; }

	const_reference next() const
	{
		if (iter_ == end_ - 1)
		{
			return *begin_;
		}
		return *(iter_ + 1);
	}

	const_reference prev() const
	{
		if (iter_ == begin_)
		{
			return *(end_ - 1);
		}
		return *(iter_ - 1);
	}

	void operator=(const XFoam_ConstCirculator& rhs)
	{
		if (this == &rhs)
		{
			return;
		}
		begin_ = rhs.begin_;
		end_ = rhs.end_;
		iter_ = rhs.iter_;
		fulcrum_ = rhs.fulcrum_;
	}

	XFoam_ConstCirculator& operator++()
	{
		++iter_;
		if (iter_ == end_)
		{
			iter_ = begin_;
		}
		return *this;
	}

	XFoam_ConstCirculator operator++(int)
	{
		XFoam_ConstCirculator tmp = *this;
		++(*this);
		return tmp;
	}

	XFoam_ConstCirculator& operator--()
	{
		if (iter_ == begin_)
		{
			iter_ = end_;
		}
		--iter_;
		return *this;
	}

	XFoam_ConstCirculator operator--(int)
	{
		XFoam_ConstCirculator tmp = *this;
		--(*this);
		return tmp;
	}

	bool operator==(const XFoam_ConstCirculator& c) const
	{
		return begin_ == c.begin_ && end_ == c.end_ && iter_ == c.iter_ && fulcrum_ == c.fulcrum_;
	}

	bool operator!=(const XFoam_ConstCirculator& c) const { return !(*this == c); }

	const_reference operator*() const { return *iter_; }

	const_reference operator()() const { return operator*(); }

	difference_type operator-(const XFoam_ConstCirculator& c) const { return iter_ - c.iter_; }
};

#endif
