#include "XFoam/utilities/xfoam_list.h"

namespace xfoam_link_base_detail
{
static XFoam_DLListBase dlEndList_;
static XFoam_SLListBase slEndList_;
}

XFoam_DLListBase::iterator XFoam_DLListBase::endIter_(
	const_cast<XFoam_DLListBase&>(static_cast<const XFoam_DLListBase&>(xfoam_link_base_detail::dlEndList_)));

XFoam_DLListBase::const_iterator XFoam_DLListBase::endConstIter_(
	static_cast<const XFoam_DLListBase&>(xfoam_link_base_detail::dlEndList_),
	reinterpret_cast<const XFoam_DLListBase::link*>(0));

XFoam_DLListBase::const_reverse_iterator XFoam_DLListBase::endConstRevIter_(
	static_cast<const XFoam_DLListBase&>(xfoam_link_base_detail::dlEndList_),
	reinterpret_cast<const XFoam_DLListBase::link*>(0));

XFoam_SLListBase::iterator XFoam_SLListBase::endIter_(
	const_cast<XFoam_SLListBase&>(static_cast<const XFoam_SLListBase&>(xfoam_link_base_detail::slEndList_)));

XFoam_SLListBase::const_iterator XFoam_SLListBase::endConstIter_(
	static_cast<const XFoam_SLListBase&>(xfoam_link_base_detail::slEndList_),
	reinterpret_cast<const XFoam_SLListBase::link*>(0));

void XFoam_DLListBase::insert(XFoam_DLListBase::link* a)
{
	nElmts_++;

	if (!first_)
	{
		a->prev_ = a;
		a->next_ = a;
		first_ = last_ = a;
	}
	else
	{
		a->prev_ = a;
		a->next_ = first_;
		first_->prev_ = a;
		first_ = a;
	}
}

void XFoam_DLListBase::append(XFoam_DLListBase::link* a)
{
	nElmts_++;

	if (!first_)
	{
		a->prev_ = a;
		a->next_ = a;
		first_ = last_ = a;
	}
	else
	{
		last_->next_ = a;
		a->prev_ = last_;
		a->next_ = a;
		last_ = a;
	}
}

bool XFoam_DLListBase::swapUp(XFoam_DLListBase::link* a)
{
	if (first_ != a)
	{
		link* ap = a->prev_;

		if (ap == first_)
		{
			first_ = a;
			ap->prev_ = a;
		}
		else
		{
			ap->prev_->next_ = a;
		}

		if (a == last_)
		{
			last_ = ap;
			a->next_ = ap;
		}
		else
		{
			a->next_->prev_ = ap;
		}

		a->prev_ = ap->prev_;
		ap->prev_ = a;

		ap->next_ = a->next_;
		a->next_ = ap;

		return true;
	}
	return false;
}

bool XFoam_DLListBase::swapDown(XFoam_DLListBase::link* a)
{
	if (last_ != a)
	{
		link* an = a->next_;

		if (a == first_)
		{
			first_ = an;
			a->prev_ = an;
		}
		else
		{
			a->prev_->next_ = an;
		}

		if (an == last_)
		{
			last_ = a;
			an->next_ = a;
		}
		else
		{
			an->next_->prev_ = a;
		}

		an->prev_ = a->prev_;
		a->prev_ = an;

		a->next_ = an->next_;
		an->next_ = a;

		return true;
	}
	return false;
}

XFoam_DLListBase::link* XFoam_DLListBase::removeHead()
{
	nElmts_--;

	if (!first_)
	{
		throw XFoam_Error(XFoam_String("XFoam_DLListBase::removeHead: remove from empty list"));
	}

	link* f = first_;
	first_ = f->next_;

	if (!first_)
	{
		last_ = nullptr;
	}

	f->deregister();
	return f;
}

XFoam_DLListBase::link* XFoam_DLListBase::remove(XFoam_DLListBase::link* l)
{
	nElmts_--;

	link* ret = l;

	if (l == first_ && first_ == last_)
	{
		first_ = nullptr;
		last_ = nullptr;
	}
	else if (l == first_)
	{
		first_ = first_->next_;
		first_->prev_ = first_;
	}
	else if (l == last_)
	{
		last_ = last_->prev_;
		last_->next_ = last_;
	}
	else
	{
		l->next_->prev_ = l->prev_;
		l->prev_->next_ = l->next_;
	}

	ret->deregister();
	return ret;
}

XFoam_DLListBase::link* XFoam_DLListBase::replace(XFoam_DLListBase::link* oldLink, XFoam_DLListBase::link* newLink)
{
	link* ret = oldLink;

	newLink->prev_ = oldLink->prev_;
	newLink->next_ = oldLink->next_;

	if (oldLink == first_ && first_ == last_)
	{
		first_ = newLink;
		last_ = newLink;
	}
	else if (oldLink == first_)
	{
		first_ = newLink;
		newLink->next_->prev_ = newLink;
	}
	else if (oldLink == last_)
	{
		last_ = newLink;
		newLink->prev_->next_ = newLink;
	}
	else
	{
		newLink->prev_->next_ = newLink;
		newLink->next_->prev_ = newLink;
	}

	ret->deregister();
	return ret;
}

void XFoam_SLListBase::insert(XFoam_SLListBase::link* a)
{
	nElmts_++;

	if (last_)
	{
		a->next_ = last_->next_;
	}
	else
	{
		last_ = a;
	}

	last_->next_ = a;
}

void XFoam_SLListBase::append(XFoam_SLListBase::link* a)
{
	nElmts_++;

	if (last_)
	{
		a->next_ = last_->next_;
		last_ = last_->next_ = a;
	}
	else
	{
		last_ = a->next_ = a;
	}
}

XFoam_SLListBase::link* XFoam_SLListBase::removeHead()
{
	nElmts_--;

	if (last_ == nullptr)
	{
		throw XFoam_Error(XFoam_String("XFoam_SLListBase::removeHead: remove from empty list"));
	}

	link* f = last_->next_;

	if (f == last_)
	{
		last_ = nullptr;
	}
	else
	{
		last_->next_ = f->next_;
	}

	return f;
}

XFoam_SLListBase::link* XFoam_SLListBase::remove(XFoam_SLListBase::link* it)
{
	iterator iter = begin();
	link* prev = &(*iter);

	if (it == prev)
	{
		return removeHead();
	}

	nElmts_--;

	for (++iter; iter != end(); ++iter)
	{
		link* p = &(*iter);

		if (p == it)
		{
			prev->next_ = p->next_;

			if (p == last_)
			{
				last_ = prev;
			}

			return it;
		}

		prev = p;
	}

	return nullptr;
}
