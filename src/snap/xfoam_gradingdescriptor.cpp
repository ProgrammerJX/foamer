#include "XFoam/snap/xfoam_gradingdescriptor.h"
#include <cmath>

namespace
{
inline bool xf_gradingEqual(const XFoam_Scalar a, const XFoam_Scalar b)
{
	return XFoam_mag(a - b) <= XFoam_small;
}

// OpenFOAM List<gradingDescriptor> 读入：label 前缀列表或裸 '(' 列表（SLList 路径）。
void readGradingDescriptorList(XFoam_IStream& is, XFoam_List<XFoam_GradingDescriptor>& L)
{
	L.setSize(0);
	XFoam_Token firstToken(is);
	if (firstToken.isLabel())
	{
		const XFoam_Label s = firstToken.labelToken();
		L.setSize(s);
		const char delim = is.readBeginList("List<XFoam_GradingDescriptor>");
		if (s > 0)
		{
			if (delim == static_cast<char>(XFoam_Token::BEGIN_LIST))
			{
				for (XFoam_Label i = 0; i < s; ++i)
				{
					is >> L[i];
				}
			}
			else
			{
				XFoam_GradingDescriptor el;
				is >> el;
				for (XFoam_Label i = 0; i < s; ++i)
				{
					L[i] = el;
				}
			}
		}
		is.readEndList("List<XFoam_GradingDescriptor>");
	}
	else if (firstToken.isPunctuation() && firstToken.pToken() == XFoam_Token::BEGIN_LIST)
	{
		is.putBack(firstToken);
		(void)is.readBeginList("XFoam_GradingDescriptors");
		for (;;)
		{
			XFoam_Token t(is);
			if (t.good() && t.isPunctuation() && t.pToken() == XFoam_Token::END_LIST)
			{
				break;
			}
			is.putBack(t);
			L.append(XFoam_GradingDescriptor(is));
		}
	}
	else
	{
		XFoam_FatalIOErrorInFunction(XFoam_IOerrorLocation(static_cast<const XFoam_String&>(is.name())))
			<< "incorrect first token while reading XFoam_GradingDescriptor list, expected label or '('"
			<< XFoam_exit(XFoam_FatalIOError, 1);
	}
}
} // namespace

// * * * * * * * * * * * * * * * XFoam_GradingDescriptor * * * * * * * * * * * * //

XFoam_GradingDescriptor::XFoam_GradingDescriptor()
	: blockFraction_(1)
	, nDivFraction_(1)
	, expansionRatio_(1)
{}

XFoam_GradingDescriptor::XFoam_GradingDescriptor(
	const XFoam_Scalar blockFraction,
	const XFoam_Scalar nDivFraction,
	const XFoam_Scalar expansionRatio)
	: blockFraction_(blockFraction)
	, nDivFraction_(nDivFraction)
	, expansionRatio_(expansionRatio)
{}

XFoam_GradingDescriptor::XFoam_GradingDescriptor(const XFoam_Scalar expansionRatio)
	: blockFraction_(1.0)
	, nDivFraction_(1.0)
	, expansionRatio_(expansionRatio)
{}

XFoam_GradingDescriptor::XFoam_GradingDescriptor(XFoam_IStream& is)
	: blockFraction_(1)
	, nDivFraction_(1)
	, expansionRatio_(1)
{
	is >> *this;
}

XFoam_GradingDescriptor XFoam_GradingDescriptor::inv() const
{
	return XFoam_GradingDescriptor(blockFraction_, nDivFraction_, 1.0 / expansionRatio_);
}

bool XFoam_GradingDescriptor::operator==(const XFoam_GradingDescriptor& gd) const
{
	return xf_gradingEqual(blockFraction_, gd.blockFraction_) && xf_gradingEqual(nDivFraction_, gd.nDivFraction_)
		   && xf_gradingEqual(expansionRatio_, gd.expansionRatio_);
}

bool XFoam_GradingDescriptor::operator!=(const XFoam_GradingDescriptor& gd) const
{
	return !operator==(gd);
}

void XFoam_GradingDescriptor::normalizeAfterListRead_(
	const XFoam_Scalar sumBlockFraction,
	const XFoam_Scalar sumNDivFraction)
{
	blockFraction_ /= sumBlockFraction;
	nDivFraction_ /= sumNDivFraction;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_GradingDescriptor& gd)
{
	if (xf_gradingEqual(gd.blockFraction(), 1))
	{
		os << gd.expansionRatio();
	}
	else
	{
		os << static_cast<char>(XFoam_Token::BEGIN_LIST) << gd.blockFraction()
		   << static_cast<char>(XFoam_Token::SPACE) << gd.nDivFraction()
		   << static_cast<char>(XFoam_Token::SPACE) << gd.expansionRatio()
		   << static_cast<char>(XFoam_Token::END_LIST);
	}
	return os;
}

// * * * * * * * * * * * * * * * XFoam_GradingDescriptors * * * * * * * * * * * //

XFoam_GradingDescriptors::XFoam_GradingDescriptors()
	: XFoam_List<XFoam_GradingDescriptor>(1, XFoam_GradingDescriptor())
{}

XFoam_GradingDescriptors::XFoam_GradingDescriptors(const XFoam_GradingDescriptor& gd)
	: XFoam_List<XFoam_GradingDescriptor>(1, gd)
{}

XFoam_GradingDescriptors XFoam_GradingDescriptors::inv() const
{
	XFoam_GradingDescriptors ret(*this);
	for (XFoam_Label i = 0; i < ret.size(); ++i)
	{
		ret[i] = (*this)[ret.size() - i - 1].inv();
	}
	return ret;
}

XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_GradingDescriptors& gds)
{
	XFoam_Token t(is);
	if (t.isNumber())
	{
		gds = XFoam_GradingDescriptors(XFoam_GradingDescriptor(t.number()));
	}
	else
	{
		is.putBack(t);
		readGradingDescriptorList(is, static_cast<XFoam_List<XFoam_GradingDescriptor>&>(gds));
		(void)is.check("operator>>(XFoam_IStream&, XFoam_GradingDescriptors&)");
		XFoam_Scalar sumBlockFraction = 0;
		XFoam_Scalar sumNDivFraction = 0;
		for (XFoam_Label i = 0; i < gds.size(); ++i)
		{
			sumBlockFraction += gds[i].blockFraction();
			sumNDivFraction += gds[i].nDivFraction();
		}
		for (XFoam_Label i = 0; i < gds.size(); ++i)
		{
			gds[i].normalizeAfterListRead_(sumBlockFraction, sumNDivFraction);
		}
	}
	return is;
}
