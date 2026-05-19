#ifndef XFoam_Gradingdescriptor_H_
#define XFoam_Gradingdescriptor_H_

// 对标 OpenFOAM src/mesh/blockMesh/gradingDescriptor/gradingDescriptor.H、gradingDescriptors.H
// 实现合并于本头文件与 xfoam_gradingdescriptor.cpp（见 doc/foam_code.md）。

#include "XFoam/utilities/xfoam_common.h"

/*---------------------------------------------------------------------------*\
                        Class XFoam_GradingDescriptor Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_GradingDescriptor
{
	XFoam_Scalar blockFraction_;
	XFoam_Scalar nDivFraction_;
	XFoam_Scalar expansionRatio_;

public:
	friend inline XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_GradingDescriptor& gd)
	{
		XFoam_Token t(is);
		if (t.isNumber())
		{
			gd.blockFraction_ = 1.0;
			gd.nDivFraction_ = 1.0;
			gd.expansionRatio_ = t.number();
		}
		else if (t.good() && t.isPunctuation() && t.pToken() == XFoam_Token::BEGIN_LIST)
		{
			is >> gd.blockFraction_ >> gd.nDivFraction_ >> gd.expansionRatio_;
			is.readEnd("XFoam_GradingDescriptor");
		}
		(void)is.check("operator>>(XFoam_IStream&, XFoam_GradingDescriptor&)");
		return is;
	}

	XFoam_GradingDescriptor();

	XFoam_GradingDescriptor(
		const XFoam_Scalar blockFraction,
		const XFoam_Scalar nDivFraction,
		const XFoam_Scalar expansionRatio);

	explicit XFoam_GradingDescriptor(const XFoam_Scalar expansionRatio);

	explicit XFoam_GradingDescriptor(XFoam_IStream& is);

	~XFoam_GradingDescriptor() = default;

	XFoam_Scalar blockFraction() const { return blockFraction_; }
	XFoam_Scalar nDivFraction() const { return nDivFraction_; }
	XFoam_Scalar expansionRatio() const { return expansionRatio_; }

	XFoam_GradingDescriptor inv() const;

	bool operator==(const XFoam_GradingDescriptor& gd) const;
	bool operator!=(const XFoam_GradingDescriptor& gd) const;

	// gradingDescriptors::operator>> 在读完列表后对 blockFraction_/nDivFraction_ 归一化（与 OF 一致）。
	void normalizeAfterListRead_(XFoam_Scalar sumBlockFraction, XFoam_Scalar sumNDivFraction);
};

XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_GradingDescriptor& gd);

/*---------------------------------------------------------------------------*\
                        Class XFoam_GradingDescriptors Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_GradingDescriptors
	: public XFoam_List<XFoam_GradingDescriptor>
{
public:
	XFoam_GradingDescriptors();

	explicit XFoam_GradingDescriptors(const XFoam_GradingDescriptor& gd);

	XFoam_GradingDescriptors inv() const;

	friend XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_GradingDescriptors& gds);
};

#endif
