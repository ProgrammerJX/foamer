#ifndef XFoam_Utilities_Randomgenerator_H_
#define XFoam_Utilities_Randomgenerator_H_

// 对齐 OpenFOAM primitives/randomGenerator（drand48 线性同余：a=0x5DEECE66D, c=0xB, m=2^48）。
// 实现见 xfoam_randomgenerator.cpp。

#include "XFoam/utilities/xfoam_types.h"

class XFoam_API XFoam_RandomGenerator
{
	bool global_;
	XFoam_UInt64 x_;

	void checkSync() const;

public:
	class XFoam_API Seed
	{
		friend class XFoam_RandomGenerator;
		XFoam_UInt64 x0_;

	public:
		explicit Seed(XFoam_Label s);
		explicit Seed(const XFoam_String& w);
	};

	XFoam_RandomGenerator();
	explicit XFoam_RandomGenerator(Seed s, bool global = false);
	explicit XFoam_RandomGenerator(XFoam_Label labelSeed, bool global = false);

	XFoam_RandomGenerator(const XFoam_RandomGenerator&);
	XFoam_RandomGenerator(XFoam_RandomGenerator&&) noexcept;
	XFoam_RandomGenerator& operator=(const XFoam_RandomGenerator&);
	XFoam_RandomGenerator& operator=(XFoam_RandomGenerator&&) noexcept;

	bool global() const noexcept;

	XFoam_UInt64 rawState() const noexcept;

	void setRawState(XFoam_UInt64 state);

	XFoam_UInt64 sample();

	XFoam_Scalar scalar01NoCheckSync();

	XFoam_Scalar scalar01();

	XFoam_Scalar scalarABNoCheckSync(XFoam_Scalar a, XFoam_Scalar b);

	XFoam_Scalar scalarAB(XFoam_Scalar a, XFoam_Scalar b);

	XFoam_Label integerAB(XFoam_Label a, XFoam_Label b);

	template<class Container>
	void permute(Container& c);

	static XFoam_RandomGenerator generator();

	friend XFoam_API XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_RandomGenerator& g);
	friend XFoam_API XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_RandomGenerator& g);
};

template<class Container>
void XFoam_RandomGenerator::permute(Container& c)
{
	using std::swap;
	const auto n = static_cast<XFoam_Label>(c.size());
	for (XFoam_Label i = n - 1; i > 0; --i)
	{
		const XFoam_Label j = integerAB(0, i);
		swap(c[static_cast<XFoam_Size>(i)], c[static_cast<XFoam_Size>(j)]);
	}
}

#endif
