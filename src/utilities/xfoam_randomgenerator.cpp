#include "XFoam/utilities/xfoam_randomgenerator.h"
#include "XFoam/utilities/xfoam_stream.h"
#if defined(_MSC_VER) && !defined(__clang__) && !defined(__INTEL_COMPILER)
#include <intrin.h>
#endif

namespace
{

constexpr XFoam_UInt64 kA = UINT64_C(0x5DEECE66D);
constexpr XFoam_UInt64 kC = UINT64_C(0xB);
constexpr XFoam_UInt64 kM = UINT64_C(1) << 48;
constexpr XFoam_UInt64 kMask = kM - UINT64_C(1);

XFoam_UInt64 lcgm()
{
	return kM;
}

XFoam_UInt64 lcgmMask()
{
	return kMask;
}

XFoam_UInt64 mulAModM(XFoam_UInt64 x)
{
	x &= kMask;
	// (kA*x) 真值可达约 2^84；取其对 2^48 的同余只需 (真值 mod 2^64) mod 2^48，与 unsigned uint64_t 溢出语义一致。
#if defined(_MSC_VER) && !defined(__clang__) && !defined(__INTEL_COMPILER)
	XFoam_UInt64 hi = 0;
	const XFoam_UInt64 lo = _umul128(kA, x, &hi);
	(void)hi;
	return lo & kMask;
#else
	return (kA * x) & kMask;
#endif
}

XFoam_UInt64 advanceState(XFoam_UInt64 x)
{
	x &= kMask;
	const XFoam_UInt64 y = mulAModM(x);
	return (y + kC) & kMask;
}

XFoam_UInt64 labelSeedToState48(XFoam_Label s)
{
	const auto u = static_cast<XFoam_UInt64>(static_cast<XFoam_UInt32>(s));
	return ((u << 16) | UINT64_C(0x330E)) & kMask;
}

XFoam_UInt64 wordSeedToState48(const XFoam_String& w)
{
	XFoam_UInt64 u = static_cast<XFoam_UInt64>(std::hash<XFoam_String>{}(w));
	u ^= u >> 32;
	u ^= u << 17;
	u &= kMask;
	if (u == 0)
	{
		u = UINT64_C(0x330E);
	}
	return u;
}

} // namespace

void XFoam_RandomGenerator::checkSync() const
{
	(void)global_;
}

XFoam_RandomGenerator::Seed::Seed(XFoam_Label s)
	: x0_(labelSeedToState48(s))
{}

XFoam_RandomGenerator::Seed::Seed(const XFoam_String& w)
	: x0_(wordSeedToState48(w))
{}

XFoam_RandomGenerator::XFoam_RandomGenerator()
	: global_(false)
	, x_(labelSeedToState48(0))
{}

XFoam_RandomGenerator::XFoam_RandomGenerator(Seed s, bool global)
	: global_(global)
	, x_(s.x0_)
{
	checkSync();
}

XFoam_RandomGenerator::XFoam_RandomGenerator(XFoam_Label labelSeed, bool global)
	: XFoam_RandomGenerator(Seed(labelSeed), global)
{}

XFoam_RandomGenerator::XFoam_RandomGenerator(const XFoam_RandomGenerator&) = default;
XFoam_RandomGenerator::XFoam_RandomGenerator(XFoam_RandomGenerator&&) noexcept = default;
XFoam_RandomGenerator& XFoam_RandomGenerator::operator=(const XFoam_RandomGenerator&) = default;
XFoam_RandomGenerator& XFoam_RandomGenerator::operator=(XFoam_RandomGenerator&&) noexcept = default;

bool XFoam_RandomGenerator::global() const noexcept
{
	return global_;
}

XFoam_UInt64 XFoam_RandomGenerator::rawState() const noexcept
{
	return x_;
}

void XFoam_RandomGenerator::setRawState(XFoam_UInt64 state)
{
	x_ = state & lcgmMask();
	checkSync();
}

XFoam_UInt64 XFoam_RandomGenerator::sample()
{
	x_ = advanceState(x_);
	return x_;
}

XFoam_Scalar XFoam_RandomGenerator::scalar01NoCheckSync()
{
	const XFoam_UInt64 s = sample();
	return static_cast<XFoam_Scalar>(s) / static_cast<XFoam_Scalar>(lcgm());
}

XFoam_Scalar XFoam_RandomGenerator::scalar01()
{
	checkSync();
	return scalar01NoCheckSync();
}

XFoam_Scalar XFoam_RandomGenerator::scalarABNoCheckSync(XFoam_Scalar a, XFoam_Scalar b)
{
	return a + scalar01NoCheckSync() * (b - a);
}

XFoam_Scalar XFoam_RandomGenerator::scalarAB(XFoam_Scalar a, XFoam_Scalar b)
{
	checkSync();
	return scalarABNoCheckSync(a, b);
}

XFoam_Label XFoam_RandomGenerator::integerAB(XFoam_Label a, XFoam_Label b)
{
	const XFoam_Label lo = XFoam_min(a, b);
	const XFoam_Label hi = XFoam_max(a, b);
	const XFoam_Label span = hi - lo + 1;
	const XFoam_Scalar t = scalar01();
	const XFoam_Label off =
		static_cast<XFoam_Label>(std::floor(t * static_cast<XFoam_Scalar>(span)));
	return lo + off;
}

XFoam_RandomGenerator XFoam_RandomGenerator::generator()
{
	std::random_device rd;
	XFoam_UInt64 u =
		(static_cast<XFoam_UInt64>(rd()) << 32) ^ static_cast<XFoam_UInt64>(rd());
	u = (u ^ (u >> 16) ^ (u << 32)) & lcgmMask();
	if (u == 0)
	{
		u = UINT64_C(0x330E);
	}
	XFoam_RandomGenerator r;
	r.setRawState(u);
	return r;
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_RandomGenerator& g)
{
	g.checkSync();
	return os << g.x_;
}

XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_RandomGenerator& g)
{
	is >> g.x_;
	g.x_ &= lcgmMask();
	g.checkSync();
	return is;
}
