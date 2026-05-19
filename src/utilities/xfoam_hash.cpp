#include "XFoam/utilities/xfoam_hash.h"
#if defined(__GLIBC__)
#include <endian.h>
#endif

#define XFOAM_HashRotl(x, nBits) (((x) << (nBits)) | ((x) >> (32 - (nBits))))

#define XFOAM_HashMix(a, b, c)                                                    \
	do                                                                         \
	{                                                                          \
		(a) -= (c);                                                            \
		(a) ^= XFOAM_HashRotl((c), 4);                                            \
		(c) += (b);                                                            \
		(b) -= (a);                                                            \
		(b) ^= XFOAM_HashRotl((a), 6);                                            \
		(a) += (c);                                                            \
		(c) -= (b);                                                            \
		(c) ^= XFOAM_HashRotl((b), 8);                                            \
		(b) += (a);                                                            \
		(a) -= (c);                                                            \
		(a) ^= XFOAM_HashRotl((c), 16);                                           \
		(c) += (b);                                                            \
		(b) -= (a);                                                            \
		(b) ^= XFOAM_HashRotl((a), 19);                                           \
		(a) += (c);                                                            \
		(c) -= (b);                                                            \
		(c) ^= XFOAM_HashRotl((b), 4);                                            \
		(b) += (a);                                                            \
	} while (0)

#define XFOAM_HashMixFinal(a, b, c)                                               \
	do                                                                         \
	{                                                                          \
		(c) ^= (b);                                                            \
		(c) -= XFOAM_HashRotl((b), 14);                                          \
		(a) ^= (c);                                                            \
		(a) -= XFOAM_HashRotl((c), 11);                                          \
		(b) ^= (a);                                                            \
		(b) -= XFOAM_HashRotl((a), 25);                                          \
		(c) ^= (b);                                                            \
		(c) -= XFOAM_HashRotl((b), 16);                                          \
		(a) ^= (c);                                                            \
		(a) -= XFOAM_HashRotl((c), 4);                                           \
		(b) ^= (a);                                                            \
		(b) -= XFOAM_HashRotl((a), 14);                                          \
		(c) ^= (b);                                                            \
		(c) -= XFOAM_HashRotl((b), 24);                                          \
	} while (0)

static unsigned XFoam_jenkinsHashLittle(
	const void* key, XFoam_Size length, unsigned initval)
{
	XFoam_UInt32 a, b, c;
	union
	{
		const void* ptr;
		XFoam_Size i;
	} u;

	a = b = c = 0xdeadbeefU + static_cast<XFoam_UInt32>(length) + initval;
	u.ptr = key;
	if ((u.i & 0x3) == 0)
	{
		const XFoam_UInt32* k = reinterpret_cast<const XFoam_UInt32*>(key);
		while (length > 12)
		{
			a += k[0];
			b += k[1];
			c += k[2];
			XFOAM_HashMix(a, b, c);
			length -= 12;
			k += 3;
		}
		const XFoam_UInt8* k8 = reinterpret_cast<const XFoam_UInt8*>(k);
		switch (length)
		{
		case 12:
			c += k[2];
			b += k[1];
			a += k[0];
			break;
		case 11:
			c += static_cast<XFoam_UInt32>(k8[10]) << 16;
		case 10:
			c += static_cast<XFoam_UInt32>(k8[9]) << 8;
		case 9:
			c += k8[8];
		case 8:
			b += k[1];
			a += k[0];
			break;
		case 7:
			b += static_cast<XFoam_UInt32>(k8[6]) << 16;
		case 6:
			b += static_cast<XFoam_UInt32>(k8[5]) << 8;
		case 5:
			b += k8[4];
		case 4:
			a += k[0];
			break;
		case 3:
			a += static_cast<XFoam_UInt32>(k8[2]) << 16;
		case 2:
			a += static_cast<XFoam_UInt32>(k8[1]) << 8;
		case 1:
			a += k8[0];
			break;
		case 0:
			return c;
		}
	}
	else if ((u.i & 0x1) == 0)
	{
		const XFoam_UInt16* k = reinterpret_cast<const XFoam_UInt16*>(key);
		while (length > 12)
		{
			a += k[0] + (static_cast<XFoam_UInt32>(k[1]) << 16);
			b += k[2] + (static_cast<XFoam_UInt32>(k[3]) << 16);
			c += k[4] + (static_cast<XFoam_UInt32>(k[5]) << 16);
			XFOAM_HashMix(a, b, c);
			length -= 12;
			k += 6;
		}
		const XFoam_UInt8* k8 = reinterpret_cast<const XFoam_UInt8*>(k);
		switch (length)
		{
		case 12:
			c += k[4] + (static_cast<XFoam_UInt32>(k[5]) << 16);
			b += k[2] + (static_cast<XFoam_UInt32>(k[3]) << 16);
			a += k[0] + (static_cast<XFoam_UInt32>(k[1]) << 16);
			break;
		case 11:
			c += static_cast<XFoam_UInt32>(k8[10]) << 16;
		case 10:
			c += k[4];
			b += k[2] + (static_cast<XFoam_UInt32>(k[3]) << 16);
			a += k[0] + (static_cast<XFoam_UInt32>(k[1]) << 16);
			break;
		case 9:
			c += k8[8];
		case 8:
			b += k[2] + (static_cast<XFoam_UInt32>(k[3]) << 16);
			a += k[0] + (static_cast<XFoam_UInt32>(k[1]) << 16);
			break;
		case 7:
			b += static_cast<XFoam_UInt32>(k8[6]) << 16;
		case 6:
			b += k[2];
			a += k[0] + (static_cast<XFoam_UInt32>(k[1]) << 16);
			break;
		case 5:
			b += k8[4];
		case 4:
			a += k[0] + (static_cast<XFoam_UInt32>(k[1]) << 16);
			break;
		case 3:
			a += static_cast<XFoam_UInt32>(k8[2]) << 16;
		case 2:
			a += k[0];
			break;
		case 1:
			a += k8[0];
			break;
		case 0:
			return c;
		}
	}
	else
	{
		const XFoam_UInt8* k = reinterpret_cast<const XFoam_UInt8*>(key);
		while (length > 12)
		{
			a += k[0];
			a += static_cast<XFoam_UInt32>(k[1]) << 8;
			a += static_cast<XFoam_UInt32>(k[2]) << 16;
			a += static_cast<XFoam_UInt32>(k[3]) << 24;
			b += k[4];
			b += static_cast<XFoam_UInt32>(k[5]) << 8;
			b += static_cast<XFoam_UInt32>(k[6]) << 16;
			b += static_cast<XFoam_UInt32>(k[7]) << 24;
			c += k[8];
			c += static_cast<XFoam_UInt32>(k[9]) << 8;
			c += static_cast<XFoam_UInt32>(k[10]) << 16;
			c += static_cast<XFoam_UInt32>(k[11]) << 24;
			XFOAM_HashMix(a, b, c);
			length -= 12;
			k += 12;
		}
		switch (length)
		{
		case 12:
			c += static_cast<XFoam_UInt32>(k[11]) << 24;
		case 11:
			c += static_cast<XFoam_UInt32>(k[10]) << 16;
		case 10:
			c += static_cast<XFoam_UInt32>(k[9]) << 8;
		case 9:
			c += k[8];
		case 8:
			b += static_cast<XFoam_UInt32>(k[7]) << 24;
		case 7:
			b += static_cast<XFoam_UInt32>(k[6]) << 16;
		case 6:
			b += static_cast<XFoam_UInt32>(k[5]) << 8;
		case 5:
			b += k[4];
		case 4:
			a += static_cast<XFoam_UInt32>(k[3]) << 24;
		case 3:
			a += static_cast<XFoam_UInt32>(k[2]) << 16;
		case 2:
			a += static_cast<XFoam_UInt32>(k[1]) << 8;
		case 1:
			a += k[0];
			break;
		case 0:
			return c;
		}
	}
	XFOAM_HashMixFinal(a, b, c);
	return c;
}

static unsigned XFoam_jenkinsHashBig(
	const void* key, XFoam_Size length, unsigned initval)
{
	XFoam_UInt32 a, b, c;
	union
	{
		const void* ptr;
		XFoam_Size i;
	} u;

	a = b = c = 0xdeadbeefU + static_cast<XFoam_UInt32>(length) + initval;
	u.ptr = key;
	if ((u.i & 0x3) == 0)
	{
		const XFoam_UInt32* k = reinterpret_cast<const XFoam_UInt32*>(key);
		while (length > 12)
		{
			a += k[0];
			b += k[1];
			c += k[2];
			XFOAM_HashMix(a, b, c);
			length -= 12;
			k += 3;
		}
		const XFoam_UInt8* k8 = reinterpret_cast<const XFoam_UInt8*>(k);
		switch (length)
		{
		case 12:
			c += k[2];
			b += k[1];
			a += k[0];
			break;
		case 11:
			c += static_cast<XFoam_UInt32>(k8[10]) << 8;
		case 10:
			c += static_cast<XFoam_UInt32>(k8[9]) << 16;
		case 9:
			c += static_cast<XFoam_UInt32>(k8[8]) << 24;
		case 8:
			b += k[1];
			a += k[0];
			break;
		case 7:
			b += static_cast<XFoam_UInt32>(k8[6]) << 8;
		case 6:
			b += static_cast<XFoam_UInt32>(k8[5]) << 16;
		case 5:
			b += static_cast<XFoam_UInt32>(k8[4]) << 24;
		case 4:
			a += k[0];
			break;
		case 3:
			a += static_cast<XFoam_UInt32>(k8[2]) << 8;
		case 2:
			a += static_cast<XFoam_UInt32>(k8[1]) << 16;
		case 1:
			a += static_cast<XFoam_UInt32>(k8[0]) << 24;
			break;
		case 0:
			return c;
		}
	}
	else
	{
		const XFoam_UInt8* k = reinterpret_cast<const XFoam_UInt8*>(key);
		while (length > 12)
		{
			a += static_cast<XFoam_UInt32>(k[0]) << 24;
			a += static_cast<XFoam_UInt32>(k[1]) << 16;
			a += static_cast<XFoam_UInt32>(k[2]) << 8;
			a += static_cast<XFoam_UInt32>(k[3]);
			b += static_cast<XFoam_UInt32>(k[4]) << 24;
			b += static_cast<XFoam_UInt32>(k[5]) << 16;
			b += static_cast<XFoam_UInt32>(k[6]) << 8;
			b += static_cast<XFoam_UInt32>(k[7]);
			c += static_cast<XFoam_UInt32>(k[8]) << 24;
			c += static_cast<XFoam_UInt32>(k[9]) << 16;
			c += static_cast<XFoam_UInt32>(k[10]) << 8;
			c += static_cast<XFoam_UInt32>(k[11]);
			XFOAM_HashMix(a, b, c);
			length -= 12;
			k += 12;
		}
		switch (length)
		{
		case 12:
			c += k[11];
		case 11:
			c += static_cast<XFoam_UInt32>(k[10]) << 8;
		case 10:
			c += static_cast<XFoam_UInt32>(k[9]) << 16;
		case 9:
			c += static_cast<XFoam_UInt32>(k[8]) << 24;
		case 8:
			b += k[7];
		case 7:
			b += static_cast<XFoam_UInt32>(k[6]) << 8;
		case 6:
			b += static_cast<XFoam_UInt32>(k[5]) << 16;
		case 5:
			b += static_cast<XFoam_UInt32>(k[4]) << 24;
		case 4:
			a += k[3];
		case 3:
			a += static_cast<XFoam_UInt32>(k[2]) << 8;
		case 2:
			a += static_cast<XFoam_UInt32>(k[1]) << 16;
		case 1:
			a += static_cast<XFoam_UInt32>(k[0]) << 24;
			break;
		case 0:
			return c;
		}
	}
	XFOAM_HashMixFinal(a, b, c);
	return c;
}

unsigned XFoam_hashBytes(const void* data, XFoam_Size len, unsigned seed)
{
#ifdef __BYTE_ORDER
#if (__BYTE_ORDER == __BIG_ENDIAN)
	return XFoam_jenkinsHashBig(data, len, seed);
#else
	return XFoam_jenkinsHashLittle(data, len, seed);
#endif
#else
	const short endianTest = 0x0100;
	if (*(reinterpret_cast<const char*>(&endianTest)))
	{
		return XFoam_jenkinsHashBig(data, len, seed);
	}
	return XFoam_jenkinsHashLittle(data, len, seed);
#endif
}

unsigned XFoam_hashWords(const XFoam_UInt32* data, XFoam_Size nWords, unsigned seed)
{
	XFoam_UInt32 a, b, c;
	a = b = c = 0xdeadbeefU + (static_cast<XFoam_UInt32>(nWords) << 2) + seed;
	while (nWords > 3)
	{
		a += data[0];
		b += data[1];
		c += data[2];
		XFOAM_HashMix(a, b, c);
		nWords -= 3;
		data += 3;
	}
	switch (nWords)
	{
	case 3:
		c += data[2];
	case 2:
		b += data[1];
	case 1:
		a += data[0];
		XFOAM_HashMixFinal(a, b, c);
	case 0:
		break;
	}
	return c;
}

unsigned XFoam_hashWordsDual(
	const XFoam_UInt32* data,
	XFoam_Size nWords,
	unsigned& hash1,
	unsigned& hash2)
{
	XFoam_UInt32 a, b, c;
	a = b = c = 0xdeadbeefU + (static_cast<XFoam_UInt32>(nWords) << 2) + hash1;
	c += hash2;
	while (nWords > 3)
	{
		a += data[0];
		b += data[1];
		c += data[2];
		XFOAM_HashMix(a, b, c);
		nWords -= 3;
		data += 3;
	}
	switch (nWords)
	{
	case 3:
		c += data[2];
	case 2:
		b += data[1];
	case 1:
		a += data[0];
		XFOAM_HashMixFinal(a, b, c);
	case 0:
		break;
	}
	hash1 = c;
	hash2 = b;
	return c;
}

#undef XFOAM_HashMixFinal
#undef XFOAM_HashMix
#undef XFOAM_HashRotl
