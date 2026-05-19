#ifndef XFoam_PointIndexHit_H_
#define XFoam_PointIndexHit_H_

// 对标 OpenFOAM meshes/primitiveShapes/objectHit/PointIndexHit.H 与 pointIndexHit.H
//（Foam::PointIndexHit<Foam::point> / Foam::pointIndexHit）。
// 未移植：Ostream/Istream 的 BINARY 格式与 os.check/is.check；无并发相关逻辑可屏蔽。
// 未移植：从 IOdictionary 等字典构造的路径（本类无字典构造；Istream 构造仅按流读取）。
// hitPoint()/missPoint() 在 OF 中非法访问为 FatalError；此处标注并返回空向量引用。

#include "XFoam/utilities/xfoam_types.h"
#include "XFoam/utilities/xfoam_vector.h"

/*---------------------------------------------------------------------------*\
                        Class XFoam_PointIndexHit Declaration
\*---------------------------------------------------------------------------*/

class XFoam_API XFoam_PointIndexHit
{
	// Private Data

		//- Hit success
		bool hit_;

		//- Point of hit; invalid for misses
		XFoam_Vector3D hitPoint_;

		//- Label of face hit
		XFoam_Label index_;


public:

	// Constructors

		//- Construct from components
		XFoam_PointIndexHit(const bool success, const XFoam_Vector3D& p, const XFoam_Label index)
			: hit_(success)
			, hitPoint_(p)
			, index_(index)
		{}

		//- Construct from point. Hit and distance set later
		XFoam_PointIndexHit(const XFoam_Vector3D& p)
			: hit_(false)
			, hitPoint_(p)
			, index_(-1)
		{}

		//- Construct null
		XFoam_PointIndexHit()
			: hit_(false)
			, hitPoint_(XFoam_Zero_v)
			, index_(-1)
		{}

		//- Construct from Istream
		XFoam_PointIndexHit(XFoam_IStream& is)
		{
			is >> *this;
		}


	// Member Functions

		//- Is there a hit
		bool hit() const
		{
			return hit_;
		}

		//- Return index
		XFoam_Label index() const
		{
			return index_;
		}

		//- Return hit point
		const XFoam_Vector3D& hitPoint() const
		{
			if (!hit_)
			{
				// 未移植：OF FatalErrorInFunction；返回空向量。
				static const XFoam_Vector3D s_empty(XFoam_Zero_v);
				return s_empty;
			}

			return hitPoint_;
		}

		//- Return miss point
		const XFoam_Vector3D& missPoint() const
		{
			if (hit_)
			{
				// 未移植：OF FatalErrorInFunction；返回空向量。
				static const XFoam_Vector3D s_empty(XFoam_Zero_v);
				return s_empty;
			}

			return hitPoint_;
		}

		//- Return point with no checking
		const XFoam_Vector3D& rawPoint() const
		{
			return hitPoint_;
		}

		XFoam_Vector3D& rawPoint()
		{
			return hitPoint_;
		}

		void setHit()
		{
			hit_ = true;
		}

		void setMiss()
		{
			hit_ = false;
		}

		void setPoint(const XFoam_Vector3D& p)
		{
			hitPoint_ = p;
		}

		void setIndex(const XFoam_Label index)
		{
			index_ = index;
		}

		bool operator==(const XFoam_PointIndexHit& rhs) const
		{
			return hit_ == rhs.hit() && hitPoint_ == rhs.rawPoint() && index_ == rhs.index();
		}

		bool operator!=(const XFoam_PointIndexHit& rhs) const
		{
			return !operator==(rhs);
		}

		void write(XFoam_OStream& os)
		{
			if (hit())
			{
				os << "hit:" << hitPoint() << " index:" << index();
			}
			else
			{
				os << "miss:" << missPoint() << " index:" << index();
			}
		}

		friend XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_PointIndexHit& pHit)
		{
			// 未移植：IOstream::BINARY 与 raw write；仅 ASCII 路径。
			os << pHit.hit_ << ' ' << pHit.hitPoint_ << ' ' << pHit.index_;
			return os;
		}

		friend XFoam_IStream& operator>>(XFoam_IStream& is, XFoam_PointIndexHit& pHit)
		{
			// 未移植：BINARY 与 is.check；仅 ASCII 路径。
			int32_t hitFlag = 0;
			is >> hitFlag >> pHit.hitPoint_ >> pHit.index_;
			pHit.hit_ = (hitFlag != 0);
			return is;
		}
};


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
// pointIndexHit.H: contiguous<pointIndexHit> 依赖 contiguous<point>()
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

template<class T>
inline bool contiguous()
{
	return false;
}

template<>
inline bool contiguous<XFoam_Vector3D>()
{
	return true;
}

template<>
inline bool contiguous<XFoam_PointIndexHit>()
{
	return contiguous<XFoam_Vector3D>();
}

#endif

// ************************************************************************* //
