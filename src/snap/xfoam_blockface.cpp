#include "XFoam/snap/xfoam_blockface.h"
#include "XFoam/snap/xfoam_blockvert.h"
#include "XFoam/snap/xfoam_block.h"
#include <cctype>
#include <string>

namespace
{
void skipWs(XFoam_IStream& is)
{
	while (is.good())
	{
		const int ch = is.peek();
		if (ch == std::char_traits<char>::eof())
		{
			break;
		}
		if (!std::isspace(static_cast<unsigned char>(ch)))
		{
			break;
		}
		is.get();
	}
}

XFoam_Face readFaceFromStream(XFoam_IStream& is, const XFoam_Dictionary&)
{
	char c = 0;
	skipWs(is);
	if (!(is.get(c)))
	{
		return XFoam_Face();
	}
	if (c == '(')
	{
		XFoam_LabelList buf;
		for (;;)
		{
			skipWs(is);
			if (!(is.get(c)))
			{
				break;
			}
			if (c == ')')
			{
				break;
			}
			is.putback(c);
			XFoam_Label v = 0;
			is >> v;
			buf.append(v);
		}
		return XFoam_Face(buf);
	}
	is.putback(c);
	XFoam_Label n = 0;
	is >> n;
	XFoam_LabelList buf(n);
	for (XFoam_Label i = 0; i < n; ++i)
	{
		is >> buf[i];
	}
	return XFoam_Face(buf);
}

void linearInterpolationWeightsValue(
	const XFoam_Field<XFoam_Scalar>& knots,
	const XFoam_Scalar lambda,
	XFoam_LabelList& indices,
	XFoam_Field<XFoam_Scalar>& weights)
{
	const XFoam_Label n = knots.size();
	if (n < 2)
	{
		indices.setSize(0);
		weights.setSize(0);
		return;
	}
	if (lambda <= knots[0])
	{
		indices.setSize(2);
		weights.setSize(2);
		indices[0] = 0;
		indices[1] = 1;
		weights[0] = 1;
		weights[1] = 0;
		return;
	}
	if (lambda >= knots[n - 1])
	{
		indices.setSize(2);
		weights.setSize(2);
		indices[0] = n - 2;
		indices[1] = n - 1;
		weights[0] = 0;
		weights[1] = 1;
		return;
	}
	XFoam_Label k = 1;
	for (; k < n; ++k)
	{
		if (lambda < knots[k])
		{
			break;
		}
	}
	--k;
	const XFoam_Scalar span = knots[k + 1] - knots[k];
	const XFoam_Scalar s =
		span > XFoam_small ? (lambda - knots[k]) / span : XFoam_Scalar(0.5);
	indices.setSize(2);
	weights.setSize(2);
	indices[0] = k;
	indices[1] = k + 1;
	weights[0] = 1 - s;
	weights[1] = s;
}

void fieldAddAssign(
	XFoam_Field<XFoam_Vector3D>& a, const XFoam_Field<XFoam_Vector3D>& b)
{
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		a[i] += b[i];
	}
}

void fieldZero(XFoam_Field<XFoam_Vector3D>& a)
{
	for (XFoam_Label i = 0; i < a.size(); ++i)
	{
		a[i] = XFoam_Vector3D(0, 0, 0);
	}
}
} // namespace

XFoam_BlockFace::XFoam_BlockFace(const XFoam_Face& vertices)
	: vertices_(vertices)
{}

XFoam_BlockFace::XFoam_BlockFace(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	XFoam_IStream& is)
	: vertices_(readFaceFromStream(is, dict))
{
	(void)index;
}

XFoam_BlockFace::~XFoam_BlockFace() = default;

void XFoam_BlockFace::write(XFoam_OStream& os, const XFoam_Dictionary& d) const
{
	os << vertices_.size() << ' ' << '(';
	for (XFoam_Label i = 0; i < vertices_.size(); ++i)
	{
		if (i > 0)
		{
			os << ' ';
		}
		XFoam_BlockVert::write(os, vertices_[i], d);
	}
	os << ')';
}

XFoam_AutoPtr<XFoam_BlockFace> XFoam_BlockFace::clone() const
{
	return XFoam_AutoPtr<XFoam_BlockFace>();
}

XFoam_AutoPtr<XFoam_BlockFace> XFoam_BlockFace::New(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	const XFoam_SearchableSurfaceList& geometry,
	XFoam_IStream& is)
{
	XFoam_String faceType;
	is >> faceType;
	if (faceType == "plane")
	{
		return XFoam_AutoPtr<XFoam_BlockFace>(
			new XFoam_blockFaces::XFoam_PlaneFace(dict, index, is));
	}
	if (faceType == "project")
	{
		return XFoam_AutoPtr<XFoam_BlockFace>(
			new XFoam_blockFaces::XFoam_BlockProjectFace(dict, index, geometry, is));
	}
	XFoam_FatalErrorInFunction
		<< "Unknown blockFace type " << faceType << "\nValid blockFace types are: plane project\n"
		<< XFoam_abort(XFoam_FatalError);
	return XFoam_AutoPtr<XFoam_BlockFace>();
}

XFoam_OStream& operator<<(XFoam_OStream& os, const XFoam_BlockFace& p)
{
	const XFoam_Face& f = p.vertices_;
	os << f.size() << ' ' << '(';
	for (XFoam_Label i = 0; i < f.size(); ++i)
	{
		if (i > 0)
		{
			os << ' ';
		}
		os << f[i];
	}
	os << ')' << '\n';
	return os;
}

namespace XFoam_blockFaces
{
XFoam_PlaneFace::XFoam_PlaneFace(const XFoam_Face& vertices)
	: XFoam_BlockFace(vertices)
{}

XFoam_PlaneFace::XFoam_PlaneFace(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	XFoam_IStream& is)
	: XFoam_BlockFace(dict, index, is)
{}

void XFoam_PlaneFace::project(
	const XFoam_BlockDescriptor&,
	const XFoam_Label,
	XFoam_Field<XFoam_Vector3D>&) const
{}

const XFoam_searchableSurface& XFoam_BlockProjectFace::lookupSurface(
	const XFoam_SearchableSurfaceList& geometry,
	XFoam_IStream& is) const
{
	XFoam_String name;
	is >> name;
	for (XFoam_Label i = 0; i < geometry.size(); ++i)
	{
		if (geometry[i].name() == name)
		{
			return geometry[i];
		}
	}
	XFoam_FatalErrorInFunction << "Cannot find surface " << name << " in geometry"
							   << XFoam_abort(XFoam_FatalError);
	return geometry[0];
}

XFoam_Label XFoam_BlockProjectFace::index(
	const XFoam_Tuple2<XFoam_Label, XFoam_Label>& n,
	const XFoam_Tuple2<XFoam_Label, XFoam_Label>& coord) const
{
	return coord.first() + coord.second() * n.first();
}

void XFoam_BlockProjectFace::calcLambdas(
	const XFoam_Tuple2<XFoam_Label, XFoam_Label>& n,
	const XFoam_Field<XFoam_Vector3D>& points,
	XFoam_Field<XFoam_Scalar>& lambdaI,
	XFoam_Field<XFoam_Scalar>& lambdaJ) const
{
	lambdaI.setSize(points.size());
	lambdaJ.setSize(points.size());
	for (XFoam_Label i = 0; i < points.size(); ++i)
	{
		lambdaI[i] = 0;
		lambdaJ[i] = 0;
	}

	for (XFoam_Label i = 1; i < n.first(); ++i)
	{
		for (XFoam_Label j = 1; j < n.second(); ++j)
		{
			const XFoam_Label ij = index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j));
			const XFoam_Label iMin1j =
				index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i - 1, j));
			lambdaI[ij] = lambdaI[iMin1j] + (points[ij] - points[iMin1j]).mag();

			const XFoam_Label ijMin1 =
				index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j - 1));
			lambdaJ[ij] = lambdaJ[ijMin1] + (points[ij] - points[ijMin1]).mag();
		}
	}

	for (XFoam_Label i = 1; i < n.first(); ++i)
	{
		const XFoam_Label ijLast =
			index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, n.second() - 1));
		for (XFoam_Label j = 1; j < n.second(); ++j)
		{
			const XFoam_Label ij = index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j));
			const XFoam_Scalar den = lambdaJ[ijLast];
			lambdaJ[ij] /= (den > XFoam_small ? den : 1);
		}
	}
	for (XFoam_Label j = 1; j < n.second(); ++j)
	{
		const XFoam_Label iLastj =
			index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(n.first() - 1, j));
		for (XFoam_Label i = 1; i < n.first(); ++i)
		{
			const XFoam_Label ij = index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j));
			const XFoam_Scalar den = lambdaI[iLastj];
			lambdaI[ij] /= (den > XFoam_small ? den : 1);
		}
	}
}

XFoam_BlockProjectFace::XFoam_BlockProjectFace(
	const XFoam_Dictionary& dict,
	const XFoam_Label index,
	const XFoam_SearchableSurfaceList& geometry,
	XFoam_IStream& is)
	: XFoam_BlockFace(dict, index, is)
	, surface_(lookupSurface(geometry, is))
{}

XFoam_BlockProjectFace::~XFoam_BlockProjectFace() = default;

void XFoam_BlockProjectFace::project(
	const XFoam_BlockDescriptor& desc,
	const XFoam_Label blockFacei,
	XFoam_Field<XFoam_Vector3D>& points) const
{
	XFoam_Tuple2<XFoam_Label, XFoam_Label> n(-1, -1);
	switch (blockFacei)
	{
	case 0:
	case 1:
		n.first() = desc.density()[1] + 1;
		n.second() = desc.density()[2] + 1;
		break;

	case 2:
	case 3:
		n.first() = desc.density()[0] + 1;
		n.second() = desc.density()[2] + 1;
		break;

	case 4:
	case 5:
		n.first() = desc.density()[0] + 1;
		n.second() = desc.density()[1] + 1;
		break;
	default:
		break;
	}

	XFoam_Field<XFoam_Scalar> lambdaI(points.size(), 0);
	XFoam_Field<XFoam_Scalar> lambdaJ(points.size(), 0);
	calcLambdas(n, points, lambdaI, lambdaJ);

	const XFoam_Label maxIter = 10;
	const XFoam_Scalar relTol = 0.1;
	const XFoam_Scalar absTol = 1e-4;
	XFoam_Scalar initialResidual = 0;

	for (XFoam_Label iter = 0; iter < maxIter; ++iter)
	{
		{
			XFoam_List<XFoam_PointIndexHit> hits;
			const XFoam_Scalar d0 =
				(points[0] - points[points.size() - 1]).magSqr();
			XFoam_Field<XFoam_Scalar> nearestDistSqr(points.size(), d0);
			surface_.findNearest(points, nearestDistSqr, hits);

			for (XFoam_Label i = 0; i < hits.size(); ++i)
			{
				if (hits[i].hit())
				{
					points[i] = hits[i].hitPoint();
				}
			}
		}

		XFoam_Field<XFoam_Vector3D> residual(points.size());
		fieldZero(residual);

		XFoam_LabelList indices;
		XFoam_Field<XFoam_Scalar> weights;
		for (XFoam_Label j = 1; j < n.second() - 1; ++j)
		{
			XFoam_Field<XFoam_Scalar> projLambdas(n.first());
			projLambdas[0] = 0;
			for (XFoam_Label i = 1; i < n.first(); ++i)
			{
				const XFoam_Label ij = index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j));
				const XFoam_Label iMin1j =
					index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i - 1, j));
				projLambdas[i] = projLambdas[i - 1] + (points[ij] - points[iMin1j]).mag();
			}
			{
				const XFoam_Scalar den = projLambdas[n.first() - 1];
				for (XFoam_Label i = 0; i < n.first(); ++i)
				{
					projLambdas[i] /= (den > XFoam_small ? den : 1);
				}
			}

			for (XFoam_Label i = 1; i < n.first() - 1; ++i)
			{
				const XFoam_Label ij = index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j));

				linearInterpolationWeightsValue(projLambdas, lambdaI[ij], indices, weights);

				XFoam_Vector3D predicted(0, 0, 0);
				for (XFoam_Label indexi = 0; indexi < indices.size(); ++indexi)
				{
					const XFoam_Label ptIndex =
						index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(indices[indexi], j));
					predicted += weights[indexi] * points[ptIndex];
				}
				residual[ij] = predicted - points[ij];
			}
		}

		XFoam_Scalar scalarResidual = XFoam_sum(XFoam_mag(residual));

		fieldAddAssign(points, residual);

		fieldZero(residual);
		for (XFoam_Label i = 1; i < n.first() - 1; ++i)
		{
			XFoam_Field<XFoam_Scalar> projLambdas(n.second());
			projLambdas[0] = 0;
			for (XFoam_Label j = 1; j < n.second(); ++j)
			{
				const XFoam_Label ij = index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j));
				const XFoam_Label ijMin1 =
					index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j - 1));
				projLambdas[j] = projLambdas[j - 1] + (points[ij] - points[ijMin1]).mag();
			}
			{
				const XFoam_Scalar den = projLambdas[n.second() - 1];
				for (XFoam_Label j = 0; j < n.second(); ++j)
				{
					projLambdas[j] /= (den > XFoam_small ? den : 1);
				}
			}

			for (XFoam_Label j = 1; j < n.second() - 1; ++j)
			{
				const XFoam_Label ij = index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, j));

				linearInterpolationWeightsValue(projLambdas, lambdaJ[ij], indices, weights);

				XFoam_Vector3D predicted(0, 0, 0);
				for (XFoam_Label indexi = 0; indexi < indices.size(); ++indexi)
				{
					const XFoam_Label ptIndex =
						index(n, XFoam_Tuple2<XFoam_Label, XFoam_Label>(i, indices[indexi]));
					predicted += weights[indexi] * points[ptIndex];
				}
				residual[ij] = predicted - points[ij];
			}
		}

		scalarResidual += XFoam_sum(XFoam_mag(residual));

		if (scalarResidual < absTol * static_cast<XFoam_Scalar>(lambdaI.size()))
		{
			break;
		}
		else if (iter == 0)
		{
			initialResidual = scalarResidual;
		}
		else if (initialResidual > XFoam_small && scalarResidual / initialResidual < relTol)
		{
			break;
		}

		fieldAddAssign(points, residual);
	}
}
} // namespace XFoam_blockFaces
