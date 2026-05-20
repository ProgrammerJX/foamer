#include "doctest/doctest.h"
#include "XFoam/snap/xfoam_primitivemesh.h"

namespace
{
class XFoam_TestPrimitiveMesh final : public XFoam_PrimitiveMesh
{
	XFoam_List<XFoam_Vector3D> pts_;
	XFoam_List<XFoam_Face> fcs_;
	XFoam_LabelList own_;
	XFoam_LabelList nei_;

public:
	XFoam_TestPrimitiveMesh(
		XFoam_List<XFoam_Vector3D> pts,
		XFoam_List<XFoam_Face> fcs,
		XFoam_LabelList own,
		XFoam_LabelList nei,
		const XFoam_Label nInternalFaces,
		const XFoam_Label nCells)
		: XFoam_PrimitiveMesh(pts.size(), nInternalFaces, fcs.size(), nCells)
		, pts_(XFoam_move(pts))
		, fcs_(XFoam_move(fcs))
		, own_(XFoam_move(own))
		, nei_(XFoam_move(nei))
	{
		reset(pts_.size(), nInternalFaces, fcs_.size(), nCells);
	}

	const XFoam_UList<XFoam_Vector3D>& points() const override { return pts_; }
	const XFoam_UList<XFoam_Face>& faces() const override { return fcs_; }
	const XFoam_UList<XFoam_Label>& faceOwner() const override { return own_; }
	const XFoam_UList<XFoam_Label>& faceNeighbour() const override { return nei_; }
};
} // namespace

TEST_CASE("XFoam_PrimitiveMesh::calcCells two cells one internal face")
{
	XFoam_LabelList own({0, 0, 0, 1, 1});
	XFoam_LabelList nei({1, -1, -1, -1, -1});
	XFoam_List<XFoam_Cell> cells;
	XFoam_PrimitiveMesh::calcCells(cells, own, nei, 2);
	REQUIRE(cells.size() == 2);
	CHECK(cells[0].size() == 3);
	CHECK(cells[1].size() == 3);
	CHECK(cells[0] == XFoam_Cell({0, 1, 2}));
	CHECK(cells[1] == XFoam_Cell({0, 3, 4}));
}

TEST_CASE("XFoam_PrimitiveMesh::calcPointOrder single closed face list")
{
	XFoam_List<XFoam_Face> fcs;
	fcs.append(XFoam_Face({0, 1, 2}));

	XFoam_Label nInt = 0;
	XFoam_LabelList map;
	const bool ordered = XFoam_PrimitiveMesh::calcPointOrder(nInt, map, fcs, 1, 3);
	CHECK(ordered);
	CHECK(nInt == 3);
	REQUIRE(map.size() == 3);
	CHECK(map[0] == 0);
	CHECK(map[1] == 1);
	CHECK(map[2] == 2);
}

TEST_CASE("XFoam_PrimitivePatch two triangles shared edge")
{
	XFoam_List<XFoam_Vector3D> pts;
	pts.append(XFoam_Vector3D(0, 0, 0));
	pts.append(XFoam_Vector3D(1, 0, 0));
	pts.append(XFoam_Vector3D(0, 1, 0));
	pts.append(XFoam_Vector3D(0, 0, 1));

	XFoam_List<XFoam_Face> fs;
	fs.append(XFoam_Face({0, 1, 2}));
	fs.append(XFoam_Face({0, 1, 3}));

	const XFoam_UList<XFoam_Vector3D>& ptRef = pts;
	XFoam_FacePrimitivePatch patch(XFoam_move(fs), ptRef);
	CHECK(patch.nFaces() == 2);
	CHECK(static_cast<XFoam_Label>(patch.points().size()) == 4);
	CHECK(patch.nPoints() == 4);
	CHECK(patch.nEdges() == 5);

	const XFoam_LabelList& mp = patch.meshPoints();
	CHECK(mp.size() == 4);
	REQUIRE(patch.meshPointMap().found(2));
	CHECK(patch.meshPointMap().find(2)() == 2);
	CHECK(patch.whichPoint(2) == 2);
	CHECK(patch.whichPoint(100) == -1);

	CHECK(patch.nFaces() == 2);
	CHECK(patch.localFaces().size() == 2);

	const XFoam_List<XFoam_Vector3D>& fc = patch.faceCentres();
	REQUIRE(fc.size() == 2);
	CHECK(fc[0].z() == doctest::Approx(0.0));
	CHECK(fc[1].z() > 0.0);

	const XFoam_List<XFoam_Vector3D>& fa = patch.faceAreas();
	CHECK(fa[0].z() == doctest::Approx(0.5));
	CHECK(fa[1].mag() == doctest::Approx(0.5));

	const XFoam_List<XFoam_Vector3D>& fn = patch.faceNormals();
	REQUIRE(fn.size() == 2);
	CHECK(fn[0].mag() == doctest::Approx(1.0));
	CHECK(fn[1].mag() == doctest::Approx(1.0));

	patch.clearGeom();
	CHECK(patch.faceCentres()[0].z() == doctest::Approx(0.0));
	patch.clearTopology();
	CHECK(patch.meshPoints().size() == 4);
	CHECK(patch.edges().size() == 5);
}

TEST_CASE("XFoam_PrimitiveMesh reset and movePoints")
{
	XFoam_List<XFoam_Vector3D> pts;
	pts.append(XFoam_Vector3D(0, 0, 0));
	pts.append(XFoam_Vector3D(1, 0, 0));
	pts.append(XFoam_Vector3D(0, 1, 0));

	XFoam_List<XFoam_Face> fcs;
	fcs.append(XFoam_Face({0, 1, 2}));

	XFoam_LabelList own(1, 0);
	XFoam_LabelList nei(1, -1);

	XFoam_TestPrimitiveMesh mesh(XFoam_move(pts), XFoam_move(fcs), XFoam_move(own), XFoam_move(nei), 1, 1);
	CHECK(mesh.nInternalFaces() == 1);
	CHECK(mesh.nInternalPoints() >= 0);

	XFoam_List<XFoam_Vector3D> oldPts(mesh.points().cbegin(), mesh.points().cend());
	XFoam_List<XFoam_Vector3D> newPts(oldPts);
	newPts[2] = XFoam_Vector3D(0, 1, 0.5);

	const XFoam_ScalarList swept = mesh.movePoints(newPts, oldPts);
	REQUIRE(swept.size() == 1);
	CHECK(swept[0] != doctest::Approx(0.0));
}
