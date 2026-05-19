#include "doctest/doctest.h"
#include "XFoam/topo/xfoam_mbrep.h"
#include "XFoam/topo/xfoam_topo.h"
#include "XFoam/utilities/xfoam_common.h"
#include <boost/filesystem.hpp>

TEST_CASE("foam-mesh smoke")
{
	XFoam_TopoModel model;
	CHECK(model.bounds().volume() == doctest::Approx(0.0));
	CHECK_NOTHROW(model.virtualBrep().clear());
	XFoam_TopoVert v(&model);
	CHECK(v.model() == &model);
	CHECK_NOTHROW(model.mbrep().clear());
}

TEST_CASE("XFoam_TopoModel readFromBdf cylinder.bdf")
{
	const boost::filesystem::path bdf =
		(boost::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
			"data" / "bdf" / "cylinder.bdf")
			.lexically_normal();
	REQUIRE(boost::filesystem::exists(bdf));

	XFoam_TopoModel model;
	CHECK_NOTHROW(model.readFromBdf(bdf.string()));

	const XFoam_MBrep& br = model.mbrep();
	const XFoam_BoundBox bb = model.bounds();
	CHECK(bb.min().x() < bb.max().x());
	CHECK(bb.volume() > 0.0);
	CHECK(bb.contains(br.positions()));
	CHECK(br.bounds() == bb);
	CHECK(br.positions().size() > 100);
	CHECK(br.elems().size() > 100);
	// faces_：仅 $HMMOVE 且全部为壳单元的 component（cylinder：inlet / outlet / wall）
	CHECK(br.faces().size() == 3);
	CHECK(br.faces().size() != br.elems().size());
	CHECK((br.faces()[0].size() == 78));
	CHECK((br.faces()[1].size() == 78));
	CHECK((br.faces()[2].size() == 600));
	for (XFoam_Label fi = 0; fi < br.faces().size(); ++fi)
	{
		for (XFoam_Label j = 0; j < br.faces()[fi].size(); ++j)
		{
			const XFoam_Label ei = br.faces()[fi][j];
			CHECK((ei >= 0 && ei < br.elems().size()));
		}
	}
}

TEST_CASE("XFoam_TopoModel readFromBdf missing file throws")
{
	XFoam_TopoModel model;
	CHECK_THROWS_AS(model.readFromBdf("___nonexistent_bdf_file___.bdf"), const XFoam_Error&);
}
