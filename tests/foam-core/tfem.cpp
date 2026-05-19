#include "doctest/doctest.h"
#include "tcommon.h"
#include "XFoam/utilities/xfoam_common.h"
#include "XFoam/utilities/xfoam_fem.h"
#include <boost/filesystem.hpp>

TEST_CASE("XFoam_FEM.bdf write and read")
{
	const boost::filesystem::path srcBdf = XFoam_testsDataDir(__FILE__) / "bdf" / "cylinder.bdf";
	REQUIRE(boost::filesystem::exists(srcBdf));

	const boost::filesystem::path tmpDir = XFoam_testsTmpDir(__FILE__) / "bdf";
	boost::filesystem::create_directories(tmpDir);
	const boost::filesystem::path dstBdf = tmpDir / "cylinder_roundtrip.bdf";

	XFoam_FEM fem;
	fem.readBdf(XFoam_FileName(srcBdf.generic_string()));
	REQUIRE(fem.nNodes() > 0);
	REQUIRE(fem.nElements() > 0);
	const XFoam_Vector3D p0 = fem.getNodePosition(0);

	fem.writeBdf(XFoam_FileName(dstBdf.generic_string()));
	REQUIRE(boost::filesystem::exists(dstBdf));

	XFoam_FEM fem2;
	fem2.readBdf(XFoam_FileName(dstBdf.generic_string()));
	CHECK(fem2.nNodes() == fem.nNodes());
	CHECK(fem2.nElements() == fem.nElements());
	const XFoam_Vector3D q0 = fem2.getNodePosition(0);
	CHECK(p0.x() == doctest::Approx(q0.x()));
	CHECK(p0.y() == doctest::Approx(q0.y()));
	CHECK(p0.z() == doctest::Approx(q0.z()));
}

TEST_CASE("XFoam_FEM.key write and read")
{
	const boost::filesystem::path srcKey = XFoam_testsDataDir(__FILE__) / "key" / "cylinder.key";
	REQUIRE(boost::filesystem::exists(srcKey));

	const boost::filesystem::path tmpDir = XFoam_testsTmpDir(__FILE__) / "key";
	boost::filesystem::create_directories(tmpDir);
	const boost::filesystem::path dstKey = tmpDir / "cylinder_roundtrip.key";

	XFoam_FEM fem;
	fem.readKey(XFoam_FileName(srcKey.generic_string()));
	REQUIRE(fem.nNodes() > 0);
	REQUIRE(fem.nElements() > 0);
	const XFoam_Vector3D p0 = fem.getNodePosition(0);

	fem.writeKey(XFoam_FileName(dstKey.generic_string()));
	REQUIRE(boost::filesystem::exists(dstKey));

	XFoam_FEM fem2;
	fem2.readKey(XFoam_FileName(dstKey.generic_string()));
	CHECK(fem2.nNodes() == fem.nNodes());
	CHECK(fem2.nElements() == fem.nElements());
	const XFoam_Vector3D q0 = fem2.getNodePosition(0);
	CHECK(p0.x() == doctest::Approx(q0.x()));
	CHECK(p0.y() == doctest::Approx(q0.y()));
	CHECK(p0.z() == doctest::Approx(q0.z()));
}
