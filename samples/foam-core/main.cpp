#include "XFoam/XFoam_API.h"
#include "XFoam/topo/xfoam_topo.h"
#include <iostream>

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		std::cerr << "usage: x-sample-foam-core <bdf-file>\n";
		return 0;
	}

	XFoam_TopoModel model;
	model.readFromBdf(argv[1]);
	model.rebuildIdentityFromBrep(30.0);

	const XFoam_BoundBox bb = model.bounds();
	std::cout << "loaded BDF: " << argv[1] << '\n'
	          << "  brepKind   : "
	          << (model.brepKind() == XFoam_BrepKind::Triangulated ? "Triangulated" : "Parametric")
	          << '\n'
	          << "  nPoints    : " << model.vbrep().nVerts() << '\n'
	          << "  nFaces(tri): " << model.vbrep().nFaces() << '\n'
	          << "  bbox       : (" << bb.min().x() << ',' << bb.min().y() << ',' << bb.min().z()
	          << ") -- (" << bb.max().x() << ',' << bb.max().y() << ',' << bb.max().z() << ")\n"
	          << "  TopoFaces  : " << model.nFaces() << '\n'
	          << "  TopoEdges  : " << model.nEdges() << '\n'
	          << "  TopoVerts  : " << model.nVerts() << '\n'
	          << "  TopoBodies : " << model.nBodies() << '\n';
	return 0;
}
