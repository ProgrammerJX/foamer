#include "XFoam/XFoam_API.h"
#include "XFoam/topo/xfoam_mbrep.h"
#include "XFoam/topo/xfoam_topo.h"
#include <iostream>

int main(int argc, char* argv[])
{
	if (argc < 2) {
		std::cerr << "usage: x-sample-foam-core <bdf-file>\n";
		return 0;
	}

	XFoam_TopoModel model;
	model.readFromBdf(argv[1]);
	(void)model.bounds();
	std::cout << "loaded BDF\n";
	return 0;
}
