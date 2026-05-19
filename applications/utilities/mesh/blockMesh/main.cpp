// myfoam/applications/utilities/mesh/blockMesh/main.cpp
//
// Placeholder. Stage 3 will replace this with real CLI entry point that
// reads a blockMeshDict and writes a polyMesh.
//
#include "OpenFOAM/primitives/Vector.hpp"
#include "OpenFOAM/meshes/polyMesh/Face.hpp"

#include <iostream>

int main() {
    using namespace myfoam;

    // Quick sanity: unit square as a Face.
    std::vector<point> pts = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}
    };
    Face f{0, 1, 2, 3};

    std::cout << "myfoam blockMesh skeleton\n"
              << "  face centre = " << f.centre(pts) << '\n'
              << "  face area   = " << f.area(pts)   << '\n';
    return 0;
}
