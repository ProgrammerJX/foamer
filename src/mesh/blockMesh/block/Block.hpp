// myfoam/src/mesh/blockMesh/block/Block.hpp
//
// A `Block` is a structured hex region defined by 8 corner vertices and the
// number of cell divisions in each of the three local (i,j,k) directions.
//
// Local hex vertex numbering (matches OpenFOAM `blockMesh`):
//
//          7---------6
//         /|        /|        local axes
//        / |       / |          k(+z)  j(+y)
//       4---------5  |             | /
//       |  |      |  |             |/
//       |  3------|--2             +------ i(+x)
//       | /       | /
//       |/        |/
//       0---------1
//
// 0,1,2,3 are the k=0 face; 4,5,6,7 are the k=nz face.
//
// Face local IDs and vertex orderings (outward normal by right-hand rule):
//   0  x-min : {0,4,7,3}
//   1  x-max : {1,2,6,5}
//   2  y-min : {0,1,5,4}
//   3  y-max : {3,7,6,2}
//   4  z-min : {0,3,2,1}
//   5  z-max : {4,5,6,7}
//
// Stage 2 scope: generate the block's *local* points, cells, and the four
// internal-face groups (i-, j-, k-directional internal faces) needed by the
// BlockMesh assembler in Stage 3. Boundary faces on each of the six sides
// are also exposed for patch creation.
//
// Grading: a simple per-direction expansion ratio is supported, equivalent
// to OpenFOAM's `simpleGrading (rx ry rz)`. Edge-grading is deferred to
// Stage 6.
//
#pragma once

#include "OpenFOAM/meshes/polyMesh/Face.hpp"
#include "OpenFOAM/primitives/Vector.hpp"

#include <array>
#include <vector>

namespace myfoam {

class Block {
public:
    // 6 block sides (i.e. local face IDs above).
    enum class Side : int {
        XMin = 0, XMax = 1,
        YMin = 2, YMax = 3,
        ZMin = 4, ZMax = 5
    };
    static constexpr int nSides = 6;

    Block(std::array<point, 8> corners,
          label nx, label ny, label nz,
          Vector grading = {1, 1, 1});

    // -- divisions ----------------------------------------------------------
    label nx() const noexcept { return nx_; }
    label ny() const noexcept { return ny_; }
    label nz() const noexcept { return nz_; }

    label nPoints() const noexcept { return (nx_ + 1) * (ny_ + 1) * (nz_ + 1); }
    label nCells()  const noexcept { return nx_ * ny_ * nz_; }

    // -- generated local data ----------------------------------------------
    const std::vector<point>& points() const noexcept { return points_; }
    const std::vector<Cell>&  cells()  const noexcept { return cells_;  }

    // Local 1D index for the grid point (i,j,k), i in [0,nx], etc.
    label pointIndex(label i, label j, label k) const noexcept {
        return (k * (ny_ + 1) + j) * (nx_ + 1) + i;
    }

    // Local 1D index for the cell (i,j,k), i in [0,nx).
    label cellIndex(label i, label j, label k) const noexcept {
        return (k * ny_ + j) * nx_ + i;
    }

    // -- face access used by BlockMesh assembler ---------------------------
    //
    // For each of the three internal directions we return the faces and
    // their owner/neighbour cells (LOCAL cell indices within this block).
    // Face vertex ordering is chosen so the area vector points from owner
    // to neighbour, exactly as OpenFOAM requires.
    //
    struct InternalFaces {
        std::vector<Face>  faces;
        std::vector<label> owner;
        std::vector<label> neighbour;
    };

    const InternalFaces& internalIFaces() const noexcept { return iFaces_; }
    const InternalFaces& internalJFaces() const noexcept { return jFaces_; }
    const InternalFaces& internalKFaces() const noexcept { return kFaces_; }

    // Quad faces on a given block side. Face vertex ordering has the area
    // vector pointing *outward* from the block. The companion vector tells
    // the assembler which local cell each face belongs to.
    struct BoundaryFaces {
        std::vector<Face>  faces;
        std::vector<label> owner;  // local cell index
    };

    BoundaryFaces sideFaces(Side s) const;

private:
    void generatePoints();
    void generateCells();
    void generateInternalFaces();

    // 1D node spacing in [0,1] with simple expansion ratio `r`.
    // r == 1 -> uniform.
    // r != 1 -> geometric: dx_i = dx_0 * r^i so that sum = 1.
    static std::vector<scalar> spacing(label n, scalar r);

    std::array<point, 8> corners_;
    label nx_, ny_, nz_;
    Vector grading_;

    std::vector<point> points_;
    std::vector<Cell>  cells_;

    InternalFaces iFaces_, jFaces_, kFaces_;
};

} // namespace myfoam
