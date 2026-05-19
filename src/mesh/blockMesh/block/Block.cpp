// myfoam/src/mesh/blockMesh/block/Block.cpp
#include "mesh/blockMesh/block/Block.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace myfoam {

// ---------------------------------------------------------------------------
// Hex shape function tables.
//
// Trilinear interpolation weights at local (u,v,w) in [0,1]^3, evaluated at
// the 8 corner indices in the OpenFOAM hex ordering shown in the header.
// ---------------------------------------------------------------------------
namespace {

// (du, dv, dw) flags for each corner: 1 == upper face along that axis.
constexpr int cornerSign[8][3] = {
    {0, 0, 0},  // 0
    {1, 0, 0},  // 1
    {1, 1, 0},  // 2
    {0, 1, 0},  // 3
    {0, 0, 1},  // 4
    {1, 0, 1},  // 5
    {1, 1, 1},  // 6
    {0, 1, 1},  // 7
};

point trilinear(const std::array<point, 8>& C, scalar u, scalar v, scalar w) {
    point p{};
    for (int n = 0; n < 8; ++n) {
        const scalar wu = cornerSign[n][0] ? u : (1 - u);
        const scalar wv = cornerSign[n][1] ? v : (1 - v);
        const scalar ww = cornerSign[n][2] ? w : (1 - w);
        p += C[n] * (wu * wv * ww);
    }
    return p;
}

} // anonymous namespace


// ---------------------------------------------------------------------------
// 1D simple grading: returns n+1 node positions in [0,1].
//
// Uniform (r == 1): positions are 0, 1/n, 2/n, ..., 1.
//
// Geometric (r != 1): cell sizes form a geometric sequence
//     dx_0, dx_0*r, dx_0*r^2, ..., dx_0*r^{n-1}
// with sum = 1. Hence
//     dx_0 = (1 - r) / (1 - r^n)        for r != 1.
//
// `r` is the size ratio of the LAST cell to the FIRST cell (matches the
// OpenFOAM `simpleGrading` convention).
// ---------------------------------------------------------------------------
std::vector<scalar> Block::spacing(label n, scalar r) {
    if (n <= 0) throw std::invalid_argument("Block::spacing: n must be > 0");

    std::vector<scalar> s(static_cast<std::size_t>(n + 1));
    s.front() = 0;
    s.back()  = 1;

    if (std::fabs(r - 1) < 1e-12) {
        for (label i = 1; i < n; ++i) {
            s[static_cast<std::size_t>(i)] = static_cast<scalar>(i) / n;
        }
        return s;
    }

    // r != 1: solve for per-cell ratio q such that q^n = r, i.e. q = r^(1/n).
    // Then dx_i = dx_0 * q^i with sum = 1 ==> dx_0 = (1 - q) / (1 - q^n).
    const scalar q     = std::pow(r, scalar(1) / static_cast<scalar>(n));
    const scalar dx0   = (1 - q) / (1 - std::pow(q, n));
    scalar       accum = 0;
    scalar       dx    = dx0;
    for (label i = 1; i < n; ++i) {
        accum += dx;
        s[static_cast<std::size_t>(i)] = accum;
        dx *= q;
    }
    return s;
}


// ---------------------------------------------------------------------------
Block::Block(std::array<point, 8> corners,
             label nx, label ny, label nz,
             Vector grading)
    : corners_(corners), nx_(nx), ny_(ny), nz_(nz), grading_(grading)
{
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        throw std::invalid_argument("Block: divisions must be positive");
    }
    generatePoints();
    generateCells();
    generateInternalFaces();
}


void Block::generatePoints() {
    const auto us = spacing(nx_, grading_.x);
    const auto vs = spacing(ny_, grading_.y);
    const auto ws = spacing(nz_, grading_.z);

    points_.resize(static_cast<std::size_t>(nPoints()));
    for (label k = 0; k <= nz_; ++k) {
        for (label j = 0; j <= ny_; ++j) {
            for (label i = 0; i <= nx_; ++i) {
                points_[static_cast<std::size_t>(pointIndex(i, j, k))]
                    = trilinear(corners_, us[i], vs[j], ws[k]);
            }
        }
    }
}


// Each cell stores its 8 corner POINT indices in hex order. We don't store
// face indices yet because faces aren't globally numbered until Stage 3.
void Block::generateCells() {
    cells_.clear();
    cells_.reserve(static_cast<std::size_t>(nCells()));
    for (label k = 0; k < nz_; ++k) {
        for (label j = 0; j < ny_; ++j) {
            for (label i = 0; i < nx_; ++i) {
                Cell c{
                    pointIndex(i,   j,   k  ),  // 0
                    pointIndex(i+1, j,   k  ),  // 1
                    pointIndex(i+1, j+1, k  ),  // 2
                    pointIndex(i,   j+1, k  ),  // 3
                    pointIndex(i,   j,   k+1),  // 4
                    pointIndex(i+1, j,   k+1),  // 5
                    pointIndex(i+1, j+1, k+1),  // 6
                    pointIndex(i,   j+1, k+1)   // 7
                };
                cells_.push_back(std::move(c));
            }
        }
    }
}


// ---------------------------------------------------------------------------
// Internal faces.
//
// For each interior plane perpendicular to i (likewise j, k), build a quad
// whose vertex order makes the area vector point in the +axis direction.
// That direction goes from the LOWER cell to the UPPER cell, so owner is
// the lower cell and neighbour is the upper one. This is exactly the
// owner<neighbour convention OpenFOAM enforces.
// ---------------------------------------------------------------------------
void Block::generateInternalFaces() {
    iFaces_ = {}; jFaces_ = {}; kFaces_ = {};

    // i-direction: i = 1..nx-1
    for (label k = 0; k < nz_; ++k) {
        for (label j = 0; j < ny_; ++j) {
            for (label i = 1; i < nx_; ++i) {
                iFaces_.faces.push_back(Face{
                    pointIndex(i, j,   k  ),
                    pointIndex(i, j+1, k  ),
                    pointIndex(i, j+1, k+1),
                    pointIndex(i, j,   k+1)
                });
                iFaces_.owner    .push_back(cellIndex(i - 1, j, k));
                iFaces_.neighbour.push_back(cellIndex(i,     j, k));
            }
        }
    }

    // j-direction
    for (label k = 0; k < nz_; ++k) {
        for (label j = 1; j < ny_; ++j) {
            for (label i = 0; i < nx_; ++i) {
                jFaces_.faces.push_back(Face{
                    pointIndex(i,   j, k  ),
                    pointIndex(i,   j, k+1),
                    pointIndex(i+1, j, k+1),
                    pointIndex(i+1, j, k  )
                });
                jFaces_.owner    .push_back(cellIndex(i, j - 1, k));
                jFaces_.neighbour.push_back(cellIndex(i, j,     k));
            }
        }
    }

    // k-direction
    for (label k = 1; k < nz_; ++k) {
        for (label j = 0; j < ny_; ++j) {
            for (label i = 0; i < nx_; ++i) {
                kFaces_.faces.push_back(Face{
                    pointIndex(i,   j,   k),
                    pointIndex(i+1, j,   k),
                    pointIndex(i+1, j+1, k),
                    pointIndex(i,   j+1, k)
                });
                kFaces_.owner    .push_back(cellIndex(i, j, k - 1));
                kFaces_.neighbour.push_back(cellIndex(i, j, k    ));
            }
        }
    }
}


// ---------------------------------------------------------------------------
// Boundary faces per side. Vertex ordering produces an OUTWARD area vector.
// The local face IDs and orderings match those in the header documentation.
// ---------------------------------------------------------------------------
Block::BoundaryFaces Block::sideFaces(Side s) const {
    BoundaryFaces bf;

    auto add = [&](Face f, label owner) {
        bf.faces.push_back(std::move(f));
        bf.owner.push_back(owner);
    };

    switch (s) {
        case Side::XMin: {
            // Outward normal = -x. Vertex order: (i=0,j,k) (i=0,j,k+1)
            // (i=0,j+1,k+1) (i=0,j+1,k) which by RHR gives -x.
            for (label k = 0; k < nz_; ++k)
                for (label j = 0; j < ny_; ++j) {
                    add(Face{
                        pointIndex(0, j,   k  ),
                        pointIndex(0, j,   k+1),
                        pointIndex(0, j+1, k+1),
                        pointIndex(0, j+1, k  )
                    }, cellIndex(0, j, k));
                }
            break;
        }
        case Side::XMax: {
            for (label k = 0; k < nz_; ++k)
                for (label j = 0; j < ny_; ++j) {
                    add(Face{
                        pointIndex(nx_, j,   k  ),
                        pointIndex(nx_, j+1, k  ),
                        pointIndex(nx_, j+1, k+1),
                        pointIndex(nx_, j,   k+1)
                    }, cellIndex(nx_ - 1, j, k));
                }
            break;
        }
        case Side::YMin: {
            for (label k = 0; k < nz_; ++k)
                for (label i = 0; i < nx_; ++i) {
                    add(Face{
                        pointIndex(i,   0, k  ),
                        pointIndex(i+1, 0, k  ),
                        pointIndex(i+1, 0, k+1),
                        pointIndex(i,   0, k+1)
                    }, cellIndex(i, 0, k));
                }
            break;
        }
        case Side::YMax: {
            for (label k = 0; k < nz_; ++k)
                for (label i = 0; i < nx_; ++i) {
                    add(Face{
                        pointIndex(i,   ny_, k  ),
                        pointIndex(i,   ny_, k+1),
                        pointIndex(i+1, ny_, k+1),
                        pointIndex(i+1, ny_, k  )
                    }, cellIndex(i, ny_ - 1, k));
                }
            break;
        }
        case Side::ZMin: {
            for (label j = 0; j < ny_; ++j)
                for (label i = 0; i < nx_; ++i) {
                    add(Face{
                        pointIndex(i,   j,   0),
                        pointIndex(i,   j+1, 0),
                        pointIndex(i+1, j+1, 0),
                        pointIndex(i+1, j,   0)
                    }, cellIndex(i, j, 0));
                }
            break;
        }
        case Side::ZMax: {
            for (label j = 0; j < ny_; ++j)
                for (label i = 0; i < nx_; ++i) {
                    add(Face{
                        pointIndex(i,   j,   nz_),
                        pointIndex(i+1, j,   nz_),
                        pointIndex(i+1, j+1, nz_),
                        pointIndex(i,   j+1, nz_)
                    }, cellIndex(i, j, nz_ - 1));
                }
            break;
        }
    }
    return bf;
}

} // namespace myfoam
