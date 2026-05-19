// myfoam/tests/test_block.cpp
//
// Stage 2 tests: Block point/cell/face generation and orientation invariants.
//
#include "mesh/blockMesh/block/Block.hpp"
#include "OpenFOAM/primitives/Vector.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace myfoam;

static bool nearly(scalar a, scalar b, scalar tol = 1e-12) {
    return std::fabs(a - b) <= tol;
}
static bool nearly(const Vector& a, const Vector& b, scalar tol = 1e-12) {
    return nearly(a.x, b.x, tol) && nearly(a.y, b.y, tol) && nearly(a.z, b.z, tol);
}

static std::array<point, 8> unitCubeCorners() {
    return {{
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}
    }};
}

static void test_1x1x1() {
    Block b(unitCubeCorners(), 1, 1, 1);
    assert(b.nPoints() == 8);
    assert(b.nCells()  == 1);

    assert(b.internalIFaces().faces.empty());
    assert(b.internalJFaces().faces.empty());
    assert(b.internalKFaces().faces.empty());

    // Each side should have exactly one quad face.
    for (int s = 0; s < Block::nSides; ++s) {
        auto bf = b.sideFaces(static_cast<Block::Side>(s));
        assert(bf.faces.size() == 1);
        assert(bf.owner.front() == 0);
    }
}

static void test_2x2x2_counts() {
    Block b(unitCubeCorners(), 2, 2, 2);
    assert(b.nPoints() == 27);
    assert(b.nCells()  == 8);

    // Internal faces per direction: (n-1) * n * n = 1*2*2 = 4 each.
    assert(b.internalIFaces().faces.size() == 4);
    assert(b.internalJFaces().faces.size() == 4);
    assert(b.internalKFaces().faces.size() == 4);

    // Each side: 2*2 = 4 quads.
    for (int s = 0; s < Block::nSides; ++s) {
        auto bf = b.sideFaces(static_cast<Block::Side>(s));
        assert(bf.faces.size() == 4);
    }
}

static void test_internal_face_orientation() {
    Block b(unitCubeCorners(), 2, 2, 2);
    const auto& pts = b.points();

    for (const auto& f : b.internalIFaces().faces) {
        assert(nearly(normalised(f.areaVector(pts)), {1, 0, 0}));
    }
    for (const auto& f : b.internalJFaces().faces) {
        assert(nearly(normalised(f.areaVector(pts)), {0, 1, 0}));
    }
    for (const auto& f : b.internalKFaces().faces) {
        assert(nearly(normalised(f.areaVector(pts)), {0, 0, 1}));
    }
}

static void test_internal_owner_lt_neighbour() {
    Block b(unitCubeCorners(), 3, 2, 2);
    const auto check = [](const Block::InternalFaces& f) {
        for (std::size_t i = 0; i < f.owner.size(); ++i) {
            assert(f.owner[i] < f.neighbour[i]);  // OpenFOAM invariant
        }
    };
    check(b.internalIFaces());
    check(b.internalJFaces());
    check(b.internalKFaces());
}

static void test_boundary_face_outward_normals() {
    Block b(unitCubeCorners(), 2, 2, 2);
    const auto& pts = b.points();

    const Vector expected[6] = {
        {-1, 0, 0}, { 1, 0, 0},
        { 0,-1, 0}, { 0, 1, 0},
        { 0, 0,-1}, { 0, 0, 1}
    };
    for (int s = 0; s < Block::nSides; ++s) {
        auto bf = b.sideFaces(static_cast<Block::Side>(s));
        for (const auto& f : bf.faces) {
            assert(nearly(normalised(f.areaVector(pts)), expected[s]));
        }
    }
}

static void test_grading_monotonic() {
    // x-direction grading = 4 means last cell is 4x the first cell.
    Block b(unitCubeCorners(), 5, 1, 1, Vector{4, 1, 1});
    const auto& pts = b.points();

    // Walk x-coordinates along j=k=0.
    scalar prev = -1;
    for (label i = 0; i <= 5; ++i) {
        const point& p = pts[static_cast<std::size_t>(b.pointIndex(i, 0, 0))];
        assert(p.x > prev);
        prev = p.x;
    }
    // First cell smaller than last cell.
    const point& p0 = pts[static_cast<std::size_t>(b.pointIndex(0, 0, 0))];
    const point& p1 = pts[static_cast<std::size_t>(b.pointIndex(1, 0, 0))];
    const point& pN_1 = pts[static_cast<std::size_t>(b.pointIndex(4, 0, 0))];
    const point& pN   = pts[static_cast<std::size_t>(b.pointIndex(5, 0, 0))];
    assert((p1.x - p0.x) < (pN.x - pN_1.x));
}

static void test_uniform_spacing() {
    Block b(unitCubeCorners(), 4, 1, 1);  // uniform
    const auto& pts = b.points();
    for (label i = 0; i <= 4; ++i) {
        const point& p = pts[static_cast<std::size_t>(b.pointIndex(i, 0, 0))];
        assert(nearly(p.x, static_cast<scalar>(i) / 4));
    }
}

int main() {
    test_1x1x1();
    test_2x2x2_counts();
    test_internal_face_orientation();
    test_internal_owner_lt_neighbour();
    test_boundary_face_outward_normals();
    test_grading_monotonic();
    test_uniform_spacing();
    std::cout << "Stage 2 Block: OK\n";
    return 0;
}
