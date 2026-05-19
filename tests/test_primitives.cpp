// myfoam/tests/test_primitives.cpp
//
// Minimal sanity tests for Stage 1 primitives. No external test framework
// to keep the dependency surface zero -- a failed assertion exits non-zero.
//
#include "OpenFOAM/primitives/Vector.hpp"
#include "OpenFOAM/meshes/polyMesh/Face.hpp"

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

static void test_vector_basics() {
    Vector a{1, 2, 3}, b{4, 5, 6};
    assert(nearly(a + b, {5, 7, 9}));
    assert(nearly(b - a, {3, 3, 3}));
    assert(nearly(dot(a, b), 32.0));
    assert(nearly(cross({1, 0, 0}, {0, 1, 0}), {0, 0, 1}));
    assert(nearly(mag(Vector{3, 4, 0}), 5.0));
}

static void test_face_unit_square() {
    std::vector<point> pts = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}
    };
    Face f{0, 1, 2, 3};

    assert(nearly(f.centre(pts), {0.5, 0.5, 0.0}));
    assert(nearly(f.area(pts),  1.0));
    // CCW in xy plane -> +z normal
    assert(nearly(f.areaVector(pts), {0, 0, 1}));
}

static void test_face_triangle() {
    std::vector<point> pts = {{0, 0, 0}, {2, 0, 0}, {0, 2, 0}};
    Face t{0, 1, 2};
    assert(nearly(t.area(pts), 2.0));
}

static void test_face_flip() {
    Face f{10, 20, 30, 40};
    f.flip();
    assert(f[0] == 10 && f[1] == 40 && f[2] == 30 && f[3] == 20);

    std::vector<point> pts = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 0}
    };
    // Map verts so we can reuse: build CCW face then flipped CW face.
    Face f1{0, 1, 2, 3};
    Face f2 = f1; f2.flip();
    assert(nearly(f1.areaVector(pts), -f2.areaVector(pts)));
}

int main() {
    test_vector_basics();
    test_face_unit_square();
    test_face_triangle();
    test_face_flip();
    std::cout << "Stage 1 primitives: OK\n";
    return 0;
}
