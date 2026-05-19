// myfoam/src/OpenFOAM/meshes/polyMesh/Face.cpp
#include "OpenFOAM/meshes/polyMesh/Face.hpp"

#include <algorithm>

namespace myfoam {

void Face::flip() {
    if (verts_.size() < 2) return;
    // Keep verts_[0] fixed and reverse the remainder. This matches the
    // OpenFOAM convention used by face::reverseFace().
    std::reverse(verts_.begin() + 1, verts_.end());
}

// Fan decomposition around the average point. For each edge (p_i, p_{i+1})
// we form a triangle (avg, p_i, p_{i+1}); the polygon centroid is the
// area-weighted sum of these triangle centroids.
point Face::centre(const std::vector<point>& pts) const {
    const std::size_t n = verts_.size();
    if (n == 0) return {};
    if (n == 1) return pts[verts_[0]];
    if (n == 2) return (pts[verts_[0]] + pts[verts_[1]]) * scalar(0.5);

    point avg{};
    for (label v : verts_) avg += pts[v];
    avg /= static_cast<scalar>(n);

    Vector sumA{};   // total area vector
    point  sumAC{};  // area-weighted centroid

    for (std::size_t i = 0; i < n; ++i) {
        const point& p1 = pts[verts_[i]];
        const point& p2 = pts[verts_[(i + 1) % n]];

        const point  triC = (avg + p1 + p2) / scalar(3);
        const Vector triA = cross(p1 - avg, p2 - avg) * scalar(0.5);
        const scalar a    = mag(triA);

        sumA  += triA;
        sumAC += triC * a;
    }

    const scalar A = mag(sumA);
    return (A > VSMALL) ? sumAC / A : avg;
}

Vector Face::areaVector(const std::vector<point>& pts) const {
    const std::size_t n = verts_.size();
    if (n < 3) return {};

    point avg{};
    for (label v : verts_) avg += pts[v];
    avg /= static_cast<scalar>(n);

    Vector sumA{};
    for (std::size_t i = 0; i < n; ++i) {
        const point& p1 = pts[verts_[i]];
        const point& p2 = pts[verts_[(i + 1) % n]];
        sumA += cross(p1 - avg, p2 - avg) * scalar(0.5);
    }
    return sumA;
}

} // namespace myfoam
