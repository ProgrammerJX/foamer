// myfoam/src/OpenFOAM/meshes/polyMesh/Face.hpp
//
// A face is an ordered list of point labels defining a (generally non-planar)
// polygon. Modeled on Foam::face.
//
// Geometric operations (centre, area vector) use the fan decomposition around
// the polygon's average point, which is what OpenFOAM does for robustness
// with non-convex / warped faces.
//
#pragma once

#include "OpenFOAM/primitives/Types.hpp"
#include "OpenFOAM/primitives/Vector.hpp"

#include <cstddef>
#include <initializer_list>
#include <vector>

namespace myfoam {

class Face {
public:
    Face() = default;
    Face(std::initializer_list<label> init) : verts_(init) {}
    explicit Face(std::vector<label> v) : verts_(std::move(v)) {}

    std::size_t size() const noexcept { return verts_.size(); }
    bool        empty() const noexcept { return verts_.empty(); }

    label  operator[](std::size_t i) const { return verts_[i]; }
    label& operator[](std::size_t i)       { return verts_[i]; }

    const std::vector<label>& vertices() const noexcept { return verts_; }
    std::vector<label>&       vertices()       noexcept { return verts_; }

    // Reverse the orientation in-place (flip the outward normal).
    void flip();

    // Geometric centre. For triangles this is the centroid; for general
    // polygons it is computed by fan decomposition weighted by sub-triangle
    // area, identical to Foam::face::centre.
    point centre(const std::vector<point>& meshPoints) const;

    // Area vector (magnitude = area, direction = outward normal by the
    // right-hand rule applied to the vertex ordering).
    Vector areaVector(const std::vector<point>& meshPoints) const;

    scalar area(const std::vector<point>& meshPoints) const {
        return mag(areaVector(meshPoints));
    }

private:
    std::vector<label> verts_;
};

// Cell = ordered list of face labels (we keep them unordered; the polyMesh
// layer knows the owner/neighbour relationship).
class Cell {
public:
    Cell() = default;
    Cell(std::initializer_list<label> init) : faces_(init) {}
    explicit Cell(std::vector<label> v) : faces_(std::move(v)) {}

    std::size_t size() const noexcept { return faces_.size(); }
    label  operator[](std::size_t i) const { return faces_[i]; }
    label& operator[](std::size_t i)       { return faces_[i]; }

    const std::vector<label>& faces() const noexcept { return faces_; }
    std::vector<label>&       faces()       noexcept { return faces_; }

private:
    std::vector<label> faces_;
};

} // namespace myfoam
