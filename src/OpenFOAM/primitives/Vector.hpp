// myfoam/src/OpenFOAM/primitives/Vector.hpp
//
// 3D Cartesian vector. Equivalent to Foam::vector (Foam::Vector<scalar>).
//
#pragma once

#include "OpenFOAM/primitives/Types.hpp"

#include <cmath>
#include <iosfwd>
#include <ostream>

namespace myfoam {

class Vector {
public:
    scalar x{0}, y{0}, z{0};

    constexpr Vector() = default;
    constexpr Vector(scalar xx, scalar yy, scalar zz) : x(xx), y(yy), z(zz) {}

    // Component access by index (0,1,2). No bounds check in release builds.
    constexpr scalar  operator[](label i) const { return (&x)[i]; }
    constexpr scalar& operator[](label i)       { return (&x)[i]; }

    // ---- arithmetic ----
    constexpr Vector operator+(const Vector& r) const { return {x + r.x, y + r.y, z + r.z}; }
    constexpr Vector operator-(const Vector& r) const { return {x - r.x, y - r.y, z - r.z}; }
    constexpr Vector operator-()               const { return {-x, -y, -z}; }
    constexpr Vector operator*(scalar s)        const { return {x * s, y * s, z * s}; }
    constexpr Vector operator/(scalar s)        const { return {x / s, y / s, z / s}; }

    Vector& operator+=(const Vector& r) { x += r.x; y += r.y; z += r.z; return *this; }
    Vector& operator-=(const Vector& r) { x -= r.x; y -= r.y; z -= r.z; return *this; }
    Vector& operator*=(scalar s)        { x *= s;   y *= s;   z *= s;   return *this; }
    Vector& operator/=(scalar s)        { x /= s;   y /= s;   z /= s;   return *this; }

    constexpr bool operator==(const Vector& r) const { return x == r.x && y == r.y && z == r.z; }
    constexpr bool operator!=(const Vector& r) const { return !(*this == r); }
};

inline constexpr Vector operator*(scalar s, const Vector& v) { return v * s; }

// Dot product (Foam::operator&)
inline constexpr scalar dot(const Vector& a, const Vector& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Cross product (Foam::operator^)
inline constexpr Vector cross(const Vector& a, const Vector& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline scalar magSqr(const Vector& v) { return dot(v, v); }
inline scalar mag   (const Vector& v) { return std::sqrt(magSqr(v)); }

inline Vector normalised(const Vector& v) {
    const scalar m = mag(v);
    return (m > VSMALL) ? v / m : Vector{0, 0, 0};
}

inline std::ostream& operator<<(std::ostream& os, const Vector& v) {
    return os << '(' << v.x << ' ' << v.y << ' ' << v.z << ')';
}

// Conventional alias mirroring OpenFOAM's `point` typedef.
using point = Vector;

} // namespace myfoam
