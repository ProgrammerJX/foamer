// myfoam/src/OpenFOAM/primitives/Types.hpp
//
// Fundamental scalar/label typedefs, modeled on OpenFOAM.
// `label` is the integer type used to index mesh entities (points, faces, cells).
// `scalar` is the default floating-point type.
//
#pragma once

#include <cstdint>
#include <limits>

namespace myfoam {

using label  = std::int32_t;
using scalar = double;

inline constexpr label labelMax = std::numeric_limits<label>::max();
inline constexpr label labelMin = std::numeric_limits<label>::min();

// Sentinel used in OpenFOAM to mean "no such entity" (e.g. boundary face
// has no neighbour cell).
inline constexpr label labelNone = -1;

inline constexpr scalar SMALL    = 1.0e-15;
inline constexpr scalar VSMALL   = 1.0e-300;
inline constexpr scalar GREAT    = 1.0e+15;

} // namespace myfoam
