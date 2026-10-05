#ifndef TYPES_HPP
#define TYPES_HPP

#include <cstdint>
#include <limits>

using NodeID = std::uint32_t;
using EdgeWeight = double;

constexpr EdgeWeight INF_WEIGHT = std::numeric_limits<EdgeWeight>::infinity();
constexpr NodeID INVALID_NODE = std::numeric_limits<NodeID>::max();

#endif // TYPES_HPP
