//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <limits>

const float float_infinity = std::numeric_limits<float>::infinity();
const float float_tiny = std::numeric_limits<float>::min();
const double double_infinity = std::numeric_limits<double>::infinity();
const int int_infinity = std::numeric_limits<int>::max();

enum class SparseApprox { MatchingPursuit = 0, OrthogonalPursuit = 1 };

#endif // CONSTANTS_H
