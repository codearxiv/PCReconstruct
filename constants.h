//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CONSTANTS_H
#define CONSTANTS_H

//#include <Eigen/Dense>
//#include <limits>

#define __ALIGNED_MEMORY

#ifdef __ALIGNED_MEMORY
const size_t ALIGNEDX = Eigen::Aligned16;
inline size_t align_padded(size_t n) {
	return ALIGNEDX > 0 ? ALIGNEDX*(1+((n-1)/ALIGNEDX)) : n;
}
#else
const size_t ALIGNEDX = Eigen::Unaligned;
inline size_t align_padded(size_t n) { return n; }
#endif

const float float_infinity = std::numeric_limits<float>::infinity();
const double double_infinity = std::numeric_limits<double>::infinity();

#endif // CONSTANTS_H
