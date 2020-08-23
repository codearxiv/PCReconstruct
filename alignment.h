//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <Eigen/Dense>


#define ALIGNED_MEMORY


#ifdef ALIGNED_MEMORY
const size_t ALIGNEDX = Eigen::Aligned16;
inline size_t align_padded(size_t n) {
	return ALIGNEDX > 0 ? ALIGNEDX*(1+((n-1)/ALIGNEDX)) : n;
}
#else
const size_t ALIGNEDX = Eigen::Unaligned;
inline size_t align_padded(size_t n) { return n; }
#endif

#endif // ALIGNMENT_H
