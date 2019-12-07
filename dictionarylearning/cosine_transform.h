//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef COSINETRANSFORM_H
#define COSINETRANSFORM_H

#include "constants.h"
//#include <Eigen/Dense>

extern const size_t ALIGNEDX;

void cosine_transform(
		const Eigen::VectorXf& U,
		const Eigen::VectorXf& V,
//		const Eigen::VectorXf& U,
//		const Eigen::VectorXf& V,
		Eigen::Index nfreq,
		std::vector<float, Eigen::aligned_allocator<float>>& dwork,
		Eigen::Ref<Eigen::MatrixXf, ALIGNEDX> T
		);

#endif // COSINETRANSFORM_H
