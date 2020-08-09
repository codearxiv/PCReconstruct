//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.


#ifndef COSINETRANSFORM_H
#define COSINETRANSFORM_H

#include "alignment.h"

#include <Eigen/Dense>
#include <vector>


void cosine_transform(
		const Eigen::VectorXf& U,
		const Eigen::VectorXf& V,
		Eigen::Index nfreq,
		std::vector<float, Eigen::aligned_allocator<float>>& dwork,
		Eigen::Ref<Eigen::MatrixXf, ALIGNEDX> T
		);

#endif // COSINETRANSFORM_H
