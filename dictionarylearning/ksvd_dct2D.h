//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#ifndef KSVD_DCT2D_H
#define KSVD_DCT2D_H

#include "constants.h"
//#include <Eigen/Dense>
//#include <functional>

namespace std {
template<class T> class function;
}


void ksvd_dct2D(
		bool useOpenMP,
		const std::vector<Eigen::VectorXf>& Y,
		const std::vector<Eigen::VectorXf>& U,
		const std::vector<Eigen::VectorXf>& V,
		Eigen::Index nfreq,
		Eigen::Index latm,
		int maxIters,
		float maxError,
		const std::function<void(
			const Eigen::VectorXf&,
			const Eigen::MatrixXf&,
			Eigen::Index,
			Eigen::VectorXf&,
			Eigen::VectorXf&
			)> sparseFunct,
		Eigen::MatrixXf& D,
		Eigen::MatrixXf& X
		);


void print_error_dct2D(
		const std::vector<Eigen::VectorXf>& Y,
		const std::vector<Eigen::VectorXf>& U,
		const std::vector<Eigen::VectorXf>& V,
		const Eigen::MatrixXf& D,
		const Eigen::MatrixXf& X,
		Eigen::Index nfreq);


#endif // KSVD_DCT2D_H
