//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included with this distribution.


#ifndef KSVD_DCT2D_H
#define KSVD_DCT2D_H

#include "alignment.h"

#include <Eigen/Dense>
#include <vector>
#include <functional>

class MessageLogger;

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
		Eigen::MatrixXf& X,
		MessageLogger* msgLogger = nullptr
		);


void print_error_dct2D(
		bool useOpenMP,
		const std::vector<Eigen::VectorXf>& Y,
		const std::vector<Eigen::VectorXf>& U,
		const std::vector<Eigen::VectorXf>& V,
		const Eigen::MatrixXf& D,
		const Eigen::MatrixXf& X,
		Eigen::Index nfreq,
		Eigen::Index iter,
		MessageLogger* msgLogger
		);


void column_normalize(Eigen::Ref<Eigen::MatrixXf, ALIGNEDX> M,
					  Eigen::Ref<Eigen::VectorXf, ALIGNEDX> NrmInv);

#endif // KSVD_DCT2D_H
