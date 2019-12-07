//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef KSVD_H
#define KSVD_H

//#include <Eigen/Dense>
//#include <functional>

namespace std {
template<class T> class function;
}


void ksvd(
        bool useOpenMP,
		const Eigen::MatrixXf& Y,
		Eigen::Index latm,
        int maxIters,
		float maxError,
        int svPowIters,
        const std::function<void(
			const Eigen::VectorXf&,
			const Eigen::MatrixXf&,
			Eigen::Index,
			Eigen::VectorXf&,
			Eigen::VectorXf&)> sparseFunct,
		Eigen::MatrixXf& D,
		Eigen::MatrixXf& X
        );

#endif // KSVD_H
