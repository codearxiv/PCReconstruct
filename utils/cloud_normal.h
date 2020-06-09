//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CLOUD_NORMAL_H
#define CLOUD_NORMAL_H

//#include <Eigen/Dense>

Eigen::Vector3f cloud_normal(
		Eigen::Vector3f p0, const Eigen::Matrix3Xf& cloud, int niters, double zeroTol = 0.0);

#endif // CLOUD_NORMAL_H
