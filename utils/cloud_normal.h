//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.


#ifndef CLOUD_NORMAL_H
#define CLOUD_NORMAL_H

#include <Eigen/Dense>
#include <vector>


Eigen::Vector3f cloud_normal(
		const Eigen::Vector3f& p0, const std::vector<Eigen::Vector3f>& cloud,
		int niters, std::vector<Eigen::Vector3f>& vwork);


#endif // CLOUD_NORMAL_H
