//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.

#ifndef ROTATIONS_H
#define ROTATIONS_H

#include <Eigen/Dense>

void vector_to_vector_rotation_matrix(
		const Eigen::Vector3f& v, const Eigen::Vector3f& w,
		bool normalized, bool lineThruW, Eigen::Matrix3f& M);

void angle_vector_rotation_matrix(
		float angle, const Eigen::Vector3f& u,
		Eigen::Matrix3f& M);

void cos_sin_angle_vector_rotation_matrix(
		float cosa, float sina, const Eigen::Vector3f& u,
		Eigen::Matrix3f& M);

#endif // ROTATIONS_H
