//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included with this distribution.


#ifndef PT_TO_PT_DISTSQ_H
#define PT_TO_PT_DISTSQ_H

#include <Eigen/Dense>

double pt_to_pt_distsq(const Eigen::VectorXf& v, const Eigen::VectorXf& w);

double pt_to_pt_distsq(const float v[3], const float w[3]);


#endif // PT_TO_PT_DISTSQ_H
