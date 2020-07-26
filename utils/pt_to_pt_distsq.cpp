//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "pt_to_pt_distsq.h"

//-----------------------------------------------------------

double pt_to_pt_distsq(const Eigen::VectorXf& v, const Eigen::VectorXf& w)
{
	double distsq = 0.0;
	for(size_t i=0; i < v.size(); ++i) {
		distsq += (v[i]-w[i])*(v[i]-w[i]);
	}
	return distsq;
}
//-----------------------------------------------------------

double pt_to_pt_distsq(const float v[3], const float w[3])
{
	double distsq = 0.0;
	for(size_t i=0; i < 3; ++i) {
		distsq += (v[i]-w[i])*(v[i]-w[i]);
	}
	return distsq;
}
//-----------------------------------------------------------
