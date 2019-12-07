//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

using Eigen::VectorXf;
using Eigen::Index;

double pt_to_pt_distsq(const VectorXf& v, const VectorXf& w)
{
	double distsq = 0.0;
	for(Index i=0; i < v.size(); ++i) {
		distsq += (v[i]-w[i])*(v[i]-w[i]);
	}
	return distsq;
}
