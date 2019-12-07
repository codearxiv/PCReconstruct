//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#ifndef MATCHINGPURSUIT_H
#define MATCHINGPURSUIT_H

#include "constants.h"

class MatchingPursuit
{
	using Index = Eigen::Index;
	using MatrixXf = Eigen::MatrixXf;
	using VectorXf = Eigen::VectorXf;

public:
	MatchingPursuit() :
		ndim(0), natm(0), lmax(0) {}

	MatchingPursuit(const MatchingPursuit &mp) : MatchingPursuit() {}
	//{ ensure(sa.ndim, sa.natm, sa.lmax); }

	~MatchingPursuit(){}

	void ensure(Index nd, Index na, Index lm);

	void operator() (
			const VectorXf& Y, const MatrixXf& D, Index l,
			VectorXf& X, VectorXf& R);

private:
	Index ndim; // Size of signal vector.
	Index natm; // No. of 'atoms' in dictionary.
	Index lmax; // maximum sparsity constraint <= natm.

};

#endif // MATCHINGPURSUIT_H
