//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#define EIGEN_NO_MALLOC

#include "MatchingPursuit.h"
#include "ensure_buffer_size.h"
#include "constants.h"

//-------------------------------------------------------------------------
void MatchingPursuit::ensure(
		Index nd, Index na, Index lm)
{
	if(nd != ndim || na != natm || lm != lmax){
		ndim = nd;
		natm = na;
		lmax = (na < lm) ? na : lm;
	}
}


//-------------------------------------------------------------------------
/**
   Given a size n signal vector Y, an n x m dictionary matrix D of
   m atoms (unit column vectors of size n), and an integer L,
   our task is to find a sparse size m code vector X that encodes Y
   as closely as possible using no more than L atoms in D. In detail,
   we try to solve to minimization problem

                  min_X ||Y - DX||_F
   subject to
                  ||X||_0  <=  L

   where ||.||_F is the matrix Frobenius norm, and ||.||_0 is the
   vector L_0 norm (the number of non-zero entries in a vector).

   This is a greedy approach using the matching pursuit algorithm.

   @param[in]  Y: size n vector.
   @param[in]  D: n x m dictionary matrix.
   @param[in]  latm: Sparsity constraint L.
   @param[out] X: size m code vector.
   @param[out] R: size n residual vector.

*/



void MatchingPursuit::operator()(
		const VectorXf& Y, const MatrixXf& D, Index latm, VectorXf& X, VectorXf& R)
{
	assert(D.rows() == Y.rows());
	assert(D.cols() == X.rows());
	ensure(D.rows(), D.cols(), latm);

	X.setZero();
	R = Y;

	for(Index j = 1; j <= latm; ++j){
		float absprojmax = -float_infinity;
		float projmax = 0;
		Index imax = 0;
		for(Index i = 0; i < natm; ++i){
			if( X(i) != 0.0f ) continue;
			float proj = R.dot(D.col(i));
			float absproj = abs(proj);
			if( absproj > absprojmax ){
				projmax = proj;
				absprojmax = absproj;
				imax = i;
			}
		}
		X(imax) = projmax;
		R = R - projmax*D.col(imax);
	}
}
//-------------------------------------------------------------------------
