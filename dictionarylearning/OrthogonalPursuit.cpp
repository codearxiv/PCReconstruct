//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#define EIGEN_NO_MALLOC

#include "OrthogonalPursuit.h"
#include "ensure_buffer_size.h"
#include "constants.h"
//#include <functional>

using Eigen::Matrix;
using Eigen::Map;
using Eigen::Dynamic;
using Eigen::LDLT;
using Eigen::Aligned16;



//-------------------------------------------------------------------------
void OrthogonalPursuit::ensure(Index nd, Index na, Index lm)
{
	if(nd != ndim || na != natm || lm != lmax){
		ndim = nd;
		natm = na;
		lmax = (na < lm) ? na : lm;
		ensureWorkspace();
	}
}

//-------------------------------------------------------------------------

void OrthogonalPursuit::ensureWorkspace()
{
	// Allocate more workspace as necessary.
	//std::function< size_t(Index) > align_padded =
	//		[=](Index n) ->size_t { return ALIGNEDX*(1+(n/ALIGNEDX)); };
	size_t paddednd = align_padded(ndim);
	size_t paddedlm = align_padded(lmax);
	size_t paddedndlm = align_padded(ndim*lmax);
	size_t paddedlmsq = align_padded(lmax*lmax);

	size_t ndworkNeed = paddednd + 3*paddedlm + 2*paddedndlm + 2*paddedlmsq;
	ensure_buffer_size(ndworkNeed+ndworkNeed/2, dwork);

	// Map to pre-allocated workspace.
	size_t p = 0;
	new (&U) MapVectf(&dwork[0],ndim);
	p = p + paddednd;
	new (&V) MapVectf(&dwork[p],lmax);
	p = p + paddedlm;
	new (&W) MapVectf(&dwork[p],lmax);
	p = p + paddedlm;
	new (&XI) MapVectf(&dwork[p],lmax);
	p = p + paddedlm;
	new (&E) MapMtrxf(&dwork[p],ndim,lmax);
	p = p + paddedndlm;
	new (&F) MapMtrxf(&dwork[p],lmax,lmax);
	p = p + paddedlmsq;
	dworkOffset = p;

	size_t niworkNeed = lmax;
	ensure_buffer_size(niworkNeed+niworkNeed/2, iwork);

	// Map to pre-allocated workspace.
	new (&I) Map<Matrix<Index,Dynamic,1>>(&iwork[0],lmax);

    if( lmax*lmax > ldltSize ){
        ldltSize = lmax*lmax;
        delete ldlt;
		ldlt = new LDLT<MatrixXf>(ldltSize);
    }
}

//-------------------------------------------------------------------------
/**
   Orthogonal Matching Pursuit:

   Similar to matching pursuit, but provides a better approximation
   at the expense of significantly more computation. Namely, updates
   all the coefficients of the current code vector X' at each iteration
   so that DX' is an orthogonal projection of the signal vector Y onto
   the subspace spanned by the dictionary atoms corresponding to the
   nonzero entries of X'.

   @param[in]  Y: size n vector.
   @param[in]  D: n x m dictionary matrix of unit column vectors.
   @param[in]  latm: Sparsity constraint.
   @param[out] X: size m code vector.
   @param[out] R: size n residual vector.

*/



void OrthogonalPursuit::operator() (
		const VectorXf& Y, const MatrixXf& D, Index latm,
		VectorXf& X, VectorXf& R)
{
	assert(D.rows() == Y.rows());
	assert(D.cols() == X.rows());
	ensure(D.rows(),D.cols(),latm);

	X.setZero();
    R = Y;    

	for(Index j = 1; j <= latm; ++j){
        // Find the next 'nearest' atom to current residual R.
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
		U = D.col(imax);   // Dictionary atom U 'nearest' to R
        E.col(j-1) = U;    // ...save it in j^th column of E
		I(j-1) = imax;     // ...and save column index of U
		X(imax) = 1.0f;    // Set temporarily 1.0 to mark traversed.

        // Map to pre-allocated workspace.
        Index p = dworkOffset;
		new (&ETblk) MapMtrxf(&dwork[p],j,ndim);
		p = p + align_padded(j*ndim);
		new (&Fblk) MapMtrxf(&dwork[p],j,j);

        // With U added to the current set E of j nearest atoms,
        // optimise the coefficients of XI w.r.t this E. This is
        // done by projecting Y onto the subspace spanned by E.
        if( j > 1 ) {
			// Compute the product E^T(:,1:j) * E(:,1:j),
			// This can be done quicker by reusing the product
            // E^T(:,1:j-1) * E(:,1:j-1) from the previous
            // iteration.

            V.segment(0,j-1).noalias() = U.transpose() * E.block(0,0,ndim,j-1);
            F.col(j-1).segment(0,j-1) = V.segment(0,j-1);
            F.row(j-1).segment(0,j-1) = V.segment(0,j-1);
			F(j-1,j-1) = 1.0f;

            Fblk = F.block(0,0,j,j);
            ETblk = E.block(0,0,ndim,j).transpose();
            W.segment(0,j).noalias() = ETblk * Y;
			// Solve (E^T*E)*XI = (E^T)*Y
            ldlt->compute(Fblk);
            XI.segment(0,j) = ldlt->solve(W.segment(0,j));

            //Update residual R
            R = Y;
            R.noalias() -= (E.block(0,0,ndim,j))*(XI.segment(0,j));
        }
        else{
			F(0,0) = 1.0f;
            XI(0) = Y.dot(U);
            //Update residual R
            R = Y;
            R.noalias() -= XI(0)*U;
        }
    }

    // Map back to code vector.
	for(Index i = 0; i < latm; ++i){
        X(I(i)) = XI(i);
    }


}
//-------------------------------------------------------------------------
