//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#define EIGEN_NO_MALLOC
//#define __DEBUG_KSVD

#include "ksvd.h"
#include "constants.h"
//#include "Cover_Tree.h"
//#include <functional>
//#include <math.h>
//#include <vector>
//#include <Eigen/Dense>
//#include <omp.h>


using std::cout;
using std::endl;
using std::vector;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Matrix;
using Eigen::VectorXf;
using Eigen::VectorXi;
using Eigen::Index;
using Eigen::Map;
using Eigen::Dynamic;
using Eigen::Aligned16;


//-----------------------------------------------------------

/**
   The K-SVD dictionary learning algorithm.

   Provided an integer L and n x k matrix Y consisting of our
   k size n signal vectors, we look for an n x m dictionary
   matrix D of m atoms (unit column vectors of size n) and a
   sparse m x k matrix X of column code vectors that encode the
   signals in Y as closely as possible using no more than L
   atoms in D. In detail, we solve the minimization problem

                  min_{X, D} ||Y - DX||_F
   subject to
                  ||X||_0  <=  L

   where ||.||_F is the matrix Frobenius norm, and ||.||_0 is the
   vector L_0 norm (the number of non-zero entries in a vector).


   @param[in]  useOpenMP: Whether to parallelize using OpenMP.
   @param[in]  Y: n x k signal matrix.
   @param[in]  latm: Sparsity constraint L.
   @param[in]  maxIters: Max. number of K-SVD iterations.
   @param[in]  maxError: Max. error ||Y-D*X||^2 before an iteration
			   can be aborted (< 0.0 for none).
   @param[in]  svdPowIters: Number of power iterations to approximate
			   first singular vectors.
   @param[in]  Sparse approximation functor.
   @param[in/out]  D in: first approximation n x m 'dictionary' matrix.
				   D out: learnt dictionary adapted to the signals Y.
   @param[out] X: m x k 'code' matrix.

*/


void ksvd(
		bool useOpenMP,
		const MatrixXf& Y,
		Index latm,
        int maxIters,
		float maxError,
        int svPowIters,
        const std::function<void(
			const VectorXf&,
			const MatrixXf&,
            Index,
			VectorXf&,
			VectorXf&)> sparseFunct,
		MatrixXf& D,
		MatrixXf& X
        )
{
    Index ndim = D.rows();
    Index natm = D.cols();
    Index nsig = Y.cols();
    assert(ndim == Y.rows());
    assert(natm == X.rows());
    assert(nsig == X.cols());
    assert(maxIters >= 1);
	assert(svPowIters >= 1);
	assert(latm <= ndim && latm <= natm);

	bool stopAtMaxError = (maxError >= 0.0f);
	float maxErrorSq = 0.0f;
	bool smallError;
	float* errsig = new float[nsig];
	for(Index i=0; i < nsig; ++i) { errsig[i] = float_infinity; }
	if( stopAtMaxError ) maxErrorSq = maxError*maxError;

	float *dwork = new float[nsig*ndim];

	MatrixXf Z(ndim,nsig);
	Map<MatrixXf, ALIGNEDX> Zblk(nullptr, 0, 0);
	VectorXf ZTA(nsig);
	VectorXf ZZTA(ndim);
	VectorXf A(ndim);
	VectorXf B(nsig);
	MatrixXi iatmUsed(latm,nsig);
	VectorXi natmUsed(nsig);

#pragma omp parallel if(useOpenMP) default(shared) firstprivate(sparseFunct)
{

	VectorXf Ysig(ndim);
	VectorXf Xsig(natm);
	VectorXf R(ndim);

    for(int iter = 1; iter <= maxIters; ++iter){
//***
#pragma omp single
{
#ifdef __DEBUG_KSVD
		if(iter == 1) cout << "\nAverge error (coordinate difference):\n" ;
		cout << (Y-(D*X)).cwiseAbs().sum()/(ndim*nsig) << endl;
#endif
}

		// Fix dictionary D and optimize code matrix X.
#pragma omp for schedule(dynamic)
		for(Index isig = 0; isig < nsig; ++isig){
			Ysig = Y.col(isig);
			sparseFunct(Ysig, D, latm, Xsig, R);
			float error = R.dot(R);
			if( error <= errsig[isig] ) {
				X.col(isig) = Xsig;
				errsig[isig] = error;
			}
		}

		if( stopAtMaxError ){
			// Stop if Y and D*X are similar within tolerance.
#pragma omp single
{
			smallError = true;
			for(Index isig = 0; isig < nsig; ++isig){
				if( errsig[isig] > maxErrorSq ){
					smallError = false;
					break;
				}
			}
}//single
			if( smallError ) break;
		}
		// ---
        // Now optimize dictionary D for the current code vector X
        // one column (atom) at a time.
//#pragma omp for schedule(dynamic)
#pragma omp single
{
		// queue up atoms used by each signal
		for(Index isig = 0; isig < nsig; ++isig){
			int ic = 0;
			for(Index iatm = 0; iatm < natm; ++iatm){
				if( X(iatm,isig) == 0.0f ) continue;
				iatmUsed(ic,isig) = int(iatm);
				++ic;
				if(ic >= latm) break;
			}
			natmUsed(isig) = ic;
		}


        for(Index iatm = 0; iatm < natm; ++iatm){
			A = D.col(iatm); // Original atom is our initial approx.
			// Compute the matrix Z of residuals for current atom.
			Index nsigUsing = 0;
			for(Index isig = 0; isig < nsig; ++isig){
				if( X(iatm,isig) == 0.0f ) continue;
				Z.col(nsigUsing) = Y.col(isig);
				for(Index i = 0; i < natmUsed(isig); ++i){
					Index jatm = iatmUsed(i,isig);
                    if( jatm == iatm ) continue;
					Z.col(nsigUsing) -= X(jatm,isig)*D.col(jatm);
                }
				++nsigUsing;
            }

			if( nsigUsing == 0 ) continue;

			// Map to workspace
			new (&Zblk) Map<MatrixXf, ALIGNEDX>(dwork,ndim,nsigUsing);
			Zblk = Z.block(0,0,ndim,nsigUsing);

			// We only need the first singular vector, do a power
			// iteration to approximate it. This is our new improved atom.
			for(int i=1; i <= svPowIters; ++i){
				ZTA.segment(0,nsigUsing).noalias() = Zblk.transpose()*A;
				ZZTA.noalias() = Zblk*ZTA.segment(0,nsigUsing);
				A = ZZTA.normalized(); // Optimized atom
			}

			// The projection coefficients describe the code vector
			// corresponding to the updated atom.
			B.segment(0,nsigUsing).noalias() = Zblk.transpose()*A;
			Index ic2 = 0;
            for(Index isig = 0; isig < nsig; ++isig){
				if( X(iatm,isig) == 0.0f ) continue;
				X(iatm,isig) = B(ic2);
                ++ic2;
            }

			D.col(iatm) = A;
        }

}//single
	}

}//parallel

	delete[] dwork;
	if( stopAtMaxError ) delete[] errsig;

}

//-----------------------------------------------------------

