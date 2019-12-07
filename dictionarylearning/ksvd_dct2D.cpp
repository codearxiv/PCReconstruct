//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#define EIGEN_NO_MALLOC
//#define __DEBUG_KSVDDCT

#include "ksvd_dct2D.h"
#include "cosine_transform.h"
#include "ensure_buffer_size.h"
#include "constants.h"

//#include <functional>
//#include <math.h>
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
using Eigen::LDLT;
//using Eigen::Ref;

template<typename T> using aligned = Eigen::aligned_allocator<T>;
using vectorfa = vector<float, aligned<float>>;
using vectoria = vector<int, aligned<int>>;

template<typename T> using Ref = Eigen::Ref<T, ALIGNEDX>;
using MapMtrxf = Map<MatrixXf, ALIGNEDX>;
using MapVectf = Map<VectorXf, ALIGNEDX>;


void column_normalize(Ref<MatrixXf> M, Ref<VectorXf> NrmInv);


//-----------------------------------------------------------

/**
   An extension of K-SVD to the domain of continuous signals and
   dictionary atoms, in this case realized by linear combinations
   of products of cosines of various frequencies coming from 2D
   discrete cosine transform sampled non-uniformly at given (u,v)
   coordinates associated to each signal's coordinates (samples).
   This allows signals of varying length as well estimating values
   of signals away from their sampled (u,v)-coordinates.

   C.f. "Cloud Dictionary: Coding and Modeling for Point Clouds",
   https://arxiv.org/abs/1612.04956

   Note: accuracy seems to drop of considerably as signals exceed
   nfreq^2 number of samples.

   @param[in]  useOpenMP: Whether to parallelize using OpenMP.
   @param[in]  Y: array of signal vectors (possibly of variable length).
   @param[in]  U: array of vectors of 'u' coordinates for each coordinate
			   of vector in Y.
   @param[in]  V: array of vectors of 'v' coordinates for each coordinate
			   of vector in Y.
   @param[in]  nfreq: Largest cosine frequency.
   @param[in]  latm: Sparsity constraint L.
   @param[in]  maxIters: Max. number of K-SVD iterations.
   @param[in]  maxError: Max. error ||Y-D*X||^2 before an iteration
			   can be aborted (< 0.0 for none).
   @param[in]  svdPowIters: Number of power iterations to approximate
			   first singular vectors.
   @param[in]  Sparse approx. functor.
   @param[in/out]  D in: first approx. (nfreq*nfreq) x m 'dictionary' matrix.
				   D out: learnt dictionary adapted to the signals Y.
   @param[out] X: m x nsig 'code' matrix.

*/


void ksvd_dct2D(
		bool useOpenMP,
		const vector<VectorXf>& Y,
		const vector<VectorXf>& U,
		const vector<VectorXf>& V,
		Index nfreq,
		Index latm,
		int maxIters,
		float maxError,
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

	Index nsig = Y.size();
	Index nfreqsq = nfreq*nfreq;
	Index natm = D.cols();
	assert(size_t(nsig) == U.size());
	assert(size_t(nsig) == V.size());
	assert(nfreqsq == D.rows());
	assert(natm == X.rows());
	assert(nsig == X.cols());
	assert(maxIters >= 1);
	assert(latm <= natm);
	Index maxThreads = omp_get_max_threads();

	bool stopAtMaxError = (maxError >= 0.0f);
	float maxErrorSq = 0.0f;
	bool smallError;	
	float* errsig = new float[nsig];
	for(Index i=0; i < nsig; ++i) { errsig[i] = float_infinity; }
	if( stopAtMaxError ) maxErrorSq = maxError*maxError;

	Index maxSamples = 0;
	Index totalSamplesPadded = 0;
	for(Index isig = 0; isig < nsig; ++isig){		
		Index nsmpl = Y[isig].size();
		maxSamples = std::max(maxSamples, nsmpl);
		totalSamplesPadded += align_padded(nsmpl);
	}

	vectorfa dworkZ;
	ensure_buffer_size(totalSamplesPadded, dworkZ);
	vector<MapVectf> Zblk(nsig, MapVectf(nullptr,0));

	Index p = 0;
	for(Index isig = 0; isig < nsig; ++isig){
		Index nsmpl = Y[isig].size();
		new (&Zblk[isig]) MapVectf(&dworkZ[p], nsmpl);
		p = p + align_padded(nsmpl);
	}

	bool aSigUsingAtm;
	MatrixXi iatmUsed(latm,nsig);
	VectorXi natmUsed(nsig);

	VectorXf TZS(nfreqsq);
	MatrixXf TTS(nfreqsq,nfreqsq);
	vector<VectorXf> TZSs(maxThreads,VectorXf(nfreqsq));
	vector<MatrixXf> TTSs(maxThreads,MatrixXf(nfreqsq,nfreqsq));

	Eigen::LDLT<MatrixXf> *ldlt = new LDLT<MatrixXf>(nfreqsq*nfreqsq);


#pragma omp parallel if(useOpenMP) default(shared) firstprivate(sparseFunct)
{


	int numThreads = omp_get_num_threads();
	int iThread = omp_get_thread_num();

	VectorXf TDatm(maxSamples);
	VectorXf TA(maxSamples);
	VectorXf TZ(nfreqsq);
	VectorXf Xsig(natm);
	VectorXf R(nfreqsq);

	size_t paddedA[2] = {
		align_padded(maxSamples*nfreqsq),
		align_padded(maxSamples*natm)
	};
	vectorfa dworkDctA(paddedA[0] + paddedA[1]);
	vectorfa dworkDctB;//(4*align_padded(maxSamples*nfreq));

	MapMtrxf T(nullptr, maxSamples, nfreqsq);
	MatrixXf TT(nfreqsq,nfreqsq);
	MapMtrxf TD(nullptr, maxSamples, natm);
	VectorXf NrmInv(natm);
	VectorXf TY(nfreqsq);

	for(int iter = 1; iter <= maxIters; ++iter){

#ifdef __DEBUG_KSVDDCT
#pragma omp single
{
		if(iter == 1) cout << "\nAverge error (coord. diff., cosine angle, length vect. diff.)\n" ;
		print_error_dct2D(Y, U, V, D, X, nfreq);
		cout << endl;
}
#endif

		// Fix dictionary D and optimize code matrix X.
#pragma omp for schedule(dynamic)
		for(Index isig = 0; isig < nsig; ++isig){
			Index nsmpl = Y[isig].size();
			new (&T) MapMtrxf(&dworkDctA[0], nsmpl, nfreqsq);
			cosine_transform(U[isig], V[isig], nfreq, dworkDctB, T);

			new (&TD) MapMtrxf(&dworkDctA[paddedA[0]], nsmpl, natm);
			TD.noalias() = T*D;
			column_normalize(TD, NrmInv);
			sparseFunct(Y[isig], TD, latm, Xsig, R);

			float error = R.dot(R);
			if( error <= errsig[isig] ) {
				X.col(isig) = Xsig.cwiseProduct(NrmInv);
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

} //single


		// ---
		// Now optimize dictionary D for the current code vector X
		// one column (atom) at a time.

		for(Index iatm = 0; iatm < natm; ++iatm){
			TTSs[iThread].setZero();
			TZSs[iThread].setZero();
#pragma omp for schedule(static)
			for(Index isig = 0; isig < nsig; ++isig){
				if(X(iatm,isig) == 0.0f) continue;
				Index nsmpl = Y[isig].size();
				new (&T) MapMtrxf(&dworkDctA[0], nsmpl, nfreqsq);
				cosine_transform(U[isig], V[isig], nfreq, dworkDctB, T);

				// Compute the residual Z for current atom and signal.
				MapVectf& Z = Zblk[isig];
				Z = Y[isig];
				for(Index i = 0; i < natmUsed(isig); ++i){
					Index jatm = iatmUsed(i,isig);
					if( jatm == iatm ) continue;
					TDatm.segment(0,nsmpl).noalias() = T*D.col(jatm);
					Z -= X(jatm,isig)*TDatm.segment(0,nsmpl);
				}

				TT.noalias() = T.transpose() * T;
				TA.segment(0,nsmpl).noalias() = T*D.col(iatm);
				float normsqTA = TA.segment(0,nsmpl).squaredNorm();
				TZ.noalias() = T.transpose() * Z;
				float r = Z.dot(TA.segment(0,nsmpl)) / normsqTA;
				TTSs[iThread] += (r*r)*TT;
				TZSs[iThread] += r*TZ;
				aSigUsingAtm = true;
			}


#pragma omp single
{
			if( aSigUsingAtm ){
				TTS.setZero();
				TZS.setZero();
				for(int i = 0; i < numThreads; ++i){
					TTS += TTSs[i];
					TZS += TZSs[i];
				}
				// This is an analog of a single SVD power
				// iteration. Now the vector optimized to
				// best match each residual is constrained
				// to be a linear transformation of another
				// vector.
				// It can be worked out with some matrix
				// calculus by minimizing the sum of inner
				// products:
				//
				//    Sum_i(Z_i-a_iT_iD').(Z_i-a_iT_iD')
				//
				// over D' and reals a_i, where D' is the
				// atom being optimized, Z_i the residual
				// for the ith signal that uses atom D',
				// and T_i the cosine transform for this
				// signal. The a_i's are the projections
				// Z_i.(T_iD')/(T_iD').(T_iD') .
				ldlt->compute(TTS);
				D.col(iatm) = ldlt->solve(TZS); // Optimized atom
			}

} //single
			if( aSigUsingAtm ){
#pragma omp for schedule(static)
				for(Index isig = 0; isig < nsig; ++isig){
					if(X(iatm,isig) == 0.0f) continue;
					Index nsmpl = Y[isig].size();
					new (&T) MapMtrxf(&dworkDctA[0], nsmpl, nfreqsq);
					cosine_transform(U[isig], V[isig], nfreq, dworkDctB, T);
					TA.segment(0,nsmpl).noalias() = T*D.col(iatm);
					float normsqTA = TA.segment(0,nsmpl).squaredNorm();
					// The projection coefficients describe the code vector
					// corresponding to the updated atom.
					X(iatm,isig) = Zblk[isig].dot(TA.segment(0,nsmpl)) / normsqTA;
				}
			}


		}
	}


} //parallel



	if( stopAtMaxError ) delete[] errsig;

	delete ldlt;


}


//-----------------------------------------------------------

void print_error_dct2D(
		const vector<VectorXf>& Y,
		const vector<VectorXf>& U,
		const vector<VectorXf>& V,
		const MatrixXf& D,
		const MatrixXf& X,
		Index nfreq)
{
	Index nsig = Y.size();
	Index nfreqsq = nfreq*nfreq;
	MapMtrxf T(nullptr, 0, 0);
	MapVectf TDX(nullptr, 0);
//	VectorXf TDX;

	vectorfa dworkA;
	vectorfa dworkB;
	size_t paddedA[2];

	float error = 0.0f;
	float error2 = 0.0f;
	float error3 = 0.0f;
	Index totalsmpl = 0;

	for(Index isig = 0; isig < nsig; ++isig){
		Index nsmpl = Y[isig].size();
		paddedA[0] = align_padded(nsmpl*nfreqsq);
		paddedA[1] = align_padded(nsmpl);
		ensure_buffer_size(paddedA[0]+paddedA[1], dworkA);
		new (&T) MapMtrxf(&dworkA[0], nsmpl, nfreqsq);
		new (&TDX) MapVectf(&dworkA[paddedA[0]], nsmpl);
		cosine_transform(U[isig], V[isig], nfreq, dworkB, T);
		TDX.noalias() = T*D*(X.col(isig));
		error += (Y[isig] - TDX).cwiseAbs().sum();
		error2 +=  Y[isig].dot(TDX)/(Y[isig].norm() * TDX.norm());
		error3 += (Y[isig] - TDX).norm()/Y[isig].norm();
		totalsmpl = totalsmpl + nsmpl;
	}
	cout << error/totalsmpl << ", " << error2/nsig  << ", " << error3/nsig;


}
//-----------------------------------------------------------


void column_normalize(Ref<MatrixXf> M, Ref<VectorXf> NrmInv)
{
	for(Index i=0; i < M.cols(); ++i) {
		NrmInv(i) = 1.0f/M.col(i).norm();
		M.col(i) *= NrmInv(i);
	}
}


//-----------------------------------------------------------
