//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#define _USE_MATH_DEFINES
#define EIGEN_NO_MALLOC

#include "cosine_transform.h"
#include "ensure_buffer_size.h"
#include "constants.h"

#include <math.h>
//#include <vector>
//#include <cmath>
//#include <Eigen/Dense>



using std::max;
using std::vector;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::VectorXf;
using Eigen::Index;
using Eigen::Map;
using Eigen::Dynamic;
//using Eigen::Ref;
template<typename T> using Ref = Eigen::Ref<T, ALIGNEDX>;


using alloc = Eigen::aligned_allocator<float>;

//-----------------------------------------------------------

/**
   2D non-uniform discrete cosine transform sampled at (u,v)
   coordinates in vectors U and V.

   @param[in]      U: size n vector of u coordinates.
   @param[in]      V: size n vector of v coordinates.
   @param[in]      nfreq: max frequency in the u and v directions
				   of transform.
   @param[in/out]  dwork:  work-array. Reallocated to correct size
				   if too small.
   @param[out] T:  2D cosine transform, a dimension n x (nfreq*nfreq)
				   matrix (must already be of this size on input).
*/


void cosine_transform(
		const VectorXf& U,
		const VectorXf& V,
		Index nfreq,
		vector<float,alloc>& dwork,
		Ref<MatrixXf> T
		)
{
	Index nsmpl = U.size();
	Index nfreqsq = nfreq*nfreq;
	size_t nf = nsmpl*nfreq;

	assert(nsmpl == V.size());
	assert(nsmpl == T.rows());
	assert(nfreqsq == T.cols());

	Map<MatrixXf> CU(nullptr, nsmpl, nfreq);
	Map<MatrixXf> SU(nullptr, nsmpl, nfreq);
	Map<MatrixXf> CV(nullptr, nsmpl, nfreq);
	Map<MatrixXf> SV(nullptr, nsmpl, nfreq);

	size_t p = align_padded(nf);
	ensure_buffer_size(4*p, dwork);
	new (&SU) Map<MatrixXf, ALIGNEDX>(&dwork[0], nsmpl, nfreq);
	new (&CU) Map<MatrixXf, ALIGNEDX>(&dwork[p], nsmpl, nfreq);
	new (&SV) Map<MatrixXf, ALIGNEDX>(&dwork[2*p], nsmpl, nfreq);
	new (&CV) Map<MatrixXf, ALIGNEDX>(&dwork[3*p], nsmpl, nfreq);

	float maxu = 0.0f;
	float maxv = 0.0f;
	for(int k=0; k<nsmpl; ++k){
		maxu = max(maxu, abs(U(k)));
		maxv = max(maxv, abs(V(k)));
	}

	//----

	float scaleu = (maxu > 0.0 ? M_PI/maxu : M_PI);
	float scalev = (maxv > 0.0 ? M_PI/maxv : M_PI);

//	for(Index i=0; i<nfreq; ++i){
//		float scaleui = scaleu*i;
//		float scalevi = scalev*i;
//		for(Index k=0; k<nsmpl; ++k){
//			CU(k,i) = cos(scaleui*U(k));
//			CV(k,i) = cos(scalevi*V(k));
//		}
//	}

	// This part is a bit performance sensitive. Instead of
	// computing cosines over each frequency directly, compute
	// the lowest frequency and apply angle-sum trig. identities
	// to recursively compute the remaining frequencies. We can
	// use a quick sin/cos approximation for the lowest frequency
	// since we are in the range (-PI,PI).

//	std::function< float(float) > approx_sin = [=](float t) {
//		if(t < 0){ return 1.27323954f*t + 0.405284735f*t*t; }
//		else{ return 1.27323954f*t - 0.405284735f*t*t; }
//	};
//	std::function< float(float) > approx_cos = [=](float t) {
//		return approx_sin(t+1.57079632f);
//	};

	for(Index k=0; k<nsmpl; ++k){
		CU(k,0) = 1.0f;
		CV(k,0) = 1.0f;
//		SU(k,1) = approx_sin(scaleu*U(k));
//		CU(k,1) = approx_cos(scaleu*U(k));
//		SV(k,1) = approx_sin(scalev*V(k));
//		CV(k,1) = approx_cos(scalev*V(k));
		SU(k,1) = sin(scaleu*U(k));
		CU(k,1) = cos(scaleu*U(k));
		SV(k,1) = sin(scalev*V(k));
		CV(k,1) = cos(scalev*V(k));
	}

	for(Index i=2; i<nfreq; ++i){
		for(Index k=0; k<nsmpl; ++k){
			SU(k,i) = SU(k,i-1)*CU(k,1) + CU(k,i-1)*SU(k,1);
			CU(k,i) = CU(k,i-1)*CU(k,1) - SU(k,i-1)*SU(k,1);
			SV(k,i) = SV(k,i-1)*CV(k,1) + CV(k,i-1)*SV(k,1);
			CV(k,i) = CV(k,i-1)*CV(k,1) - SV(k,i-1)*SV(k,1);
		}
	}

	//----


	Index l = 0;
	for(Index i=0; i<nfreq; ++i){
		for(Index j=0; j<nfreq; ++j){
			for(Index k=0; k<nsmpl; ++k){
				T(k,l) = CU(k,i)*CV(k,j);
			}
			l = l + 1;
		}
	}

}

//-----------------------------------------------------------
