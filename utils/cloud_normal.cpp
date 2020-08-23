//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included with this distribution.


#include "cloud_normal.h"
#include "ensure_buffer_size.h"
#include "constants.h"

#include <math.h>
#include <vector>

using Eigen::Vector3f;
//using Eigen::Matrix;
using Eigen::Index;
using Eigen::Ref;

template<typename T> using vector = std::vector<T>;


//-------------------------------------------------------------------------
/**
   Finds a best fitting unit normal N at a point p0 w.r.t. a point cloud.
   We do this by minimizing over nonzero vectors N the sum of squares

				  F(N)  =  Sum_i w_i (N.V_i)^2 / (N.N)

   where V_i = (c_i-p0)/|c_i-p0| for c_i the i^th cloud point, where w_i
   is a given weighting. We simply take each w_i = 1.0.

   The gradient of F is
				  Grad F = Sum_i w_i X_i
   where
				  X_i = 2(N.V_i)/(N.N) V_i - 2 (N.V_i)^2/(N.N)^2 N

   with which we do gradient descent.


   @param[in]  p0: point to query normal.
   @param[in]  cloud: point cloud of n points.
   @param[in]  niters: number of iterations to optimize the normal.
   @param[inout]  vwork: workspace for V_i's. Resized automatically.

*/

Vector3f cloud_normal(const Vector3f& p0, const vector<Vector3f>& cloud,
					  int niters, std::vector<Eigen::Vector3f>& vwork)
{
	static const float ONE_OVER_SQRT3 = 1.0f/sqrt(3.0f);

	Vector3f N(ONE_OVER_SQRT3, ONE_OVER_SQRT3, ONE_OVER_SQRT3);

	size_t npoints = cloud.size();

	vwork.resize(npoints);
	for(size_t i=0; i < npoints; ++i) {
		vwork[i] = (cloud[i] - p0).normalized();
	}

	const double s = 0.5/niters;
	for(int iter=0; iter < niters; ++iter) {
		Vector3f G(0.0f, 0.0f, 0.0f);
		//float E = 0.0f;
		for(size_t i=0; i < npoints; ++i) {
			Vector3f V = vwork[i];
			double t = (N.dot(V));
			Vector3f X = t*(V - t*N);
			G += X;
			//E += t*t;
		}

		double rate = s*double(niters-iter);
		Vector3f N1 = N - rate*G.normalized();
		float N1N1 = N1.dot(N1);
		if( N1N1 <= float_tiny ) break;
		N = N1/sqrt(N1N1);

	}

//	float minE = float_infinity;
//	for(int iter=0; iter < 100; ++iter) {
//		Vector3f N1;
//		N1.setRandom().normalized();
//		float E = 0.0f;
//		for(size_t i=0; i < npoints; ++i) {
//			Vector3f V = (cloud[i] - p0).normalized();
//			double t = (N1.dot(V));
//			E += t*t;
//		}
//		if( E < minE ){
//			minE = E;
//			N = N1;
//		}
//	}


	return N;
}
