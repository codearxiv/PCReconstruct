//-------------------------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

//#include <math.h>
#include "cloud_normal.h"
#include "ensure_buffer_size.h"

using Eigen::Vector3f;
//using Eigen::Matrix;
using Eigen::Index;
using Eigen::Ref;

template<typename T> using vector = std::vector<T>;


//-------------------------------------------------------------------------
/**
   Finds a best fitting unit normal N at a point p0 w.r.t. a 3D point cloud.  
   We do this by minimizing over nonzero vectors N the sum of squares

				  F(N)  =  Sum_i w_i (N.V_i)^2 / (N.N)

   where V_i = c_i - p_0 for c_i the i^th cloud point, where w_i is a
   distance-based weighting. We avoid weighting and choose each w_i = 1.0.

   The gradient of F is
				  Grad F = Sum_i w_i X_i
   where
				  X_i = 2(N.V_i)/(N.N) V_i - 2 (N.V_i)^2/(N.N)^2 N

   with which we do gradient descent.


   @param[in]  p0: point to query normal.
   @param[in]  cloud: point cloud of n points.
   @param[in]  niters: number of iterations to optimize the normal.
   @param[in]  zeroTol: tolerance to zero vector length.

*/

Vector3f cloud_normal(const Vector3f& p0, const vector<Vector3f>& cloud,
					  int niters, double zeroTol)
{
	static const float ONE_OVER_SQRT3 = 1.0f/sqrt(3.0f);

	size_t npoints = cloud.size();

	Vector3f N(ONE_OVER_SQRT3, ONE_OVER_SQRT3, ONE_OVER_SQRT3);
	double rate = 0.5;

	for(int iter=0; iter < niters; ++iter) {
		Vector3f G(0.0f, 0.0f, 0.0f);
		//float E = 0.0f;
		for(size_t i=0; i < npoints; ++i) {
			Vector3f V = cloud[i] - p0;
			double t = (N.dot(V));
			Vector3f X = 2.0f*t*(V - t*N);
			G += X;
			//E += t*(N.dot(V));
		}

		Vector3f N1 = N - rate*G.normalized();
		if(N1.dot(N1) <= zeroTol) break;
		N = N1.normalized();//
		rate = 0.9*rate;

	}

	return N;
}
