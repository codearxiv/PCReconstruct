//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

using Eigen::Vector3f;
using Eigen::Matrix;
using Eigen::Index;
using Eigen::Ref;

using Matrix3Xf = Matrix<float, 3, Eigen::Dynamic>;


//-------------------------------------------------------------------------
/**
   Finds a best fitting unit normal N at a point p0 w.r.t. a 3D point cloud.  
   In other words we minimize over nonzero vectors N

				  F(N)  = Sum_i w_i (N.V_i)/sqrt(N.N)

   where V_i = c_i - p_0 for c_i the i^th cloud point, where w_i
   is a given distance-based weighting. We choose 
                  w_i  = 1.0/||V_i||^4
   Finding a zero gradient of F using matrix calculus yields the relation   
				  N = ((N.N)/(N.X)) X
   where
				  X = Sum_i (w_i V_i)
   Thus we do niters number of iterations of

                  N_0 = some random non-zero vector.
				  N_{i+1} = (N_i.N_i)/(N_i.X) X

   hoping that this converges to a fixed point.		  


   @param[in]  p0: point to query normal.
   @param[in]  cloud: point cloud of n points as a 3 x n matrix.
   @param[in]  niters: number of iterations to optimize the normal.
   @param[in]  zeroTol: tolerance to zero vector length.

*/

Vector3f cloud_normal(Vector3f p0, const Matrix3Xf& cloud, int niters, double zeroTol = 0.0)
{
	const float ONE_OVER_SQRT3 = 1.0f/sqrt(3.0f);

	Vector3f N(ONE_OVER_SQRT3, ONE_OVER_SQRT3, ONE_OVER_SQRT3);

	Vector3f X(0.0f, 0.0f, 0.0f);
	Vector3f V;
	for(Index i=0; i < cloud.cols(); ++i) {
		V = cloud.col(i) - p0;
		float v4 = V.dot(V);
		v4 = v4*v4;
		if(v4 <= zeroTol) continue;
		float w = 1.0f/v4;
		X += w*V;
	}

	for(int iter=0; iter < niters; ++iter) {
		float NX = N.dot(X);
		std::cout << NX/N.norm() << std::endl;
		if(NX <= zeroTol) break;
		float t = (N.dot(N))/NX;
		N = t*X;
	}
	N.normalize();
	
	return N;
}
