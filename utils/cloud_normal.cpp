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

                  F(N)  = Sum_i w_i (N.V_i)/N.N

   where V_i = c_i - p_0 for c_i the i^th cloud point, where w_i
   is a given distance-based weighting. We choose 
                  w_i  = 1.0/||V_i||^4
   Finding a zero gradient of F using matrix calculus yields the relation   
                  N = (N.N)(N.X)
   where
                  X = Sum_i (w_i V_i) / Sum_i 2w_i
   Thus we do niters number of iterations of

                  N_0 = some random non-zero vector.
		  N_{i+1} = (N_i.N_i)(N_i.X)

   hoping that this converges to a fixed point.		  


   @param[in]  p0: point to query normal.
   @param[in]  cloud: point cloud of n points as a 3 x n matrix.
   @param[in]  niters: number of iterations to optimize the normal.
   @param[in]  zeroTol: tolerance to zero vector length.

*/

Vector3f cloud_normal(Vector3f p0, const Matrix3Xf& cloud, int niters, double zeroTol = 0.0f)
{
  const float ONE_OVER_SQRT3 = 1.0f/sqrt(3.0f);
  
  Vector3f N(ONE_OVER_SQRT3, ONE_OVER_SQRT3, ONE_OVER_SQRT3);
  
  Vector3f X(0.0f, 0.0f, 0.0f);
  Vector3f V;	
  float wsum = 0.0f;	
  for(Index i=0; i < cloud.cols(); ++i) {	  
    V = cloud.col(i) - p0;
    float vv = V.dot(V);
    if(vv <= zeroTol) continue;
    wsum += 2.0f/(vv*vv);
    X += w*V; 
  }
  if(wsum <= zeroTol) return N;
  X = X/wsum;

  std::cout << (wsum*N.dot(X))/(N.dot(N)) << std::endl;
  float NN;
  float NX;
  for(int iter=0; iter < niters; ++iter) {
    NN = N.dot(N);
    NX = N.dot(X);
    N = NN*NX;
    std::cout << (wsum*N.dot(X))/(N.dot(N)) << std::endl;
  }	
  NN = N.dot(N);
  N = N/NN;
	
  return N;
}
