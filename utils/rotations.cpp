#include <Eigen/Geometry>
#include <math.h>
#include "rotations.h"


using Eigen::Vector3f;
using Eigen::Matrix3f;


//-----------------------------------------------------------
/*
	 Returns the rotation matrix M that rotates vector v to vector w.
*/
void vector_to_vector_rotation_matrix(
		const Vector3f& v, const Vector3f& w,
		bool normalized, Matrix3f& M)
{
	  Vector3f vxw = v.cross(w);
	  float vxwLen = vxw.norm();

	  float cosa = v.dot(w);
	  float sina = vxwLen;

	  if( !normalized ){
		  float vvww = sqrt(v.dot(v)*w.dot(w));

		  if( vvww > 0.0f ){
			  cosa = cosa/vvww;
			  sina = sina/vvww;
		  }
		  else{
			  cosa = 1.0f;
			  sina = 0.0f;
		  }
	  }

	  if( vxwLen > 0.0f ){
		  Vector3f u = vxw/vxwLen;
		  cos_sin_angle_vector_rotation_matrix(cosa, sina, u, M);
	  }
	  else{
		  M.setIdentity();
		  if( cosa < 0.0f ) M = -M;
	  }

}

//-----------------------------------------------------------
/*
	 Returns the counter-clockwise rotation matrix M around
	 a unit vector U by an angle. U must have unit lenght.
*/
void angle_vector_rotation_matrix(
		float angle, const Vector3f& u, Matrix3f& M)
{
	  cos_sin_angle_vector_rotation_matrix(cos(angle), sin(angle), u, M);
}

//-----------------------------------------------------------

void cos_sin_angle_vector_rotation_matrix(
		float cosa, float sina, const Vector3f& u,
		Matrix3f& M)
{

	  Vector3f tu = (1.0f-cosa)*u;
	  Vector3f su = sina*u;

	  M(1,1) = tu(1)*u(1) + cosa;
	  M(1,2) = tu(2)*u(1) - su(3);
	  M(1,3) = tu(3)*u(1) + su(2);

	  M(2,1) = tu(1)*u(2) + su(3);
	  M(2,2) = tu(2)*u(2) + cosa;
	  M(2,3) = tu(3)*u(2) - su(1);

	  M(3,1) = tu(1)*u(3) - su(2);
	  M(3,2) = tu(2)*u(3) + su(1);
	  M(3,3) = tu(3)*u(3) + cosa;
}
//-----------------------------------------------------------
