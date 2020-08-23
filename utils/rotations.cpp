//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included with this distribution.


#include <Eigen/Geometry>
#include <math.h>
#include "rotations.h"
#include "constants.h"

#include <Eigen/Dense>

using Eigen::Vector3f;
using Eigen::Matrix3f;


//-----------------------------------------------------------
// Returns the rotation matrix M that rotates vector v to a
// vector w (or to a line through w if lineThruW is true).

void vector_to_vector_rotation_matrix(
		const Vector3f& v, const Vector3f& w,
		bool normalized, bool lineThruW, Matrix3f& M)
{

	Vector3f vxw = v.cross(w);
	float vxwLen = vxw.norm();
	float cosa = v.dot(w);
	float sina = vxwLen;

	if( lineThruW ){
		if( cosa < 0.0f ){
			cosa = -cosa;
			vxw = -vxw;
		}
	}

	if( !normalized ){
		float vvww = sqrt(v.dot(v)*w.dot(w));

		if( vvww > float_tiny ){
			cosa = cosa/vvww;
			sina = sina/vvww;
		}
		else{
			cosa = 1.0f;
			sina = 0.0f;
		}
	}

	if( vxwLen > float_tiny ){
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
	 a unit vector U by an angle. U must have unit length.
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

	M(0,0) = tu(0)*u(0) + cosa;
	M(1,0) = tu(0)*u(1) + su(2);
	M(2,0) = tu(0)*u(2) - su(1);

	M(0,1) = tu(1)*u(0) - su(2);
	M(1,1) = tu(1)*u(1) + cosa;
	M(2,1) = tu(1)*u(2) + su(0);

	M(0,2) = tu(2)*u(0) + su(1);
	M(1,2) = tu(2)*u(1) - su(0);
	M(2,2) = tu(2)*u(2) + cosa;
}
//-----------------------------------------------------------
