#include "Plane.h"

void Plane::set(const Vector3f p0, const Vector3f norm)
{

	const float RRT2 = 1.0f/sqrt(2.0f);
	_p0 = p0;
	_norm = norm;

	if( abs(norm(0)) < RRT2 ) {
		_u(0) = 0.0f;
		_u(1) = norm(2);
		_u(2) = -norm(1);
	}
	else{
		_u(0) = -norm(2);
		_u(1) = 0.0f;
		_u(2) = norm(0);
	}
	_v = _u.cross(norm);

	_u.normalize();
	_v.normalize();

}

Eigen::Vector2f Plane::project_uv(const Vector3f q)
{
	Vector3f w = q - _p0;
	Vector2f puv;
	puv(0) = w.dot(_u);
	puv(1) = w.dot(_v);
	return puv;
}



Eigen::Vector3f Plane::project(const Vector3f q, Vector2f& puv)
{
	puv = project_uv(q);
	Vector3f p = puv(0)*_u + puv(1)*_v;
	return p;
}



