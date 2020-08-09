//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.


#ifndef PLANE_H
#define PLANE_H

#include <Eigen/Dense>

class Plane
{
	using Index = Eigen::Index;
	using Matrix3f = Eigen::Matrix3f;
	using Vector3f = Eigen::Vector3f;
	using Vector2f = Eigen::Vector2f;

public:
	Plane(const Vector3f p0, const Vector3f norm) { set(p0,norm); }

	void set(const Vector3f p0, const Vector3f norm);
	Vector2f project_uv(const Vector3f q);
	Vector3f project(const Vector3f q, Vector2f& puv);
	void getUVAxes(Vector3f& u, Vector3f& v) {u = m_u; v = m_v;}

private:
	Vector3f m_p0;
	Vector3f m_norm;
	Vector3f m_u;
	Vector3f m_v;

};

#endif // PLANE_H
