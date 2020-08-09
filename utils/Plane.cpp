//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.

#include "Plane.h"

#include <Eigen/Dense>

using Eigen::Index;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;

//-----------------------------------------------------------

void Plane::set(const Vector3f p0, const Vector3f norm)
{

	const float RRT2 = 1.0f/sqrt(2.0f);
	m_p0 = p0;
	m_norm = norm;

	if( abs(norm(0)) < RRT2 ) {
		m_u(0) = 0.0f;
		m_u(1) = norm(2);
		m_u(2) = -norm(1);
	}
	else{
		m_u(0) = -norm(2);
		m_u(1) = 0.0f;
		m_u(2) = norm(0);
	}
	m_v = m_u.cross(norm);

	m_u.normalize();
	m_v.normalize();

}
//-----------------------------------------------------------

Eigen::Vector2f Plane::project_uv(const Vector3f q)
{
	Vector3f w = q - m_p0;
	Vector2f puv;
	puv(0) = w.dot(m_u);
	puv(1) = w.dot(m_v);
	return puv;
}

//-----------------------------------------------------------


Eigen::Vector3f Plane::project(const Vector3f q, Vector2f& puv)
{
	puv = project_uv(q);
	Vector3f p = puv(0)*m_u + puv(1)*m_v;
	return p;
}

//-----------------------------------------------------------


