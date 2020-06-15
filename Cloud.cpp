//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

//#include <qmath.h>

#include "Cloud.h"
#include "cloud_normal.h"
#include "Cover_Tree.h"
#include "CoverTreePoint.h"

template<typename T> using vector = std::vector<T>;
using Vector3f = Eigen::Vector3f;

//---------------------------------------------------------

Cloud::Cloud()
{
	m_cloud.reserve(2500);
	m_norms.reserve(2500);
	m_vertGL.reserve(2500 * 6);
}

//---------------------------------------------------------

void Cloud::create(CloudPtr cloud, int normIters, int normKNN)
{

	Vector3f n(0.0f, 0.0f, 1.0f);

	size_t npoints = cloud->points.size();

	float centx = 0.0f;
	float centy = 0.0f;
	float centz = 0.0f;

	for(size_t i = 0; i < npoints; ++i){
		centx = centx + cloud->points[i].x;
		centy = centy + cloud->points[i].y;
		centz = centz + cloud->points[i].z;
	}
	centx = centx/float(npoints);
	centy = centy/float(npoints);
	centz = centz/float(npoints);	

	for(size_t i = 0; i < cloud->points.size(); ++i){
		Vector3f v;
		v[0] = cloud->points[i].x - centx;
		v[1] = cloud->points[i].y - centy;
		v[2] = cloud->points[i].z - centz;
		addPoint(v, n);
	}

}

//---------------------------------------------------------

const GLfloat *Cloud::vertGLData() {
	m_vertGL.resize(6 * m_cloud.size());
	for(size_t i = 0, j = 0; i < m_cloud.size(); ++i){
		m_vertGL[j] = m_cloud[i][0];
		m_vertGL[j+1] = m_cloud[i][1];
		m_vertGL[j+2] = m_cloud[i][2];
		m_vertGL[j+3] = m_norms[i][0];
		m_vertGL[j+4] = m_norms[i][1];
		m_vertGL[j+5] = m_norms[i][2];
		j += 6;
	}

	return static_cast<const GLfloat*>(m_vertGL.data());
}

//---------------------------------------------------------

const GLfloat *Cloud::normGLData(float scale) {
	m_normGL.resize(12 * m_cloud.size());
	for(size_t i = 0, j = 0; i < m_cloud.size(); ++i){
		m_normGL[j] = m_cloud[i][0];
		m_normGL[j+1] = m_cloud[i][1];
		m_normGL[j+2] = m_cloud[i][2];
		m_normGL[j+3] = m_norms[i][0];
		m_normGL[j+4] = m_norms[i][1];
		m_normGL[j+5] = m_norms[i][2];
		m_normGL[j+6] = m_cloud[i][0] + scale*m_norms[i][0];
		m_normGL[j+7] = m_cloud[i][1] + scale*m_norms[i][1];
		m_normGL[j+8] = m_cloud[i][2] + scale*m_norms[i][2];
		m_normGL[j+9] = m_norms[i][0];
		m_normGL[j+10] = m_norms[i][1];
		m_normGL[j+11] = m_norms[i][2];
		j += 12;
	}

	return static_cast<const GLfloat*>(m_normGL.data());
}
//---------------------------------------------------------

void Cloud::addPoint(const Vector3f &v, const Vector3f &n)
{
	m_cloud.push_back(v);
	m_norms.push_back(n);
}
//---------------------------------------------------------

