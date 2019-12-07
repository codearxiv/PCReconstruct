//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "cloud.h"
#include <qmath.h>

Cloud::Cloud()
    : m_count(0)
{

	m_data.resize(2500 * 6);

}


void Cloud::addPoint(const QVector3D &v)
{
	GLfloat *p = m_data.data() + m_count;
	*p++ = v.x();
	*p++ = v.y();
	*p++ = v.z();
	m_count += 3;
}

void Cloud::addPointNorm(const QVector3D &v, const QVector3D &n)
{
	GLfloat *p = m_data.data() + m_count;
	*p++ = v.x();
	*p++ = v.y();
	*p++ = v.z();
	*p++ = n.x();
	*p++ = n.y();
	*p++ = n.z();
	m_count += 6;
}


void Cloud::create(CloudPtr cloud)
{
	m_count = 0;
	m_data.resize(cloud->points.size() * 6);

	QVector3D n(0.0f, 0.0f, 1.0f);
	GLfloat centx = 0.0f;
	GLfloat centy = 0.0f;
	GLfloat centz = 0.0f;

	size_t npoints = cloud->points.size();

	for(size_t i = 0; i < npoints; ++i){
		centx = centx + cloud->points[i].x;
		centy = centy + cloud->points[i].y;
		centz = centz + cloud->points[i].z;
	}
	centx = centx/GLfloat(npoints);
	centy = centy/GLfloat(npoints);
	centz = centz/GLfloat(npoints);

	for(size_t i = 0; i < cloud->points.size(); ++i){
		GLfloat x = cloud->points[i].x - centx;
		GLfloat y = cloud->points[i].y - centy;
		GLfloat z = cloud->points[i].z - centz;
		addPointNorm(QVector3D(x,y,z), n);
	}

}



