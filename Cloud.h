//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CLOUD_H
#define CLOUD_H

#include <qopengl.h>
#include <QVector>
#include <QVector3D>
#include <pcl/point_types.h>

class Cloud
{
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	Cloud();
    const GLfloat *constData() const { return m_data.constData(); }
    int count() const { return m_count; }
    int vertexCount() const { return m_count / 6; }

	void create(CloudPtr cloud);

private:
	void addPoint(const QVector3D &v);
	void addPointNorm(const QVector3D &v, const QVector3D &n);

	void quad(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2, GLfloat x3, GLfloat y3, GLfloat x4, GLfloat y4);
	void extrude(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2);
	void add(const QVector3D &v, const QVector3D &n);


    QVector<GLfloat> m_data;
    int m_count;
};

#endif // CLOUD_H
