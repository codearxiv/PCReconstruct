//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CLOUD_H
#define CLOUD_H

//#include <qopengl.h>
//#include <vector>
//#include <Eigen/Dense>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>

class Cloud
{
	template<typename T> using vector = std::vector<T>;
	using Index = Eigen::Index;
	using Vector3f = Eigen::Vector3f;

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	Cloud();
	const Vector3f point(size_t idx) const { return m_cloud[idx]; }
	const GLfloat *vertGLData();
	const GLfloat *normGLData(float scale);
	size_t pointCount() const { return m_cloud.size(); }
	void create(CloudPtr cloud, int normIters=10, int normKNN=10);
	void addPoint(const Vector3f &v, const Vector3f &n);
	void refreshNorms(int normIters=10, int normKNN=10);
	void pointKNN(
			int k, const Vector3f &v, vector<Vector3f>& neighs);
	void pointNeighboursWithin(
			double radius, const Vector3f &v, vector<Vector3f>& neighs);
private:

	vector<Vector3f> m_cloud;
	vector<Vector3f> m_norms;
	vector<GLfloat> m_vertGL;
	vector<GLfloat> m_normGL;
};


#endif // CLOUD_H
