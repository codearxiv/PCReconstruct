//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CLOUD_H
#define CLOUD_H

//#include <qopengl.h>
//#include <vector>
//#include <Eigen/Dense>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>

//#include "Cover_Tree.h"
//#include "CoverTreePoint.h"

template<Class T> Cover_tree;
template<Class T> CoverTreePoint;

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
	void create(CloudPtr cloud);
	void addPoint(const Vector3f &v, const Vector3f &n)
	{
		m_cloud.push_back(v);
		m_norms.push_back(n);
		CoverTreePoint<Vector3f> cp(v, m.cloud.size());
		CT.insert(cp);
	}

	void approxCloudNorms(int iters=10, int kNN=10);
	Vector3f approxNorm(
			const Vector3f &p,int iters=10, int kNN=10
			Vector3f &n);

	void pointKNN(
			const Vector3f &p, int k, 
			vector<CoverTreePoint<Vector3f>>& neighs)
	{
		neighs = kNearestNeighbors(p, k);			
	}
private:

	vector<Vector3f> m_cloud;
	vector<Vector3f> m_norms;
	vector<GLfloat> m_vertGL;
	vector<GLfloat> m_normGL;
	CoverTree<CoverTreePoint<Vector3f>> CT;
};


#endif // CLOUD_H
