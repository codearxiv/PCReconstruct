//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef CLOUD_H
#define CLOUD_H

//#include <qopengl.h>
//#include <vector>
//#include <Eigen/Dense>
//#include <QObject>
#include <QRecursiveMutex>
#include <QMutexLocker>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include "Cover_Tree.h"
#include "CoverTreePoint.h"
#include "MessageLogger.h"

class Cloud : public QObject
{
	Q_OBJECT

	template<typename T> using vector = std::vector<T>;
	using Index = Eigen::Index;
	using Vector3f = Eigen::Vector3f;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;
	
public:
	Cloud(MessageLogger* msgLogger = nullptr);
	~Cloud();
	const Vector3f point(size_t idx) const { return m_cloud[idx]; }
	const GLfloat *vertGLData();
	const GLfloat *normGLData(float scale);
	size_t pointCount() const { return m_cloud.size(); }
	void clear();
	void fromPCL(CloudPtr cloud);
	void toPCL(CloudPtr& cloud);
	void buildSpatialIndex();
	void approxCloudNorms(int iters=10, int kNN=10);

	void addPoint(const Vector3f& v, const Vector3f &n,
				  bool threadSafe = false);
	Vector3f approxNorm(
			const Vector3f& p, int iters=10, int kNN=10);
	void pointKNN(
			const Vector3f& p, int k,
			vector<CoverTreePoint<Vector3f>>& neighs);

signals:
	void logMessage(const QString& text);
	void logProgress(const QString& msgPrefix,
					 size_t i, size_t n, int infreq, int& threshold);

private:
	vector<Vector3f> m_cloud;
	vector<Vector3f> m_norms;
	vector<GLfloat> m_vertGL;
	vector<GLfloat> m_normGL;
	CoverTree<CoverTreePoint<Vector3f>> *m_CT;
	MessageLogger* m_msgLogger;
	QRecursiveMutex m_recMutex;

};


#endif // CLOUD_H
