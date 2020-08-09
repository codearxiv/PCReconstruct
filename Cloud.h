//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.

#ifndef CLOUD_H
#define CLOUD_H

#include "Cover_Tree.h"
#include "CoverTreePoint.h"
//#include "MessageLogger.h"

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <qopengl.h>
#include <QObject>
#include <QRecursiveMutex>
//#include <QMutexLocker>
#include <vector>
#include <functional>

class MessageLogger;
class BoundBox;

enum class SparseApprox { MatchingPursuit = 0, OrthogonalPursuit = 1 };

class Cloud : public QObject
{
	Q_OBJECT

	template<typename T> using vector = std::vector<T>;
	using Index = Eigen::Index;
	using Vector3f = Eigen::Vector3f;
	using Matrix3f = Eigen::Matrix3f;
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
	void fromRandomPlanePoints(
			Vector3f norm, size_t npoints,
			const std::function<float(float xu, float xv)> heightFun = nullptr);

	void buildSpatialIndex();
	void approxCloudNorms(int iters=100, size_t kNN=50);
	void reconstruct(
			int kSVDIters, size_t kNN, size_t nfreq, size_t natm, size_t latm,
			size_t maxNewPoints, BoundBox* BBox = nullptr,
			SparseApprox method = SparseApprox::OrthogonalPursuit);

	size_t addPoint(const Vector3f& v, const Vector3f &n,
				  bool threadSafe = false);
	void replacePoint(
			size_t idx, const Vector3f& v, const Vector3f &n,
			bool threadSafe = false);

	Vector3f approxNorm(
			const Vector3f& p, int iters,
			const vector<CoverTreePoint<Vector3f>>& neighs,
			vector<Vector3f>& vneighs);
	void pointKNN(
			const Vector3f& p, size_t kNN,
			vector<CoverTreePoint<Vector3f>>& neighs);

signals:
	void logMessage(const QString& text);
	void logProgress(const QString& msgPrefix,
					 size_t i, size_t n, int infreq, int& threshold);

private:
	void getNeighVects(const Vector3f& p,
					   const vector<CoverTreePoint<Vector3f>>& neighs,
					   vector<Vector3f>& vneighs);

	vector<Vector3f> m_cloud;
	vector<Vector3f> m_norms;
	vector<GLfloat> m_vertGL;
	vector<GLfloat> m_normGL;
	CoverTree<CoverTreePoint<Vector3f>> *m_CT;
	MessageLogger* m_msgLogger;
	QRecursiveMutex m_recMutex;
};


#endif // CLOUD_H
