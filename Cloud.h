//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.

#ifndef CLOUD_H
#define CLOUD_H

#include "Cover_Tree.h"
#include "CoverTreePoint.h"
#include "constants.h"
//#include "MessageLogger.h"

#include <Eigen/Core>
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

class Cloud : public QObject
{
	Q_OBJECT

	template<typename T> using vector = std::vector<T>;
	using Index = Eigen::Index;
	using Vector3f = Eigen::Vector3f;
	using Matrix3f = Eigen::Matrix3f;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	Cloud(MessageLogger* msgLogger = nullptr, QObject *parent = nullptr);
	~Cloud();
	const Vector3f point(size_t idx) const { return m_cloud[idx]; }
	const GLfloat* vertGLData();
	const GLfloat* normGLData(float scale);
	const GLfloat* debugGLData();

	size_t pointCount() const { return m_cloud.size(); }
	size_t pointCountOrig() const { return m_npointsOrig; }
	size_t debugCount() const { return m_debug.size(); }

	void setBoundBox(BoundBox *bBox);
	void clear();
	void fromPCL(CloudPtr cloud);
	void toPCL(CloudPtr& cloud);
	void fromRandomPlanePoints(
			Vector3f norm, size_t npoints,
			const std::function<float(float xu, float xv)> heightFun = nullptr);

	size_t addPoint(const Vector3f& v, const Vector3f &n,
				  bool threadSafe = false);
	void replacePoint(
			size_t idx, const Vector3f& v, const Vector3f &n,
			bool threadSafe = false);

	Vector3f approxNorm(
			const Vector3f& p, int iters,
			const vector<CoverTreePoint<Vector3f>>& neighs,
			vector<Vector3f>& vneighs, vector<Vector3f>& vwork);
	void pointKNN(
			const Vector3f& p, size_t kNN,
			vector<CoverTreePoint<Vector3f>>& neighs);

	void buildSpatialIndex();
	void approxCloudNorms(int iters=25, size_t kNN=25);
    void decimate(size_t nHoles, size_t kNN);
	void sparsify(float percent);
	void reconstruct(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
            size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method = SparseApprox::OrthogonalPursuit);

private:
	void getNeighVects(const Vector3f& p,
					   const vector<CoverTreePoint<Vector3f>>& neighs,
					   vector<Vector3f>& vneighs);

	vector<Vector3f> m_cloud;
	vector<Vector3f> m_norms;
	vector<std::pair<Vector3f,Vector3f>> m_debug;
	vector<GLfloat> m_vertGL;
	vector<GLfloat> m_normGL;
	vector<GLfloat> m_debugGL;
	CoverTree<CoverTreePoint<Vector3f>> *m_CT;
	BoundBox* m_bBox = nullptr;
	bool m_CTStale = true;
	MessageLogger* m_msgLogger;
	QRecursiveMutex m_recMutex;
	size_t m_npointsOrig = 0;

};


#endif // CLOUD_H
