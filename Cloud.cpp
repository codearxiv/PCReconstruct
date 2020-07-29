//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "Cloud.h"
#include "constants.h"
#include "cloud_normal.h"
#include "rotations.h"
#include "MatchingPursuit.h"
#include "OrthogonalPursuit.h"
#include "ksvd_dct2D.h"

//---------------------------------------------------------

Cloud::Cloud(MessageLogger* msgLogger)
	: QObject(), m_CT(nullptr), m_msgLogger(msgLogger)
{
	m_cloud.reserve(2500);
	m_norms.reserve(2500);
	m_vertGL.reserve(2500 * 6);

	connect(this, &Cloud::logMessage, m_msgLogger, &MessageLogger::logMessage);
	connect(this, &Cloud::logProgress, m_msgLogger, &MessageLogger::logProgress);
}

//---------------------------------------------------------

Cloud::~Cloud()
{
	delete m_CT;
}


//---------------------------------------------------------


const GLfloat *Cloud::vertGLData()
{
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

const GLfloat *Cloud::normGLData(float scale)
{
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

void Cloud::clear()
{
	QMutexLocker locker(&m_recMutex);

	m_cloud.clear();
	m_norms.clear();
	delete m_CT;
	m_CT = nullptr;
}
//---------------------------------------------------------

void Cloud::fromPCL(CloudPtr cloud)
{
	QMutexLocker locker(&m_recMutex);

	clear();

	Vector3f n(0.0f, 0.0f, 1.0f);

	//***
	size_t npoints = cloud->points.size()/25;

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

	for(size_t i = 0; i < npoints; ++i){
		Vector3f v;
		v[0] = cloud->points[i].x - centx;
		v[1] = cloud->points[i].y - centy;
		v[2] = cloud->points[i].z - centz;
		//addPoint(v, n);
		m_cloud.push_back(v);
		m_norms.push_back(n);
	}

	if(m_msgLogger != nullptr) {
		emit logMessage(QString::number(npoints) + " points loaded.");
	}

}

//---------------------------------------------------------

void Cloud::toPCL(CloudPtr& cloud)
{
	QMutexLocker locker(&m_recMutex);

	for(size_t i = 0; i < m_cloud.size(); ++i){
		pcl::PointXYZ v;
		v.x = m_cloud[i][0];
		v.y = m_cloud[i][1];
		v.z = m_cloud[i][2];
		cloud->push_back(v);
	}

}


//---------------------------------------------------------

void Cloud::addPoint(const Vector3f& v, const Vector3f& n, bool threadSafe)
{

	if ( threadSafe ) m_recMutex.lock();

	m_cloud.push_back(v);
	m_norms.push_back(n);

	if( m_CT != nullptr ) {
		CoverTreePoint<Vector3f> cp(v, m_cloud.size());
		m_CT->insert(cp);
	}

	if ( threadSafe ) m_recMutex.unlock();

}

//---------------------------------------------------------

void Cloud::buildSpatialIndex()
{
	QMutexLocker locker(&m_recMutex);

	if( m_CT != nullptr ) {
		delete m_CT;
		m_CT = nullptr;
	}

	m_CT = new CoverTree<CoverTreePoint<Vector3f>>();

	size_t npoints = m_cloud.size();
	int threshold = 0;

	for(size_t i = 0; i < npoints; ++i){
		// Log progress
		//***
		QCoreApplication::processEvents();
		if(m_msgLogger != nullptr) {
			emit logProgress(
						"Building cloud spatial index",
						i, npoints, 5, threshold);
		}

		Vector3f v;
		v[0] = m_cloud[i][0];
		v[1] = m_cloud[i][1];
		v[2] = m_cloud[i][2];
		CoverTreePoint<Vector3f> cp(v, i);
		m_CT->insert(cp);
	}
}
//---------------------------------------------------------
Eigen::Vector3f Cloud::approxNorm(
	const Vector3f& p, int iters, int kNN,
	vector<CoverTreePoint<Vector3f>>& neighs,
	vector<Vector3f>& vneighs)
{
	assert(m_CT != nullptr);
	CoverTreePoint<Vector3f> cp(p, 0);
	neighs = m_CT->kNearestNeighbors(cp, kNN);
	vneighs.reserve(neighs.size());
	vneighs.resize(0);
    typename vector<CoverTreePoint<Vector3f>>::const_iterator it;
    for(it=neighs.begin(); it!=neighs.end(); ++it){
		vneighs.push_back( it->getVec() );
	}
	return cloud_normal(p, vneighs, iters);
}
//---------------------------------------------------------

void Cloud::approxCloudNorms(int iters, int kNN)
{
	QMutexLocker locker(&m_recMutex);

	assert(m_CT != nullptr);

	size_t npoints = m_cloud.size();
	int threshold = 0;
	vector<CoverTreePoint<Vector3f>> neighs;
	vector<Vector3f> vneighs;

	for(size_t i = 0; i < npoints; ++i){
		// Log progress
		//QCoreApplication::processEvents();
		if(m_msgLogger != nullptr) {
			emit logProgress(
						"Building cloud normals",
						i, npoints, 5, threshold);
		}

		Vector3f p = m_cloud[i];
		m_norms[i] = approxNorm(p, iters, kNN, neighs, vneighs);
	}

}

//---------------------------------------------------------

void Cloud::pointKNN(
		const Vector3f& p, int k,
		vector<CoverTreePoint<Vector3f>>& neighs)
{
	assert(m_CT != nullptr);
	CoverTreePoint<Vector3f> cp(p, 0);
	neighs = m_CT->kNearestNeighbors(cp, k);
}

//---------------------------------------------------------

void Cloud::reconstruct(
		int kSVDIters, int kNN, int natm, int latm)
{
	QMutexLocker locker(&m_recMutex);
	assert(m_CT != nullptr);

	size_t npoints = m_cloud.size();
	vector<CoverTreePoint<Vector3f>> neighs;
	vector<Vector3f> vneighs;
	vector<Vector3f> vneighsXY;
	Vector3f zaxis(0.0f, 0.0f, 1.0f);
	Matrix3f M;

	int nfreq = std::ceil(sqrt(float(kNN)));
	MatrixXi gridXY(nfreq, nfreq);

	//--------
	auto fit_gridXY = [](
			const vector<Vector3f>& vs, float SD,
			float& sizeX, float& sizeY)
	{
		sizeX = sizeY = 0.0f;
		for(uint i=0; i<=vs.size(); ++i){
			Vector3f v = vs[i];
			sizeX += v(0);
			sizeY += v(1);
		}
		sizeX = SD*(sizeX/vs.size());
		sizeY = SD*(sizeY/vs.size());
	};
	//--------
	auto get_gridXY_occupancy = [=](
			const vector<Vector3f>& vs,
			float sizeX, float sizeY, MatrixXi& grid)
	{
		grid.setZero(nfreq, nfreq);
		for(uint i=0; i<=vs.size(); ++i){
			Vector3f v = vs[i];
			if( std::abs(v(0)) >= sizeX ) continue;
			if( std::abs(v(1)) >= sizeY ) continue;
			int cellX = nfreq*std::floor((v(0)+sizeX)/(2*sizeX));
			int cellY = nfreq*std::floor((v(1)+sizeY)/(2*sizeY));
			++grid(cellX, cellY);
		}
	};
	//--------


	for(size_t i = 0; i < npoints; ++i){
		Vector3f p = m_cloud[i];
		Vector3f n = approxNorm(p, 10, kNN, neighs, vneighs);
		m_norms[i] = n;

		vector_to_vector_rotation_matrix(n, zaxis, true, M);
		vneighsXY.resize(vneighs.size());
		for(size_t j = 0; j < vneighs.size(); ++j){
			vneighsXY[j] = M*vneighs[j];
		}

		float sizeX, sizeY;
		fit_gridXY(vneighsXY, 2.0f, sizeX, sizeY);
		if( sizeX <= float_tiny ) continue;
		if( sizeY <= float_tiny ) continue;
		get_gridXY_occupancy(vneighsXY, sizeX, sizeY, gridXY);
	}

}


//---------------------------------------------------------

