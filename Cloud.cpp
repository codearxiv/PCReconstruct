//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included with this distribution.

#include "Cloud.h"
#include "constants.h"
#include "MessageLogger.h"
#include "BoundBox.h"
#include "cloud_normal.h"
#include "cosine_transform.h"
#include "rotations.h"
#include "Plane.h"
#include "MatchingPursuit.h"
#include "OrthogonalPursuit.h"
#include "ksvd_dct2D.h"
#include "Cover_Tree.h"
#include "CoverTreePoint.h"

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <qopengl.h>
#include <QCoreApplication>
#include <QRecursiveMutex>
#include <omp.h>
#include <queue>
#include <numeric>
#include <random>
#include <iterator>
#include <algorithm>
#include <functional>
#include <iostream>
#include <ctime>


using std::max;
using std::min;
using std::abs;
using std::floor;
using Eigen::Index;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::VectorXf;
using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using MapMtrxf = Eigen::Map<MatrixXf, ALIGNEDX>;
using MapMtrxi = Eigen::Map<MatrixXi, ALIGNEDX>;
template<typename T> using aligned = Eigen::aligned_allocator<T>;
template<typename T> using vector = std::vector<T>;
template<typename T> using queue = std::queue<T>;
using vectorfa = std::vector<float, aligned<float>>;
using vectoria = std::vector<int, aligned<int>>;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

//---------------------------------------------------------

Cloud::Cloud(MessageLogger* msgLogger, QObject *parent)
	: QObject(parent), m_CT(nullptr), m_msgLogger(msgLogger),
	  m_bBox(nullptr)

{
	m_cloud.reserve(2500);
	m_norms.reserve(2500);
	m_vertGL.reserve(2500 * 6);
}

//---------------------------------------------------------

Cloud::~Cloud()
{
	delete m_CT;
}


//---------------------------------------------------------


const GLfloat* Cloud::vertGLData()
{
	m_vertGL.resize(6 * m_cloud.size());
	for(size_t i=0, j=0; i < m_cloud.size(); ++i){
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

const GLfloat* Cloud::normGLData(float scale)
{
	m_normGL.resize(12 * m_cloud.size());
	for(size_t i=0, j=0; i < m_cloud.size(); ++i){
		m_normGL[j] = m_cloud[i][0] - scale*m_norms[i][0];
		m_normGL[j+1] = m_cloud[i][1] - scale*m_norms[i][1];
		m_normGL[j+2] = m_cloud[i][2] - scale*m_norms[i][2];
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

const GLfloat* Cloud::debugGLData()
{
	Vector3f norm(0.0f, 0.0f, 1.0f);

	m_debugGL.resize(12 * m_debug.size());

	for(size_t i=0, j=0; i < m_debug.size(); ++i){
		Vector3f p = m_debug[i].first;
		Vector3f q = m_debug[i].second;
		m_debugGL[j] = p(0);
		m_debugGL[j+1] = p(1);
		m_debugGL[j+2] = p(2);
		m_debugGL[j+3] = norm(0);
		m_debugGL[j+4] = norm(1);
		m_debugGL[j+5] = norm(2);
		m_debugGL[j+6] = q(0);
		m_debugGL[j+7] = q(1);
		m_debugGL[j+8] = q(2);
		m_debugGL[j+9] = norm(0);
		m_debugGL[j+10] = norm(1);
		m_debugGL[j+11] = norm(2);
		j += 12;
	}

	return static_cast<const GLfloat*>(m_debugGL.data());
}

//---------------------------------------------------------

void Cloud::clear()
{
	QMutexLocker locker(&m_recMutex);

	m_cloud.clear();
	m_norms.clear();
	m_cloud_bak.clear();
	m_norms_bak.clear();
	delete m_CT;
	m_CT = nullptr;
	m_CTStale = true;
	m_bBox = nullptr;
	m_npointsOrig = 0;
}


//---------------------------------------------------------

void Cloud::backup()
{
	QMutexLocker locker(&m_recMutex);

	m_cloud_bak = m_cloud;
	m_norms_bak = m_norms;
}
//---------------------------------------------------------

void Cloud::restore()
{
	QMutexLocker locker(&m_recMutex);

	std::swap(m_cloud, m_cloud_bak);
	std::swap(m_norms, m_norms_bak);
	m_npointsOrig = m_cloud.size();
	delete m_CT;
	m_CT = nullptr;
	m_CTStale = true;
}

//---------------------------------------------------------

void Cloud::setBoundBox(BoundBox *bBox) {
	QMutexLocker locker(&m_recMutex);

	m_bBox = bBox;
	bBox->setParentCloud(this);
	m_CTStale = true;
}

//---------------------------------------------------------

void Cloud::invalidateCT() {

	QMutexLocker locker(&m_recMutex);
	m_CTStale = true;
}

//---------------------------------------------------------

void Cloud::fromPCL(CloudPtr cloud)
{
	QMutexLocker locker(&m_recMutex);

	clear();

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

	for(size_t i = 0; i < npoints; ++i){
		Vector3f v;
		v(0) = cloud->points[i].x - centx;
		v(1) = cloud->points[i].y - centy;
		v(2) = cloud->points[i].z - centz;
		addPoint(v, n);
		//m_cloud.push_back(v);
		//m_norms.push_back(n);
	}

	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage(QString::number(npoints) + " points loaded.\n");
	}

	m_npointsOrig = npoints;


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

void Cloud::fromRandomPlanePoints(
		Vector3f norm, size_t npoints,
		const std::function<float(float xu, float xv)> heightFun)
{
	QMutexLocker locker(&m_recMutex);

	clear();

	Plane plane(Vector3f(0.0f,0.0f,0.0f), norm);
	Vector3f u, v;
	plane.getUVAxes(u,v);

	for(size_t i = 0; i < npoints; ++i){
		Vector2f uvScale = Vector2f::Random();
		Vector3f q = uvScale(0)*u + uvScale(1)*v;
		if( heightFun != nullptr ){
			float normScale = heightFun(uvScale(0), uvScale(1));
			q = q + normScale*norm;
		}
		addPoint(q, norm);
	}

	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage(QString::number(npoints) + " points created.\n");
	}

	m_npointsOrig = npoints;

}

//---------------------------------------------------------

size_t Cloud::addPoint(const Vector3f& v, const Vector3f& n, bool threadSafe)
{

	if ( threadSafe ) m_recMutex.lock();

	size_t idx = m_cloud.size();
	m_cloud.push_back(v);
	m_norms.push_back(n);	

	if( m_CT != nullptr ) {
		CoverTreePoint<Vector3f> cp(v, idx);
		m_CT->insert(cp);
		++m_npointsCT;
	}

	if ( threadSafe ) m_recMutex.unlock();

	return idx;
}

//---------------------------------------------------------

void Cloud::replacePoint(
		size_t idx, const Vector3f& v, const Vector3f& n, bool threadSafe)
{
	if ( threadSafe ) m_recMutex.lock();
	assert(idx < m_cloud.size());

	m_cloud[idx] = v;
	m_norms[idx] = n;

	if( m_CT != nullptr ) {
		CoverTreePoint<Vector3f> cp(v, idx);
		m_CT->remove(cp);
		m_CT->insert(cp);
	}

	if ( threadSafe ) m_recMutex.unlock();

}

//---------------------------------------------------------
void Cloud::pointKNN(
		const Vector3f& p, size_t kNN,
		vector<CoverTreePoint<Vector3f>>& neighs) const
{
	assert(m_CT != nullptr);
	CoverTreePoint<Vector3f> cp(p, 0);
	neighs = m_CT->kNearestNeighbors(cp, kNN);
}

//---------------------------------------------------------
Eigen::Vector3f Cloud::approxNorm(
	const Vector3f& p, int iters,
	const vector<CoverTreePoint<Vector3f>>& neighs,
	vector<Vector3f>& vneighs, vector<Vector3f>& vwork) const
{
	static const Vector3f origin(0.0f,0.0f,0.0f);
	assert(m_CT != nullptr);
	getNeighVects(p, neighs, vneighs);
	return cloud_normal(origin, vneighs, iters, vwork);
}

//---------------------------------------------------------
void Cloud::getNeighVects(
	const Vector3f& p,
	const vector<CoverTreePoint<Vector3f>>& neighs,
	vector<Vector3f>& vneighs) const
{
	vneighs.reserve(neighs.size());
	vneighs.resize(0);
	typename vector<CoverTreePoint<Vector3f>>::const_iterator it;
	for(it=neighs.begin(); it!=neighs.end(); ++it){
		size_t idx = it->getId();
		Vector3f v = m_cloud[idx] - p;
		vneighs.push_back(v);
	}
}

//---------------------------------------------------------

void Cloud::buildSpatialIndex(bool useBBox)
{
	QMutexLocker locker(&m_recMutex);

	if( m_CT != nullptr ) {
		delete m_CT;
		m_CT = nullptr;
	}

	m_CT = new CoverTree<CoverTreePoint<Vector3f>>();

	bool useBBox2 = (m_bBox != nullptr) && useBBox;
	size_t npoints = m_cloud.size();
	size_t threshold = 0, lastPos = 0;

	m_npointsCT = 0;
	for(size_t i = 0; i < npoints; ++i){
		// Log progress
		if(m_msgLogger != nullptr) {
//			QCoreApplication::processEvents();
			m_msgLogger->logProgress(
						"Building cloud spatial index",
						i+1, npoints, 5, threshold, lastPos);
		}
		Vector3f p = m_cloud[i];
		if( useBBox2 ) if( !m_bBox->pointInBBox(p) ) continue;
		CoverTreePoint<Vector3f> cp(p, i);
		m_CT->insert(cp);
		++m_npointsCT;
	}

	m_CTStale = false;
}

//---------------------------------------------------------

void Cloud::approxCloudNorms(int iters, size_t kNN)
{
	QMutexLocker locker(&m_recMutex);

	bool useBBox = (m_bBox != nullptr);
	Index npoints = m_cloud.size();
	size_t threshold = 0, lastPos = 0, progress = 0;

	if( m_CTStale ) buildSpatialIndex();

//	std::clock_t c_start = std::clock();//***

#pragma omp parallel default(shared)
{
	vector<CoverTreePoint<Vector3f>> neighs;
	vector<Vector3f> vneighs;
	vector<Vector3f> vwork;

#pragma omp for schedule(dynamic)
	for(Index i=0; i < npoints; ++i){
		Vector3f p = m_cloud[i];
		if( useBBox ) if( !m_bBox->pointInBBox(p) ) continue;
		CoverTreePoint<Vector3f> cp(p, 0);
		neighs = m_CT->kNearestNeighbors(cp, kNN);
		m_norms[i] = approxNorm(p, iters, neighs, vneighs, vwork);

		// Log progress
		if(m_msgLogger != nullptr) {
#pragma omp atomic
			++progress;
			m_msgLogger->logProgress(
						"Building cloud normals",
						progress, npoints, 5, threshold, lastPos);
		}

	}

} //Parallel

//	std::clock_t c_end = std::clock();//***
//	double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
//	std::cout << "\nCPU time used: " << time_elapsed_ms << " ms\n" << std::endl;


	m_msgLogger->logMessage("Building cloud normals: 100%...", false);


}


//---------------------------------------------------------

void Cloud::decimate(size_t nHoles, size_t kNN)
{
    QMutexLocker locker(&m_recMutex);

	size_t npoints = m_cloud.size();
	if(npoints == 0 || nHoles <= 0 || kNN <= 0) return;

	vector<CoverTreePoint<Vector3f>> neighs;
	vector<bool> deletedPoint(npoints, false);
	size_t threshold = 0, lastPos = 0;
	size_t ndeleted = 0;	

	bool useBBox = (m_bBox != nullptr);
	if( m_CTStale && kNN > 1 ) buildSpatialIndex();
	QString actionStr = kNN > 1 ? "Decimating" : "Sparsifying";

	//std::random_device rd;
	//std::mt19937 mt(rd());
    vector<size_t> shuffled;
    shuffled.reserve(npoints);

    for(size_t idx=0; idx < npoints; ++idx){
        if( useBBox ){
            if( !m_bBox->pointInBBox(m_cloud[idx]) ) continue;
        }
        shuffled.push_back(idx);
    }
	std::random_shuffle(shuffled.begin(), shuffled.end());

    size_t nSubset = min(nHoles,shuffled.size());

    for(size_t i=0; i < nSubset; ++i){
        // Log progress
		if(m_msgLogger != nullptr) {
			m_msgLogger->logProgress(actionStr, i+1, nSubset, 10,
							 threshold, lastPos);
		}
		size_t randIdx = shuffled[i];

		if( kNN > 1 ){
            pointKNN(m_cloud[randIdx], kNN, neighs);
			typename vector<CoverTreePoint<Vector3f>>::const_iterator it;
			for(it=neighs.begin(); it!=neighs.end(); ++it){
				size_t idx = it->getId();
				if( deletedPoint[idx] ) continue;
				//m_CT->remove(*it);
				deletedPoint[idx] = true;
				++ndeleted;
			}
		}
		else{
			deletedPoint[randIdx] = true;
			++ndeleted;
		}

		if(ndeleted >= npoints) break;

    }

	size_t nremaining = max(size_t(0), npoints-ndeleted);
	if( nremaining <= 0 ){
		m_cloud.clear();
		m_norms.clear();
	}
	else{
		vector<Vector3f> m_cloud2(nremaining);
		vector<Vector3f> m_norms2(nremaining);

		for(size_t idx=0, idx2=0; idx < npoints; ++idx){
			if( deletedPoint[idx] ) continue;
			m_cloud2[idx2] = m_cloud[idx];
			m_norms2[idx2] = m_norms[idx];
			++idx2;
		}
		m_cloud = std::move(m_cloud2);
		m_norms = std::move(m_norms2);
	}


	m_CTStale = true;
	m_npointsOrig = npoints;


	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage(
					QString::number(ndeleted) + " points deleted, " +
					QString::number(nremaining) + " points remaining.\n");
	}

}

//---------------------------------------------------------

void Cloud::sparsify(float percent)
{
	QMutexLocker locker(&m_recMutex);

    bool useBBox = (m_bBox != nullptr);

    size_t nPointsUse = 0;
    if( useBBox ){
        for(size_t idx=0; idx < m_cloud.size(); ++idx){
            if( !m_bBox->pointInBBox(m_cloud[idx]) ) continue;
            ++nPointsUse;
        }
    }
    else{
        nPointsUse = m_cloud.size();
    }

    size_t nKeep = size_t(ceil( percent*nPointsUse/100.0 ));
    size_t nRemove = max(size_t(0), nPointsUse - nKeep);
	decimate(nRemove, 1);

}

//---------------------------------------------------------

void Cloud::reconstruct(
        int kSVDIters, size_t kNN, size_t nfreq, float densify,
		size_t natm, size_t latm, size_t maxNewPoints, bool looseBBox,
        SparseApprox method)
{
	QMutexLocker locker(&m_recMutex);
	//assert(m_CT != nullptr);
	assert(kNN >= 1 && nfreq >= 1 && natm >= 1 && latm >= 1 && latm <= natm);

	m_npointsOrig = m_cloud.size();
	if(m_npointsOrig == 0) return;
	size_t nfreqsq = nfreq*nfreq;
	//size_t maxGridDim = max(1, int(floor(sqrt(float(kNN)))));
	bool useBBox = (m_bBox != nullptr);
	//--------
	struct gridLocation {
		size_t idx = -1;
		bool inCloud = true;
		bool inGrid = false;
		int cellX;
		int cellY;
		float u;
		float v;
		float w;
		size_t sigIdx = -1;
	};

	//--------
	auto fit_gridXY = [=](
			const vector<Vector3f>& vneighsXY, float SD,
			float& sizeX, float& sizeY,
			size_t& gridDimX, size_t& gridDimY)
	{
		sizeX = sizeY = 0.0f;
		size_t n = vneighsXY.size();
		for(size_t i=0; i<n; ++i){
			Vector3f v = vneighsXY[i];
			sizeX += v(0)*v(0);
			sizeY += v(1)*v(1);
		}
		sizeX = SD*sqrt(sizeX/n);
		sizeY = SD*sqrt(sizeY/n);
		if(min(sizeX,sizeY) <= 1e-3*max(sizeX,sizeY)) return false;
		if(sizeX <= float_tiny || sizeY <= float_tiny) return false;
		size_t numInGrid = 0;
		for(size_t i=0; i<n; ++i){
			Vector3f v = vneighsXY[i];
			if(abs(v(0)) > sizeX || abs(v(1)) > sizeY) continue;
			++numInGrid;
		}
		if( numInGrid <= 1 ) return false;
		gridDimX = gridDimY =
				max(1, int(floor(0.8f*sqrt(densify*numInGrid))));

		return true;
	};



	//--------
	auto get_gridXY_occupancy = [=](
			const vector<CoverTreePoint<Vector3f>>& neighs,
			const vector<Vector3f>& vneighsXY,
			float sizeX, float sizeY,
			size_t gridDimX, size_t gridDimY,
			MapMtrxi& gridXY, vector<gridLocation>& gridLoc)
	{
		gridXY.setZero();
		gridLoc.resize(neighs.size());
		for(size_t i=0; i<neighs.size(); ++i){
			gridLocation& loc = gridLoc[i];
			loc.idx = neighs[i].getId();
			loc.inCloud = true;
			loc.inGrid = false;
			const Vector3f& q = vneighsXY[i];
			if( abs(q(0)) > sizeX ) continue;
			if( abs(q(1)) > sizeY ) continue;
			loc.u = (q(0)+sizeX)/(2*sizeX);
			loc.v = (q(1)+sizeY)/(2*sizeY);
			loc.w = q(2);
			loc.cellX = int(floor(gridDimX*loc.u));
			loc.cellY = int(floor(gridDimY*loc.v));
			++gridXY(loc.cellX, loc.cellY);
			loc.inGrid = true;
		}

	};

	//--------

	auto setup_grid = [=](
			const Vector3f& norm,
			const vector<CoverTreePoint<Vector3f>>& neighs,
			const vector<Vector3f>& vneighs,
			vector<Vector3f>& vneighsXY, Matrix3f& rotXY,
			MapMtrxi& gridXY, float& sizeX, float& sizeY,
			size_t& gridDimX, size_t& gridDimY,
			vector<gridLocation>& gridLoc, vectoria& iworkGrid)
	{
		gridDimX = gridDimY = 0;
		static const Vector3f zaxis(0.0f, 0.0f, 1.0f);
		vector_to_vector_rotation_matrix(norm, zaxis, true, true, rotXY);
		vneighsXY.resize(vneighs.size());
		for(size_t j=0; j<vneighs.size(); ++j){
			vneighsXY[j].noalias() = rotXY*vneighs[j];
		}

		bool ok = fit_gridXY(
					vneighsXY, 1.5f, sizeX, sizeY, gridDimX, gridDimY);
		if( !ok ) return false;
		size_t nwork = gridDimX*gridDimY;
		if(iworkGrid.size() < nwork) iworkGrid.resize(nwork);
		new (&gridXY) MapMtrxi(&iworkGrid[0], gridDimX, gridDimY);
		get_gridXY_occupancy(
					neighs, vneighsXY, sizeX, sizeY,
					gridDimX, gridDimY, gridXY, gridLoc);

		return true;
	};

	//--------

	auto setup_grid_signal = [=](
			vector<gridLocation>& gridLoc,
			VectorXf& Usig, VectorXf& Vsig, VectorXf& Wsig)
	{
		size_t numInGrid = 0;
		for(size_t j=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inGrid ) continue;
			++numInGrid;
		}

		Usig.resize(numInGrid);
		Vsig.resize(numInGrid);
		Wsig.resize(numInGrid);
		for(size_t j=0, k=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inGrid ) continue;
			Usig(k) = gridLoc[j].u;
			Vsig(k) = gridLoc[j].v;
			Wsig(k) = gridLoc[j].w;
			gridLoc[j].sigIdx = k;
			++k;
		}
	};

	//--------
	auto uvw_to_xyz = [=](
			float cu, float cv, float cw,
			const Vector3f& p, const Matrix3f& rotXYInv,
			float sizeX, float sizeY)
	{
		Vector3f qXY, q;
		qXY(0) = (2*cu - 1.0f)*sizeX;
		qXY(1) = (2*cv - 1.0f)*sizeY;
		qXY(2) = cw;
		q.noalias() = rotXYInv*qXY;
		q += p;
		return q;
	};

	//--------
	auto get_num_empty_cells = [=](
			const Vector3f& p, const Matrix3f& rotXY,
			const MapMtrxi& gridXY, float sizeX, float sizeY,
			size_t gridDimX, size_t gridDimY
			)
	{

		bool ignoreBBox = true;
		if( useBBox ){
			ignoreBBox = !m_bBox->ballInBBox(p, max(sizeX,sizeY));
		}

		size_t numEmpty = 0;
		if( ignoreBBox ){
			for(size_t j=0; j<gridDimY; ++j){
				for(size_t k=0; k<gridDimX; ++k){
					if(gridXY(k,j) > 0) continue;
					++numEmpty;
				}
			}
		}
		else{
			Matrix3f rotXYInv = rotXY.transpose();
			for(size_t j=0; j<gridDimY; ++j){
				float cv = (j/gridDimY);
				for(size_t k=0; k<gridDimX; ++k){
					if(gridXY(k,j) > 0) continue;
					float cu = (k/gridDimX);
					Vector3f q = uvw_to_xyz(
								cu, cv, 0.0f, p, rotXYInv, sizeX, sizeY);
					if( !m_bBox->pointInBBox(q) ) continue;
					++numEmpty;
				}
			}
		}

		return numEmpty;
	};

	//--------

	vector<CoverTreePoint<Vector3f>> neighsNrm;
	vector<Vector3f> vneighsNrm;
	vector<Vector3f> vwork;

	auto update_normal = [&](
			size_t idx, const vector<CoverTreePoint<Vector3f>>& neighs)
	{
		neighsNrm.clear();
		size_t kNNNrm = min(size_t(50), neighs.size());
		for(size_t j=0;j<kNNNrm;++j){ neighsNrm.push_back(neighs[j]); }
		m_norms[idx] =
				approxNorm(m_cloud[idx], 25, neighsNrm, vneighsNrm, vwork);
		return m_norms[idx];
	};

	//--------

	vector<CoverTreePoint<Vector3f>> neighs;
	vector<Vector3f> vneighs;
	vector<Vector3f> vneighsXY;
	MapMtrxi gridXY(nullptr, 0, 0);
	vectoria iworkGrid(kNN);
	vector<gridLocation> gridLoc;

	VectorXf Usig, Vsig, Wsig;
	vector<VectorXf> Us;
	vector<VectorXf> Vs;
	vector<VectorXf> Ws;

	queue<size_t> qpoints;
	std::priority_queue<std::pair<float, size_t>> pqpoints;
	size_t threshold = 0, lastPos = 0;

	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage("** Point cloud reconstruction **");
	}


	if( m_CTStale || (looseBBox && m_npointsCT < m_cloud.size()) ){
		buildSpatialIndex(!looseBBox);
	}

	for(size_t idx = 0; idx < m_npointsOrig; ++idx){
		// Log progress
		if(m_msgLogger != nullptr) {
			m_msgLogger->logProgress(
						"Setting up signals",
						idx+1, m_npointsOrig, 5, threshold, lastPos);
		}

		Vector3f p = m_cloud[idx];
		if( useBBox ) if( !m_bBox->pointInBBox(p) ) continue;
		pointKNN(p, kNN, neighs);
		getNeighVects(p, neighs, vneighs);
		Vector3f norm = update_normal(idx, neighs);

		float sizeX, sizeY;
		size_t gridDimX, gridDimY;
		Matrix3f rotXY;
		bool success = setup_grid(
					norm, neighs, vneighs, vneighsXY, rotXY, gridXY,
					sizeX, sizeY, gridDimX, gridDimY, gridLoc, iworkGrid);

		if( !success ) continue;

		setup_grid_signal(gridLoc, Usig, Vsig, Wsig);
		Us.push_back(Usig);
		Vs.push_back(Vsig);
		Ws.push_back(Wsig);

		size_t numEmpty = get_num_empty_cells(
					p, rotXY, gridXY, sizeX, sizeY, gridDimX, gridDimY);

//		if( numEmpty > 0 ) qpoints.push(idx);
		if( numEmpty > 0 ){
			float weight = float(numEmpty)/(gridDimX*gridDimY);
			pqpoints.push(std::make_pair(weight,idx));
		}

	}


	while( !pqpoints.empty() ){
		size_t idx = pqpoints.top().second;
		pqpoints.pop();
		qpoints.push(idx);
	}

	//--------


	MatrixXf D = MatrixXf::Random(nfreqsq, natm);
	D.colwise().normalize();
	MatrixXf C = MatrixXf::Random(natm, m_npointsOrig);

	std::function<void(
				const Eigen::VectorXf&,
				const Eigen::MatrixXf&,
				Eigen::Index,
				Eigen::VectorXf&,
				Eigen::VectorXf&
				)> sparseFunct;

	MatchingPursuit mp;
	OrthogonalPursuit op;

	switch(method){
	case SparseApprox::OrthogonalPursuit :
		sparseFunct = op;
		break;
	case SparseApprox::MatchingPursuit :
		sparseFunct = mp;
		break;
	}

	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage("Training dictionary...");
	}

//	std::clock_t c_start = std::clock();//***

	ksvd_dct2D(true, Ws, Us, Vs, nfreq, latm, kSVDIters, 0.0,
			   sparseFunct, D, C, m_msgLogger);

//	std::clock_t c_end = std::clock();//***
//	double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
//	std::cout << "\nCPU time used: " << time_elapsed_ms << " ms\n" << std::endl;


	//--------

	MapMtrxf T(nullptr, kNN, nfreqsq);
	MapMtrxf TD(nullptr, kNN, natm);
	MapMtrxf TDNrm(nullptr, kNN, natm);
	MapMtrxf T0b(nullptr, kNN, nfreqsq);
	MapMtrxf T0D(nullptr, 2*kNN, natm);
	VectorXf NrmInv(natm);
	VectorXf Csig(natm);
	VectorXf CsigNrm(natm);
	VectorXf R(nfreqsq);
	VectorXf U0sig, V0sig, W0sig;
	VectorXf U0bsig, V0bsig;
	vector<gridLocation> gridLoc0;

	size_t paddedA[3] = {
		align_padded(kNN*nfreqsq),
		align_padded(kNN*natm),
		align_padded(kNN*natm)
	};
	size_t nworkDctA = paddedA[0] + paddedA[1] + paddedA[2];
	vectorfa dworkDctA(nworkDctA);
	vectorfa dworkDctA0;
	vectorfa dworkDctB;

	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage("Reconstructing point cloud...");
		m_msgLogger->logMessage(
					QString::number(qpoints.size()) + " patches in queue...");
	}

	size_t nprocessed = 0;
	size_t nNewPoints = 0;
	while( !qpoints.empty() ){
		// Log progress
		if(m_msgLogger != nullptr) {
			if( nprocessed%1000 == 0 ){
				m_msgLogger->logMessage(
							QString::number(nprocessed) + " patches processed...");
			}
			++nprocessed;
		}

		size_t idx = qpoints.front();
		qpoints.pop();

		Vector3f p = m_cloud[idx];
		pointKNN(p, kNN, neighs);
		getNeighVects(p, neighs, vneighs);

		Vector3f norm;
		if( idx < m_npointsOrig ){
			norm = m_norms[idx];
		}
		else{
			norm = update_normal(idx, neighs);
		}

		float sizeX, sizeY;
		size_t gridDimX, gridDimY;
		Matrix3f rotXY;
		bool success = setup_grid(
					norm, neighs, vneighs, vneighsXY, rotXY, gridXY,
					sizeX, sizeY, gridDimX, gridDimY, gridLoc, iworkGrid);

		if( !success ) continue;

		setup_grid_signal(gridLoc, Usig, Vsig, Wsig);

		size_t nsmpl = Wsig.size();
		size_t offset = 0;
		new (&T) MapMtrxf(&dworkDctA[offset], nsmpl, nfreqsq);
		offset += paddedA[0];
		new (&TD) MapMtrxf(&dworkDctA[offset], nsmpl, natm);
		offset += paddedA[1];
		new (&TDNrm) MapMtrxf(&dworkDctA[offset], nsmpl, natm);

		cosine_transform(Usig, Vsig, nfreq, dworkDctB, T);
		TD.noalias() = T*D;
		TDNrm = TD;
		column_normalize(TDNrm, NrmInv);

		sparseFunct(Wsig, TDNrm, latm, CsigNrm, R);
		Csig = CsigNrm.cwiseProduct(NrmInv);

		gridLoc0.clear();

		for(size_t j=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inGrid ) continue;
			if( gridLoc[j].idx < m_npointsOrig ) continue;
			gridLoc0.push_back(gridLoc[j]);
		}
		size_t nsmpl0a = gridLoc0.size();

		size_t npoints = m_cloud.size();
		for(size_t j=0; j<gridDimY; ++j){
			for(size_t k=0; k<gridDimX; ++k){
				if(gridXY(k,j) > 0) continue;
				gridLocation loc;
				loc.idx = npoints;
				loc.inCloud = false;
				loc.inGrid = true;
				loc.u = (k+0.5f)/gridDimX;
				loc.v = (j+0.5f)/gridDimY;
				loc.w = 0.0f;
				loc.cellX = k;
				loc.cellY = j;
				gridLoc0.push_back(loc);
				++npoints;
			}
		}
		size_t nsmpl0 = gridLoc0.size();
		size_t nsmpl0b = nsmpl0 - nsmpl0a;

		size_t paddedA0[2] = {
			align_padded(nsmpl0*natm),
			align_padded(nsmpl0b*nfreqsq)
		};
		size_t nworkDctA0 = paddedA0[0] + paddedA0[1];
		if(dworkDctA0.size() < nworkDctA0) dworkDctA0.resize(nworkDctA0);

		offset = 0;
		new (&T0D) MapMtrxf(&dworkDctA0[offset], nsmpl0, natm);
		offset += paddedA0[0];
		new (&T0b) MapMtrxf(&dworkDctA0[offset], nsmpl0b, nfreqsq);

		setup_grid_signal(gridLoc0, U0sig, V0sig, W0sig);

		for(size_t j=0, k=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inGrid ) continue;
			if( gridLoc[j].idx < m_npointsOrig ) continue;
			T0D.row(k) = TD.row(gridLoc[j].sigIdx);
			++k;
		}

		U0bsig.resize(nsmpl0b);
		V0bsig.resize(nsmpl0b);
		U0bsig.segment(0,nsmpl0b) = U0sig.segment(nsmpl0a,nsmpl0b);
		V0bsig.segment(0,nsmpl0b) = V0sig.segment(nsmpl0a,nsmpl0b);
		cosine_transform(U0bsig, V0bsig, nfreq, dworkDctB, T0b);
		T0D.block(nsmpl0a,0,nsmpl0b,natm).noalias() = T0b*D;

		W0sig.noalias() = T0D*Csig;

		Matrix3f rotXYInv = rotXY.transpose();

		for(size_t j=0; j<gridLoc0.size(); ++j){
			gridLocation& loc = gridLoc0[j];
			if( !loc.inCloud ){
				if( nNewPoints >= maxNewPoints ) continue;
			}
			Vector3f q = uvw_to_xyz(
						loc.u, loc.v, W0sig(j), p, rotXYInv, sizeX, sizeY);
			if( useBBox ) if( !m_bBox->pointInBBox(q) ) continue;
			if( loc.inCloud ){
				// Modifying cover tree point coordinates by point
				// removal and reinsertion is extremely slow.
				// We ignore the cover tree and change only the
				// cloud coordinates, treating the cover tree as
				// an approximation. This shouldn't cause much harm
				// since the change in coordinates should be modest.
				//replacePoint(loc.idx, q, m_norms[loc.idx]);
				m_cloud[loc.idx] = q;
			}
			else{
				loc.idx = addPoint(q, norm);
				++nNewPoints;
				qpoints.push(loc.idx);
			}
		}

		if( nNewPoints >= maxNewPoints ) break;

	}


	m_CTStale = true;

	if(m_msgLogger != nullptr) {
		m_msgLogger->logMessage(
					QString::number(nNewPoints) + " points added to cloud, " +
					QString::number(m_cloud.size()) + " points total.\n");
	}


}


//---------------------------------------------------------

