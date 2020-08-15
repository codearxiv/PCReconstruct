//-----------------------------------------------------------
//  Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//  See LICENSE included.

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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <qopengl.h>
#include <QCoreApplication>
#include <QRecursiveMutex>
#include <queue>
#include <numeric>
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
	size_t npoints = cloud->points.size()/400;

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
		emit logMessage(QString::number(npoints) + " points created.");
	}

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
		vector<CoverTreePoint<Vector3f>>& neighs)
{
	assert(m_CT != nullptr);
	CoverTreePoint<Vector3f> cp(p, 0);
	neighs = m_CT->kNearestNeighbors(cp, kNN);
}

//---------------------------------------------------------
Eigen::Vector3f Cloud::approxNorm(
	const Vector3f& p, int iters,
	const vector<CoverTreePoint<Vector3f>>& neighs,
	vector<Vector3f>& vneighs, vector<Vector3f>& vwork)
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
	vector<Vector3f>& vneighs)
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

void Cloud::buildSpatialIndex()
{
	QMutexLocker locker(&m_recMutex);

	if( m_CT != nullptr ) {
		delete m_CT;
		m_CT = nullptr;
	}

	m_CT = new CoverTree<CoverTreePoint<Vector3f>>();

	size_t npoints = m_cloud.size();
	size_t threshold = 0, lastPos = 0;

	for(size_t i = 0; i < npoints; ++i){
		// Log progress
		if(m_msgLogger != nullptr) {
			//***
			QCoreApplication::processEvents();
			emit logProgress(
						"Building cloud spatial index",
						i+1, npoints, 5, threshold, lastPos);
		}

		CoverTreePoint<Vector3f> cp(m_cloud[i], i);
		m_CT->insert(cp);
	}

	m_CTStale = false;
}

//---------------------------------------------------------

void Cloud::approxCloudNorms(int iters, size_t kNN)
{
	QMutexLocker locker(&m_recMutex);

	size_t npoints = m_cloud.size();
	size_t threshold = 0, lastPos = 0;
	vector<CoverTreePoint<Vector3f>> neighs;
	vector<Vector3f> vneighs;
	vector<Vector3f> vwork;

	if( m_CTStale ) buildSpatialIndex();

	for(size_t i=0; i < npoints; ++i){
		// Log progress
		if(m_msgLogger != nullptr) {
			//***
			QCoreApplication::processEvents();
			emit logProgress(
						"Building cloud normals",
						i+1, npoints, 5, threshold, lastPos);
		}

		Vector3f p = m_cloud[i];
		CoverTreePoint<Vector3f> cp(p, 0);
		neighs = m_CT->kNearestNeighbors(cp, kNN);
		m_norms[i] = approxNorm(p, iters, neighs, vneighs, vwork);
//***
//		if( i == 0 ){
//			for(size_t j=0; j < neighs.size(); ++j){
//				Vector3f q = m_cloud[neighs[j].getId()];
//				m_debug.push_back(std::make_pair(p,q));
//			}
//		}
	}

}


//---------------------------------------------------------

void Cloud::decimate(size_t nHoles, size_t kNN)
{
    QMutexLocker locker(&m_recMutex);
	assert(kNN >= 1);

	size_t npoints = m_cloud.size();
	vector<CoverTreePoint<Vector3f>> neighs;
	vector<bool> deletedPoint(npoints, false);
	size_t threshold = 0, lastPos = 0;
	size_t ndeleted = 0;

	if( m_CTStale ) buildSpatialIndex();


	for(size_t i=0; i < nHoles; ++i){
        // Log progress
        if(m_msgLogger != nullptr) {
            //***
            QCoreApplication::processEvents();
            emit logProgress("Decimating", i+1, nHoles, 10,
							 threshold, lastPos);
        }
		size_t randIdx = std::rand()%npoints;
		if( deletedPoint[randIdx] ) continue;
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

	vector<Vector3f> m_cloud2(npoints-ndeleted);
	vector<Vector3f> m_norms2(npoints-ndeleted);

	for(size_t idx=0, idx2=0; idx < npoints; ++idx){
		if( deletedPoint[idx] ) continue;
		m_cloud2[idx2] = m_cloud[idx];
		m_norms2[idx2] = m_norms[idx];
		++idx2;
	}
//	m_cloud = std::move(m_cloud2);
//	m_norms = std::move(m_norms2);
	m_cloud = m_cloud2;
	m_norms = m_norms2;

	m_CTStale = true;

}

//---------------------------------------------------------

void Cloud::reconstruct(
		int kSVDIters, size_t kNN, size_t nfreq, size_t natm, size_t latm,
		size_t maxNewPoints, BoundBox* BBox, SparseApprox method)
{
	QMutexLocker locker(&m_recMutex);
	//assert(m_CT != nullptr);
	assert(kNN >= 1 && nfreq >= 1 && natm >= 1 && latm >= 1 && latm <= natm);
	static const Vector3f zaxis(0.0f, 0.0f, 1.0f);
	double time_elapsed_ms;
	std::clock_t c_start, c_end;

	size_t npoints_orig = m_cloud.size();
	size_t nfreqsq = nfreq*nfreq;
	size_t maxGridDim = max(1, int(floor(sqrt(float(kNN)))));
	bool useBBox = (BBox != nullptr);
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
	auto fit_gridXY = [](
			const vector<Vector3f>& vneighsXY, float SD,
			float& sizeX, float& sizeY, size_t& gridDim)
	{
		sizeX = sizeY = 0.0f;
		size_t n = vneighsXY.size();
		for(size_t i=0; i<n; ++i){
			Vector3f v = vneighsXY[i];
			sizeX += abs(v(0));
			sizeY += abs(v(1));
		}
		sizeX = SD*(sizeX/n);
		sizeY = SD*(sizeY/n);
		size_t numInGrid = 0;
		for(size_t i=0; i<n; ++i){
			if( abs(vneighsXY[i](0)) > sizeX ) continue;
			if( abs(vneighsXY[i](1)) > sizeY ) continue;
			++numInGrid;
		}
		gridDim = max(1, int(0.8f*floor(sqrt(float(numInGrid)))));
	};
	//--------
	auto get_gridXY_occupancy = [=](
			const vector<CoverTreePoint<Vector3f>>& neighs,
			const vector<Vector3f>& vneighsXY,
			float sizeX, float sizeY, size_t gridDim,
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
			loc.cellX = int(floor(gridDim*loc.u));
			loc.cellY = int(floor(gridDim*loc.v));
			++gridXY(loc.cellX, loc.cellY);
			loc.inGrid = true;
		}
	};
	//--------

	auto setup_grid = [&](
			Vector3f norm,
			const vector<CoverTreePoint<Vector3f>>& neighs,
			const vector<Vector3f>& vneighs,
			vector<Vector3f>& vneighsXY, Matrix3f& rotXY,
			MapMtrxi& gridXY, float& sizeX, float& sizeY,
			size_t& gridDim, vector<gridLocation>& gridLoc,
			vectoria& iworkGrid)
	{
		gridDim = 0;
		vector_to_vector_rotation_matrix(norm, zaxis, true, true, rotXY);
		vneighsXY.resize(vneighs.size());
		for(size_t j=0; j<vneighs.size(); ++j){
			vneighsXY[j].noalias() = rotXY*vneighs[j];
		}

		fit_gridXY(vneighsXY, 1.5f, sizeX, sizeY, gridDim);
		if(min(sizeX,sizeY) <= 1e-5*max(sizeX,sizeY)) return false;
		if(sizeX <= float_tiny || sizeY <= float_tiny) return false;
		new (&gridXY) MapMtrxi(&iworkGrid[0], gridDim, gridDim);
		get_gridXY_occupancy(
					neighs, vneighsXY, sizeX, sizeY,
					gridDim, gridXY, gridLoc);

		return true;
	};

	//--------

	auto setup_grid_signal = [&](
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
	MapMtrxi gridXY(nullptr, maxGridDim, maxGridDim);
	vectoria iworkGrid(maxGridDim*maxGridDim);
	Matrix3f rotXY, rotXYInv;
	vector<gridLocation> gridLoc;

	VectorXf Usig, Vsig, Wsig;
	vector<VectorXf> Us;
	vector<VectorXf> Vs;
	vector<VectorXf> Ws;

	queue<size_t> qpoints;
	size_t threshold = 0, lastPos = 0;

	if(m_msgLogger != nullptr) {
		//***
		QCoreApplication::processEvents();
		emit logMessage("\nPoint cloud reconstruction:");
	}

	if( m_CTStale ) buildSpatialIndex();

	for(size_t idx = 0; idx < npoints_orig; ++idx){
		// Log progress
		if(m_msgLogger != nullptr) {
			//***
			QCoreApplication::processEvents();
			emit logProgress(
						"Setting up signals",
						idx+1, npoints_orig, 5, threshold, lastPos);
		}

		Vector3f p = m_cloud[idx];
		pointKNN(p, kNN, neighs);
		getNeighVects(p, neighs, vneighs);
		Vector3f norm = update_normal(idx, neighs);

		float sizeX, sizeY;
		size_t gridDim;
		bool success = setup_grid(
					norm, neighs, vneighs, vneighsXY, rotXY,
					gridXY, sizeX, sizeY, gridDim, gridLoc,
					iworkGrid);

		if( !success ) continue;

		setup_grid_signal(gridLoc, Usig, Vsig, Wsig);
		Us.push_back(Usig);
		Vs.push_back(Vsig);
		Ws.push_back(Wsig);

		bool hasEmptyCell = false;
		for(size_t j=0; j<gridDim; ++j){
			for(size_t k=0; k<gridDim; ++k){
				if(gridXY(k,j) > 0) continue;
				hasEmptyCell = true;
				goto loopEnd0;
			}
		}
loopEnd0:
		if( hasEmptyCell ) qpoints.push(idx);

	}

	//--------


	MatrixXf D = MatrixXf::Random(nfreqsq, natm);
	D.colwise().normalize();
	MatrixXf C = MatrixXf::Random(natm, npoints_orig);

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
	case SparseApprox::MatchingPursuit :
		sparseFunct = mp;
		break;
	case SparseApprox::OrthogonalPursuit :
		sparseFunct = op;
		break;
	}

	if(m_msgLogger != nullptr) {
		emit logMessage("Training dictionary...");
		QCoreApplication::processEvents();
	}

	c_start = std::clock();//***

	ksvd_dct2D(true, Ws, Us, Vs, nfreq, latm,
			   kSVDIters, 0.0, sparseFunct, D, C);

	c_end = std::clock();//***
	time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
	std::cout << "\nCPU time used: " << time_elapsed_ms << " ms\n" << std::endl;


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

	size_t paddedA[5] = {
		align_padded(kNN*nfreqsq),
		align_padded(kNN*natm),
		align_padded(kNN*natm),
		align_padded(kNN*natm),
		align_padded(2*kNN*nfreqsq)
	};
	size_t nworkDctA =
			paddedA[0] + paddedA[1] + paddedA[2] +
			paddedA[3] + paddedA[4];
	vectorfa dworkDctA(nworkDctA);
	vectorfa dworkDctB;

	if(m_msgLogger != nullptr) {
		emit logMessage("Reconstructing point cloud...");
		QCoreApplication::processEvents();
		//***
		emit logMessage(QString::number(qpoints.size()) + " points in queue...");
		QCoreApplication::processEvents();
	}

	size_t nprocessed = 0;
	size_t nNewPoints = 0;
	while( !qpoints.empty() ){
		//std::cout << qpoints.size() << std::endl;
		// Log progress
		if(m_msgLogger != nullptr) {
			if( nprocessed%1000 == 0 ){
				//***
				emit logMessage(QString::number(nprocessed) + " points processed...");
				QCoreApplication::processEvents();
			}
			++nprocessed;
		}

		size_t idx = qpoints.front();
		qpoints.pop();

		Vector3f p = m_cloud[idx];
		pointKNN(p, kNN, neighs);

		//qpoints.push(idx);
		//if( nprocessed > 25000 ) break;
		//continue;//***


		getNeighVects(p, neighs, vneighs);

		Vector3f norm;
		if( idx < npoints_orig ){
			norm = m_norms[idx];
		}
		else{
			norm = update_normal(idx, neighs);
		}

		float sizeX, sizeY;
		size_t gridDim;
		bool success = setup_grid(
					norm, neighs, vneighs, vneighsXY, rotXY,
					gridXY, sizeX, sizeY, gridDim, gridLoc,
					iworkGrid);

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
			if( gridLoc[j].idx < npoints_orig ) continue;
			gridLoc0.push_back(gridLoc[j]);
		}
		size_t nsmpl0a = gridLoc0.size();

		size_t npoints = m_cloud.size();
		for(size_t j=0; j<gridDim; ++j){
			for(size_t k=0; k<gridDim; ++k){
				if(gridXY(k,j) > 0) continue;
				gridLocation loc;
				loc.idx = npoints;
				loc.inCloud = false;
				loc.inGrid = true;
				loc.u = (k+0.5f)/gridDim;
				loc.v = (j+0.5f)/gridDim;
				loc.w = 0.0f;
				loc.cellX = k;
				loc.cellY = j;
				gridLoc0.push_back(loc);
				++npoints;
			}
		}
		size_t nsmpl0b = gridLoc0.size() - nsmpl0a;
		size_t nsmpl0 = gridLoc0.size();
		assert(nsmpl0 <= kNN && nsmpl0b <= 2*kNN);

		setup_grid_signal(gridLoc0, U0sig, V0sig, W0sig);

		offset += paddedA[2];
		new (&T0D) MapMtrxf(&dworkDctA[offset], nsmpl0, natm);

		for(size_t j=0, k=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inGrid ) continue;
			if( gridLoc[j].idx < npoints_orig ) continue;
			T0D.row(k) = TD.row(gridLoc[j].sigIdx);
			++k;
		}

		offset += paddedA[3];
		new (&T0b) MapMtrxf(&dworkDctA[offset], nsmpl0b, nfreqsq);
		U0bsig.resize(nsmpl0b);
		V0bsig.resize(nsmpl0b);
		U0bsig.segment(0,nsmpl0b) = U0sig.segment(nsmpl0a,nsmpl0b);
		V0bsig.segment(0,nsmpl0b) = V0sig.segment(nsmpl0a,nsmpl0b);
		cosine_transform(U0bsig, V0bsig, nfreq, dworkDctB, T0b);
		T0D.block(nsmpl0a,0,nsmpl0b,natm).noalias() = T0b*D;

		W0sig.noalias() = T0D*Csig;

		rotXYInv = rotXY.transpose();
		for(size_t j=0; j<gridLoc0.size(); ++j){
			gridLocation& loc = gridLoc0[j];
			if( !loc.inCloud ){
				if( nNewPoints >= maxNewPoints ) continue;
			}
			Vector3f qXY, q;
			qXY(0) = (2*loc.u - 1.0f)*sizeX;
			qXY(1) = (2*loc.v - 1.0f)*sizeY;
			qXY(2) = W0sig(j);
			q.noalias() = rotXYInv*qXY;
			q += p;
			if( useBBox ){
				if( !BBox->pointInBBox(q) ) continue;
			}
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

		//***
//		std::cout << " " << maxNewPoints
//				  << " " << nNewPoints
//				  << std::endl;

//		std::cout << " " << nsmpl
//				  << " " << nsmpl0
//				  << " " << nsmpl0a
//				  << " " << nsmpl0b
//				  << " " << gridDim
//				  << " " << m_cloud.size()
//				  << std::endl;

		if( nNewPoints >= maxNewPoints ) break;

	}




	m_CTStale = true;


}


//---------------------------------------------------------

