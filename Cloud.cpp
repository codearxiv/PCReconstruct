//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

//#include <numeric>
//#include <algorithm>

#include "Cloud.h"
#include "constants.h"
#include "cloud_normal.h"
#include "cosine_transform.h"
#include "rotations.h"
#include "MatchingPursuit.h"
#include "OrthogonalPursuit.h"
#include "ksvd_dct2D.h"

template<typename T> using queue = std::queue<T>;
using std::max;
using std::min;
using std::abs;
using std::floor;
using VectorXf = Eigen::VectorXf;
using MatrixXf = Eigen::MatrixXf;
using MatrixXi = Eigen::MatrixXi;
using MapMtrxf = Eigen::Map<MatrixXf, ALIGNEDX>;
template<typename T> using aligned = Eigen::aligned_allocator<T>;
using vectorfa = std::vector<float, aligned<float>>;


namespace std {
template<class T> class function;
}


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

void Cloud::replacePoint(
		size_t idx, const Vector3f& v, const Vector3f& n, bool threadSafe)
{
	if ( threadSafe ) m_recMutex.lock();
	assert(idx < m_cloud.size());

	m_cloud[idx] = v;
	m_norms[idx] = n;

	if( m_CT != nullptr ) {
		CoverTreePoint<Vector3f> cp(v, m_cloud.size());
		m_CT->remove(cp);
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
	const Vector3f& p, int iters,
	const vector<CoverTreePoint<Vector3f>>& neighs,
	vector<Vector3f>& vneighs)
{
	assert(m_CT != nullptr);
	getNeighVects(p, neighs, vneighs);
	return cloud_normal(p, vneighs, iters);
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
		vneighs.push_back( it->getVec() );
	}
}

//---------------------------------------------------------

void Cloud::approxCloudNorms(int iters, size_t kNN)
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
		CoverTreePoint<Vector3f> cp(p, 0);
		neighs = m_CT->kNearestNeighbors(cp, kNN);
		m_norms[i] = approxNorm(p, iters, neighs, vneighs);
	}

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

void Cloud::reconstruct(
		int kSVDIters, size_t kNN, size_t nfreq, size_t natm, size_t latm,
		SparseApprox method)
{
	QMutexLocker locker(&m_recMutex);
	assert(m_CT != nullptr);
	assert(kNN >= 1 && nfreq >= 1 && natm >= 1 && latm >= 1 && latm <= natm);

	size_t npoints_orig = m_cloud.size();
	//size_t nfreq = std::ceil(sqrt(float(kNN)));
	size_t ndim = nfreq*nfreq;

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
		for(size_t i=0; i<=n; ++i){
			Vector3f v = vneighsXY[i];
			sizeX += v(0);
			sizeY += v(1);
		}
		sizeX = SD*(sizeX/n);
		sizeY = SD*(sizeY/n);
		gridDim = max(1, int(floor(sqrt(float(n)))));
	};
	//--------
	auto get_gridXY_occupancy = [=](
			const vector<CoverTreePoint<Vector3f>>& neighs,
			const vector<Vector3f>& vneighsXY,
			float sizeX, float sizeY, size_t gridDim,
			MatrixXi& gridXY, vector<gridLocation>& gridLoc)
	{
		gridXY.setZero(nfreq, nfreq);
		gridLoc.resize(neighs.size());
		for(size_t i=0; i<=neighs.size(); ++i){
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
			loc.cellX = gridDim*floor(loc.u);
			loc.cellY = gridDim*floor(loc.v);
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
			MatrixXi& gridXY, float& sizeX, float& sizeY,
			size_t& gridDim, vector<gridLocation>& gridLoc)
	{
		const Vector3f zaxis(0.0f, 0.0f, 1.0f);
		gridDim = 0;
		vector_to_vector_rotation_matrix(norm, zaxis, true, true, rotXY);
		vneighsXY.resize(vneighs.size());
		for(size_t j=0; j<vneighs.size(); ++j){
			vneighsXY[j] = rotXY*vneighs[j];
		}

		fit_gridXY(vneighsXY, 2.0f, sizeX, sizeY, gridDim);
		if(min(sizeX,sizeY) <= 1e-5*max(sizeX,sizeY)) return false;
		if(sizeX <= float_tiny || sizeY <= float_tiny) return false;
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
		for(size_t j=0; j<=gridLoc.size(); ++j){
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

	vector<CoverTreePoint<Vector3f>> neighs;
	vector<Vector3f> vneighs;
	vector<Vector3f> vneighsXY;
	MatrixXi gridXY(nfreq, nfreq);
	Matrix3f rotXY, rotXYInv;
	vector<gridLocation> gridLoc;

	VectorXf Wsig, Usig, Vsig;
	vector<VectorXf> Ws(npoints_orig);
	vector<VectorXf> Us(npoints_orig);
	vector<VectorXf> Vs(npoints_orig);

	queue<size_t> qfront;

	approxCloudNorms(10, 25);

	for(size_t i = 0; i < npoints_orig; ++i){
		Vector3f p = m_cloud[i];
		Vector3f norm = m_norms[i];
		pointKNN(p, kNN, neighs);
		getNeighVects(p, neighs, vneighs);

		float sizeX, sizeY;
		size_t gridDim;
		bool success = setup_grid(
					norm, neighs, vneighs, vneighsXY, rotXY,
					gridXY, sizeX, sizeY, gridDim, gridLoc);

		if( !success ) continue;

		setup_grid_signal(gridLoc, Usig, Vsig, Wsig);

		bool hasEmptyCell = false;
		for(size_t j=0; j<gridDim; ++j){
			for(size_t k=0; k<gridDim; ++k){
				if(gridXY(k,j) > 0) continue;
				hasEmptyCell = true;
				goto loopEnd0;
			}
		}
loopEnd0:
		if( hasEmptyCell ) qfront.push(i);

	}

	//--------


	MatrixXf D = MatrixXf::Random(ndim, natm);
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

	ksvd_dct2D(true, Ws, Us, Vs, nfreq, latm,
			   kSVDIters, 0.0, sparseFunct, D, C);

	//--------

	vector<CoverTreePoint<Vector3f>> neighsNrm;
	vector<Vector3f> vneighsNrm;

	MapMtrxf T(nullptr, kNN, ndim);
	MapMtrxf TD(nullptr, kNN, natm);
	MapMtrxf TDNrm(nullptr, kNN, natm);
	MapMtrxf T0b(nullptr, 2*kNN, ndim);
	MapMtrxf T0D(nullptr, 2*kNN, natm);
	VectorXf NrmInv(natm);
	VectorXf Csig(natm);
	VectorXf CsigNrm(natm);
	VectorXf R(ndim);
	VectorXf W0sig, U0sig, V0sig;
	VectorXf U0bsig, V0bsig;

	size_t paddedA[3] = {
		align_padded(kNN*ndim),
		align_padded(kNN*natm),
		align_padded(kNN*natm)
	};
	vectorfa dworkDctA(paddedA[0] + paddedA[1] + paddedA[2]);
	//vectorfa dworkDctA0(paddedA0[0] + paddedA0[1]);
	vectorfa dworkDctB;


	while( !qfront.empty() ){
		size_t i = qfront.front();
		qfront.pop();

		Vector3f p = m_cloud[i];
		Vector3f norm;
		if( i < npoints_orig ){
			norm = m_norms[i];
		}
		else{
			pointKNN(p, 25, neighsNrm);
			norm = approxNorm(p, 10, neighsNrm, vneighsNrm);
			m_norms[i] = norm;
		}

		pointKNN(p, kNN, neighs);
		getNeighVects(p, neighs, vneighs);

		float sizeX, sizeY;
		size_t gridDim;
		bool success = setup_grid(
					norm, neighs, vneighs, vneighsXY, rotXY,
					gridXY, sizeX, sizeY, gridDim, gridLoc);

		if( !success ) continue;

		setup_grid_signal(gridLoc, Usig, Vsig, Wsig);

		size_t nsmpl = Wsig.size();
		new (&T) MapMtrxf(&dworkDctA[0], nsmpl, ndim);
		new (&TD) MapMtrxf(&dworkDctA[paddedA[0]], nsmpl, natm);
		new (&TDNrm) MapMtrxf(&dworkDctA[paddedA[1]], nsmpl, natm);

		cosine_transform(Usig, Vsig, nfreq, dworkDctB, T);
		TD.noalias() = T*D;
		TDNrm = TD;
		column_normalize(TDNrm, NrmInv);

		sparseFunct(Wsig, TDNrm, latm, CsigNrm, R);
		Csig = CsigNrm.cwiseProduct(NrmInv);

		size_t nsmpl0a = 0;
		for(size_t j=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inGrid ) continue;
			if(gridLoc[j].idx <= npoints_orig){
				gridLoc[j].inGrid = false;
			}
			else{
				++nsmpl0a;
			}
		}

		size_t npoints = m_cloud.size();
		size_t nsmpl0b = 0;
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
				gridLoc.push_back(loc);
				++nsmpl0b;
				++npoints;
			}
		}

		setup_grid_signal(gridLoc, U0sig, V0sig, W0sig);

		size_t nsmpl0 = W0sig.size();
		new (&T0D) MapMtrxf(&dworkDctA[paddedA[2]], nsmpl0, natm);

		for(size_t j=0, k=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inCloud ) break;
			if( !gridLoc[j].inGrid ) continue;
			T0D.row(k) = TD.row(j);
			++k;
		}

		new (&T0b) MapMtrxf(&dworkDctA[0], nsmpl0b, ndim);
		U0bsig.resize(nsmpl0b);
		V0bsig.resize(nsmpl0b);
		U0bsig.segment(0,nsmpl0b) = U0sig.segment(nsmpl0a,nsmpl0b);
		V0bsig.segment(0,nsmpl0b) = V0sig.segment(nsmpl0a,nsmpl0b);
		cosine_transform(U0bsig, V0bsig, nfreq, dworkDctB, T0b);
		T0D.block(nsmpl0a,0,nsmpl0b,natm).noalias() = T0b*D;

		W0sig = T0D*Csig;

		for(size_t j=0; j<gridLoc.size(); ++j){
			if( !gridLoc[j].inGrid ) continue;
			gridLoc[j].w = W0sig(gridLoc[j].sigIdx);
		}

		rotXYInv = rotXY.transpose();
		for(size_t j=0; j<gridLoc.size(); ++j){
			gridLocation& loc = gridLoc[i];
			if( !loc.inGrid ) continue;
			Vector3f qXY, q;
			qXY(0) = (2*loc.u - 1.0f)*sizeX;
			qXY(1) = (2*loc.v - 1.0f)*sizeY;
			qXY(2) = loc.w;
			q = rotXYInv*q;
			if( loc.inCloud ){
				replacePoint(loc.idx, q, m_norms[loc.idx]);
			}
			else{
				addPoint(q, norm);
			}
		}


	}


}


//---------------------------------------------------------

