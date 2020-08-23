#include "CloudWorker.h"
#include "Cloud.h"
#include "constants.h"

//-------------------------------------------------------------------------
CloudWorker::CloudWorker(Cloud& cloud, QObject *parent) :
	QObject(parent)
{
	m_cloud = &cloud;
}

//-------------------------------------------------------------------------
CloudWorker::~CloudWorker()
{
}


//-------------------------------------------------------------------------
void CloudWorker::decimateCloud(size_t nHoles, size_t kNN)
{
	if(m_cloud->pointCount() == 0) return;
	m_cloud->backup();
	m_cloud->decimate(nHoles, kNN);

	emit finished(false);
}

//-------------------------------------------------------------------------
void CloudWorker::sparsifyCloud(float percent)
{	
	if(m_cloud->pointCount() == 0) return;
	m_cloud->backup();
	m_cloud->sparsify(percent);

	emit finished(false);
}

//-------------------------------------------------------------------------

void CloudWorker::reconstructCloud(
		int kSVDIters, size_t kNN, size_t nfreq, float densify,
		size_t natm, size_t latm, size_t maxNewPoints, bool looseBBox,
		SparseApprox method)
{
	if(m_cloud->pointCount() == 0) return;
	m_cloud->backup();
	m_cloud->reconstruct(
				kSVDIters, kNN, nfreq, densify, natm, latm,
				maxNewPoints, looseBBox, method);

	emit finished(false);

}


//-------------------------------------------------------------------------
