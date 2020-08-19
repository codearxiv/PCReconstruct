#include "CloudWorker.h"
#include "Cloud.h"
#include "constants.h"

//-------------------------------------------------------------------------
CloudWorker::CloudWorker(Cloud& cloud, BoundBox& boundBox, QObject *parent) :
	QObject(parent)
{
	m_cloud = &cloud;
	m_boundBox = &boundBox;
}

//-------------------------------------------------------------------------
CloudWorker::~CloudWorker()
{
}


//-------------------------------------------------------------------------
void CloudWorker::decimateCloud(size_t nHoles, size_t kNN)
{
	if(m_cloud->pointCount() == 0) return;
	m_cloud->decimate(nHoles, kNN);

	emit finished();
}

//-------------------------------------------------------------------------

void CloudWorker::reconstructCloud(
		int kSVDIters, size_t kNN, size_t nfreq,
		size_t natm, size_t latm, size_t maxNewPoints,
		SparseApprox method)
{
	if(m_cloud->pointCount() == 0) return;
	m_cloud->reconstruct(
				kSVDIters, kNN, nfreq, natm, latm,
				maxNewPoints, m_boundBox, method);

	emit finished();

}


//-------------------------------------------------------------------------
