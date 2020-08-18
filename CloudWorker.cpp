#include "CloudWorker.h"
#include "Cloud.h"

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
	m_cloud->decimate(nHoles, kNN);

	emit finished();
}

//-------------------------------------------------------------------------
