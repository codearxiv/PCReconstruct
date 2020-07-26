#include "ReconstructThread.h"

//-------------------------------------------------------------------------
ReconstructThread::ReconstructThread(QObject *parent)
	: QThread(parent)
{

}

//-------------------------------------------------------------------------
ReconstructThread::~ReconstructThread()
{
	requestInterruption();
	wait();
}

//-------------------------------------------------------------------------
void ReconstructThread::reconstruct()
{
	QMutexLocker locker(&m_mutex);

	start();
}

//-------------------------------------------------------------------------

void ReconstructThread::run()
{


}

//-------------------------------------------------------------------------
