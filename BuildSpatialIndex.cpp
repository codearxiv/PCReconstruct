#include "BuildSpatialIndex.h"

//-------------------------------------------------------------------------
BuildSpatialIndex::BuildSpatialIndex(QObject *parent)
	: QThread(parent)
{

}

//-------------------------------------------------------------------------
BuildSpatialIndex::~BuildSpatialIndex()
{
	requestInterruption();
	wait();
}

//-------------------------------------------------------------------------
void BuildSpatialIndex::build(const vector<Vector3f>& cloud,
							  CoverTree<CoverTreePoint<Vector3f>>& CT)
{
	QMutexLocker locker(&m_mutex);

	m_cloud = &cloud;
	m_CT = &CT;

	start();
}

//-------------------------------------------------------------------------

void BuildSpatialIndex::run()
{

	size_t npoints = m_cloud->size();
	int threshold = 0;

	for(size_t i = 0; i < npoints; ++i){
//		break;
		// Log progress
		QCoreApplication::processEvents();
		if(m_msgLogger != nullptr) {
			emit logProgress(
						"Building cloud spatial index",
						i, npoints, 5, threshold);
			//***
			if(threshold > 25) break;
		}

		Vector3f v;
		v[0] = (*m_cloud)[i][0];
		v[1] = (*m_cloud)[i][1];
		v[2] = (*m_cloud)[i][2];
		CoverTreePoint<Vector3f> cp(v, i);
		m_CT->insert(cp);
	}

}
//-------------------------------------------------------------------------
