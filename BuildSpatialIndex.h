#ifndef BUILDSPATIALINDEX_H
#define BUILDSPATIALINDEX_H

//#include <QMutex>
//#include <QThread>
//#include <vector>
//#include <Eigen/Dense>
#include "Cover_Tree.h"
#include "CoverTreePoint.h"

class BuildSpatialIndex : public QThread
{
	Q_OBJECT

public:
	BuildSpatialIndex(QObject *parent = nullptr);
	~BuildSpatialIndex();

	void build(const vector<Vector3f>& cloud,
			   CoverTree<CoverTreePoint<Vector3f>>& CT);

protected:
	void run() override;

private:
	vector<Vector3f>* m_cloud;
	CoverTree<CoverTreePoint<Vector3f>>* m_CT;
	QMutex m_mutex;

};


#endif // BUILDSPATIALINDEX_H
