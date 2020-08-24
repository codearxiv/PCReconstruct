//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef RECONSTRUCTTHREAD_H
#define RECONSTRUCTTHREAD_H

#include "constants.h"

#include <QMutex>
#include <QObject>

class Cloud;
class BoundBox;


class CloudWorker : public QObject
{
	Q_OBJECT

public:
	explicit CloudWorker(
			Cloud& cloud, QObject *parent = nullptr);
	~CloudWorker();

public slots:
	void approxCloudNorms(int nIters, size_t kNN);
	void decimateCloud(size_t nHoles, size_t kNN);
	void sparsifyCloud(float percent);
	void reconstructCloud(
			int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints, bool looseBBox,
			SparseApprox method);


signals:
	void finished(bool updateBBox);

private:
	Cloud *m_cloud;
	QMutex m_mutex;
};


#endif // RECONSTRUCTTHREAD_H
