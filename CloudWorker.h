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
			Cloud& cloud, BoundBox& boundBox, QObject *parent = nullptr);
	~CloudWorker();

public slots:
	void decimateCloud(size_t nHoles, size_t kNN);
	void sparsifyCloud(float percent);
	void reconstructCloud(
			int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method);


signals:
	void finished();

private:
	Cloud *m_cloud;
	BoundBox *m_boundBox;
	QMutex m_mutex;
};


#endif // RECONSTRUCTTHREAD_H
