#ifndef RECONSTRUCTTHREAD_H
#define RECONSTRUCTTHREAD_H

#include "Cover_Tree.h"
#include "CoverTreePoint.h"

#include <Eigen/Dense>
#include <QMutex>
#include <QThread>
#include <vector>

class ReconstructThread : public QThread
{
	Q_OBJECT

public:
	ReconstructThread(QObject *parent = nullptr);
	~ReconstructThread();

	void reconstruct();

protected:
	void run() override;

private:
	QMutex m_mutex;

};


#endif // RECONSTRUCTTHREAD_H
