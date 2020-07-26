#ifndef RECONSTRUCTTHREAD_H
#define RECONSTRUCTTHREAD_H

//#include <QMutex>
//#include <QThread>
//#include <vector>
//#include <Eigen/Dense>
#include "Cover_Tree.h"
#include "CoverTreePoint.h"

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
