#ifndef RECONSTRUCTTHREAD_H
#define RECONSTRUCTTHREAD_H

#include <QMutex>
#include <QObject>

class Cloud;


class CloudWorker : public QObject
{
	Q_OBJECT

public:
	explicit CloudWorker(Cloud& cloud, QObject *parent = nullptr);
	~CloudWorker();

public slots:
	void decimateCloud(size_t nHoles, size_t kNN);

signals:
	void finished();

private:
	Cloud *m_cloud;
	QMutex m_mutex;
};


#endif // RECONSTRUCTTHREAD_H
