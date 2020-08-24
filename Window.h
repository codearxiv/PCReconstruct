//-----------------------------------------------------------
//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef WINDOW_H
#define WINDOW_H

#include "MessageLogger.h"
#include "constants.h"

//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <QWidget>

QT_BEGIN_NAMESPACE
class QSlider;
class QPushButton;
class QMainWindow;
QT_END_NAMESPACE

class GLWidget;


class Window : public QWidget
{
    Q_OBJECT

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	Window(QMainWindow *mw, MessageLogger* msgLogger = nullptr);

public slots:
	void setCloud(CloudPtr cloud)
	{ emit cloudChanged(cloud); }

	void getCloud(CloudPtr& cloud)
	{ emit cloudQueried(cloud); }

	void undoCloud() { emit cloudUndo(); }

	void viewGLCloudNorms(bool enabled)
	{ emit cloudNormsViewGL(enabled); }

    void setRandomCloud(size_t nPoints)
    { emit cloudSetRandom(nPoints); }

    void setCloudBBox(float minBBox[3], float maxBBox[3])
    { emit cloudSetBBox(minBBox, maxBBox); }

	void approxCloudNorms(int nIters, size_t kNN)
	{ emit cloudApproxNorms(nIters, kNN); }

	void decimateCloud(size_t nHoles, size_t kNN)
	{ emit cloudDecimate(nHoles, kNN); }

	void sparsifyCloud(float percent)
	{ emit cloudSparsify(percent); }

	void reconstructCloud(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints, bool looseBBox,
			SparseApprox method)
	{
		emit cloudReconstruct(
                    kSVDIters, kNN, nfreq, densify, natm, latm,
					maxNewPoints, looseBBox, method);
	}

    void setPointSize(float size)
    { emit pointSizeChanged(size); }

	void setNormScale(float scale)
	{ emit normScaleChanged(scale); }

	void changeBBoxFields(float minBBox[3], float maxBBox[3])
	{ emit bBoxFieldsChanged(minBBox, maxBBox);}


signals:
	void cloudChanged(CloudPtr cloud);
	void cloudQueried(CloudPtr& cloud);
	void cloudUndo();
	void cloudNormsViewGL(bool enabled);
	void cloudSetRandom(size_t nPoints);
    void cloudSetBBox(float minBBox[3], float maxBBox[3]);
	void cloudApproxNorms(int nIters, size_t kNN);
	void cloudDecimate(size_t nHoles, size_t kNN);
	void cloudSparsify(float percent);
	void cloudReconstruct(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints, bool looseBBox,
			SparseApprox method);
    void pointSizeChanged(float size);
	void normScaleChanged(float scale);
	void bBoxFieldsChanged(float minBBox[3], float maxBBox[3]);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private:
    QSlider *createSlider();

    GLWidget *glWidget;
	QMainWindow *mainWindow;
	MessageLogger* m_msgLogger;

};

#endif
