//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "constants.h"

//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <QMainWindow>

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QPlainTextEdit;
class QListWidget;
QT_END_NAMESPACE

class GLWidget;
class Window;
class MessageLogger;
class SetRandomDialog;
class DecimateDialog;
class ReconstructDialog;

class MainWindow : public QMainWindow
{
	Q_OBJECT

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	explicit MainWindow(QWidget *parent = nullptr);
	void badInputMessageBox(const QString& info);

private:
	void open();
	void saveAs();
	void setRandom();
	void decimate();
	void sparsify();
	void reconstruct();
	void about();

public slots:
	void setCloud(CloudPtr cloud)
	{ emit cloudChanged(cloud); }

	void getCloud(CloudPtr& cloud)
	{ emit cloudQueried(cloud); }

	void setRandomCloud(size_t nPoints)
	{ emit cloudSetRandom(nPoints); }

	void decimateCloud(size_t nHoles, size_t kNN)
	{ emit cloudDecimate(nHoles, kNN); }

	void sparsifyCloud(float percent)
	{ emit cloudSparsify(percent); }

	void reconstructCloud(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method)
	{
		emit cloudReconstruct(
                    kSVDIters, kNN, nfreq, densify, natm, latm,
					maxNewPoints, method);
	}

	void appendLogText(const QString& text);
	void insertLogText(const QString& text);

signals:
	void cloudChanged(CloudPtr cloud);
	void cloudQueried(CloudPtr& cloud);
	void cloudSetRandom(size_t nPoints);
	void cloudDecimate(size_t nHoles, size_t kNN);
	void cloudSparsify(float percent);
	void cloudReconstruct(
            int kSVDIters, size_t kNN, size_t nfreq, float densify,
			size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method);

private:
	Window *centralWidget;
	QToolBar *mainToolBar;
	QStatusBar *statusBar;
	QToolBar *toolBar;
	QPlainTextEdit *logText;
	SetRandomDialog *setRandomDialog;
	DecimateDialog *decimateDialog;
	ReconstructDialog *reconstructDialog;

	MessageLogger *msgLogger;

};

#endif // MAINWINDOW_H
