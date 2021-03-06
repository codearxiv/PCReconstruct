//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

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
class RandomSurfDialog;
class BoundBoxDialog;
class NormalsDialog;
class DecimateDialog;
class SparsifyDialog;
class ReconstructDialog;
class OptionsDialog;

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
	void undo();
	void viewGLNorms(bool enabled);
	void setRandom();
    void setBBox();
	void approxNorms();
	void decimate();
	void sparsify();
	void reconstruct();
    void options();
    void about();

public slots:
	void changeBBoxFields(float minBBox[3], float maxBBox[3]);
	void appendLogText(const QString& text);
	void insertLogText(const QString& text);

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

private:
	Window *centralWidget;
	QToolBar *mainToolBar;
	QStatusBar *statusBar;
	QToolBar *toolBar;
	QPlainTextEdit *logText;
    RandomSurfDialog *randomSurfDialog;
    BoundBoxDialog *boundBoxDialog;
	NormalsDialog *normalsDialog;
	DecimateDialog *decimateDialog;
	SparsifyDialog *sparsifyDialog;
	ReconstructDialog *reconstructDialog;
    OptionsDialog *optionsDialog;

	MessageLogger *msgLogger;

};

#endif // MAINWINDOW_H
