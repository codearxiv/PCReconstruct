//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include <QMainWindow>
#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include "MessageLogger.h"

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QPlainTextEdit;
class QListWidget;
QT_END_NAMESPACE

class GLWidget;
class Window;


class MainWindow : public QMainWindow
{
	Q_OBJECT

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	explicit MainWindow(QWidget *parent = nullptr);

private:
	void open();
	void saveAs();
	void about();

public slots:
	void setCloud(CloudPtr cloud)
	{ emit cloudChanged(cloud); }

	void getCloud(CloudPtr& cloud)
	{ emit cloudQueried(cloud); }

	void appendLogText(const QString& text);

signals:
	void cloudChanged(CloudPtr cloud);
	void cloudQueried(CloudPtr& cloud);

private:
	Window *centralWidget;
	QToolBar *mainToolBar;
	QStatusBar *statusBar;
	QToolBar *toolBar;
	QPlainTextEdit *logText;

	MessageLogger msgLogger;

};

#endif // MAINWINDOW_H
