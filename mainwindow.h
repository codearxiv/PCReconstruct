//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include <QMainWindow>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class GLWidget;

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
	Q_OBJECT

	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

public:
	explicit MainWindow(QWidget *parent = nullptr);
	~MainWindow();

private slots:
	void on_openButton_pressed();
	void on_saveButton_pressed();

	void setCloud(CloudPtr cloud)
	{ emit cloudChanged(cloud); }

signals:
	void cloudChanged(CloudPtr cloud);


private:
	Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
