//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "window.h"
//#include "glwidget.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <QMenuBar>
//#include <QMenu>
//#include <QMessageBox>
//#include <QMainWindow>


MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	connect(this, &MainWindow::cloudChanged, ui->centralWidget, &Window::setCloud);

}

MainWindow::~MainWindow()
{
	delete ui;
}


void MainWindow::on_openButton_pressed()
{

	QString q_pcdPath = QFileDialog::getOpenFileName(
	  this, tr("Open File"), "", tr("PCD (*.pcd)")
	);
	std::string pcdPath = q_pcdPath.toStdString();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcdPath, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read PCD file\n");
		return;
	}

	emit cloudChanged(cloud);

}


void MainWindow::on_saveButton_pressed()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	emit cloudChanged(cloud);

}
