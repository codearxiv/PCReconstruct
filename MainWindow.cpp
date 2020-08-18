//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#include "MainWindow.h"
#include "Window.h"
#include "MessageLogger.h"
#include "SetRandomDialog.h"
#include "DecimateDialog.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QApplication>
#include <QCoreApplication>
#include <QToolBar>
#include <QDockWidget>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QFileDialog>
#include <QMenuBar>
#include <QMenu>
#include <QMessageBox>
#include <QMainWindow>


MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent)
{
	if (objectName().isEmpty())
		setObjectName(QString::fromUtf8("MainWindow"));
	resize(1204, 640);

	setWindowTitle(QCoreApplication::translate("MainWindow", "PCReconstruct", nullptr));

	//------
	// Add docks
	QDockWidget *dock = new QDockWidget(tr("Log Window"), this);
	dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	logText = new QPlainTextEdit;
	logText->setReadOnly(true);
	dock->setWidget(logText);
	addDockWidget(Qt::RightDockWidgetArea, dock);

	//------
	// Add actions
	QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
	QMenu *viewMenu = menuBar()->addMenu(tr("&View"));
	QMenu *toolsMenu = menuBar()->addMenu(tr("&Tools"));
	QMenu *helpMenu = menuBar()->addMenu(tr("&Help"));

	QToolBar *fileToolBar = addToolBar(tr("File"));

	const QIcon openIcon =
			QIcon::fromTheme("document-open", QIcon(":/images/open.png"));
	QAction *openAct = new QAction(openIcon, tr("&Open..."), this);
	openAct->setShortcuts(QKeySequence::Open);
	openAct->setStatusTip(tr("Open an existing PCD file"));
	connect(openAct, &QAction::triggered, this, &MainWindow::open);
	fileMenu->addAction(openAct);
	fileToolBar->addAction(openAct);

	const QIcon saveAsIcon =
			QIcon::fromTheme("document-save-as", QIcon(":/images/save.png"));
	QAction *saveAsAct = new QAction(saveAsIcon, tr("Save &As..."), this);
	saveAsAct->setShortcuts(QKeySequence::SaveAs);
	saveAsAct->setStatusTip(tr("Save PCD to disk"));
	connect(saveAsAct, &QAction::triggered, this, &MainWindow::saveAs);
	fileMenu->addAction(saveAsAct);
	fileToolBar->addAction(saveAsAct);

	const QIcon exitIcon = QIcon::fromTheme("application-exit");
	QAction *exitAct =
			fileMenu->addAction(exitIcon, tr("E&xit"), this, &QWidget::close);
	exitAct->setShortcuts(QKeySequence::Quit);
	exitAct->setStatusTip(tr("Exit PCReconstruct"));

	viewMenu->addAction(dock->toggleViewAction());

	QAction *setRandomAct = new QAction(tr("&Set Random"), this);
	saveAsAct->setStatusTip(tr("Sample a point cloud randomly from a random surface"));
	connect(setRandomAct, &QAction::triggered, this, &MainWindow::setRandom);
	toolsMenu->addAction(setRandomAct);

	QAction *decimateAct = new QAction(tr("&Decimate"), this);
	saveAsAct->setStatusTip(tr("Generate random holes in point cloud"));
	connect(decimateAct, &QAction::triggered, this, &MainWindow::decimate);
	toolsMenu->addAction(decimateAct);

	QAction *aboutAct =
			helpMenu->addAction(tr("&About"), this, &MainWindow::about);
	aboutAct->setStatusTip(tr("About"));

	QAction *aboutQtAct =
			helpMenu->addAction(tr("About &Qt"), qApp, &QApplication::aboutQt);
	aboutQtAct->setStatusTip(tr("About Qt"));


	//------
	// Central widget

	msgLogger = new MessageLogger(logText);
	centralWidget = new Window(this, msgLogger);
	centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
	setCentralWidget(centralWidget);

	connect(this, &MainWindow::cloudChanged,
			centralWidget, &Window::setCloud);

	connect(this, &MainWindow::cloudQueried,
			centralWidget, &Window::getCloud);

	connect(this, &MainWindow::cloudSetRandom,
			centralWidget, &Window::setRandomCloud);

	connect(this, &MainWindow::cloudDecimate,
			centralWidget, &Window::decimateCloud);


	//------
	// Dialogs

	setRandomDialog = new SetRandomDialog(this);
	decimateDialog = new DecimateDialog(this);
	//sparsityDialog = new sparsifyDialog(this);


	//------

	QMetaObject::connectSlotsByName(this);

}

//---------------------------------------------------------

void MainWindow::open()
{

	QString q_pcdPath = QFileDialog::getOpenFileName(
				this, tr("Open File"), "", tr("PCD (*.pcd)")
				);

	std::string pcdPath = q_pcdPath.toStdString();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	int success = 0;
	try{
		success = pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *cloud);
	}
	catch (...){
		success = -1;
	}

	if (success == -1) {
		//PCL_ERROR ("Couldn't read PCD file\n");
		appendLogText("Couldn't read PCD file\n");
		return;
	}
	else{
		appendLogText("Opened: " + q_pcdPath + "\n");
	}

	setCloud(cloud);

}

//---------------------------------------------------------

void MainWindow::saveAs()
{
	QString q_pcdPath = QFileDialog::getSaveFileName(
	  this, tr("Open File"), "", tr("PCD (*.pcd)")
	);
	std::string pcdPath = q_pcdPath.toStdString();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	getCloud(cloud);

	int success = 0;
	try{
		success = pcl::io::savePCDFile<pcl::PointXYZ>(pcdPath, *cloud);
	}
	catch (...){
		success = -1;
	}

	if (success == -1) {
		//PCL_ERROR ("Couldn't save PCD file\n");
		appendLogText("Couldn't save PCD file\n");
		return;
	}
	else{
		appendLogText("Saved: " + q_pcdPath + "\n");
	}


}


//---------------------------------------------------------

void MainWindow::setRandom()
{
	// Show the dialog as modal
	if(setRandomDialog->exec() == QDialog::Accepted){
		size_t nPoints;
		bool ok = setRandomDialog->getFields(nPoints);
		if(!ok){
			badInputMessageBox("All fields should be integers bigger than zero.");
			return;
		}
		setRandomCloud(nPoints);
	}

}

//---------------------------------------------------------

void MainWindow::decimate()
{
	// Show the dialog as modal
	if(decimateDialog->exec() == QDialog::Accepted){
		size_t nHoles, kNN;
		bool ok = decimateDialog->getFields(nHoles, kNN);
		if(!ok){
			badInputMessageBox("All fields should be integers bigger than zero.");
			return;
		}
		decimateCloud(nHoles,kNN);
	}

}

//---------------------------------------------------------

void MainWindow::about()
{
   QMessageBox::about(this, tr("About"),
			tr("PCReconstruct. Copyright 2019 Piotr (Peter) Beben."));
}

//---------------------------------------------------------

void MainWindow::badInputMessageBox(const QString& info)
{
	QMessageBox msgBox;
	msgBox.setText("Invalid input.");
	msgBox.setInformativeText(info);
	msgBox.setStandardButtons(QMessageBox::Ok);
	msgBox.setDefaultButton(QMessageBox::Ok);
	msgBox.setIcon(QMessageBox::Information);
	msgBox.exec();
}

//---------------------------------------------------------

void MainWindow::appendLogText(const QString& text)
{
	logText->appendPlainText(text);
	logText->verticalScrollBar()->setValue(
				logText->verticalScrollBar()->maximum());
}


//---------------------------------------------------------
