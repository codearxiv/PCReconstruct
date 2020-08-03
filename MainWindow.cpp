//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <QDockWidget>
//#include <QPlainTextEdit>
//#include <QMenuBar>
//#include <QMenu>
//#include <QMessageBox>
//#include <QMainWindow>

#include "MainWindow.h"
#include "Window.h"
#include "MessageLogger.h"


MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent), msgLogger()
{
	if (objectName().isEmpty())
		setObjectName(QString::fromUtf8("MainWindow"));
	resize(1204, 640);

	setWindowTitle(QCoreApplication::translate("MainWindow", "PCReconstruct", nullptr));

	// Add actions
	QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
	QMenu *viewMenu = menuBar()->addMenu(tr("&View"));
	QToolBar *fileToolBar = addToolBar(tr("File"));

	const QIcon openIcon = QIcon::fromTheme("document-open", QIcon(":/images/open.png"));
	QAction *openAct = new QAction(openIcon, tr("&Open..."), this);
	openAct->setShortcuts(QKeySequence::Open);
	openAct->setStatusTip(tr("Open an existing PCD file"));
	connect(openAct, &QAction::triggered, this, &MainWindow::open);
	fileMenu->addAction(openAct);
	fileToolBar->addAction(openAct);

	const QIcon saveAsIcon = QIcon::fromTheme("document-save-as", QIcon(":/images/save.png"));
	QAction *saveAsAct = new QAction(saveAsIcon, tr("Save &As..."), this);
	saveAsAct->setShortcuts(QKeySequence::SaveAs);
	saveAsAct->setStatusTip(tr("Save PCD to disk"));
	connect(saveAsAct, &QAction::triggered, this, &MainWindow::saveAs);
	fileMenu->addAction(saveAsAct);
	fileToolBar->addAction(saveAsAct);

	const QIcon exitIcon = QIcon::fromTheme("application-exit");
	QAction *exitAct = fileMenu->addAction(exitIcon, tr("E&xit"), this, &QWidget::close);
	exitAct->setShortcuts(QKeySequence::Quit);
	exitAct->setStatusTip(tr("Exit PCReconstruct"));

	QMenu *helpMenu = menuBar()->addMenu(tr("&Help"));
	QAction *aboutAct = helpMenu->addAction(tr("&About"), this, &MainWindow::about);
	aboutAct->setStatusTip(tr("About"));

	QAction *aboutQtAct = helpMenu->addAction(tr("About &Qt"), qApp, &QApplication::aboutQt);
	aboutQtAct->setStatusTip(tr("About Qt"));

	// Add docks
	QDockWidget *dock = new QDockWidget(tr("Log Window"), this);
	dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	logText = new QPlainTextEdit;
	logText->setReadOnly(true);
	dock->setWidget(logText);
	addDockWidget(Qt::RightDockWidgetArea, dock);
	viewMenu->addAction(dock->toggleViewAction());

	// Central widget
	msgLogger.set(logText);
	centralWidget = new Window(this, &msgLogger);
	centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
	setCentralWidget(centralWidget);
	connect(this, &MainWindow::cloudChanged, centralWidget, &Window::setCloud);
	connect(this, &MainWindow::cloudQueried, centralWidget, &Window::getCloud);

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
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *cloud) == -1) //* load the file
	{
		//PCL_ERROR ("Couldn't read PCD file\n");
		appendLogText("Couldn't read PCD file\n");
		return;
	}
	else{
		appendLogText("Opened: " + q_pcdPath + "\n");
	}

	emit cloudChanged(cloud);

}

//---------------------------------------------------------

void MainWindow::saveAs()
{
	QString q_pcdPath = QFileDialog::getSaveFileName(
	  this, tr("Open File"), "", tr("PCD (*.pcd)")
	);
	std::string pcdPath = q_pcdPath.toStdString();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	emit cloudQueried(cloud);

	if (pcl::io::savePCDFile<pcl::PointXYZ>(pcdPath, *cloud) == -1) //* save the file
	{
		//PCL_ERROR ("Couldn't save PCD file\n");
		appendLogText("Couldn't save PCD file\n");
		return;
	}
	else{
		appendLogText("Saved: " + q_pcdPath + "\n");
	}


}
//---------------------------------------------------------

void MainWindow::about()
{
   QMessageBox::about(this, tr("About"),
			tr("PCReconstruct. Copyright 2019 Piotr (Peter) Beben."));
}

//---------------------------------------------------------

void MainWindow::appendLogText(const QString& text)
{
	logText->appendPlainText(text);
	//logText->verticalScrollBar()->setValue(logText->verticalScrollBar()->maximum());
	//QCoreApplication::processEvents();
}

//---------------------------------------------------------
