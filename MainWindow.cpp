//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#include "MainWindow.h"
#include "Window.h"
#include "MessageLogger.h"
#include "SetRandomDialog.h"
#include "DecimateDialog.h"
#include "SparsifyDialog.h"
#include "ReconstructDialog.h"
#include "OptionsDialog.h"
#include "constants.h"

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
	QDockWidget *dock = new QDockWidget("Log Window", this);
	dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	logText = new QPlainTextEdit;
	logText->setReadOnly(true);
	dock->setWidget(logText);
	addDockWidget(Qt::RightDockWidgetArea, dock);

	//------
	// Add actions
	QMenu *fileMenu = menuBar()->addMenu("&File");
	QMenu *viewMenu = menuBar()->addMenu("&View");
	QMenu *toolsMenu = menuBar()->addMenu("&Tools");
	QMenu *helpMenu = menuBar()->addMenu("&Help");

	QToolBar *fileToolBar = addToolBar("File");

	const QIcon openIcon =
			QIcon::fromTheme("document-open", QIcon(":/images/open.png"));
	QAction *openAct = new QAction(openIcon, "&Open...", this);
	openAct->setShortcuts(QKeySequence::Open);
	openAct->setStatusTip("Open an existing PCD file");
	connect(openAct, &QAction::triggered, this, &MainWindow::open);
	fileMenu->addAction(openAct);
	fileToolBar->addAction(openAct);

	const QIcon saveAsIcon =
			QIcon::fromTheme("document-save-as", QIcon(":/images/save.png"));
	QAction *saveAsAct = new QAction(saveAsIcon, "Save &As...", this);
	saveAsAct->setShortcuts(QKeySequence::SaveAs);
	saveAsAct->setStatusTip("Save PCD to disk");
	connect(saveAsAct, &QAction::triggered, this, &MainWindow::saveAs);
	fileMenu->addAction(saveAsAct);
	fileToolBar->addAction(saveAsAct);

	const QIcon exitIcon = QIcon::fromTheme("application-exit");
	QAction *exitAct =
			fileMenu->addAction(exitIcon, "E&xit", this, &QWidget::close);
	exitAct->setShortcuts(QKeySequence::Quit);
	exitAct->setStatusTip("Exit PCReconstruct");

	viewMenu->addAction(dock->toggleViewAction());

	QAction *setRandomAct = new QAction("&Set Random", this);
	setRandomAct->setStatusTip(
				"Sample a point cloud randomly from a random surface");
	connect(setRandomAct, &QAction::triggered, this, &MainWindow::setRandom);
	toolsMenu->addAction(setRandomAct);

	QAction *decimateAct = new QAction("&Decimate", this);
	decimateAct->setStatusTip("Generate random holes in point cloud");
	connect(decimateAct, &QAction::triggered, this, &MainWindow::decimate);
	toolsMenu->addAction(decimateAct);

	QAction *sparsifyAct = new QAction("&Sparsify", this);
	sparsifyAct->setStatusTip("Take a random subset of the point cloud");
	connect(sparsifyAct, &QAction::triggered, this, &MainWindow::sparsify);
	toolsMenu->addAction(sparsifyAct);

	QAction *reconstructAct = new QAction("&Reconstruct", this);
	reconstructAct->setStatusTip("Reconstruct point cloud");
	connect(reconstructAct, &QAction::triggered, this, &MainWindow::reconstruct);
	toolsMenu->addAction(reconstructAct);

    toolsMenu->addSeparator();

    QAction *optionsAct = new QAction("&Options", this);
    optionsAct->setStatusTip("Change app settings.");
    connect(optionsAct, &QAction::triggered, this, &MainWindow::options);
    toolsMenu->addAction(optionsAct);


	QAction *aboutAct =
			helpMenu->addAction("&About", this, &MainWindow::about);
	aboutAct->setStatusTip("About");

	QAction *aboutQtAct =
			helpMenu->addAction("About &Qt", qApp, &QApplication::aboutQt);
	aboutQtAct->setStatusTip("About Qt");


	//------
	// Central widget

	msgLogger = new MessageLogger(logText);
	centralWidget = new Window(this, msgLogger);
	centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
	setCentralWidget(centralWidget);

	connect(msgLogger, &MessageLogger::logTextAppend,
			this, &MainWindow::appendLogText);

	connect(msgLogger, &MessageLogger::logTextInsert,
			this, &MainWindow::insertLogText);

	connect(this, &MainWindow::cloudChanged,
			centralWidget, &Window::setCloud);

	connect(this, &MainWindow::cloudQueried,
			centralWidget, &Window::getCloud);

	connect(this, &MainWindow::cloudSetRandom,
			centralWidget, &Window::setRandomCloud);

	connect(this, &MainWindow::cloudDecimate,
			centralWidget, &Window::decimateCloud);

	connect(this, &MainWindow::cloudSparsify,
			centralWidget, &Window::sparsifyCloud);

	connect(this, &MainWindow::cloudReconstruct,
			centralWidget, &Window::reconstructCloud);

    connect(this, &MainWindow::pointSizeChanged,
            centralWidget, &Window::setPointSize);


	//------
	// Dialogs

	setRandomDialog = new SetRandomDialog(this);
	decimateDialog = new DecimateDialog(this);
	sparsifyDialog = new SparsifyDialog(this);
	reconstructDialog = new ReconstructDialog(this);
    optionsDialog = new OptionsDialog(this);


	//------

	QMetaObject::connectSlotsByName(this);

}

//---------------------------------------------------------

void MainWindow::open()
{

	QString q_pcdPath = QFileDialog::getOpenFileName(
				this, "Open File", "", "PCD (*.pcd)"
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

    emit cloudChanged(cloud);

}

//---------------------------------------------------------

void MainWindow::saveAs()
{
	QString q_pcdPath = QFileDialog::getSaveFileName(
	  this, "Open File", "", "PCD (*.pcd)"
	);
	std::string pcdPath = q_pcdPath.toStdString();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    emit cloudQueried(cloud);

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
        emit cloudSetRandom(nPoints);
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
        emit cloudDecimate(nHoles,kNN);
	}

}

//---------------------------------------------------------

void MainWindow::sparsify()
{
	// Show the dialog as modal
	if(sparsifyDialog->exec() == QDialog::Accepted){
		float percent;
		bool ok = sparsifyDialog->getFields(percent);
		if(!ok){
			badInputMessageBox("Percent field should be between 0 and 100.");
			return;
		}
        emit cloudSparsify(percent);
	}

}


//---------------------------------------------------------

void MainWindow::reconstruct()
{
	// Show the dialog as modal
	if(reconstructDialog->exec() == QDialog::Accepted){
		int kSVDIters;
		size_t kNN, nfreq, natm, latm, maxNewPoints;
        float densify;
        SparseApprox method;

		int ok = reconstructDialog->getFields(
                    kSVDIters, kNN, nfreq, densify, natm, latm,
					maxNewPoints, method);
		switch( ok ){
		case -1:
			badInputMessageBox(
						"Number of iterations field must be bigger than zero.");
			break;
		case -2:
			badInputMessageBox(
						"Patch size field must be bigger than zero.");
			break;
		case -3:
			badInputMessageBox(
                        "Frequency field must be bigger than zero.");
			break;
        case -4:
            badInputMessageBox(
                        "Densification field must be bigger than zero.");
            break;
        case -5:
			badInputMessageBox(
						"Number of atoms field must be bigger than zero.");
			break;
        case -6:
			badInputMessageBox(
						QString("Sparsity constraint must be bigger than zero, ") +
						QString("and no bigger than the number of atoms field.")
						);
			break;
        case -7:
			badInputMessageBox(
						"Max. new points field should be bigger than zero.");
			break;
		default:
            emit cloudReconstruct(
                        kSVDIters, kNN, nfreq, densify, natm, latm,
                        maxNewPoints, method);
		}
	}

}


//---------------------------------------------------------

void MainWindow::options()
{
    // Show the dialog as modal
    if(optionsDialog->exec() == QDialog::Accepted){
        float pointSize;
        bool ok = optionsDialog->getFields(pointSize);
        if(!ok){
            badInputMessageBox("point size field should be greater than zero.");
            return;
        }
        emit pointSizeChanged(pointSize);
    }

}

//---------------------------------------------------------

void MainWindow::about()
{
   QMessageBox::about(this, "About",
			"PCReconstruct. Copyright 2019 Piotr (Peter) Beben.");
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

void MainWindow::insertLogText(const QString& text)
{
	logText->undo();
	logText->appendPlainText(text);
	logText->verticalScrollBar()->setValue(
				logText->verticalScrollBar()->maximum());
}

//---------------------------------------------------------
