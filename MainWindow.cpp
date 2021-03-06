//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.


#include "MainWindow.h"
#include "Window.h"
#include "MessageLogger.h"
#include "RandomSurfDialog.h"
#include "BoundBoxDialog.h"
#include "NormalsDialog.h"
#include "DecimateDialog.h"
#include "SparsifyDialog.h"
#include "ReconstructDialog.h"
#include "OptionsDialog.h"
#include "constants.h"

#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
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
	QMenu *editMenu = menuBar()->addMenu("&Edit");
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


	QToolBar *editToolBar = addToolBar("Edit");

	const QIcon undoIcon =
			QIcon::fromTheme("application-undo", QIcon(":/images/undo.png"));
	QAction *undoAct = new QAction(undoIcon, "&Undo", this);
	undoAct->setShortcuts(QKeySequence::Undo);
	undoAct->setStatusTip("Undo");
	connect(undoAct, &QAction::triggered, this, &MainWindow::undo);
	editMenu->addAction(undoAct);
	editToolBar->addAction(undoAct);


	viewMenu->addAction(dock->toggleViewAction());

	QAction *viewNormsAct = new QAction("&View normals", this);
	viewNormsAct->setStatusTip("View point cloud normals.");
	viewNormsAct->setCheckable(true);
	connect(viewNormsAct, &QAction::triggered, this, &MainWindow::viewGLNorms);
	viewMenu->addAction(viewNormsAct);

    QAction *randomSurfAct = new QAction("&Random surface", this);
    randomSurfAct->setStatusTip(
				"Sample a point cloud randomly from a random surface");
    connect(randomSurfAct, &QAction::triggered, this, &MainWindow::setRandom);
    toolsMenu->addAction(randomSurfAct);

    QAction *boundBoxAct = new QAction("&Set bounding box", this);
    boundBoxAct->setStatusTip(
                "Set bounding box, inside which operations are performed");
    connect(boundBoxAct, &QAction::triggered, this, &MainWindow::setBBox);
    toolsMenu->addAction(boundBoxAct);

	QAction *normalsAct = new QAction("&Approx. Normals", this);
	normalsAct->setStatusTip(
				"Approximate cloud surface normals");
	connect(normalsAct, &QAction::triggered, this, &MainWindow::approxNorms);
	toolsMenu->addAction(normalsAct);

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

	connect(this, &MainWindow::cloudUndo,
			centralWidget, &Window::undoCloud);

	connect(this, &MainWindow::cloudNormsViewGL,
			centralWidget, &Window::viewGLCloudNorms);

	connect(this, &MainWindow::cloudSetRandom,
			centralWidget, &Window::setRandomCloud);

    connect(this, &MainWindow::cloudSetBBox,
            centralWidget, &Window::setCloudBBox);

	connect(this, &MainWindow::cloudApproxNorms,
			centralWidget, &Window::approxCloudNorms);

	connect(this, &MainWindow::cloudDecimate,
			centralWidget, &Window::decimateCloud);

	connect(this, &MainWindow::cloudSparsify,
			centralWidget, &Window::sparsifyCloud);

	connect(this, &MainWindow::cloudReconstruct,
			centralWidget, &Window::reconstructCloud);

    connect(this, &MainWindow::pointSizeChanged,
            centralWidget, &Window::setPointSize);

	connect(this, &MainWindow::normScaleChanged,
			centralWidget, &Window::setNormScale);

	connect(centralWidget, &Window::bBoxFieldsChanged,
			this, &MainWindow::changeBBoxFields);


	//------
	// Dialogs

    randomSurfDialog = new RandomSurfDialog(this);
    boundBoxDialog = new BoundBoxDialog(this);
	normalsDialog = new NormalsDialog(this);
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

//	QString q_path = QFileDialog::getOpenFileName(
//				this, "Open File", "", "(*.pcd *.ply)"
//				);

	QString q_path = QFileDialog::getOpenFileName(
				this, "Open File", "", "PCD (*.pcd)"
				);

	std::string path = q_path.toStdString();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	int success = 0;
	try{
//		if( q_path.endsWith("pcd", Qt::CaseInsensitive) ){
		success = pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud);
//		}
//		else if( q_path.endsWith("ply", Qt::CaseInsensitive) ){
//			success = pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud);
//			if( success == -1 ){
//				success = pcl::io::loadPolygonFilePLY<pcl::PointXYZ>(path, *cloud);
//			}
//		}
//		else{
//			success = -1;
//		}

	}
	catch (...){
		success = -1;
	}

	if (success == -1) {
		appendLogText("Couldn't read PCD file\n");
		return;
	}
	else{
		appendLogText("Opened: " + q_path + "\n");
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

void MainWindow::undo()
{
	emit cloudUndo();
}

//---------------------------------------------------------

void MainWindow::viewGLNorms(bool enabled)
{
	emit cloudNormsViewGL(enabled);
}

//---------------------------------------------------------

void MainWindow::setRandom()
{
	// Show the dialog as modal
    if(randomSurfDialog->exec() == QDialog::Accepted){
		size_t nPoints;
        bool ok = randomSurfDialog->getFields(nPoints);
		if(!ok){
			badInputMessageBox("All fields should be integers bigger than zero.");
			return;
		}
        emit cloudSetRandom(nPoints);
	}

}


//---------------------------------------------------------

void MainWindow::setBBox()
{
    // Show the dialog as modal
    if(boundBoxDialog->exec() == QDialog::Accepted){
        float minBBox[3], maxBBox[3];
        bool ok = boundBoxDialog->getFields(minBBox, maxBBox);
        if(!ok){
            badInputMessageBox(
                        QString("All fields should be bigger than zero,\n") +
                        QString("and min. extents less than max. extents."));
            return;
        }
        emit cloudSetBBox(minBBox, maxBBox);
    }

}


//---------------------------------------------------------

void MainWindow::approxNorms()
{
	// Show the dialog as modal
	if(normalsDialog->exec() == QDialog::Accepted){
		int nIters;
		size_t kNN;
		bool ok = normalsDialog->getFields(nIters, kNN);
		if(!ok){
			badInputMessageBox(
						QString("All fields should be bigger than zero."));
			return;
		}
		emit cloudApproxNorms(nIters, kNN);
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
		bool looseBBox;
		SparseApprox method;

		int ok = reconstructDialog->getFields(
                    kSVDIters, kNN, nfreq, densify, natm, latm,
					maxNewPoints, looseBBox, method);
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
						maxNewPoints, looseBBox, method);
		}
	}

}


//---------------------------------------------------------

void MainWindow::options()
{
    // Show the dialog as modal
    if(optionsDialog->exec() == QDialog::Accepted){
		float pointSize, normScale;
		bool ok = optionsDialog->getFields(pointSize, normScale);
        if(!ok){
			badInputMessageBox("All field should be greater than zero.");
            return;
        }
        emit pointSizeChanged(pointSize);
		emit normScaleChanged(normScale);
	}

}

//---------------------------------------------------------

void MainWindow::about()
{
   QMessageBox::about(this, "About",
					  QString("PCReconstruct.\n") +
					  QString("Copyright 2019 Piotr (Peter) Beben.\n") +
					  QString("pdbcas@gmail.com\n"));
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
void MainWindow::changeBBoxFields(float minBBox[3], float maxBBox[3])
{
	boundBoxDialog->setFields(minBBox, maxBBox);
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
