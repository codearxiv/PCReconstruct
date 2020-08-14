//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "DecimateDialog.h"

#include <QMainWindow>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QObject>
#include <QWidget>


DecimateDialog::DecimateDialog(QWidget *parent) : QDialog(parent)
{
	form = new QFormLayout(this);
	form->addRow(new QLabel("Create random holes in point cloud"));
	lineEditNumHoles = new QLineEdit(this);
	form->addRow(QString("Number of holes:"), lineEditNumHoles);
	lineEditNumPoints = new QLineEdit(this);
	form->addRow(QString("Number of points per hole:"), lineEditNumPoints);

	//QList<QLineEdit*> fields;
	//fields << lineEditHoles << lineEditNumPoints;

	// Add some standard buttons (Cancel/Ok) at the bottom of the dialog
	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

	// Show the dialog as modal
//	if (dialog.exec() == QDialog::Accepted) {
//		// If the user didn't dismiss the dialog, do something with the fields
//		foreach(QLineEdit* lineEdit, fields) {
//			qDebug() << lineEdit->text();
//		}
//	}
}
