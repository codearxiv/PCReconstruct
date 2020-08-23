//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "DecimateDialog.h"
#include "constants.h"

#include <QMainWindow>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QObject>
#include <QWidget>
#include <QIntValidator>


DecimateDialog::DecimateDialog(QWidget *parent) : QDialog(parent)
{
	validator = new QIntValidator(1, int_infinity, this);

	form = new QFormLayout(this);
	form->addRow(new QLabel("Create random holes within bounding box"));
	nHolesLineEdit = new QLineEdit(this);
	nHolesLineEdit->setValidator(validator);
	nHolesLineEdit->setText("50");
	form->addRow(QString("Number of holes:"), nHolesLineEdit);
	nPointsLineEdit = new QLineEdit(this);
	nPointsLineEdit->setValidator(validator);
	nPointsLineEdit->setText("100");
	form->addRow(QString("Number of points per hole:"), nPointsLineEdit);

	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}



bool DecimateDialog::getFields(size_t& nHoles, size_t& kNN) const
{
	QString nHolesStr = nHolesLineEdit->text();
	int pos = 0;
	if(validator->validate(nHolesStr, pos) != QValidator::Acceptable){
		nHolesLineEdit->clear();
		return false;
	}
	else{
		bool ok;
		nHoles = nHolesStr.toULongLong(&ok);
		if(!ok) {
			nHolesLineEdit->clear();
			return false;
		}
	}

	QString kNNStr = nPointsLineEdit->text();
	pos = 0;
	if(validator->validate(kNNStr, pos) != QValidator::Acceptable){
		nPointsLineEdit->clear();
		return false;
	}
	else{
		bool ok;
		kNN = kNNStr.toULongLong(&ok);
		if(!ok) {
			nPointsLineEdit->clear();
			return false;
		}
	}


	return true;
}




