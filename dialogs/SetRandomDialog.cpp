//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "SetRandomDialog.h"
#include "constants.h"

#include <QMainWindow>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QObject>
#include <QWidget>
#include <QIntValidator>


SetRandomDialog::SetRandomDialog(QWidget *parent) : QDialog(parent)
{
	validator = new QIntValidator(1, int_infinity, this);

	form = new QFormLayout(this);
	form->addRow(new QLabel(
					 "Create a point cloud sampled randomly from a random surface"));
	nPointsLineEdit = new QLineEdit(this);
	nPointsLineEdit->setValidator(validator);
	nPointsLineEdit->setText("1000");
	form->addRow(QString("Number of points sampled:"), nPointsLineEdit);

	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}


bool SetRandomDialog::getFields(size_t& nPoints)
{

	QString nPointsStr = nPointsLineEdit->text();
	int pos = 0;
	if(validator->validate(nPointsStr, pos) != QValidator::Acceptable){
		nPointsLineEdit->clear();
		return false;
	}
	else{
		bool ok;
		nPoints = nPointsStr.toULongLong(&ok);
		if(!ok) {
			nPointsLineEdit->clear();
			return false;
		}
	}

	return true;
}


