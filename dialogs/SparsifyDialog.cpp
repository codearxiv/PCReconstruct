//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "SparsifyDialog.h"
#include "constants.h"

#include <QMainWindow>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QObject>
#include <QWidget>
#include <QIntValidator>


SparsifyDialog::SparsifyDialog(QWidget *parent) : QDialog(parent)
{
	validator = new QDoubleValidator(0.0, 100.0, 2, this);

	form = new QFormLayout(this);
	form->addRow(new QLabel(
					 "Take a random subset of point cloud within bounding box"));
	percentLineEdit = new QLineEdit(this);
	percentLineEdit->setValidator(validator);
	percentLineEdit->setText("7.0");
	form->addRow(QString("Percentage of points to keep:"), percentLineEdit);

	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}


bool SparsifyDialog::getFields(float& percent) const
{

	QString nPointsStr = percentLineEdit->text();
	int pos = 0;
	if(validator->validate(nPointsStr, pos) != QValidator::Acceptable){
		percentLineEdit->clear();
		return false;
	}
	else{
		bool ok;
		percent = nPointsStr.toFloat(&ok);
		if(!ok) {
			percentLineEdit->clear();
			return false;
		}
	}

	return true;
}

