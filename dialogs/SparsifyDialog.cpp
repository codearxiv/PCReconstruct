//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "SparsifyDialog.h"
#include "get_field.h"
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
	validator = new QDoubleValidator(0.0, 100.0, 5, this);

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
	bool ok = get_float_field(percentLineEdit, validator, percent);
	return ok;
}

