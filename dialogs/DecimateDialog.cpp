//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "DecimateDialog.h"
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
	bool ok;

	ok = get_integer_field(nHolesLineEdit, validator, nHoles);
	if(!ok) return false;

	ok = get_integer_field(nPointsLineEdit, validator, kNN);
	if(!ok) return false;

	return true;
}




