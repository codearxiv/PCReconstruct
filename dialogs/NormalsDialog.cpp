//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "NormalsDialog.h"
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


NormalsDialog::NormalsDialog(QWidget *parent) : QDialog(parent)
{
	validator = new QIntValidator(1, int_infinity, this);

	form = new QFormLayout(this);
	form->addRow(new QLabel("Approximate cloud normals"));
	nItersLineEdit = new QLineEdit(this);
	nItersLineEdit->setValidator(validator);
	nItersLineEdit->setText("25");
	form->addRow(QString("Number of iterations:"), nItersLineEdit);
	kNNLineEdit = new QLineEdit(this);
	kNNLineEdit->setValidator(validator);
	kNNLineEdit->setText("25");
	form->addRow(QString("Number of neighbouring points used:"), kNNLineEdit);

	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}



bool NormalsDialog::getFields(int& nIters, size_t& kNN) const
{
	bool ok;
	size_t temp;

	ok = get_integer_field(nItersLineEdit, validator, temp);
	if(!ok) return false;
	nIters = int(temp);

	ok = get_integer_field(kNNLineEdit, validator, kNN);
	if(!ok) return false;

	return true;
}

