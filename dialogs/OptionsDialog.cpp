//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "OptionsDialog.h"
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

OptionsDialog::OptionsDialog(QWidget *parent) : QDialog(parent)
{
	validator = new QDoubleValidator(0.0, double_infinity, 5, this);

    form = new QFormLayout(this);
    form->addRow(new QLabel("Change app settings"));

	pointSizeLineEdit = new QLineEdit(this);
    pointSizeLineEdit->setValidator(validator);
	pointSizeLineEdit->setText("5.0");
    form->addRow(QString("Display point size:"), pointSizeLineEdit);

	normScaleLineEdit = new QLineEdit(this);
	normScaleLineEdit->setValidator(validator);
	normScaleLineEdit->setText("0.01");
	form->addRow(QString("Display normals size:"), normScaleLineEdit);

    buttonBox = new QDialogButtonBox(
                QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                Qt::Horizontal, this);

    form->addRow(buttonBox);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}


bool OptionsDialog::getFields(float& pointSize, float& normScale) const
{

	bool ok;

	ok = get_float_field(pointSizeLineEdit, validator, pointSize);
	if(!ok) return false;

	ok = get_float_field(normScaleLineEdit, validator, normScale);
	if(!ok) return false;

    return true;
}

