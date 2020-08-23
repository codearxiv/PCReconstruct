//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "OptionsDialog.h"
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
    validator = new QDoubleValidator(0.0, double_infinity, 2, this);

    form = new QFormLayout(this);
    form->addRow(new QLabel("Change app settings"));
    pointSizeLineEdit = new QLineEdit(this);
    pointSizeLineEdit->setValidator(validator);
	pointSizeLineEdit->setText("5.0");
    form->addRow(QString("Display point size:"), pointSizeLineEdit);

    buttonBox = new QDialogButtonBox(
                QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                Qt::Horizontal, this);

    form->addRow(buttonBox);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}


bool OptionsDialog::getFields(float& percent) const
{

    QString nPointsStr = pointSizeLineEdit->text();
    int pos = 0;
    if(validator->validate(nPointsStr, pos) != QValidator::Acceptable){
        pointSizeLineEdit->clear();
        return false;
    }
    else{
        bool ok;
        percent = nPointsStr.toFloat(&ok);
        if(!ok) {
            pointSizeLineEdit->clear();
            return false;
        }
    }

    return true;
}

