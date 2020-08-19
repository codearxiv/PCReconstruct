//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.


#include "BoundBoxDialog.h"
#include "constants.h"

#include <QMainWindow>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QDialogButtonBox>
#include <QObject>
#include <QWidget>
#include <QIntValidator>

BoundBoxDialog::BoundBoxDialog(QWidget *parent) : QDialog(parent)
{
    validator = new QDoubleValidator(-double_infinity, double_infinity, 8, this);

    form = new QFormLayout(this);
    form->addRow(new QLabel(
                     "Set bounding box to perform operations within"));
    for(int i=0; i < 3; ++i){
        minLineEdit[i] = new QLineEdit(this);
        minLineEdit[i]->setValidator(validator);
    }

    form->addRow(QString("Minimum X coordinate:"), minLineEdit[0]);
    form->addRow(QString("Minimum Y coordinate:"), minLineEdit[1]);
    form->addRow(QString("Minimum Z coordinate:"), minLineEdit[2]);
    form->addRow(QString("Maximum X coordinate:"), maxLineEdit[0]);
    form->addRow(QString("Maximum Y coordinate:"), maxLineEdit[1]);
    form->addRow(QString("Maximum Z coordinate:"), maxLineEdit[2]);

    buttonBox = new QDialogButtonBox(
                QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                Qt::Horizontal, this);

    form->addRow(buttonBox);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}


bool BoundBoxDialog::getFields(float minBBox[], float maxBBox[])
{
    for(int i=0; i < 3; ++i){
        QString minStr = minLineEdit[i]->text();
        int pos = 0;
        if(validator->validate(minStr, pos) != QValidator::Acceptable){
            minLineEdit[i]->clear();
            return false;
        }
        else{
            bool ok;
            minBBox[i] = minStr.toFloat(&ok);
            if(!ok) {
                minLineEdit[i]->clear();
                return false;
            }
        }

        QString maxStr = maxLineEdit[i]->text();
        pos = 0;
        if(validator->validate(maxStr, pos) != QValidator::Acceptable){
            maxLineEdit[i]->clear();
            return false;
        }
        else{
            bool ok;
            maxBBox[i] = maxStr.toFloat(&ok);
            if( maxBBox[i] <= minBBox[i] ) ok = false;
            if(!ok) {
                maxLineEdit[i]->clear();
                return false;
            }
        }

    }

    return true;
}
