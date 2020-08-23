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

	QString label[3] = {"X","Y","Z"};

	form = new QFormLayout(this);
    form->addRow(new QLabel(
                     "Set bounding box to perform operations within"));
    for(int i=0; i < 3; ++i){
        minLineEdit[i] = new QLineEdit(this);
        minLineEdit[i]->setValidator(validator);
		minLineEdit[i]->setToolTip(
					QString("The smallest ") + label[i]	+
					QString(" coordinate in bounding box."));
		form->addRow(QString("Minimum ") + label[i] + QString(" coordinate:"),
					 minLineEdit[i]);
		maxLineEdit[i] = new QLineEdit(this);
		maxLineEdit[i]->setValidator(validator);
		maxLineEdit[i]->setToolTip(
					QString("The largest ") + label[i] +
					QString(" coordinate in bounding box."));
		form->addRow(QString("Maximum ") + label[i] + QString(" coordinate:"),
					 maxLineEdit[i]);
	}


    buttonBox = new QDialogButtonBox(
                QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                Qt::Horizontal, this);

    form->addRow(buttonBox);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

//-----------------------------------------------------------
bool BoundBoxDialog::getFields(float minBBox[], float maxBBox[]) const
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
//-----------------------------------------------------------
void BoundBoxDialog::setFields(const float minBBox[3], const float maxBBox[3])
{
	for(int i=0; i < 3; ++i){
		minLineEdit[i]->setText(QString::number(minBBox[i]));
		maxLineEdit[i]->setText(QString::number(maxBBox[i]));
	}

}

//-----------------------------------------------------------
