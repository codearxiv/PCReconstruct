//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef BOUNDBOXDIALOG_H
#define BOUNDBOXDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QDoubleValidator;
QT_END_NAMESPACE

class BoundBoxDialog : public QDialog
{
    Q_OBJECT

public:
    BoundBoxDialog(QWidget *parent = nullptr);
	bool getFields(float minBBox[], float maxBBox[]) const;
	void setFields(const float minBBox[3], const float maxBBox[3]);

private:
    QFormLayout *form;
    QLineEdit* minLineEdit[3];
    QLineEdit* maxLineEdit[3];
    QDialogButtonBox *buttonBox;
    QDoubleValidator *validator;

};


#endif // BOUNDBOXDIALOG_H
