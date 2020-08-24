//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef NORMALSDIALOG_H
#define NORMALSDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QIntValidator;
QT_END_NAMESPACE

class NormalsDialog : public QDialog
{
	Q_OBJECT

public:
	NormalsDialog(QWidget *parent = nullptr);
	bool getFields(int& nIters, size_t& kNN) const;

private:
	QFormLayout *form;
	QLineEdit *nItersLineEdit;
	QLineEdit *kNNLineEdit;
	QDialogButtonBox *buttonBox;
	QIntValidator *validator;

};


#endif // NORMALSDIALOG_H
