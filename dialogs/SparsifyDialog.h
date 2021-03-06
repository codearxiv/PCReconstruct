//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef SPARSIFYDIALOG_H
#define SPARSIFYDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QDoubleValidator;
QT_END_NAMESPACE

class SparsifyDialog : public QDialog
{
	Q_OBJECT

public:
	SparsifyDialog(QWidget *parent = nullptr);
	bool getFields(float& percent) const;

private:
	QFormLayout *form;
	QLineEdit *percentLineEdit;
	QDialogButtonBox *buttonBox;
	QDoubleValidator *validator;

};

#endif // SPARSIFYDIALOG_H
