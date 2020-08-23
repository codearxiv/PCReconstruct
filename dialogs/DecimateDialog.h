//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef DECIMATEDIALOG_H
#define DECIMATEDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QIntValidator;
QT_END_NAMESPACE

class DecimateDialog : public QDialog
{
	Q_OBJECT

public:
	DecimateDialog(QWidget *parent = nullptr);
	bool getFields(size_t& nHoles, size_t& kNN) const;

private:
	QFormLayout *form;
	QLineEdit *nHolesLineEdit;
	QLineEdit *nPointsLineEdit;
	QDialogButtonBox *buttonBox;
	QIntValidator *validator;

};


#endif // DECIMATEDIALOG_H
