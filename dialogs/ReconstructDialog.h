//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef RECONSTRUCTDIALOG_H
#define RECONSTRUCTDIALOG_H

#include "constants.h"

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QMainWindow;
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QIntValidator;
class QComboBox;
QT_END_NAMESPACE

class ReconstructDialog : public QDialog
{
	Q_OBJECT

public:
	ReconstructDialog(QWidget *parent = nullptr);
	int getFields(int& kSVDIters, size_t& kNN, size_t& nfreq,
				  size_t& natm, size_t& latm, size_t& maxNewPoints,
				  SparseApprox& method);

public slots:
	void reconstructCloud(int kSVDIters, size_t kNN, size_t nfreq,
						  size_t natm, size_t latm, size_t maxNewPoints,
						  SparseApprox method)
	{
		emit cloudReconstruct(kSVDIters, kNN, nfreq, natm, latm,
							  maxNewPoints, method);
	}

signals:
	void cloudReconstruct(
			int kSVDIters, size_t kNN, size_t nfreq,
			size_t natm, size_t latm, size_t maxNewPoints,
			SparseApprox method);

private:
	QFormLayout *form;
	QLineEdit *nItersLineEdit;
	QLineEdit *kNNLineEdit;
	QLineEdit *nFreqLineEdit;
	QLineEdit *nAtmLineEdit;
	QLineEdit *lAtmLineEdit;
	QLineEdit *maxNewLineEdit;
	QComboBox *methodComboBox;

	QDialogButtonBox *buttonBox;
	QIntValidator *validator;
	QMainWindow *mainWindow;

};

#endif // RECONSTRUCTDIALOG_H
