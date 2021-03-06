//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef RECONSTRUCTDIALOG_H
#define RECONSTRUCTDIALOG_H

#include "constants.h"

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QIntValidator;
class QDoubleValidator;
class QComboBox;
QT_END_NAMESPACE

class ReconstructDialog : public QDialog
{
	Q_OBJECT

public:
	ReconstructDialog(QWidget *parent = nullptr);
    int getFields(
            int& kSVDIters, size_t& kNN, size_t& nfreq, float& densify,
			size_t& natm, size_t& latm, size_t& maxNewPoints, bool& looseBBox,
			SparseApprox& method) const;

public slots:
	void bBoxComboChanged(int idx) { m_bBoxComboIdx = idx; }
	void methodComboChanged(int idx) { m_methodComboIdx = idx; }

private:
	QFormLayout *form;
	QLineEdit *nItersLineEdit;
	QLineEdit *kNNLineEdit;
	QLineEdit *nFreqLineEdit;
    QLineEdit *densifyLineEdit;
    QLineEdit *nAtmLineEdit;
	QLineEdit *lAtmLineEdit;
	QLineEdit *maxNewLineEdit;
	QComboBox *bBoxComboBox;
	int m_bBoxComboIdx = 0;
	QComboBox *methodComboBox;
	int m_methodComboIdx = 0;

	QDialogButtonBox *buttonBox;
    QIntValidator *intValidator;
    QDoubleValidator *doubleValidator;

};

#endif // RECONSTRUCTDIALOG_H
