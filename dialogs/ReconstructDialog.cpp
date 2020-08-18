//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "ReconstructDialog.h"
#include "constants.h"

#include <QMainWindow>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QObject>
#include <QWidget>
#include <QIntValidator>


ReconstructDialog::ReconstructDialog(QWidget *parent) : QDialog(parent)
{
	validator = new QIntValidator(1, int_infinity, this);

	form = new QFormLayout(this);

	form->addRow(new QLabel("Reconstruct point cloud within bounding box"));
	nItersLineEdit = new QLineEdit(this);
	nItersLineEdit->setValidator(validator);
	nItersLineEdit->setText("10");
	form->addRow(QString("Number of dictionary learning iterations:"), nItersLineEdit);

	form->addRow(new QLabel("Reconstruct point cloud within bounding box"));
	kNNLineEdit = new QLineEdit(this);
	kNNLineEdit->setValidator(validator);
	kNNLineEdit->setText("10");
	form->addRow(QString("Number of dictionary learning iterations:"), nItersLineEdit);


	methodComboBox = new QComboBox;
	methodComboBox->addItem(tr("Matching Pursuit"));
	methodComboBox->addItem(tr("Orthogonal Pursuit"));
	form->addRow(QString("Sparse approximation method:"), methodComboBox);

	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}


int ReconstructDialog::getFields(
		int& kSVDIters, size_t& kNN, size_t& nfreq,
		size_t& natm, size_t& latm, size_t& maxNewPoints,
		SparseApprox& method)
{

	auto good = QValidator::Acceptable;
	bool ok = true;

	int pos = 0;
	QString kSVDItersStr = nItersLineEdit->text();
	if(validator->validate(kSVDItersStr, pos) != good){ ok = false; }
	else{ kSVDIters = kSVDItersStr.toULongLong(&ok); }

	if(!ok) {
		nItersLineEdit->clear();
		return -1;
	}

	pos = 0;
	QString kNNStr = kNNLineEdit->text();
	if(validator->validate(kNNStr, pos) != good){ ok = false; }
	else{ kNN = kNNStr.toULongLong(&ok); }

	if(!ok) {
		kNNLineEdit->clear();
		return -2;
	}

	pos = 0;
	QString nfreqStr = nFreqLineEdit->text();
	if(validator->validate(nfreqStr, pos) != good){ ok = false; }
	else{ nfreq = nfreqStr.toULongLong(&ok); }

	if(!ok) {
		nFreqLineEdit->clear();
		return -3;
	}

	pos = 0;
	QString natmStr = nAtmLineEdit->text();
	if(validator->validate(natmStr, pos) != good){ ok = false; }
	else{ natm = natmStr.toULongLong(&ok); }

	if(!ok) {
		nAtmLineEdit->clear();
		return -4;
	}

	pos = 0;
	QString latmStr = lAtmLineEdit->text();
	if(validator->validate(latmStr, pos) != good){ ok = false; }
	else{
		latm = latmStr.toULongLong(&ok);
		if(latm > natm) ok = false;
	}

	if(!ok) {
		lAtmLineEdit->clear();
		return -5;
	}

	pos = 0;
	QString maxNewStr = maxNewLineEdit->text();
	if(validator->validate(maxNewStr, pos) != good){ ok = false; }
	else{ maxNewPoints = maxNewStr.toULongLong(&ok); }

	if(!ok) {
		maxNewLineEdit->clear();
		return -6;
	}

	int index = 0;
	methodComboBox->activated(index);
	switch(index){
	case 0:
		method = SparseApprox::MatchingPursuit;
		break;
	case 1:
		method = SparseApprox::OrthogonalPursuit;
		break;
	}

	return 0;
}
