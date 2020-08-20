//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#include "ReconstructDialog.h"
#include "constants.h"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QObject>
#include <QWidget>
#include <QIntValidator>
#include <QDoubleValidator>


ReconstructDialog::ReconstructDialog(QWidget *parent) : QDialog(parent)
{
    intValidator = new QIntValidator(1, int_infinity, this);
    doubleValidator = new QDoubleValidator(0.0, double_infinity, 2, this);

	form = new QFormLayout(this);
	form->addRow(new QLabel("Fill in surface gaps in current point cloud"));

	nItersLineEdit = new QLineEdit(this);
    nItersLineEdit->setValidator(intValidator);
	nItersLineEdit->setText("15");
	nItersLineEdit->setToolTip(
				QString("This sets the number of training iterations within which\n") +
				QString("the various local surface patterns in the point cloud\n") +
				QString("are learned, and used in the cloud's reconstruction."));
	form->addRow(QString("Number of dictionary learning iterations:"), nItersLineEdit);

	kNNLineEdit = new QLineEdit(this);
    kNNLineEdit->setValidator(intValidator);
	kNNLineEdit->setText("50");
	kNNLineEdit->setToolTip(
				QString("The cloud surface is reconstructed patch-by-patch.\n") +
				QString("This sets the maximum number of points in a patch.\n\n") +
				QString("The larger gaps in the cloud are relative to density\n") +
				QString("of point sampling, the larger this field should be.\n") +
				QString("Expect crazy results otherwise!"));
	form->addRow(QString("local patch size:"), kNNLineEdit);

	nFreqLineEdit = new QLineEdit(this);
    nFreqLineEdit->setValidator(intValidator);
	nFreqLineEdit->setText("4");
	nFreqLineEdit->setToolTip(
				QString("Each local patch has a measure of complexity given by the\n") +
				QString("surface bumpiness along an axis. This sets the maximum\n") +
				QString("number of bumps along an axis that can be expected for\n") +
				QString("the given patch size.\n\n") +
				QString("Note training time and memory footprint will degrade\n") +
				QString("quadratically as this value increases."));
	form->addRow(QString("Maximum frequency in a patch:"), nFreqLineEdit);

    densifyLineEdit = new QLineEdit(this);
    densifyLineEdit->setValidator(doubleValidator);
    densifyLineEdit->setText("1.0");
    densifyLineEdit->setToolTip(
				QString("This controls the density of the reconstruction in\n") +
				QString("each region compared to the density of the nearby\n") +
				QString("points in the original cloud."));
    form->addRow(QString("Densification factor:"), densifyLineEdit);

	nAtmLineEdit = new QLineEdit(this);
    nAtmLineEdit->setValidator(intValidator);
	nAtmLineEdit->setText("10");
	nAtmLineEdit->setToolTip(
				QString("Total number of dictionary atoms available.\n\n") +
				QString("A too large value leads to overfitting, and too small\n") +
				QString("leads to underfitting, depending on max. frequency."));
	form->addRow(QString("Number of dictionary atoms:"), nAtmLineEdit);

	lAtmLineEdit = new QLineEdit(this);
    lAtmLineEdit->setValidator(intValidator);
	lAtmLineEdit->setText("4");
	lAtmLineEdit->setToolTip(
				QString("Maximum dictionary atoms used in patch reconstruction.\n\n") +
				QString("A too large value leads to overfitting, and too small\n") +
				QString("leads to underfitting, depending on max. frequency."));
	form->addRow(QString("Atom sparsity constraint:"), lAtmLineEdit);

	maxNewLineEdit = new QLineEdit(this);
    maxNewLineEdit->setValidator(intValidator);
	maxNewLineEdit->setText("45000");
	maxNewLineEdit->setToolTip(
				QString("Maximum number of new points to add to the cloud."));
	form->addRow(QString("Maximum number of new points to add:"), maxNewLineEdit);

	methodComboBox = new QComboBox;
	methodComboBox->addItem(tr("Orthogonal Pursuit"));
	methodComboBox->addItem(tr("Matching Pursuit"));
	methodComboBox->setToolTip(
				QString("Sparse approximation method to use during training\n") +
				QString("and patch reconstruction."));
	form->addRow(QString("Sparse approximation method:"), methodComboBox);

	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

}


int ReconstructDialog::getFields(
        int& kSVDIters, size_t& kNN, size_t& nfreq, float& densify,
		size_t& natm, size_t& latm, size_t& maxNewPoints,
		SparseApprox& method)
{

	auto good = QValidator::Acceptable;
	bool ok = true;

	int pos = 0;
	QString kSVDItersStr = nItersLineEdit->text();
    if(intValidator->validate(kSVDItersStr, pos) != good){ ok = false; }
	else{ kSVDIters = kSVDItersStr.toULongLong(&ok); }

	if(!ok) {
		nItersLineEdit->clear();
		return -1;
	}

	pos = 0;
	QString kNNStr = kNNLineEdit->text();
    if(intValidator->validate(kNNStr, pos) != good){ ok = false; }
	else{ kNN = kNNStr.toULongLong(&ok); }

	if(!ok) {
		kNNLineEdit->clear();
		return -2;
	}

	pos = 0;
	QString nfreqStr = nFreqLineEdit->text();
    if(intValidator->validate(nfreqStr, pos) != good){ ok = false; }
	else{ nfreq = nfreqStr.toULongLong(&ok); }

	if(!ok) {
		nFreqLineEdit->clear();
		return -3;
	}

    pos = 0;
    QString densifyStr = densifyLineEdit->text();
    if(doubleValidator->validate(densifyStr, pos) != good){ ok = false; }
    else{ densify = densifyStr.toFloat(&ok); }

    if(!ok) {
        densifyLineEdit->clear();
        return -4;
    }

	pos = 0;
	QString natmStr = nAtmLineEdit->text();
    if(intValidator->validate(natmStr, pos) != good){ ok = false; }
	else{ natm = natmStr.toULongLong(&ok); }

	if(!ok) {
		nAtmLineEdit->clear();
        return -5;
	}

	pos = 0;
	QString latmStr = lAtmLineEdit->text();
    if(intValidator->validate(latmStr, pos) != good){ ok = false; }
	else{
		latm = latmStr.toULongLong(&ok);
		if(latm > natm) ok = false;
	}

	if(!ok) {
		lAtmLineEdit->clear();
        return -6;
	}

	pos = 0;
	QString maxNewStr = maxNewLineEdit->text();
    if(intValidator->validate(maxNewStr, pos) != good){ ok = false; }
	else{ maxNewPoints = maxNewStr.toULongLong(&ok); }

	if(!ok) {
		maxNewLineEdit->clear();
        return -7;
	}

	int index = 0;
	methodComboBox->activated(index);
	switch(index){
	case 0:
		method = SparseApprox::OrthogonalPursuit;
		break;
	case 1:
		method = SparseApprox::MatchingPursuit;
		break;
	}

	return 0;
}
