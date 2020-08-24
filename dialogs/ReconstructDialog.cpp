//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "ReconstructDialog.h"
#include "get_field.h"
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
	form->addRow(new QLabel("Fill in surface gaps in cloud within bounding box"));

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
	form->addRow(QString("Local patch size:"), kNNLineEdit);

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

	bBoxComboBox = new QComboBox;
	bBoxComboBox->addItem(tr("True"));
	bBoxComboBox->addItem(tr("False"));
	bBoxComboBox->setToolTip(
				QString("Whether positions of points outside the bounding box\n") +
				QString("are to be considered when reconstructing within the\n") +
				QString("bounding box.\n\n") +
				QString("This helps the reconstruction to agree with points just\n") +
				QString("outside the bounding box.\n\n") +
				QString("Note that this may have the undesirable effect of\n") +
				QString("including points from two areas parallel surfaces where\n") +
				QString("they nearly meet, depending on the patch size. In this\n") +
				QString("case the bounding box for reconstruction might have to\n") +
				QString("be adjusted manually to include only desired points,\n") +
				QString("leaving this field false."));
	form->addRow(QString("Use points outside bounding box:"), bBoxComboBox);

	connect(bBoxComboBox, QOverload<int>::of(&QComboBox::activated),
			this, &ReconstructDialog::bBoxComboChanged);

	methodComboBox = new QComboBox;
	methodComboBox->addItem(tr("Orthogonal Pursuit"));
	methodComboBox->addItem(tr("Matching Pursuit"));
	methodComboBox->setToolTip(
				QString("Sparse approximation method to use during training\n") +
				QString("and patch reconstruction."));
	form->addRow(QString("Sparse approximation method:"), methodComboBox);

	connect(methodComboBox, QOverload<int>::of(&QComboBox::activated),
			this, &ReconstructDialog::methodComboChanged);

	buttonBox = new QDialogButtonBox(
				QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
				Qt::Horizontal, this);

	form->addRow(buttonBox);
	connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));



}

//---------------------------------------------------------

int ReconstructDialog::getFields(
        int& kSVDIters, size_t& kNN, size_t& nfreq, float& densify,
		size_t& natm, size_t& latm, size_t& maxNewPoints, bool& looseBBox,
		SparseApprox& method) const
{
	bool ok;
	size_t temp;

	ok = get_integer_field(nItersLineEdit, intValidator, temp);
	if(!ok) return -1;
	kSVDIters = int(temp);

	ok = get_integer_field(kNNLineEdit, intValidator, kNN);
	if(!ok) return -2;

	ok = get_integer_field(nFreqLineEdit, intValidator, nfreq);
	if(!ok) return -3;

	ok = get_float_field(densifyLineEdit, doubleValidator, densify);
	if(!ok) return -4;

	ok = get_integer_field(nAtmLineEdit, intValidator, natm);
	if(!ok) return -5;

	ok = get_integer_field(lAtmLineEdit, intValidator, latm);
	if(ok) if(latm > natm) ok = false;
	if(!ok) return -6;

	ok = get_integer_field(maxNewLineEdit, intValidator, maxNewPoints);
	if(!ok) return -7;
	
	switch(m_bBoxComboIdx){
	case 0:
		looseBBox = true;
		break;
	case 1:
		looseBBox = false;
		break;
	}

	switch(m_methodComboIdx){
	case 0:
		method = SparseApprox::OrthogonalPursuit;
		break;
	case 1:
		method = SparseApprox::MatchingPursuit;
		break;
	}

	return 0;
}
