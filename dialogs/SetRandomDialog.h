//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef SETRANDOMDIALOG_H
#define SETRANDOMDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QMainWindow;
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QIntValidator;
QT_END_NAMESPACE

class SetRandomDialog : public QDialog
{
	Q_OBJECT

public:
	SetRandomDialog(QWidget *parent = nullptr);
	bool getFields(size_t& nPoints);

public slots:
	void setRandomCloud(size_t nPoints)
	{ emit cloudSetRandom(nPoints); }

signals:
	void cloudSetRandom(size_t nPoints);

private:
	QFormLayout *form;
	QLineEdit *nPointsLineEdit;
	QDialogButtonBox *buttonBox;
	QIntValidator *validator;
	QMainWindow *mainWindow;

};

#endif // SETRANDOMDIALOG_H
