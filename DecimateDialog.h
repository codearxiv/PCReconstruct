#ifndef DECIMATEDIALOG_H
#define DECIMATEDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QMainWindow;
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
QT_END_NAMESPACE

class DecimateDialog : public QDialog
{
	Q_OBJECT

public:
	DecimateDialog(QWidget *parent = nullptr);

public slots:
	void decimateCloud(size_t nHoles, size_t kNN)
	{ emit cloudDecimate(nHoles, kNN); }

signals:
	void cloudDecimate(size_t nHoles, size_t kNN);

private:
	QFormLayout *form;
	QLineEdit *lineEditNumHoles;
	QLineEdit *lineEditNumPoints;
	QDialogButtonBox *buttonBox;

	QMainWindow *mainWindow;

};


#endif // DECIMATEDIALOG_H
