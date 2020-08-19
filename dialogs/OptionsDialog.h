//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included.

#ifndef OPTIONSDIALOG_H
#define OPTIONSDIALOG_H

#include <QWidget>
#include <QDialog>
#include <QObject>

QT_BEGIN_NAMESPACE
class QFormLayout;
class QLineEdit;
class QDialogButtonBox;
class QDoubleValidator;
QT_END_NAMESPACE

class OptionsDialog : public QDialog
{
    Q_OBJECT

public:
    OptionsDialog(QWidget *parent = nullptr);
    bool getFields(float& percent);

public slots:
    void SparsifyCloud(float percent)
    { emit cloudSparsify(percent); }

signals:
    void cloudSparsify(float percent);

private:
    QFormLayout *form;
    QLineEdit *pointSizeLineEdit;
    QDialogButtonBox *buttonBox;
    QDoubleValidator *validator;

};


#endif // OPTIONSDIALOG_H
