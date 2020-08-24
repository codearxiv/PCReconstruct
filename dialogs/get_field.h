//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#ifndef GET_FIELD_H
#define GET_FIELD_H

#include <QObject>

QT_BEGIN_NAMESPACE
class QLineEdit;
class QIntValidator;
class QDoubleValidator;
QT_END_NAMESPACE

bool get_integer_field(
		QLineEdit *lineEdit, QIntValidator *validator, size_t& field);

bool get_float_field(
		QLineEdit *lineEdit, QDoubleValidator *validator, float& field);

#endif // GET_FIELD_H
