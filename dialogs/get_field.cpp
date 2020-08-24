//     Copyright (C) 2019 Piotr (Peter) Beben <pdbcas@gmail.com>
//     See LICENSE included with this distribution.

#include "get_field.h"

#include <QLineEdit>
#include <QIntValidator>
#include <QDoubleValidator>

bool get_integer_field(
		QLineEdit *lineEdit, QIntValidator *validator, size_t& field)
{
	QString fieldStr = lineEdit->text();
	int pos = 0;
	if(validator->validate(fieldStr, pos) != QValidator::Acceptable){
		lineEdit->clear();
		return false;
	}
	else{
		bool ok;
		field = fieldStr.toULongLong(&ok);
		if(!ok) {
			lineEdit->clear();
			return false;
		}
	}

	return true;
}

bool get_float_field(
		QLineEdit *lineEdit, QDoubleValidator *validator, float& field)
{
	QString fieldStr = lineEdit->text();
	int pos = 0;
	if(validator->validate(fieldStr, pos) != QValidator::Acceptable){
		lineEdit->clear();
		return false;
	}
	else{
		bool ok;
		field = fieldStr.toFloat(&ok);
		if(!ok) {
			lineEdit->clear();
			return false;
		}
	}

	return true;
}
