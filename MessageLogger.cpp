#include "MessageLogger.h"

#include <QPlainTextEdit>


MessageLogger::~MessageLogger()
{

}

void MessageLogger::logMessage(const QString& text) {
	m_logText->appendPlainText(text);
}

void MessageLogger::logProgress(
		const QString& msgPrefix,
		size_t i, size_t n, int infreq, int& threshold) {
	float percent = (100.0f*i)/n;
	if(percent >= threshold) {
		if(threshold > 0) { m_logText->undo(); }
		logMessage(msgPrefix + ": " + QString::number(int(percent)) + "%...");
		threshold += infreq;
	}
}
