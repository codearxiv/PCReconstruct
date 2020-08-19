#include "MessageLogger.h"

#include <QPlainTextEdit>
#include <QRecursiveMutex>
#include <QScrollBar>


MessageLogger::MessageLogger(QObject *parent) :
	QObject(parent)
{

}


MessageLogger::~MessageLogger()
{

}

void MessageLogger::logMessage(const QString& text, bool append) {

	QMutexLocker locker(&m_recMutex);

	if( append ){
		emit logTextAppend(text);
		++m_lastPos;
	}
	else{
		emit logTextInsert(text);
	}

}

void MessageLogger::logProgress(
		const QString& msgPrefix, size_t i, size_t n, int infreq,
		size_t& threshold, size_t& lastPos) {
	float percent = (100.0f*i)/n;
	if(percent >= threshold) {
		QMutexLocker locker(&m_recMutex);
		bool append = threshold > 0 ? (lastPos < m_lastPos) : true;
		logMessage(msgPrefix + ": " + QString::number(int(percent)) + "%...",
					append);
		threshold += infreq;
		lastPos = m_lastPos;
	}
}
