#ifndef MESSAGELOGGER_H
#define MESSAGELOGGER_H

#include <QObject>
#include <QRecursiveMutex>

QT_BEGIN_NAMESPACE
class QPlainTextEdit;
QT_END_NAMESPACE

class MessageLogger : public QObject
{
	Q_OBJECT

public:
	explicit MessageLogger(QPlainTextEdit *logText = nullptr) :
		QObject(), m_logText(logText) {}
	~MessageLogger();

	void set(QPlainTextEdit *logText) {
		m_logText = logText;
		m_lastPos = 0;
	}

public slots:
	void logMessage(const QString& text, bool append = true);

	void logProgress(const QString& msgPrefix, size_t i, size_t n,
					 int infreq, size_t& threshold, size_t& lastPos);
private:
	QPlainTextEdit *m_logText;
	size_t m_lastPos = 0;
	QRecursiveMutex m_recMutex;
};

#endif // MESSAGELOGGER_H
