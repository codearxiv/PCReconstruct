#ifndef MESSAGELOGGER_H
#define MESSAGELOGGER_H

#include <QObject>

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

	void set(QPlainTextEdit *logText) { m_logText = logText; }

public slots:
	void logMessage(const QString& text);

	void logProgress(const QString& msgPrefix,
					 size_t i, size_t n, int infreq, int& threshold);
private:
	QPlainTextEdit *m_logText;

};

#endif // MESSAGELOGGER_H
