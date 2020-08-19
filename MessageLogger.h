#ifndef MESSAGELOGGER_H
#define MESSAGELOGGER_H

#include <QObject>
#include <QRecursiveMutex>

//QT_BEGIN_NAMESPACE
//class QPlainTextEdit;
//QT_END_NAMESPACE

class MessageLogger : public QObject
{
	Q_OBJECT

public:
	explicit MessageLogger(QObject *parent = nullptr);
	~MessageLogger();

	void logMessage(const QString& text, bool append = true);

	void logProgress(const QString& msgPrefix, size_t i, size_t n,
					 int infreq, size_t& threshold, size_t& lastPos);

signals:
	void undo();
	void logTextAppend(const QString& text);
	void logTextInsert(const QString& text);

private:
	//QPlainTextEdit *m_logText;
	size_t m_lastPos = 0;
	QRecursiveMutex m_recMutex;
};

#endif // MESSAGELOGGER_H
