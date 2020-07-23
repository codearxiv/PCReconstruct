#ifndef MESSAGELOGGER_H
#define MESSAGELOGGER_H

//#include <QPlainTextEdit>
//#include <QObject>
//#include <QMutex>


class MessageLogger : public QObject
{
	Q_OBJECT

public:
	explicit MessageLogger(QPlainTextEdit *logText = nullptr) :
		QObject(), m_logText(logText) {}
	~MessageLogger();

	void set(QPlainTextEdit *logText) { m_logText = logText; }

public slots:
	void logMessage(const QString& text) {
		m_logText->appendPlainText(text);
	}

	void logProgress(const QString& msgPrefix,
					 size_t i, size_t n, int infreq, int& threshold) {
		float percent = (100.0f*i)/n;
		if(percent >= threshold) {
			if(threshold > 0) { m_logText->undo(); }
			logMessage(msgPrefix + ": " + QString::number(int(percent)) + "%...");
			threshold += infreq;
		}
	}

private:
	QPlainTextEdit *m_logText;

};

#endif // MESSAGELOGGER_H
