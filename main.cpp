#include <QApplication>
#include <QFile>
#include <QTextStream>
#include "MainWindow.h"
#include "SimulatorView.h"
#include "Simulator.h"
#include <stdio.h>
#include <QDebug>

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
	static QFile logFile("awts_log");
	static QTextStream logFileStream(&logFile);
	static bool logFileOpenError = false;
	if( !logFile.isOpen() && !logFileOpenError )
	{
		if( !logFile.open(QIODevice::WriteOnly) )
		{
			qWarning() << "Error opening logFile!";
			logFileOpenError = true;
		}
	}
	QTextStream stdOutStream(stdout);
	QTextStream stdErrStream(stderr);
	stdOutStream.setRealNumberNotation(QTextStream::FixedNotation);
	stdErrStream.setRealNumberNotation(QTextStream::FixedNotation);
	stdOutStream.setRealNumberPrecision(3);
	stdErrStream.setRealNumberPrecision(3);
	switch (type)
	{
	case QtDebugMsg:
		stdErrStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl;
		if( logFileStream.device()->isOpen() )
			{ logFileStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl; }
		break;
	case QtInfoMsg:
		stdOutStream << msg << endl;
		if( logFileStream.device()->isOpen() )
			{ logFileStream << msg << endl; }
		break;
	case QtWarningMsg:
		stdErrStream << msg << " @" << context.file <<":"<< context.line << endl;
		if( logFileStream.device()->isOpen() )
			{ logFileStream << msg << " @" << context.file <<":"<< context.line << endl; }
		break;
	case QtCriticalMsg:
		stdErrStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl;
		if( logFileStream.device()->isOpen() )
			{ logFileStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl; }
		break;
	case QtFatalMsg:
		stdErrStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl;
		if( logFileStream.device()->isOpen() )
			{ logFileStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl; }
	  //abort();
	}
	stdErrStream.flush();
	stdOutStream.flush();
	logFileStream.flush();
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    app.setApplicationName("AWts");
	app.setOrganizationName("okaresz");

	qInstallMessageHandler(myMessageOutput);

	// Create simulator
	Simulator sim;

	// Simulation view
	SimulatorView *simView = new SimulatorView(&sim);

	// main window
	MainWindow mainWin(simView);
    mainWin.setWindowTitle( app.applicationName() );

    mainWin.show();
	simView->update();

	sim.start();

    return app.exec();
}
