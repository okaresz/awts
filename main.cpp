#include <QApplication>
#include <QTextStream>
#include "MainWindow.h"
#include "SimulatorView.h"
#include "Simulator.h"
#include <stdio.h>

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
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
		break;
	case QtInfoMsg:
		stdOutStream << msg << endl;
		break;
	case QtWarningMsg:
		stdErrStream << msg << " @" << context.file <<":"<< context.line << endl;
		break;
	case QtCriticalMsg:
		stdErrStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl;
		break;
	case QtFatalMsg:
		stdErrStream << msg << " @" << context.file <<":"<< context.line << "|"<< context.function << endl;
	  //abort();
	}
	stdErrStream.flush();
	stdOutStream.flush();
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
