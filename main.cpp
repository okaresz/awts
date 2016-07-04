#include <QApplication>
#include "MainWindow.h"
#include "SimulatorView.h"
#include "Simulator.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    app.setApplicationName("AWts");
	app.setOrganizationName("okaresz");

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
