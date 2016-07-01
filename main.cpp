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

    MainWindow mainWin;
    mainWin.setWindowTitle( app.applicationName() );

	// Simulation view
	SimulatorView *simView = new SimulatorView(&sim);
	mainWin.setCentralWidget(simView);
	simView->update();

    mainWin.show();

	sim.start();

    return app.exec();
}
