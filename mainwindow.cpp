#include "mainwindow.h"
#include <QVBoxLayout>
#include <QLabel>
#include "simviewwidget.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{   
    paramsWidget = new QWidget(this);
    new QVBoxLayout(paramsWidget);
    paramsWidget->setMinimumWidth(200);

    QLabel *label = new QLabel(paramsWidget);
    label->setText("I'm just a label.");

    paramsDock = new QDockWidget(tr("Params"), this);
    paramsDock->setAllowedAreas( Qt::AllDockWidgetAreas );
    paramsDock->setWidget(paramsWidget);
    addDockWidget( Qt::RightDockWidgetArea, paramsDock );

    // Simulation view
    simView = new SimViewWidget(this);
    setCentralWidget(simView);
    simView->update();
}
