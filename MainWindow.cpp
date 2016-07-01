#include "MainWindow.h"
#include <QVBoxLayout>
#include <QLabel>

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
}
