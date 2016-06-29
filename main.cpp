#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    app.setApplicationName("AWts");

    MainWindow mainWin;
    mainWin.setWindowTitle( app.applicationName() );
    mainWin.show();

    return app.exec();
}
