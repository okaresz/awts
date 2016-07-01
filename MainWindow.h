#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>

class SimulatorView;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);

signals:

public slots:

private:
    QDockWidget *paramsDock;
    QWidget *paramsWidget;
    SimulatorView *simView;
};

#endif // MAINWINDOW_H
