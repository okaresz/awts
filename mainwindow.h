#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>

class SimViewWidget;

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
    SimViewWidget *simView;
};

#endif // MAINWINDOW_H
