#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>

class SimulatorView;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
	explicit MainWindow( SimulatorView *view, QWidget *parent = 0);

signals:
	void updateMaxAccelRatioDisplay(int ratio);

public slots:
	void onSimUpdated();
	void updateSpeedDisplay(double speedKmh);
	void pixelPerMeterChanged(double newVal);

private slots:
	void pxPerMeterChangeReqInt( int sliderVal );
	void cruiseSpeedValueChanged( double val );
	void onSimRunButtonToggled(bool clicked);

private:
	void buildParamsWidget();
	void buildDashboardWidget();

	QDockWidget *mParamsDock, *mDasboardDock;
	QWidget *mParamsWidget, *mDashboardWidget;
	SimulatorView *mSimView;
};

#endif // MAINWINDOW_H
