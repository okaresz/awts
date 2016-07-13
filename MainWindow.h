#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>

class SimulatorView;
class QStatusBar;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
	explicit MainWindow( SimulatorView *view, QWidget *parent = 0);

signals:

public slots:
	void onSimUpdated();
	void updateSpeedDisplay(double speedKmh);
	void pixelPerMeterChanged(double newVal);

	void onCarTractionLost(double atTravel);
	void onCarCrashed(double atTravel);
	void onCarUnavoidableTractionLossDetected(double atTravel);
	void onCarUnavoidableCrashDetected(double atTravel);

private slots:
	void pxPerMeterChangeReqInt( int sliderVal );
	void cruiseSpeedValueChanged( double val );
	void onSimRunButtonToggled(bool clicked);
	void onSteeringSliderChanged(int value);
	void onTargetCrossPosSPinBoxChanged(double value);

private:
	void buildParamsWidget();
	void buildDashboardWidget();

	QDockWidget *mParamsDock, *mDasboardDock;
	QWidget *mParamsWidget, *mDashboardWidget;
	QStatusBar *mStatusBar;
	SimulatorView *mSimView;
};

#endif // MAINWINDOW_H
