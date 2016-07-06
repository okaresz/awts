#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QObject>
#include <QTimer>
#include "RoadGenerator.h"
#include "CarDriver.h"
#include "Car.h"

/// Simulator is responsible for running the simulation and managing all the components, interactions.
class Simulator : public QObject
{
	Q_OBJECT
public:
	explicit Simulator(QObject *parent = 0);

	const Car *car() const
		{ return &mCar; }

	CarDriver *carDriver() const
		{ return (CarDriver*)&mDriver; }

	double currentMaxAccelerationRatio() const;

signals:
	void simUpdated();
	void simRunStateChanged(bool running);

public slots:
	/** Start simulation.
	 * Simulation is resumed where it has been stopped.*/
	void start();
	/** Stop simulation.
	 *  Simulation timer stopped, all state is preserved. Practically this is a pause.*/
	void stop();

	/// Update simulation periodically
	void simUpdate();

	const RoadGenerator *roadGen() const
		{ return &mRoadGen; }

	double roadVisibility() const
		{ return mRoadVisibility; }

	void onCarTractionLost(double atOdometer);
	void onCarCrashed(double atOdometer);

private:

	QTimer mSimTimer;
	quint64 mSimTime;
	int mSimIntervalMs;
	double mRoadVisibility;
	RoadGenerator mRoadGen;

	Car mCar;
	CarDriver mDriver;
};

#endif // SIMULATOR_H
