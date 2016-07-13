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

	struct carPositionOnRoad_t
	{
		double travel;  ///< The distance the car has covered along the road, expressed with the road's curve parameter. The car's travel.
		double cross;	///< The car position on the road, perpendicular to the centerline, in meters, signed
		double heading;	///< Heading angle, 0 being car is pointing "upwards".
	};

	Car *car() const
		{ return (Car*)&mCar; }

	CarDriver *carDriver() const
		{ return (CarDriver*)&mDriver; }

	double currentMaxAccelerationRatio() const;

signals:
	void simUpdated();
	void simRunStateChanged(bool running);

	void carTractionLost(double atTravel);
	void carUnavoidableTractionLossDetected(double atTravel);
	void carUnavoidableCrashDetected(double atTravel);
	void carCrashed(double atTravel);

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

	void onCarTractionLost(double atTravel);
	void onCarCrashed(double atTravel);
	void onCarUnavoidableTractionLossDetected(double atTravel);
	void onCarUnavoidableCrashDetected(double atTravel);

	carPositionOnRoad_t carPositionOnRoad() const
		{ return mCarPosOnRoad; }

private:

	QTimer mSimTimer;
	quint64 mSimTime;
	int mSimIntervalMs;
	double mRoadVisibility;
	RoadGenerator mRoadGen;

	Car mCar;
	CarDriver mDriver;
	carPositionOnRoad_t mCarPosOnRoad;

	/** Find delta travel (advancement of car expressed in roadParameter space).
	 *	As it's above me to find a clear geometric solution how to calculate the exact roadParam, where car y position in road coordSys is 0
	 *  (meaning the car crossPosition is on the x axis of roadCoordsys, so road sys (0,0) is on the centerline, at the roadParam I'm interested in).
	 *  Problem is if the segment at travel is a bend or there is a segment change between fromTravel and new travel....
	 *  So instead, find this magic point with an iterative approximation. */
	double findTravel(double fromTravel, double &carCrossPos, double &carHeading);
};

#endif // SIMULATOR_H
