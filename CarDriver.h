#ifndef CARDRIVER_H
#define CARDRIVER_H

#include <QObject>
#include "Car.h"
#include "RoadGenerator.h"
#include <QList>
#include "TrajectorySection.h"

class CarDriver : public QObject
{
	Q_OBJECT
public:
	explicit CarDriver( Car *car, RoadGenerator *roadGen, QObject *parent = 0);

	double cruiseSpeedKmh() const;
	double cruiseSpeedMps() const;
	void setCruiseSpeedMps(double cruiseSpeedMps);
	void setCruiseSpeedKmh(double cruiseSpeedKmh);
	double targetSpeedKmh() const;
	double targetSpeedMps() const;

	double currentCentripetalAccel() const;
	double currentNetAccel() const;
	double currentMaxPossibleSpeedOnBend() const;

signals:
	void tractionLost(double atOdoPos);
	void unavoidableTractionLossDetected(double lossOdoPos);
	void unavoidableCrashDetected(double crashOdoPos);
	void crashed(double atOdoPos);

public slots:
	void simUpdate(const quint64 simTime);

private:
	// trajectory stuff. Should move to separate class?...
	void planTrajectory( double odometer );
	double odoEndOfTrajectory() const;

	// Calculate the cross-position on road where car should be when reaches obstacle
	double calculateObstacleAvoidancePoint(const RoadObstacle *obstacle);

	Car *mCar;
	RoadGenerator *mRoadGen;
	double mCruiseSpeed, mTargetSpeed;
	double mCarCrossPos;	///< Normal (not tangent) car position across the road, in meters from the road center (signed).
	QList<TrajectorySection*> mTrajectory;
	static const double obstacleAvoidanceDistance;	///< Avoid obstacles by this much between the car and the obstacle

	enum roadFeaturePointType
	{
		BendStartFeaturePoint,
		StraightStart,
		ObstacleFeaturePoint
	};

	struct roadFeaturePoint
	{
		double odoPos;
	};
};

#endif // CARDRIVER_H
