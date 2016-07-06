#ifndef CARDRIVER_H
#define CARDRIVER_H

#include <QObject>
#include "Car.h"
#include "RoadGenerator.h"
#include <QList>
#include "TrajectorySection.h"
#include <QVector2D>

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

	/// Get current centripetal acceleration (signed, left turns are negative).
	double currentCentripetalAccel() const;
	/// Get net (sum) acceleration (left turns are pointing towards negative).
	QVector2D currentNetAccel() const;
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

	enum accelerationMode_t
	{
		MaxAcceleration,
		ProportionalAcceleration
	};

	Car *mCar;
	RoadGenerator *mRoadGen;
	double mCruiseSpeed, mTargetSpeed;
	double mCarCrossPos;	///< Normal (not tangent) car position across the road, in meters from the road center (signed).
	accelerationMode_t mAccelerationMode;
	struct threatAccumulator
	{
		int tractionLoss;
		int crash;
	} mThreats;
	bool mCrashed, mTractionLost;

	// Safety factors
	/** With full speed in bend, there can be no deceleration
	*	due to net force being already at maximum allowed by friction, so create this safety factor.*/
	double mBendMaxSpeedSafetyFactor;
	static const double obstacleAvoidanceDistance;	///< Avoid obstacles by this much between the car and the obstacle


	QList<TrajectorySection*> mTrajectory;

	enum roadFeaturePointType
	{
		UnknownFeaturePoint,
		BendStartFeaturePoint,
		StraightStartFeaturePoint,
		ObstacleFeaturePoint
	};

	struct roadFeaturePoint
	{
		roadFeaturePointType type;
		bool brakePointSet; ///< driver algorithm has set a brakepoint for it at least once
		double odoPos;	///< position of feature point along road.
		double radius;	///< radius of the bend that the feature point belongs to.
		double maxSpeed;	///< max speed from this feature point.
		roadFeaturePoint()
		{
			type = UnknownFeaturePoint;
			brakePointSet = false;
			odoPos = -1.0;
			radius = 0.0;
			maxSpeed = 0.0;
		}
	};
	QList<roadFeaturePoint*> mRoadFeaturePoints;
};

#endif // CARDRIVER_H
