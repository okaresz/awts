#ifndef CARDRIVER_H
#define CARDRIVER_H

#include <QObject>
#include "Car.h"
#include "RoadGenerator.h"
#include <QHash>
#include <QList>
#include <QVector2D>

class CarDriver : public QObject
{
	Q_OBJECT
public:
	explicit CarDriver( Car *car, RoadGenerator *roadGen, QObject *parent = 0);

	struct carLocation_t
	{
		double tangent;	///< tangential position along the road
		double normal; ///< norma position along the road in meters, from the road centerline
		double heading; ///< heading of the car, relative to the road, 0 is going straight parallel to the road
	};

	struct steeringControl_t
	{
		double targetCrossPos;	///< Target cross position on road to follow with steering, in meters from the road center (signed).
		double P,I,D;
		double feedForward;
		double integratorSum;
	};

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

	steeringControl_t steeringControlParams() const
		{ return mSteeringControl; }

	bool keyboardEvent( Qt::Key key, bool pressed );

	bool manualDrive() const
		{ return mManualDrive; }

	double targetCrossPos() const
		{ return mSteeringControl.targetCrossPos; }

signals:
	void tractionLost(double atTravel);
	void unavoidableTractionLossDetected(double atTravel);
	void unavoidableCrashDetected(double atTravel);
	void crashed(double atTravel);

public slots:
	void simUpdate(const quint64 simTime,const double travel,const double carCrossPosOnRoad, const double carHeadingOnRoad);

	void setSteeringControlP(const double P)
		{ mSteeringControl.P = P; }
	void setSteeringControlI(const double I)
		{ mSteeringControl.I = I; }
	void setSteeringControlD(const double D)
		{ mSteeringControl.D = D; }

	void setManualDrive(bool value)
		{ mManualDrive = value; }

	void setTargetCrossPos(double crossPos);

	void setSteeringControlFeedForward(const double value);

private:

	enum accelerationMode_t
	{
		MaxAcceleration,
		ProportionalAcceleration
	};

	Car *mCar;
	RoadGenerator *mRoadGen;
	double mCruiseSpeed, mTargetSpeed;
	steeringControl_t mSteeringControl;
	accelerationMode_t mAccelerationMode;
	struct threatAccumulator
	{
		int tractionLoss;
		int crash;
	} mThreats;
	bool mCrashed, mTractionLost;
	bool mManualDrive;
	QHash<Qt::Key,bool> mKeyStatus;

	// Safety factors
	/** With full speed in bend, there can be no deceleration
	*	due to net force being already at maximum allowed by friction, so create this safety factor.*/
	double mBendMaxSpeedSafetyFactor;
	static const double obstacleAvoidanceDistance;	///< Avoid obstacles by this much between the car and the obstacle

	// Feature point stuff

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
		double roadParam;	///< position of feature point along road.
		double crossPos; ///< Position across the road (in meters, from the road centerline) (used with obstacles)
		double curvature;	///< radius of the bend that the feature point belongs to.
		double minRoadWidth; ///< minimum width on the segment.
		double maxSpeed;	///< max speed from this feature point.
		double size;		///< only for obstacle
		const RoadSegment *segment;	///< relevant roadSegment belonging to the feature point
		roadFeaturePoint()
		{
			type = UnknownFeaturePoint;
			brakePointSet = false;
			roadParam = -1.0;
			crossPos = 0.0;
			curvature = 0.0;
			minRoadWidth = 0.0;
			maxSpeed = 0.0;
			segment = nullptr;
		}
	};
	QList<roadFeaturePoint*> mRoadFeaturePoints;

	/// Insert given point into the list, at the correct position so that point list is sorted (ascending) by odoPos.
	void insertRoadFeaturePoint(roadFeaturePoint *rfp );

	// Calculate the cross-position on road where car should be when reaches obstacle
	double calculateObstacleAvoidancePoint(const double carCrossPos, const double obstOdoPos, const double obstCrossPos, const double obstSize);

};

#endif // CARDRIVER_H
