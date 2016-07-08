#include "CarDriver.h"
#include <QDebug>
#include <QLineF>

/* NOTES
 * - left bend radius is negative.
 * - a turn span angle is always positive.*/

const double CarDriver::obstacleAvoidanceDistance = 0.5;

CarDriver::CarDriver(Car *car, RoadGenerator *roadGen, QObject *parent) : QObject(parent),
	mCar(car), mRoadGen(roadGen), mCruiseSpeed(14), mTargetSpeed(mCruiseSpeed), mTargetCrossPos(0.0), mAccelerationMode(ProportionalAcceleration),
	mThreats({0,0}), mCrashed(false), mTractionLost(false), mBendMaxSpeedSafetyFactor(0.8)
{

}

//void CarDriver::updateCarLocation(double odometer, double previousOdometer, const RoadSegment *rSeg)
//{
//	// car
//	double dOdoCar = odometer - previousOdometer;
//	double carTurnC = mCar->turnCurvature();
//	double dHeadingCar = dOdoCar*carTurnC;
//	// define for carTurnC == 0.0
//	double dPtc = dOdoCar;
//	double dPnc = 0;
//	if( carTurnC != 0.0 )
//	{
//		dPtc = sin(dHeadingCar)/carTurnC;
//		dPnc = (1-cos(dHeadingCar))/carTurnC;
//	}
//	// rotate with current heading (dPt is x coord, dPc is y, rotate around (0,0) with mCarLocation.heading)
//	dPnc = dPnc*cos(mCarLocation.heading) + dPtc*sin(mCarLocation.heading);
//	dPtc = -dPnc*sin(mCarLocation.heading) + dPtc*cos(mCarLocation.heading);

//	// road
//	double roadC = rSeg->curvature();
//	double roadDeltaOdo = dOdoCar;
//	double dHeadingRoad = roadDeltaOdo*roadC;
//	double dPtr = dOdoCar;
//	double dPnr = 0;
//	if( roadC != 0.0 )
//	{
//		roadDeltaOdo = atan(dPtc/(1/roadC-dPnc)) / roadC;
//		dPtr = sin(dHeadingRoad)/roadC;
//		dPnr = (1-cos(dHeadingRoad))/roadC;
//	}

//	mCarLocation.tangent += roadDeltaOdo;
//	mCarLocation.heading += dHeadingCar - dHeadingRoad;
//	// normal pos change is a bit more complicated, as we have to find the distance from the road centerline
//	// along the line _perpendicular_ to the centerline at the point of roadDeltaOdo (see "road advancement" in calculation notes)
//	QVector2D dCarPos( dPnc, dPtc );
//	QVector2D dRoadPos( dPnr, dPtr );
//	QVector2D dCarMinusRoad = dCarPos - dRoadPos;
//	if( dCarMinusRoad.x() < 0 )
//		{ mCarLocation.normal -= dCarMinusRoad.length(); }
//	else
//		{ mCarLocation.normal += dCarMinusRoad.length(); }
//}

void CarDriver::simUpdate(const quint64 simTime, const double travel, const double carCrossPosOnRoad, const double carHeadingOnRoad)
{
	Q_UNUSED(simTime);

	// some facts
	double currentSpeed = mCar->speed();
	QVector2D currentAcceleration = currentNetAccel();
	double maxAccelAllowedByFriction = mCar->frictionCoeffStatic() * 9.81;

	if( currentAcceleration.length() > maxAccelAllowedByFriction && !mTractionLost )
	{
		mTractionLost = true;
		emit tractionLost(travel);
		qWarning() << "TRACTION LOST at travel=" << travel;
	}

	/* === FEATURE POINT GENERATION ============================================================*/
	QQueue<RoadSegment> visibleRoad( mRoadGen->visibleRoad(travel) );
	QQueue<RoadObstacle> visibleObstacles( mRoadGen->visibleObstacles(travel) );
	static double featurePointGenHorizon = 0;

	// scan road, generate featurePoints
	for( int i=0; i<visibleRoad.size(); ++i )
	{
		// find the segment at featurePointGenHorizon
		if( visibleRoad.at(i).startLocation().parameter >= featurePointGenHorizon )
		{
			const RoadSegment *seg = &visibleRoad.at(i);
			roadFeaturePoint *rfp = new roadFeaturePoint;
			rfp->roadParam = seg->startLocation().parameter;
			if( seg->isBend() )
			{
				rfp->type = BendStartFeaturePoint;
				rfp->curvature = seg->curvature();
				rfp->maxSpeed = sqrt( maxAccelAllowedByFriction * seg->radiusAbs() ) * mBendMaxSpeedSafetyFactor;
			}
			else
			{
				rfp->type = StraightStartFeaturePoint;
				rfp->maxSpeed = mCruiseSpeed;
			}
			mRoadFeaturePoints.append(rfp);
		}
	}
	// scan obstacles, generate featurePoints
	for( int i=0; i<visibleObstacles.size(); ++i )
	{
		// find the segment at featurePointGenHorizon
		if( visibleObstacles.at(i).roadParam() >= featurePointGenHorizon )
		{
			const RoadObstacle *obst = &visibleObstacles.at(i);
			const RoadSegment *segAtObst = mRoadGen->segmentAt(obst->roadParam());
			if( !segAtObst )
			{
				qWarning() << "No segment at obstacle@"<<obst->roadParam();
				continue;
			}
			roadFeaturePoint *rfp = new roadFeaturePoint;
			rfp->type = ObstacleFeaturePoint;
			rfp->roadParam = obst->roadParam();
			rfp->crossPos = obst->normalPos()*segAtObst->widthAt( obst->roadParam()-segAtObst->startLocation().parameter )/2;
			rfp->size = obst->size();
			rfp->maxSpeed = mCruiseSpeed;	// TODO is this OK?
			//insertRoadFeaturePoint( rfp );
		}
	}
	featurePointGenHorizon = visibleRoad.last().endRoadParam();

	// OBSTACLE AVOIDANCE
	// search for the nearest obstacle that is on a collision course and set target crossPos to avoid
	for( int i=0; i<mRoadFeaturePoints.size(); ++i )
	{
		roadFeaturePoint *rfp =  mRoadFeaturePoints.at(i);
		if( rfp->type == ObstacleFeaturePoint && rfp->roadParam > travel )
		{
			double targetCrossPosToAvoid = calculateObstacleAvoidancePoint( carCrossPosOnRoad, rfp->roadParam, rfp->crossPos, rfp->size );
			if( targetCrossPosToAvoid != carCrossPosOnRoad )
			{
				mTargetCrossPos = targetCrossPosToAvoid;
				break;
			}
		}
	}

	/* SPEED & ACCELERATION CONTROL. ==============================================================================================================
	 * Algorithm: the following sequence is followed:
	 * - if a featurePoint is passed (left behind), set it's speed as target and set ProportionalAccel method.
	 * - till the nearest brakePoint, continue to go with/accel to targetSpeed (given by last left-behind featurePoint)
	 * - calculate closest brakePoint from roadFeaturePoints
	 * - if a brakePoint reached, set MaxAccel method and targetSpeed of the featurePoint belonging to the brakePoint
	 * */

	double unavoidableTractionLossAtTravel = -1.0;
	double unavoidableCrashAtTravel = -1.0;

	//more facts
	double currentCentripetalAccel = currentAcceleration.x();
	Car::accelDecelPair_t maxTangentialAccel = mCar->maxTangentialAcceleration(currentCentripetalAccel);
	if( maxTangentialAccel.deceleration < 0.05 )
		{ qWarning() << "Max possible tangential decel is practically zero!"; }
	Car::accelDecelPair_t maxSafeTangentialAccel( maxTangentialAccel.acceleration * 0.95,
												maxTangentialAccel.deceleration * 0.95 ); // don't go on the limit
	qInfo() << "accVect=" << currentAcceleration;
	qInfo() << "speed=" << currentSpeed*3.6;

	// if a feature point is passed (left behind, but not yet deleted at the end of this simUpdate), set it's target speed and proportional acc
	roadFeaturePoint *lastPassedFP = nullptr;
	for( int i=0; i<mRoadFeaturePoints.size(); ++i )
	{
		if( mRoadFeaturePoints.at(i)->roadParam < travel )
			{ lastPassedFP = mRoadFeaturePoints.at(i); }
	}
	if( lastPassedFP )
	{
		mAccelerationMode = ProportionalAcceleration;
		mTargetSpeed = qBound(0.0,lastPassedFP->maxSpeed,mCruiseSpeed);
		qInfo() << "Passed FeaturePoint@"<<lastPassedFP->roadParam<<", set targetSpeed="<<mTargetSpeed;

		// TODO work on this
		mCar->steerForTurnCurvature(lastPassedFP->curvature);
	}

	// Search for and set nearest brakePoint -------------------------------------------------------------------------
	double nearestBrakePointRoadParam = -1;
	roadFeaturePoint *featurePointOfNearestBrake = nullptr;
	bool brakeSetCopyOfFeaturePointOfNearestBrake = false;
	for( int i=0; i<mRoadFeaturePoints.size(); ++i )
	{
		if( mRoadFeaturePoints.at(i)->maxSpeed < currentSpeed )
		{
			double decelTimeForNextBend = (currentSpeed-mRoadFeaturePoints.at(i)->maxSpeed) / maxSafeTangentialAccel.deceleration;
			double decelDistance = currentSpeed*decelTimeForNextBend - maxSafeTangentialAccel.deceleration * pow(decelTimeForNextBend,2) / 2; // minus, because deceleration
			double brakeAtRoadParam = mRoadFeaturePoints.at(i)->roadParam - decelDistance;
			if( nearestBrakePointRoadParam < 0 || brakeAtRoadParam < nearestBrakePointRoadParam )
			{
				nearestBrakePointRoadParam = brakeAtRoadParam;
				featurePointOfNearestBrake = mRoadFeaturePoints.at(i);
				brakeSetCopyOfFeaturePointOfNearestBrake = featurePointOfNearestBrake->brakePointSet;
				featurePointOfNearestBrake->brakePointSet = true;
			}
		}
	}

	// this section is active from the detection of the feature/brake point till car reaches target speed
	if( featurePointOfNearestBrake && nearestBrakePointRoadParam < travel )
	{
		if( !brakeSetCopyOfFeaturePointOfNearestBrake )
		{ // set threat signals
			if( featurePointOfNearestBrake )
			{
				if( featurePointOfNearestBrake->type == BendStartFeaturePoint )
				{
					++mThreats.tractionLoss;
					if( unavoidableTractionLossAtTravel < 0 || unavoidableTractionLossAtTravel > featurePointOfNearestBrake->roadParam )
						{ unavoidableTractionLossAtTravel = featurePointOfNearestBrake->roadParam; }
				}
				else
				{
					++mThreats.crash;
					if( unavoidableCrashAtTravel < 0 || unavoidableCrashAtTravel > featurePointOfNearestBrake->roadParam )
						{ unavoidableCrashAtTravel = featurePointOfNearestBrake->roadParam; }
				}
			}
			else
				{ ++mThreats.crash; }
		}

		mAccelerationMode = MaxAcceleration;
		mTargetSpeed = featurePointOfNearestBrake->maxSpeed;
	}

	// calculate acceleration -------------------------------------------------------------------------
	double actualAcceleration = 0;
	double speedError = mTargetSpeed - currentSpeed;

	static double previousSpeed = 0;

	if( mAccelerationMode == MaxAcceleration )
	{
		if( (previousSpeed > mTargetSpeed && currentSpeed < mTargetSpeed) || (previousSpeed < mTargetSpeed && currentSpeed > mTargetSpeed) )
		{
			mAccelerationMode = ProportionalAcceleration;
			actualAcceleration = 0.0;
		}
		else
		{
			if( speedError < 0.0 )
				{ actualAcceleration = -maxSafeTangentialAccel.deceleration; }
			else if( speedError > 0.0 )
				{ actualAcceleration = maxSafeTangentialAccel.acceleration; }
			else	// double equality? ...eh... not so critical here
				{ actualAcceleration = 0.0; }
		}
	}
	else if( mAccelerationMode == ProportionalAcceleration )
	{
		actualAcceleration = qBound(-maxSafeTangentialAccel.deceleration, speedError * 0.8, maxSafeTangentialAccel.acceleration);	// P controller
	}

	previousSpeed = currentSpeed;

	/* CROSS-POSITION CONTROL ====================================================================================
	 * -*/
//	double crossPosError = mTargetCrossPos - mCarLocation.normal;
//	double steerAngleRad = crossPosError * 0.01;
//	double minTurnCurvatureWithCurrentSpeed = (mCar->frictionCoeffStatic()*9.81)/pow(currentSpeed,2)*0.95;	// *0.95 for safety
//	double maxWheelAngleWithCurrentSpeed = mCar->wheelAngleAtTurnCurvature(minTurnCurvatureWithCurrentSpeed);
//	if( steerAngleRad > maxWheelAngleWithCurrentSpeed )
//		{ steerAngleRad = maxWheelAngleWithCurrentSpeed; }
//	else if( steerAngleRad < -maxWheelAngleWithCurrentSpeed )
//		{ steerAngleRad = -maxWheelAngleWithCurrentSpeed; }
//	mCar->steer( steerAngleRad );
//	qInfo() << "Steer(deg): " << steerAngleRad/M_PI*180.0;

	// set car params -------------------------------
	mCar->accelerate( actualAcceleration );
	qInfo() << "Set car acc: " << actualAcceleration;
	qInfo() << "Car wheelAngle (deg): " << mCar->wheelAngle()/M_PI*180.0;

	// end stuff -----------------------------

	// signal threats
	if( !mTractionLost && mThreats.tractionLoss > 0 )
	{
		qWarning() << "Unavoidable traction loss at travel " << unavoidableTractionLossAtTravel;
		emit unavoidableTractionLossDetected(unavoidableTractionLossAtTravel);
	}
	if( !mCrashed && mThreats.crash > 0 )
	{
		qWarning() << "Unavoidable crash at travel " << unavoidableCrashAtTravel;
		emit unavoidableCrashDetected(unavoidableCrashAtTravel);
	}
	mThreats.tractionLoss = 0;
	mThreats.crash = 0;

	// delete left-behind feature point(s)
	while( !mRoadFeaturePoints.isEmpty() && mRoadFeaturePoints.first()->roadParam < travel )
		{ delete mRoadFeaturePoints.takeFirst(); }

	qInfo() << "------------------------------------------------------";
}

double CarDriver::calculateObstacleAvoidancePoint( const double carCrossPos, const double obstRoadParam, const double obstCrossPos, const double obstSize )
{
	const RoadSegment *roadSegmentAtObstacle = mRoadGen->segmentAt(obstRoadParam);
	if( !roadSegmentAtObstacle )
	{
		qWarning() << "No roadSegment at obstacle@"<<obstRoadParam;
		return 0.0;
	}
	double roadWidth = roadSegmentAtObstacle->widthAt( obstRoadParam-roadSegmentAtObstacle->startLocation().parameter );

	double obstacleCrossPosRelToCar = carCrossPos - obstCrossPos;
	double minDistanceToObst = (obstSize/2+mCar->size().width()/2+obstacleAvoidanceDistance);

	double targetPoint = carCrossPos;	// init

	// do we crash into it?
	if( qAbs(obstacleCrossPosRelToCar) < minDistanceToObst )
	{ // we are on a crash trajectory!
		QLineF spaceOnLeft(-roadWidth/2,0, obstCrossPos-obstSize/2,0);
		QLineF spaceOnRight(obstCrossPos+obstSize/2,0, roadWidth/2,0 );
		double neededSpace = mCar->size().width()+obstacleAvoidanceDistance;

		// Dooooooooooooooooomed?
		if( spaceOnRight.length() < neededSpace && spaceOnLeft.length() < neededSpace)
		{
			qWarning() << "Car doesn't fit on either side of obstacle (@" << obstRoadParam << ")!";
			emit unavoidableCrashDetected(obstRoadParam);
			return targetPoint;
		}

		// then let's offset!
		double targetPosRelToObstacle = obstSize/2+obstacleAvoidanceDistance+mCar->size().width()/2;

		// do we fit in the space on this side if we offset? If not, offset to the other way...
		if( obstacleCrossPosRelToCar >= 0 )
		{
			if( spaceOnLeft.length() >= neededSpace )
				{ targetPosRelToObstacle *= -1; }
		}
		else
		{
			if( spaceOnRight.length() < neededSpace )
				{ targetPosRelToObstacle *= -1; }
		}
		targetPoint = obstCrossPos+targetPosRelToObstacle;
	}

	return targetPoint;
}

void CarDriver::insertRoadFeaturePoint(CarDriver::roadFeaturePoint *rfp)
{
	int i = 0;
	for( ; i<mRoadFeaturePoints.size(); ++i )
	{
		if( mRoadFeaturePoints.at(i)->roadParam > rfp->roadParam )
		{
			break;
		}
	}
	mRoadFeaturePoints.insert(i, rfp);
}

double CarDriver::currentCentripetalAccel() const
{
	// old
//	const RoadSegment *currentRoadSegment = mRoadGen->segmentAtOdo(mCar->odometer());
//	if( !currentRoadSegment )
//	{
//		qWarning("No current roadSegment in CarDriver::currentCentripetalAccel()!");
//		return 0;
//	}
//	double currentRoadRadius = currentRoadSegment->radius();
//	double currentCentripetalAccel = 0;
//	if( currentRoadRadius != 0 )
//		{ currentCentripetalAccel = pow(mCar->speed(),2) / currentRoadRadius; }
//	return currentCentripetalAccel;

	double currentTurnCurvature = mCar->turnCurvature();
	double currentCentripetalAccel = 0;
	if( currentTurnCurvature != 0 )
		{ currentCentripetalAccel = pow(mCar->speed(),2) * currentTurnCurvature; }
	return currentCentripetalAccel;
}

QVector2D CarDriver::currentNetAccel() const
{
	QVector2D tangentAccel(0,mCar->acceleration());
	QVector2D centripAccel(currentCentripetalAccel(),0);
	return tangentAccel + centripAccel;
}

double CarDriver::currentMaxPossibleSpeedOnBend() const
{
	const RoadSegment *currentRoadSegment = mRoadGen->segmentAt(mCar->odometer());
	if( !currentRoadSegment )
	{
		qWarning("No current roadSegment in CarDriver::currentMaxPossibleSpeedOnBend()!");
		return 0;
	}
	double currentRoadRadius = currentRoadSegment->radiusAbs();
	double maxSpeed = 1000000;
	if( currentRoadRadius > 0 )
		{ maxSpeed = sqrt( mCar->frictionCoeffStatic() * 9.81 * currentRoadSegment->radiusAbs() ); }
	return maxSpeed;
}

double CarDriver::cruiseSpeedMps() const
	{ return mCruiseSpeed; }

double CarDriver::cruiseSpeedKmh() const
	{ return mCruiseSpeed * 3.6; }

void CarDriver::setCruiseSpeedMps(double cruiseSpeedMps)
{
	if( mTargetSpeed == mCruiseSpeed )
		{ mTargetSpeed = cruiseSpeedMps; }
	mCruiseSpeed = cruiseSpeedMps;

}

void CarDriver::setCruiseSpeedKmh(double cruiseSpeedKmh)
{
	setCruiseSpeedMps( cruiseSpeedKmh / 3.6 );
}

double CarDriver::targetSpeedKmh() const
{
	return mTargetSpeed * 3.6;
}

double CarDriver::targetSpeedMps() const
{
	return mTargetSpeed;
}
