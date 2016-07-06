#include "CarDriver.h"
#include <QDebug>
#include <QLineF>

/* NOTES
 * - left bend radius is negative.
 * - a turn span angle is always positive.*/

const double CarDriver::obstacleAvoidanceDistance = 0.5;

CarDriver::CarDriver(Car *car, RoadGenerator *roadGen, QObject *parent) : QObject(parent),
	mCar(car), mRoadGen(roadGen), mCruiseSpeed(14), mTargetSpeed(mCruiseSpeed), mCarCrossPos(0), mAccelerationMode(ProportionalAcceleration),
	mThreats({0,0}), mCrashed(false), mTractionLost(false), mBendMaxSpeedSafetyFactor(0.8)
{

}

void CarDriver::simUpdate(const quint64 simTime)
{
	// some facts
	double currentSpeed = mCar->speed();
	QVector2D currentAcceleration = currentNetAccel();
	double odometer = mCar->odometer();
	double maxAccelAllowedByFriction = mCar->frictionCoeffStatic() * 9.81;

	if( currentAcceleration.length() > maxAccelAllowedByFriction && !mTractionLost )
	{
		mTractionLost = true;
		emit tractionLost(odometer);
		qWarning() << "TRACTION LOST at odo=" << odometer;
	}

	/* --- FEATURE POINT GENERATION -------------------------------------------------------------------*/
	QQueue<RoadSegment> visibleRoad( mRoadGen->visibleRoad(odometer) );
	QQueue<RoadObstacle> visibleObstacles( mRoadGen->visibleObstacles(odometer) );
	static double featurePointGenHorizon = 0;

	for( int i=0; i<visibleRoad.size(); ++i )
	{
		// find the segment at featurePointGenHorizon
		if( visibleRoad.at(i).odoStartLoc() >= featurePointGenHorizon )
		{
			const RoadSegment *seg = &visibleRoad.at(i);
			roadFeaturePoint *rfp = new roadFeaturePoint;
			rfp->odoPos = seg->odoStartLoc();
			if( seg->isBend() )
			{
				rfp->type = BendStartFeaturePoint;
				rfp->radius = seg->radius();
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
	featurePointGenHorizon = visibleRoad.last().odoEndLoc();

	/* SPEED & ACCELERATION CONTROL. ==============================================================================================================
	 * Algorithm: the following sequence is followed:
	 * - if a featurePoint is passed (left behind), set it's speed as target and set ProportionalAccel method.
	 * - till the nearest brakePoint, continue to go with/accel to targetSpeed (given by last left-behind featurePoint)
	 * - calculate closest brakePoint from roadFeaturePoints
	 * - if a brakePoint reached, set MaxAccel method and targetSpeed of the featurePoint belonging to the brakePoint
	 * */

	double unavoidableTractionLossAtOdo = -1.0;
	double unavoidableCrashAtOdo = -1.0;

	//more facts
	double centripetalAccel = currentAcceleration.x();
	double maxTangentialAccel;
	if( pow(maxAccelAllowedByFriction,2) < pow(centripetalAccel,2) )
		{ maxTangentialAccel = 0; }
	else
		{ maxTangentialAccel = sqrt( pow(maxAccelAllowedByFriction,2) - pow(centripetalAccel,2) ); }
	if( maxTangentialAccel < 0.05 )
		{ qWarning() << "Max possible tangential accel is practically zero!"; }
	double maxSafeTangentialAccel = maxTangentialAccel * 0.95; // don't go on the limit
	qInfo() << "accVect=" << currentAcceleration;
	qInfo() << "speed=" << currentSpeed*3.6;

	// if a feature point is passed (left behind, but not yet deleted at the end of this simUpdate), set it's target speed and proportional acc
	roadFeaturePoint *lastPassedFP = nullptr;
	for( int i=0; i<mRoadFeaturePoints.size(); ++i )
	{
		if( mRoadFeaturePoints.at(i)->odoPos < odometer )
			{ lastPassedFP = mRoadFeaturePoints.at(i); }
	}
	if( lastPassedFP )
	{
		mAccelerationMode = ProportionalAcceleration;
		mTargetSpeed = qBound(0.0,lastPassedFP->maxSpeed,mCruiseSpeed);
		qInfo() << "Passed FeaturePoint@"<<lastPassedFP->odoPos<<", set targetSpeed="<<mTargetSpeed;

		// TODO work on this
		if( lastPassedFP->type == BendStartFeaturePoint )
			{ mCar->steerForTurnRadius(lastPassedFP->radius); }
		else if( lastPassedFP->type == StraightStartFeaturePoint )
			{ mCar->steer(0); }
	}

	// Search for and set nearest brakePoint -------------------------------------------------------------------------
	double nearestBrakePointOdo = -1;
	roadFeaturePoint *featurePointOfNearestBrake = nullptr;
	bool brakeSetCopyOfFeaturePointOfNearestBrake = false;
	for( int i=0; i<mRoadFeaturePoints.size(); ++i )
	{
		if( mRoadFeaturePoints.at(i)->maxSpeed < currentSpeed )
		{
			double decelTimeForNextBend = (currentSpeed-mRoadFeaturePoints.at(i)->maxSpeed) / maxSafeTangentialAccel;
			double decelDistance = currentSpeed*decelTimeForNextBend - maxSafeTangentialAccel * pow(decelTimeForNextBend,2) / 2; // minus, because deceleration
			double brakeOdoPoint = mRoadFeaturePoints.at(i)->odoPos - decelDistance;
			if( nearestBrakePointOdo < 0 || brakeOdoPoint < nearestBrakePointOdo )
			{
				nearestBrakePointOdo = brakeOdoPoint;
				featurePointOfNearestBrake = mRoadFeaturePoints.at(i);
				brakeSetCopyOfFeaturePointOfNearestBrake = featurePointOfNearestBrake->brakePointSet;
				featurePointOfNearestBrake->brakePointSet = true;
			}
		}
	}

	// this section is active from the detection of the feature/brake point till car reaches target speed
	if( featurePointOfNearestBrake && nearestBrakePointOdo < odometer )
	{
		if( !brakeSetCopyOfFeaturePointOfNearestBrake )
		{ // set threat signals
			if( featurePointOfNearestBrake )
			{
				if( featurePointOfNearestBrake->type == BendStartFeaturePoint )
				{
					++mThreats.tractionLoss;
					if( unavoidableTractionLossAtOdo < 0 || unavoidableTractionLossAtOdo > featurePointOfNearestBrake->odoPos )
						{ unavoidableTractionLossAtOdo = featurePointOfNearestBrake->odoPos; }
				}
				else
				{
					++mThreats.crash;
					if( unavoidableCrashAtOdo < 0 || unavoidableCrashAtOdo > featurePointOfNearestBrake->odoPos )
						{ unavoidableCrashAtOdo = featurePointOfNearestBrake->odoPos; }
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
	short int accelSign = 1;
	if( speedError < 0 ) {  accelSign = -1; }

	static double previousSpeed = 0;

	if( mAccelerationMode == MaxAcceleration )
	{
		if( (previousSpeed > mTargetSpeed && currentSpeed < mTargetSpeed) || (previousSpeed < mTargetSpeed && currentSpeed > mTargetSpeed) )
		{
			mAccelerationMode = ProportionalAcceleration;
			actualAcceleration = 0.0;
		}
		else
			{ actualAcceleration = accelSign * maxSafeTangentialAccel; }
	}
	else if( mAccelerationMode == ProportionalAcceleration )
	{
		actualAcceleration = qBound(-maxSafeTangentialAccel, speedError * 0.8, maxSafeTangentialAccel);	// P controller
	}

	// set car params -------------------------------
	mCar->accelerate( actualAcceleration );
	qInfo() << "Set car acc: " << actualAcceleration;

	// end stuff -----------------------------

	// signal threats
	if( !mTractionLost && mThreats.tractionLoss > 0 )
	{
		qWarning() << "Unavoidable traction loss at odo " << unavoidableTractionLossAtOdo;
		emit unavoidableTractionLossDetected(unavoidableTractionLossAtOdo);
	}
	if( !mCrashed && mThreats.crash > 0 )
	{
		qWarning() << "Unavoidable crash at odo " << unavoidableCrashAtOdo;
		emit unavoidableCrashDetected(unavoidableCrashAtOdo);
	}
	mThreats.tractionLoss = 0;
	mThreats.crash = 0;

	// delete left-behind feature point(s)
	while( !mRoadFeaturePoints.isEmpty() && mRoadFeaturePoints.first()->odoPos < odometer )
		{ delete mRoadFeaturePoints.takeFirst(); }

	previousSpeed = currentSpeed;

	qInfo() << "------------------------------------------------------";
}

void CarDriver::planTrajectory(double odometer)
{
	/* TRAJECTORY PLANNING
	*  Start from the end of the current trajectory, and check if there is road ahead of trajectory.
	*  If so, lengthen the trajectory or append a new section if another section type is needed.*/

	QQueue<RoadSegment> visibleRoad( mRoadGen->visibleRoad(odometer) );
	QPointF trajEndPoint(mTrajectory.last()->endPos());
	double trajectoryOdoEndLoc = trajEndPoint.y();
	TrajectorySection *lastTrajSection = mTrajectory.last();
	while( qAbs(visibleRoad.last().odoEndLoc()-trajectoryOdoEndLoc) > 0.01 )	// try to avoid exact floating point number comparison...
	{
		// get the segment at the trajectory end
		const RoadSegment *segAtTrajEnd = nullptr;
		for( int i=0; i<visibleRoad.size(); ++i )
		{
			// if trajectory planning doesn't advance her, couse of float rounding errors, the trajectory never reaches the road end,
			// implement roadSegment and trajectorySection indexing
			if( (visibleRoad.at(i).odoStartLoc()<=trajectoryOdoEndLoc) && (visibleRoad.at(i).odoEndLoc()>trajectoryOdoEndLoc) )
			{
				segAtTrajEnd = &visibleRoad.at(i);
				break;
			}
		}

		if( !segAtTrajEnd )
		{
			qWarning() << "No roadSegment at trajectory end! Weird... (float overrounding?)";
			break;
		}

		if( segAtTrajEnd->isBend() )
		{
			/* there could be BendLaneShift here as well, but as laneShift is generated only if it has an endpoint,
			 * it cannot be continued, so a new segment must be appended regardless.*/
			if( (lastTrajSection->type() == TrajectorySection::BendSection) &&
				( ((TrajectorySectionBend*)lastTrajSection)->radius() == segAtTrajEnd->radius() ) )
			{
				//continue bendsection
				lastTrajSection->setLength( segAtTrajEnd->odoEndLoc()-trajectoryOdoEndLoc );
			}
			else
			{
				// append bendsection
				mTrajectory.append( new TrajectorySectionBend(trajEndPoint, segAtTrajEnd->radius(), segAtTrajEnd->odoEndLoc()-trajectoryOdoEndLoc) );
			}

		}
		else
		{
			// again, BendLaneShift cannot be continued, so a new segment must be appended regardless.
			if( lastTrajSection->type() != TrajectorySection::StraightSection )
			{
				mTrajectory.append( new TrajectorySectionStraight(trajEndPoint, segAtTrajEnd->odoEndLoc()-trajectoryOdoEndLoc) );
			}
			else
			{
				// continue straight section
				lastTrajSection->setLength( segAtTrajEnd->odoEndLoc()-trajectoryOdoEndLoc );
			}
		}
	}
}

double CarDriver::odoEndOfTrajectory() const
{
	return mTrajectory.last()->endPos().y();
}

double CarDriver::calculateObstacleAvoidancePoint( const RoadObstacle* obstacle )
{
	const RoadSegment *roadSegmentAtObstacle = mRoadGen->segmentAtOdo(obstacle->odoPos());
	if( !roadSegmentAtObstacle )
	{
		qWarning() << "No roadSegment at obstacle@"<<obstacle->odoPos();
		return 0.0;
	}
	double roadWidth = roadSegmentAtObstacle->widthAt( obstacle->odoPos()-roadSegmentAtObstacle->odoStartLoc() );
	double obstacleCrossPos = obstacle->normalPos()*roadWidth/2;

	double obstacleCrossPosRelToCar = mCarCrossPos - obstacleCrossPos;
	double minDistanceToObst = (obstacle->size()/2+mCar->size().width()/2+obstacleAvoidanceDistance);
	double targetPoint = mCarCrossPos;

	// do we crash into it?
	if( qAbs(obstacleCrossPosRelToCar) < minDistanceToObst )
	{ // we are on a crash trajectory!
		QLineF spaceOnLeft(-roadWidth/2,0, obstacleCrossPos-obstacle->size()/2,0);
		QLineF spaceOnRight(obstacleCrossPos+obstacle->size()/2,0, roadWidth/2,0 );
		double neededSpace = mCar->size().width()+obstacleAvoidanceDistance;

		// Dooooooooooooooooomed?
		if( spaceOnRight.length() < neededSpace && spaceOnLeft.length() < neededSpace)
		{
			qWarning() << "Car doesn't fit on either side of obstacle (@" << obstacle->odoPos() << ")!";
			emit unavoidableCrashDetected(obstacle->odoPos());
			return targetPoint;
		}

		// then let's offset!
		double targetPosRelToObstacle = obstacle->size()/2+obstacleAvoidanceDistance+mCar->size().width()/2;

		// do we fit in the space on this side if we offset? If not, offset to the other way...
		if( obstacleCrossPosRelToCar >= 0 )
			{ targetPosRelToObstacle *= -1; }
		if( spaceOnRight.length() >= neededSpace )
		{
			targetPoint = obstacleCrossPos+targetPosRelToObstacle;
		}
		else
		{
			targetPoint = obstacleCrossPos-targetPosRelToObstacle;
		}
	}

	return targetPoint;
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

	double currentRoadRadius = mCar->turnRadius();
	double currentCentripetalAccel = 0;
	if( currentRoadRadius != 0 )
		{ currentCentripetalAccel = pow(mCar->speed(),2) / currentRoadRadius; }
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
	const RoadSegment *currentRoadSegment = mRoadGen->segmentAtOdo(mCar->odometer());
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
