#include "CarDriver.h"
#include <QDebug>
#include <QLineF>

const double CarDriver::obstacleAvoidanceDistance = 0.5;

CarDriver::CarDriver(Car *car, RoadGenerator *roadGen, QObject *parent) : QObject(parent),
	mCar(car), mRoadGen(roadGen), mCruiseSpeed(14), mTargetSpeed(mCruiseSpeed), mCarCrossPos(0)
{

}

void CarDriver::simUpdate(const quint64 simTime)
{
	/* TRAJECTORY KEEPING
	 * warning: if you cut off from the beginning of a trajectory segment, set startPos accordingly!*/

	/* SPEED CONTROL.
	 * Algorithm: the following wequence is followed:
	 * 0. target speed is cruiseSpeed
	 * 1. check the next bend on the road (not counting the one we are possibly on) and calculate the maximum speed we can take the turn with.
	 *	  Check if we should already start braking, using the current largest possible deceleration. If we should, set the calculated target speed.
	 *	  If no bend visible, leave cruiseSpeed as target.
	 * 2. check if target speed is not greater than current max possible speed to stay on the road if we are on a bend.
	 * 3. try to reach target speed, calculate acceleration for that, and limit it by the forces resulting on the current bend if we are on one.
	 * 4. give the acceleration value to car to do he acceleration.*/

	// some facts
	double currentSpeed = mCar->speed();
	double odometer = mCar->odometer();
	double maxAccelAllowedByFriction = mCar->frictionCoeffStatic() * 9.81;
	double centripetalAccel = currentCentripetalAccel();
	double maxTangentialAccel;
	double maxTangentialAccelHeadroomFactor = 0.95; // don't go on the limit
	if( pow(maxAccelAllowedByFriction,2) < pow(centripetalAccel,2) )
		{ maxTangentialAccel = 0; }
	else
		{ maxTangentialAccel = sqrt( pow(maxAccelAllowedByFriction,2) - pow(centripetalAccel,2) ); }
	if( maxTangentialAccel < 0.05 )
		{ qWarning() << "Max tangential accel is too low! (" << maxTangentialAccel << ") Cant slow down!"; }
	double maxSafeTangentialAccel = maxTangentialAccel * maxTangentialAccelHeadroomFactor;
	double bendSpeedHeadroomFactor = 0.8; // with full speed in bend, there can be no deceleration due to net force being already at maximum allowed by friction
	double extraDecelDistanceForNextBend = 5.0;	// extra distance to add to decelDistance -> start braking sooner
	qInfo() << "centripAcc=" << centripetalAccel;
	qInfo() << "sumAcc=" << currentNetAccel();
	qInfo() << "speed=" << currentSpeed*3.6;

	// calc speed for next bend, decide if we should start decelerating
	const RoadSegment *nextBend = mRoadGen->nextBend( mCar->odometer() );
	static RoadSegment *lastCalculatedNextBend;
	if( nextBend /*&& nextBend != lastCalculatedNextBend*/ )
	{
		qInfo() << "Calc speed for next bend (@" << nextBend->odoStartLoc() << ")";
		lastCalculatedNextBend = (RoadSegment*)nextBend;
		double maxSpeedForNextBend = sqrt( maxAccelAllowedByFriction * nextBend->radiusAbs() );
		double maxSafeSpeedForNextBend = maxSpeedForNextBend * bendSpeedHeadroomFactor;
		if( maxSafeSpeedForNextBend < currentSpeed )
		{
			double decelTimeForNextBend = (currentSpeed-maxSafeSpeedForNextBend) / maxSafeTangentialAccel;
			double decelDistanceForNextBend = currentSpeed*decelTimeForNextBend - maxSafeTangentialAccel * pow(decelTimeForNextBend,2) / 2; // minus, because deceleration
			decelDistanceForNextBend += extraDecelDistanceForNextBend;
			qInfo() << "\tdist2Next: " << nextBend->odoStartLoc()-odometer << ", maxSafeSpeed: " << maxSafeSpeedForNextBend*3.6 << ", decelDistance: " << decelDistanceForNextBend;

			/// TODO: check if decelDistance is smaller than the available road ahead
			/// TODO: set accelVal here?

			if( decelDistanceForNextBend >= (nextBend->odoStartLoc()-odometer) )
			{
				mTargetSpeed = maxSafeSpeedForNextBend;
				qInfo() << "\tlimit speed NOW for next bend kmh: " << mTargetSpeed*3.6;
			}
			else
			{
				qInfo() << "\tbend is still far ahead...";
			}
		}
		else
		{
			mTargetSpeed = mCruiseSpeed;
			qInfo() << "\tcurrent speed is OK for next bend, set cruise";
		}
	}
	else
	{
		mTargetSpeed = mCruiseSpeed;
	}

	// chack for max bend speed due to centripetal forces
	double currentMaxPossibleBendSpeed = currentMaxPossibleSpeedOnBend();
	if( mTargetSpeed > currentMaxPossibleBendSpeed )
	{
		mTargetSpeed = currentMaxPossibleBendSpeed * bendSpeedHeadroomFactor;
		qInfo() << "Limit target speed due to current bend to " << mTargetSpeed*3.6 << " km/h";
	}

	// P control won't do -> next bend algorithm calculates with max tangential accel, so use that -> but now there is always acceleration....
	double speedControlP = 0.5;
	double speedError = mTargetSpeed-currentSpeed;
	double accelerateVal;
	if( fabs(speedError) < 0.5 )
		{ accelerateVal = speedError*speedControlP; }
	else
	{
		accelerateVal = maxSafeTangentialAccel;
		if( mTargetSpeed < currentSpeed )
			{ accelerateVal *= -1; }
	}

	if( accelerateVal > maxSafeTangentialAccel )
		{ accelerateVal = maxSafeTangentialAccel;	}
	if( accelerateVal < -maxSafeTangentialAccel )
		{ accelerateVal = -maxSafeTangentialAccel;	}

	// accelerate / decelerate
	qInfo() << "Set acceleration: " << accelerateVal;
	mCar->accelerate( accelerateVal );

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
	const RoadSegment *currentRoadSegment = mRoadGen->segmentAtOdo(mCar->odometer());
	if( !currentRoadSegment )
	{
		qWarning("No current roadSegment in CarDriver::currentCentripetalAccel()!");
		return 0;
	}
	double currentRoadRadius = currentRoadSegment->radius();
	double currentCentripetalAccel = 0;
	if( currentRoadRadius != 0 )
		{ currentCentripetalAccel = pow(mCar->speed(),2) / currentRoadRadius; }
	return currentCentripetalAccel;
}

double CarDriver::currentNetAccel() const
{
	return sqrt( pow(mCar->acceleration(),2) + pow(currentCentripetalAccel(),2));
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
