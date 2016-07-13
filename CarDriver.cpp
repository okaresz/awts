#include "CarDriver.h"
#include <QDebug>
#include <QLineF>
#include <QRectF>
#include <SettingsManager.h>

const double CarDriver::obstacleAvoidanceDistance = 0.5;

CarDriver::CarDriver(Car *car, RoadGenerator *roadGen, QObject *parent) : QObject(parent),
	mCar(car), mRoadGen(roadGen), mCruiseSpeed(14), mTargetSpeed(mCruiseSpeed), mSteeringControl({0.0,0.0,0.0,0.0,0.0,0.0}), mAccelerationMode(ProportionalAcceleration),
	mThreats({0,0}), mCrashed(false), mTractionLost(false), mManualDrive(false), mBendMaxSpeedSafetyFactor(0.8)
{
	if( !SettingsManager::instance()->contains("carDriver/steerControlP") )
		{ SettingsManager::instance()->setValue("carDriver/steerControlP", 0.0); }
	else
		{ mSteeringControl.P = SettingsManager::instance()->value("carDriver/steerControlP").toDouble(); }

	if( !SettingsManager::instance()->contains("carDriver/steerControlI") )
		{ SettingsManager::instance()->setValue("carDriver/steerControlI", 0.0); }
	else
		{ mSteeringControl.I = SettingsManager::instance()->value("carDriver/steerControlI").toDouble(); }

	if( !SettingsManager::instance()->contains("carDriver/steerControlD") )
		{ SettingsManager::instance()->setValue("carDriver/steerControlD", 0.0); }
	else
		{ mSteeringControl.D = SettingsManager::instance()->value("carDriver/steerControlD").toDouble(); }

	if( !SettingsManager::instance()->contains("carDriver/manualDriveEnabled") )
		{ SettingsManager::instance()->setValue("carDriver/manualDriveEnabled", mManualDrive); }
	else
		{ mManualDrive = SettingsManager::instance()->value("carDriver/manualDriveEnabled").toBool(); }
}

void CarDriver::simUpdate(const quint64 simTime, const double travel, const double carCrossPosOnRoad, const double carHeadingOnRoad)
{
	static quint64 prevSimTime = 0;
	int simdTms = simTime - prevSimTime;
	double simdTSec	= (double)simdTms/1000.0;

	// some facts
	double currentSpeed = mCar->speed();
	QVector2D currentAcceleration = currentNetAccel();
	double maxAccelAllowedByFriction = mCar->maxAcceleration();

	// CHECK TRACTION LOSS
	if( currentAcceleration.length() > maxAccelAllowedByFriction && !mTractionLost )
	{
		mTractionLost = true;
		emit tractionLost(travel);
		qWarning() << "TRACTION LOST at travel=" << travel << "(" <<currentAcceleration.length() << ">" <<maxAccelAllowedByFriction<<")";
	}

	// CHECK IF CAR HAS LEFT THE ROAD
	const RoadSegment *segAtCar = mRoadGen->segmentAt(travel);
	if( segAtCar != nullptr )
	{
		if( !mCrashed && (qAbs(carCrossPosOnRoad)+mCar->size().width()/2 > segAtCar->widthAt(travel-segAtCar->startLocation().parameter)/2) )
		{
			mCrashed = true;
			emit crashed(travel);
			qWarning() << "CRASHED at travel=" << travel << "(car left the road)";
		}
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
			rfp->segment = nullptr;
			rfp->minRoadWidth = seg->minWidth();
			if( seg->isBend() )
			{
				rfp->type = BendStartFeaturePoint;
				rfp->curvature = seg->curvature();
				rfp->maxSpeed = sqrt( maxAccelAllowedByFriction * seg->radiusAbs() ) * mBendMaxSpeedSafetyFactor;
			}
			else
			{
				rfp->type = StraightStartFeaturePoint;
				rfp->maxSpeed = 10000000.0;	// dirty hack for infinite
			}
			mRoadFeaturePoints.append(rfp);
		}
	}
	// scan obstacles, generate featurePoints
	for( int i=0; i<visibleObstacles.size(); ++i )
	{
		const RoadObstacle *obst = &visibleObstacles.at(i);
		const RoadSegment *segAtObst = mRoadGen->segmentAt(obst->roadParam());
		if( !segAtObst )
		{
			qWarning() << "No segment at obstacle@"<<obst->roadParam();
			continue;
		}
		double obstCrossPos = obst->normalPos()*segAtObst->widthAt( obst->roadParam()-segAtObst->startLocation().parameter )/2;

		if( obst->roadParam() >= featurePointGenHorizon )
		{
			roadFeaturePoint *rfp = new roadFeaturePoint;
			rfp->segment = segAtObst;
			rfp->type = ObstacleFeaturePoint;
			rfp->roadParam = obst->roadParam();
			rfp->crossPos = obstCrossPos;
			rfp->size = obst->size();
			rfp->maxSpeed = 10000000.0;	// dirty hack for infinite
			insertRoadFeaturePoint( rfp );
		}

		// check obstacle collision (y axis points up)
		double obstRadius = obst->size()/2;
		QRectF obstacleRect( obstCrossPos-obstRadius, obst->roadParam()-obstRadius, obstRadius*2, obstRadius*2 );
		QSizeF carSize(mCar->size());
		QRectF carRect( carCrossPosOnRoad-carSize.width()/2, travel-carSize.height()/2, carSize.width(), carSize.height() );
		if( carRect.intersects(obstacleRect) )
		{
			mCrashed = true;
			emit crashed(travel);
			qWarning() << "CRASHED at travel=" << travel << "(car crashed into obstacle)";
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
				mSteeringControl.targetCrossPos = targetCrossPosToAvoid;
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
	qInfo() << "accVect=" << currentAcceleration << "= |" << currentAcceleration.length() << "|";
	qInfo() << "speed=" << currentSpeed*3.6;

	// FEATURE POINT PASSED -------------------------
	// if a feature point is passed (left behind, but not yet deleted at the end of this simUpdate),
	// set it's target speed and proportional acc, set steering feedForward and targetCrossPos if needed 'cause of width change
	roadFeaturePoint *lastPassedFP = nullptr;
	for( int i=0; i<mRoadFeaturePoints.size(); ++i )
	{
		if( mRoadFeaturePoints.at(i)->roadParam < travel )
			{ lastPassedFP = mRoadFeaturePoints.at(i); }
	}
	if( lastPassedFP )
	{
		if( lastPassedFP->type != ObstacleFeaturePoint )
		{
			if( lastPassedFP->type == StraightStartFeaturePoint )
				{ mAccelerationMode = ProportionalAcceleration; }
			mTargetSpeed = qBound(0.0,lastPassedFP->maxSpeed,mCruiseSpeed);
		}

		if( lastPassedFP->type == StraightStartFeaturePoint )
		{
			// set steering feedForward for coming segment, according to the radius on the current crossPos
			mSteeringControl.feedForward = 0.0;
			qInfo() << "Passed FeaturePoint@"<<lastPassedFP->roadParam<<", set targetSpeed="<<mTargetSpeed<<", steerFeedfwd=0.0";
		}
		if( lastPassedFP->type == StraightStartFeaturePoint || lastPassedFP->type == ObstacleFeaturePoint )
		{
			// if car would leave road, change targetCrossPos
			const RoadSegment *currentSegment = mRoadGen->segmentAt(travel);
			double maxCrossPos = (currentSegment->minWidth()/2-mCar->size().width()/2);
			if( qAbs(mSteeringControl.targetCrossPos) > maxCrossPos )
			{
				if( mSteeringControl.targetCrossPos < 0.0 )
					{ mSteeringControl.targetCrossPos = -maxCrossPos; }
				else
					{ mSteeringControl.targetCrossPos = maxCrossPos; }
				qInfo() << "Passed FeaturePoint@"<<lastPassedFP->roadParam<<", set targetSpeed="<<mTargetSpeed<<", update targetCrossPos to stay on road:"<<mSteeringControl.targetCrossPos;
			}
		}
		if( lastPassedFP->type == BendStartFeaturePoint )
		{
			double bendRadius = 1/lastPassedFP->curvature;
			mSteeringControl.feedForward = mCar->wheelAngleAtTurnCurvature( 1/(bendRadius-carCrossPosOnRoad) );
			qInfo() << "Passed FeaturePoint@"<<lastPassedFP->roadParam<<", set targetSpeed="<<mTargetSpeed<<", steerFeedfwd="<<mSteeringControl.feedForward;
		}
	}

	// Search for and set nearest brakePoint -------------------------------------------------------------------------
	double nearestBrakePointRoadParam = -1;
	roadFeaturePoint *featurePointOfNearestBrake = nullptr;
	bool brakeSetCopyOfFeaturePointOfNearestBrake = false;
	for( int i=0; i<mRoadFeaturePoints.size(); ++i )
	{
		double fpMaxSpeed = mRoadFeaturePoints.at(i)->maxSpeed;
		if( mRoadFeaturePoints.at(i)->type == BendStartFeaturePoint )
		{
			double bendRadiuAtCurrentCrossPos = 1/mRoadFeaturePoints.at(i)->curvature - carCrossPosOnRoad;
			fpMaxSpeed = sqrt( maxAccelAllowedByFriction * qAbs(bendRadiuAtCurrentCrossPos) ) * mBendMaxSpeedSafetyFactor;
		}
		if( fpMaxSpeed < currentSpeed )
		{
			double decelTimeForNextBend = (currentSpeed-fpMaxSpeed) / maxSafeTangentialAccel.deceleration;
			double decelDistance = currentSpeed*decelTimeForNextBend - maxSafeTangentialAccel.deceleration * pow(decelTimeForNextBend,2) / 2; // minus, because deceleration
			decelDistance += simdTSec*currentSpeed;	// consider simulation resolution... (should add dTravel, not dOdo, but whatever)
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
		qInfo() << "Target speed set to " << mTargetSpeed;
	}

	// calculate acceleration -------------------------------------------------------------------------
	double actualAcceleration = 0;
	static double previousSpeed = 0;

	if( mManualDrive )
	{
		if( mKeyStatus.value(Qt::Key_Up,false) )
			{ actualAcceleration = maxSafeTangentialAccel.acceleration; }
		else if( mKeyStatus.value(Qt::Key_Down,false) )
			{ actualAcceleration = -maxSafeTangentialAccel.deceleration; }
		else
			{ actualAcceleration = 0.0; }
	}
	else
	{
		double speedError = mTargetSpeed - currentSpeed;
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
	}

	/* STEERING CONTROL ====================================================================================
	 * -*/
	static double previousCrossPosError = 0;
	double steerAngleRad = 0.0;
	if( mManualDrive )
	{
		double arrowWheelAngleDeg = 20.0;
		if( mKeyStatus.value(Qt::Key_Left,false) )
			{ steerAngleRad = -arrowWheelAngleDeg/180.0*M_PI; }
		else if( mKeyStatus.value(Qt::Key_Right,false) )
			{ steerAngleRad = arrowWheelAngleDeg/180.0*M_PI; }
		else
			{ steerAngleRad = mSteeringControl.feedForward; }
	}
	else
	{
		double crossPosError = mSteeringControl.targetCrossPos - carCrossPosOnRoad;
		double integral = mSteeringControl.I * crossPosError * simdTSec;
		steerAngleRad = mSteeringControl.feedForward +
				crossPosError * mSteeringControl.P +
				mSteeringControl.integratorSum+integral +
				mSteeringControl.D*(crossPosError-previousCrossPosError)/simdTSec;

		double currentMaxSteerAngle = mCar->maxWheelAngleAtCurrentAcceleration(actualAcceleration)*0.9;
		if( qAbs(steerAngleRad) <= currentMaxSteerAngle )
		{
			mSteeringControl.integratorSum += integral;		// integrator anti windup
		}
		else
		{
			if( steerAngleRad < -currentMaxSteerAngle )
				{ steerAngleRad = -currentMaxSteerAngle; }
			else if( steerAngleRad > currentMaxSteerAngle )
				{ steerAngleRad = currentMaxSteerAngle; }
		}
		previousCrossPosError = crossPosError;
	}

	// set car params =======================================
	mCar->steer( steerAngleRad );
	mCar->accelerate( actualAcceleration );
	qInfo() << "Car wheelAngle (deg): " << mCar->wheelAngle()/M_PI*180.0;
	qInfo() << "Set car acc: " << actualAcceleration;
	qInfo() << "Target speed = " << mTargetSpeed*3.6;
	qInfo() << "TargetCrossPos = " << mSteeringControl.targetCrossPos;

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

	prevSimTime = simTime;
	qInfo() << "------------------------------------------------------";
}

void CarDriver::setTargetCrossPos(double crossPos)
{
	mSteeringControl.targetCrossPos = crossPos;
}

void CarDriver::setSteeringControlFeedForward(const double value)
{
	mSteeringControl.feedForward = value;
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

	double obstacleCrossPosRelToCar = obstCrossPos - carCrossPos;
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
	return pow(mCar->speed(),2) * mCar->turnCurvature();
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
		{ maxSpeed = sqrt( mCar->maxAcceleration() * currentRoadSegment->radiusAbs() ); }
	return maxSpeed;
}

bool CarDriver::keyboardEvent(Qt::Key key, bool pressed)
{
	bool handled = false;
	switch(key)
	{
	case Qt::Key_Left:
		mKeyStatus[Qt::Key_Left] = pressed;
		handled = true;
		break;
	case Qt::Key_Right:
		mKeyStatus[Qt::Key_Right] = pressed;
		handled = true;
		break;
	case Qt::Key_Up:
		mKeyStatus[Qt::Key_Up] = pressed;
		handled = true;
		break;
	case Qt::Key_Down:
		mKeyStatus[Qt::Key_Down] = pressed;
		handled = true;
		break;
	default:;
	}
	return handled;
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
