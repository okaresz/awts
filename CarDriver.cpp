#include "CarDriver.h"
#include <QDebug>

CarDriver::CarDriver(Car *car, RoadGenerator *roadGen, QObject *parent) : QObject(parent),
	mCar(car), mRoadGen(roadGen), mCruiseSpeed(14), mTargetSpeed(mCruiseSpeed)
{

}

void CarDriver::simUpdate(const quint64 simTime)
{
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
		{ qDebug() << "Max tangential accel is too low! (" << maxTangentialAccel << ") Cant slow down!"; }
	double maxSafeTangentialAccel = maxTangentialAccel * maxTangentialAccelHeadroomFactor;
	double bendSpeedHeadroomFactor = 0.8; // with full speed in bend, there can be no deceleration due to net force being already at maximum allowed by friction
	qDebug() << "centripAcc=" << centripetalAccel;
	qDebug() << "sumAcc=" << currentNetAccel();
	qDebug() << "speed=" << currentSpeed*3.6;

	// calc speed for next bend, decide if we should start decelerating
	const RoadSegment *nextBend = mRoadGen->nextBend( mCar->odometer() );
	static RoadSegment *lastCalculatedNextBend;
	if( nextBend /*&& nextBend != lastCalculatedNextBend*/ )
	{
		qDebug() << "Calc speed for next bend (@" << nextBend->odoStartLoc() << ")";
		lastCalculatedNextBend = (RoadSegment*)nextBend;
		double maxSpeedForNextBend = sqrt( maxAccelAllowedByFriction * nextBend->radiusAbs() );
		double maxSafeSpeedForNextBend = maxSpeedForNextBend * bendSpeedHeadroomFactor;
		if( maxSafeSpeedForNextBend < currentSpeed )
		{
			double decelTimeForNextBend = (currentSpeed-maxSafeSpeedForNextBend) / maxSafeTangentialAccel;
			double decelDistanceForNextBend = currentSpeed*decelTimeForNextBend - maxSafeTangentialAccel * pow(decelTimeForNextBend,2) / 2; // minus, because deceleration
			qDebug() << "\tdist2Next: " << nextBend->odoStartLoc()-odometer << ", maxSafeSpeed: " << maxSafeSpeedForNextBend*3.6 << ", decelDistance: " << decelDistanceForNextBend;

			/// TODO: check if decelDistance is smaller than the available road ahead
			/// TODO: set accelVal here?

			if( decelDistanceForNextBend >= (nextBend->odoStartLoc()-odometer) )
			{
				mTargetSpeed = maxSafeSpeedForNextBend;
				qDebug() << "\tlimit speed NOW for next bend kmh: " << mTargetSpeed*3.6;
			}
			else
			{
				qDebug() << "\tbend is still far ahead...";
			}
		}
		else
		{
			mTargetSpeed = mCruiseSpeed;
			qDebug() << "\tcurrent speed is OK for next bend, set cruise";
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
		qDebug() << "Limit target speed due to current bend to " << mTargetSpeed*3.6 << " km/h";
	}

	// P control won't do -> next bend algorithm calculates with max tangential accel, so use that -> but now there is always acceleration....
//	double speedControlP = 0.4;
//	double speedError = mTargetSpeed-currentSpeed;
//	double accelerateVal = speedError*speedControlP;
	double accelerateVal = maxSafeTangentialAccel;
	if( mTargetSpeed < currentSpeed )
		{ accelerateVal *= -1; }

	if( accelerateVal > maxSafeTangentialAccel )
		{ accelerateVal = maxSafeTangentialAccel;	}
	if( accelerateVal < -maxSafeTangentialAccel )
		{ accelerateVal = -maxSafeTangentialAccel;	}

	// accelerate / decelerate
	qDebug() << "Set acceleration: " << accelerateVal;
	mCar->accelerate( accelerateVal );

	qDebug() << "------------------------------------------------------";
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
