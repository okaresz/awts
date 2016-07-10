#include "Car.h"
#include "SettingsManager.h"
#include <QSizeF>
#include <QDateTime>
#include <QVector2D>
#include <QDebug>

Car::Car(QObject *parent) : QObject(parent),
	mMassKg(1000), mSizeM(1.6,4.0), mAccelLimitMpss(5.0), mFrictionCoeffStatic(0.2), mMaxWheelAngle(50.0/180.0*M_PI), mWeightRatioOnDrivenWheels(0.55),
	mAcceleration(0.0), mSpeed(0.0), mWheelAngle(0.0), mLocation({0.0,0.0,0.0,0.0}), mLastSimUpdateTime(0)
{
	if( !SettingsManager::instance()->contains("car/massKg") )
		{ SettingsManager::instance()->setValue("car/massKg", 1000); }
	else
		{ mMassKg = SettingsManager::instance()->value("car/massKg").toInt(); }
	if( !SettingsManager::instance()->contains("car/sizeM") )
		{ SettingsManager::instance()->setValue("car/sizeM", QSizeF(1.6,4.0)); }
	else
		{ mSizeM = SettingsManager::instance()->value("car/sizeM").toSizeF(); }

	if( !SettingsManager::instance()->contains("car/maxAccelerationMpss") )
		{ SettingsManager::instance()->setValue("car/maxAccelerationMpss", 5.0); }
	else
		{ mAccelLimitMpss = SettingsManager::instance()->value("car/maxAccelerationMpss").toDouble(); }

	if( !SettingsManager::instance()->contains("car/frictionCoeff") )
		{ SettingsManager::instance()->setValue("car/frictionCoeff", 0.2); }
	else
		{ mFrictionCoeffStatic = SettingsManager::instance()->value("car/frictionCoeff").toDouble(); }

	if( !SettingsManager::instance()->contains("car/maxWheelAngleDeg") )
		{ SettingsManager::instance()->setValue("car/maxWheelAngleDeg", 50.0); }
	else
		{ mMaxWheelAngle = SettingsManager::instance()->value("car/maxWheelAngleDeg").toDouble()/180.0*M_PI; }

	if( !SettingsManager::instance()->contains("car/weightRatioOnDrivenWheels") )
		{ SettingsManager::instance()->setValue("car/weightRatioOnDrivenWheels", 0.55); }
	else
		{ mWeightRatioOnDrivenWheels = SettingsManager::instance()->value("car/weightRatioOnDrivenWheels").toDouble(); }

	mAxisDistance = mSizeM.height()*0.8;
}

void Car::accelerate(float accelMpss)
{
	mAcceleration = qBound(-mAccelLimitMpss,(double)accelMpss,mAccelLimitMpss);
}

void Car::decelerate(float decelerateMpss)
{
	mAcceleration = -qBound(0.0,(double)qAbs(decelerateMpss),mAccelLimitMpss);
}

Car::accelDecelPair_t Car::maxTangentialAcceleration(const double currentCentripetalAcc) const
{
	double maxAccelAllowedByFriction = mFrictionCoeffStatic * 9.81;
	double maxTangentialAccel = 0;
	if( pow(maxAccelAllowedByFriction,2) > pow(currentCentripetalAcc,2) )
	{
		maxTangentialAccel = sqrt( pow(maxAccelAllowedByFriction,2) - pow(currentCentripetalAcc,2) );
	}
	double maxPossibleTangentialAccel = mFrictionCoeffStatic * 9.81 * mWeightRatioOnDrivenWheels;
	accelDecelPair_t accDec;
	accDec.acceleration = qMin(maxTangentialAccel,maxPossibleTangentialAccel);
	accDec.deceleration = maxTangentialAccel; // braking is not affected by weight distribution (in this model..)
	return accDec;
}

double Car::minTurnRadius() const
{
	double tc = turnCurvatureAtWheelAngle(mMaxWheelAngle);
	if( tc == 0.0 )
		{ return 10000000000; } // not gonna happen at any reasonable mMaxWheelAngle...
	else
		{ return 1/tc; }
}

double Car::maxTurnCurvature() const
{
	return turnCurvatureAtWheelAngle(mMaxWheelAngle);
}

double Car::turnCurvatureAtWheelAngle(double angleRad) const
{
	if( qAbs(angleRad) < 0.00000001 )
		{ return 0.0; }

	short int sign = 1; if( angleRad < 0 ) { sign *= -1; }
	// average: (front + back) / 2 + w/2
	angleRad = qBound(0.0, qAbs(angleRad), mMaxWheelAngle);
	return sign*( 2 / (mAxisDistance/sin(angleRad) + mAxisDistance/tan(angleRad) + mSizeM.width()) );
}

void Car::steer(double wheelAngleRad)
{
	if( wheelAngleRad > mMaxWheelAngle )
		{ wheelAngleRad = mMaxWheelAngle; }
	else if( wheelAngleRad < -mMaxWheelAngle )
		{ wheelAngleRad = -mMaxWheelAngle; }
	mWheelAngle = wheelAngleRad;
}

double Car::turnCurvature() const
{
	return turnCurvatureAtWheelAngle(mWheelAngle);
}

double Car::wheelAngleAtTurnCurvature(double turnCurvature, bool boundByCarMaxWheelAngle)
{
	if( turnCurvature == 0.0 )
		{ return 0.0; }

	short int sign = 1; if( turnCurvature < 0 ) { sign *= -1; }
	turnCurvature = qAbs(turnCurvature);
	if( boundByCarMaxWheelAngle )
	{
		double maxTurnC = maxTurnCurvature();
		if( turnCurvature > maxTurnC )
			{ turnCurvature = maxTurnC; }
	}
	// this is the inverse function of the one in turnCurvatureAtWheelAngle() for positive angles
	return sign*( 2*(M_PI_2-atan( (2/turnCurvature-mSizeM.width())/mAxisDistance )) ); // pi/2-arctan(x) = arccot(x)
}

void Car::steerForTurnCurvature(double turnCurvature)
{
	// turnCurvature < 0 -> left turn
	steer( wheelAngleAtTurnCurvature(turnCurvature,true) ); // pi/2-arctan(x) = arccot(x)
}

void Car::simUpdate(const quint64 simTime)
{
	quint64 dTms = simTime - mLastSimUpdateTime;
	double dT = dTms/1000.0;

	// update speed -------------------------------
	mSpeed += dT * mAcceleration;
	if( mSpeed < 0 )
		{ mSpeed = 0; }

	double dOdo = mSpeed * dT;

	// update car location -------------------------------
	double turnCurv = turnCurvature();
	double alpha = dOdo*turnCurv;
	QVector2D deltaPos;
	if( turnCurv == 0.0 )
	{
		deltaPos.setX( 0.0 );
		deltaPos.setY(dOdo);
	}
	else
	{
		deltaPos.setX( (1-cos(alpha)) / turnCurv );
		deltaPos.setY( sin(alpha) / turnCurv );
	}
	qInfo() << "Car deltaPos: @d" << dOdo <<"; " << deltaPos << "turnCurvature: " << turnCurv;
	// rotate (coord. sys) with current heading (watch the sign! geometric  rotation "to the left" is positive!, but rotating the coord. sys left is like rotating the point right...)
	QVector2D deltaPosRot(deltaPos);
	if( mLocation.heading != 0.0 )
	{
		double rotAngle = mLocation.heading;
		deltaPosRot.setX( deltaPos.x()*cos(rotAngle) + deltaPos.y()*sin(rotAngle) );
		deltaPosRot.setY( -deltaPos.x()*sin(rotAngle) + deltaPos.y()*cos(rotAngle) );
	}
	qInfo() << "Car deltaPos(rot): " << deltaPosRot;
	mLocation.odometer += dOdo;
	mLocation.x += deltaPosRot.x();
	mLocation.y += deltaPosRot.y();
	mLocation.heading += alpha;

	qInfo() << "CarLoc: @" << mLocation.odometer << "|" << mLocation.x << "," << mLocation.y << "/" << mLocation.heading/M_PI*180.0;

	// ---------------------------------------------------

	mLastSimUpdateTime = simTime;
}
