#include "Car.h"
#include "SettingsManager.h"
#include <QSizeF>
#include <QDateTime>

Car::Car(QObject *parent) : QObject(parent),
	mMassKg(1000), mSizeM(1.6,4.0), mMaxAccelMpss(5.0), mFrictionCoeffStatic(0.2), mMaxWheelAngle(50.0/180.0*M_PI), mOdometer(0.0), mAcceleration(0.0), mSpeed(0.0), mLastSimUpdateTime(0)
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
		{ mMaxAccelMpss = SettingsManager::instance()->value("car/maxAccelerationMpss").toDouble(); }

	if( !SettingsManager::instance()->contains("car/frictionCoeff") )
		{ SettingsManager::instance()->setValue("car/frictionCoeff", 0.2); }
	else
		{ mFrictionCoeffStatic = SettingsManager::instance()->value("car/frictionCoeff").toDouble(); }

	if( !SettingsManager::instance()->contains("car/maxWheelAngleDeg") )
		{ SettingsManager::instance()->setValue("car/maxWheelAngleDeg", 50.0); }
	else
		{ mMaxWheelAngle = SettingsManager::instance()->value("car/maxWheelAngleDeg").toDouble()/180.0*M_PI; }

	mAxisDistance = mSizeM.height()*0.8;
}

double Car::odometer() const
{
	return mOdometer;
}

void Car::accelerate(float accelMpss)
{
	if( -mMaxAccelMpss < accelMpss && accelMpss < mMaxAccelMpss )
		{ mAcceleration = accelMpss; }
}

void Car::decelerate(float decelerateMpss)
{
	if( 0 < decelerateMpss && decelerateMpss < mMaxAccelMpss )
	{ mAcceleration = -decelerateMpss; }
}

double Car::minTurnRadius() const
{
	return turnRadiusAtWheelAngle(mMaxWheelAngle);
}

double Car::turnRadiusAtWheelAngle(double angleRad) const
{
	// average: (front + back) / 2 + w/2
	angleRad = qBound(0.0, qAbs(angleRad), mMaxWheelAngle);
	return (mAxisDistance/sin(angleRad) + mAxisDistance/tan(angleRad)) / 2 + mSizeM.width()/2;
}

void Car::simUpdate(const quint64 simTime)
{
	quint64 dTms = simTime - mLastSimUpdateTime;
	double dT = dTms/1000.0;

	// update speed
	mSpeed += dT * mAcceleration;
	if( mSpeed < 0 )
		{ mSpeed = 0; }

	// update odometer
	mOdometer += mSpeed * dT;

	mLastSimUpdateTime = simTime;
}
