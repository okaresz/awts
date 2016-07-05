#ifndef CAR_H
#define CAR_H

#include <QObject>
#include <QSizeF>

class Car : public QObject
{
	Q_OBJECT
public:
	explicit Car(QObject *parent = 0);

	/// get car odometer in meters.
	double odometer() const;
	/// Accelerate. Can be negative for deceleration.
	void accelerate( float accelMpss );
	void decelerate( float decelerateMpss );

	double speed() const
		{ return mSpeed; }
	double speedKmh() const
		{ return mSpeed*3.6; }

	double frictionCoeffStatic() const
		{ return mFrictionCoeffStatic; }

	double massKg() const
		{ return mMassKg; }

	QSizeF size() const
		{ return mSizeM; }

	double maxAccelerationMpss() const
		{ return mMaxAccelMpss; }

	double acceleration() const
		{ return mAcceleration; }

	/** Get minimum turning radius.
	 *	Calculated using Ackermann steering geometry, for the inner wheels, avg of front and back wheel turn radius + carWidth / 2.*/
	double minTurnRadius() const;

	/** Get turning radius (unsigned) at given wheel angle.
	 *	Calculated using Ackermann steering geometry, for the inner wheels, avg of front and back wheel turn radius + carWidth / 2.*/
	double turnRadiusAtWheelAngle( double angleRad ) const;

signals:

public slots:
	void simUpdate(const quint64 simTime);

private:
	// properties
	int mMassKg;
	QSizeF mSizeM;
	double mAxisDistance;
	double mMaxAccelMpss;
	double mFrictionCoeffStatic;
	double mMaxWheelAngle;

	// state variables
	double mOdometer;
	double mAcceleration;	///< can be negative for deceleration
	double mSpeed;

	// other
	quint64 mLastSimUpdateTime;
};

#endif // CAR_H
