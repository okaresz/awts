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

	double maxAccelerationMpss() const
		{ return mMaxAccelMpss; }

	double acceleration() const
		{ return mAcceleration; }

signals:

public slots:
	void simUpdate(const quint64 simTime);

private:
	int mMassKg;
	QSizeF mSizeM;
	double mOdometer;
	double mMaxAccelMpss;
	double mAcceleration;	///< can be negative for deceleration
	double mSpeed;
	quint64 mLastSimUpdateTime;
	double mFrictionCoeffStatic;
};

#endif // CAR_H
