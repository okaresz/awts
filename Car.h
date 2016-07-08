#ifndef CAR_H
#define CAR_H

#include <QObject>
#include <QSizeF>

class Car : public QObject
{
	Q_OBJECT
public:
	explicit Car(QObject *parent = 0);

	struct carLocation_t
	{
		double odometer;	///< Car odometer, the distance the car has travelled so far in 2D space, from the beginning of the simulation.
		double x,y;			///< The absolute location of the car in 2D space, calculated from the beginning of the simulation.
		double heading;		///< Heading angle, 0 being car is pointing "upwards".
		carLocation_t()
			{ odometer = x = y = heading = 0.0; }
		carLocation_t(const double odometer,const double x,const double y,const double heading)
		{
			this->odometer = odometer;
			this->x = x;
			this->y = y;
			this->heading = heading;
		}
	};

	struct accelDecelPair_t
	{
		double acceleration;
		double deceleration;
		accelDecelPair_t() {
			acceleration = 0.0;
			deceleration = 0.0; }
		accelDecelPair_t(const double accel, const double decel) {
			acceleration = accel;
			deceleration = decel; }
	};

	/// get car odometer in meters.
	double odometer() const
		{ return mLocation.odometer; }
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

	double accelerationLimitMpss() const
		{ return mAccelLimitMpss; }

	double weightRatioOnDrivenWheels() const
		{ return mWeightRatioOnDrivenWheels; }

	double acceleration() const
		{ return mAcceleration; }

	/** Return maximum possible tangential acceleration, considering currentCentripetalAcc as well.
	 *  The algorithm also takes into account the reduced possible tangential acceleration
	 *  due to the weight distribution on the driven wheels, which plays a role only when accelerating. Braking uses all 4 wheels.
	 * @return Max possible tangential acceleration: first value is the acceleration, second is the deceleration/braking.*/
	accelDecelPair_t maxTangentialAcceleration(const double currentCentripetalAcc) const;

	/** Get minimum turning radius.
	 *	Calculated using Ackermann steering geometry, for the inner wheels, avg of front and back wheel turn radius + carWidth / 2.*/
	double minTurnRadius() const;

	/** Get maximum turning curvature.
	 *	Calculated using Ackermann steering geometry, for the inner wheels, avg of front and back wheel turn radius + carWidth / 2.*/
	double maxTurnCurvature() const;

	/** Get turning curvature (signed) at given wheel angle (signed, negative means left turn).
	 *	Calculated using Ackermann steering geometry, for the inner wheels, avg of front and back wheel turn radius + carWidth / 2.
	 *  @return The turn curvature. If given angle was < 0, this will be negative as well.*/
	double turnCurvatureAtWheelAngle( double angleRad ) const;

	/// Set wheelAngle
	void steer(double wheelAngleRad);

	/// Get current turn curvature (left turns are negative!).
	double turnCurvature() const;

	/// Get wheel angle according to the given turnCurvature (signed, negative means left turn).
	double wheelAngleAtTurnCurvature(double turnCurvature, bool boundByCarMaxWheelAngle = true);
	/// Set wheel angle according to the given turnCurvature (signed, negative means left turn).
	void steerForTurnCurvature(double turnCurvature);

	double wheelAngle() const
		{ return mWheelAngle; }

	carLocation_t location() const
		{ return mLocation; }

signals:

public slots:
	void simUpdate(const quint64 simTime);

private:
	// properties
	int mMassKg;
	QSizeF mSizeM;
	double mAxisDistance;
	double mAccelLimitMpss;
	double mFrictionCoeffStatic;
	double mMaxWheelAngle;
	double mWeightRatioOnDrivenWheels;

	// state variables
	double mAcceleration;	///< can be negative for deceleration
	double mSpeed;
	double mWheelAngle;	///< negative is left turn
	carLocation_t mLocation;	///< Absolute location of the car in 2D space.

	// other
	quint64 mLastSimUpdateTime;
};

#endif // CAR_H
