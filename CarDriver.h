#ifndef CARDRIVER_H
#define CARDRIVER_H

#include <QObject>
#include "Car.h"
#include "RoadGenerator.h"

class CarDriver : public QObject
{
	Q_OBJECT
public:
	explicit CarDriver( Car *car, RoadGenerator *roadGen, QObject *parent = 0);

	double cruiseSpeedKmh() const;
	double cruiseSpeedMps() const;
	void setCruiseSpeedMps(double cruiseSpeedMps);
	void setCruiseSpeedKmh(double cruiseSpeedKmh);
	double targetSpeedKmh() const;
	double targetSpeedMps() const;

	double currentCentripetalAccel() const;
	double currentNetAccel() const;
	double currentMaxPossibleSpeedOnBend() const;

signals:

public slots:
	void simUpdate(const quint64 simTime);

private:
	Car *mCar;
	RoadGenerator *mRoadGen;
	double mCruiseSpeed, mTargetSpeed;
};

#endif // CARDRIVER_H
