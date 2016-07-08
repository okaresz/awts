#include "Simulator.h"
#include <QDateTime>
#include <QTransform>
#include "SettingsManager.h"

Simulator::Simulator(QObject *parent) : QObject(parent),
	mSimTime(0), mSimIntervalMs(50), mRoadVisibility(50.0), mRoadGen(mRoadVisibility), mDriver(&mCar,&mRoadGen), mCarPosOnRoad({0.0,0.0,0.0})
{
	if( !SettingsManager::instance()->contains("simulation/updateIntervalMs") )
		{ SettingsManager::instance()->setValue("simulation/updateIntervalMs", 50); }
	else
		{ mSimIntervalMs = SettingsManager::instance()->value("simulation/updateIntervalMs").toInt(); }

	if( !SettingsManager::instance()->contains("simulation/visibleRoadAheadM") )
		{ SettingsManager::instance()->setValue("simulation/visibleRoadAheadM", 50.0); }
	else
		{ mRoadVisibility = SettingsManager::instance()->value("simulation/visibleRoadAheadM").toDouble(); }
	mRoadGen.setRoadGenerationHorizon(mRoadVisibility);

	connect( &mSimTimer, SIGNAL(timeout()), this, SLOT(simUpdate()) );

	connect( &mDriver, SIGNAL(tractionLost(double)), this, SLOT(onCarTractionLost(double)) );
	connect( &mDriver, SIGNAL(crashed(double)), this, SLOT(onCarCrashed(double)) );
}

double Simulator::currentMaxAccelerationRatio() const
{
	double scalarRatio = mDriver.currentNetAccel().length() / (mCar.frictionCoeffStatic() * 9.81);
	if( mDriver.currentNetAccel().x() < 0 )
		{ scalarRatio *= -1; }
	return scalarRatio;
}

void Simulator::start()
{
	mSimTimer.start( mSimIntervalMs );
	emit simRunStateChanged(true);
}

void Simulator::stop()
{
	mSimTimer.stop();
	emit simRunStateChanged(false);
}

double Simulator::findTravel(double fromTravel, double &carCrossPos, double &carHeading)
{
	const static double yPosEpsilon = 0.001; // meters!
	double travel = fromTravel;
	Car::carLocation_t carLoc = mCar.location();
	RoadSegment::roadLocation_t roadLoc;
	QPointF carPositionInRoadSys;

	do
	{
		roadLoc = mRoadGen.location(travel);

		QTransform roadCoordSys;
		roadCoordSys.rotateRadians(roadLoc.heading);
		roadCoordSys.translate(roadLoc.x,-roadLoc.y);

		carPositionInRoadSys = roadCoordSys.map(QPointF(carLoc.x,carLoc.y));
		travel += carPositionInRoadSys.y()/2;
	} while( qAbs(carPositionInRoadSys.y()) > yPosEpsilon );

	carCrossPos = carPositionInRoadSys.x();
	carHeading = carLoc.heading - roadLoc.heading;
	return travel;
}

void Simulator::simUpdate()
{
	// calculate car position on road
	double carHeadingOnRoad;
	double carCrossPosOnRoad;
	mCarPosOnRoad.travel = findTravel(mCarPosOnRoad.travel,carCrossPosOnRoad,carHeadingOnRoad);
	mCarPosOnRoad.cross = carCrossPosOnRoad;
	mCarPosOnRoad.heading = carHeadingOnRoad;

	mRoadGen.simUpdate(mSimTime,mCarPosOnRoad.travel);
	mDriver.simUpdate(mSimTime,mCarPosOnRoad.travel,carCrossPosOnRoad,carHeadingOnRoad);
	mCar.simUpdate(mSimTime);

	emit simUpdated();

	mSimTime += mSimIntervalMs;
}

void Simulator::onCarTractionLost(double atTravel)
{
	Q_UNUSED(atTravel);
	stop();
}

void Simulator::onCarCrashed(double atTravel)
{
	Q_UNUSED(atTravel);
	stop();
}
