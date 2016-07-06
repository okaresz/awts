#include "Simulator.h"
#include <QDateTime>
#include "SettingsManager.h"

Simulator::Simulator(QObject *parent) : QObject(parent),
	mSimTime(0), mSimIntervalMs(50), mRoadVisibility(50.0), mRoadGen(mRoadVisibility), mDriver(&mCar,&mRoadGen)
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

void Simulator::simUpdate()
{
	mRoadGen.simUpdate(mSimTime,mCar.odometer());
	mDriver.simUpdate(mSimTime);
	mCar.simUpdate(mSimTime);

	emit simUpdated();

	mSimTime += mSimIntervalMs;
}

void Simulator::onCarTractionLost(double atOdometer)
{
	stop();
}

void Simulator::onCarCrashed(double atOdometer)
{
	stop();
}
