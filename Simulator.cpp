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
}

double Simulator::currentMaxAccelerationRatio() const
{
	return mDriver.currentNetAccel() / (mCar.frictionCoeffStatic() * 9.81);
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
