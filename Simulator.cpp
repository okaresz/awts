#include "Simulator.h"
#include <QDateTime>
#include "SettingsManager.h"

Simulator::Simulator(QObject *parent) : QObject(parent),
	mStartTimeStamp(0), mRoadVisibility(50.0)
{
	if( !SettingsManager::instance()->contains("simulation/updateIntervalMs") )
		{ SettingsManager::instance()->setValue("simulation/updateIntervalMs", 50); }

	mRoadVisibility = SettingsManager::instance()->value("simulation/visibleRoadAheadM").toDouble();

	connect( &mSimTimer, SIGNAL(timeout()), this, SLOT(simUpdate()) );
}

void Simulator::start()
{
	mStartTimeStamp = QDateTime::currentMSecsSinceEpoch();
	mSimTimer.start( SettingsManager::instance()->value("simulation/updateIntervalMs").toInt() );
}

void Simulator::simUpdate()
{
	mRoadGen.simUpdate(0);
}
