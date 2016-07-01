#include "RoadGenerator.h"
#include "SettingsManager.h"

RoadGenerator::RoadGenerator(QObject *parent) : QObject(parent)
{
	if( !SettingsManager::instance()->contains("simulation/visibleRoadAheadM") )
		{ SettingsManager::instance()->setValue("simulation/visibleRoadAheadM", 50.0); }
}

void RoadGenerator::simUpdate(double simOdometer)
{
	// if necessary, add new segment
	while( lengthAhead(simOdometer) < SettingsManager::instance()->value( "simulation/visibleRoadAheadM").toDouble() )
	{
		mSegmentQueue.enqueue( new RoadSegment(endOfRoad()) );
	}

	// check if we can delete an already left-behind segment
	if( !mSegmentQueue.isEmpty() && mSegmentQueue.first()->odoEndLoc() < simOdometer )
	{
		mSegmentQueue.dequeue()->deleteLater();
	}
}

double RoadGenerator::lengthAhead(double simOdometer) const
{
	if( mSegmentQueue.isEmpty() )
		{ return 0.0; }

	return mSegmentQueue.last()->odoStartLoc() + mSegmentQueue.last()->length() - simOdometer;
}

double RoadGenerator::endOfRoad() const
{
	if( mSegmentQueue.isEmpty() )
		{ return 0.0; }

	return mSegmentQueue.last()->odoStartLoc() + mSegmentQueue.last()->length();
}
