#include "RoadGenerator.h"
#include "SettingsManager.h"
#include "RandGen.h"
#include <QDebug>

RoadGenerator::RoadGenerator(double roadVisibilityM, QObject *parent) : QObject(parent),
	mRoadVisibilityM(roadVisibilityM)
{
	if( !SettingsManager::instance()->contains("road/obstacleprobabilityPerMeter") )
		{ SettingsManager::instance()->setValue("road/obstacleprobabilityPerMeter", 0.005); }
}

void RoadGenerator::simUpdate(const quint64 simTime, const double simOdometer)
{
	Q_UNUSED(simTime);

	//DEBUG
//	mSegmentQueue.enqueue( new RoadSegment(endOfRoad(), 0.0, 10.0, 10.0, 10.0) );
//	mSegmentQueue.enqueue( new RoadSegment(endOfRoad(), 60.0, 30.0, 10.0, 10.0) );
//	mSegmentQueue.enqueue( new RoadSegment(endOfRoad(), 0.0, 10.0, 10.0, 10.0) );
//	mObstacleQueue.enqueue( new RoadObstacle( 7, 0.1, 0.6 ) );
//	mObstacleQueue.enqueue( new RoadObstacle( 28, 0.5, 1 ) );
//	mObstacleQueue.enqueue( new RoadObstacle( 45, -0.8, 1.6 ) );
//	return;
	//DEBUG END

	// if necessary, add new segment
	while( lengthAhead(simOdometer) < SettingsManager::instance()->value( "simulation/visibleRoadAheadM").toDouble() )
	{
		mSegmentQueue.enqueue( new RoadSegment(endOfRoad()) );

		// generate obstcles for the new segment
		double obstacleprobabilityOnSegment = SettingsManager::instance()->value("road/obstacleprobabilityPerMeter").toDouble()*mSegmentQueue.last()->length();
		int obstacleCount = (int)obstacleprobabilityOnSegment;
		double remainderProbability = obstacleprobabilityOnSegment - (double)obstacleCount;
		if( RandGen::instance()->generateF() <= remainderProbability )
			{ ++obstacleCount; }
		for( int i=0; i<obstacleCount; i++ )
		{
			double normalPosAlongSegment = RandGen::instance()->generateF();
			mObstacleQueue.enqueue( new RoadObstacle(mSegmentQueue.last()->odoStartLoc()+mSegmentQueue.last()->length()*normalPosAlongSegment) );
		}
	}

	// delete already left-behind segments
	if( !mSegmentQueue.isEmpty() && mSegmentQueue.first()->odoEndLoc() < simOdometer )
	{
		mSegmentQueue.dequeue()->deleteLater();
	}

	// delete already left-behind obstacles
	deleteObstaclesBefore(simOdometer);

	qDebug() << "#Road obstacle count = " << mObstacleQueue.size();
	qDebug() << "#Road segment count = " << mSegmentQueue.size();
}

double RoadGenerator::lengthAhead(double simOdometer) const
{
	if( mSegmentQueue.isEmpty() )
		{ return 0.0; }

	return mSegmentQueue.last()->odoEndLoc() - simOdometer;
}

double RoadGenerator::endOfRoad() const
{
	if( mSegmentQueue.isEmpty() )
		{ return 0.0; }

	return mSegmentQueue.last()->odoStartLoc() + mSegmentQueue.last()->length();
}

const RoadSegment *RoadGenerator::segmentAtOdo(double odometerVal)
{
	for( int i=0; i<mSegmentQueue.size(); i++ )
	{
		RoadSegment *seg = mSegmentQueue.at(i);
		if( seg->odoStartLoc() <= odometerVal && seg->odoEndLoc() >= odometerVal )
		{
			return seg;
		}
	}
	return nullptr;
}

const RoadSegment *RoadGenerator::nextBend(double odometerVal)
{
	for( int i=0; i<mSegmentQueue.size(); i++ )
	{
		RoadSegment *seg = mSegmentQueue.at(i);
		if( seg->odoStartLoc() > odometerVal && seg->isBend() )
			{ return seg; }
	}
	return nullptr;
}

void RoadGenerator::deleteObstaclesBefore(double odoMark)
{
	while( !mObstacleQueue.isEmpty() && mObstacleQueue.first()->odoPos() < odoMark )
		{ mObstacleQueue.dequeue()->deleteLater(); }
}
