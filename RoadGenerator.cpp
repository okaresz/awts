#include "RoadGenerator.h"
#include "SettingsManager.h"
#include "RandGen.h"
#include <QDebug>

RoadGenerator::RoadGenerator(double roadVisibility, QObject *parent) : QObject(parent),
	mRoadGenerationHorizon(roadVisibility), mRoadVisibility(roadVisibility)
{
	if( !SettingsManager::instance()->contains("road/obstacleProbabilityPerMeter") )
		{ SettingsManager::instance()->setValue("road/obstacleProbabilityPerMeter", 0.008); }
}

void RoadGenerator::simUpdate(const quint64 simTime, const double travel)
{
	Q_UNUSED(simTime);

	//DEBUG
/*	if( simTime == 0 )
	{
		mSegmentQueue.enqueue( new RoadSegment(1,{0.0,0.0,0.0,0.0}, 0.0, 200.0, 10.0, 10.0) );
		mSegmentQueue.enqueue( new RoadSegment(2,mSegmentQueue.last()->endLocation(), 0.015, 100.0, 10.0, 10.0) );
		mSegmentQueue.enqueue( new RoadSegment(3,mSegmentQueue.last()->endLocation(), 0.0, 120.0, 10.0, 10.0) );
		//mObstacleQueue.enqueue( new RoadObstacle( 7, 0.1, 0.6 ) );
		//mObstacleQueue.enqueue( new RoadObstacle( 28, 0.5, 1 ) );
		//mObstacleQueue.enqueue( new RoadObstacle( 45, -0.8, 1.6 ) );
		return;
	}*/
	//DEBUG END

	// if necessary, add new segment
	while( lengthAhead(travel) < SettingsManager::instance()->value( "simulation/visibleRoadAheadM").toDouble() )
	{
		RoadSegment::roadLocation_t lastSegmentEndLocation = {0.0,0.0,0.0,0.0};
		double lastSegmentEndWidth = 0.0;	// segment object will default to value in Settings
		if( !mSegmentQueue.isEmpty() )
		{
			lastSegmentEndLocation = mSegmentQueue.last()->endLocation();
			lastSegmentEndWidth = mSegmentQueue.last()->endWidth();
		}
		mSegmentQueue.enqueue( new RoadSegment(lastSegmentEndLocation,lastSegmentEndWidth) );

		// generate obstacles for the new segment
		double obstacleprobabilityOnSegment = SettingsManager::instance()->value("road/obstacleProbabilityPerMeter").toDouble()*mSegmentQueue.last()->length();
		int obstacleCount = (int)obstacleprobabilityOnSegment;
		double remainderProbability = obstacleprobabilityOnSegment - (double)obstacleCount;
		if( RandGen::instance()->generateF() <= remainderProbability )
			{ ++obstacleCount; }
		for( int i=0; i<obstacleCount; i++ )
		{
			double normalPosAlongSegment = RandGen::instance()->generateF();
			mObstacleQueue.enqueue( new RoadObstacle(mSegmentQueue.last()->startLocation().parameter + mSegmentQueue.last()->length()*normalPosAlongSegment) );
		}
	}

	// delete already left-behind segments
	if( !mSegmentQueue.isEmpty() && mSegmentQueue.first()->endRoadParam() < travel )
	{
		delete mSegmentQueue.dequeue();
	}

	// delete already left-behind obstacles
	deleteObstaclesBefore(travel);

	qInfo() << "#Road obstacle count = " << mObstacleQueue.size();
	qInfo() << "#Road segment count = " << mSegmentQueue.size();
}

double RoadGenerator::lengthAhead(double travel) const
{
	if( mSegmentQueue.isEmpty() )
		{ return 0.0; }

	return mSegmentQueue.last()->endRoadParam() - travel;
}

QQueue<RoadSegment> RoadGenerator::visibleRoad(const double travel) const
{
	double horizon = travel + mRoadVisibility;
	QQueue<RoadSegment> visibleRoad;
	for( int i=0; i<mSegmentQueue.size(); i++ )
	{
		RoadSegment *seg = mSegmentQueue.at(i);
		if( seg->startLocation().parameter < horizon )
		{
			if( seg->endRoadParam() > travel )
			{
				double visibleLength = seg->length();
				if( seg->endRoadParam() > horizon )
					{ visibleLength = horizon - seg->startLocation().parameter; }
				visibleRoad.enqueue( RoadSegment( seg->segmentId(), seg->startLocation(), seg->curvature(), visibleLength, seg->startWidth(), seg->widthAt(visibleLength) ) );
			}
		}
		else break;
	}
	return visibleRoad;	// let the compiler do the return value optimization
}

QQueue<RoadObstacle> RoadGenerator::visibleObstacles(const double travel) const
{
	double horizon = travel + mRoadVisibility;
	QQueue<RoadObstacle> visibleObstacles;
	for( int i=0; i<mObstacleQueue.size(); i++ )
	{
		if( mObstacleQueue.at(i)->roadParam() < horizon )
			{ visibleObstacles.enqueue( *mObstacleQueue.at(i) ); }
	}
	return visibleObstacles;	// let the compiler do the return value optimization
}

const RoadSegment *RoadGenerator::segmentAt(double roadParam) const
{
	for( int i=0; i<mSegmentQueue.size(); i++ )
	{
		RoadSegment *seg = mSegmentQueue.at(i);
		if( seg->startLocation().parameter <= roadParam && seg->endRoadParam() >= roadParam )
		{
			return seg;
		}
	}
	return nullptr;
}

RoadSegment::roadLocation_t RoadGenerator::location(double roadParam) const
{
	const RoadSegment *seg = segmentAt( roadParam );
	if( seg )
	{
		double lengthOnSegment = roadParam - seg->startLocation().parameter;
		return seg->endLocation( lengthOnSegment );
	}
	else
	{
		qWarning("No segment at given roadParam!");
		RoadSegment::roadLocation_t nullLoc({0.0,0.0,0.0,0.0});
		return nullLoc;
	}
}

const RoadSegment *RoadGenerator::nextBend(double travel)
{
	for( int i=0; i<mSegmentQueue.size(); i++ )
	{
		RoadSegment *seg = mSegmentQueue.at(i);
		if( seg->startLocation().parameter > travel && seg->isBend() )
			{ return seg; }
	}
	return nullptr;
}

void RoadGenerator::deleteObstaclesBefore(double travel)
{
	while( !mObstacleQueue.isEmpty() && mObstacleQueue.first()->roadParam() < travel )
		{ delete mObstacleQueue.dequeue(); }
}
