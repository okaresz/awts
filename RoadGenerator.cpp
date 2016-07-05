#include "RoadGenerator.h"
#include "SettingsManager.h"
#include "RandGen.h"
#include <QDebug>

RoadGenerator::RoadGenerator(double roadVisibility, QObject *parent) : QObject(parent),
	mRoadGenerationHorizon(roadVisibility), mRoadVisibility(roadVisibility)
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
		delete mSegmentQueue.dequeue();
	}

	// delete already left-behind obstacles
	deleteObstaclesBefore(simOdometer);

	qInfo() << "#Road obstacle count = " << mObstacleQueue.size();
	qInfo() << "#Road segment count = " << mSegmentQueue.size();
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

QQueue<RoadSegment> RoadGenerator::visibleRoad(const double odometer) const
{
	double horizon = odometer + mRoadVisibility;
	QQueue<RoadSegment> visibleRoad;
	for( int i=0; i<mSegmentQueue.size(); i++ )
	{
		RoadSegment *seg = mSegmentQueue.at(i);
		if( seg->odoStartLoc() < horizon )
		{
			if( seg->odoEndLoc() > odometer )
			{
				double visibleLength = seg->length();
				if( seg->odoEndLoc() > horizon )
					{ visibleLength = horizon - seg->odoStartLoc(); }
				visibleRoad.enqueue( RoadSegment( seg->odoStartLoc(), seg->radius(), visibleLength, seg->startWidth(), seg->widthAt(visibleLength) ) );
			}
		}
		else break;
	}
	return visibleRoad;	// let the compiler do the return value optimization
}

const QQueue<RoadObstacle> RoadGenerator::visibleObstacles(const double odometer) const
{
	double horizon = odometer + mRoadVisibility;
	QQueue<RoadObstacle> visibleObstacles;
	for( int i=0; i<mObstacleQueue.size(); i++ )
	{
		if( mObstacleQueue.at(i)->odoPos() < horizon )
			{ visibleObstacles.enqueue( *mObstacleQueue.at(i) ); }
	}
	return visibleObstacles;	// let the compiler do the return value optimization
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
		{ delete mObstacleQueue.dequeue(); }
}
