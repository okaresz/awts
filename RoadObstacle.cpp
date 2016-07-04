#include "RoadObstacle.h"
#include "RandGen.h"
#include "SettingsManager.h"
#include <QDebug>

RoadObstacle::RoadObstacle(const double odoPosition, const double crossPosition, const double size, QObject *parent) : QObject(parent),
	mOdoPos(odoPosition), mCrossPos(crossPosition), mSize(size)
{
	if( !SettingsManager::instance()->contains( "road/obstacleSizeMinM") )
		{ SettingsManager::instance()->setValue("road/obstacleSizeMinM", 0.8); }
	if( !SettingsManager::instance()->contains( "road/obstacleSizeMaxM") )
		{ SettingsManager::instance()->setValue("road/obstacleSizeMaxM", 2.0); }

	if( mCrossPos == 0.0 )
	{
		mCrossPos = RandGen::instance()->generateF( -1.0, 1.0 );
	}

	if( mSize == 0.0 )
	{
		mSize = RandGen::instance()->generateF(
					SettingsManager::instance()->value( "road/obstacleSizeMinM").toDouble(),
					SettingsManager::instance()->value( "road/obstacleSizeMaxM").toDouble() );
	}

	qDebug() << QString("Road obstacle created ( @%1, crossP=%2, size=%3 )").arg(mOdoPos).arg( mCrossPos ).arg(mSize);
}
