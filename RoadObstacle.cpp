#include "RoadObstacle.h"
#include "RandGen.h"
#include "SettingsManager.h"
#include <QDebug>

RoadObstacle::RoadObstacle(const double roadparam, const double normalPosition, const double size) :
	mRoadParam(roadparam), mNormalPos(normalPosition), mSize(size)
{
	if( !SettingsManager::instance()->contains( "road/obstacleSizeMinM") )
		{ SettingsManager::instance()->setValue("road/obstacleSizeMinM", 0.8); }
	if( !SettingsManager::instance()->contains( "road/obstacleSizeMaxM") )
		{ SettingsManager::instance()->setValue("road/obstacleSizeMaxM", 2.0); }

	if( mNormalPos == 0.0 )
	{
		mNormalPos = RandGen::instance()->generateF( -1.0, 1.0 );
	}

	if( mSize == 0.0 )
	{
		mSize = RandGen::instance()->generateF(
					SettingsManager::instance()->value( "road/obstacleSizeMinM").toDouble(),
					SettingsManager::instance()->value( "road/obstacleSizeMaxM").toDouble() );
	}

	qDebug() << QString("Road obstacle created ( @%1, normP=%2, size=%3 )").arg(mRoadParam).arg( mNormalPos ).arg(mSize);
}
