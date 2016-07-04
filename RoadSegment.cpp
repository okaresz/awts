#include "RoadSegment.h"
#include "SettingsManager.h"
#include "RandGen.h"
#include <QDebug>

RoadSegment::RoadSegment(double odoStartLoc, QObject *parent) : QObject(parent),
	mOdoStartLoc(odoStartLoc)
{
	initSettings();

	mLength = RandGen::instance()->generateF( SettingsManager::instance()->value("road/segmentMinLengthM").toDouble(), SettingsManager::instance()->value("road/segmentMaxLengthM").toDouble() );
	mStartWidth = mEndWidth = SettingsManager::instance()->value("road/defaultWidthM").toDouble();

	// decide if bend or straight
	double straightProb = RandGen::instance()->generateF();
	if( straightProb < SettingsManager::instance()->value("road/straightVsBendSegmentProbability").toDouble() )
	{
		mRadius = 0;
	}
	else // Generate random radius for bend
	{
		mRadius = RandGen::instance()->generateF( SettingsManager::instance()->value("road/bendMinRadiusM").toDouble(), SettingsManager::instance()->value("road/bendMaxRadiusM").toDouble() );
		// check for max turn and adjust length if necessary
		double maxTurnRad = SettingsManager::instance()->value("road/bendMaxTurnDegree").toDouble() / 180.0 * M_PI;
		if( maxTurnRad * mRadius < mLength )
			{ mLength = maxTurnRad * mRadius; }
		if( RandGen::instance()->generateF() > 0.5 ) { mRadius *= -1; }
	}

	qDebug() << QString("Road segment created ( @%1, L=%2, R=%3 )").arg(mOdoStartLoc).arg( mLength ).arg(mRadius);
}

RoadSegment::RoadSegment(double odoStartLoc, double radius, double length, double startWidth, double endWidth, QObject *parent) : QObject(parent),
	mOdoStartLoc(odoStartLoc), mRadius(radius), mLength(length), mStartWidth(startWidth), mEndWidth(endWidth)
{
	initSettings();
	if( mRadius != 0 )
	{
		double maxTurnRad = SettingsManager::instance()->value("road/bendMaxTurnDegree").toDouble() / 180.0 * M_PI;
		if( maxTurnRad * fabs(mRadius) < mLength )
			{ mLength = maxTurnRad * mRadius; }
	}
	qDebug() << QString("Road segment created ( @%1, L=%2, R=%3 )").arg(mOdoStartLoc).arg( mLength ).arg(mRadius);
}

double RoadSegment::length() const
{
	return mLength;
}

double RoadSegment::widthAt(const double metersFromSegmentStart) const
{
	return mStartWidth + (mEndWidth-mStartWidth) * (metersFromSegmentStart/mLength);
}

void RoadSegment::initSettings()
{
	// init default settings
	if( !SettingsManager::instance()->contains( "road/straightVsBendSegmentProbability") )
		{ SettingsManager::instance()->setValue("road/straightVsBendSegmentProbability", 0.65); }

	if( !SettingsManager::instance()->contains( "road/bendMinRadiusM") )
		{ SettingsManager::instance()->setValue("road/bendMinRadiusM", 30.0); }
	if( !SettingsManager::instance()->contains( "road/bendMaxRadiusM") )
		{ SettingsManager::instance()->setValue("road/bendMaxRadiusM", 400.0); }
	if( !SettingsManager::instance()->contains("road/bendMaxTurnDegree") )
		{ SettingsManager::instance()->setValue("road/bendMaxTurnDegree", 160.0); }

	if( !SettingsManager::instance()->contains( "road/segmentMinLengthM") )
		{ SettingsManager::instance()->setValue("road/segmentMinLengthM", 30.0); }
	if( !SettingsManager::instance()->contains( "road/segmentMaxLengthM") )
		{ SettingsManager::instance()->setValue("road/segmentMaxLengthM", 400.0); }

	if( !SettingsManager::instance()->contains("road/defaultWidthM") )
		{ SettingsManager::instance()->setValue("road/defaultWidthM", 10.0); }
}
