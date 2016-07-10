#include "RoadSegment.h"
#include "SettingsManager.h"
#include "RandGen.h"
#include <QDebug>
#include <QVector2D>

long int RoadSegment::segmentCounter = 0;

RoadSegment::RoadSegment(roadLocation_t startLocation) :
	mSegmentId(++segmentCounter), mStartLoc(startLocation)
{
	initSettings();

	mLength = RandGen::instance()->generateF( SettingsManager::instance()->value("road/segmentMinLengthM").toDouble(), SettingsManager::instance()->value("road/segmentMaxLengthM").toDouble() );
	mStartWidth = mEndWidth = SettingsManager::instance()->value("road/defaultWidthM").toDouble();

	// decide if bend or straight
	double straightProb = RandGen::instance()->generateF();
	if( straightProb < SettingsManager::instance()->value("road/straightVsBendSegmentProbability").toDouble() )
	{
		mCurvature = 0;
	}
	else // Generate random radius for bend
	{
		mCurvature = 1.0/RandGen::instance()->generateF( SettingsManager::instance()->value("road/bendMinRadiusM").toDouble(), SettingsManager::instance()->value("road/bendMaxRadiusM").toDouble() );
		// check for max turn and adjust length if necessary
		double maxTurnRad = SettingsManager::instance()->value("road/bendMaxTurnDegree").toDouble() / 180.0 * M_PI;
		if( maxTurnRad / mCurvature < mLength )
			{ mLength = maxTurnRad / mCurvature; }
		if( RandGen::instance()->generateF() > 0.5 ) { mCurvature *= -1; }
	}

	qDebug() << QString("Road segment created ( @%1(%2,%3/%4), L=%5, R/C=%6/%7 )")
				.arg(mStartLoc.parameter)
				.arg(mStartLoc.x)
				.arg(mStartLoc.y)
				.arg(mStartLoc.heading)
				.arg( mLength )
				.arg(radius())
				.arg(mCurvature);
}

RoadSegment::RoadSegment(long segmentId, roadLocation_t startRoadLocation, double curvature, double length, double startWidth, double endWidth) :
	mSegmentId(segmentId), mStartLoc(startRoadLocation), mCurvature(curvature), mLength(length), mStartWidth(startWidth), mEndWidth(endWidth)
{
	initSettings();

	if( mCurvature != 0 )
	{
		double maxTurnRad = SettingsManager::instance()->value("road/bendMaxTurnDegree").toDouble() / 180.0 * M_PI;
		if( maxTurnRad / fabs(mCurvature) < mLength )
			{ mLength = maxTurnRad / mCurvature; }
	}
	qDebug() << QString("Road segment created ( @%1(%2,%3/%4), L=%5, R/C=%6/%7 )")
				.arg(mStartLoc.parameter)
				.arg(mStartLoc.x)
				.arg(mStartLoc.y)
				.arg(mStartLoc.heading)
				.arg( mLength )
				.arg(radius())
				.arg(mCurvature);
}

RoadSegment::RoadSegment(const RoadSegment &other) : mSegmentId(other.mSegmentId)
{
	copy(other);
}

double RoadSegment::length() const
{
	return mLength;
}

double RoadSegment::widthAt(const double roadParamFromSegmentStart) const
{
	return mStartWidth + (mEndWidth-mStartWidth) * (roadParamFromSegmentStart/mLength);
}

RoadSegment::roadLocation_t RoadSegment::endLocation( double paramFromSegmentStart ) const
{
	QVector2D deltaPos;
	roadLocation_t endLoc;
	double alpha = paramFromSegmentStart*mCurvature;

	if( mCurvature == 0.0 )
	{
		deltaPos.setX(0.0);
		deltaPos.setY(paramFromSegmentStart);
	}
	else
	{
		deltaPos.setX( (1-cos(alpha)) / mCurvature );
		deltaPos.setY( sin(alpha) / mCurvature );
	}
	// rotate (coord. sys) with current heading (watch the sign! geometric  rotation "to the left" is positive!, but rotating the coord. sys left is like rotating the point right...)
	QVector2D deltaPosRot(deltaPos);
	if( mStartLoc.heading != 0.0 )
	{
		double rotAngle = mStartLoc.heading;
		deltaPosRot.setX( deltaPos.x()*cos(rotAngle) + deltaPos.y()*sin(rotAngle) );
		deltaPosRot.setY( -deltaPos.x()*sin(rotAngle) + deltaPos.y()*cos(rotAngle) );
	}

	endLoc.parameter = mStartLoc.parameter + paramFromSegmentStart;
	endLoc.x = mStartLoc.x + deltaPosRot.x();
	endLoc.y = mStartLoc.y + deltaPosRot.y();
	endLoc.heading = mStartLoc.heading + alpha;

	return endLoc;
}

RoadSegment::roadLocation_t RoadSegment::endLocation() const
{
	return endLocation(mLength);
}

RoadSegment &RoadSegment::operator=(const RoadSegment &other)
{
	return copy(other);
}

RoadSegment &RoadSegment::copy(const RoadSegment &other)
{
	mSegmentId = other.mSegmentId;
	mStartLoc = other.mStartLoc;
	mCurvature = other.mCurvature;
	mLength = other.mLength;
	mStartWidth = other.mStartWidth;
	mEndWidth = other.mEndWidth;
	return *this;
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
