#ifndef ROADSEGMENT_H
#define ROADSEGMENT_H

#include <QtGlobal>

class RoadSegment
{
public:
	struct roadLocation_t
	{
		double parameter;	///< road curve parameter from the beginning (of simulation), unit in meters.
		double x,y;			///< x,y coordinates of road point in 2D space, calculated from the beginning of the simulation.
		double heading;		///< heading angle, 0 being the road is pointing "upwards".
		roadLocation_t()
			{ parameter = x = y = heading = 0.0; }
		roadLocation_t(const double parameter,const double x,const double y,const double heading)
		{
			this->parameter = parameter;
			this->x = x;
			this->y = y;
			this->heading = heading;
		}
	};

	RoadSegment(roadLocation_t startLocation);
	/// constructor for copying another segment with modified parameters.
	RoadSegment(long int segmentId, roadLocation_t startRoadLocation, double curvature, double length, double startWidth, double endWidth);
	RoadSegment( const RoadSegment &other );

	/** Get segment length along the centerline.
	 *  return The length in meters.*/
	double length() const;

	/// Get the radius of the segment (signed! left turns are negative, zero is straight road)
	double radius() const
	{
		if( mCurvature == 0.0 )
			{ return 0.0; }
		else
			{ return 1/mCurvature; }
	}

	double curvature() const
	{
		return mCurvature;
	}

	/// Get the absolute value of the segment radius
	double radiusAbs() const
		{ return qAbs(radius()); }

	bool isBend() const
		{ return mCurvature != 0.0; }

	/** Get segment width at given point.
	 *  return The width in meters.*/
	double widthAt( const double metersFromSegmentStart ) const;

	roadLocation_t startLocation() const
		{ return mStartLoc; }

	/// Get road location at the end of the segment
	roadLocation_t endLocation() const;
	/// Get road location on the segment, paramFromSegmentStart meters from the segment start.
	roadLocation_t endLocation(double paramFromSegmentStart) const;

	double endRoadParam() const
		{ return mStartLoc.parameter+mLength; }

	double startWidth() const
		{ return mStartWidth; }
	double endWidth() const
		{ return mEndWidth; }

	long int segmentId() const
		{ return mSegmentId; }

	bool operator==(const RoadSegment &other) const
		{ return this->mSegmentId == other.mSegmentId; }

	RoadSegment& operator=(const RoadSegment &other);
	RoadSegment& copy(const RoadSegment &other);

private:
	void initSettings();

	static long int segmentCounter;
	long int mSegmentId;
	roadLocation_t mStartLoc;	///< Starting location of the segment
	double mCurvature; ///< Radius of the centerline
	double mLength; ///< Length of the centerline, expressed in road curve parameter space
	double mStartWidth;  ///< Width at the start of the segment
	double mEndWidth;  ///< Width at the end of the segment
};

#endif // ROADSEGMENT_H
