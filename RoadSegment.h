#ifndef ROADSEGMENT_H
#define ROADSEGMENT_H

#include <QtGlobal>

class RoadSegment
{
public:
	RoadSegment(double odoStartLoc);
	RoadSegment(double odoStartLoc, double radius, double length, double startWidth, double endWidth);
	RoadSegment( const RoadSegment &other );

public:
	/** Get segment length along the centerline.
	 *  return The length in meters.*/
	double length() const;
	/// Get the radius of the segment (signed! right turns are negative)
	double radius() const
		{ return mRadius; }
	/// Get the absolute value of the segment radius
	double radiusAbs() const
		{ return qAbs(mRadius); }
	bool isBend() const
		{ return mRadius != 0.0; }
	/** Get segment width at given point.
	 *  return The width in meters.*/
	double widthAt( const double metersFromSegmentStart ) const;
	double odoStartLoc() const
		{ return mOdoStartLoc; }
	double odoEndLoc() const
		{ return mOdoStartLoc+mLength; }

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
	double mOdoStartLoc;	///< Starting location of the segment along the road (got from odometer)
	double mRadius; ///< Radius of the centerline
	double mLength; ///< Length of the centerline
	double mStartWidth;  ///< Width at the start of the segment
	double mEndWidth;  ///< Width at the end of the segment
};

#endif // ROADSEGMENT_H
