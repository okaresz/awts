#ifndef ROADSEGMENT_H
#define ROADSEGMENT_H

#include <QObject>

class RoadSegment : public QObject
{
    Q_OBJECT
public:
	explicit RoadSegment( double odoStartLoc, QObject *parent = 0);
	RoadSegment(double odoStartLoc, double radius, double length, double startWidth, double endWidth, QObject *parent = 0);

signals:

public slots:
	/** Get segment length along the centerline.
	 *  return The length in meters.*/
	double length() const;
	/// Get the radius of the segment (signed! right turns are negative)
	double radius() const
		{ return mRadius; }
	/// Get the absolute value of the segment radius
	double radiusAbs() const
		{ return fabs(mRadius); }
	bool isBend() const
		{ return mRadius != 0.0; }
	/** Get segment width at given point.
	 *  return The width in meters.*/
	double widthAt( const double metersFromSegmentStart ) const;
	double odoStartLoc() const
		{ return mOdoStartLoc; }
	double odoEndLoc() const
		{ return mOdoStartLoc+mLength; }

private:
	void initSettings();

	double mOdoStartLoc;	///< Starting location of the segment along the road (got from odometer)
	double mRadius; ///< Radius of the centerline
	double mLength; ///< Length of the centerline
	double mStartWidth;  ///< Width at the start of the segment
	double mEndWidth;  ///< Width at the end of the segment
};

#endif // ROADSEGMENT_H
