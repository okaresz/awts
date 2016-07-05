#ifndef TRAJECTORYSECTION_H
#define TRAJECTORYSECTION_H

#include <QPointF>

class TrajectorySection
{
public:
	enum sectionType {
		StraightSection,
		BendSection,
		LaneShiftSection,
		BendLaneShiftSection
	};

	TrajectorySection( const sectionType type );

	sectionType type() const
		{ return mType; }

	virtual double maxSpeed() const
		{ return mMaxSpeed; }

	/** Get the start position of the trajectory.
	 *  @return The start position. The X coordinate is the position across the road, in meters from the road center (signed).
	 *	The Y coordinate is the odometer position along the road.*/
	virtual QPointF startPos() const
		{ return mStartPos; }

	/** Get the end position of the trajectory.
	 *  @return The end position. The X coordinate is the position across the road, in meters from the road center (signed).
	 *	The Y coordinate is the odometer position along the road.*/
	virtual QPointF endPos() const = 0;

	/** Set new length for section.
	 *  @param newLength Length (in meters, along the path!) to set.*/
	virtual void setLength( double newLength )
		{ mLength = newLength; }

protected:
	sectionType mType;
	double mMaxSpeed;
	QPointF mStartPos;
	double mLength;
};

class TrajectorySectionStraight : public TrajectorySection
{
public:
	TrajectorySectionStraight( const QPointF startPos, const double length );

	QPointF endPos() const
		{ return QPointF(0.0,0.0); }
};

class TrajectorySectionBend : public TrajectorySection
{
public:
	TrajectorySectionBend( const QPointF startPos, const double radius, const double length );

	double radius() const
		{ return mRadius; }

	QPointF endPos() const
		{ return QPointF(0.0,0.0); }

private:
	double mRadius;
};

class TrajectorySectionLaneShift : public TrajectorySection
{
public:
	TrajectorySectionLaneShift( const QPointF startPos, const double shift );

	double shift() const
		{ return mShift; }

	QPointF endPos() const
		{ return QPointF(0.0,0.0); }

private:
	double mShift;
};

class TrajectorySectionBendLaneShift : public TrajectorySectionLaneShift, public TrajectorySectionBend
{
public:
	TrajectorySectionBendLaneShift( const QPointF startPos, const double radius, const double shift );

	QPointF endPos() const
		{ return QPointF(0.0,0.0); }

private:

};

#endif // TRAJECTORYSECTION_H
