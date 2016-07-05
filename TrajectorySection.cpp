#include "TrajectorySection.h"

TrajectorySection::TrajectorySection(const sectionType type)
{
	mType = type;
}

TrajectorySectionStraight::TrajectorySectionStraight(const QPointF startPos, const double length) : TrajectorySection(StraightSection)
{
	mStartPos = startPos;
	mLength = length;
}

TrajectorySectionBend::TrajectorySectionBend(const QPointF startPos, const double radius, const double length) : TrajectorySection(BendSection)
{
	mStartPos = startPos;
	mLength = length;
	mRadius = radius;
}

TrajectorySectionLaneShift::TrajectorySectionLaneShift(const QPointF startPos, const double shift) : TrajectorySection(LaneShiftSection)
{
	mStartPos = startPos;
	mShift = shift;
}

TrajectorySectionBendLaneShift::TrajectorySectionBendLaneShift(const QPointF startPos, const double radius, const double shift) : TrajectorySectionLaneShift(startPos,shift), TrajectorySectionBend(startPos,radius,0)
{

}
