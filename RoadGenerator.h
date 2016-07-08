#ifndef ROADGENERATOR_H
#define ROADGENERATOR_H

#include <QObject>
#include <QQueue>
#include "RoadSegment.h"
#include "RoadObstacle.h"

class RoadGenerator : public QObject
{
    Q_OBJECT
public:
	explicit RoadGenerator(double roadVisibility, QObject *parent = 0);

	void setRoadGenerationHorizon(double roadGenerationHorizon)
		{ mRoadGenerationHorizon = roadGenerationHorizon; }

signals:

public slots:
	/** Simulation update.
	 * Generate more road if necessary.*/
	void simUpdate(const quint64 simTime, const double travel);

	/** Get visible road.
	 *  Get segments which start param < travel, but limit their length to the extent the visibility allows.
	 *  So segments startRoadLoc is unchanged, but their length may be truncated.
	 *	@return A the visible portion (copy) of the segmentQueue.*/
	QQueue<RoadSegment> visibleRoad(const double travel) const;

	/** Get visible obstacles.
	 *	@return A copy of the obstacleQueue with only the visible obstacles.*/
	QQueue<RoadObstacle> visibleObstacles(const double travel) const;

	const QQueue<RoadSegment*> *segments() const
		{ return &mSegmentQueue; }

	const QQueue<RoadObstacle*> *obstacles() const
		{ return &mObstacleQueue; }

	const RoadSegment *segmentAt(double roadParam ) const;

	/// Get road location at specified parameter value.
	RoadSegment::roadLocation_t location( double roadParam ) const;

private:
	/// Return next bend segment ahead of given odometer value.
	const RoadSegment *nextBend( double travel );

	/// Get the generated road length ahead in meters.
	double lengthAhead(double travel ) const;

	void deleteObstaclesBefore(double travel);

	double mRoadGenerationHorizon;
	double mRoadVisibility;
	QQueue<RoadSegment*> mSegmentQueue;
	QQueue<RoadObstacle*> mObstacleQueue;
};

#endif // ROADGENERATOR_H
