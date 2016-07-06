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
	void simUpdate(const quint64 simTime, const double simOdometer);

	QQueue<RoadSegment> visibleRoad(const double odometer) const;

	QQueue<RoadObstacle> visibleObstacles(const double odometer) const;

	const QQueue<RoadSegment*> *segments() const
		{ return &mSegmentQueue; }

	const QQueue<RoadObstacle*> *obstacles() const
		{ return &mObstacleQueue; }

	const RoadSegment *segmentAtOdo( double odometerVal );

private:
	/// Return next bend segment ahead of given odometer value.
	const RoadSegment *nextBend( double odometerVal );

	/// Get the generated road length ahead in meters.
	double lengthAhead(double simOdometer ) const;
	/// Get the location of the end of the generated road.
	double endOfGeneratedRoad() const;

	void deleteObstaclesBefore(double odoMark);

	double mRoadGenerationHorizon;
	double mRoadVisibility;
	QQueue<RoadSegment*> mSegmentQueue;
	QQueue<RoadObstacle*> mObstacleQueue;
};

#endif // ROADGENERATOR_H
