#ifndef ROADGENERATOR_H
#define ROADGENERATOR_H

#include <QObject>
#include <QQueue>
#include "RoadSegment.h"

class RoadGenerator : public QObject
{
    Q_OBJECT
public:
    explicit RoadGenerator(QObject *parent = 0);

signals:

public slots:
	/** Simulation update.
	 * Generate more road if necessary.*/
	void simUpdate(double simOdometer );

	/// Get the generated road length ahead in meters.
	double lengthAhead(double simOdometer ) const;
	/// Get the location of the end of the generated road.
	double endOfRoad() const;

	const QQueue<RoadSegment*> *segments() const
		{ return &mSegmentQueue; }

private:
	QQueue<RoadSegment*> mSegmentQueue;
};

#endif // ROADGENERATOR_H
