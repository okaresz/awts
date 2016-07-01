#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <QObject>
#include <QTimer>
#include "RoadGenerator.h"

/// Simulator is responsible for running the simulation and managing all the components, interactions.
class Simulator : public QObject
{
	Q_OBJECT
public:
	explicit Simulator(QObject *parent = 0);

signals:

public slots:
	/// Start simulation.
	void start();
	/// Update simulation periodically
	void simUpdate();

	const RoadGenerator *roadGen() const
		{ return &mRoadGen; }

	const double roadVisibility() const
		{ return mRoadVisibility; }

private:
	QTimer mSimTimer;
	quint64 mStartTimeStamp;
	RoadGenerator mRoadGen;
	double mRoadVisibility;
};

#endif // SIMULATOR_H
