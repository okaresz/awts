#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <QObject>

class TrajectoryPlanner : public QObject
{
	Q_OBJECT
public:
	explicit TrajectoryPlanner(QObject *parent = 0);

signals:

public slots:
};

#endif // TRAJECTORYPLANNER_H