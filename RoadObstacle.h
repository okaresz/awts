#ifndef ROADOBSTACLE_H
#define ROADOBSTACLE_H

#include <QObject>

class RoadObstacle : public QObject
{
	Q_OBJECT
public:
	explicit RoadObstacle(const double odoPosition, const double crossPosition = 0.0, const double size = 0.0, QObject *parent = 0);

signals:

public:
	/** Get the absolute odometer position of the obstacle.*/
	double odoPos() const
		{ return mOdoPos; }

	/** Get the normalized obstacle position across the road.
	 *  @return positin across road given in the range of [-1.0,1.0] */
	double crossPos() const
		{ return mCrossPos; }

	/** Get the size (diameter) of the obstacle.*/
	double size() const
		{ return mSize; }

public slots:

private:
	double mOdoPos;	///< position along road in odometer meters
	double mCrossPos; ///< positin across road given in the range of [-1.0,1.0]
	double mSize;	///< Diameter of obstacle in meters
};

#endif // ROADOBSTACLE_H
