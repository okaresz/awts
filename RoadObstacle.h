#ifndef ROADOBSTACLE_H
#define ROADOBSTACLE_H

class RoadObstacle
{
public:
	RoadObstacle(const double odoPosition, const double normalPosition = 0.0, const double size = 0.0);

	/** Get the absolute odometer position of the obstacle.*/
	double odoPos() const
		{ return mOdoPos; }

	/** Get the normalized obstacle position across the road.
	 *  @return positin across road given in the range of [-1.0,1.0] */
	double normalPos() const
		{ return mNormalPos; }

	/** Get the size (diameter) of the obstacle.*/
	double size() const
		{ return mSize; }

private:
	double mOdoPos;	///< position along road in odometer meters
	double mNormalPos; ///< position across road given in the range of [-1.0,1.0]
	double mSize;	///< Diameter of obstacle in meters
};

#endif // ROADOBSTACLE_H
