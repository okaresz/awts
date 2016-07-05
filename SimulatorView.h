#ifndef SIMVIEWWIDGET_H
#define SIMVIEWWIDGET_H

#include <QWidget>
#include "Simulator.h"

class SimulatorView : public QWidget
{
    Q_OBJECT
public:
	explicit SimulatorView( Simulator *simulator, QWidget *parent = 0);

	Simulator *simulator() const
		{ return mSimulator; }

	double pixelPerMeter() const
		{ return mPixelPerMeter; }

signals:
	void pixelPerMeterChanged(double val);

public slots:
	void setPixelPerMeter( double pxPerM );

protected:
    void paintEvent(QPaintEvent *event);

private:
	void paintCar(QPainter *painter, const QPointF carCenterPoint, const QSize carSize );
	void paintRoad(QPainter *painter, QPointF startPoint, double odometer, double visibility );

	/** Paint road segment from the current painter position (0,0), straight road points upwards.
	 *	@param paintFromNormalPos Paint the segment from this normalized position along segment length.
	 *	@param paintToNormalPos Paint the segment until this normalized position along segment length.
	 *	@param[out] endPoint The point where the segment drawing ended.
	 *	@param[out] endTangent The tangent at the segment drawing endPoint.*/
	void paintRoadSegment(QPainter *painter, RoadSegment *segment, double paintFromNormalPos, double paintToNormalPos, QPointF &endPoint, double &endTangent );
	/** Paint road segment obstacles from the current painter position (0,0), straight road points upwards.
	 *	Params are similar to paintRoadSegment().*/
	void paintRoadObstaclesOnSegment(QPainter *painter, const RoadGenerator *roadGen, RoadSegment *segment, double paintFromNormalPos, double odometer, double visibility);

	Simulator *mSimulator;
	double mPixelPerMeter;
};

#endif // SIMVIEWWIDGET_H
