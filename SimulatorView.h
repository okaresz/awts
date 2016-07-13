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
	void onCarTractionLost(double atTravel);
	void onCarCrashed(double atTravel);
	void onCarUnavoidableTractionLossDetected(double atTravel);
	void onCarUnavoidableCrashDetected(double atTravel);

protected:
    void paintEvent(QPaintEvent *event);
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);
	void resizeEvent(QResizeEvent *event);

private:
	void paintCar(QPainter *painter, const QPointF carCenterPoint, const double carHeading, const QSize carSize );
	/** Paint road.
	 *  @param[out] endPoint: set to the endPoint (on the centerline) of the last drawn segment.*/
	void paintRoad(QPainter *painter, const QPointF startPoint, double travel, double visibility , QPointF &endPoint);
	void paintTargetCrossPos(QPainter *painter, const QPointF startPoint);
	void paintSpeedometer(QPainter *painter, const QPointF centerPoint, const QSizeF size);
	void paintGmeter(QPainter *painter, const QPointF centerPoint, const QSizeF size);

	/** Paint road segment from the current painter position (0,0), straight road points upwards.
	 *	@param paintFromNormalPos Paint the segment from this normalized position along segment length.
	 *	@param paintToNormalPos Paint the segment until this normalized position along segment length.
	 *	@param[out] endPoint The point where the segment drawing ended.
	 *	@param[out] endTangent The tangent at the segment drawing endPoint.*/
	void paintRoadSegment(QPainter *painter, RoadSegment *segment, double paintFromNormalPos, double paintToNormalPos, QPointF &endPoint, double &endTangent );
	/** Paint road segment obstacles from the current painter position (0,0), straight road points upwards.
	 *	Params are similar to paintRoadSegment().*/
	void paintRoadObstaclesOnSegment(QPainter *painter, const RoadGenerator *roadGen, RoadSegment *segment, double paintFromNormalPos, double travel, double visibility);

	Simulator *mSimulator;
	double mPixelPerMeter;
	bool mPxPerMeterSetForViewSize;
	QColor mBackgroundColor;
	bool mEaster;
};

#endif // SIMVIEWWIDGET_H
