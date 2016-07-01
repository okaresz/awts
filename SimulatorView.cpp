#include "SimulatorView.h"
#include <QPaintEvent>
#include <QPainter>

SimulatorView::SimulatorView(Simulator *simulator, QWidget *parent) : QWidget(parent),
	mSimulator(simulator)
{
    setMinimumSize(QSize(400,400));
    update();
}

void SimulatorView::paintEvent(QPaintEvent *event)
{
	static double odoMeter = 0; // temp fake
	const double visibilityM = mSimulator->roadVisibility();

	// init painting
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);

	QSize viewPortSize(size());

    // background
	//painter.fillRect(event->rect(), QBrush(Qt::white));
	painter.fillRect(QRect(0,0,viewPortSize.width(),viewPortSize.height()), QBrush(Qt::white));

	// ---------------------
	QPointF carCenter(viewPortSize.width()/2,viewPortSize.height()-10);
	painter.translate( carCenter );
	carCenter = QPointF(0.0,0.0);

	// car
	//paintCar( &painter, carCenter, QSize(60,150) );

	/* PAINT ROAD SEGMENTS
	* these are painted per segment, coord sys transformed before very segment
	* so that segment drawing can take place with no rotation*/
	painter.save();

	const RoadSegment *segment;
	double endTangent = M_PI_2;
	QPointF endPoint(carCenter);
	for( int i=0; i<mSimulator->roadGen()->segments()->size(); i++ )
	{
		segment = mSimulator->roadGen()->segments()->at(i);

		// calculate segment length to draw, based on current car position (odometer) and road visibility range
		if( segment->odoEndLoc() <= odoMeter )
			{ continue; }	// segment left behind

		double posOnSegment = 1 - (segment->odoEndLoc() - odoMeter) / segment->length();
		if( posOnSegment < 0 )	// negative means this is a future segment, should be drawn fully
			{ posOnSegment = 0; }

		double drawnLength = segment->length()*(1-posOnSegment);
		double segmentLengthOverVisibility = segment->odoStartLoc() + drawnLength - (odoMeter + visibilityM);
		if( segmentLengthOverVisibility > 0 )
			{ drawnLength -= segmentLengthOverVisibility; }
		if( drawnLength <= 0 )
			{ break; }

		/* transform the coordinate system to the segment end (next segment start)
		* and rotate so that current road tangent points always "upward"
		* (because arc drawing begins at zero degrees, going anti-clockwise)*/
		painter.translate(endPoint);
		painter.rotate(90-endTangent/M_PI*180.0);

		QPainterPath roadPath;
		if( !segment->isBend() )
		{
			roadPath.lineTo( QPointF(0,-drawnLength ) );
		}
		else
		{
			QRectF arcBounds;
			QPointF startPos = QPointF(0,0);
			arcBounds.setBottomRight(QPointF(startPos.x(),startPos.y()+fabs(segment->radius())/2));
			arcBounds.setTopLeft(QPointF(startPos.x()-segment->radius(),startPos.y()-fabs(segment->radius())/2));
			roadPath.arcTo( arcBounds, 0.0, drawnLength );

			// draw arcBound rect for debug
			QPen dbgPen;
			dbgPen.setWidth(1);
			dbgPen.setColor(QColor(Qt::cyan));
			painter.setPen(dbgPen);
			painter.drawRect(arcBounds);
		}
		endTangent = roadPath.angleAtPercent(1)/180.0*M_PI;
		endPoint = roadPath.currentPosition();

		QPen roadPen;
		roadPen.setWidth(3);
		roadPen.setColor(QColor(50, 50, 50));
		painter.setPen(roadPen);
		painter.drawPath(roadPath);
	}

	painter.restore();
	// END OF ROAD PAINTING

	painter.end();

	odoMeter += 0.8;
}

void SimulatorView::paintCar(QPainter *painter, const QPointF carCenterPoint, const QSize carSize)
{
	painter->save();

	QPixmap carImg("../awts/images/car.png");
	carImg = carImg.scaledToWidth(carSize.width());
	QPoint carOrigin;
	carOrigin.setX( carCenterPoint.x()-carSize.width()/2 );
	carOrigin.setY( carCenterPoint.y()-carSize.height()/2 );
	if( !carImg.isNull() )
	{
		painter->drawPixmap(carOrigin, carImg);
	}
	else
	{
		qWarning("Car pixmap could not be loaded! Falling back to rectangle.");
		QPen pen(Qt::black);
		pen.setWidth(1);
		painter->setPen(pen);
		QBrush brush(QColor(0,127,0));
		painter->setBrush(brush);
		painter->drawRoundedRect(QRect(carOrigin,carSize), 10, 10);
	}

	painter->restore();
}
