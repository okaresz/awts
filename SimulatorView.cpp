#include "SimulatorView.h"
#include <QPaintEvent>
#include <QPainter>
#include "SettingsManager.h"

/* NOTES
 * - left bend radius is negative.
 * - a turn span angle is always positive. */

SimulatorView::SimulatorView(Simulator *simulator, QWidget *parent) : QWidget(parent),
	mSimulator(simulator), mPixelPerMeter(6.0)
{
	if( !SettingsManager::instance()->contains("simulatorView/defaultPixelPerMeter") )
		{ SettingsManager::instance()->setValue("simulatorView/defaultPixelPerMeter", mPixelPerMeter); }
	else
		{ mPixelPerMeter = SettingsManager::instance()->value("simulatorView/defaultPixelPerMeter").toDouble(); }

    setMinimumSize(QSize(400,400));
	connect( simulator, SIGNAL(simUpdated()), this, SLOT(update()) );
}

void SimulatorView::setPixelPerMeter(double pxPerM)
{
	if( pxPerM > 0.1 && pxPerM < 500 )
	{
		mPixelPerMeter = pxPerM;
		update();
	}
}

void SimulatorView::paintRoadSegment(QPainter *painter, RoadSegment *segment, double paintFromNormalPos, double paintToNormalPos , QPointF &endPoint, double &endTangent)
{
	// inside here, one paint unit is one meter
	// painter should be at desired starting point coordinates

	QPen sideLinePen;
	sideLinePen.setStyle(Qt::SolidLine);
	sideLinePen.setColor(Qt::white);
	sideLinePen.setWidthF(0.3);
	sideLinePen.setCapStyle(Qt::FlatCap);

	QPen roadBodyAsPen;
	QColor roadBodyColor(200,200,200);
	roadBodyAsPen.setStyle(Qt::SolidLine);
	roadBodyAsPen.setColor(roadBodyColor);
	roadBodyAsPen.setWidthF(segment->widthAt(0));
	roadBodyAsPen.setCapStyle(Qt::FlatCap);
	QBrush roadBodyAsBrush(roadBodyColor,Qt::SolidPattern);

	QPen centerLinePen;
	double centerLineWidth = 0.3;
	centerLinePen.setStyle(Qt::DashLine);
	QVector<qreal> dashes;
	dashes << 4.0/centerLineWidth << 4.0/centerLineWidth; // dash pattern is specified in units of the pens width
	centerLinePen.setDashPattern(dashes);
	centerLinePen.setColor(Qt::white);
	centerLinePen.setWidthF(centerLineWidth);
	centerLinePen.setCapStyle(Qt::FlatCap);
	centerLinePen.setDashOffset((segment->odoStartLoc()+segment->length()*paintFromNormalPos)/centerLineWidth);

	if( segment->isBend() )
	{
		double width = segment->widthAt(0);
		{	// road body and centerline
			double radiusAbs = segment->radiusAbs();
			double radius = segment->radius();
			double fullAlpha = segment->length() / segment->radiusAbs();
			double drawnAlpha = fullAlpha * (paintToNormalPos-paintFromNormalPos);
			QRectF arcBounds;
			QPointF startPos = QPointF(0,0);
			arcBounds.setBottomRight(QPointF(startPos.x(),startPos.y()+radiusAbs));
			arcBounds.setTopLeft(QPointF(startPos.x()+radius*2,startPos.y()-radiusAbs));

			QPainterPath bendPath(startPos);
			bendPath.arcTo( arcBounds, 0.0, drawnAlpha/M_PI*180.0 );
			painter->setPen(roadBodyAsPen);
			painter->setBrush(Qt::NoBrush);
			painter->drawPath(bendPath);
			painter->setPen(centerLinePen);
			painter->setBrush(Qt::NoBrush);
			painter->drawPath(bendPath);

			endPoint = bendPath.currentPosition();
			if( segment->radius() >= 0 ) { drawnAlpha *= -1; }
			endTangent = M_PI_2 + drawnAlpha;
		}
		painter->setPen(sideLinePen);
		painter->setBrush(Qt::NoBrush);
		{	// outer sideLine
			double radiusAbs = segment->radiusAbs()+width/2;
			double radius = radiusAbs;
			QPointF startPos = QPointF(width/2,0);
			if( segment->radius() < 0 )
				{ radius *= -1; }
			else
				{ startPos.setX(startPos.x()*-1); }
			double fullAlpha = segment->length() / segment->radiusAbs();
			double drawnAlpha = fullAlpha * (paintToNormalPos-paintFromNormalPos);
			QRectF arcBounds;
			arcBounds.setBottomRight(QPointF(startPos.x(),startPos.y()+radiusAbs));
			arcBounds.setTopLeft(QPointF(startPos.x()+radius*2,startPos.y()-radiusAbs));

			QPainterPath bendPath(startPos);
			bendPath.arcTo( arcBounds, 0.0, drawnAlpha/M_PI*180.0 );
			painter->drawPath(bendPath);
		}
		{	// inner sideLine
			double radiusAbs = segment->radiusAbs()-width/2; if( radiusAbs < 0 ) { radiusAbs = 1; qWarning("Road inner radius < 0!"); }
			double radius = radiusAbs;
			QPointF startPos = QPointF(-width/2,0);
			if( segment->radius() < 0 )
				{ radius *= -1; }
			else
				{ startPos.setX(startPos.x()*-1); }
			double fullAlpha = segment->length() / segment->radiusAbs();
			double drawnAlpha = fullAlpha * (paintToNormalPos-paintFromNormalPos);
			QRectF arcBounds;
			arcBounds.setBottomRight(QPointF(startPos.x(),startPos.y()+radiusAbs));
			arcBounds.setTopLeft(QPointF(startPos.x()+radius*2,startPos.y()-radiusAbs));

			QPainterPath bendPath(startPos);
			bendPath.arcTo( arcBounds, 0.0, drawnAlpha/M_PI*180.0 );
			painter->drawPath(bendPath);
		}
	}
	else	// straight segment
	{
		double widthStart = segment->widthAt(0.0);
		double widthEnd = segment->widthAt(segment->length());
		double drawnLength = segment->length() * (paintToNormalPos-paintFromNormalPos);

		// road body
		painter->setPen(Qt::NoPen);
		painter->setBrush(roadBodyAsBrush);
		QPointF roadPolygonPoints[4] = {
			QPointF(widthStart/2,0),
			QPointF(widthEnd/2,-drawnLength),
			QPointF(-widthEnd/2,-drawnLength),
			QPointF(-widthStart/2,0)
		};
		painter->drawConvexPolygon(roadPolygonPoints,4);

		// left sideLine
		painter->setPen(sideLinePen);
		painter->drawLine(
					QPointF( -widthStart/2, 0 ),
					QPointF( -widthEnd/2, -drawnLength ));

		// right sideLine
		painter->setPen(sideLinePen);
		painter->drawLine(
					QPointF( widthStart/2, 0 ),
					QPointF( widthEnd/2, -drawnLength ));

		// centerLine
		painter->setPen(centerLinePen);
		painter->drawLine(
					QPointF( 0, 0 ),
					QPointF( 0, -drawnLength ));

		endPoint.setX( 0 );
		endPoint.setY( -drawnLength );
		endTangent = M_PI_2;
	}
}

void SimulatorView::paintRoadObstaclesOnSegment(QPainter *painter, const RoadGenerator *roadGen, RoadSegment *segment, double paintFromNormalPos, double odometer, double visibility)
{
	QColor obstacleColor(200,20,20);
	QBrush obstacleBrush(obstacleColor,Qt::SolidPattern);
	painter->setPen(Qt::NoPen);
	painter->setBrush(obstacleBrush);

	const QQueue<RoadObstacle*> *obstacles = roadGen->obstacles();
	for( int i=0; i<obstacles->size(); ++i )
	{
		RoadObstacle *obst = obstacles->at(i);
		if( !( obst->odoPos() > odometer &&
			   obst->odoPos() > segment->odoStartLoc() &&
			   obst->odoPos() < odometer+visibility &&
			   obst->odoPos() < segment->odoEndLoc() ) )
			{ continue; }

		double obstaclePosAlongPathFromOrigin = obst->odoPos() - segment->odoStartLoc() - (segment->length()*paintFromNormalPos);
		QPointF obstaclePos;
		if( segment->isBend() )
		{
			double gamma = obstaclePosAlongPathFromOrigin / segment->radiusAbs();
			double obstacleRadius = -segment->radius() + obst->normalPos()*(segment->widthAt(obstaclePosAlongPathFromOrigin)/2);	// signed! (sign of roadRadius determines the sign of the obstacle X coord)
			obstaclePos.setX( segment->radius() + obstacleRadius * cos(gamma) );
			obstaclePos.setY( -fabs(obstacleRadius) * sin(gamma) );
		}
		else
		{
			obstaclePos.setX( obst->normalPos() * (segment->widthAt(obstaclePosAlongPathFromOrigin)/2) );
			obstaclePos.setY( -obstaclePosAlongPathFromOrigin );
		}

		painter->drawEllipse( obstaclePos, obst->size()/2, obst->size()/2 );
	}
}

void SimulatorView::paintRoad( QPainter *painter, QPointF startPoint, double odometer, double visibility )
{
	if( mSimulator->roadGen()->segments()->isEmpty() )
		{ return; }

	painter->save();

	painter->scale(mPixelPerMeter,mPixelPerMeter);
	QPointF endPoint = startPoint;
	double endTangent = 0;
	double horizon = odometer+visibility;

	for( int i=0; (i<mSimulator->roadGen()->segments()->size()) && (mSimulator->roadGen()->segments()->at(i)->odoStartLoc()<horizon); ++i )
	{
		RoadSegment *segment = mSimulator->roadGen()->segments()->at(i);
		double paintFromNormalPos = 0.0;
		if( segment->odoStartLoc() < odometer )
			{ paintFromNormalPos = (odometer - segment->odoStartLoc()) / segment->length(); }
		double paintToNormalPos = 1.0;
		if( segment->odoEndLoc() > horizon )
			{ paintToNormalPos = (horizon - segment->odoStartLoc()) / segment->length(); }

		paintRoadSegment(painter, segment, paintFromNormalPos, paintToNormalPos, endPoint, endTangent);
		paintRoadObstaclesOnSegment(painter, mSimulator->roadGen(), segment, paintFromNormalPos, odometer, visibility);

		/* transform the coordinate system to the segment end (next segment start)
		* and rotate so that current road tangent points always "upward"
		* (because arc drawing begins at zero degrees, going anti-clockwise)*/
		painter->translate(endPoint);
		painter->rotate(90-endTangent/M_PI*180.0);
	}

	painter->restore();
}

void SimulatorView::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);

	double odometer = mSimulator->car()->odometer();
	const double visibilityM = mSimulator->roadVisibility();

	// init painting
	QPainter painter;
	painter.begin(this);
	painter.setRenderHint(QPainter::Antialiasing);

	QSize viewPortSize(size());

	// background
	painter.fillRect(QRect(0,0,viewPortSize.width(),viewPortSize.height()), QBrush(Qt::green));

	QPointF carCenter(viewPortSize.width()/2,viewPortSize.height()-20);
	painter.translate(carCenter);
	carCenter = QPointF(0.0,0.0);

	paintRoad( &painter, carCenter, odometer, visibilityM );
	paintCar( &painter, carCenter, QSize(1.6*mPixelPerMeter,4.0*mPixelPerMeter) );

	painter.end();
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
