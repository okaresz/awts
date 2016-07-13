#include "SimulatorView.h"
#include <QPaintEvent>
#include <QPainter>
#include "SettingsManager.h"

/* NOTES
 * - left bend radius is negative.
 * - a turn span angle is always positive. */

SimulatorView::SimulatorView(Simulator *simulator, QWidget *parent) : QWidget(parent),
	mSimulator(simulator), mPixelPerMeter(6.0), mPxPerMeterSetForViewSize(false), mBackgroundColor(37,175,95), mEaster(false)
{
	if( !SettingsManager::instance()->contains("simulatorView/defaultPixelPerMeter") )
		{ SettingsManager::instance()->setValue("simulatorView/defaultPixelPerMeter", mPixelPerMeter); }
	else
		{ mPixelPerMeter = SettingsManager::instance()->value("simulatorView/defaultPixelPerMeter").toDouble(); }

    setMinimumSize(QSize(400,400));
	setFocusPolicy(Qt::ClickFocus);
	connect( simulator, SIGNAL(simUpdated()), this, SLOT(update()) );
	connect( simulator, SIGNAL(carUnavoidableTractionLossDetected(double)), this, SLOT(onCarUnavoidableTractionLossDetected(double)) );
	connect( simulator, SIGNAL(carUnavoidableCrashDetected(double)), this, SLOT(onCarUnavoidableCrashDetected(double)) );
	connect( simulator, SIGNAL(carTractionLost(double)), this, SLOT(onCarTractionLost(double)) );
	connect( simulator, SIGNAL(carCrashed(double)), this, SLOT(onCarCrashed(double)) );
}

void SimulatorView::setPixelPerMeter(double pxPerM)
{
	if( pxPerM > 0.1 && pxPerM < 500 )
	{
		mPixelPerMeter = pxPerM;
		update();
	}
}

void SimulatorView::onCarTractionLost(double atTravel)
{
	Q_UNUSED(atTravel);
	mBackgroundColor = QColor(210,210,20);
	update();
}

void SimulatorView::onCarCrashed(double atTravel)
{
	Q_UNUSED(atTravel);
	mBackgroundColor = QColor(170,90,90);
	update();
}

void SimulatorView::onCarUnavoidableTractionLossDetected(double atTravel)
{
	Q_UNUSED(atTravel);
}

void SimulatorView::onCarUnavoidableCrashDetected(double atTravel)
{
	Q_UNUSED(atTravel);
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
	centerLinePen.setDashOffset((segment->startLocation().parameter+segment->length()*paintFromNormalPos)/centerLineWidth);

	if( segment->isBend() )
	{
		double width = segment->widthAt(0.0);
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
		double widthStart = segment->widthAt(paintFromNormalPos*segment->length());
		double widthEnd = segment->widthAt(paintToNormalPos*segment->length());
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

void SimulatorView::paintRoadObstaclesOnSegment(QPainter *painter, const RoadGenerator *roadGen, RoadSegment *segment, double paintFromNormalPos, double travel, double visibility)
{
	QColor obstacleColor(200,20,20);
	QBrush obstacleBrush(obstacleColor,Qt::SolidPattern);
	painter->setPen(Qt::NoPen);
	painter->setBrush(obstacleBrush);

	const QQueue<RoadObstacle*> *obstacles = roadGen->obstacles();
	for( int i=0; i<obstacles->size(); ++i )
	{
		RoadObstacle *obst = obstacles->at(i);
		if( !( obst->roadParam() > travel &&
			   obst->roadParam() > segment->startLocation().parameter &&
			   obst->roadParam() < travel+visibility &&
			   obst->roadParam() < segment->endRoadParam() ) )
			{ continue; }

		double obstaclePosAlongPathFromOrigin = obst->roadParam() - segment->startLocation().parameter - (segment->length()*paintFromNormalPos);
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

		if( mEaster )
		{
			painter->save();
			painter->translate(obstaclePos);
			painter->scale(1/mPixelPerMeter,1/mPixelPerMeter);
			QPixmap obstImg(":/images/bobstacle.png");
			painter->drawPixmap( QPointF(
									 -obst->size()*mPixelPerMeter/2,
									 -obst->size()*mPixelPerMeter/2),
								 obstImg.scaledToWidth( obst->size()*mPixelPerMeter, Qt::SmoothTransformation ) );
			painter->restore();
		}
		else
			{ painter->drawEllipse( obstaclePos, obst->size()/2, obst->size()/2 ); }
	}
}

void SimulatorView::paintRoad(QPainter *painter, const QPointF startPoint, double travel, double visibility, QPointF &endPoint )
{
	if( mSimulator->roadGen()->segments()->isEmpty() )
		{ return; }

	painter->save();

	painter->scale(mPixelPerMeter,mPixelPerMeter);
	painter->translate(startPoint);
	endPoint = startPoint;
	double endTangent = 0;
	double horizon = travel+visibility;

	for( int i=0; (i<mSimulator->roadGen()->segments()->size()) && (mSimulator->roadGen()->segments()->at(i)->startLocation().parameter<horizon); ++i )
	{
		RoadSegment *segment = mSimulator->roadGen()->segments()->at(i);
		double paintFromNormalPos = 0.0;
		if( segment->startLocation().parameter < travel )
			{ paintFromNormalPos = (travel - segment->startLocation().parameter) / segment->length(); }
		double paintToNormalPos = 1.0;
		if( segment->endRoadParam() > horizon )
			{ paintToNormalPos = (horizon - segment->startLocation().parameter) / segment->length(); }

		paintRoadSegment(painter, segment, paintFromNormalPos, paintToNormalPos, endPoint, endTangent);
		paintRoadObstaclesOnSegment(painter, mSimulator->roadGen(), segment, paintFromNormalPos, travel, visibility);

		/* transform the coordinate system to the segment end (next segment start)
		* and rotate so that current road tangent points always "upward"
		* (because arc drawing begins at zero degrees, going anti-clockwise)*/
		painter->translate(endPoint);
		painter->rotate(90-endTangent/M_PI*180.0);
	}

	painter->restore();
}

void SimulatorView::paintTargetCrossPos(QPainter *painter, const QPointF startPoint)
{
	painter->save();

	painter->scale(mPixelPerMeter,mPixelPerMeter);
	painter->translate(startPoint);

	QPen linePen;
	linePen.setStyle(Qt::SolidLine);
	linePen.setColor(QColor(200,20,20,180));
	linePen.setWidthF(0.1);
	linePen.setCapStyle(Qt::FlatCap);
	painter->setPen(linePen);

	double targerCrossPos = mSimulator->carDriver()->targetCrossPos();
	painter->drawLine(
				QPointF( targerCrossPos, 0.0 ),
				QPointF( targerCrossPos, -3.0 ));

	painter->restore();
}

void SimulatorView::paintSpeedometer(QPainter *painter, const QPointF centerPoint, const QSizeF size)
{
	painter->save();

	QFont font = painter->font();
	font.setWeight( QFont::Bold );
	painter->setFont(font);
	QPen textPen;
	textPen.setColor( QColor(255,255,255,250) );
	painter->setPen(textPen);

	float speedTextHeightRatio = 0.6;
	QPointF topLeft( centerPoint.x()-size.width()/2, centerPoint.y()-size.height()/2 );

	// MAIN SPEED
	QRectF mainSpeedBoundingRect( topLeft, QSize(size.width(),size.height()*speedTextHeightRatio) );
	QString mainSpeed( QString::number(static_cast<int>(mSimulator->car()->speedKmh()+0.5)) );
	font.setPixelSize( mainSpeedBoundingRect.height()*0.7 );
	painter->setFont(font);
	painter->drawText(mainSpeedBoundingRect, Qt::AlignHCenter|Qt::AlignBottom, mainSpeed);
	//painter->drawRect(mainSpeedBoundingRect);

	// SPEED UNIT
	double subSpeedsWidthRatio = 0.3;
	double subSpeedHeight = size.height()*(1-speedTextHeightRatio);
	font.setPixelSize( subSpeedHeight*0.3 );
	font.setWeight( QFont::Normal );
	painter->setFont(font);
	textPen.setColor( QColor(220,220,220,255) );
	painter->setPen(textPen);
	QRectF mainSpeedUnitBoundingRect(
				QPointF(
					mainSpeedBoundingRect.left()+size.width()*subSpeedsWidthRatio,
					mainSpeedBoundingRect.bottom() ),
				QSize( size.width()*(1-2*subSpeedsWidthRatio), subSpeedHeight ) );
	painter->drawText(mainSpeedUnitBoundingRect, Qt::AlignHCenter|Qt::AlignTop, "km/h");
	//painter->drawRect(mainSpeedUnitBoundingRect);

	// CRUISE SPEED
	QString cruiseSpeed( QString::number(static_cast<int>(mSimulator->carDriver()->cruiseSpeedKmh()+0.5)) );
	QRectF cruiseSpeedBoundingRect(
				QPointF(
					mainSpeedBoundingRect.left(),
					mainSpeedBoundingRect.bottom() ),
				QSize( size.width()*subSpeedsWidthRatio, subSpeedHeight ) );
	font.setPixelSize( subSpeedHeight*0.5 );
	font.setWeight( QFont::Bold );
	painter->setFont(font);
	textPen.setColor( QColor(150,230,140,240) );
	painter->setPen(textPen);
	painter->drawText(cruiseSpeedBoundingRect, Qt::AlignHCenter|Qt::AlignTop, cruiseSpeed);
	//painter->drawRect(cruiseSpeedBoundingRect);

	// TARGET SPEED
	QString targetSpeed( QString::number(static_cast<int>(mSimulator->carDriver()->targetSpeedKmh()+0.5)) );
	QRectF targetSpeedBoundingRect(
				QPointF(
					size.width()*(1.0-subSpeedsWidthRatio),
					mainSpeedBoundingRect.bottom() ),
				cruiseSpeedBoundingRect.size() );
	textPen.setColor( QColor(200,60,60,240) );
	painter->setPen(textPen);
	painter->drawText(targetSpeedBoundingRect, Qt::AlignHCenter|Qt::AlignTop, targetSpeed);
	//painter->drawRect(targetSpeedBoundingRect);

	painter->restore();
}

void SimulatorView::paintGmeter(QPainter *painter, const QPointF centerPoint, const QSizeF size)
{
	painter->save();

	QBrush accPointBrush(QColor(200,60,60,250), Qt::SolidPattern);

	QPen gridPen;
	gridPen.setWidthF(1.0);
	gridPen.setColor( QColor(50,50,50,200) );
	painter->setPen(gridPen);
	painter->setBrush(Qt::NoBrush);

	painter->drawLine(
				centerPoint.x()-size.width()/2, centerPoint.y(),
				centerPoint.x()+size.width()/2, centerPoint.y() );
	painter->drawLine(
				centerPoint.x(), centerPoint.y()-size.width()/2,
				centerPoint.x(), centerPoint.y()+size.width()/2 );

	// outermost circle (maxAcceleration)
	gridPen.setWidthF(2.0);
	painter->setPen(gridPen);
	painter->drawEllipse( centerPoint, size.width()/2, size.width()/2 );

	// acceleration point
	painter->setPen(Qt::NoPen);
	painter->setBrush(accPointBrush);

	double accPointSizeRatio = 0.09; // portion of meter size
	QVector2D accelVect( mSimulator->carDriver()->currentNetAccel() );
	double maxAcc = mSimulator->car()->maxAcceleration();
	accelVect = accelVect/maxAcc * size.width()/2;	// scale to meter size
	QPointF accPointCenter = QPointF(
				centerPoint.x()+accelVect.x(),
				centerPoint.y()-accelVect.y() );
	painter->drawEllipse( accPointCenter, size.width()*accPointSizeRatio/2, size.width()*accPointSizeRatio/2 );

	painter->restore();
}

void SimulatorView::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);

	Simulator::carPositionOnRoad_t carPosOnRoad = mSimulator->carPositionOnRoad();
	double travel = carPosOnRoad.travel;	// this is the advancement of the car on the road (relative to 0 milestone)
	const double visibilityM = mSimulator->roadVisibility();

	// init painting
	QPainter painter;
	painter.begin(this);
	painter.setRenderHint(QPainter::Antialiasing);

	QSize viewPortSize(size());

	// background
	painter.fillRect(QRect(0,0,viewPortSize.width(),viewPortSize.height()), QBrush(mBackgroundColor));

	QPointF drawOrigin(viewPortSize.width()/2,viewPortSize.height()-2.0);
	painter.save();
	painter.translate(drawOrigin);
	drawOrigin = QPointF( 0.0, -2.0 );

	QPointF roadEndPoint;
	paintRoad( &painter, drawOrigin, travel, visibilityM, roadEndPoint );
	roadEndPoint *= mPixelPerMeter;
	if( roadEndPoint != drawOrigin && !mPxPerMeterSetForViewSize )
	{
		double y = qAbs((roadEndPoint-drawOrigin).y())/mPixelPerMeter;
		if( y > mSimulator->roadVisibility()/2 )//be sure that the full road is painted already
		{
			double h = viewPortSize.height();
			mPixelPerMeter = h/y*0.9;
			mPxPerMeterSetForViewSize = true;
		}
	}
	paintTargetCrossPos(&painter, drawOrigin);
	paintCar( &painter, QPointF(drawOrigin.x()+carPosOnRoad.cross*mPixelPerMeter,drawOrigin.y()*mPixelPerMeter), carPosOnRoad.heading, QSize(1.6*mPixelPerMeter,4.0*mPixelPerMeter) );

	painter.restore();

	QSizeF speedometerSize( 100.0, 100.0 );
	paintSpeedometer( &painter, QPointF(speedometerSize.width()/2,viewPortSize.height()-speedometerSize.height()*0.35), speedometerSize );
	QSizeF gMeterSize( 90.0, 90.0 );
	paintGmeter( &painter, QPointF(viewPortSize.width()-gMeterSize.width()/1.7,viewPortSize.height()-gMeterSize.height()/1.7), gMeterSize );

	painter.end();
}

void SimulatorView::paintCar(QPainter *painter, const QPointF carCenterPoint, const double carHeading, const QSize carSize)
{
	painter->save();

	QPixmap carImg(":/images/car.png");
	if( mEaster )
		{ carImg.load(":/images/bcar.png"); }
	QPointF carOrigin;
	carOrigin.setX( carCenterPoint.x()-carSize.width()/2 );
	carOrigin.setY( carCenterPoint.y()-carSize.height()/2 );
	carOrigin.setX( (static_cast<int>(carOrigin.x()*1000+0.5))/1000.0 );	// round to avoid pixmap position flicker
	carOrigin.setY( (static_cast<int>(carOrigin.y()*1000+0.5))/1000.0 );	// round to avoid pixmap position flicker
	if( !carImg.isNull() )
	{
		carImg = carImg.scaledToWidth(carSize.width(),Qt::SmoothTransformation);
		QTransform rotateTransform;
		rotateTransform.rotateRadians( static_cast<int>(carHeading*10000+0.5)/10000.0 );	// round to avoid pixmap position flicker
		carImg = carImg.transformed( rotateTransform, Qt::SmoothTransformation );
		painter->drawPixmap(carOrigin, carImg);
	}
	else
	{
		qWarning("Car pixmap could not be loaded! Falling back to rectangle.");
		QPen pen(Qt::black);
		pen.setWidth(2);
		painter->setPen(pen);
		QBrush brush(QColor(0,127,0));
		painter->setBrush(brush);
		painter->drawRoundedRect(QRectF(carOrigin,carSize), 10, 10);
	}

	painter->restore();
}

void SimulatorView::keyPressEvent(QKeyEvent *event)
{
	if( mSimulator->carDriver()->keyboardEvent((Qt::Key)event->key(),true) )
		{ event->accept(); }
	if( event->key() == Qt::Key_B )
		{ mEaster = true; }
}

void SimulatorView::keyReleaseEvent(QKeyEvent *event)
{
	if( event->key() == Qt::Key_Left || event->key() == Qt::Key_Right )
		{ mSimulator->carDriver()->setSteeringControlFeedForward(0.0); }
	if( mSimulator->carDriver()->keyboardEvent((Qt::Key)event->key(),false) )
		{ event->accept(); }
	if( event->key() == Qt::Key_B )
		{ mEaster = false; }
}

void SimulatorView::resizeEvent(QResizeEvent *event)
{
	Q_UNUSED(event);
	mPxPerMeterSetForViewSize = false;
}
