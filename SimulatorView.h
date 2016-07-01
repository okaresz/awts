#ifndef SIMVIEWWIDGET_H
#define SIMVIEWWIDGET_H

#include <QWidget>
#include "Simulator.h"

class SimulatorView : public QWidget
{
    Q_OBJECT
public:
	explicit SimulatorView( Simulator *simulator, QWidget *parent = 0);

signals:

public slots:

protected:
    void paintEvent(QPaintEvent *event);

private:
	void paintCar(QPainter *painter, const QPointF carCenterPoint, const QSize carSize );

	Simulator *mSimulator;
};

#endif // SIMVIEWWIDGET_H
