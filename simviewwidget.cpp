#include "simviewwidget.h"
#include <QPaintEvent>
#include <QPainter>

SimViewWidget::SimViewWidget(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(QSize(400,400));
    update();
}

void SimViewWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // background
    painter.fillRect(event->rect(), QBrush(Qt::white));

    QPen pen(Qt::black);
    pen.setWidth(2);
    painter.setPen(pen);
    QBrush brush(QColor(200,100,130,180));
    painter.setBrush(brush);
    painter.drawEllipse( QPoint(200,200), 50, 80 );

    painter.end();
}
