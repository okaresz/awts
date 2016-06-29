#ifndef SIMVIEWWIDGET_H
#define SIMVIEWWIDGET_H

#include <QWidget>

class SimViewWidget : public QWidget
{
    Q_OBJECT
public:
    explicit SimViewWidget(QWidget *parent = 0);

signals:

public slots:

protected:
    void paintEvent(QPaintEvent *event);
};

#endif // SIMVIEWWIDGET_H
