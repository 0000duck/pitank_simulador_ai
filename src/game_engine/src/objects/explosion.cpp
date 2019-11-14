#include "../../include/objects/explosion.hpp"

using namespace std;

explosion::explosion(double initial_x, double initial_y) {
  setPos(initial_x, initial_y);

  state = 0;
  width = explosion_width;

  QTimer *t = new QTimer();
  explosion::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}

QRectF explosion::boundingRect() const {
  return QRectF(-width/2, -width/2, width, width);
}

QPainterPath explosion::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void explosion::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QRectF rect = boundingRect();
  painter->setBrush(Qt::red);
  painter->drawEllipse(rect);
}

void explosion::advance() {
  switch(state) {
    case 0:
      width += width/3;
      state = 1;
    break;

    case 1:
      width += width/3;
      state = 2;
    break;

    case 2:
      width += width/3;
      state = 3;
    break;

    case 3:
      width += width/3;
      state = 4;
    break;

    case 4:
      this->deleteLater();
    break;
  }
}
