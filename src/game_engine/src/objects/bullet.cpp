#include "../../include/objects/bullet.hpp"

using namespace std;

bullet::bullet(double initial_x, double initial_y, double orientation) {

  setPos(initial_x, initial_y);
  setRotation(orientation);

  QTimer *t = new QTimer();
  bullet::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}


QRectF bullet::boundingRect() const {
  return QRectF(-bullet_lenght/2 - 1/2, -bullet_width/2 - 1/2, bullet_lenght + 1/2, bullet_width + 1/2);
}

QPainterPath bullet::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void bullet::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QRectF rect = boundingRect();
  painter->setBrush(Qt::red);
  painter->drawEllipse(rect);
}

bool bullet::collision() {
  QPointF pos = mapToScene(0,0);
  explosion *e = new explosion(pos.x(), pos.y());
  this->scene()->addItem(e);
  this->deleteLater();
}

void bullet::delete_item() {
  this->deleteLater();
}

void bullet::advance() {
  setPos(mapToParent((bullet_speed), 0));

  QList<QGraphicsItem *> list = this->collidingItems(Qt::IntersectsItemBoundingRect);
  Q_FOREACH(QGraphicsItem *i, list) {
    wall *wall_item = dynamic_cast<wall *>(i);
    if(wall_item != NULL) {
      if(wall_item->border == false){
        wall_item->life--;
        Q_EMIT wallHit(wall_item->id);
      }
      if(wall_item->life == 0)
        wall_item->deleteLater();
      collision();
    }
  }
}
