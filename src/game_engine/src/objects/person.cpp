#include "../../include/objects/person.hpp"

using namespace std;

person::person(cv::Point2f pos, bool isWorker) {
  setPos(pos.x, pos.y);
  person_width      = 30;
  state             = 0;
  stop              = false;
  animation_counter = 0;
  worker            = isWorker;

  float x_pos  = projectedImageSizeX;
  signal       = (pos.x < x_pos/2) ? 1 : -1;

  QTimer *t = new QTimer();
  person::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}

QRectF person::boundingRect() const {
  return QRectF(-person_width/2, -person_width/2, person_width * 1.3, person_width);
}

QPainterPath person::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void person::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QPen pen(Qt::black, 2);
  painter->setPen(pen);

  QRectF r_a = QRectF(person_width/8, -person_width/3.5, person_width/4, person_width/3.5);
  painter->drawEllipse(r_a);

  QLine trunk(QPoint(person_width/4, 0), QPoint(person_width/4, person_width/2));
  painter->drawLine(trunk);

  QLine arm_a(QPoint(person_width/4, person_width/3), QPoint(person_width/2, person_width/2));
  painter->drawLine(arm_a);

  QLine arm_b(QPoint(person_width/4, person_width/3), QPoint(0, person_width/2));
  painter->drawLine(arm_b);


  animation_counter++;

  if(animation_counter <= 15) {
    QLine leg_a(QPoint(person_width/4, person_width/2), QPoint(0, person_width * 1.3));
    painter->drawLine(leg_a);

    QLine leg_b(QPoint(person_width/4, person_width/2), QPoint(person_width/2, person_width * 1.3));
    painter->drawLine(leg_b);
  }
  else {
    QLine leg_a(QPoint(person_width/4, person_width/2), QPoint(person_width/4, person_width * 1.3));
    painter->drawLine(leg_a);

    if(animation_counter == 30)
      animation_counter = 0;
  }
}

void person::delete_item() {
  this->deleteLater();
}

void person::advance() {

  if(worker == true) {
    switch(state) {
      case 0:
        setPos(mapToParent(signal * person_speed, 0));
        if(stop == true)
          state = 1;
      break;

      case 1:
        if(stop == false)
          state = 2;
      break;

      case 2:
        setPos(mapToParent(-signal * person_speed, 0));
      break;
    }
  }
  else {
    switch(state) {
      case 0:
        setPos(mapToParent(0, person_speed));
        if(this->y() >= scene()->height() - wareh_wallB - 87.5)
          state = 1;
      break;

      case 1:
        setPos(mapToParent(signal * person_speed, 0));
      break;
    }
  }

  bool collision = false;

  QList<QGraphicsItem *> list = this->scene()->collidingItems(this, Qt::IntersectsItemBoundingRect);
  Q_FOREACH(QGraphicsItem *i, list) {
    collision = true;
    this->setVisible(false);

    tag *t = dynamic_cast<tag *>(i);
    if(t != NULL)
      t->immobilize = true;
  }

  if(collision == false)
    this->setVisible(true);
}
