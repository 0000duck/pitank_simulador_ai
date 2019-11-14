#include "../../include/objects/pallet.hpp"

using namespace std;

pallet::pallet(cv::Point2f pos, int palletType) {
  initial_type  = palletType;
  type          = initial_type;
  transf_counter = 0;
  position      = pos;
  pallet_width  = 35;
  ready         = false;
  catched       = false;
  processed     = false;
  broken        = false;
  robot_id      = -1;
  state         = 0;
  angle         = (pos.y < 50) ? 90 : 270;

  setPos(pos.x, pos.y);
  setRotation(angle);

  QTimer *t = new QTimer();
  pallet::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}

QRectF pallet::boundingRect() const {
  return QRectF(-pallet_width/2, -pallet_width/2, pallet_width * 1.3, pallet_width);
}

QPainterPath pallet::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void pallet::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {

  switch(type) {
    case 0:
      color = Qt::darkYellow;
    break;

    case 1:
      color = Qt::darkGreen;
    break;

    case 2:
      color = Qt::darkRed;
    break;
  }

  QRectF rect = boundingRect();
  QPen pen(color, 2);
  painter->setPen(pen);
  painter->drawRect(rect);

  QLineF catch_line(QPointF(pallet_width - pallet_width/5, -pallet_width/2), QPointF(pallet_width - pallet_width/5, pallet_width/2));
  QPen line_color(Qt::blue, 3);
  painter->setPen(line_color);
  painter->drawLine(catch_line);

  if(broken == true)
    state = 3;
  switch(state) {
    case 0:
      if(transf_counter >= 1)
        state = 1;
      else {
        QLine lineA(QPoint(-pallet_width/2, -pallet_width/3.5), QPoint(pallet_width - pallet_width/5, -pallet_width/3.5));
        painter->setPen(pen);
        painter->drawLine(lineA);

        QLine lineB(QPoint(-pallet_width/2, 0), QPoint(pallet_width - pallet_width/5, 0));
        painter->setPen(pen);
        painter->drawLine(lineB);

        QLine lineC(QPoint(-pallet_width/2, pallet_width/3.5), QPoint(pallet_width - pallet_width/5, pallet_width/3.5));
        painter->setPen(pen);
        painter->drawLine(lineC);
      }
    break;

    case 1:
      if(transf_counter >= 2)
        state = 2;
      else {
        QLine lineD(QPoint(-pallet_width/2, -pallet_width/3.5), QPoint(pallet_width - pallet_width/5, -pallet_width/3.5));
        painter->setPen(pen);
        painter->drawLine(lineD);

        QLine lineE(QPoint(-pallet_width/2, 0), QPoint(pallet_width - pallet_width/5, 0));
        painter->setPen(pen);
        painter->drawLine(lineE);

        QLine lineF(QPoint(-pallet_width/2, pallet_width/3.5), QPoint(pallet_width - pallet_width/5, pallet_width/3.5));
        painter->setPen(pen);
        painter->drawLine(lineF);

        QLine lineG(QPoint(-pallet_width/5, -pallet_width/2), QPoint(-pallet_width/5, pallet_width/2));
        painter->setPen(pen);
        painter->drawLine(lineG);

        QLine lineH(QPoint(pallet_width * 0.15, -pallet_width/2), QPoint(pallet_width * 0.15, pallet_width/2));
        painter->setPen(pen);
        painter->drawLine(lineH);

        QLine lineI(QPoint(pallet_width - pallet_width/2, -pallet_width/2), QPoint(pallet_width - pallet_width/2, pallet_width/2));
        painter->setPen(pen);
        painter->drawLine(lineI);
      }
    break;

    case 2:
      if(transf_counter >= 2) {
        QBrush b(color);
        painter->setBrush(b);
        painter->drawRect(rect);
      }
    break;

    case 3:
      transf_counter = 0;
      color = QColor(0, 0, 0);
      QBrush b(color);
      painter->setBrush(b);
      painter->drawRect(rect);
    break;
  }

  if(ready == true) {
    QRect r(-pallet_width/1.5, -pallet_width/1.5, pallet_width * 1.7, pallet_width * 1.4);
    QPen  p(Qt::darkGreen, 4);
    painter->setPen(p);
    painter->drawRect(r);
  }
}

void pallet::delete_item() {
  this->deleteLater();
}

void pallet::advance() {
  int factor = 1.8;
  if(catched == true && robot_id != -1) {
    setPos(robot[robot_id].centroid.x + pi_radius * cos(robot[robot_id].angle * PI / 180) * factor, robot[robot_id].centroid.y + pi_radius * sin(robot[robot_id].angle * PI / 180) * factor);
    angle = robot[robot_id].angle - 180;
    angle = (angle < 0) ? angle + 360 : angle;
    setRotation(angle);
  }

  bool robotHere = false;
  QList<QGraphicsItem *> list = scene()->collidingItems(this, Qt::IntersectsItemBoundingRect);
  Q_FOREACH(QGraphicsItem *i, list) {
    tag *tag_item = dynamic_cast<tag *>(i);
    if(tag_item != NULL)
      robotHere = true;
  }

  if(robotHere == false)
    ready = false;
}
