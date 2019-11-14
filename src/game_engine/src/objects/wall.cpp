#include "../../include/objects/wall.hpp"

using namespace std;

wall::wall(QPointF p1, QPointF p2, bool isBorder, bool isPodium, bool isWarehouse) {
  life = wall_life;
  border = isBorder;
  podium = isPodium;
  warehouse = isWarehouse;
  wallLine.setP1(p1);
  wallLine.setP2(p2);

  QTimer *t = new QTimer();
  wall::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}

wall::wall(QPointF p1, QPointF p2, bool isBorder, bool isPodium, bool isWarehouse, int id) {
  this->id = id;
  life = wall_life;
  border = isBorder;
  podium = isPodium;
  warehouse = isWarehouse;
  wallLine.setP1(p1);
  wallLine.setP2(p2);

  QTimer *t = new QTimer();
  wall::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}

double wall::wallRobotAngle(double tagAngle) {
  double angle = (wallLine.p1().y() == wallLine.p2().y()) ? (tagAngle - 180) : (tagAngle - 90);
  return (angle > 0) ? angle : (angle + 360);
}

QRectF wall::boundingRect() const {
  return QRectF(wallLine.p1(), wallLine.p2());
}

QPainterPath wall::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}


void wall::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QColor color;

  if(podium == false && warehouse == false) {
    switch (life) {
      case 5:
        color = QColor(0, 0, 0);
      break;

      case 4:
        color = QColor(15, 15, 15);
      break;

      case 3:
        color = QColor(30, 30, 30);
      break;

      case 2:
        color = QColor(45, 45, 45);
      break;

      case 1:
        color = QColor(60, 60, 60);
      break;
    }
  }
  else if(podium == false && warehouse == true)
    color = team[team_id].team_color;
  else if(podium == true && warehouse == false)
    color = QColor(218,165,32);

  QPen pen(color, wallWidth);
  painter->setPen(pen);
  painter->drawLine(wallLine);
}

void wall::delete_item() {
  this->deleteLater();
}

void wall::advance() {}
