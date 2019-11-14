#include "../../include/objects/racemap.hpp"

racemap::racemap(cv::Point2f pos, bool isOutside) {
  setPos(pos.x, pos.y);

  map_width   = 850;
  map_height  = 650;
  outside     = isOutside;
  raceStarted = false;

  goal_pos.push_back(cv::Point2f(goal_x + wallWidth + 25, goal_y_min + 75 + wallWidth + 50));
  goal_pos.push_back(cv::Point2f(goal_x + wallWidth + 25, goal_y_max - 75 + wallWidth + 50));
}

QRectF racemap::boundingRect() const {
  return QRectF(0, 0, map_width, map_height);
}

QPainterPath racemap::shape() const {
  QPainterPath path;

  if(outside == true) {
    path.moveTo(225, 0);

    path.lineTo(700, 0);
    path.quadTo(900, 125, 725, 225);
    path.quadTo(705, 230, 700, 250);

    path.lineTo(700, 575);
    path.quadTo(700, 650, 625, 650);

    path.lineTo(150, 650);
    path.quadTo(-100, 525, 125, 325);
    path.quadTo(145, 315, 150, 300);

    path.lineTo(150, 75);
    path.quadTo(150, 0, 225, 0);
  }
  else {
    path.moveTo(375, 150);

    path.lineTo(550, 150);
    path.quadTo(650, 175, 550, 230);

    path.lineTo(550, 425);
    path.quadTo(550, 500, 475, 500);

    path.lineTo(300, 500);
    path.quadTo(100, 475, 300, 400);

    path.lineTo(300, 225);
    path.quadTo(300, 150, 375, 150);
  }

  return path;
}

void racemap::delete_item() {
  this->deleteLater();
}

void racemap::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QColor c("orange");
   QPen pen(c, 6);
   painter->setPen(pen);
   painter->drawPath(shape());

   QLineF goal_line_A(QPointF(goal_x, goal_y_min), QPointF(goal_x, goal_y_min + 150));
   QLineF goal_line_B(QPointF(goal_x, goal_y_max - 150), QPointF(goal_x, goal_y_max));
   QLineF goal_line_C(QPointF(goal_x_min - 10, goal_y), QPointF(goal_x_min + 150, goal_y));
   QLineF goal_line_D(QPointF(goal_x_max - 150, goal_y), QPointF(goal_x_max, goal_y));

   QPen goal_pen;
   if(raceStarted == false) {
     goal_pen.setColor(Qt::white);
     goal_pen.setWidth(4);
   }
   else {
     goal_pen.setColor(Qt::black);
     goal_pen.setWidth(4);
   }
   painter->setPen(goal_pen);
   painter->drawLine(goal_line_A);
   painter->drawLine(goal_line_B);
   painter->drawLine(goal_line_C);
   painter->drawLine(goal_line_D);
}
