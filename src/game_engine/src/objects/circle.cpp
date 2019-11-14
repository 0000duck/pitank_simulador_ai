#include "../../include/objects/circle.hpp"

using namespace std;

circle::circle(int id, QColor color, bool isPodium, int whichGame) {
  setPos(team[id].class_pos.x, team[id].class_pos.y);
  circle_width = 40;
  circle_color = color;
  j = id;
  game = whichGame;
  podium = isPodium;

  QTimer *t = new QTimer();
  bullet::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}

QRectF circle::boundingRect() const {
  return QRectF(-circle_width/2, -circle_width/2, circle_width, circle_width);
}

QPainterPath circle::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void circle::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QPointF pos = mapToScene(0,0);
  QString number;

  if(pos.y() == classNr_yA || pos.y() == podium_first_pos_y + 70)
     number = "1";
  else if(pos.y() == classNr_yB || pos.y() == podium_second_pos_y + 63)
     number = "2";
  else if(pos.y() == classNr_yC || pos.y() == podium_third_pos_y + 55)
     number = "3";
  else if(pos.y() == classNr_yD)
     number = "4";
  else if(pos.y() == classNr_yE)
     number = "5";

  QPen pen(circle_color, 5);
  painter->setPen(pen);
  QRectF rect = boundingRect();
  if(podium == false)
    painter->drawEllipse(rect);

  painter->rotate(180);
  painter->drawText(rect, Qt::AlignCenter, number);
  painter->rotate(-180);

  if(podium == false) {
    QRectF r(-circle_width, -circle_width/2, circle_width * 2, 3 * circle_width);

    QString classification_info;
    if(game == 0)
      classification_info = "+  " + QString::number(team[j].kills) + "\n-  " + QString::number(team[j].damage);
    else if(game == 1)
      classification_info = QString::number(team[j].wareh_pallets);
    else if(game == 2)
      classification_info = QString::number(team[j].lapsDone);

    painter->rotate(180);
    painter->drawText(r, Qt::AlignCenter, classification_info);
    painter->rotate(-180);
  }
}

void circle::delete_item() {
  this->deleteLater();
}

void circle::advance() {
  if(podium == false)
    setPos(team[j].class_pos.x, team[j].class_pos.y);
  else {
    if(team[j].classification == 1)
      setPos(podium_first_pos_x, podium_first_pos_y + 70);
    else if(team[j].classification == 2)
      setPos(podium_second_pos_x, podium_second_pos_y + 63);
    else if(team[j].classification == 3)
      setPos(podium_third_pos_x, podium_third_pos_y + 55);
  }
}
