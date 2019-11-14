#include "../../include/objects/game_clock.hpp"

using namespace std;

game_timer::game_timer(cv::Point2f pos, QTime game_time) {
  setPos(pos.x, pos.y);
  game_timer_width = 50;
  time = game_time;

  QTimer *t = new QTimer();
  game_timer::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(1000);
}

QRectF game_timer::boundingRect() const {
  return QRectF(-game_timer_width/2, -game_timer_width/2, game_timer_width, game_timer_width);
}

QPainterPath game_timer::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void game_timer::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QPen pen(Qt::black, 5);
  painter->setPen(pen);
  QRectF rect = boundingRect();
  painter->drawRect(rect);

  painter->rotate(180);
  painter->drawText(rect, Qt::AlignCenter, time.toString("mm:ss"));
  painter->rotate(-180);
}

void game_timer::delete_item() {
  this->deleteLater();
}

void game_timer::advance() {
  if(gameRunning == true && gamePaused == false)
    time = time.addSecs(-1);
}

QTime game_timer::getTime(){
    return time;
}
