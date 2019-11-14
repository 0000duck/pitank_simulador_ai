#ifndef GAME_game_timerHPP
#define GAME_game_timerHPP

#include "map.hpp"

class game_timer : public QGraphicsObject {
  Q_OBJECT

public:
  game_timer(cv::Point2f pos, QTime game_time);
  QTime getTime();

public Q_SLOTS:
  void advance();
  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

private:
  /*! - defines the bounding rect of the clock */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - draws the clock, using the bounding rect */
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);

  QTime time;
  int game_timer_width;

};

#endif // GAME_game_timerHPP
