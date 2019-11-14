#ifndef EXPLOSION_HPP
#define EXPLOSION_HPP

#include "map.hpp"

class explosion : public QGraphicsObject {
  Q_OBJECT

public:
  explosion(double initial_x, double initial_y);

  /*! - defines the bounding rect of each explosion */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - uses the bounding rect to draw the explosion */
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

public Q_SLOTS:
  /*! - sets the state of the explosion */
  void advance();

private:
  int state;
  double width;
};

#endif // EXPLOSION_HPP
