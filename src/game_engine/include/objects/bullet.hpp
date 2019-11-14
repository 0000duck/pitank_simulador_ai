#ifndef BULLET_HPP
#define BULLET_HPP

#include "map.hpp"
#include "wall.hpp"
#include "explosion.hpp"

class bullet : public QGraphicsObject {
  Q_OBJECT

public:
  bullet(double initial_x, double initial_y, double orientation);

  /*! - defines the bounding rect of each bullet */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - uses the bounding rect to draw the bullet, a red ellipse */
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  /*! - handles the collisions of the bullet with a robot or a wall*/
  bool collision();
  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

  int robot_sender;

public Q_SLOTS:
  /*! - sets the position of the bullet */
  void advance();

Q_SIGNALS:

  void wallHit(int);

private:
  double angle, x, y;

};

#endif // BULLET_HPP
