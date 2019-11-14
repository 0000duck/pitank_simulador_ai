#ifndef WALL_HPP
#define WALL_HPP

#include "map.hpp"
#include "tag.hpp"

class wall : public QGraphicsObject {
  Q_OBJECT

public:
  /*! - class constructor */
  wall(QPointF p1, QPointF p2, bool isBorder, bool isPodium, bool isWarehouse);
  wall(QPointF p1, QPointF p2, bool isBorder, bool isPodium, bool isWarehouse, int id);

  /*! - calculates the angle between the robot and the wall in case of colision */
  double wallRobotAngle(double tagAngle);
  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

  QLineF wallLine;
  int life, team_id, id;
  bool border, podium, warehouse;

public Q_SLOTS:
  void advance();

private:
  /*! - defines the bounding rect of each wall */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - draws contours of each wall on the scene, using the bounding rect */
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);

};

#endif // WALL_HPP
