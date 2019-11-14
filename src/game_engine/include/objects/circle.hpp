#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "map.hpp"
#include "tag.hpp"

/*!
 * - This class represents the item circle
 * - Its used in the game calibration to show on the game map the corners resulting of the calibration
 */

class circle : public QGraphicsObject {
 Q_OBJECT

public:
  circle(int id, QColor color, bool isPodium, int whichGame);

  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

public Q_SLOTS:
  /*! - sets the position of the circle */
  void advance();


private:
  /*! - defines the bounding rect of each circle */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - uses the bounding rect to draw the circle */
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

  QColor circle_color;
  int circle_width, j, game;
  bool podium;
};

#endif // CIRCLE_HPP
