#ifndef GRID_HPP
#define GRID_HPP

//#include <QPointF>
//#include <QLineF>
//#include <QPainter>
//#include <QPen>
//#include <QColor>
//#include <QGraphicsObject>
#include "machine.hpp"

#define MODE 0

/*!
 * - This class represents the item circle
 * - Its used in the game calibration to show on the game map the corners resulting of the calibration
 */

class grid : public QGraphicsObject {
 Q_OBJECT

public:
   grid(QPointF topleft, QPointF topright, QPointF bottomleft, QPointF bottomright, int space);

private:
  /*! - defines the bounding rect of each circle */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - uses the bounding rect to draw the circle */
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

  QPointF topleft;
  QPointF bottomleft;
  QPointF topright;
  QPointF bottomright;
  int space;
  QPointF point;
  QLineF line;
  bool mode;
};

#endif // GRID_HPP
