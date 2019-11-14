#ifndef PALLET_HPP
#define PALLET_HPP

#include "map.hpp"
#include "tag.hpp"

/*!
 * \brief This class represents the pallet object of the robot factory game
 * - Each pallet has a type (one, two or three)
 * - Possible transformations: 1 -> 2; 2 -> 3; 3 -> 1
 */


class pallet : public QGraphicsObject {
  Q_OBJECT

public:
  pallet(cv::Point2f pos, int palletType);

  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

  bool ready, catched, processed, broken;
  double angle;
  int robot_id, initial_type, type, transf_counter, state;

public Q_SLOTS:
  void advance();

private:
  /*! - defines the bounding rect of each pallet */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - draws the pallet, using the bounding rect */
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);

  cv::Point2f position;
  int pallet_width;
  QColor color;
};

#endif // PALLET_HPP
