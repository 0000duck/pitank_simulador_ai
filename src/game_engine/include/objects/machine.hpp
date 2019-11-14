#ifndef MACHINE_HPP
#define MACHINE_HPP

#include "map.hpp"
#include "pallet.hpp"
#include "person.hpp"

/*!
 * \brief This class represents the machine object of the robot factory game
 * Each machine has three possible states:
 *  - state = 0 if the machine is free
 *  - state = 1 if the machine is processing a pallet
 *  - state = 2 if the machine is broken
 *
 * Each machine has a type:
 *  - type = 0 if the machine processes pallets of the type A transforming it to B type
 *  - type = 1 if the machine processes pallets of the type B transforming it to C type and pallets of the C transforming it to A type
 */

class machine : public QGraphicsObject {
  Q_OBJECT

public:
  machine(cv::Point2f pos, int machineType);

  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

public Q_SLOTS:
  void advance();

private:
  /*! - defines the bounding rect of each machine */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - draws the machine, using the bounding rect */
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);

  int type, global_state, state, machine_width, processing_time, number_pallets;
  bool broken;
  QColor state_color, type_color;
  cv::Point2f position;
  person* worker;

};

#endif // MACHINE_HPP
