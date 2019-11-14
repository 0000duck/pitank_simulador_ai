#ifndef PERSON_HPP
#define PERSON_HPP

#include "map.hpp"
#include "tag.hpp"

/*!
 * \brief Class that represents a person
 *  - It is essentially used to add persons to robot @ factory game map, representing the workers of the factory
 */

class person : public QGraphicsObject {
  Q_OBJECT

public:
  person(cv::Point2f pos, bool isWorker);

  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

  bool stop;

public Q_SLOTS:
  void advance();

private:
  /*! - defines the bounding rect of each person */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - draws the person, using the bounding rect */
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);

  int person_width, state, signal, animation_counter;
  bool worker;
};

#endif // PERSON_HPP
