#ifndef RACEMAP_HPP
#define RACEMAP_HPP

#include "map.hpp"

class racemap : public QGraphicsObject{
  Q_OBJECT

public:
  racemap(cv::Point2f pos, bool isOutside);

  /*! - delete the Qt item from the scene, safely, without going out of scope */
  void delete_item();

  int map_width, map_height;
  bool outside, raceStarted;
  std::vector<cv::Point2f> goal_pos;

private:
  /*! - defines the bounding rect of the race map */
  QRectF boundingRect() const;
  QPainterPath shape() const;
  /*! - draws the race map, using the bounding rect */
  void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget);
};

#endif // RACEMAP_HPP
