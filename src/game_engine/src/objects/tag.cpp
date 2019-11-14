#include "../../include/objects/tag.hpp"

using namespace std;

vector<robot_info> robot;
std::vector<team_info> team;
int id, driving_counter = 0, number_robots;
char trama[15];
vector<float> linearTransformation, offsetTranformation;
cv::Point3d cam_pos;
bool raceHasStarted;

tag::tag(apriltags_ros::AprilTagDetection msg, char addr, double robot_z, int team_id) {

  robot_info robotObj;

  robotObj.tagId               = msg.id;
  robotObj.teamId              = team_id;
  robotObj.height              = robot_z;
  robotObj.addr0               = 0x10;
  robotObj.addr1               = addr;
  robotObj.vel1                = 0;
  robotObj.vel2                = 0;
  robotObj.previous_vel        = 0;
  robotObj.collisionStateVar   = -1;
  robotObj.collisionFlag       = false;
  robotObj.threadIsRunning     = false;
  robotObj.kills               = 1;
  robotObj.damage              = 1;
  if(team_id >= 0)
    robotObj.robot_color       = team[team_id].team_color;
  else
    robotObj.robot_color       = Qt::black;
  robotObj.autonomous_drive    = false;
  robotObj.linear_direction    = -2;
  robotObj.driving_state       = 0;
  robotObj.driving_pos         = cv::Point2f(0,0);
  robotObj.catch_pallet        = false;
  robotObj.transporting_pallet = false;
  robotObj.immobilized         = false;
  robotObj.slow_motion         = false;
  robotObj.goal_crosses        = 0;
  robotObj.initial_angle       = 0;
  robotObj.lap_state           = 0;

  double angle = 360 - normDegrees(atan2(2 * (msg.pose.pose.orientation.x * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.w),
                       1 - 2*(msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)) * 180 / CV_PI);
  robotObj.angle = normDegrees(angle);

  robot.push_back(robotObj);
  number_robots++;

  color = robotObj.robot_color;
  immobilize = false;
  stop_counter = 0;
  raceHasStarted = false;

  QSignalMapper* signalMapper = new QSignalMapper(this);
  timer[id] = new QTimer();
  tag::connect(timer[id], SIGNAL(timeout()), signalMapper, SLOT(map()));
  signalMapper->setMapping(timer[id], id);
  connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(advance(int)));
  timer[id]->start(50);
}

tag::tag(geometry_msgs::Pose msg, double robot_z, int team_id){
    robot_info robotObj;

    robotObj.teamId              = team_id;
    robotObj.height              = robot_z;
    robotObj.vel1                = 0;
    robotObj.vel2                = 0;
    robotObj.previous_vel        = 0;
    robotObj.collisionStateVar   = -1;
    robotObj.collisionFlag       = false;
    robotObj.threadIsRunning     = false;
    robotObj.kills               = 1;
    robotObj.damage              = 1;
    if(team_id >= 0)
      robotObj.robot_color       = team[team_id].team_color;
    else
      robotObj.robot_color       = Qt::black;
    robotObj.autonomous_drive    = false;
    robotObj.linear_direction    = -2;
    robotObj.driving_state       = 0;
    robotObj.driving_pos         = cv::Point2f(0,0);
    robotObj.catch_pallet        = false;
    robotObj.transporting_pallet = false;
    robotObj.immobilized         = false;
    robotObj.slow_motion         = false;
    robotObj.goal_crosses        = 0;
    robotObj.initial_angle       = 0;
    robotObj.lap_state           = 0;

    double angle = 90 - atan2(2 * (msg.orientation.x * msg.orientation.y + msg.orientation.z * msg.orientation.w),
                         1 - 2*(msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)) * 180 / CV_PI;
    robotObj.angle = angle;

    robot.push_back(robotObj);
    number_robots++;

    color = robotObj.robot_color;
    immobilize = false;
    stop_counter = 0;
    raceHasStarted = false;

    QSignalMapper* signalMapper = new QSignalMapper(this);
    timer[id] = new QTimer();
    tag::connect(timer[id], SIGNAL(timeout()), signalMapper, SLOT(map()));
    signalMapper->setMapping(timer[id], id);
    connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(advance(int)));
    timer[id]->start(50);
}

double tag::normDegrees(double angle) {
  return (angle > 0) ? angle : (angle + 360);
}

double tag::normDegrees180(double angle){
    angle = fmod(angle + 180,360);
    if (angle < 0)
        angle += 360;
    return angle - 180;
}

cv::Point2f tag::parallax_correction(double x, double y, double z) {
  cv::Point2f new_pos;
  double      angle, d, factor, corr_x, corr_y;

  new_pos = cv::Point2f(x, y) - cv::Point2f(cam_pos.x, cam_pos.y);

  factor = z/cam_pos.z;
  d = sqrt(pow(new_pos.x, 2) + pow(new_pos.y, 2)) * factor;
  angle = atan(new_pos.y / new_pos.x);

  if(new_pos.x > 0)
    angle += PI;

  corr_x = d * cos(angle);
  corr_y = d * sin(angle);

  if(new_pos.x > 0 && new_pos.y < 0)
    corr_x *= 0.9;
  if(new_pos.x > 0 && new_pos.y > 0) {
    corr_x *= 0.8;
    corr_y *= 0.9;
  }

  new_pos.x = new_pos.x + (corr_x * 1.4);
  new_pos.y = new_pos.y + (corr_y * 1.4);

  new_pos.x += cam_pos.x;
  new_pos.y += cam_pos.y;

  return new_pos;
}

bool tag::collisionState(int id) {
  int nextstate = 0, state = nextstate;
  bool breakFlag = false;

  robot[id].threadIsRunning = true;

  while(breakFlag == false) {
    switch (state) {
      case 0:
        if(robot[id].collisionFlag == false)
          nextstate = 2;

        if(robot[id].linear_direction > 0)
          robot[id].collisionStateVar = 0;
        else {
          robot[id].collisionStateVar = 1;
          nextstate = 1;
        }

      break;

      case 1:
        if(robot[id].collisionFlag == false)
          nextstate = 2;

        if(robot[id].linear_direction > 0)
          nextstate = 0;

      break;

      case 2:
        breakFlag = true;
      break;
    }
    state = nextstate;
  }

  robot[id].collisionStateVar = (robot[id].linear_direction == -2) ? -1 : 2;
  robot[id].threadIsRunning = false;
}

double tag::arctan(double x, double y) {

  if(x >= 0 && y > 0) {
    return atan(abs(y/x)) * 180 / PI + 180;
  }
  else if(x < 0 && y > 0) {
    return (360 - atan(abs(y/x)) * 180 / PI);
  }
  else if(x < 0 && y <= 0) {
    return atan(abs(y/x)) * 180 / PI;
  }
  else if(x >= 0 && y <= 0) {
    return (180 - atan(abs(y/x)) * 180 / PI);
  }

}

double tag::potential_fields(int id, cv::Point2f target) {
  double reference = 0;
  cv::Point2f f_att = cv::Point2f(0,0), f_rep = cv::Point2f(0,0), f = cv::Point2f(0,0);
  int ro_zero = 150;
  float zeta = sqrt(pow(target.x, 2) + pow(target.y, 2)) / sqrt(pow(target.x - robot[id].centroid.x, 2) + pow(target.y - robot[id].centroid.y, 2));
  float factor = 1.5f, eta = factor * ro_zero * ro_zero;

  f_att = zeta * cv::Point2f(target.x - robot[id].centroid.x, target.y - robot[id].centroid.y);
  f = f_att;

  for(size_t i = 0; i < robot.size(); i++) {

    double centroid_dx, centroid_dy;
    if(robot[id].centroid.x < robot[i].centroid.x)
      centroid_dx = cos(robot[id].angle * PI * 180)  - cos(robot[i].angle * PI * 180);
    else
      centroid_dx = -cos(robot[id].angle * PI * 180) + cos(robot[i].angle * PI * 180);

    if(robot[id].centroid.y < robot[i].centroid.y)
      centroid_dy = sin(robot[id].angle * PI * 180)  - sin(robot[i].angle * PI * 180);
    else
      centroid_dy = -sin(robot[id].angle * PI * 180) + sin(robot[i].angle * PI * 180);


    double d_x = robot[id].centroid.x - robot[i].centroid.x + centroid_dx;
    double d_y = robot[id].centroid.y - robot[i].centroid.y + centroid_dy;

    if(i != id && sqrt(pow(d_x, 2) + pow(d_y, 2)) < ro_zero) {
      f_rep.x = eta / pow(sqrt(pow(d_x, 2) + pow(d_y, 2)), 2) * d_x;
      f_rep.y = eta / pow(sqrt(pow(d_x, 2) + pow(d_y, 2)), 2) * d_y;

      f += f_rep;
    }
  }

  reference = atan2(f.y, f.x) * 180 / PI;
  reference = (reference > 360) ? (reference - 360) : reference;
  reference = (reference < 0)   ? (reference + 360) : reference;

  return reference;
}

bool tag::send_to_pos(cv::Point2f pos, int id) {
  double reference, error;

  reference = potential_fields(id, pos);
  error = reference - robot[id].angle;

  switch(robot[id].driving_state) {
    case 0:
      if((error > 5 && error < 180) || (error < -180 && error > -355)) {
        robot[id].vel2 = -50;
        robot[id].vel1  = 0;
      }
      else if((error < -5 && error > -180) || (error > 180 && error < 355)) {
        robot[id].vel2 = 50;
        robot[id].vel1  = 0;
      }
      else if(error < 10 && error > -10)
        robot[id].driving_state = 1;
    break;

    case 1:
      robot[id].vel1 = 0;
      robot[id].vel2  = 0;
      robot[id].driving_state = 2;
    break;

    case 2:
      robot[id].vel2 = 0;
      robot[id].vel1  = 17;

      if(abs(robot[id].centroid.x - pos.x) < 10 && abs(robot[id].centroid.y - pos.y) < 10) {
        if(game == 0 || game == 1)
          robot[id].driving_state = 3;
        else
          robot[id].driving_state = 4;

      }
      if(abs(error) > 15 && abs(error) < 345) {
        robot[id].driving_state = 0;
      }
    break;

    case 3:
      robot[id].driving_state     = -1;
      driving_counter++;
      robot[id].vel1 = 0;
      robot[id].vel2 = 0;
    break;

    case 4:
      double race_error = robot[id].initial_angle - robot[id].angle;
      if((race_error > 5 && race_error < 180) || (race_error < -180 && race_error > -355)) {
        robot[id].vel2 = -50;
        robot[id].vel1  = 0;
      }
      else if((race_error < -5 && race_error > -180) || (race_error > 180 && race_error < 355)) {
        robot[id].vel2 = 50;
        robot[id].vel1  = 0;
      }
      else if(race_error < 10 && race_error > -10)
        robot[id].driving_state = 3;
    break;
  }
}

void tag::catch_pallet(int id) {

  int counter = 0;

  QList<QGraphicsItem *> list = scene()->items(QPolygonF()
                                               << mapToScene(-pi_radius, -pi_radius/5)
                                               << mapToScene(pi_radius, -pi_radius/5)
                                               << mapToScene(pi_radius, pi_radius/5)
                                               << mapToScene(-pi_radius, pi_radius/5));

  Q_FOREACH(QGraphicsItem *i, list) {
    pallet *pallet_item = dynamic_cast<pallet *>(i);

    if(pallet_item != NULL) {
      bool uncatched = false;
      counter++;
      double alpha = pallet_item->angle - robot[id].angle;

      if(robot[id].catch_pallet == true && robot[id].transporting_pallet == false && (abs(alpha) < 190 && abs(alpha) > 170)) {
        pallet_item->ready    = false;
        pallet_item->catched  = true;
        pallet_item->robot_id = id;
        robot[id].transporting_pallet = true;
      }
      else if(robot[id].catch_pallet == false && pallet_item->catched == true) {
        pallet_item->catched = false;
        robot[id].transporting_pallet = false;
        uncatched = true;

        int factor = 1.5, width = scene()->width(), height = scene()->height();
        if(robot[id].centroid.x + pi_radius * cos(robot[id].angle * PI / 180) * factor < width - wallWidth &&
           robot[id].centroid.x + pi_radius * cos(robot[id].angle * PI / 180) * factor > width - wallWidth - 170 &&
           robot[id].centroid.y + pi_radius * sin(robot[id].angle * PI / 180) * factor < height - wallWidth &&
           robot[id].centroid.y + pi_radius * sin(robot[id].angle * PI / 180) * factor > height - wallWidth - 250) {

          if(pallet_item->transf_counter >= 2 && pallet_item->state != 3)
            team[0].wareh_pallets++;
        }

        else if(robot[id].centroid.x + pi_radius * cos(robot[id].angle * PI / 180) * factor > borderOffset_x + wallWidth &&
           robot[id].centroid.x + pi_radius * cos(robot[id].angle * PI / 180) * factor < borderOffset_x + wallWidth + 170 &&
           robot[id].centroid.y + pi_radius * sin(robot[id].angle * PI / 180) * factor < height - wallWidth &&
           robot[id].centroid.y + pi_radius * sin(robot[id].angle * PI / 180) * factor > height - wallWidth - 250) {

          if(pallet_item->transf_counter >= 2 && pallet_item->state != 3)
            team[1].wareh_pallets++;
        }
      }

      if(robot[id].catch_pallet == false && (abs(alpha) < 190 && abs(alpha) > 170) && uncatched == false) {
        pallet_item->ready = true;
      }
    }
  }

  if(counter == 0)
    robot[id].catch_pallet = false;
}


void tag::immobilus(int id) {
  stop_counter++;
  robot[id].immobilized = true;
  robot[id].vel1 = 0;
  robot[id].vel2 = 0;

  if(stop_counter == 100) {
    robot[id].immobilized = false;
    immobilize = false;
    stop_counter = 0;
  }
}

void tag::lapState(int id) {
  int goalDistance, x_goal, y_goal;

  switch (robot[id].lap_state) {
    case 0:
      if(robot[id].initial_angle == 0) {
        goalDistance = (borderOffset_x + goal_x + wallWidth + 25) - robot[id].centroid.x;
        y_goal       = goal_y_min + wallWidth + 50;

        if(goalDistance < pi_radius && goalDistance > 0 && robot[id].centroid.y > y_goal && robot[id].centroid.y < y_goal + 150 && (robot[id].angle > 270 || robot[id].angle < 90) && robot[id].linear_direction > 0)
          robot[id].lap_state = 1;

        else if(-goalDistance < pi_radius && goalDistance < 0 && robot[id].centroid.y < scene()->height()/2 && (((robot[id].angle > 270 || robot[id].angle < 90) && robot[id].linear_direction < 0) ||
                                                                                                                 (robot[id].angle > 90 && robot[id].angle < 270) && robot[id].linear_direction > 0))
          robot[id].lap_state = 2;
      }

      else if(robot[id].initial_angle == 180) {
        goalDistance = robot[id].centroid.x - (borderOffset_x + goal_x + wallWidth + 25);
        y_goal = goal_y_max + wallWidth + 50;

        if(goalDistance < pi_radius && goalDistance > 0 && robot[id].centroid.y > y_goal && robot[id].centroid.y < y_goal + 150 && (robot[id].angle > 90 && robot[id].angle < 270) && robot[id].linear_direction > 0)
          robot[id].lap_state = 1;

        else if(-goalDistance < pi_radius && goalDistance < 0 && robot[id].centroid.y > scene()->height()/2 && (((robot[id].angle > 270 || robot[id].angle < 90) && robot[id].linear_direction > 0) ||
                                                                                                                 (robot[id].angle > 90 && robot[id].angle < 270) && robot[id].linear_direction < 0))
          robot[id].lap_state = 2;
      }

      else if(robot[id].initial_angle == 270) {
        goalDistance = robot[id].centroid.y - (goal_y + wallWidth + 50);
        x_goal = borderOffset_x + goal_x_min + wallWidth + 25;

        if(goalDistance < pi_radius && goalDistance > 0 && robot[id].centroid.x > x_goal && robot[id].centroid.x < x_goal + 150 && (robot[id].angle > 180 && robot[id].angle < 360) && robot[id].linear_direction > 0)
          robot[id].lap_state = 1;

        else if(-goalDistance < pi_radius && goalDistance < 0 && robot[id].centroid.x < scene()->width()/2 && (((robot[id].angle > 180 && robot[id].angle < 360) && robot[id].linear_direction < 0) ||
                                                                                                                (robot[id].angle > 0 && robot[id].angle < 180) && robot[id].linear_direction > 0))
          robot[id].lap_state = 2;
      }

      else if(robot[id].initial_angle == 90) {
        goalDistance = (goal_y + wallWidth + 50) - robot[id].centroid.y;
        x_goal = borderOffset_x + goal_x_max + wallWidth + 25;

        if(goalDistance < pi_radius && goalDistance > 0 && robot[id].centroid.x > x_goal && robot[id].centroid.x < x_goal + 150 && (robot[id].angle > 0 && robot[id].angle < 180) && robot[id].linear_direction > 0)
          robot[id].lap_state = 1;

        else if(-goalDistance < pi_radius && goalDistance < 0 && robot[id].centroid.x > scene()->width()/2 && (((robot[id].angle > 180 && robot[id].angle < 360) && robot[id].linear_direction > 0) ||
                                                                                                                (robot[id].angle > 0 && robot[id].angle < 180) && robot[id].linear_direction < 0))
          robot[id].lap_state = 2;
      }
    break;

    case 1:
      team[robot[id].teamId].lapsDone++;
      robot[id].lap_state = 3;
    break;

    case 2:
      team[robot[id].teamId].lapsDone--;
      robot[id].lap_state = 4;
    break;

    case 3:
      if(robot[id].initial_angle == 0) {
        goalDistance = robot[id].centroid.x - (borderOffset_x + goal_x + wallWidth + 25);

        if(goalDistance > pi_radius && goalDistance > 0)
          robot[id].lap_state = 0;
      }

      else if(robot[id].initial_angle == 180) {
        goalDistance = (borderOffset_x + goal_x + wallWidth + 25) - robot[id].centroid.x;

        if(goalDistance > pi_radius && goalDistance > 0)
          robot[id].lap_state = 0;
      }

      else if(robot[id].initial_angle == 270) {
        goalDistance = (goal_y + wallWidth + 50) - robot[id].centroid.y;

        if(goalDistance > pi_radius && goalDistance > 0)
          robot[id].lap_state = 0;
      }

      else if(robot[id].initial_angle == 90) {
        goalDistance = robot[id].centroid.y - (goal_y + wallWidth + 50);

        if(goalDistance > pi_radius && goalDistance > 0)
          robot[id].lap_state = 0;
      }
    break;

    case 4:
      if(robot[id].initial_angle == 0) {
        goalDistance = robot[id].centroid.x - (borderOffset_x + goal_x + wallWidth + 25);

        if(goalDistance > pi_radius && goalDistance > 0) {
          robot[id].lap_state = 0;
          team[robot[id].teamId].lapsDone++;
        }
      }

      else if(robot[id].initial_angle == 180) {
        goalDistance = (borderOffset_x + goal_x + wallWidth + 25) - robot[id].centroid.x;

        if(goalDistance > pi_radius && goalDistance > 0) {
          robot[id].lap_state = 0;
          team[robot[id].teamId].lapsDone++;
        }
      }

      else if(robot[id].initial_angle == 270) {
        goalDistance = (goal_y + wallWidth + 50) - robot[id].centroid.y;

        if(goalDistance > pi_radius && goalDistance > 0) {
          robot[id].lap_state = 0;
          team[robot[id].teamId].lapsDone++;
        }
      }

      else if(robot[id].initial_angle == 90) {
        goalDistance = robot[id].centroid.y - (goal_y + wallWidth + 50);

        if(goalDistance > pi_radius && goalDistance > 0) {
          robot[id].lap_state = 0;
          team[robot[id].teamId].lapsDone++;
        }
      }
    break;
  }
}

QRectF tag::boundingRect() const {
  return QRectF(-pi_radius, -pi_radius, 2 * pi_radius, 2 * pi_radius);
}

QPainterPath tag::shape() const {
  QPainterPath path;
  path.addEllipse(boundingRect());
  return path;
}

void tag::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  QRectF rect = boundingRect();
  QPen pen(color, 3);
  painter->setPen(pen);
  painter->drawEllipse(rect);

  if(game == 0 || game == 2) {
    QLine line = QLine(pi_radius, 0, pi_radius + 20, 0);
    painter->drawLine(line);
  }
  else if(game == 1) {
    QLine line_a = QLine(pi_radius, -pi_radius/2, pi_radius + 20, -pi_radius/2);
    QLine line_b = QLine(pi_radius, +pi_radius/2, pi_radius + 20, +pi_radius/2);
    painter->drawLine(line_a);
    painter->drawLine(line_b);
  }
}

void tag::advance(int j) {

  setPos(robot[j].centroid.x, robot[j].centroid.y);
  setRotation(robot[j].angle);


  /* --- GAME ENGINE --- */

  bool colAuxFlag = false;
  QList<QGraphicsItem *> list = scene()->collidingItems(this, Qt::IntersectsItemShape);

  /* --- ESSENTIALLY PITANK ---- */

  Q_FOREACH(QGraphicsItem *i, list) {
    wall *wall_item = dynamic_cast<wall *>(i);
    if(wall_item != NULL) {
      colAuxFlag = true;
      if(robot[j].threadIsRunning == false) {
        th[j] = std::thread(&tag::collisionState, this, j);
        th[j].detach();
      }
    }

    bullet *bullet_item = dynamic_cast<bullet *>(i);
    if(bullet_item != NULL) {
      if(bullet_item->robot_sender != j) {
        bullet_item->collision();
        robot[j].damage++;
        robot[bullet_item->robot_sender].kills++;
      }
    }
  }

  robot[j].collisionFlag = colAuxFlag;


  /* --- ROBOT @ FACTORY ---- */

  if(game == 1) {
    catch_pallet(j);
    if(immobilize == true)
      immobilus(j);
  }

  /* --- RACE CAR --- */


  if(game == 2) {

    if(raceHasStarted == true)
      lapState(j);

    bool inside = false, outside = false;
    QList<QGraphicsItem *> race_list = scene()->collidingItems(this, Qt::IntersectsItemShape);
    Q_FOREACH(QGraphicsItem *i, race_list) {
      racemap *racemap_item = dynamic_cast<racemap *>(i);
      if(racemap_item != NULL) {
        if(racemap_item->outside == true)
          outside = true;
        if(racemap_item->outside == false)
          inside = true;
      }
    }

    if((inside == true && outside == true && game == 2) || (inside == false && outside == false && game == 2) && robot[j].autonomous_drive == false) {
      robot[j].slow_motion = true;
      color = Qt::red;
    }
    else {
      robot[j].slow_motion = false;
      color = team[robot[j].teamId].team_color;
    }
  }

  /* --- PODIUM --- */

  if(robot[j].autonomous_drive == true && cv::norm(robot[j].driving_pos) > 0)
    send_to_pos(cv::Point2f(robot[j].driving_pos.x, robot[j].driving_pos.y), j);

}


void tag::delete_item() {
  this->deleteLater();
}


team_info::team_info(QColor color, cv::Point2f pos, int id) {
  team_color = color;
  class_pos = pos;
  damage = 1;
  kills = 1;
  wareh_pallets = 0;
  lapsDone = 0;
  classification = id + 1;

  team.push_back(*this);
}
