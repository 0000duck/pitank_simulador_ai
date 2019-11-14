#include "../../include/objects/machine.hpp"

using namespace std;

int counter = 0, time_elapsed = 0;

machine::machine(cv::Point2f pos, int machineType) {
  setPos(pos.x, pos.y);

  position       = pos;
  type           = machineType;
  global_state   = 0;
  state          = 0;
  machine_width  = 50;
  number_pallets = 0;
  broken         = false;
  worker         = NULL;

  if(type == 0) {
    type_color = Qt::yellow;
    processing_time = 10;
  }
  else if(type == 1) {
    type_color = Qt::blue;
    processing_time = 7;
  }

  this->setZValue(1);

  QTimer *t = new QTimer();
  machine::connect(t, SIGNAL(timeout()), this, SLOT(advance()));
  t->start(50);
}

QRectF machine::boundingRect() const {
  return QRectF(-machine_width/2, -machine_width/2, machine_width, machine_width);
}

QPainterPath machine::shape() const {
  QPainterPath path;
  path.addRect(boundingRect());
  return path;
}

void machine::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  switch(global_state) {
    // machine is free
    case 0:
      state_color = Qt::green;
    break;

    // a pallet is overloading the machine, but was not dropped yet
    case 1:
      state_color = Qt::darkMagenta;
    break;

    // a pallet was dropped over the machine (...) processing
    case 2:
      state_color = Qt::red;
    break;

    // the machine is broken
    case 3:
      state_color = Qt::black;
    break;
  }

  QRectF rect = boundingRect();

  QPen pen(state_color, 5);
  painter->setPen(pen);
  painter->drawEllipse(rect);

  QPen pencil(type_color, 5);
  QLine line(QPoint(0, -machine_width/3), QPoint(0, machine_width/3));
  painter->setPen(pencil);
  painter->drawLine(line);
}

void machine::delete_item() {
  this->deleteLater();
}

void machine::advance() {
  if(state == 2 || state == 3 || state == 4) {
    setRotation(counter++);
    counter = (counter == 360) ? 0 : counter;
  }

  bool intercept = false, catched = false, broken_flag = false, pallet_broken = false, type_match = true;
  float x_size = projectedImageSizeX;
  int number_pallets = 0;
  pallet* p = NULL;
  vector<pallet *> pallet_array;

  QList<QGraphicsItem *> list = scene()->collidingItems(this, Qt::IntersectsItemBoundingRect);
  Q_FOREACH(QGraphicsItem *i, list) {
    pallet *pallet_item = dynamic_cast<pallet *>(i);

    if(pallet_item != NULL) {
      number_pallets++;
      p = pallet_item;
      pallet_array.push_back(p);
      catched = pallet_item->catched;
      pallet_broken = p->broken;
      intercept = true;

      if(!(((pallet_item->type == 0 || pallet_item->type == 2) && this->type == 0) || pallet_item->type == 1 && this->type == 1))
        type_match = false;
    }
  }

  broken_flag = (number_pallets <= 1) ? false : true;
  state = (intercept == false) ? 0 : state;
  state = (broken_flag == true || pallet_broken || (type_match == false && state != 5)) ? 4 : state;
  global_state = (intercept == false) ? 0 : global_state;


  switch(state) {

    // pallet overloading the machine
    case 0:
      if(catched == true) {
        global_state = 1;
        state = 1;
      }
    break;

    // pallet dropped over the machine
    case 1:
      if(catched == false) {
        cv::Point2f pos = (this->x() < (float)x_size/2) ? cv::Point2f(borderOffset_x, this->y()) : cv::Point2f(scene()->width() - wallWidth * 2, this->y());
        worker = new person(pos, true);
        scene()->addItem(worker);
        state = 2;
      }
    break;

    // waiting for the operator
    case 2:
      if(worker != NULL) {
        if((this->x() < x_size/2 && worker->x() >= this->x() - machine_width/2) || (this->x() > x_size/2 && worker->x() <= this->x() + machine_width/2)) {
          worker->stop = true;
          state = 3;
          global_state = 2;
        }
      }
    break;

    // machine processing the pallet
    case 3:
      time_elapsed++;
      if(time_elapsed/20 >= processing_time) {
        time_elapsed = 0;
        state = 5;

        if(p->type == 0)
          p->type = 1;
        else if(p->type == 1)
          p->type = 2;
        else if(p->type == 2)
          p->type = 0;

        worker->stop = false;
        p->processed = true;
        p->transf_counter++;
      }
    break;

    // the machine is broken
    case 4:
      global_state = 3;
      broken = true;

      if(broken_flag == true) {
        for(size_t i = 0; i < pallet_array.size(); i++)
          pallet_array[i]->broken = true;
      }
    break;

    //pallet processed, waiting to be catched
    case 5:
      global_state = 1;
    break;
  }
}
