#include "../../include/graphics/dialog.hpp"


void Dialog::initMap() {
  QLineF borderLine[4];
  borderLine[0].setP1(scene.sceneRect().topLeft() + QPointF(borderOffset_x, 0)); borderLine[0].setP2(scene.sceneRect().bottomLeft() + QPointF(borderOffset_x, 0));
  borderLine[1].setP1(scene.sceneRect().topLeft() + QPointF(borderOffset_x, 0)); borderLine[1].setP2(scene.sceneRect().topRight());
  borderLine[2].setP1(scene.sceneRect().bottomLeft() + QPointF(borderOffset_x, 0)); borderLine[2].setP2(scene.sceneRect().bottomRight());
  borderLine[3].setP1(scene.sceneRect().topRight()); borderLine[3].setP2(scene.sceneRect().bottomRight());
  for(int i = 0; i < 4; i++) {
    w = new wall(borderLine[i].p1(), borderLine[i].p2(), true, false, false);
    scene.addItem(w);
  }
}

void Dialog::convertWallToMessage(wall *w){
    w_info.life = w->life;
    w_info.x1 = w->wallLine.x1();
    w_info.x2 = w->wallLine.x2();
    w_info.y1 = w->wallLine.y1();
    w_info.y2 = w->wallLine.y2();
    w_info.id = w->id;
    w_info.cluster = floor(w->id/2);
    w_info.xc = (w->wallLine.x1()+w->wallLine.x2())/2;
    w_info.yc = (w->wallLine.y1()+w->wallLine.y2())/2;
    if((w->wallLine.x1()-w->wallLine.x2())==0) w_info.vertical = true; //vertical
    else w_info.vertical = false;
    w_array.wall.push_back(w_info);
}

void Dialog::drawPitankMap() {
  int offSetX = 200, offSetY = 225, width = scene.width(), height = scene.height();
  QPointF p1, p2;

  p1.setX(offSetX); p1.setY(offSetY - wallAHeight);
  p2.setX(offSetX); p2.setY(offSetY);
  w = new wall(p1, p2, false, false, false, 0);
  scene.addItem(w);
  printf("\n Parede 1 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(offSetX); p1.setY(offSetY);
  p2.setX(offSetX + wallAHeight); p2.setY(offSetY);
  w = new wall(p1, p2, false, false, false, 1);
  scene.addItem(w);
  printf("\n Parede 2 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(offSetX); p1.setY(height - offSetY);
  p2.setX(offSetX); p2.setY(height - offSetY + wallAHeight);
  w = new wall(p1, p2, false, false, false, 2);
  scene.addItem(w);
  printf("\n Parede 3 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(offSetX); p1.setY(height - offSetY);
  p2.setX(offSetX + wallAHeight); p2.setY(height - offSetY);
  w = new wall(p1, p2, false, false, false, 3);
  scene.addItem(w);
  printf("\n Parede 4 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(width - offSetX + borderOffset_x); p1.setY(offSetY - wallAHeight);
  p2.setX(width - offSetX + borderOffset_x); p2.setY(offSetY);
  w = new wall(p1, p2, false, false, false, 4);
  scene.addItem(w);
  printf("\n Parede 5 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(width - offSetX - wallAHeight + borderOffset_x); p1.setY(offSetY);
  p2.setX(width - offSetX + borderOffset_x); p2.setY(offSetY);
  w = new wall(p1, p2, false, false, false, 5);
  scene.addItem(w);
  printf("\n Parede 6 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(width - offSetX + borderOffset_x); p1.setY(height - offSetY);
  p2.setX(width - offSetX + borderOffset_x); p2.setY(height - offSetY + wallAHeight);
  w = new wall(p1, p2, false, false, false, 6);
  scene.addItem(w);
  printf("\n Parede 7 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(width - offSetX - wallAHeight + borderOffset_x); p1.setY(height - offSetY);
  p2.setX(width - offSetX + borderOffset_x); p2.setY(height - offSetY);
  w = new wall(p1, p2, false, false, false, 7);
  scene.addItem(w);
  printf("\n Parede 8 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(width/2 - wallBHeight/2); p1.setY(height/2);
  p2.setX(width/2 + wallBHeight/2); p2.setY(height/2);
  w = new wall(p1, p2, false, false, false, 8);
  scene.addItem(w);
  printf("\n Parede 9 - P1:   x = %d ; y = %d \n"
         "            P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);

  p1.setX(width/2); p1.setY(height/2 - wallBHeight/2);
  p2.setX(width/2); p2.setY(height/2 + wallBHeight/2);
  w = new wall(p1, p2, false, false, false, 9);
  scene.addItem(w);
  printf("\n Parede 10 - P1:   x = %d ; y = %d \n"
         "             P2:   x = %d ; y = %d \n\n",(int)p1.x(),(int)p1.y(),(int)p2.x(),(int)p2.y());
  convertWallToMessage(w);
  wallInfo_pub.publish(w_array);
}


void Dialog::drawRobotFactoryMap() {

  cv::Point2f pos;
  float off_1 = projectedImageSizeX, off_2 = projectedImageSizeY;

  pos = cv::Point2f(off_1/2 - 40, off_2/2 - 40);
  mc = new machine(pos, 0);
  scene.addItem(mc);

  pos = cv::Point2f(off_1/2 - 40, off_2/2 + 40);
  mc = new machine(pos, 1);
  scene.addItem(mc);

  pos = cv::Point2f(off_1/2 + 40, off_2/2 + 40);
  mc = new machine(pos, 0);
  scene.addItem(mc);

  pos = cv::Point2f(off_1/2 + 40, off_2/2 - 40);
  mc = new machine(pos, 1);
  scene.addItem(mc);

  int offSetX = 200, offSetY = 250, width = scene.width(), height = scene.height();
  QPointF p1, p2;


  p1.setX(off_1/2); p1.setY(off_2/2 - 40);
  p2.setX(off_1/2); p2.setY(off_2/2 + 40);
  w = new wall(p1, p2, false, false, false);
  scene.addItem(w);

  p1.setX(off_1/2 - 40); p1.setY(off_2/2);
  p2.setX(off_1/2 + 40); p2.setY(off_2/2);
  w = new wall(p1, p2, false, false, false);
  scene.addItem(w);

  p1.setX(borderOffset_x + wallWidth + offSetX); p1.setY(0);
  p2.setX(borderOffset_x + wallWidth + offSetX); p2.setY(wareh_wallB);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 0;
  scene.addItem(w);

  p1.setX(borderOffset_x + wallWidth + offSetX); p1.setY(175);
  p2.setX(borderOffset_x + wallWidth + offSetX); p2.setY(175 + wareh_wallB);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 0;
  scene.addItem(w);

  p1.setX(borderOffset_x + wallWidth); p1.setY(offSetY);
  p2.setX(borderOffset_x + wallWidth + wareh_wallA); p2.setY(offSetY);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 0;
  scene.addItem(w);


  p1.setX(width - offSetX); p1.setY(height - wareh_wallB);
  p2.setX(width - offSetX); p2.setY(height);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 0;
  scene.addItem(w);

  p1.setX(width - offSetX); p1.setY(height - wareh_wallB - 175);
  p2.setX(width - offSetX); p2.setY(height - 175);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 0;
  scene.addItem(w);

  p1.setX(width - wareh_wallA); p1.setY(height - offSetY);
  p2.setX(width); p2.setY(height - offSetY);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 0;
  scene.addItem(w);

  int x_step = 50, y_step = 25, pallet_w = 35;

  pos = cv::Point2f(borderOffset_x + wallWidth * 1.5 + x_step, y_step + wallWidth);
  p = new pallet(pos, 0);
  scene.addItem(p);

  pos = cv::Point2f(borderOffset_x + wallWidth * 1.5 + pallet_w + 2 * x_step, y_step + wallWidth);
  p = new pallet(pos, 1);
  scene.addItem(p);

  pos = cv::Point2f(borderOffset_x + wallWidth * 1.5 + x_step, offSetY - y_step - 2 * wallWidth);
  p = new pallet(pos, 0);
  scene.addItem(p);

  pos = cv::Point2f(borderOffset_x + wallWidth * 1.5 + pallet_w + 2 * x_step, offSetY - y_step - 2 * wallWidth);
  p = new pallet(pos, 1);
  scene.addItem(p);




  p1.setX(width - offSetX); p1.setY(0);
  p2.setX(width - offSetX); p2.setY(wareh_wallB);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 1;
  scene.addItem(w);

  p1.setX(width - offSetX); p1.setY(175);
  p2.setX(width - offSetX); p2.setY(175 + wareh_wallB);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 1;
  scene.addItem(w);

  p1.setX(width); p1.setY(offSetY);
  p2.setX(width - wareh_wallA); p2.setY(offSetY);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 1;
  scene.addItem(w);


  p1.setX(borderOffset_x + wallWidth + offSetX); p1.setY(height - wareh_wallB);
  p2.setX(borderOffset_x + wallWidth + offSetX); p2.setY(height);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 1;
  scene.addItem(w);

  p1.setX(borderOffset_x + wallWidth + offSetX); p1.setY(height - wareh_wallB - 175);
  p2.setX(borderOffset_x + wallWidth + offSetX); p2.setY(height - 175);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 1;
  scene.addItem(w);

  p1.setX(borderOffset_x + wallWidth); p1.setY(height - offSetY);
  p2.setX(borderOffset_x + wallWidth + wareh_wallA); p2.setY(height - offSetY);
  w = new wall(p1, p2, false, false, true);
  w->team_id = 1;
  scene.addItem(w);


  x_step = 50;

  pos = cv::Point2f(width - wallWidth - x_step, y_step + wallWidth);
  p = new pallet(pos, 0);
  scene.addItem(p);

  pos = cv::Point2f(width - wallWidth - pallet_w - 2 * x_step, y_step + wallWidth);
  p = new pallet(pos, 1);
  scene.addItem(p);

  pos = cv::Point2f(width - wallWidth - x_step, offSetY - y_step - 2 * wallWidth);
  p = new pallet(pos, 0);
  scene.addItem(p);

  pos = cv::Point2f(width - wallWidth - pallet_w - 2 * x_step, offSetY - y_step - 2 * wallWidth);
  p = new pallet(pos, 1);
  scene.addItem(p);

}
