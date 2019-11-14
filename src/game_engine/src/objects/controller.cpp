#include "../../include/objects/controller.hpp"

using namespace std;

controller::controller(MainWindow *w, Dialog *dialog) {

  gui = w;
  d = dialog;

  gui->idA = d->tag_id[0];
  gui->idB = d->tag_id[1];
  gui->idC = d->tag_id[2];
  gui->idD = d->tag_id[3];
}


void controller::throw_thread() {
  std::thread t(&controller::game_state, this);
  t.detach();
}

bool controller::game_state() {
  int nextstate;

  while(1) {

    gameRunning = gameStart;
    gamePaused  = d->pause_flag;

    switch(nextstate) {

      //setup buttons
      case 0:
        pitank_gamestate = 0;
        robotf_gamestate = 0;
        raceCar_gamestate = 0;
        state = 1;
      break;


      //waiting for calibration
      case 1:
      break;


      //calibration done + setup done
      case 2:
        d->finish_flag = false;
        d->init_flag = false;
        setupStart = true;
        state = 3;
      break;

      //waiting for the game to start
      case 3:
      break;


      //game play
      case 4:
        d->pause_flag = false;
        gui->paused = d->pause_flag;
        setupStart = false;
        gameStart = true;
        switch(game) {
          case 0:
            pitank_state();
          break;

          case 1:
            robotFactory_state();
          break;

          case 2:
            raceCar_state();
          break;
        }
      break;


      //game paused
      case 5:
        d->pause_flag = true;
        gui->paused = d->pause_flag;
      break;

      //reset game
      case 6:
        gameStart = false;
        d->reset_game();
        state = 0;
      break;
    }
    nextstate = state;
  }
}

bool controller::pitank_state() {

  switch(pitank_gamestate) {
    case 0:
      pitank_scores();
    break;

    case 1:
      d->finish_flag = true;
      pitank_gamestate = 2;
    break;

    case 2:
      if((teamGame == false && number_robots > 3 && driving_counter == 3) || (teamGame == true && number_robots > 3 && driving_counter == 4) ||
         (number_robots <= 3 && driving_counter == number_robots)) {

        QElapsedTimer timer;
        timer.start();
        while(timer.nsecsElapsed() <= podium_time);
        pitank_gamestate = 3;
      }
    break;

    case 3:
      state = 6;
      pitank_gamestate = -1;
    break;
  }
}

bool controller::pitank_scores() {
  std::multimap<double, int> classifications;
  std::multimap<double, int>::iterator it;

  for(size_t i = 0; i < team.size(); i++) {
    team[i].damage = 0;
    team[i].kills  = 0;
    for(size_t j = 0; j < team[i].member_id.size(); j++) {
      team[i].damage += robot[team[i].member_id[j]].damage;
      team[i].kills  += robot[team[i].member_id[j]].kills;
    }
  }

  for(size_t i = 0; i < team.size(); i++) {
    classifications.insert(std::pair<double, int>((double)team[i].damage/team[i].kills, i));
  }

  int j = 1;
  for(it = classifications.begin(); it != classifications.end(); ++it) {
    for(size_t i = 0; i < team.size(); i++) {
      if(i == it->second)
        team[i].classification = j;
    }
    j++;
  }

  for(size_t i = 0; i < team.size(); i++) {
    switch (team[i].classification) {
      case 1:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yA);
      break;

      case 2:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yB);
      break;

      case 3:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yC);
      break;

      case 4:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yD);
      break;

      case 5:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yE);
      break;
    }
  }

  return true;
}

bool controller::robotFactory_state() {

  switch(robotf_gamestate) {
    case 0:
      robotf_scores();
    break;

    case 1:
      d->finish_flag = true;
      robotf_gamestate = 2;
    break;

    case 2:
      if(driving_counter == number_robots) {
        QElapsedTimer timer;
        timer.start();
        while(timer.nsecsElapsed() <= podium_time);
        robotf_gamestate = 3;
      }
    break;

    case 3:
      state = 6;
      robotf_gamestate = -1;
    break;
  }
}

bool controller::robotf_scores() {
  std::multimap<double, int> classifications;
  std::multimap<double, int>::iterator it;

  for(size_t i = 0; i < team.size(); i++) {
    classifications.insert(std::pair<double, int>((double)1/team[i].wareh_pallets, i));
  }

  int j = 1;
  for(it = classifications.begin(); it != classifications.end(); ++it) {
    for(size_t i = 0; i < team.size(); i++) {
      if(i == it->second)
        team[i].classification = j;
    }
    j++;
  }

  for(size_t i = 0; i < team.size(); i++) {
    switch (team[i].classification) {
      case 1:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yA);
      break;

      case 2:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yB);
      break;
    }
  }
}

bool controller::raceCar_state() {

  int safe_offset;

  switch(raceCar_gamestate) {
    case 0:
      safe_offset = 60;
      robot[0].autonomous_drive = true;
      robot[0].initial_angle = 0;
      robot[0].driving_pos = cv::Point2f(borderOffset_x + goal_x + wallWidth + 25 + safe_offset, goal_y_min + 75 + wallWidth + 50);

      if(robot.size() > 1) {
        robot[1].initial_angle = 180;
        robot[1].autonomous_drive = true;
        robot[1].driving_pos = cv::Point2f(borderOffset_x + goal_x + wallWidth + 25 - safe_offset, goal_y_max - 75 + wallWidth + 50);
      }

      if(robot.size() > 2) {
        robot[2].initial_angle = 270;
        robot[2].autonomous_drive = true;
        robot[2].driving_pos = cv::Point2f(borderOffset_x + goal_x_min + wallWidth + 25 + 75, goal_y + wallWidth + 50 - safe_offset);
      }

      if(robot.size() > 3) {
        robot[3].initial_angle = 90;
        robot[3].autonomous_drive = true;
        robot[3].driving_pos = cv::Point2f(borderOffset_x + goal_x_max + wallWidth + 25 - 75, goal_y + wallWidth + 50 + safe_offset);
      }

      if(driving_counter == robot.size()) {
        for(size_t i = 0; i < robot.size(); i++) {
          robot[i].driving_state = 0;
          robot[i].autonomous_drive = false;
        }
        driving_counter = 0;
        d->paintGoals();
        raceCar_gamestate = 1;
      }
    break;

    case 1:
      raceCar_scores();
    break;

    case 2:
      d->finish_flag = true;
      raceCar_gamestate = 3;
    break;

    case 3:
      if((driving_counter == 3 && robot.size() >= 3) || (driving_counter == robot.size() && robot.size() < 3)) {
        QElapsedTimer timer;
        timer.start();
        while(timer.nsecsElapsed() <= podium_time);
        raceCar_gamestate = 4;
      }
    break;

    case 4:
      state = 6;
      raceCar_gamestate = -1;
    break;
  }
}

bool controller::raceCar_scores() {
  std::multimap<double, int> classifications;
  std::multimap<double, int>::iterator it;

  for(size_t i = 0; i < team.size(); i++) {
    classifications.insert(std::pair<double, int>((double)1/team[i].lapsDone, i));
  }

  int j = 1;
  for(it = classifications.begin(); it != classifications.end(); ++it) {
    for(size_t i = 0; i < team.size(); i++) {
      if(i == it->second)
        team[i].classification = j;

      if(team[i].lapsDone == number_laps)
        raceCar_gamestate = 2;
    }
    j++;
  }

  for(size_t i = 0; i < team.size(); i++) {
    switch (team[i].classification) {
      case 1:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yA);
      break;

      case 2:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yB);
      break;

      case 3:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yC);
      break;

      case 4:
        team[i].class_pos = cv::Point2f(classNr_x, classNr_yD);
      break;
    }
  }
}

controller::~controller() {}
