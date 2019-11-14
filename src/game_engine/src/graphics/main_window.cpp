#include "../../include/graphics/main_window.hpp"

using namespace Qt;
using namespace std;

bool setupStart = false, gameStart = false, paused = false, teamGame = false, aiGame = false, simGame = false;
int  game = -1, state = -1, pitank_gamestate = -1, robotf_gamestate = -1, raceCar_gamestate = -1, robotn_sim = -1;
QTime game_clock;

std::string int_to_string(int i){
    stringstream ss;
    ss << i;
    string str = ss.str();
    return str;
}

MainWindow::MainWindow(QDialog *dialog, QWidget *parent) : QMainWindow(parent) {

  ui.setupUi(this);

  game_option[0] = "Pi tank";
  game_option[1] = "Robot factory";
  game_option[2] = "Robot race";
  ui.game_choose->addItem(game_option[0], 0);
  ui.game_choose->addItem(game_option[1], 0);
  ui.game_choose->addItem(game_option[2], 0);
  ui.robotn_box->addItem("1",1);
  ui.robotn_box->addItem("2",2);
  ui.robotn_box->addItem("3",3);
  ui.robotn_box->addItem("4",4);

  calibrated = 0;
  state = 0;
  paused = 0;
  corners.push_back(cv::Point2f(0,0));
  corners.push_back(cv::Point2f(0,0));
  corners.push_back(cv::Point2f(0,0));
  corners.push_back(cv::Point2f(0,0));

  ui.calibrate->setEnabled(false);
  ui.setup_game->setEnabled(false);
  ui.start->setEnabled(false);
  ui.pause->setEnabled(false);
  ui.stop->setEnabled(false);

  QWidget::showMaximized();

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update_time()));
  timer->start(1000);

  //QObject::connect(ui.game_choose, SIGNAL(currentIndexChanged(int)), ui.sim_bool, SLOT(toggle_sim_button(int)));
  connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  connect(this, SIGNAL(calibrate()), dialog, SLOT(endCalibrationSIGNAL()));
  connect(this, SIGNAL(manually_calibrate(std::vector<cv::Point2f>)), dialog, SLOT(manually_calibration(std::vector<cv::Point2f>)));
  connect(this, SIGNAL(tryCalibration(int64_t)), dialog, SLOT(calibrateSIGNAL(int64_t)));
}

MainWindow::~MainWindow() {
  this->destroy();
}

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::update_time() {
  if(ui.game_time->time() == QTime(0,0) && gameStart == true) {
    switch(game) {
      case 0:
        pitank_gamestate = 1;
      break;

      case 1:
        robotf_gamestate = 1;
      break;

      case 2:
        raceCar_gamestate = 2;
      break;
    }
  }

  if(gameStart == true && paused == false && ui.game_time->time() > QTime(0,0)) {
    QTime time = ui.game_time->time().addSecs(-1);
    ui.game_time->setTime(time);
  }
}

void MainWindow::on_threshold_valueChanged(int threshold) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }

  int64_t thresh = int64_t(threshold);
  Q_EMIT tryCalibration(thresh);
}

void MainWindow::canStart(bool calibDone, cv::Mat image, std::vector<cv::Point2f> corners) {
  if(calibDone == true) {
    cv::cvtColor(image, image, CV_GRAY2BGR);
    ui.calib_result->setPixmap(QPixmap::fromImage(QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888)).scaled(1500, 1500, KeepAspectRatio));
  }
  this->corners = corners;
  ui.c1_x->setValue(corners[0].x);
  ui.c1_y->setValue(corners[0].y);
  ui.c2_x->setValue(corners[1].x);
  ui.c2_y->setValue(corners[1].y);
  ui.c3_x->setValue(corners[2].x);
  ui.c3_y->setValue(corners[2].y);
  ui.c4_x->setValue(corners[3].x);
  ui.c4_y->setValue(corners[3].y);
}

void MainWindow::on_calibrate_clicked() {
  ui.setup_game->setEnabled(true);
  ui.calibrate->setEnabled(false);
  calibrated = 1;
  Q_EMIT calibrate();
}

void MainWindow::on_setup_game_clicked() {
  if(ui.game_time->time() != QTime(0,0)) {
    state = 2;

    ui.start->setEnabled(true);

    game_clock = ui.game_time->time();

    if(ui.game_choose->currentText() == game_option[0])
      game = 0;
    else if(ui.game_choose->currentText() == game_option[1])
      game = 1;
    else if(ui.game_choose->currentText() == game_option[2])
      game = 2;

    ui.game_choose->setEnabled(false);

    if(ui.teams_bool->isChecked() == true)
      teamGame = true;
    else
      teamGame = false;

    if(ui.ai_bool->isChecked() == true)
      aiGame = true;
    else
      aiGame = false;

    if(ui.sim_bool->isChecked() == true)
      simGame = true;
    else
      simGame = false;

    robotn_sim = ui.robotn_box->currentIndex() + 1;
    printf(" YAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA %d", robotn_sim);
  }
  else
    state = 6;
}

void MainWindow::on_start_clicked() {
  ui.setup_game->setEnabled(false);
  ui.start->setEnabled(false);
  ui.pause->setEnabled(true);
  ui.stop->setEnabled(true);

  if(state == 3)
      state = 4;
}

void MainWindow::on_pause_clicked() {
  if(state == 4 || state == 5) {
    if(paused == false)
      state = 5;
    else if(paused == true)
      state = 4;
  }
}

void MainWindow::on_stop_clicked() {
  if(state == 4 || state == 5) {
    switch(game) {
      case 0:
        pitank_gamestate = 3;
      break;

      case 1:
        robotf_gamestate = 3;
      break;

      case 2:
        raceCar_gamestate = 4;
      break;
    }
    ui.game_time->setTime(QTime(0,0));
  }
}

void MainWindow::reset_buttons_slot() {
  ui.game_choose->setDisabled(false);
  ui.setup_game->setEnabled(true);
  ui.pause->setEnabled(false);
  ui.stop->setEnabled(false);

  ui.game_time->setTime(QTime(0,0));
}

void MainWindow::detected_robots_slot(std::map<int, int> robots) {
  if(robots.find(idA) != robots.end())
    ui.robotAcheck->setChecked(true);
  else
    ui.robotAcheck->setChecked(false);
  if(robots.find(idB) != robots.end())
    ui.robotBcheck->setChecked(true);
  else
    ui.robotBcheck->setChecked(false);
  if(robots.find(idC) != robots.end())
    ui.robotCcheck->setChecked(true);
  else
    ui.robotCcheck->setChecked(false);
  if(robots.find(idD) != robots.end())
    ui.robotDcheck->setChecked(true);
  else
    ui.robotDcheck->setChecked(false);
}

void MainWindow::on_c1_x_valueChanged(int c1_x) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[0].x = c1_x;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_c1_y_valueChanged(int c1_y) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[0].y = c1_y;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_c2_x_valueChanged(int c2_x) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[1].x = c2_x;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_c2_y_valueChanged(int c2_y) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[1].y = c2_y;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_c3_x_valueChanged(int c3_x) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[2].x = c3_x;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_c3_y_valueChanged(int c3_y) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[2].y = c3_y;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_c4_x_valueChanged(int c4_x) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[3].x = c4_x;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_c4_y_valueChanged(int c4_y) {
  if(gameStart == false) {
    ui.calibrate->setEnabled(true);
    ui.setup_game->setEnabled(false);
    calibrated = 0;
  }
  corners[3].y = c4_y;
  Q_EMIT manually_calibrate(corners);
}

void MainWindow::on_game_choose_currentIndexChanged(int mode){
    if(mode == 0){
        ui.sim_bool->setEnabled(true);
        ui.ai_bool->setEnabled(true);
        ui.label_2->setEnabled(true);
        ui.robotn_box->setEnabled(true);
    }
    else {
        ui.sim_bool->setChecked(false);
        ui.sim_bool->setEnabled(false);
        ui.ai_bool->setChecked(false);
        ui.ai_bool->setEnabled(false);
        ui.label_2->setEnabled(false);
        ui.robotn_box->setEnabled(false);
    }
}

void MainWindow::on_sim_bool_stateChanged(int n){
    if(n == 0){
        ui.setup_game->setEnabled(false);
    }
    else if(n == 2){
        ui.setup_game->setEnabled(true);
    }
}
