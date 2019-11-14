#include "../include/graphics/main_window.hpp"
#include "../include/graphics/dialog.hpp"
#include "../include/objects/controller.hpp"

SerialStream my_serial_stream, my_serial_port;

int main(int argc, char **argv) {

  my_serial_stream.Open("/dev/ttyUSB0");
  my_serial_stream.SetBaudRate(SerialStreamBuf::BAUD_57600);
  my_serial_port.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
  my_serial_port.SetNumOfStopBits(1);
  my_serial_port.SetParity(SerialStreamBuf::PARITY_NONE);
  my_serial_port.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);

  if(my_serial_stream.IsOpen()) {
    std::cout << "SUCCESSFUL: serial port opened at: /dev/ttyUSB0" << std::endl;
    usleep(5000);
  }
  else
    std::cout << "ERROR: Could not open serial port." << std::endl;

  QApplication app(argc, argv);
  Dialog d(argc, argv);
  MainWindow w(&d);
  w.show();
  d.show();

  controller c(&w, &d);
  c.throw_thread();

  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  QObject::connect(&d, SIGNAL(calibDone(bool, cv::Mat, std::vector<cv::Point2f>)), &w, SLOT(canStart(bool, cv::Mat, std::vector<cv::Point2f>)));
  QObject::connect(&d, SIGNAL(reset_buttons()), &w, SLOT(reset_buttons_slot()));
  QObject::connect(&d, SIGNAL(detected_robots(std::map<int,int>)), &w, SLOT(detected_robots_slot(std::map<int,int>)));
  int result = app.exec();

  return result;
}
