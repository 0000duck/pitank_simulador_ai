#include "../include/joystick/joystick.hpp"

bool shutdown_flag = false;

int main(int argc, char **argv) {
  string s;
  //s.append("sudo chmod -R 777 /dev/bus/usb/");
  s.append("gksu \"chmod -R 777 /dev/bus/usb/\"");
  int suc = system(s.c_str());
  if(suc < 0)
    return 1;

  ros::init(argc, argv, "joystick");
  NodeHandle = new ros::NodeHandle("~");
  CmdVel_pub[0] = NodeHandle->advertise<geometry_msgs::Twist>("/vel1", 1);
  CmdVel_pub[1] = NodeHandle->advertise<geometry_msgs::Twist>("/vel2", 1);
  CmdVel_pub[2] = NodeHandle->advertise<geometry_msgs::Twist>("/vel3", 1);
  CmdVel_pub[3] = NodeHandle->advertise<geometry_msgs::Twist>("/vel4", 1);
  shoot_pub[0] = NodeHandle->advertise<game_engine::ShootAndTurbo>("/robot1/shootandturbo", 1);
  shoot_pub[1] = NodeHandle->advertise<game_engine::ShootAndTurbo>("/robot1/shootandturbo", 1);
  shoot_pub[2] = NodeHandle->advertise<game_engine::ShootAndTurbo>("/robot1/shootandturbo", 1);
  shoot_pub[3] = NodeHandle->advertise<game_engine::ShootAndTurbo>("/robot1/shootandturbo", 1);

  libusb_device **devs;
  ssize_t cnt;

  int r = libusb_init(&ctx);
  libusb_set_debug(ctx, 3);

  if(r < 0) {
    cout << "Init Error. [ERROR]" << r << endl;
    return 1;
  }

  libusb_set_debug(ctx, 3);
  cnt = libusb_get_device_list(ctx, &devs);

  if(cnt < 0) {
    cout << "> Get Device Error. [ERROR]" << endl;		// There was an error.
    return 1;
  }

  ssize_t i = 0, joy_number = 0;
  joy = new joystickHandler[number_joysticks];

  while(devs[i] != NULL) {
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(devs[i], &desc);
    if (r < 0) {
      cout << "> Failed to get device descriptor. [ERROR]" << endl;
      return 1;
    }

    if((desc.idVendor == VENDOR_A || desc.idVendor == VENDOR_B) && (desc.idProduct == PRODUCT_A || desc.idProduct == PRODUCT_B)) {
      joy[joy_number].dev = devs[i];
      joy[joy_number].handle = NULL;
      joy[joy_number].product_id = desc.idProduct;
      joy[joy_number].vendor_id = desc.idVendor;

      pthread_create(&joystick[joy_number], NULL, joyHandlers[joy_number], &joy[joy_number]);
      pthread_detach(joystick[joy_number]);
      joy_number++;
    }
    i++;
  }

  string shutdown;
  while(1) {
    getline(cin, shutdown);
    if(shutdown.find("shutdown") != string::npos) {
      shutdown_flag = true;
      break;
    }
  }

  libusb_free_device_list(devs, 1);
}

/***** JOYSTICK A *****/

void* controller::joystickA(void *arg) {
  joystickHandler *data = (struct joystickHandler*)arg;
  ros::Rate loop_rate(50);

  int usb_open, result, r, n, received;
  unsigned char *buffer;

  usb_open = libusb_open(data->dev, &(data->handle));
  if(data->handle == NULL || usb_open != 0){
    cout << "> Cannot open device. [ERROR]" << endl;
    return NULL;
  }
  cout << "> Device A openned" << endl;

  if(libusb_kernel_driver_active(data->handle, 0) == 1) {
    cout << "> Kernel Driver Active. [WARNING]" << endl;
    result = libusb_detach_kernel_driver(data->handle, 0);
    if(result != 0){
      cout << "Failed to detach driver. [ERROR]" << endl;
      return NULL;
    }
    cout << "> Kernel Driver Detached. [CHECK]" << endl;
  }
  else
    cout << "> Kernel Driver Not Active. [CHECK]" << endl;

  r = libusb_claim_interface(data->handle, 0);
  if(r != 0) {
    cout << "> Cannot claim interface" << endl;
    return NULL;
  }

  n = 8+1;
  buffer = (unsigned char *)calloc(n,1);
  received = 0;

  while(1) {

    if(shutdown_flag == true)
      break;

    CmdVel[0].linear.x 	= 0;
    CmdVel[0].angular.z	= 0;

    r = libusb_claim_interface(data->handle, 0);

    if(r != 0) {
        cout <<"> Cannot Claim Interface. [ERROR]" << endl;
        return NULL;
    }

    r = libusb_bulk_transfer(data->handle, 0x81, buffer, n-1, &received, 1000);

    switch(r) {
      case LIBUSB_ERROR_TIMEOUT: cout <<"> TIMEOUT ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_PIPE: cout <<"> PIPE ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_OVERFLOW: cout <<"> OVERFLOW ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_NO_DEVICE: cout <<"> NO_DEVICE FOUND. [ERROR]"<<endl;break;
      case 0:
      buffer[received] = '\0';

      if(data->product_id == PRODUCT_A && data->vendor_id == VENDOR_A) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[0].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[0].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[0].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[0].angular.z	= -buffer[0]+128;
        }

        if(buffer[6] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          //CmdVel[0].angular.z = SHOOT;
          shoot[0].shoot = true;
        }
        else shoot[0].shoot = false;

        if(buffer[6] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[0].angular.z = TURBO;
        }

        if(buffer[6] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[0].angular.z = L1;
        }
      }
      else if(data->product_id == PRODUCT_B && data->vendor_id == VENDOR_B) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[0].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[0].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[0].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Right Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[0].angular.z	= -buffer[0]+128;
        }

        if(buffer[5] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          CmdVel[0].angular.z = SHOOT;
        }

        if(buffer[5] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[0].angular.z = TURBO;
        }

        if(buffer[5] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[0].angular.z = L1;
        }
      }
      CmdVel_pub[0].publish(CmdVel[0]);
      ros::spinOnce();
      loop_rate.sleep();
      break;
    }

    r = libusb_release_interface(data->handle, 0);
    if(r != 0) {
      cout << "> Cannot claim interface" << endl;
      return NULL;
    }
  }

  libusb_close(data->handle);
  return NULL;
}

/***** JOYSTICK B *****/

void* controller::joystickB(void *arg) {
  joystickHandler *data = (struct joystickHandler*)arg;
  ros::Rate loop_rate(50);

  int usb_open, result, r, n, received;
  unsigned char *buffer;

  usb_open = libusb_open(data->dev, &(data->handle));
  if(data->handle == NULL || usb_open != 0){
    cout << "> Cannot open device. [ERROR]" << endl;
    return NULL;
  }
  cout << "> Device B openned" << endl;

  if(libusb_kernel_driver_active(data->handle, 0) == 1) {
    cout << "> Kernel Driver Active. [WARNING]" << endl;
    result = libusb_detach_kernel_driver(data->handle, 0);
    if(result != 0){
      cout << "Failed to detach driver. [ERROR]" << endl;
      return NULL;
    }
    cout << "> Kernel Driver Detached. [CHECK]" << endl;
  }
  else
    cout << "> Kernel Driver Not Active. [CHECK]" << endl;

  r = libusb_claim_interface(data->handle, 0);
  if(r != 0) {
    cout << "> Cannot claim interface" << endl;
    return NULL;
  }

  n = 8+1;
  buffer = (unsigned char *)calloc(n,1);
  received = 0;

  while(1) {

    if(shutdown_flag == true)
      break;

    CmdVel[1].linear.x 	= 0;
    CmdVel[1].angular.z	= 0;

    r = libusb_claim_interface(data->handle, 0);

    if(r != 0) {
        cout <<"> Cannot Claim Interface. [ERROR]" << endl;
        return NULL;
    }

    r = libusb_bulk_transfer(data->handle, 0x81, buffer, n-1, &received, 1000);
    switch(r) {
      case LIBUSB_ERROR_TIMEOUT: cout <<"> TIMEOUT ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_PIPE: cout <<"> PIPE ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_OVERFLOW: cout <<"> OVERFLOW ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_NO_DEVICE: cout <<"> NO_DEVICE FOUND. [ERROR]"<<endl;break;
      case 0:
      buffer[received] = '\0';

      if(data->product_id == PRODUCT_A && data->vendor_id == VENDOR_A) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[1].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[1].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[1].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[1].angular.z	= -buffer[0]+128;
        }

        if(buffer[6] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          //CmdVel[0].angular.z = SHOOT;
          shoot[1].shoot = true;
        }
        else shoot[1].shoot = false;

        if(buffer[6] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[1].angular.z = TURBO;
        }

        if(buffer[6] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[1].angular.z = L1;
        }
      }
      else if(data->product_id == PRODUCT_B && data->vendor_id == VENDOR_B) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[1].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[1].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[1].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Right Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[1].angular.z	= -buffer[0]+128;
        }

        if(buffer[5] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          CmdVel[1].angular.z = SHOOT;
        }

        if(buffer[5] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[1].angular.z = TURBO;
        }

        if(buffer[5] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[1].angular.z = L1;
        }
      }

      CmdVel_pub[1].publish(CmdVel[1]);
      ros::spinOnce();
      loop_rate.sleep();
      break;
    }

    r = libusb_release_interface(data->handle, 0);
    if(r != 0) {
      cout << "> Cannot claim interface" << endl;
      return NULL;
    }
  }

  libusb_close(data->handle);
  return NULL;
}

/***** JOYSTICK C *****/

void* controller::joystickC(void *arg) {
  joystickHandler *data = (struct joystickHandler*)arg;
  ros::Rate loop_rate(50);

  int usb_open, result, r, n, received;
  unsigned char *buffer;

  usb_open = libusb_open(data->dev, &(data->handle));
  if(data->handle == NULL || usb_open != 0){
    cout << "> Cannot open device. [ERROR]" << endl;
    return NULL;
  }
  cout << "> Device C openned" << endl;

  if(libusb_kernel_driver_active(data->handle, 0) == 1) {
    cout << "> Kernel Driver Active. [WARNING]" << endl;
    result = libusb_detach_kernel_driver(data->handle, 0);
    if(result != 0){
      cout << "Failed to detach driver. [ERROR]" << endl;
      return NULL;
    }
    cout << "> Kernel Driver Detached. [CHECK]" << endl;
  }
  else
    cout << "> Kernel Driver Not Active. [CHECK]" << endl;

  r = libusb_claim_interface(data->handle, 0);
  if(r != 0) {
    cout << "> Cannot claim interface" << endl;
    return NULL;
  }

  n = 8+1;
  buffer = (unsigned char *)calloc(n,1);
  received = 0;

  while(1) {

    if(shutdown_flag == true)
      break;

    CmdVel[2].linear.x 	= 0;
    CmdVel[2].angular.z	= 0;

    r = libusb_claim_interface(data->handle, 0);

    if(r != 0) {
        cout <<"> Cannot Claim Interface. [ERROR]" << endl;
        return NULL;
    }

    r = libusb_bulk_transfer(data->handle, 0x81, buffer, n-1, &received, 1000);
    switch(r) {
      case LIBUSB_ERROR_TIMEOUT: cout <<"> TIMEOUT ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_PIPE: cout <<"> PIPE ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_OVERFLOW: cout <<"> OVERFLOW ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_NO_DEVICE: cout <<"> NO_DEVICE FOUND. [ERROR]"<<endl;break;
      case 0:
      buffer[received] = '\0';

      if(data->product_id == PRODUCT_A && data->vendor_id == VENDOR_A) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[2].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[2].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[2].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[2].angular.z	= -buffer[0]+128;
        }

        if(buffer[6] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          //CmdVel[0].angular.z = SHOOT;
          shoot[2].shoot = true;
        }
        else shoot[2].shoot = false;

        if(buffer[6] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[2].angular.z = TURBO;
        }

        if(buffer[6] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[2].angular.z = L1;
        }
      }
      else if(data->product_id == PRODUCT_B && data->vendor_id == VENDOR_B) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[2].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[2].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[2].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Right Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[2].angular.z	= -buffer[0]+128;
        }

        if(buffer[5] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          CmdVel[2].angular.z = SHOOT;
        }

        if(buffer[5] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[2].angular.z = TURBO;
        }

        if(buffer[5] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[2].angular.z = L1;
        }
      }

      CmdVel_pub[2].publish(CmdVel[2]);
      ros::spinOnce();
      loop_rate.sleep();
      break;
    }

    r = libusb_release_interface(data->handle, 0);
    if(r != 0) {
      cout << "> Cannot claim interface" << endl;
      return NULL;
    }
  }

  libusb_close(data->handle);
  return NULL;
}

/***** JOYSTICK D *****/

void* controller::joystickD(void *arg) {
  joystickHandler *data = (struct joystickHandler*)arg;
  ros::Rate loop_rate(50);

  int usb_open, result, r, n, received;
  unsigned char *buffer;

  usb_open = libusb_open(data->dev, &(data->handle));
  if(data->handle == NULL || usb_open != 0){
    cout << "> Cannot open device. [ERROR]" << endl;
    return NULL;
  }
  cout << "> Device D openned" << endl;

  if(libusb_kernel_driver_active(data->handle, 0) == 1) {
    cout << "> Kernel Driver Active. [WARNING]" << endl;
    result = libusb_detach_kernel_driver(data->handle, 0);
    if(result != 0){
      cout << "Failed to detach driver. [ERROR]" << endl;
      return NULL;
    }
    cout << "> Kernel Driver Detached. [CHECK]" << endl;
  }
  else
    cout << "> Kernel Driver Not Active. [CHECK]" << endl;

  r = libusb_claim_interface(data->handle, 0);
  if(r != 0) {
    cout << "> Cannot claim interface" << endl;
    return NULL;
  }

  n = 8+1;
  buffer = (unsigned char *)calloc(n,1);
  received = 0;

  while(1) {

    if(shutdown_flag == true)
      break;

    CmdVel[3].linear.x 	= 0;
    CmdVel[3].angular.z	= 0;

    r = libusb_claim_interface(data->handle, 0);

    if(r != 0) {
        cout <<"> Cannot Claim Interface. [ERROR]" << endl;
        return NULL;
    }

    r = libusb_bulk_transfer(data->handle, 0x81, buffer, n-1, &received, 1000);
    switch(r) {
      case LIBUSB_ERROR_TIMEOUT: cout <<"> TIMEOUT ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_PIPE: cout <<"> PIPE ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_OVERFLOW: cout <<"> OVERFLOW ERROR. [ERROR]"<<endl;break;
      case LIBUSB_ERROR_NO_DEVICE: cout <<"> NO_DEVICE FOUND. [ERROR]"<<endl;break;
      case 0:
      buffer[received] = '\0';

      if(data->product_id == PRODUCT_A && data->vendor_id == VENDOR_A) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[3].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[3].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[3].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[3].angular.z	= -buffer[0]+128;
        }

        if(buffer[6] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          //CmdVel[0].angular.z = SHOOT;
          shoot[3].shoot = true;
        }
        else shoot[3].shoot = false;

        if(buffer[6] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[3].angular.z = TURBO;
        }

        if(buffer[6] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[3].angular.z = L1;
        }
      }
      else if(data->product_id == PRODUCT_B && data->vendor_id == VENDOR_B) {
        if((buffer[1] >= 0) && (buffer[1] < 127)){
          //cout << "** Up - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[1]<<" ) **" << endl;
          CmdVel[3].linear.x 	= 128-buffer[1];
        }
        else if(buffer[1] > 128){
          //cout << "** Down - Left Analog **"	<< endl;
          //cout << "** ( "<<-buffer[1]+128<<" ) **"	<< endl;
          CmdVel[3].linear.x 	= -buffer[1]+128;
        }

        if(buffer[0] >=0 && buffer[0] < 127){
          //cout << "** Left - Left Analog **"	<< endl;
          //cout << "** ( "<<128-buffer[0]<<" ) **" << endl;
          CmdVel[3].angular.z 	= 128-buffer[0];
        }
        else if(buffer[0] > 128){
          //cout << "** Right - Right Analog **"	<< endl;
          //cout << "** ( "<<-buffer[0]+128<<" ) **"	<< endl;
          CmdVel[3].angular.z	= -buffer[0]+128;
        }

        if(buffer[5] == 2){
          //cout << "** SHOOT - R1 **"<< endl;
          CmdVel[3].angular.z = SHOOT;
        }

        if(buffer[5] == 8){
          //cout << "** TURBO - R2 **"<< endl;
          CmdVel[3].angular.z = TURBO;
        }

        if(buffer[5] == 1){
          //cout << "** L1 **"<< endl;
          CmdVel[3].angular.z = L1;
        }
      }

      CmdVel_pub[3].publish(CmdVel[3]);
      ros::spinOnce();
      loop_rate.sleep();
      break;
    }

    r = libusb_release_interface(data->handle, 0);
    if(r != 0) {
      cout << "> Cannot claim interface" << endl;
      return NULL;
    }
  }

  libusb_close(data->handle);
  return NULL;
}

controller::~controller() {
  libusb_exit(ctx);
}
