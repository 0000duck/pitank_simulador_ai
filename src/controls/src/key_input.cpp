/*#include <termios.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

class BufferToggle
{
    private:
        struct termios t;

    public:

        //Disables buffered input

        void off(void)
        {
            tcgetattr(STDIN_FILENO, &t); //get the current terminal I/O structure
            t.c_lflag &= ~ICANON; //Manipulate the flag bits to do what you want it to do
            //t.c_lflag &= ~ECHO; // turn off ECHO
            tcsetattr(STDIN_FILENO, TCSANOW, &t); //Apply the new settings
        }

         //Enables buffered input

        void on(void)
        {
            tcgetattr(STDIN_FILENO, &t); //get the current terminal I/O structure
            t.c_lflag |= ICANON; //Manipulate the flag bits to do what you want it to do
            tcsetattr(STDIN_FILENO, TCSANOW, &t); //Apply the new settings
        }
};

int main(int argc, char **argv)
{
    BufferToggle bt;
    geometry_msgs::Twist CmdVel[4];
    ros::Publisher CmdVel_pub[4];
    ros::NodeHandle* NodeHandle;
    ros::init(argc, argv, "key_input");
    NodeHandle = new ros::NodeHandle("~");
    CmdVel_pub[0] = NodeHandle->advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 20);
    CmdVel_pub[1] = NodeHandle->advertise<geometry_msgs::Twist>("/robot2/cmd_vel", 20);
    CmdVel_pub[2] = NodeHandle->advertise<geometry_msgs::Twist>("/robot3/cmd_vel", 20);
    CmdVel_pub[3] = NodeHandle->advertise<geometry_msgs::Twist>("/robot4/cmd_vel", 20);
    char c;
    //getchar(); //waits for you to press enter before proceeding to the next instruction
    bt.off();
    while(1){
        c = getchar(); //processes next instruction as soon as you type a character (no enter)
       //std::cout << "Key:" << c;
    //bt.on();
    //getchar(); //waits for you to press enter before proceeding to the next instruction
             if (c=='w') CmdVel[0].linear.x = 1;
	else if (c=='a') CmdVel[0].angular.z = 1;
	else if (c=='s') CmdVel[0].linear.x = -1;
	else if (c=='d') CmdVel[0].angular.z = -1;
	else if (c=='t') CmdVel[1].linear.x = 1;
	else if (c=='f') CmdVel[1].angular.z = 1;
	else if (c=='g') CmdVel[1].linear.x = -1;
	else if (c=='h') CmdVel[1].angular.z = -1;
	else if (c=='i') CmdVel[2].linear.x = 1;
	else if (c=='j') CmdVel[2].angular.z = 1;
	else if (c=='k') CmdVel[2].linear.x = -1;
	else if (c=='l') CmdVel[2].angular.z = -1;
	else if (c=='5') CmdVel[3].linear.x = 1;
	else if (c=='1') CmdVel[3].angular.z = 1;
	else if (c=='2') CmdVel[3].linear.x = -1;
        else if (c=='3') CmdVel[3].angular.z = -1;
        else{
                 CmdVel[0].linear.x = 0;
                 CmdVel[0].angular.z = 0;
                 CmdVel[1].linear.x = 0;
                 CmdVel[1].angular.z = 0;
                 CmdVel[2].linear.x = 0;
                 CmdVel[2].angular.z = 0;
                 CmdVel[3].linear.x = 0;
                 CmdVel[3].angular.z = 0;
        }
	CmdVel_pub[0].publish(CmdVel[0]);
	CmdVel_pub[1].publish(CmdVel[1]);
	CmdVel_pub[2].publish(CmdVel[2]);
	CmdVel_pub[3].publish(CmdVel[3]);

        ros::spinOnce();
    }
}

*/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "game_engine/RobotDescriptionArray.h"
#include "game_engine/UIState.h"
#include "game_engine/ShootAndTurbo.h"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <ctime>
#include <pthread.h>

#include <map>

typedef void * (*THREADFUNCPTR)(void *);

// Map for movement keys
std::map<char, std::vector<float>> moveBindings1
{
    {'w', {1, 0, 0, 0, 0}},
    {'a', {0, 0, 0, 1, 0}},
    {'s', {-1, 0, 0, 0, 0}},
    {'d', {0, 0, 0, -1, 0}},
    {'z', {0, 0, 0, 0, 1}}
};

std::map<char, std::vector<float>> moveBindings2
{
    {'t', {1, 0, 0, 0, 0}},
    {'f', {0, 0, 0, 1, 0}},
    {'g', {-1, 0, 0, 0, 0}},
    {'h', {0, 0, 0, -1, 0}},
    {'v', {0, 0, 0, 0, 1}}
};

std::map<char, std::vector<float>> moveBindings3
{
    {'i', {1, 0, 0, 0}},
    {'j', {0, 0, 0, 1}},
    {'k', {-1, 0, 0, 0}},
    {'l', {0, 0, 0, -1}}
};

std::map<char, std::vector<float>> moveBindings4
{
    {'8', {1, 0, 0, 0}},
    {'4', {0, 0, 0, 1}},
    {'5', {-1, 0, 0, 0}},
    {'6', {0, 0, 0, -1}}
};

// Init variables
float speed(127); // Linear velocity (m/s)
float turn(127); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');
int n = -1;  // Robot being controlled in each iteration
bool getchar_flag = false;
bool autodrive_flag = false;
clock_t timer_sim;

ros::Subscriber robots_sub;
ros::Publisher pub[4],shoot_pub[4];
geometry_msgs::Twist twist;
game_engine::ShootAndTurbo shoot;

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

void publish_vel(const game_engine::RobotDescriptionArray& msg){


    ros::Rate loop_rate(50);
    getchar_flag = true;

    // Get the pressed key
    key = getch();

    getchar_flag = false;

    // If the key corresponds to a key in moveBindings
    if (moveBindings1.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings1[key][0];
      y = moveBindings1[key][1];
      z = moveBindings1[key][2];
      th = moveBindings1[key][3];
      shoot.shoot = moveBindings1[key][4];
      n = 0;
    }
    else if (moveBindings2.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings2[key][0];
      y = moveBindings2[key][1];
      z = moveBindings2[key][2];
      th = moveBindings2[key][3];
      shoot.shoot = moveBindings2[key][4];
      n = 1;
    }
    else if (moveBindings3.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings3[key][0];
      y = moveBindings3[key][1];
      z = moveBindings3[key][2];
      th = moveBindings3[key][3];
      shoot.shoot = moveBindings3[key][4];
      n = 2;
    }
    else if (moveBindings4.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings4[key][0];
      y = moveBindings4[key][1];
      z = moveBindings4[key][2];
      th = moveBindings4[key][3];
      shoot.shoot = moveBindings4[key][4];
      n = 3;
    }

    // Otherwise, set the robot to stop
    else
    {
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      n = -1;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03') ros::shutdown();
    }

    // Update the Twist message
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;

    // Publish it and resolve any remaining callbacks
    if ((n != -1)){
        if(msg.robot[n].collisionFlag == false && msg.robot[n].autonomous_drive == false){
          pub[n].publish(twist);
          //ros::spinOnce();

          if(shoot.shoot) shoot_pub[n].publish(shoot);
        }

    }
    /*else{
      for (int i=0;i<4;i++){
          pub[i].publish(twist);
      }
      ros::spinOnce();
    }*/
    if(msg.robot[0].autonomous_drive == true)
        autodrive_flag = true;
}

void* reset_timer() {
    while(1){
        if(getchar_flag){
            if(((float)clock() - timer_sim)/CLOCKS_PER_SEC > 0.05 && autodrive_flag == false){
                for(int j=0;j<4;j++){
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    pub[j].publish(twist);
                    //ros::spinOnce();
                    timer_sim = clock();
                }
            }
        }
        else{
            timer_sim = clock();
        }
    }
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "key_input");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  pub[0] = nh.advertise<geometry_msgs::Twist>("/vel1", 1);
  pub[1] = nh.advertise<geometry_msgs::Twist>("/vel2", 1);
  pub[2] = nh.advertise<geometry_msgs::Twist>("/vel3", 1);
  pub[3] = nh.advertise<geometry_msgs::Twist>("/vel4", 1);
  shoot_pub[0] = nh.advertise<game_engine::ShootAndTurbo>("/robot1/shootandturbo", 1);
  shoot_pub[1] = nh.advertise<game_engine::ShootAndTurbo>("/robot2/shootandturbo", 1);
  shoot_pub[2] = nh.advertise<game_engine::ShootAndTurbo>("/robot3/shootandturbo", 1);
  shoot_pub[3] = nh.advertise<game_engine::ShootAndTurbo>("/robot4/shootandturbo", 1);

  robots_sub = nh.subscribe("/robots_description", 1, &publish_vel);

  timer_sim = clock();

  /*pthread_t p_thread;
  pthread_create(&p_thread, NULL, (THREADFUNCPTR) &reset_timer, NULL);
  pthread_detach(p_thread);*/

  ros::spin();

  return 0;
}
