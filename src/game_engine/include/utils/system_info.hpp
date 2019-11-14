#ifndef SYSTEM_INFO_HPP
#define SYSTEM_INFO_HPP

#define PI 3.14159265358979323846

#define camera_height           2.15
#define camera_xPos             477.25
#define camera_yPos             366.40

#define robot_height            0.058
#define pi_radius               23/2 * PI
#define max_number_robots       4
#define max_number_joysticks    max_number_robots

#define tag_idA                 0 /*7*/
#define tag_idB                 4 /*1*/
#define tag_idC                 13 /*2*/
#define tag_idD                 15 /*3*/

#define robotA_height           0.054
#define robotB_height           0.0525
#define robotC_height           0.048
#define robotD_height           0.0475

#define wallWidth               10
#define wallAHeight             110
#define wallBHeight             240
#define projectedImageSizeX     984 - wallWidth
#define projectedImageSizeY     760 - wallWidth
#define offset                  wallWidth/2
#define borderOffset_x          70

#define bullet_speed            10
#define bullet_width            10
#define bullet_lenght           15
#define nsecs_between_bullets   800000000

#define explosion_width         10

#define robot_life              99
#define wall_life               5

#define xPosMin                 100
#define yPosMin                 100
#define xPosMax                 700
#define yPosMax                 700

#define classNr_x               35
#define classNr_yE              50
#define classNr_yD              220
#define classNr_yC              390
#define classNr_yB              560
#define classNr_yA              730

#define podium_time             5000000000
#define podium_first_pos_x      492
#define podium_first_pos_y      280
#define podium_second_pos_x     592
#define podium_second_pos_y     320
#define podium_third_pos_x      392
#define podium_third_pos_y      340

#define wareh_wallA             200
#define wareh_wallB             75

#define person_speed            2.5

#define goal_x                  450
#define goal_y_min              0
#define goal_y_max              650
#define goal_y                  325
#define goal_x_min              150
#define goal_x_max              700

#define number_laps             2

#define SPACE                   50

#define SHOOT                   0.1
#define TURBO                   0.2
#define L1                      0.3

#define LINEAR_SPEED            0.082
//#define ANGULAR_SPEED           2.734
#define ANGULAR_SPEED           1

/*! \mainpage 3 PI robot game engine
 *  \section Introduction
 *  This is an introduction
 *  \section Game setup
 *  This is the game installation
 */

#endif // SYSTEM_INFO_HPP
