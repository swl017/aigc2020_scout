
#ifndef DefineList_H
#define DefineList_H

// essential header for ROS-OpenCV operation
#include <ros/ros.h>

// for topic message
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#include <std_msgs/UInt16.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>


// math param define

#define R2D	      180.0/3.141592653
#define D2R	      3.141592653/180.0
#define eps           0.00000001
#define PI            3.1415926535

#define BUFF_SIZE     4096

#define PRINT_HZ      20.0

ros::Publisher  pub_goalaction;
ros::Publisher  pub_goalpoint;
ros::Publisher  pub_goalpoint_;
ros::Publisher  pub_mission;
ros::Publisher  pub_servo;
ros::Publisher  pub_max_idx;
ros::Subscriber sub_local_pos;
ros::Subscriber sub_local_vel;
ros::Subscriber sub_detection;
ros::Subscriber sub_scan;
ros::Subscriber sub_scan_filter;
ros::Subscriber sub_sonar;
ros::Subscriber sub_max_idx;

// system variable
void Publish(void);
void Mission_Update(void);

void PHASE_Takeoff(void);
void PHASE_Window_Pass(void);
void PHASE_Window_Pass_alt(void);
void PHASE_Obstacle_Pass(void);
void PHASE_Net_Pass(void);
void PHASE_Target_Pickup(void);
void PHASE_Target_Release(void);
void PHASE_Wind_Pass(void);
void PHASE_Landing(void);

float Cur_Pos_m[3];
float Cur_Vel_mps[3];
float Cur_Att_rad[3];

int count_ros = 0;
float t_cur = 0.0;

// mission parameter
int mission_start = 0;

// window pass parameter
double window_width = 0.0;
double window_height = 0.0;

int camera_width = 0.0;
int camera_height = 0.0;

int fx = 0.0;
int fy = 0.0;
int cx = 0.0;
int cy = 0.0;

double VEL_FACTOR = 1.0;

int PHASE_Takeoff_mission = 0;
int PHASE_Takeoff_next = 0;
double PHASE_Takeoff_x = 0.0;
double PHASE_Takeoff_y = 0.0;
double PHASE_Takeoff_z = 0.0;
double PHASE_Takeoff_r = 0.0;
double PHASE_Takeoff_vx = 0.0;
double PHASE_Takeoff_vz = 0.0;

int PHASE_Wall_Pass_mission = 0;
int PHASE_Wall_Pass_next = 0;
double PHASE_Wall_Pass_x = 0.0;
double PHASE_Wall_Pass_y = 0.0;
double PHASE_Wall_Pass_z = 0.0;
double PHASE_Wall_Pass_r = 0.0;
double PHASE_Wall_Pass_vx = 0.0;
double PHASE_Wall_Pass_vz = 0.0;







int PHASE_Window_Pass_mission = 0;
int PHASE_Window_Pass_next = 0;
double PHASE_Window_Pass_x = 0.0;
double PHASE_Window_Pass_y = 0.0;
double PHASE_Window_Pass_z = 0.0;
double PHASE_Window_Pass_r = 0.0;
double PHASE_Window_Pass_vx = 0.0;
double PHASE_Window_Pass_vz = 0.0;

int PHASE_Pole_Pass_mission = 0;
int PHASE_Pole_Pass_next = 0;
double PHASE_Pole_Pass_x = 0.0;
double PHASE_Pole_Pass_y = 0.0;
double PHASE_Pole_Pass_z = 0.0;
double PHASE_Pole_Pass_r = 0.0;
double PHASE_Pole_Pass_vx = 0.0;
double PHASE_Pole_Pass_vz = 0.0;

int PHASE_Pipe_Pass_mission = 0;
int PHASE_Pipe_Pass_next = 0;
double PHASE_Pipe_Pass_x = 0.0;
double PHASE_Pipe_Pass_y = 0.0;
double PHASE_Pipe_Pass_z = 0.0;
double PHASE_Pipe_Pass_r = 0.0;
double PHASE_Pipe_Pass_vx = 0.0;
double PHASE_Pipe_Pass_vz = 0.0;

int PHASE_Tree_Pass_mission = 0;
int PHASE_Tree_Pass_next = 0;
double PHASE_Tree_Pass_x = 0.0;
double PHASE_Tree_Pass_y = 0.0;
double PHASE_Tree_Pass_z = 0.0;
double PHASE_Tree_Pass_r = 0.0;
double PHASE_Tree_Pass_vx = 0.0;
double PHASE_Tree_Pass_vz = 0.0;

int PHASE_Net_Pass_mission = 0;
int PHASE_Net_Pass_next = 0;
double PHASE_Net_Pass_x = 0.0;
double PHASE_Net_Pass_y = 0.0;
double PHASE_Net_Pass_z = 0.0;
double PHASE_Net_Pass_r = 0.0;
double PHASE_Net_Pass_vx = 0.0;
double PHASE_Net_Pass_vz = 0.0;

int PHASE_Wind_Pass_mission = 0;
int PHASE_Wind_Pass_next = 0;
int PHASE_Wind_Pass_mode = 0;
double PHASE_Wind_Pass_x = 0.0;
double PHASE_Wind_Pass_y = 0.0;
double PHASE_Wind_Pass_z = 0.0;
double PHASE_Wind_Pass_r = 0.0;
double PHASE_Wind_Pass_vx = 0.0;
double PHASE_Wind_Pass_vz = 0.0;
double PHASE_Wind_Pass_box_x = 0.0;
double PHASE_Wind_Pass_box_y = 0.0;

int PHASE_Landing_mission = 0;
double PHASE_Landing_x = 0.0;
double PHASE_Landing_y = 0.0;
double PHASE_Landing_z = 0.0;
double PHASE_Landing_r = 0.0;
double PHASE_Landing_vx = 0.0;
double PHASE_Landing_vz = 0.0;

float lidar_data[10];
float min_dist = 60;
float target_dist = 60;
float right_dist = 60;
float left_dist = 60;
float sonar_dist = 1.0;
float sonar_dist_pre = 1.0;
float sonar_dist_lpf = 1.0;
float last_box_x=0;
float last_box_y=0;
float temp_x = 0.0;
float temp_y = 0.0;
int maxind =0;
float maxang_deg =0.0;
float maxang_rad =0.0;
bool window_search_complete=false;
float box_x = 6.3;
float box_y = -7.78;
float sub_dist = 10.0;
int   count_wind = 0;
int   count_no_box = 100000;
int   count_open = 0;
double PHASE_Wind_Pass_box_alt=0;

float WayPoint_X[100];
float WayPoint_Y[100];
int   WP_num = 5;
int   WP_index = 0;
float WP_dist = 0.0;

float satmax(float data, float max)
{
    float res;

    if(fabs(data) > max)
        res = (data + eps)/fabs(data + eps)*max;
    else
        res = data;

    return res;
}


float satmin(float data, float min)
{
    float res;

    if(fabs(data) < min)
        res = (data + eps)/fabs(data + eps)*min;
    else
        res = data;

    return res;
}


float LPF(float data, float data_pre, float freq)
{
    float res;
    float K_LPF;
    float delT = 0.1;

    K_LPF = freq*2*PI*delT / (1 + freq*2*PI*delT);

    res = (data - data_pre) * K_LPF + data_pre;

    return res;
}


#endif
