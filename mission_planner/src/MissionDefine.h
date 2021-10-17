
#ifndef MissionDefine_H
#define MissionDefine_H

std_msgs::Float32MultiArray  GoalAction;
std_msgs::Float32MultiArray  GoalPoint;
std_msgs::Float32MultiArray  Mission;
std_msgs::UInt16             Servo;
geometry_msgs::PoseStamped   GoalPose;
// Camera Define (GAZEBO)
/*
#define  CAMERA_PARAM_U0   453.186
#define  CAMERA_PARAM_a    216.507935

#define  CAMERA_PARAM_V0   377.357
#define  CAMERA_PARAM_b    200.712952

#define  a0               -477.419872
#define  a1                  0.0
#define  a2                  6.319908e-4
#define  a3                  4.473616e-7
#define  a4                 -3.29653e-10
*/

// Camera Define
#define  CAMERA_PARAM_U0   320.186
#define  CAMERA_PARAM_a    216.507935

#define  CAMERA_PARAM_V0   240.357
#define  CAMERA_PARAM_b    200.712952

#define  a0               -477.419872
#define  a1                  0.0
#define  a2                  6.319908e-4
#define  a3                  4.473616e-7
#define  a4                 -3.29653e-10

#define  refangle          -90*D2R

int flag_input = 0;
int mission_count = 0;

float u_diff;
float target_y;

// Geolocation
float phi,theta,psi;
float rho, F, s, h_ref;
float u, v, x_tar, y_tar;

float R11,R12,R13;
float R21,R22,R23;
float R31,R32,R33;

struct mission_pose
{
    double x   = 0;
    double y   = 0;
    double z   = 0;
    double yaw = 0;
};

// Mission targets (read from csv instead of ros params)
struct mission_command
{
    double x  = 0;
    double y  = 0;
    double z  = 0;
    double r  = 0;
    double vx = 0;
    double vz = 0;
    int mode       = 4;
    int TAKEOFF    = 1;
    int LAND       = 2;
    int WAYPOINT   = 3;
    int PATHFLIGHT = 4;
    int HOVER      = 5;
};

// Target_Tracking Structure
struct struct_Tar_data
{
    float   pos[3];
    float   posfil[2];
    float   vel[3];
    float   impos[2];
    float   imposfil[2];
    float   dist;
    uint8_t flag_detect;
};

struct struct_flag
{
    int mission           = 0;
    int PHASE_Takeoff     = 0;
    int PHASE_Wall_Pass   = 0;
    int PHASE_Window_Pass = 1;
    int PHASE_Poll_Pass   = 0;
    int PHASE_Pipe_Pass   = 0;
    int PHASE_Tree_Pass   = 0;
    int PHASE_Net_Pass    = 0;
    int PHASE_Wind_Pass   = 0;
    int PHASE_Target_Landing = 0;
};

struct struct_detection
{
    int flag = 0;
    float size_x = 10.0;
    float size_y = 10.0;
    float pos_x = 320;
    float pos_y = 240;
};

struct struct_flag      flag;
struct struct_detection detection;
struct struct_Tar_data  tar_data;

#endif
