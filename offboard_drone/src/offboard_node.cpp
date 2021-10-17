
#include<iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include "sensor_msgs/Range.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "SubUtil.h"

#define VEL_TRACKING   0.6
#define RETURN_TIME  1500.0

ros::Time landing_request;
ros::Time takeoff_request;

using namespace ros;
using namespace std;

mavros_msgs::State             g_current_state;
geometry_msgs::TwistStamped    velcmd;
std_msgs::UInt8                MotorAct_msg;
std_msgs::UInt8                FlightMode_msg;
mavros_msgs::RCIn              rc_in;
geometry_msgs::PoseStamped     Local;
geometry_msgs::TwistStamped    Localvel;
std::string                    robot_namespace;
std_msgs::Float32MultiArray    GoalAction;
std_msgs::Bool land;

void Mission_Update(void);
void Auto_Takeoff(void);
void Auto_Landing(void);
void WP_Flight(void);
void Path_Flight(void);
void Hovering(void);
void Path_Flight_Alt(void);

float    cmd_x = 0.0;
float    cmd_y = 0.0;
float    cmd_z = 0.0;
float    cmd_r = 0.0;

float    cmd_RCx = 0.0;
float    cmd_RCy = 0.0;
float    cmd_RCz = 0.0;
float    cmd_RCr = 0.0;

float    Cur_Att_rad[3];
float    Cur_Pos_m[3];
float    Cur_Vel_mps[3];

float    vel_NED[3];
float    angle_err;

float    takeoff_x = 0.0;
float    takeoff_y = 0.0;
float    init_heading = 0.0;

float    goal_dist = 10.0;
float    goal_heading = 0.0;

float    crusing_height = 0.0;
float    takeoff_height = 0.0;
float    sp_pos_cmd[3];
float    hover[2];
float    hover_heading = 0.0;
float    return_heading = 0.0;
float    t_cur = 0.0;
float    scan_up_minimum=0;

int      flag_takeoff = 0;
int      flag_landing = 0;
int      flag_goal = 0;
int      flag_start = 0;
int      flag_armed = 0;
int      flag_turning = 0;
int      flag_return = 0;

int      turning_dir = 0;

int      goal_service = 0;
int      debug_hardware = 0;
int      emergency_landing = 0;
int      return_home = 0;

float    goal[4];
float    goal_velx;
float    goal_velz;
float    target_yaw = 0;
float    takeoff_yaw = 0;

float    GF_cmd_x;
float    GF_cmd_y;


int      count_ros = 0;
int      WP_num = 0;
int      WP_index = 0;
float    WayPoint_X[200];
float    WayPoint_Y[200];
float    WayPoint_Z[200];
double     cancel=0;

visualization_msgs::Marker TargetList, Target;
ros::Publisher pub_target;
ros::Publisher min_up_pub;
ros::Publisher land_pub;
void publish_target(void);

int    point_ind = 0;

float fq = 20.0;
float vel = 1.0;
float eta = 0.2;

struct Trajectory
{
    float x;
    float y;
    float z;
    float psi;
    float wp_x;
    float wp_y;
}path;

void callback_scan_up(const std_msgs::Float64::ConstPtr& msg){
	scan_up_minimum = msg->data;
	//std::cout<<scan_up_minimum<<std::endl;
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
	float min_up_limit=65.0;
	std_msgs::Float64 dist_data;
	//std::cout<<msg->ranges[msg->ranges.size()-1]<<std::endl;
	for(int idx=0; idx<20; idx++){

		if ((msg->ranges[idx])<min_up_limit){
			min_up_limit = msg->ranges[idx];
		}
		if ((msg->ranges[msg->ranges.size()-1-idx])<min_up_limit){
			min_up_limit = msg->ranges[msg->ranges.size()-1-idx];
		}		
	}
	dist_data.data = min_up_limit;
	min_up_pub.publish(dist_data);

}

void callback_goal(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    GoalAction = *msg;

    goal_service = GoalAction.data[0];

    goal[0] = GoalAction.data[1];
    goal[1] = GoalAction.data[2];
    goal[2] = GoalAction.data[3];
    goal[3] = GoalAction.data[4];

    goal_velx = GoalAction.data[5];
    goal_velz = GoalAction.data[6];
}

void callback_global_path(const std_msgs::Float32MultiArray& msg)
{
    WP_index = 1;
    WP_num = msg.layout.data_offset;

    for (int ind = 0; ind < WP_num; ind++)
    {
        WayPoint_X[ind] = msg.data[3*ind];
        WayPoint_Y[ind] = msg.data[3*ind+1];
        WayPoint_Z[ind] = msg.data[3*ind+2];
        //printf("%.3f, %.3f, %.3f, \n", WayPoint_X[ind], WayPoint_Y[ind], WayPoint_Z[ind]);
    }
}

void cbState(const mavros_msgs::State::ConstPtr& msg)
{
    g_current_state = *msg;

    std::cout << "\n[USRG] state_cb(), -----------";
    std::cout << "\n   connected    = " << ((g_current_state.connected) ? "OK!" : "Not yet!");
    std::cout << "\n   armed        = " << ((g_current_state.armed ) ? "OK!" : "Not yet!");
    std::cout << "\n   guided       = " << ((g_current_state.guided) ? "OK!" : "Not yet!");
    std::cout << "\n   mode         = " << g_current_state.mode;
    std::cout << "\n   yaw  yaw_cmd = " << Cur_Att_rad[2]*180.0/3.141592 << ", " << path.psi*180.0/3.141592;
    std::cout << "\n   Cur   X Y r  = " << Cur_Pos_m[0] << ", "<< Cur_Pos_m[1] << ", "<<  Cur_Att_rad[2]*R2D;
    std::cout << "\n   Path  X Y r  = " << path.x << ", "<< path.y << ", "<<  path.psi*R2D;
    std::cout << "\n   vel command  = " << velcmd.twist.linear.x << ", "<<velcmd.twist.linear.y << ", "<<velcmd.twist.linear.z <<  ", "<<velcmd.twist.angular.z;
    std::cout << "\n   goal service = " << goal_service;
    std::cout << "\n   elapsed time = " << t_cur;
    std::cout << "\n   debug H/W    = " << debug_hardware;
	
    //printf("\ncmd : %.3f, %.3f, %.3f, %.3f  cmd_RCx: %.3f, %.3f, %.3f, %.3f\n", cmd_x, cmd_y, cmd_z, cmd_r, cmd_RCx, cmd_RCy, cmd_RCz, cmd_RCr);
    std::cout << "\n[USRG] ------------------------\n";

/*
	ROS_INFO("pos: %.3f %.3f %.3f %.3f", Cur_Pos_m[0], Cur_Pos_m[1], Cur_Pos_m[2], Cur_Att_rad[2]*R2D);
	ROS_INFO("goal service: %d", goal_service);
	ROS_INFO("mode: %s", g_current_state.mode);	
        */
}

void callback_rc_in(const mavros_msgs::RCIn::ConstPtr& msg_input)
{
    rc_in = *msg_input;

    cmd_RCy =  (rc_in.channels[0] - PWM_ROL)/PWM_LEN*VELX_MAX;
    cmd_RCx = -(rc_in.channels[1] - PWM_PIT)/PWM_LEN*VELX_MAX;
    cmd_RCz =  (rc_in.channels[2] - PWM_THR)/PWM_LEN*VELZ_MAX;
    cmd_RCr =  (rc_in.channels[3] - PWM_YAW)/PWM_LEN*VELR_MAX;

    if (rc_in.channels[7] > 1700)
        debug_hardware = 1;
    else
        debug_hardware = 0;

    if (rc_in.channels[8] > 1700)
        emergency_landing = 1;
    else
        emergency_landing = 0;

    if (rc_in.channels[9] > 1700)
        return_home = 1;
    else
        return_home = 0;
	
    //printf("%.3f %.3f %.3f %.3f\n",cmd_RCx, cmd_RCy, cmd_RCz, cmd_RCr);
}

void callback_GF_cmd(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    float GF_cmd_1 = msg->data[0];
    float GF_cmd_2 = msg->data[1];
    float GF_cmd_3 = msg->data[2];
    float GF_cmd_4 = msg->data[3];
    float GF_cmd_5 = msg->data[4];

    GF_cmd_x = (-1.0)*GF_cmd_3 + (-0.0)*GF_cmd_2*sin(45.0*D2R) + (-0.0)*GF_cmd_4*sin(45.0*D2R);
    GF_cmd_y = (-1.0)*GF_cmd_5 + GF_cmd_1 + GF_cmd_2*sin(45.0*D2R) + (-1.0)*GF_cmd_4*sin(45.0*D2R);
}

void callback_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg_input)
{
    Local = *msg_input;

    Cur_Pos_m[0] =  Local.pose.position.x;
    Cur_Pos_m[1] =  Local.pose.position.y;
    Cur_Pos_m[2] =  Local.pose.position.z;

    q[0] = Local.pose.orientation.x;
    q[1] = Local.pose.orientation.y;
    q[2] = Local.pose.orientation.z;
    q[3] = Local.pose.orientation.w;
    QuaterniontoEuler(Cur_Att_rad[0], Cur_Att_rad[1], Cur_Att_rad[2]);
    //printf("%.3f\n",Cur_Att_rad[2]);
}

void callback_local_vel(const geometry_msgs::TwistStamped::ConstPtr& msg_input)
{
    Localvel = *msg_input;

    Cur_Vel_mps[0] = Localvel.twist.linear.x;
    Cur_Vel_mps[1] = Localvel.twist.linear.y;
    Cur_Vel_mps[2] = Localvel.twist.linear.z;
}

void astar(const std_msgs::Float32MultiArray& astar_path_msg)
{
    path.x = astar_path_msg.data[0]; //longitudinal
    path.y = astar_path_msg.data[1]; //lateral
    path.z = astar_path_msg.data[2];
    //path.psi = astar_path_msg.data[2]; //Desired PSI angle
    path.wp_x = astar_path_msg.data[3];
    cancel = astar_path_msg.data[4];

    target_yaw = atan2(goal[1] - Cur_Pos_m[1], goal[0] - Cur_Pos_m[0]);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    robot_namespace= ros::this_node::getNamespace();
    robot_namespace.erase(0, 2);
    ros::NodeHandle nh;

    ros::Subscriber     state_sub        = nh.subscribe<mavros_msgs::State>         ("/scout/mavros/state" , 10, cbState);
    ros::Subscriber     rc_in_sub        = nh.subscribe<mavros_msgs::RCIn>          ("/scout/mavros/rc/in", 5, callback_rc_in);
    ros::Subscriber     local_pos_sub    = nh.subscribe<geometry_msgs::PoseStamped> ("/scout/mavros/local_position/pose", 10, callback_local_pos);
    ros::Subscriber     local_vel_sub    = nh.subscribe<geometry_msgs::TwistStamped>("/scout/mavros/local_position/velocity_local", 10, callback_local_vel);
    ros::Subscriber     path_sub_        = nh.subscribe("/scout/astar_path", 1, &astar);
    ros::Subscriber     goal_sub         = nh.subscribe ("/scout/GoalAction", 2,     &callback_goal);
    ros::Subscriber     scan_sub         = nh.subscribe ("/scout/scan", 1,     &callback_scan);
    ros::Subscriber     geofence_sub     = nh.subscribe("/scout/geofence", 1, &callback_GF_cmd);
    ros::Subscriber     global_path_sub  = nh.subscribe ("/global_wp", 2,                         &callback_global_path);
    ros::Subscriber     min_up_sub       = nh.subscribe ("/scout/scan_up_min", 1,                         &callback_scan_up);

    // Publish Topic
    ros::Publisher      local_vel_pub    = nh.advertise<geometry_msgs::TwistStamped>("/scout/mavros/setpoint_velocity/cmd_vel", 10);
    min_up_pub                           = nh.advertise<std_msgs::Float64>("/scout/scan_up_min", 1);
    pub_target                           = nh.advertise<visualization_msgs::Marker>   ("/current_waypoint", 1);
    land_pub = nh.advertise<std_msgs::Bool>("/is_landing", 1);

    ros::ServiceClient  arming_client    = nh.serviceClient<mavros_msgs::CommandBool>("/scout/mavros/cmd/arming");
    ros::ServiceClient  set_mode_client  = nh.serviceClient<mavros_msgs::SetMode>    ("/scout/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && g_current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    velcmd.twist.linear.x = 0.0;
    velcmd.twist.linear.y = 0.0;
    velcmd.twist.linear.z = 0.0;
    velcmd.twist.angular.z = 0.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_vel_pub.publish(velcmd);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode LAND_set_mode;
    LAND_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandLong disarm_cmd;
    arm_cmd.request.value = true;
    disarm_cmd.request.command = 400;
    disarm_cmd.request.param2 = 21196;

    ros::Time last_request = ros::Time::now();
    ros::Time global_request = ros::Time::now();

    //goal[0] = 10.0;
    //goal[1] = -3.5;

    while(ros::ok())
    {
        /*
	if (g_current_state.mode == "OFFBOARD")
	{
        	t_cur = ros::Time::now().toSec() - global_request.toSec();
        	if ((t_cur > RETURN_TIME) && (t_cur <RETURN_TIME+1.0))
       		{
            		flag_return = 1;
			return_heading = hover_heading + 180.0*D2R; 
       		}
	}
	else
	{
		global_request = ros::Time::now();
	}
        */

        if( g_current_state.mode == "OFFBOARD")
        {
            if (goal_service == 0)
            {
                flag_start = 0;
                takeoff_request = ros::Time::now();
            }
            else
            {
                flag_start = 1;
            }
        }

        
	// if (debug_hardware == 0)
	// {
		if (flag_start == 1)
		{
			if(flag_armed == 0)
			{
				if((!g_current_state.armed) && (ros::Time::now() - last_request > ros::Duration(2.0)))
				{
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        flag_armed = 1;
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
				}
			}
		}
		else
		{
			if (flag_armed == 1)
			{
				ROS_INFO("Vehicle Disarmed");
                if (set_mode_client.call(LAND_set_mode) && LAND_set_mode.response.mode_sent)
				// if (arming_client.call(disarm_cmd)&& disarm_cmd.response.success)
				{
                    flag_armed = 0;
                    last_request = ros::Time::now();
				}
				//mission.data = 0;  // [None]
				last_request = ros::Time::now();
			}
		}
	// }
        



	if( g_current_state.mode == "OFFBOARD")
	{	
	
	    Mission_Update();

            velcmd.twist.linear.x =   cmd_x + (cmd_RCx + GF_cmd_x)*cos(Cur_Att_rad[2]) + (cmd_RCy - GF_cmd_y)*sin(Cur_Att_rad[2]);
            velcmd.twist.linear.y =   cmd_y + (cmd_RCx + GF_cmd_x)*sin(Cur_Att_rad[2]) - (cmd_RCy - GF_cmd_y)*cos(Cur_Att_rad[2]);
            velcmd.twist.linear.z =   cmd_z;
            velcmd.twist.angular.z = -(cmd_r + cmd_RCr);

            count_ros = count_ros + 1;
	}
	else
	{
            cmd_x = 0.0;
            cmd_y = 0.0;
            cmd_z = 0.0;
            cmd_r = 0.0;

            velcmd.twist.linear.x = 0.0;
            velcmd.twist.linear.y = 0.0;
            velcmd.twist.linear.z = 0.0;
            velcmd.twist.angular.z = 0.0;

	    takeoff_x = Cur_Pos_m[0];
	    takeoff_y = Cur_Pos_m[1];
	    init_heading = Cur_Att_rad[2];
	    goal_heading = Cur_Att_rad[2];

	    goal_dist = 10.0;

	    last_request = ros::Time::now();
	}
	
        local_vel_pub.publish(velcmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void Mission_Update(void)
{

    switch (goal_service)
    {
    // Basic Mission
        case 1:
            Auto_Takeoff();
            break;

        case 2:
            Auto_Landing();
            break;

        case 3:
            WP_Flight();
            break;

        case 4:
            Path_Flight();
            break;

        case 5:
            Hovering();
            break;

        case 6:
            Path_Flight_Alt();
            break;
        default:
            cmd_x = 0.0;
            cmd_y = 0.0;
            cmd_z = 0.0;
            cmd_r = 0.0;

            velcmd.twist.linear.x = 0.0;
            velcmd.twist.linear.y = 0.0;
            velcmd.twist.linear.z = 0.0;
            velcmd.twist.angular.z = 0.0;

            takeoff_x = Cur_Pos_m[0];
            takeoff_y = Cur_Pos_m[1];
            init_heading = Cur_Att_rad[2];
            goal_heading = Cur_Att_rad[2];

            goal_dist = 10.0;

            break;
    }
}


void Auto_Takeoff(void)
{
    cmd_x = satmax(Kpx*(takeoff_x - Cur_Pos_m[0]), goal_velx);
    cmd_y = satmax(Kpx*(takeoff_y - Cur_Pos_m[1]), goal_velx);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]), goal_velz);

    angle_err = GetNED_angle_err(init_heading, Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);

    if (fabs(goal[2] - Cur_Pos_m[2]) < 0.1)
    {
         goal_service = 5;
         hover[0] = Cur_Pos_m[0];
         hover[1] = Cur_Pos_m[1];
         hover[2] = goal[2];
         hover_heading = Cur_Att_rad[2];
    }
}


void Auto_Landing(void)
{
    cmd_x = satmax(Kpx*(goal[0] - Cur_Pos_m[0]),goal_velx);
    cmd_y = satmax(Kpx*(goal[1] - Cur_Pos_m[1]),goal_velx);
    cmd_z = goal_velz;

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = 0;
    takeoff_x = Cur_Pos_m[0];
    takeoff_y = Cur_Pos_m[1];
    land.data = true;
    land_pub.publish(land);
    // if (flag_landing != 1)
    // {
    //     landing_request = ros::Time::now();
    // }

    // if (Cur_Pos_m[2] < 0.2)
    // {
    //     cmd_z = -0.5;
    //     flag_landing = 1;
    //     if (ros::Time::now() - landing_request > ros::Duration(1.0))
    //     {
    //         flag_takeoff = 0;
    //         takeoff_x = Cur_Pos_m[0];
    //         takeoff_y = Cur_Pos_m[1];
    //         flag_landing = 0;
    //         goal_service = 0;
    //         takeoff_request = ros::Time::now();
    //     }
    // }
}

void WP_Flight(void)
{
    float dist = sqrt((path.x-Cur_Pos_m[0])*(path.x-Cur_Pos_m[0]) + (path.y-Cur_Pos_m[1])*(path.y-Cur_Pos_m[1]));
    if (dist > 0.2)
    {
        path.psi = atan2(path.y-Cur_Pos_m[1], path.x-Cur_Pos_m[0]);
    }

    cmd_x = satmax(Kpx*(goal[0] - Cur_Pos_m[0]),goal_velx);
    cmd_y = satmax(Kpx*(goal[1] - Cur_Pos_m[1]),goal_velx);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]),goal_velz) + Kdz*(0.0 - Cur_Vel_mps[2]);

    // angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    //angle_err = GetNED_angle_err(path.psi, Cur_Att_rad[2]);
    angle_err = GetNED_angle_err(target_yaw + goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);

    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = goal[2];
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];

}

void Path_Flight(void)
{

    //float dist = sqrt((path.x - Cur_Pos_m[0])*(path.x - Cur_Pos_m[0])+(path.y - Cur_Pos_m[1])*(path.y - Cur_Pos_m[1]));
    //float velcmd = satmax(Kpx*dist,goal_velx);
    //float psi = atan2(path.y - Cur_Pos_m[1], path.x - Cur_Pos_m[0]);
    /*
    cmd_x = goal_velx * cos(psi);
    cmd_y = goal_velx * sin(psi);
    //cmd_x = velcmd * cos(psi);
    //cmd_y = velcmd * sin(psi);
    */
    /*
    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);

    float velcmd = goal_velx;
    if (angle_err > 30.0*D2R)
        velcmd = goal_velx*0.1;
    else
        velcmd = goal_velx;

    cmd_x = satmax(0.0*velcmd * cos(psi) + Kpx*(path.x - Cur_Pos_m[0])*1.5,velcmd*3.0);
    cmd_y = satmax(0.0*velcmd * sin(psi) + Kpx*(path.y - Cur_Pos_m[1])*1.5,velcmd*3.0);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]), goal_velz);

    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = goal[2];
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];
    */
    flag_landing = 0;
    float dist = sqrt((path.x-Cur_Pos_m[0])*(path.x-Cur_Pos_m[0]) + (path.y-Cur_Pos_m[1])*(path.y-Cur_Pos_m[1]));
    if (dist > 0.2)
    {
        path.psi = atan2(path.y-Cur_Pos_m[1], path.x-Cur_Pos_m[0]);
    }
    path.psi = atan2(path.y-Cur_Pos_m[1], path.x-Cur_Pos_m[0]);
    // angle_err = GetNED_angle_err(path.psi, Cur_Att_rad[2]);
    // angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    angle_err = GetNED_angle_err(target_yaw + goal[3], Cur_Att_rad[2]);
    // angle_err = GetNED_angle_err(target_yaw, Cur_Att_rad[2]);

    float velcmd = goal_velx;
    if (fabs(angle_err) > 10.0*D2R)
        velcmd = goal_velx*0.5;
    else
        velcmd = goal_velx;

    cmd_x = velcmd * cos(path.psi);
    cmd_y = velcmd * sin(path.psi);
    //cmd_x = satmax(Kpx*(path.x - Cur_Pos_m[0]),goal_velx);
    //cmd_y = satmax(Kpx*(path.y - Cur_Pos_m[1]),goal_velx);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]), goal_velz);
//    cmd_z = satmax(Kpz*(path.z- Cur_Pos_m[2]), goal_velz) + Kdz*(0.0 - Cur_Vel_mps[2]);

    //angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    //angle_err = GetNED_angle_err(path.psi, Cur_Att_rad[2]);
    //if (cancel <= 0.5){
    //  cmd_r = -satmax(Kr*angle_err, R_MAX);
    //}
    //else{
    //  cmd_r = 0;
    //}
    cmd_r = -satmax(Kr*angle_err, R_MAX);
    takeoff_x = Cur_Pos_m[0]; // Why update?
    takeoff_y = Cur_Pos_m[1]; // Why update?
    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = goal[2];
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];

}

void Hovering(void)
{
    cmd_x = satmax(Kpx*(hover[0] - Cur_Pos_m[0]),goal_velx);
    cmd_y = satmax(Kpx*(hover[1] - Cur_Pos_m[1]),goal_velx);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]),goal_velz);

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);
    goal_heading = Cur_Att_rad[2];
}


void Path_Flight_Alt(void)
{

    //float dist = sqrt((path.x - Cur_Pos_m[0])*(path.x - Cur_Pos_m[0])+(path.y - Cur_Pos_m[1])*(path.y - Cur_Pos_m[1]));
    //float velcmd = satmax(Kpx*dist,goal_velx);
    //float psi = atan2(path.y - Cur_Pos_m[1], path.x - Cur_Pos_m[0]);
    /*
    cmd_x = goal_velx * cos(psi);
    cmd_y = goal_velx * sin(psi);
    //cmd_x = velcmd * cos(psi);
    //cmd_y = velcmd * sin(psi);
    */
    /*
    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);

    float velcmd = goal_velx;
    if (angle_err > 30.0*D2R)
        velcmd = goal_velx*0.1;
    else
        velcmd = goal_velx;

    cmd_x = satmax(0.0*velcmd * cos(psi) + Kpx*(path.x - Cur_Pos_m[0])*1.5,velcmd*3.0);
    cmd_y = satmax(0.0*velcmd * sin(psi) + Kpx*(path.y - Cur_Pos_m[1])*1.5,velcmd*3.0);
    cmd_z = satmax(Kpz*(goal[2] - Cur_Pos_m[2]), goal_velz);

    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = goal[2];
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];
    */
    float goal_z;
    float dist = sqrt((path.x-Cur_Pos_m[0])*(path.x-Cur_Pos_m[0]) + (path.y-Cur_Pos_m[1])*(path.y-Cur_Pos_m[1]));
    if (dist > 0.2)
    {
        path.psi = atan2(path.y-Cur_Pos_m[1], path.x-Cur_Pos_m[0]);
    }
    path.psi = atan2(path.y-Cur_Pos_m[1], path.x-Cur_Pos_m[0]);

    angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);

    float velcmd = goal_velx;
    if (fabs(angle_err) > 20.0*D2R)
        velcmd = goal_velx*0.5;
    else
        velcmd = goal_velx;

    cmd_x = velcmd * cos(path.psi);
    cmd_y = velcmd * sin(path.psi);
    //cmd_x = satmax(Kpx*(path.x - Cur_Pos_m[0]),goal_velx);
    //cmd_y = satmax(Kpx*(path.y - Cur_Pos_m[1]),goal_velx);
    if (scan_up_minimum < 1){
	goal_z = (Cur_Pos_m[2] + scan_up_minimum) /2;
    }
    else{
	goal_z = goal[2];
    }
    cmd_z = satmax(Kpz*(goal_z - Cur_Pos_m[2]), goal_velz) + Kdz*(0.0 - Cur_Vel_mps[2]);

    //angle_err = GetNED_angle_err(goal[3], Cur_Att_rad[2]);
    //angle_err = GetNED_angle_err(path.psi, Cur_Att_rad[2]);
    cmd_r = -satmax(Kr*angle_err, R_MAX);

    hover[0] = Cur_Pos_m[0];
    hover[1] = Cur_Pos_m[1];
    hover[2] = goal_z;
    hover_heading = Cur_Att_rad[2];
    goal_heading = Cur_Att_rad[2];
}


void publish_target(void)
{
    Target.type = visualization_msgs::Marker::SPHERE;
    Target.header.frame_id = "scout/map";
    Target.pose.orientation.w = 1.0;
    Target.scale.x = Target.scale.y = Target.scale.z = 0.1;
    Target.ns = "target point";
    Target.color.r = 1.0;
    Target.color.b = 0.1;
    Target.color.g = 0.1;
    Target.color.a = 1.0;

    Target.pose.position.x = WayPoint_X[point_ind];
    Target.pose.position.y = WayPoint_Y[point_ind];
    Target.pose.position.z = WayPoint_Z[point_ind];

    point_ind = point_ind + 1;

    if (point_ind == WP_num + 1)
    {
        point_ind = WP_num;
    }
}

