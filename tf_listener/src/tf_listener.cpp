#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"



#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

using namespace std;
using namespace cv;
using namespace ros;

void toEulerAngle(Vec4f &);

Vec3f _state;
Vec4f _quaternion;
float height_m = 0.1;
float height_RL = 0.1;
float height_LPF = 0.1;
float height_pre = 0.1;
float captHeight = 0.1;
float PI = 3.1415926535;

float Hz = 60.0;
float LIMIT = 0.5;
float mission = 0.0;

void Lidar(const sensor_msgs::Range &Lidar_Height_msg)
{

    if (isinf(Lidar_Height_msg.range))
        height_m = 0.1;
    else
        height_m = Lidar_Height_msg.range;
    /*
    if (height_m < 0.3)
    {
        if (isinf(Lidar_Height_msg.range))
        {
            height_m = 0.1;
        }
        else
        {
            height_m = Lidar_Height_msg.range;
        }
    }
    else
    {
        height_m = Lidar_Height_msg.range;
    }

    if (isinf(Lidar_Height_msg.range))
        height_m  = captHeight;
    */
    if (isnan(Lidar_Height_msg.range))
        height_m  = captHeight;

    captHeight = height_m;
}

void missionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    mission = msg->data[0];
}

float LPF(float data, float data_pre, float freq)
{
    float res;
    float K_LPF;
    float delT = 1/Hz;

    K_LPF = freq*2*PI*delT / (1 + freq*2*PI*delT);

    res = (data - data_pre) * K_LPF + data_pre;

    return res;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener");
  ros::NodeHandle nh_;

  ros::Subscriber Lidar_sub_    = nh_.subscribe("teraranger_evo", 1, &Lidar);
  ros::Subscriber mission_sub   = nh_.subscribe("Mission", 1, missionCallback);
  ros::Publisher uav_pose       = nh_.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
  tf::TransformListener listener;

  ros::Rate rate(Hz);
  ROS_INFO("TF_Listener Node start");

  while( ok() )
  {
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped pos_msg;

    try{
      listener.lookupTransform("scout/map", "scout/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
      continue;
    }

    _state = Vec3f(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    _quaternion = Vec4f(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());

    float rate_height = (height_m - height_RL)*Hz;

    if (rate_height > LIMIT)
        rate_height = LIMIT;
    if (rate_height < -LIMIT)
        rate_height = -LIMIT;

    height_RL = height_RL + rate_height/Hz;

    height_LPF = LPF(height_RL, height_pre, 2.0);
    height_pre = height_LPF;

    pos_msg.header.stamp = ros::Time::now();
    pos_msg.header.frame_id = "scout/map";
    pos_msg.pose.position.x = _state[0];
    pos_msg.pose.position.y = _state[1];

    if (mission == 7.0)
    {
        pos_msg.pose.position.z = height_LPF;
    }
    else
    {
        pos_msg.pose.position.z = height_m;
    }

    pos_msg.pose.orientation.x = _quaternion[0];
    pos_msg.pose.orientation.y = _quaternion[1];
    pos_msg.pose.orientation.z = _quaternion[2];
    pos_msg.pose.orientation.w = _quaternion[3];
    uav_pose.publish(pos_msg);
    //ROS_INFO("PosX: %.3f, PosY: %.3f, PosZ: %.3f\n", pos_msg.pose.position.x, pos_msg.pose.position.y, height_m);

    rate.sleep();
    spinOnce();
  }
  return 0;
};

