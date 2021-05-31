//#define WP_GEN

#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

#include <tf/transform_datatypes.h>
#include "aStarLibrary.h"
#include <time.h>

std::string robot_namespace;

float SAFEDISTANSE  = 0.3f;
float TARGETDISTANCE = 0.50f;

const int GOALNUM = 1;
const float desiredVelocity = 1.0;
nav_msgs::OccupancyGrid Map;
nav_msgs::OccupancyGrid distanceMap;

ros::Publisher path_pub;
ros::Publisher marker_pub;
ros::Publisher map_check_pub;
ros::Publisher target_pub;
ros::Publisher wpt_array_pub;
ros::Publisher goal_pub;
ros::Publisher wpt_pub;

std_msgs::Float32MultiArray target;
std_msgs::Float32MultiArray wpt_array;

float des_ang = 0;
float des_ang_snap = 0;

float v_x = 0.0;
float v_y = 0.0;
float v_ang = 0.0;
float radius = 0.0;
float prev_radius = 100.0;
int SafeDistance_x = 0;
int SafeDistance_y = 0;
int MapDataAvailable = 0;
int isProcessing = 0;

int PtSx = 0;
int PtSy = 0;
int PtGx_local = 0;
int PtGy_local = 0;

int PtGx_global = 0;
int PtGy_global = 0;
int MaxX, MaxY ;

float localGoalX = 0.0;
float localGoalY = 0.0;
int PtStartX = 0;
int PtStartY = 0;
int goal_service = 0;

float Goal[GOALNUM][2];
int PtGx = 0;
int PtGy = 0;
int CurrentGoal = 0;
float AcceptanceRadius = 0.4;

void initialization()
{
  Goal[0][0] = 0;
  Goal[0][1] = 0;
}


void find_path()
{
        //check_WPT();
	// ROS_INFO("Find Path_changed");
	isProcessing = 1;
	double MapInfo_origin_x = Map.info.origin.position.x;
	double MapInfo_origin_y = Map.info.origin.position.y;
	double MapInfo_resolution = Map.info.resolution;

        //ROS_INFO("MapInfo_origin : [%f %f]", MapInfo_origin_x, MapInfo_origin_y);

	double *out_waypt1 ;
	double obs_array[1000] = {0} ;

	int **ObsMap ;

	MaxX = Map.info.width ;
	MaxY = Map.info.height ;

	ObsMap = new int*[MaxX] ;
	for(int i = 0; i<MaxX; i++){
		ObsMap[i] = new int [MaxY] ;
	}

        int MaxDistance = 0;
        for ( int num = 0 ; num < MaxX ; num++)
        {
            for ( int num2 = 0 ; num2 < MaxY ; num2++)
            {
                if(DistanceField[num2][num] > MaxDistance){
                    MaxDistance = DistanceField[num2][num];
                }
            }
        }

	int OccupancyValue = 0;
        for ( int num = 0 ; num < MaxX ; num++)
        {
                for ( int num2 = 0 ; num2 < MaxY ; num2++)
                {
                        //walkable = 0; unwalkable = 1;
                        if(Map.data[num+(MaxX)*num2]==-1)  //unknown area;
                        {
                                OccupancyValue = 0;
                        }
                        else if(Map.data[num+(MaxX)*num2]>50) // occupancy probability is 50% or up
                        {
                                OccupancyValue = 1;
                        }
                        else
                        {
                                OccupancyValue = 0;
                        }

                        OriginalMap[num][num2] = OccupancyValue;

                        if ( ( num >= MaxX ) || ( num2 >= MaxY ) )
                        {
                                OriginalMap[num][num2] = unwalkable ;
                        }

                        ObsMap[num][num2] = walkable ;
                }
        }

        float SafeDistance = SAFEDISTANSE; //[m]
        int SafeDis = int(SafeDistance/MapInfo_resolution) ;
	for ( int num = 0 ; num < MaxX ; num++)
	{
		for ( int num2 = 0 ; num2 < MaxY ; num2++)
		{
			if ( OriginalMap[num][num2] > 0 )
			{
				for ( int xnum = -SafeDis ; xnum<(SafeDis+1) ; xnum++ )
				{
					if ( ( (num+xnum) >= MaxX) || ( (num+xnum) < 0) )
					{
                                                continue ;
                                        }
                                        for ( int ynum = -SafeDis ; ynum<(SafeDis+1) ; ynum++ )
					{
						if ( ( (num2+ynum) >= MaxY) || ( (num2+ynum) < 0) )
						{
                                                        continue ;
                                                }

						ObsMap[num+xnum][num2+ynum] = unwalkable ;
					}
				}
			}
		}
	}


	if(ObsMap[PtSx][PtSy]==1)
	{
		int search_x = 1;
		int search_y = 1;
		int isfree = 0;
		while(isfree==0)
		{
			for(int i=-search_x;i<=search_x;i++)
			{
				for(int j=-search_y;j<=search_y;j++)
				{
					if(PtSx+i>0 && PtSx+i<MaxX)
					{
						if(PtSy+j>0 && PtSy+j<MaxY)
						{
							if(ObsMap[PtSx+i][PtSy+j]==0)
							{
								PtSx = PtSx+i;
								PtSy = PtSy+j;
								isfree = 1;
								break;
							}
						}
					}
					if(isfree == 1)
					{
						break;
					}

				}
				if(isfree == 1)
				break;
			}
			search_y++;
			search_x++;
			if(isfree ==1)
			break;
		}
	}


	if(ObsMap[PtGx][PtGy]==1)
	{
		int search_x = 1;
		int search_y = 1;
		int isfree = 0;
		while(isfree==0)
		{
			for(int i=-search_x;i<=search_x;i++)
			{
				for(int j=-search_y;j<=search_y;j++)
				{
					if(PtGx+i>0 && PtGx+i<MaxX)
					{
						if(PtGy+j>0 && PtGy+j<MaxY)
						{
							if(ObsMap[PtGx+i][PtGy+j]==0)
							{
								PtGx = PtGx+i;
								PtGy = PtGy+j;
								isfree = 1;
								break;
							}
						}
					}
					if(isfree == 1)
					{
						break;
					}

				}
				if(isfree == 1)
				break;
			}
			search_y++;
			search_x++;
			if(isfree ==1)
			break;
		}
	}

	//Generate obs_map
	nav_msgs::OccupancyGrid map;
	//fill in some map parameters
	map.header.stamp = Map.header.stamp;
        map.header.frame_id = "scout/map";
	map.info.width = MaxX;
	map.info.height = MaxY;
	map.info.origin.orientation.w = 1;
	map.info.resolution = MapInfo_resolution;
	map.info.origin.position.x = MapInfo_origin_x;
	map.info.origin.position.y = MapInfo_origin_y;

	for(int i=0;i<MaxY;i++)
	{
		for(int j=0;j<MaxX;j++)
                {
                        if(ObsMap[j][i]==0)
			map.data.push_back(0);
			else
                        map.data.push_back(100);
		}
	}
        map_check_pub.publish(map);


        float path_ang = 0.0;
        float goalpt_x = float(PtGx)*MapInfo_resolution+MapInfo_origin_x;
        float goalpt_y = float(PtGy)*MapInfo_resolution+MapInfo_origin_y;



        out_waypt1 = GenPath(PtSx, PtSy, PtGx, PtGy, ObsMap, MaxX, MaxY);

	int Waypoint_size = out_waypt1[0];
	nav_msgs::Path path;

	geometry_msgs::PoseStamped pose;
	path.header.stamp = ros::Time::now();
        path.header.frame_id = "scout/map";

	if(Waypoint_size>1)
	{                
                std::vector<float> target_data;
                target_data.clear();

                float cur_posx = float(out_waypt1[2*(Waypoint_size)-1])*MapInfo_resolution+MapInfo_origin_x;
                float cur_posy = float(out_waypt1[2*(Waypoint_size)])*MapInfo_resolution+MapInfo_origin_y;
                float goalx = float(out_waypt1[2*(Waypoint_size-1)-1])*MapInfo_resolution+MapInfo_origin_x;
                float goaly = float(out_waypt1[2*(Waypoint_size-1)])*MapInfo_resolution+MapInfo_origin_y;



                //TODO (Sunggoo): this should be modified for a proper trajectory following
                if(Waypoint_size > 3)
                {
                    goalx = float(out_waypt1[2*(Waypoint_size-2)-1])*MapInfo_resolution+MapInfo_origin_x;
                    goaly = float(out_waypt1[2*(Waypoint_size-2)])*MapInfo_resolution+MapInfo_origin_y;
                    des_ang = atan2(goaly-v_y, goalx-v_x);
                }
                else
                {
                    goalx = float(out_waypt1[2*(Waypoint_size-1)-1])*MapInfo_resolution+MapInfo_origin_x;
                    goaly = float(out_waypt1[2*(Waypoint_size-1)])*MapInfo_resolution+MapInfo_origin_y;
                    des_ang = atan2(goaly-v_y, goalx-v_x);
                }

                float length = sqrt((goalx-cur_posx)*(goalx-cur_posx)+(goaly-cur_posy)*(goaly-cur_posy));


                float sum_dist = 0.0;
                float pre_goalx = cur_posx;
                float pre_goaly = cur_posy;
                float length_goal = 0.0;
                for (int ind = 2; ind < Waypoint_size; ind++)
                {
                    if (sum_dist > TARGETDISTANCE)
                    {
                        if (length_goal > TARGETDISTANCE)
                        {
                            goalx = pre_goalx + (goalx-pre_goalx)/length_goal;
                            goaly = pre_goaly + (goaly-pre_goaly)/length_goal;
                        }
                        else
                        {
                            goalx = pre_goalx;
                            goaly = pre_goaly;
                        }
                    }
                    else
                    {
                        goalx = float(out_waypt1[2*(Waypoint_size-ind)-1])*MapInfo_resolution+MapInfo_origin_x;
                        goaly = float(out_waypt1[2*(Waypoint_size-ind)])*MapInfo_resolution+MapInfo_origin_y;

                        length_goal = sqrt((goalx-pre_goalx)*(goalx-pre_goalx)+(goaly-pre_goaly)*(goaly-pre_goaly));
                        sum_dist = sum_dist + length_goal;
                        length = sqrt((goalx-cur_posx)*(goalx-cur_posx)+(goaly-cur_posy)*(goaly-cur_posy));

                        pre_goalx = goalx;
                        pre_goaly = goaly;
                    }
                }

                target_data.push_back(goalx);
                target_data.push_back(goaly);
                target_data.push_back(des_ang);
                target_data.push_back(goalpt_x);
                target_data.push_back(goalpt_y);
                target.data.clear();
                target.data.insert(target.data.end(),target_data.begin(),target_data.end());
                wpt_pub.publish(target);

		visualization_msgs::Marker points;
                points.header.frame_id = "scout/map";
                points.header.stamp= ros::Time::now();
                points.ns="Goal_Pt";
                points.id = 0;
                points.type = visualization_msgs::Marker::POINTS;
                points.scale.x = 0.2;
                points.scale.y = 0.2;
		points.color.r = 1.0f;
		points.color.a = 1.0;

		geometry_msgs::Point p;
		p.x = goalx;
		p.y = goaly;
		points.points.push_back(p);
                //points.pose.push_back(p);
                marker_pub.publish(points);
        }

        /*
        visualization_msgs::Marker goalpt;
        goalpt.header.frame_id ="scout/map";
        goalpt.header.stamp= ros::Time::now();
        goalpt.ns="Goal_Pt";
        goalpt.type = visualization_msgs::Marker::POINTS;
        goalpt.scale.x = goalpt.scale.y = 0.2;
        goalpt.color.r = 0.2f;
        goalpt.color.b = 1.0f;
        goalpt.color.a = 1.0;
        geometry_msgs::Point p1;
        p1.x = goalpt_x;
        p1.y = goalpt_y;
        goalpt.points.push_back(p1);
        goal_pub.publish(goalpt);
        */
	// wpt_data.push_back(Waypoint_size);
	// wpt_data.push_back(0.0f);

        //ROS_INFO("%d",Waypoint_size);
        for(int i=0;i<Waypoint_size;i++)
	{
                //A* path generation
		pose.pose.position.x = double(out_waypt1[2*(Waypoint_size-i)-1])*MapInfo_resolution+MapInfo_origin_x+MapInfo_resolution/2.0f; //Get the value from the back
		pose.pose.position.y = double(out_waypt1[2*(Waypoint_size-i)])*MapInfo_resolution+MapInfo_origin_y+MapInfo_resolution/2.0f;
		path.poses.push_back(pose);
	}

        path_pub.publish(path);


	for(int i = 0; i<MaxX; i++){
		delete []ObsMap[i];
	}
	delete [] ObsMap;
	isProcessing = 0;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // ROS_INFO("Map Received!");
    Map.header = msg->header;
    Map.info = msg->info;
    Map.data = msg->data;
    MapDataAvailable = 1;
    if(isProcessing == 0)
    {
            find_path();
    }
}


void distanceMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Map Received!");
    distanceMap.header = msg->header;
    distanceMap.info = msg->info;
    distanceMap.data = msg->data;

    for ( int num = 0 ; num < MaxY ; num++)
    {
        for ( int num2 = 0 ; num2 < MaxX ; num2++)
        {
            DistanceField[num][num2] = distanceMap.data[num2+(MaxX)*num];
        }
    }
}

void posCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(MapDataAvailable==1)
    {
        double MapInfo_origin_x = Map.info.origin.position.x;
        double MapInfo_origin_y = Map.info.origin.position.y;
        double MapInfo_resolution = Map.info.resolution;
        PtSx = int((msg->pose.position.x-MapInfo_origin_x)/(MapInfo_resolution));
        PtSy = int((msg->pose.position.y-MapInfo_origin_y)/(MapInfo_resolution));
        v_ang = tf::getYaw(msg->pose.orientation);
        v_x = msg->pose.position.x;
        v_y = msg->pose.position.y;
        if(isProcessing == 0)
        {
                find_path();
        }
    }
    /*
    float desiredVelx = sqrt( (desiredVelocity*desiredVelocity) / (1 + (tan(des_ang_snap)*tan(des_ang_snap))) );
    float desiredVely = sqrt( (desiredVelocity*desiredVelocity*tan(des_ang_snap)*tan(des_ang_snap)) / (1 + (tan(des_ang_snap)*(tan(des_ang_snap)))) );
    if(des_ang_snap < 0.0) desiredVely = desiredVely * -1.0;
    if(fabs(des_ang_snap) > 1.57) desiredVelx = desiredVelx * -1.0;

    wpt_array.data.clear();
    wpt_array.data.resize(9);
    wpt_array.data[0] = msg->pose.position.x;
    wpt_array.data[1] = msg->pose.position.y;
    wpt_array.data[2] = localGoalX;
    wpt_array.data[3] = localGoalY;
    wpt_array.data[4] = (sqrt((msg->pose.position.x-localGoalX)*(msg->pose.position.x-localGoalX) + (msg->pose.position.y-localGoalY)*(msg->pose.position.y-localGoalY)))/desiredVelocity;
    wpt_array.data[5] = desiredVelx;
    wpt_array.data[6] = desiredVely;
    wpt_array.data[7] = 0.0;
    wpt_array.data[8] = 0.0;
    wpt_array_pub.publish(wpt_array);
    */
}

void goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(MapDataAvailable==1)
    {
        double MapInfo_origin_x = Map.info.origin.position.x;
        double MapInfo_origin_y = Map.info.origin.position.y;
        double MapInfo_resolution = Map.info.resolution;
        ROS_INFO("GoalReceived");

        PtGx = int((msg->data[0]-MapInfo_origin_x)/MapInfo_resolution);
        PtGy = int((msg->data[1]-MapInfo_origin_y)/MapInfo_resolution);

        if (float(PtGx) >= float(Map.info.width-1))  PtGx = Map.info.width-1;
        if (float(PtGx) < 0)                         PtGx = 0;
        if (float(PtGy) >= float(Map.info.height-1)) PtGy = Map.info.height-1;
        if (float(PtGy) < 0)                         PtGy = 0;

        if(isProcessing == 0)
        {
                find_path();
        }
    }
}

void missionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(MapDataAvailable==1)
    {
        float mission = msg->data[0];
        if(mission == 6.0)
        {
            SAFEDISTANSE  = 0.4f;
            TARGETDISTANCE = 0.4f;
        }
        else
        {
            SAFEDISTANSE  = 0.3f;
            TARGETDISTANCE = 0.5f;
        }
    }
}

int main(int argc, char* argv[])
{
    initialization();
    ros::init(argc, argv, "astar_path");
    robot_namespace = ros::this_node::getNamespace();
    robot_namespace.erase(0, 2);

    ros::NodeHandle nh_map;
    ros::NodeHandle nh_odo;
    ros::NodeHandle nh_goal;
    ros::NodeHandle nh_pub;

    ros::Subscriber map_sub = nh_map.subscribe("map", 1, mapCallback);
    ros::Subscriber distanceMap_sub = nh_map.subscribe("map_distance", 1, distanceMapCallback);
    ros::Subscriber odo_sub = nh_odo.subscribe("mavros/vision_pose/pose", 1, posCallback);
    ros::Subscriber goal_sub = nh_goal.subscribe("GoalPoint", 1, goalCallback);
    ros::Subscriber mission_sub = nh_goal.subscribe("Mission", 1, missionCallback);

    map_check_pub = nh_pub.advertise<nav_msgs::OccupancyGrid>("map_check", 1);
    wpt_pub = nh_pub.advertise<std_msgs::Float32MultiArray>("astar_path",1);

    path_pub = nh_pub.advertise<nav_msgs::Path>("astar_path_vis",1);
    marker_pub = nh_pub.advertise<visualization_msgs::Marker>("marker_target",1);

    ROS_INFO("Astar Planner Started...");
    ros::spin();
    return 0;
}
