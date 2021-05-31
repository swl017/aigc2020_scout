#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include "aStarLibrary.h"
#include <time.h>

nav_msgs::OccupancyGrid Map;
ros::Publisher path_pub;
ros::Publisher marker_pub;
ros::Publisher map_check_pub;
ros::Publisher wpt_pub;

std_msgs::Float32MultiArray wpt;

int PtSx = 0;
int PtSy = 0;
int PtGx = 0;
int PtGy = 0;

float v_x = 0;
float v_y = 0;
//float v_ang = 0;
int MapDataAvailable = 0;
int isProcessing = 0;


void find_path()
{
	ROS_INFO("Find Path_changed");
	isProcessing = 1;
	double MapInfo_origin_x = Map.info.origin.position.x;
	double MapInfo_origin_y = Map.info.origin.position.y;
	double MapInfo_resolution = Map.info.resolution;

	// ROS_INFO("MapInfo_origin : [%f %f]", MapInfo_origin_x, MapInfo_origin_y);

	double *out_waypt1 ;
	double obs_array[1000] = {0} ;
	int MaxX, MaxY ;
	int **ObsMap ;

	MaxX = Map.info.width ;
	MaxY = Map.info.height ;

	ObsMap = new int*[MaxX] ;
	for(int i = 0; i<MaxX; i++){
		ObsMap[i] = new int [MaxY] ;
	}

	// PtSx = 10 ; PtSy = 10 ;
	// PtGx = 75 ; PtGy = 63 ;

	// fopen_s(&MapData, "ObsMap.txt", "r") ;

	int OccupancyValue = 0;
	for ( int num = 0 ; num < MaxX ; num++)
	{
		// fgets (mystring , 100 , MapData);

		for ( int num2 = 0 ; num2 < MaxY ; num2++)
		{
			if(Map.data[num+(MaxX)*num2]==-1)
			{
				OccupancyValue = 0;
			}
			else if(Map.data[num+(MaxX)*num2]>50)
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
	// fclose(MapData) ;

	// fopen_s(&OutputMap, "OutputMap.txt", "w") ;
        float SafeDistance = 0.7; //[m]
	int SafeDis = int(SafeDistance/MapInfo_resolution) ; // �̰ݰŸ� ����
	// ROS_INFO("Padding = %d",SafeDis);
	// ROS_INFO("MapResoluation = %f",MapInfo_resolution);
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

	//If the vehicle is inside unwalkable region, find the nearest vacant position

	if(ObsMap[PtSx][PtSy]==1)
	{
		int search_x = 1;
		int search_y = 1;
		int isfree = 0;
		while(isfree==0)
		{
			ROS_INFO("Reset Starting Point");
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
						ROS_INFO("Start Change : [%d %d]", PtSx, PtSy);
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
	ROS_INFO("Start : [%d %d]", PtSx, PtSy);

	if(ObsMap[PtGx][PtGy]==1)
	{
		int search_x = 1;
		int search_y = 1;
		int isfree = 0;
		while(isfree==0)
		{
			ROS_INFO("Reset Starting Point");
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
						ROS_INFO("Start Change : [%d %d]", PtSx, PtSy);
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

	ROS_INFO("Goal : [%d %d]", PtGx, PtGy);



	//Generate obs_map
	nav_msgs::OccupancyGrid map;
	//fill in some map parameters
	map.header.stamp = Map.header.stamp;
	map.header.frame_id = "map";
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
	out_waypt1 = GenPath(PtSx, PtSy, PtGx, PtGy, ObsMap, MaxX, MaxY) ;

	int Waypoint_size = out_waypt1[0];
	nav_msgs::Path path;

	geometry_msgs::PoseStamped pose;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "map";
        float des_ang = 0;
	if(Waypoint_size>1)
	{
		std::vector<float> wpt_data;
		wpt_data.clear();

		float map_vx = float(out_waypt1[2*(Waypoint_size)-1])*MapInfo_resolution+MapInfo_origin_x+MapInfo_resolution/2.0f;
		float map_vy = float(out_waypt1[2*(Waypoint_size)])*MapInfo_resolution+MapInfo_origin_y+MapInfo_resolution/2.0f;

		float goalx = float(out_waypt1[2*(Waypoint_size-1)-1])*MapInfo_resolution+MapInfo_origin_x+MapInfo_resolution/2.0f;
		float goaly = float(out_waypt1[2*(Waypoint_size-1)])*MapInfo_resolution+MapInfo_origin_y+MapInfo_resolution/2.0f;
                des_ang = atan2(goaly-v_y, goalx-v_x);

		float length = sqrt((goalx-map_vx)*(goalx-map_vx)+(goaly-map_vy)*(goaly-map_vy));
		ROS_INFO("%f",length);
		if(length>1.0f)
		{
			goalx = map_vx+(goalx-map_vx)/length;
			goaly = map_vy+(goaly-map_vy)/length;
		}

		wpt_data.push_back(goalx);
		wpt_data.push_back(goaly);
                wpt_data.push_back(des_ang);
		wpt.data.clear();
		wpt.data.insert(wpt.data.end(),wpt_data.begin(),wpt_data.end());
		wpt_pub.publish(wpt);
		visualization_msgs::Marker points;

		points.header.frame_id ="/map";
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
		marker_pub.publish(points);


	}

	// wpt_data.push_back(Waypoint_size);
	// wpt_data.push_back(0.0f);

	// ROS_INFO("%d",Waypoint_size);
	for(int i=0;i<Waypoint_size;i++)
	{

		pose.pose.position.x = double(out_waypt1[2*(Waypoint_size-i)-1])*MapInfo_resolution+MapInfo_origin_x+MapInfo_resolution/2.0f; //Get the value from the back
		pose.pose.position.y = double(out_waypt1[2*(Waypoint_size-i)])*MapInfo_resolution+MapInfo_origin_y+MapInfo_resolution/2.0f;
		path.poses.push_back(pose);
	}

	//Normalize to 1m
	path_pub.publish(path);

	for(int i = 0; i<MaxX; i++){
		delete []ObsMap[i];
	}
	delete [] ObsMap;
	isProcessing = 0;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("Map Received!");
	Map.header = msg->header;
	Map.info = msg->info;
	Map.data = msg->data;
	MapDataAvailable = 1;
	if(isProcessing == 0)
	{
		find_path();
	}

}

void posCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("Odometry Received");
	if(MapDataAvailable==1)
	{
		double MapInfo_origin_x = Map.info.origin.position.x;
		double MapInfo_origin_y = Map.info.origin.position.y;
		double MapInfo_resolution = Map.info.resolution;
		PtSx = int((msg->pose.pose.position.x-MapInfo_origin_x)/(MapInfo_resolution));
		PtSy = int((msg->pose.pose.position.y-MapInfo_origin_y)/(MapInfo_resolution));
//		v_ang = tf::getYaw(msg->pose.pose.orientation);
		v_x = msg->pose.pose.position.x;
		v_y = msg->pose.pose.position.y;
		// ROS_INFO("Angle : %f", v_ang*180.0/3.14159265358979);
		// ROS_INFO("Position : [%f %f]", v_x, v_y);
		if(isProcessing == 0)
		{
			find_path();
		}
	}
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(MapDataAvailable==1)
	{
		double MapInfo_origin_x = Map.info.origin.position.x;
		double MapInfo_origin_y = Map.info.origin.position.y;
		double MapInfo_resolution = Map.info.resolution;
		ROS_INFO("GoalReceived");
		PtGx = int((msg->pose.position.x-MapInfo_origin_x)/MapInfo_resolution);
		PtGy = int((msg->pose.position.y-MapInfo_origin_y)/MapInfo_resolution);
		ROS_INFO("[%d %d]",PtGx,PtGy);

		if(isProcessing == 0)
		{
			find_path();
		}
	}
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "astar_path");
  ros::NodeHandle nh_map;
	ros::NodeHandle nh_odo;
	ros::NodeHandle nh_goal;
	ros::NodeHandle nh_pub;
	ros::Subscriber map_sub = nh_map.subscribe("/map", 1, mapCallback);
	ros::Subscriber odo_sub = nh_odo.subscribe("/nav_position", 1, posCallback);
	ros::Subscriber goal_sub = nh_goal.subscribe("/move_base_simple/goal",10,goalCallback);
  path_pub = nh_pub.advertise<nav_msgs::Path>("/astar_path_vis",1);
  marker_pub = nh_pub.advertise<visualization_msgs::Marker>("StartGoalPoints",1);
	map_check_pub = nh_pub.advertise<nav_msgs::OccupancyGrid>("/map_check", 1);
	wpt_pub = nh_pub.advertise<std_msgs::Float32MultiArray>("/astar_path",1);
	ROS_INFO("Astar Planner Started...");
	ros::spin();
	return 0;
}
