

#include "DefineList.h"
#include "MissionUtil.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>

#include <fstream>

// setup the initial name
using namespace ros;
using namespace std;

// x, y, z, r, vx
void parseMissionCsvFile(const std::string& inputFileName, std::vector<mission_command>& data) {
    std::ifstream inputFile(inputFileName);

    mission_command mission_cmd;

    int l = 0;
    bool first_line = true;
    while (inputFile) {
        l++;
        string s;
        if (!getline(inputFile, s))
            break;
        if (s[0] != '#') {
            istringstream ss(s);

            int cnt = 0;
            bool nan_flg = false;
            while (ss) {
                string line;
                if (!getline(ss, line, ','))
                    break;
                try {
                    if (cnt == 0)
                        mission_cmd.x = stof(line);
                    else if (cnt == 1)
                        mission_cmd.y = stof(line);
                    else if (cnt == 2)
                        mission_cmd.z = stof(line);
                    else if (cnt == 3)
                        mission_cmd.r = stof(line);
                    else if (cnt == 4)
                        mission_cmd.vx = stof(line);
                    else if (cnt == 5)
                        mission_cmd.vz = stof(line);
                } catch (const std::invalid_argument e) {
                    cout << "NaN found in file " << inputFileName << " line " << l
                         << endl;
                    e.what();
                    nan_flg = true;
                }
                cnt++;
            }
            if (nan_flg == false) {
                if (first_line) {
                    mission_cmd.mode = mission_cmd.TAKEOFF;
                    first_line = false;
                } else {
                    mission_cmd.mode = mission_cmd.WAYPOINT;
                }
                data.push_back(mission_cmd);
            }
        }
    }

    if (!inputFile.eof()) {
        cerr << "Could not read file " << inputFileName << "\n";
        __throw_invalid_argument("File not found.");
    }
    data.at(data.size()-1).mode = mission_cmd.LAND;
    for (int i = 0; i < data.size(); i++) {
        std::cout << "[" << i << "]"
                  <<  " x: "  << data.at(i).x
                  << ", y: "  << data.at(i).y 
                  << ", z: "  << data.at(i).z 
                  << ", vx: " << data.at(i).vx 
                  << ", vz: " << data.at(i).vz << std::endl;
    }

}


void callback_local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg_input)
{
    Cur_Pos_m[0] = msg_input->pose.position.x;
    Cur_Pos_m[1] = msg_input->pose.position.y;
    Cur_Pos_m[2] = msg_input->pose.position.z;

    float q[4];
    q[0] = msg_input->pose.orientation.x;
    q[1] = msg_input->pose.orientation.y;
    q[2] = msg_input->pose.orientation.z;
    q[3] = msg_input->pose.orientation.w;
    QuaterniontoEuler(q, Cur_Att_rad[0], Cur_Att_rad[1], Cur_Att_rad[2]);
}

void callback_local_vel(const geometry_msgs::TwistStamped::ConstPtr& msg_input)
{
    Cur_Vel_mps[0] = msg_input->twist.linear.x;
    Cur_Vel_mps[1] = msg_input->twist.linear.y;
    Cur_Vel_mps[2] = msg_input->twist.linear.z;
}

void callback_detection(const std_msgs::Float32MultiArray::ConstPtr& msg_input)
{
    detection.flag   = msg_input->data[0];
    detection.size_x = msg_input->data[1];
    detection.size_y = msg_input->data[2];
    detection.pos_x  = msg_input->data[3];
    detection.pos_y  = msg_input->data[4];
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    min_dist = 65.0;
    for (int ind=0; ind<10; ind ++)
   {
        lidar_data[ind] = msg->ranges[500+4*ind];
        if (min_dist > lidar_data[ind])
        {
            min_dist = lidar_data[ind];
        }
        //printf("scan data : %.3f\n",lidar_data[ind]);
    }
   target_dist = min_dist;
    //printf("\n");
    right_dist = msg->ranges[90];
    left_dist = msg->ranges[870];
    //float center_dist = msg->ranges[500];
    //printf("right : %.3f left : %.3f center : %.3f \n", right_dist, left_dist, center_dist);
}

void callback_scan_filter(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float max_val=0;
	float max_idx=0;
	std_msgs::UInt16 idx_dat;

	for (int indx=0; indx<msg->ranges.size(); indx++){
		if (msg->ranges[indx] > max_val){
			max_val = msg->ranges[indx];
			max_idx = indx;
		}
	}
        idx_dat.data = max_idx;
	pub_max_idx.publish(idx_dat);	
}

void callback_max_idx(const std_msgs::UInt16::ConstPtr& msg)
{
	maxind     = msg->data;
	maxang_deg = -90 + 0.25*maxind;
        maxang_rad = maxang_deg * 3.1415 / 180;
	//std::cout<<"maxang_deg"<<" "<<maxang_deg<<std::endl;
	//std::cout<<"maxang_rad"<<" "<<maxang_rad<<std::endl;
}

void callback_sonar(const std_msgs::Float32::ConstPtr& msg)
{
    sonar_dist = (msg->data)*0.01;
    sonar_dist_lpf = LPF(sonar_dist, sonar_dist_pre, 2.0);
    sonar_dist_pre = sonar_dist_lpf;

    //printf("sonar data : %.3f  LPF : %.3f\n", sonar_dist, sonar_dist_lpf);
}


// node main loop, for ROS
int main(int argc, char** argv)
{
    init(argc, argv, "mission_planner");     // node name initialization
    NodeHandle nh_sub;                       // assign node handler
    NodeHandle nh_pub;                       // assign node handler
    NodeHandle nh_scan;                      // assign node handler
    NodeHandle nh_param("~");

    printf("Initiate: mission_planner\n");   // for debugging

    // Subscribe Topic
    sub_local_pos  = nh_sub.subscribe ("/scout/mavros/local_position/pose", 2,  &callback_local_pos);
    sub_local_vel  = nh_sub.subscribe ("/scout/mavros/local_position/velocity_local", 2,  &callback_local_vel);
    sub_detection  = nh_sub.subscribe ("/detection", 2,  &callback_detection);
    sub_scan_filter= nh_scan.subscribe ("/scout/scan_filtered", 2, &callback_scan_filter);
    sub_scan       = nh_scan.subscribe ("/scout/scan", 2, &callback_scan);
    sub_sonar      = nh_scan.subscribe ("/scout/arduino/ultrasound1", 2, &callback_sonar);
    sub_max_idx    = nh_scan.subscribe ("/scout/scan_filtered/max_idx", 1, &callback_max_idx);

    // Publish Topic
    pub_goalaction = nh_pub.advertise<std_msgs::Float32MultiArray>  ("GoalAction", 2);
    pub_goalpoint  = nh_pub.advertise<std_msgs::Float32MultiArray>  ("GoalPoint", 2);
    pub_goalpoint_  = nh_pub.advertise<geometry_msgs::PoseStamped>  ("global_goal", 2);
    pub_mission    = nh_pub.advertise<std_msgs::Float32MultiArray>  ("Mission", 2);
    pub_servo      = nh_pub.advertise<std_msgs::UInt16>             ("/scout/arduino/servo1", 2);
    pub_max_idx    = nh_pub.advertise<std_msgs::UInt16>             ("/scout/scan_filtered/max_idx", 2);


    //double pos_x;
    //nh_param.param("pos_x", pos_x, 0.0);
    //printf("pos_x : %.3f\n",pos_x);
    nh_param.param("mission_start", mission_start, 0);

    nh_param.param("use_legacy_mission", use_legacy_mission, true);
    nh_param.param<std::string>("mission_file", mission_file, "mission_file");
    nh_param.param("next_mission_thres_xy", next_mission_thres_xy, 0.5);
    nh_param.param("next_mission_thres_z", next_mission_thres_z, -1.0);

    nh_param.param("window_width", window_width, 1.0);
    nh_param.param("window_height", window_height, 1.0);
    nh_param.param("camera_width", camera_width, 640);
    nh_param.param("camera_height", camera_height, 480);
    nh_param.param("fx", fx, 480);
    nh_param.param("fy", fy, 480);
    nh_param.param("cx", cx, 320);
    nh_param.param("cy", cy, 240);

    nh_param.param("VEL_FACTOR", VEL_FACTOR, 1.0);

    nh_param.param("PHASE_Takeoff_mission", PHASE_Takeoff_mission, 0);
    nh_param.param("PHASE_Takeoff_next", PHASE_Takeoff_next, 0);
    nh_param.param("PHASE_Takeoff_x", PHASE_Takeoff_x, 0.0);
    nh_param.param("PHASE_Takeoff_y", PHASE_Takeoff_y, 0.0);
    nh_param.param("PHASE_Takeoff_z", PHASE_Takeoff_z, 1.0);
    nh_param.param("PHASE_Takeoff_r", PHASE_Takeoff_r, 0.0);
    nh_param.param("PHASE_Takeoff_vx", PHASE_Takeoff_vx, 0.0);
    nh_param.param("PHASE_Takeoff_vz", PHASE_Takeoff_vz, 0.0);

    nh_param.param("PHASE_Wall_Pass_mission", PHASE_Wall_Pass_mission, 0);
    nh_param.param("PHASE_Wall_Pass_next", PHASE_Wall_Pass_next, 0);
    nh_param.param("PHASE_Wall_Pass_x", PHASE_Wall_Pass_x, 0.0);
    nh_param.param("PHASE_Wall_Pass_y", PHASE_Wall_Pass_y, 0.0);
    nh_param.param("PHASE_Wall_Pass_z", PHASE_Wall_Pass_z, 1.0);
    nh_param.param("PHASE_Wall_Pass_r", PHASE_Wall_Pass_r, 0.0);
    nh_param.param("PHASE_Wall_Pass_vx", PHASE_Wall_Pass_vx, 0.0);
    nh_param.param("PHASE_Wall_Pass_vz", PHASE_Wall_Pass_vz, 0.0);

    nh_param.param("PHASE_Pole_Pass_mission", PHASE_Pole_Pass_mission, 0);
    nh_param.param("PHASE_Pole_Pass_next", PHASE_Pole_Pass_next, 0);
    nh_param.param("PHASE_Pole_Pass_x", PHASE_Pole_Pass_x, 0.0);
    nh_param.param("PHASE_Pole_Pass_y", PHASE_Pole_Pass_y, 0.0);
    nh_param.param("PHASE_Pole_Pass_z", PHASE_Pole_Pass_z, 1.0);
    nh_param.param("PHASE_Pole_Pass_r", PHASE_Pole_Pass_r, 0.0);
    nh_param.param("PHASE_Pole_Pass_vx", PHASE_Pole_Pass_vx, 0.0);
    nh_param.param("PHASE_Pole_Pass_vz", PHASE_Pole_Pass_vz, 0.0);

    nh_param.param("PHASE_Net_Pass_mission", PHASE_Net_Pass_mission, 0);
    nh_param.param("PHASE_Net_Pass_next", PHASE_Net_Pass_next, 0);
    nh_param.param("PHASE_Net_Pass_x", PHASE_Net_Pass_x, 0.0);
    nh_param.param("PHASE_Net_Pass_y", PHASE_Net_Pass_y, 0.0);
    nh_param.param("PHASE_Net_Pass_z", PHASE_Net_Pass_z, 1.0);
    nh_param.param("PHASE_Net_Pass_r", PHASE_Net_Pass_r, 0.0);
    nh_param.param("PHASE_Net_Pass_vx", PHASE_Net_Pass_vx, 0.0);
    nh_param.param("PHASE_Net_Pass_vz", PHASE_Net_Pass_vz, 0.0);

    nh_param.param("PHASE_Tree_Pass_mission", PHASE_Tree_Pass_mission, 0);
    nh_param.param("PHASE_Tree_Pass_next", PHASE_Tree_Pass_next, 0);
    nh_param.param("PHASE_Tree_Pass_x", PHASE_Tree_Pass_x, 0.0);
    nh_param.param("PHASE_Tree_Pass_y", PHASE_Tree_Pass_y, 0.0);
    nh_param.param("PHASE_Tree_Pass_z", PHASE_Tree_Pass_z, 1.0);
    nh_param.param("PHASE_Tree_Pass_r", PHASE_Tree_Pass_r, 0.0);
    nh_param.param("PHASE_Tree_Pass_vx", PHASE_Tree_Pass_vx, 0.0);
    nh_param.param("PHASE_Tree_Pass_vz", PHASE_Tree_Pass_vz, 0.0);

    nh_param.param("PHASE_Pipe_Pass_mission", PHASE_Pipe_Pass_mission, 0);
    nh_param.param("PHASE_Pipe_Pass_next", PHASE_Pipe_Pass_next, 0);
    nh_param.param("PHASE_Pipe_Pass_x", PHASE_Pipe_Pass_x, 0.0);
    nh_param.param("PHASE_Pipe_Pass_y", PHASE_Pipe_Pass_y, 0.0);
    nh_param.param("PHASE_Pipe_Pass_z", PHASE_Pipe_Pass_z, 1.0);
    nh_param.param("PHASE_Pipe_Pass_r", PHASE_Pipe_Pass_r, 0.0);
    nh_param.param("PHASE_Pipe_Pass_vx", PHASE_Pipe_Pass_vx, 0.0);
    nh_param.param("PHASE_Pipe_Pass_vz", PHASE_Pipe_Pass_vz, 0.0);

    nh_param.param("PHASE_Window_Pass_mission", PHASE_Window_Pass_mission, 0);
    nh_param.param("PHASE_Window_Pass_next", PHASE_Window_Pass_next, 0);
    nh_param.param("PHASE_Window_Pass_x", PHASE_Window_Pass_x, 0.0);
    nh_param.param("PHASE_Window_Pass_y", PHASE_Window_Pass_y, 0.0);
    nh_param.param("PHASE_Window_Pass_z", PHASE_Window_Pass_z, 1.0);
    nh_param.param("PHASE_Window_Pass_r", PHASE_Window_Pass_r, 0.0);
    nh_param.param("PHASE_Window_Pass_vx", PHASE_Window_Pass_vx, 0.0);
    nh_param.param("PHASE_Window_Pass_vz", PHASE_Window_Pass_vz, 0.0);

    nh_param.param("PHASE_Wind_Pass_mission", PHASE_Wind_Pass_mission, 0);
    nh_param.param("PHASE_Wind_Pass_next", PHASE_Wind_Pass_next, 0);
    nh_param.param("PHASE_Wind_Pass_x", PHASE_Wind_Pass_x, 0.0);
    nh_param.param("PHASE_Wind_Pass_y", PHASE_Wind_Pass_y, 0.0);
    nh_param.param("PHASE_Wind_Pass_z", PHASE_Wind_Pass_z, 1.0);
    nh_param.param("PHASE_Wind_Pass_r", PHASE_Wind_Pass_r, 0.0);
    nh_param.param("PHASE_Wind_Pass_vx", PHASE_Wind_Pass_vx, 0.0);
    nh_param.param("PHASE_Wind_Pass_vz", PHASE_Wind_Pass_vz, 0.0);
    nh_param.param("PHASE_Wind_Pass_mode", PHASE_Wind_Pass_mode, 0);
    nh_param.param("PHASE_Wind_Pass_box_x", PHASE_Wind_Pass_box_x, 0.0);
    nh_param.param("PHASE_Wind_Pass_box_y", PHASE_Wind_Pass_box_y, 0.0);
    nh_param.param("PHASE_Wind_Pass_box_alt", PHASE_Wind_Pass_box_alt, 0.0);

    nh_param.param("PHASE_Landing_mission", PHASE_Landing_mission, 0);
    nh_param.param("PHASE_Landing_x", PHASE_Landing_x, 0.0);
    nh_param.param("PHASE_Landing_y", PHASE_Landing_y, 0.0);
    nh_param.param("PHASE_Landing_z", PHASE_Landing_z, 1.0);
    nh_param.param("PHASE_Landing_r", PHASE_Landing_r, 0.0);
    nh_param.param("PHASE_Landing_vx", PHASE_Landing_vx, 0.0);
    nh_param.param("PHASE_Landing_vz", PHASE_Landing_vz, 0.0);

    printf("%.3f, %.3f, %d, %d\n", window_width, window_height, camera_width, camera_height);

    InitParam();
    std::vector<mission_command> mission_cmd_array;
    parseMissionCsvFile(mission_file, mission_cmd_array);
    //TaskSelection();
    flag.mission = mission_start;

    Rate loop_rate(20.0);                    // setup the loop speed, [Hz]

    // node loop, for ROS, check ros status, ros::ok()
    while( ok() )
    {
        t_cur = count_ros/20.0;

        if (use_legacy_mission)
        {
            Mission_Update();
        }
        else
        {
            mission_command next_command;
            findCurrentMission(mission_cmd_array, next_command);
            publishCommand(next_command);
        }

        Publish();

        count_ros = count_ros + 1;
        loop_rate.sleep();
        spinOnce();
    }

    // for debugging
    printf("Terminate: mission_planner\n");

    return 0;
}

void Publish(void)
{
    Mission.data[0] = flag.mission;
    Mission.data[1] = 0.0;
    Mission.data[2] = 0.0;
    Mission.data[3] = 0.0;
    Mission.data[4] = 0.0;

    pub_mission.publish(Mission);
    //pub_goalaction.publish(GoalAction);
}


void Mission_Update(void)
{

    switch (flag.mission)
    {
        // Basic Mission
        case 1:
            PHASE_Takeoff();
            break;

        case 2:
            PHASE_Wall_Pass();
            break;

        case 3:
            PHASE_Pole_Pass();
            break;

        case 4:
            PHASE_Net_Pass();
            break;

        case 5:
            PHASE_Tree_Pass();
            break;

        case 6:
            PHASE_Pipe_Pass();
            break;

        case 7:
            PHASE_Window_Pass_alt();
            break;

        case 8:
            PHASE_Wind_Pass();
            break;

        case 9:
            PHASE_Landing();
            break;

        default:
            break;
    }

    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("[time] %3.1f\n",t_cur);
        printf("[Cur] [x] %3.3f  [y] %3.3f  [z] %3.3f\n", Cur_Pos_m[0],Cur_Pos_m[1],Cur_Pos_m[2]);
        printf("[WP]  [x] %3.3f  [y] %3.3f  [z] %3.3f\n", GoalAction.data[1], GoalAction.data[2], GoalAction.data[3]);
        printf("\n\n\n\n\n");
    }

}
