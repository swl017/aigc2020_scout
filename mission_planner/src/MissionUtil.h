
#ifndef MissionUtil_H
#define MissionUtil_H

#include "MissionDefine.h"
#include "DefineList.h"

void GeoLocation(void);

void InitParam(void)
{
    GoalPoint.data.resize(2);
    GoalAction.data.resize(7);
    //GoalPose
    Mission.data.resize(5);
    pub_goalaction.publish(GoalAction);
}

float wrap(float data)
{
    float res;
    data = fmod(data+180.0*D2R, 360.0*D2R);
    if(data < 0.0)
        data = data + 360.0*D2R;

    res = data-180.0*D2R;
    return res;
}

void WaypointGenerator(void)
{
    WayPoint_X[0] = PHASE_Window_Pass_x;
    WayPoint_Y[0] = PHASE_Window_Pass_y - 1.0;

    WayPoint_X[1] = PHASE_Window_Pass_x - 1.5;
    WayPoint_Y[1] = PHASE_Window_Pass_y - 1.0;

    WayPoint_X[2] = PHASE_Window_Pass_x - 1.5;
    WayPoint_Y[2] = PHASE_Window_Pass_y + 1.0;

    WayPoint_X[3] = PHASE_Window_Pass_x - 3.0;
    WayPoint_Y[3] = PHASE_Window_Pass_y + 1.0;

    WayPoint_X[4] = PHASE_Window_Pass_x - 3.0;
    WayPoint_Y[4] = PHASE_Window_Pass_y - 1.0;

    WayPoint_X[5] = PHASE_Window_Pass_x - 4.5;
    WayPoint_Y[5] = PHASE_Window_Pass_y;
}

void OpenArms(void)
{
    Servo.data = 100;
    pub_servo.publish(Servo);
}


void CloseArms(void)
{
    Servo.data = 160;
    pub_servo.publish(Servo);
}

void TaskSelection(void)
{
    // Mission start from where
    printf("\n");
    printf("===========================\n");
    printf(" Mission Selection\n");
    printf("---------------------------\n");
    printf(" 1. Auto Takeoff\n");
    printf(" 2. Window Pass\n");
    printf(" 3. Poll Pass\n");
    printf(" 4. Pipe Pass\n");
    printf(" 5. Tree Pass\n");
    printf(" 6. Net Pass\n");
    printf(" 7. Wind Pass\n");
    printf(" 8. Target Landing\n");
    printf("===========================\n\n\n");

    printf("Type Mission Start : ");
    scanf("%d", &flag_input);

    flag.mission = flag_input;
}

void PHASE_Takeoff(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 1. Auto Takeoff\n");
        printf("===========================\n\n");
    }
    if (flag.PHASE_Takeoff == 0)
    {
        // target action
        GoalAction.data[0] = PHASE_Takeoff_mission;

        // target position
        GoalAction.data[1] = PHASE_Takeoff_x;
        GoalAction.data[2] = PHASE_Takeoff_y;
        GoalAction.data[3] = PHASE_Takeoff_z;
        GoalAction.data[4] = PHASE_Takeoff_r;

        // target velocity
        GoalAction.data[5] = PHASE_Takeoff_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Takeoff_vz;
        pub_goalaction.publish(GoalAction);

        //flag.PHASE_Takeoff = 1;
    }
    // Astar Path Goal
    GoalPoint.data[0] = PHASE_Takeoff_x;
    GoalPoint.data[1] = PHASE_Takeoff_y;
    GoalPose.header.stamp = ros::Time::now();
    GoalPose.header.frame_id = "odom";
    GoalPose.pose.position.x = PHASE_Takeoff_x;
    GoalPose.pose.position.y = PHASE_Takeoff_y;
    GoalPose.pose.position.z = PHASE_Takeoff_z;
    pub_goalpoint_.publish(GoalPose);
    pub_goalpoint.publish(GoalPoint);
    float delx = GoalPoint.data[0] - Cur_Pos_m[0];
    float dely = GoalPoint.data[1] - Cur_Pos_m[1];

    float dist = sqrt(delx*delx + dely*dely);
    if (fabs(dist) < 1.0)
    {
        flag.mission = PHASE_Takeoff_next;
    }

    CloseArms();
}


void PHASE_Wall_Pass(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 2. Wall Pass\n");
        printf("===========================\n\n");
    }

    if (flag.PHASE_Wall_Pass == 0)
    {

        // target action
        GoalAction.data[0] = PHASE_Wall_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Wall_Pass_x;
        GoalAction.data[2] = PHASE_Wall_Pass_y;
        GoalAction.data[3] = PHASE_Wall_Pass_z;
        GoalAction.data[4] = PHASE_Wall_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Wall_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Wall_Pass_vz;
        pub_goalaction.publish(GoalAction);

        GoalPoint.data[0] = PHASE_Wall_Pass_x;
        GoalPoint.data[1] = PHASE_Wall_Pass_y;
    GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Wall_Pass_x;
    GoalPose.pose.position.y = PHASE_Wall_Pass_y;
    GoalPose.pose.position.z = PHASE_Wall_Pass_z;
    pub_goalpoint_.publish(GoalPose);
        pub_goalpoint.publish(GoalPoint);

        float delx = GoalPoint.data[0] - Cur_Pos_m[0];
        float dely = GoalPoint.data[1] - Cur_Pos_m[1];

        float dist = sqrt(delx*delx + dely*dely);
        if (fabs(dist) < 1.5)
        {
            flag.mission = PHASE_Wall_Pass_next;
        }
    }
}



void PHASE_Window_Pass(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 2. Window Pass\n");
        printf("===========================\n\n");
        printf("Current Phase : %d  dist : %.3f %.3f\n",flag.PHASE_Window_Pass, sonar_dist, sonar_dist_lpf);
    }

    if (flag.PHASE_Window_Pass == 0)
    {

        // target action
        GoalAction.data[0] = PHASE_Window_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Window_Pass_x;
        GoalAction.data[2] = PHASE_Window_Pass_y;
        GoalAction.data[3] = PHASE_Window_Pass_z;
        GoalAction.data[4] = PHASE_Window_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Window_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Window_Pass_vz;
        pub_goalaction.publish(GoalAction);

        GoalPoint.data[0] = PHASE_Window_Pass_x;
        GoalPoint.data[1] = PHASE_Window_Pass_y;
    GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Window_Pass_x;
    GoalPose.pose.position.y = PHASE_Window_Pass_y;
    GoalPose.pose.position.z = PHASE_Window_Pass_z;
    pub_goalpoint_.publish(GoalPose);
        pub_goalpoint.publish(GoalPoint);

        float delx = GoalPoint.data[0] - Cur_Pos_m[0];
        float dely = GoalPoint.data[1] - Cur_Pos_m[1];

        float dist = sqrt(delx*delx + dely*dely);
        if(fabs(dist) < 0.5)
        {
            flag.mission = PHASE_Window_Pass_next;
            WaypointGenerator();
        }

        if(fabs(sonar_dist_lpf) < 0.2)
        {
            temp_x = Cur_Pos_m[0]+2.0;
            temp_y = Cur_Pos_m[1];
            flag.PHASE_Window_Pass = 1;
        }

    }

    if (flag.PHASE_Window_Pass == 1)
    {
        GoalAction.data[0] = 3.0;

        // target position
        GoalAction.data[1] = temp_x;
        GoalAction.data[2] = temp_y;
        GoalAction.data[3] = PHASE_Window_Pass_z;
        GoalAction.data[4] = PHASE_Window_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Window_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Window_Pass_vz;
        pub_goalaction.publish(GoalAction);

        GoalPoint.data[0] = temp_x;
        GoalPoint.data[1] = temp_y;
        pub_goalpoint.publish(GoalPoint);


        float delx = temp_x - Cur_Pos_m[0];
        float dely = temp_y - Cur_Pos_m[1];

        float dist = sqrt(delx*delx + dely*dely);
        if (fabs(dist) < 0.2)
        {
            temp_x = Cur_Pos_m[0];

            if (right_dist > left_dist)
            {
                temp_y = Cur_Pos_m[1] + 2.0;
            }
            else
            {
                temp_y = Cur_Pos_m[1] - 2.0;
            }

            flag.PHASE_Window_Pass = 2;
        }
    }

    if (flag.PHASE_Window_Pass == 2)
    {
        GoalAction.data[0] = 3.0;

        // target position
        GoalAction.data[1] = temp_x;
        GoalAction.data[2] = temp_y;
        GoalAction.data[3] = PHASE_Window_Pass_z;
        GoalAction.data[4] = PHASE_Window_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Window_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Window_Pass_vz;
        pub_goalaction.publish(GoalAction);

        GoalPoint.data[0] = temp_x;
        GoalPoint.data[1] = temp_y;
        pub_goalpoint.publish(GoalPoint);


        float delx = temp_x - Cur_Pos_m[0];
        float dely = temp_y - Cur_Pos_m[1];

        float dist = sqrt(delx*delx + dely*dely);
        if (fabs(dist) < 0.2)
        {
            flag.PHASE_Window_Pass = 0;
        }
    }

}

void PHASE_Window_Pass_alt(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 2. Window Pass\n");
        printf("===========================\n\n");
        printf("Current Phase : %d  dist : %.3f %.3f\n",flag.PHASE_Window_Pass, sonar_dist, sonar_dist_lpf);
    }

    if (flag.PHASE_Window_Pass == 1)
    {

        // target action
        GoalAction.data[0] = PHASE_Window_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Window_Pass_x;
        GoalAction.data[2] = PHASE_Window_Pass_y;
        GoalAction.data[3] = PHASE_Window_Pass_z;
	if (window_search_complete == false){
		GoalAction.data[4] = PHASE_Window_Pass_r;
	}
	else{
		GoalAction.data[4] = PHASE_Window_Pass_r;
	}
        // target velocity
        GoalAction.data[5] = PHASE_Window_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Window_Pass_vz;
        pub_goalaction.publish(GoalAction);

        GoalPoint.data[0] = PHASE_Window_Pass_x;
        GoalPoint.data[1] = PHASE_Window_Pass_y;
    GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Window_Pass_x;
    GoalPose.pose.position.y = PHASE_Window_Pass_y;
    GoalPose.pose.position.z = PHASE_Window_Pass_z;
    pub_goalpoint_.publish(GoalPose);
        pub_goalpoint.publish(GoalPoint);

        float delx = GoalPoint.data[0] - Cur_Pos_m[0];
        float dely = GoalPoint.data[1] - Cur_Pos_m[1];

        float dist = sqrt(delx*delx + dely*dely);
        if(fabs(dist) < 0.5)
        {
            flag.mission = PHASE_Window_Pass_next;
            WaypointGenerator();
        }

    }

    if (flag.PHASE_Window_Pass == 0)
    {
        GoalAction.data[0] = 3.0;

        // target position
        GoalAction.data[1] = PHASE_Pipe_Pass_x;
        GoalAction.data[2] = PHASE_Pipe_Pass_y;
        GoalAction.data[3] = PHASE_Window_Pass_z;
        GoalAction.data[4] = PHASE_Window_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Window_Pass_vx*VEL_FACTOR;;
        GoalAction.data[6] = PHASE_Window_Pass_vz;
        pub_goalaction.publish(GoalAction);

        GoalPoint.data[0] = Cur_Pos_m[0];
        GoalPoint.data[1] = Cur_Pos_m[1];
    GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Window_Pass_x;
    GoalPose.pose.position.y = PHASE_Window_Pass_y;
    GoalPose.pose.position.z = PHASE_Window_Pass_z;
    pub_goalpoint_.publish(GoalPose);
        pub_goalpoint.publish(GoalPoint);


        float delz = GoalAction.data[3] - Cur_Pos_m[2];

        float dist = fabs(delz);
        if (fabs(dist) < 0.2)
        {
            flag.PHASE_Window_Pass = 1;
            window_search_complete = true;
        }
    }

}

void PHASE_Pole_Pass(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 3. Pole Pass\n");
        printf("===========================\n\n");
    }

    if (flag.PHASE_Poll_Pass == 0)
    {
        // target action
        GoalAction.data[0] = PHASE_Pole_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Pole_Pass_x;
        GoalAction.data[2] = PHASE_Pole_Pass_y;
        GoalAction.data[3] = PHASE_Pole_Pass_z;
        GoalAction.data[4] = PHASE_Pole_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Pole_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Pole_Pass_vz;
        pub_goalaction.publish(GoalAction);

        flag.PHASE_Poll_Pass = 0;
    }

    GoalPoint.data[0] = PHASE_Pole_Pass_x;
    GoalPoint.data[1] = PHASE_Pole_Pass_y;
      GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Pole_Pass_x;
    GoalPose.pose.position.y = PHASE_Pole_Pass_y;
    GoalPose.pose.position.z = PHASE_Pole_Pass_z;
    pub_goalpoint_.publish(GoalPose);
    pub_goalpoint.publish(GoalPoint);

    float dist = sqrt((GoalPoint.data[0] - Cur_Pos_m[0])*(GoalPoint.data[0] - Cur_Pos_m[0]) + (GoalPoint.data[1] - Cur_Pos_m[1])*(GoalPoint.data[1] - Cur_Pos_m[1]));
    if (fabs(dist) < 1.0)
    {
        flag.mission = PHASE_Pole_Pass_next;
    }

}

void PHASE_Pipe_Pass(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 4. Pipe Pass\n");
        printf("===========================\n\n");
    }

    if (flag.PHASE_Pipe_Pass == 0)
    {
        // target action
        GoalAction.data[0] = PHASE_Pipe_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Pipe_Pass_x;
        GoalAction.data[2] = PHASE_Pipe_Pass_y;
        GoalAction.data[3] = PHASE_Pipe_Pass_z;
        GoalAction.data[4] = PHASE_Pipe_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Pipe_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Pipe_Pass_vz;
        pub_goalaction.publish(GoalAction);

        flag.PHASE_Pipe_Pass = 1;
    }

    GoalPoint.data[0] = PHASE_Pipe_Pass_x;
    GoalPoint.data[1] = PHASE_Pipe_Pass_y;
      GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Pipe_Pass_x;
    GoalPose.pose.position.y = PHASE_Pipe_Pass_y;
    GoalPose.pose.position.z = PHASE_Pipe_Pass_z;
    pub_goalpoint_.publish(GoalPose);
    pub_goalpoint.publish(GoalPoint);

    float dist = sqrt((GoalPoint.data[0] - Cur_Pos_m[0])*(GoalPoint.data[0] - Cur_Pos_m[0]) + (GoalPoint.data[1] - Cur_Pos_m[1])*(GoalPoint.data[1] - Cur_Pos_m[1]));
    if (fabs(dist) < 1.0)
    {
        //flag.PHASE_Pipe_Pass = 2;
        flag.mission = PHASE_Pipe_Pass_next;
    }
}

void PHASE_Tree_Pass(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 5. Tree Pass\n");
        printf("===========================\n\n");
    }

    if (flag.PHASE_Tree_Pass == 0)
    {
        // target action
        GoalAction.data[0] = PHASE_Tree_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Tree_Pass_x;
        GoalAction.data[2] = PHASE_Tree_Pass_y;
        GoalAction.data[3] = PHASE_Tree_Pass_z;
        GoalAction.data[4] = PHASE_Tree_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Tree_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Tree_Pass_vz;
        pub_goalaction.publish(GoalAction);

        flag.PHASE_Tree_Pass = 1;
    }

    GoalPoint.data[0] = PHASE_Tree_Pass_x;
    GoalPoint.data[1] = PHASE_Tree_Pass_y;
      GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Tree_Pass_x;
    GoalPose.pose.position.y = PHASE_Tree_Pass_y;
    GoalPose.pose.position.z = PHASE_Tree_Pass_z;
    pub_goalpoint_.publish(GoalPose);
    pub_goalpoint.publish(GoalPoint);

    float dist = sqrt((GoalPoint.data[0] - Cur_Pos_m[0])*(GoalPoint.data[0] - Cur_Pos_m[0]) + (GoalPoint.data[1] - Cur_Pos_m[1])*(GoalPoint.data[1] - Cur_Pos_m[1]));
    if (fabs(dist) < 1.0)
    {
        flag.mission = PHASE_Tree_Pass_next;
        mission_count = 0;
    }

}

void PHASE_Net_Pass(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 6. Net Pass\n");
        printf("===========================\n\n");

        printf("flag : %d\n", flag.PHASE_Net_Pass);
        printf("target_dist : %.3f\n", target_dist);
    }


    if (flag.PHASE_Net_Pass == 0)
    {
        // target action
        GoalAction.data[0] = PHASE_Net_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Net_Pass_x;
        GoalAction.data[2] = PHASE_Net_Pass_y;
        GoalAction.data[3] = PHASE_Net_Pass_z;
        GoalAction.data[4] = PHASE_Net_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Net_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Net_Pass_vz;
        pub_goalaction.publish(GoalAction);

        mission_count = mission_count + 1;

        if (mission_count > 10)
            flag.PHASE_Net_Pass = 0;
    }

    GoalAction.data[1] = PHASE_Net_Pass_x;
    GoalAction.data[2] = PHASE_Net_Pass_y;

    float vel = fabs(Cur_Vel_mps[0]);

    if (flag.PHASE_Net_Pass == 1)
    {
        if ( target_dist < 0.8)
        {
            flag.PHASE_Net_Pass = 2;
        }
    }

    if (flag.PHASE_Net_Pass == 2)
    {
        GoalAction.data[5] = 0.0;
        GoalAction.data[6] = 0.5;

        if (Cur_Pos_m[2] > 1.5)
        {
            GoalAction.data[3] = 0.7;
        }
        else
        {
            GoalAction.data[3] = 2.0;
        }

        flag.PHASE_Net_Pass = 3;
    }

    if (flag.PHASE_Net_Pass == 3)
    {
        if (fabs(GoalAction.data[3] - Cur_Pos_m[2]) < 0.2)
        {
            GoalAction.data[5] = PHASE_Net_Pass_vx*VEL_FACTOR;

            if (vel > 0.20)
            {
                flag.PHASE_Net_Pass = 1;
            }
        }
    }

    GoalPoint.data[0] = PHASE_Net_Pass_x;
    GoalPoint.data[1] = PHASE_Net_Pass_y;
      GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Net_Pass_x;
    GoalPose.pose.position.y = PHASE_Net_Pass_y;
    GoalPose.pose.position.z = PHASE_Net_Pass_z;
    pub_goalpoint_.publish(GoalPose);
    pub_goalpoint.publish(GoalPoint);

    pub_goalaction.publish(GoalAction);

    float delx = PHASE_Net_Pass_x - Cur_Pos_m[0];
    float dely = PHASE_Net_Pass_y - Cur_Pos_m[1];

    float dist = sqrt(delx*delx + dely*dely);
    if (fabs(dist) < 0.5)
    {
        flag.mission = PHASE_Net_Pass_next;
        mission_count = 0;
    }
}

void PHASE_Wind_Pass(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 7. Wind Pass\n");
        printf("===========================\n\n");

        printf("state : %d\n", flag.PHASE_Wind_Pass);
    }

    GeoLocation();

    if (detection.flag == 1)
    {
        box_x = tar_data.pos[0];
        box_y = tar_data.pos[1];

        count_wind = count_wind + 1;
    }

    //box_x = 4.0;
    //box_y = -6.5;
    //count_wind = 20;
    flag.PHASE_Wind_Pass =9;
    if (PHASE_Wind_Pass_mode == 0)
    {
        if (flag.PHASE_Wind_Pass == 0)
        {
            // target action
            GoalAction.data[0] = PHASE_Wind_Pass_mission;

            // target position
            GoalAction.data[1] = WayPoint_X[WP_index];
            GoalAction.data[2] = WayPoint_Y[WP_index];
            GoalAction.data[3] = PHASE_Wind_Pass_z;
            GoalAction.data[4] = PHASE_Wind_Pass_r;

            // target velocity
            GoalAction.data[5] = PHASE_Wind_Pass_vx*VEL_FACTOR;
            GoalAction.data[6] = PHASE_Wind_Pass_vz;
            pub_goalaction.publish(GoalAction);

            GoalPoint.data[0] = WayPoint_X[WP_index];
            GoalPoint.data[1] = WayPoint_Y[WP_index];
            pub_goalpoint.publish(GoalPoint);

            float delx = WayPoint_X[WP_index] - Cur_Pos_m[0];
            float dely = WayPoint_Y[WP_index] - Cur_Pos_m[1];

            float dist = sqrt(delx*delx + dely*dely);

            if (fabs(dist) < 0.2)
            {
                WP_index = WP_index + 1;
                if (WP_index == WP_num+1)
                {
                    WP_index = WP_num;
                    flag.PHASE_Wind_Pass = 9;
                }
            }

            if (count_wind > 20)
                flag.PHASE_Wind_Pass = 1;
        }
    }
    else
    {
        if (flag.PHASE_Wind_Pass == 0)
        {
            // target action
            GoalAction.data[0] = PHASE_Wind_Pass_mission;

            // target position
            GoalAction.data[1] = PHASE_Wind_Pass_box_x;
            GoalAction.data[2] = PHASE_Wind_Pass_box_y;
            GoalAction.data[3] = PHASE_Wind_Pass_z;
            GoalAction.data[4] = PHASE_Wind_Pass_r;

            // target velocity
            GoalAction.data[5] = PHASE_Wind_Pass_vx*VEL_FACTOR;
            GoalAction.data[6] = PHASE_Wind_Pass_vz;
            pub_goalaction.publish(GoalAction);

            GoalPoint.data[0] = PHASE_Wind_Pass_box_x;
            GoalPoint.data[1] = PHASE_Wind_Pass_box_y;
      GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Wind_Pass_box_x;
    GoalPose.pose.position.y = PHASE_Wind_Pass_box_y;
    GoalPose.pose.position.z = PHASE_Wind_Pass_z;
    pub_goalpoint_.publish(GoalPose);
            pub_goalpoint.publish(GoalPoint);

            float delx = PHASE_Wind_Pass_box_x - Cur_Pos_m[0];
            float dely = PHASE_Wind_Pass_box_y - Cur_Pos_m[1];

            float dist = sqrt(delx*delx + dely*dely);

            if (fabs(dist) < 0.2)
            {
                count_no_box = count_no_box + 1;
            }

            if (count_no_box > 100)
            {
                flag.PHASE_Wind_Pass = 9;
            }

            if (count_wind > 20)
            {
                flag.PHASE_Wind_Pass = 1;
            }
        }
    }


    if (flag.PHASE_Wind_Pass == 1)
    {
        // target action
        GoalAction.data[0] = PHASE_Wind_Pass_mission;

        // target position
        GoalAction.data[1] = box_x;
        GoalAction.data[2] = box_y;
        GoalAction.data[3] = PHASE_Wind_Pass_z;
        GoalAction.data[4] = PHASE_Wind_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Wind_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Wind_Pass_vz;
        pub_goalaction.publish(GoalAction);

        sub_dist = sqrt((box_x - Cur_Pos_m[0])*(box_x - Cur_Pos_m[0]) + (box_y - Cur_Pos_m[1])*(box_y - Cur_Pos_m[1]));
        if (fabs(sub_dist) < 0.2){
            flag.PHASE_Wind_Pass = 2;
            last_box_x = GoalAction.data[1];
            last_box_y = GoalAction.data[2];
            }
    }

    if (flag.PHASE_Wind_Pass == 2)
    {
        // target action
        GoalAction.data[0] = PHASE_Wind_Pass_mission;

        // target position
        GoalAction.data[1] = last_box_x;
        GoalAction.data[2] = last_box_y;
        GoalAction.data[3] = PHASE_Wind_Pass_box_alt;
        GoalAction.data[4] = PHASE_Wind_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Wind_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Wind_Pass_vz;
        pub_goalaction.publish(GoalAction);

        if (fabs(PHASE_Wind_Pass_box_alt - Cur_Pos_m[2]) < 0.1)
        {
            OpenArms();
            flag.PHASE_Wind_Pass = 3;
        }
    }

    if (flag.PHASE_Wind_Pass == 3)
    {
        OpenArms();

        // target action
        GoalAction.data[0] = PHASE_Wind_Pass_mission;

        // target position
        GoalAction.data[1] = box_x;
        GoalAction.data[2] = box_y;
        GoalAction.data[3] = 1.5;
        GoalAction.data[4] = PHASE_Wind_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Wind_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Wind_Pass_vz;
        pub_goalaction.publish(GoalAction);

        if (fabs(1.5 - Cur_Pos_m[2]) < 0.1)
            flag.PHASE_Wind_Pass = 9;
    }

    if (flag.PHASE_Wind_Pass == 9)
    {
        // target action
        GoalAction.data[0] = PHASE_Wind_Pass_mission;

        // target position
        GoalAction.data[1] = PHASE_Wind_Pass_x;
        GoalAction.data[2] = PHASE_Wind_Pass_y;
        GoalAction.data[3] = PHASE_Wind_Pass_z;
        GoalAction.data[4] = PHASE_Wind_Pass_r;

        // target velocity
        GoalAction.data[5] = PHASE_Wind_Pass_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Wind_Pass_vz;
        pub_goalaction.publish(GoalAction);

        GoalPoint.data[0] = PHASE_Wind_Pass_x;
        GoalPoint.data[1] = PHASE_Wind_Pass_y;
      GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Wind_Pass_x;
    GoalPose.pose.position.y = PHASE_Wind_Pass_y;
    GoalPose.pose.position.z = PHASE_Wind_Pass_z;
    pub_goalpoint_.publish(GoalPose);
        pub_goalpoint.publish(GoalPoint);

        flag.PHASE_Wind_Pass = 9;
    }

    float delx = PHASE_Wind_Pass_x - Cur_Pos_m[0];
    float dely = PHASE_Wind_Pass_y - Cur_Pos_m[1];

    float dist = sqrt(delx*delx + dely*dely);
    if (fabs(dist) < 0.5)
    {
        flag.mission = PHASE_Wind_Pass_next;
    }
}

void PHASE_Landing(void)
{
    if(fmod(t_cur*20.0,PRINT_HZ)==0.0)
    {
        printf("===========================\n");
        printf(" 8. Target Landing\n");
        printf("===========================\n\n");

        printf("[cur] %.3f %.3f\n",Cur_Pos_m[0], Cur_Pos_m[1]);
        printf("[tar] %.3f %.3f\n", tar_data.pos[0], tar_data.pos[1]);
    }

    if (flag.PHASE_Target_Landing == 0)
    {
        // target action
        GoalAction.data[0] = PHASE_Landing_mission;

        // target position
        GoalAction.data[1] = PHASE_Landing_x;
        GoalAction.data[2] = PHASE_Landing_y;
        GoalAction.data[3] = PHASE_Landing_z;
        GoalAction.data[4] = PHASE_Landing_r;

        // target velocity
        GoalAction.data[5] = PHASE_Landing_vx*VEL_FACTOR;
        GoalAction.data[6] = PHASE_Landing_vz;
        pub_goalaction.publish(GoalAction);

        flag.PHASE_Target_Landing = 1;
    }

    GeoLocation();

    if (detection.flag == 1)
    {
        GoalAction.data[1] = tar_data.pos[0];
        GoalAction.data[2] = tar_data.pos[1];

        GoalPoint.data[0] = tar_data.pos[0];
        GoalPoint.data[1] = tar_data.pos[1];
    }
    else
    {
        GoalAction.data[1] = PHASE_Landing_x;
        GoalAction.data[2] = PHASE_Landing_y;

        GoalPoint.data[0] = PHASE_Landing_x;
        GoalPoint.data[1] = PHASE_Landing_y;
    }
      GoalPose.header.stamp = ros::Time::now();
    GoalPose.pose.position.x = PHASE_Landing_x;
    GoalPose.pose.position.y = PHASE_Landing_y;
    GoalPose.pose.position.z = PHASE_Landing_z;
    pub_goalpoint_.publish(GoalPose);
    pub_goalpoint.publish(GoalPoint);

    float dist = sqrt((GoalAction.data[1] - Cur_Pos_m[0])*(GoalAction.data[1] - Cur_Pos_m[0]) + (GoalAction.data[2] - Cur_Pos_m[1])*(GoalAction.data[2] - Cur_Pos_m[1]));

    if(dist < 0.2)
        GoalAction.data[6] = PHASE_Landing_vz;
    else
        GoalAction.data[6] = PHASE_Landing_vz;


    pub_goalaction.publish(GoalAction);
}

static void QuaterniontoEuler(float* q, float& roll, float& pitch, float& yaw)
{

    // roll (x-axis rotation)
    float t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
    float t1 = +1.0 - 2.0 * (q[0] * q[0] + q[1]*q[1]);
    roll = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    float t2 = +2.0 * (q[3] * q[1] - q[2] * q[0]);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    pitch = -std::asin(t2);

    // yaw (z-axis rotation)
    float t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
    float t4 = +1.0 - 2.0 * (q[1]*q[1] + q[2] * q[2]);
    yaw = std::atan2(t3, t4);
}

void GeoLocation(void)
{
    phi   = Cur_Att_rad[0];
    theta = Cur_Att_rad[1];
    psi   = Cur_Att_rad[2];

    R11 = cos(psi)*cos(refangle)*cos(theta) - cos(phi)*sin(psi)*sin(refangle) + cos(psi)*sin(phi)*sin(refangle)*sin(theta);
    R12 = cos(psi)*cos(refangle)*sin(phi)*sin(theta) - cos(psi)*cos(theta)*sin(refangle) - cos(phi)*cos(refangle)*sin(psi);
    R13 = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);

    R21 = cos(phi)*cos(psi)*sin(refangle) + cos(refangle)*cos(theta)*sin(psi) + sin(phi)*sin(psi)*sin(refangle)*sin(theta);
    R22 = cos(phi)*cos(psi)*cos(refangle) - cos(theta)*sin(psi)*sin(refangle) + cos(refangle)*sin(phi)*sin(psi)*sin(theta);
    R23 = cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);

    R31 = cos(theta)*sin(phi)*sin(refangle) - cos(refangle)*sin(theta);
    R32 = sin(refangle)*sin(theta) + cos(refangle)*cos(theta)*sin(phi);
    R33 = cos(phi)*cos(theta);

    //
    // image data low pass filter
    //
    u = (float)(detection.pos_x - CAMERA_PARAM_U0)*(-1);
    v = (float)(detection.pos_y - CAMERA_PARAM_V0);

    rho = sqrt(u*u + v*v);
    F = a0 + a1*rho + a2*pow(rho,2) + a3*pow(rho,3) + a4*pow(rho,4);

    h_ref = Cur_Pos_m[2];

    s = (R31*u + R32*v + R33*F)/h_ref;
    x_tar = (R11*u + R12*v + R13*F)/s;
    y_tar = (R21*u + R22*v + R23*F)/s;

    //tar_data.pos[0] = Cur_Pos_m[0] + x_tar*cos(psi) - y_tar*sin(psi);
    //tar_data.pos[1] = Cur_Pos_m[1] + x_tar*sin(psi) + y_tar*cos(psi);
    tar_data.pos[0] = Cur_Pos_m[0] + x_tar;
    tar_data.pos[1] = Cur_Pos_m[1] + y_tar;

    //tar_data.pos[2] = nav_data.posNED[1] - tar_data.dist;
}

#endif
