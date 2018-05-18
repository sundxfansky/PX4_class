#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/Command.h>
using namespace std;

enum Command
{
    Standby = 0,
    Takeoff,
    Hold,
    Land,
    Move,
    Disarm,
    Moving_Body
};

float flag_vision;
mavros_msgs::Command Command_now;
geometry_msgs::PoseStamped pos_drone;                    //无人机当前位置 姿态 
geometry_msgs::Point relative_position;                 //机体固连坐标系下 降落板的位置

void relative_position_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    relative_position.x = -msg->x;
    relative_position.y = msg->y;
    relative_position.z = msg->z;
}

void relative_position_flag_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    flag_vision = msg->x;
}


void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_land");
    ros::NodeHandle nh;
    //10hz
    ros::Rate loopRate(10);

    //降落板与无人机的相对位置
    ros::Subscriber relative_pos_sub = nh.subscribe<geometry_msgs::Point>("/relative_position", 10, relative_position_cb);

    //视觉flag
    ros::Subscriber relative_pos_flag_sub = nh.subscribe<geometry_msgs::Point>("/relative_position_flag", 10, relative_position_flag_cb);

    // 订阅无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/drone/pos", 10, pos_cb);

    // 发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);

    int comid = 1;
    int flag_x = 0;
    int flag_y = 0;
    int flag_z = 0;
    float kx = 0.3;
    float ky = 0.3;
    float kz = 0.0001;


    while(ros::ok())
    {

	Command_now.command = 6;
	Command_now.comid = comid;
	comid++;
	Command_now.sub_mode = 3; // xy velocity z position
	Command_now.vel_sp[0] =  kx * (relative_position.x + 0.1);
	Command_now.vel_sp[1] =  ky * relative_position.y;
	Command_now.vel_sp[2] =  kz * (relative_position.z - 0.4);
	Command_now.yaw_sp = 0 ;

	
        //判断是否满足降落条件 死区
	if(abs(relative_position.x ) < 0.08)
	{
           flag_x = 1;
	   Command_now.vel_sp[0] = 0;
	}else
	{
	   flag_x = 0;
	}

	if(abs(relative_position.y ) < 0.08)
	{
           flag_y = 1;
	   Command_now.vel_sp[1] = 0;
	}else
	{
	   flag_y = 0;
	}

	if(abs(relative_position.z - 0.3) < 0.1)
	{
           flag_z = 1;
	   Command_now.vel_sp[2] = 0;	
	}else
	{
	   flag_z = 0;
	}

	
	if(flag_vision == 0)
	{
		Command_now.sub_mode = 1; // xy velocity z position
		Command_now.pos_sp[0] =  0;
		Command_now.pos_sp[1] =  0;
		Command_now.vel_sp[2] =  -0.1;
		Command_now.yaw_sp = 0 ;

	}
	if(flag_x == 1 && flag_y == 1 && flag_z == 1)
	{
           Command_now.command = 5;
        }
	
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

	cout << "flag_vision: " << flag_vision <<endl;
	cout << "relative position: " << relative_position.x << " [m] "<< relative_position.y << " [m] "<< relative_position.z << " [m] "<<endl;
	
	cout << "flag: " << flag_x << " "<< flag_y << "  "<< flag_z << " "<<endl;

	cout << "command: " << Command_now.vel_sp[0] << " [m/s] "<< Command_now.vel_sp[1] << " [m/s] "<< Command_now.vel_sp[2] << " [m/s] "<<endl;

	//Command_now.command = Disarm;


    command_pub.publish(Command_now);
    ros::spinOnce();
    loopRate.sleep();


    }
    return 0;

}


