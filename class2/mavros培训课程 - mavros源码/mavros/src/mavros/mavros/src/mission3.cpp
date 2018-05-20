/*
 * mission3.cpp
 *
 * Author: Qyp
 *
 * Time: 2018.5.10
 *
 */
//ros头文件
#include <ros/ros.h>

//topic 头文件
#include <mavros_msgs/Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Point.h>

//其他头文件
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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
geometry_msgs::PoseStamped pos_drone;                   //无人机当前位置（从飞控中读取）
geometry_msgs::TwistStamped vel_drone;                  //无人机当前速度（从飞控中读取）
float PIX_Euler[3];                                     //无人机当前欧拉角(从飞控中读取)

geometry_msgs::Pose origin_point;
float yaw_origin;

geometry_msgs::Pose vision_data1;
geometry_msgs::Pose vision_data2;
geometry_msgs::Pose vision_data3;

int Num_StateMachine = 0;                                 // 状态机编号
int flag_vision_1 = 0;                                    // vision flag1
int flag_vision_2 = 0;                                    // vision flag2


geometry_msgs::Pose leader_position;

mavros_msgs::Command Command_now;                      //无人机当前执行命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角
void euler_2_quaternion(float angle[3], float quat[4]);
float abs_distance(float pos_sp[3], float pos_drone_now[3]);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone = *msg;
}

void euler_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    float q[4];
    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;
    quaternion_2_euler(q, PIX_Euler);
}

void origin_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    origin_point = *msg;

    float q[4];
    float euler_origin[3];
    q[0] = origin_point.orientation.w;
    q[1] = origin_point.orientation.x;
    q[2] = origin_point.orientation.y;
    q[3] = origin_point.orientation.z;
    quaternion_2_euler(q, euler_origin);

    yaw_origin = euler_origin[2];
}


void vision_cb1(const geometry_msgs::Pose::ConstPtr &msg)
{
    vision_data1 = *msg;
}

void vision_cb2(const geometry_msgs::Pose::ConstPtr &msg)
{
    vision_data2 = *msg;
}

void vision_cb3(const geometry_msgs::Pose::ConstPtr &msg)
{
    vision_data3 = *msg;
}

void leader_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    leader_position = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission3");
    ros::NodeHandle nh;

    // 【订阅】无人机当前位置 坐标系:NED系 [这里订阅的位置来自于飞控（飞控根据GPS、IMU等解算的自身位置）]
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:NED系 [这里订阅的速度来自于飞控（飞控根据GPS、IMU等解算的自身速度）]
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 10, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:NED系
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, euler_cb);

    // 【订阅】起飞原点 坐标系:NED系
    ros::Subscriber origin_sub = nh.subscribe<geometry_msgs::Pose>("/origin_point", 10, origin_cb);

    // 【订阅】视觉消息1
    ros::Subscriber vision_sub1 = nh.subscribe<geometry_msgs::Pose>("/vision1", 10, vision_cb1);

    // 【订阅】视觉消息2
    ros::Subscriber vision_sub2 = nh.subscribe<geometry_msgs::Pose>("/vision2", 10, vision_cb2);

    // 【订阅】视觉消息3
    ros::Subscriber vision_sub3 = nh.subscribe<geometry_msgs::Pose>("/vision3", 10, vision_cb3);

    // 【订阅】领机位置
    ros::Subscriber leader_sub = nh.subscribe<geometry_msgs::Pose>("/leader_postion", 10, leader_cb);

    // 【发布】发送给position_control_outdoor.cpp的命令
    ros::Publisher command_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);

    // 频率 [50Hz]
    ros::Rate rate(50.0);


    while (ros::ok())
    {
        switch (Num_StateMachine)
        {
            // 起飞
            case 0:
                //因为默认是起飞到一米高度的指令，所以这里只需要判定是否成功起飞即可

                static int num_count = 0;

                float pos_sp[3];
                float pos_drone_now[3];

                pos_sp[0] = origin_point.position.x;
                pos_sp[1] = origin_point.position.y;
                pos_sp[2] = origin_point.position.z;

                pos_drone_now[0] = pos_drone.pose.position.x;
                pos_drone_now[1] = pos_drone.pose.position.y;
                pos_drone_now[2] = pos_drone.pose.position.z;

                if(abs_distance(pos_sp,pos_drone_now) < 0.1)
                {
                    num_count++;
                }else
                {
                    num_count = 0;
                }

                if(abs_distance(pos_sp,pos_drone_now) < 0.1 && num_count >100)
                {
                    Num_StateMachine == 1;
                }
                break;

            // 爬升至目标高度
            case 1:

                static int comid1 = 0;

                Command_now.command = Move;
                Command_now.comid = comid1;
                Command_now.sub_mode = 0; // xy position z position
                Command_now.pos_sp[0] =  origin_point.position.x;
                Command_now.pos_sp[1] =  origin_point.position.y;
                Command_now.pos_sp[2] =  origin_point.position.z - 50;   //飞到50米高度？
                Command_now.yaw_sp = yaw_origin;

                comid1++;

                pos_sp[0] = origin_point.position.x;
                pos_sp[1] = origin_point.position.x;
                pos_sp[2] = origin_point.position.x - 50;

                if(abs_distance(pos_sp,pos_drone_now) < 0.1)
                {
                    num_count++;
                }else
                {
                    num_count = 0;
                }

                if(abs_distance(pos_sp,pos_drone_now) < 0.1 && num_count >100)
                {
                    Num_StateMachine == 2;
                }

                break;

            // 飞行至对接区域附近,目前的策略为直线飞行，后续根据实际情况决定。
            //有可能需要根据给定的对接区域GPS信息去解算对接区域的XYZ
            case 2:

                Num_StateMachine == 3;

            break;

            // 等待
            case 3:

                Num_StateMachine == 4;
            break;

            // 领飞无人机接近，进入追踪路径
            case 4:

                Num_StateMachine == 5;

            break;

            // 识别领飞无人机
            case 5:
                static int comid5 = 0;
                float k1,k2,k3;
                k1 = 1;
                k2 = 1;
                k3 = 1;

                //此处的参数一定是需要做一定量的实验才能确定的了得，不然没法保证此段代码能够追踪上目标

                Command_now.command = Moving_Body;
                Command_now.comid = comid5;
                Command_now.sub_mode = 3; // xy velocity z velocity
                Command_now.vel_sp[0] =  k1 * vision_data1.position.x;
                Command_now.vel_sp[1] =  k2 * vision_data1.position.y;
                Command_now.vel_sp[2] =  k3 * vision_data1.position.z;
                Command_now.yaw_sp = yaw_origin;  //这里偏航角也需要重新计算

                comid5++;

                //判断进入6的逻辑 需要自行考虑
                Num_StateMachine == 6;

            break;

            // 识别红色靶标，并尝试维持在任务区域2秒
            case 6:
                //模仿5写

                //判断任务已经结束的逻辑 需要自行考虑
                Num_StateMachine == 7;

                break;

            // 任务完成 - 返航
            case 7:

                rate.sleep();

                break;
            }


        command_pub.publish(Command_now);
        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}


float abs_distance(float pos_sp[3], float pos_drone_now[3])
{
    float abs_distance;
    abs_distance = sqrt((pos_sp[0] - pos_drone_now[0]) * (pos_sp[0] - pos_drone_now[0]) + (pos_sp[1] - pos_drone_now[1]) * (pos_sp[1] - pos_drone_now[1]) + (pos_sp[2] - pos_drone_now[2]) * (pos_sp[2] - pos_drone_now[2]));
    return abs_distance;
}

// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
   // angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
}


// Euler转四元数
// q0 q1 q2 q3
// w x y z
void euler_2_quaternion(float angle[3], float quat[4])
{
    double cosPhi_2 = cos(double(angle[0]) / 2.0);

    double sinPhi_2 = sin(double(angle[0]) / 2.0);

    double cosTheta_2 = cos(double(angle[1] ) / 2.0);

    double sinTheta_2 = sin(double(angle[1] ) / 2.0);

    double cosPsi_2 = cos(double(angle[2]) / 2.0);

    double sinPsi_2 = sin(double(angle[2]) / 2.0);


    quat[0] = float(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);

    quat[1] = float(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);

    quat[2] = float(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);

    quat[3] = float(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);

}


