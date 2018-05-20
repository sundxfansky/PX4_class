/*
 * position_estimator_laser.cpp
 *
 * Author: Qyp
 *
 * Time: 2017.10.25
 *
 */


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
#include<iomanip>
using namespace std;
//topic
#include <mavros_msgs/Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
/*
 * 主要功能:
 * 根据laser获得无人机当前位置和偏航角,
 * 位置 和 速度信息发送给position_control.cpp
 * 偏航角要用mocap发给飞控，作为飞控的偏航角
 *
 */


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

geometry_msgs::Pose pos_drone;                          //无人机当前位置 (来自laser)
geometry_msgs::Pose pos_drone_last;                     //无人机上一时刻位置 (来自laser)
geometry_msgs::Pose vel_drone;                          //无人机当前速度 (来自laser)
geometry_msgs::Pose euler;                              //无人机当前姿态 (来自laser)

geometry_msgs::PoseStamped yaw;                         //发送给飞控的偏航角

geometry_msgs::TransformStamped laser_last;
sensor_msgs::LaserScan Laser;                          //激光雷达点云数据
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角[与PX4相同]
void euler_2_quaternion(float angle[3], float quat[4]);                              //欧拉角转四元数
float get_dt(ros::Time last);                                                        //获取时间间隔
void printf();                                                                       //打印函数
void save_flight_data(std::ofstream& out_file, float timenow);                       //储存数据函数

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void laser_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    //确定是cartographer发出来的/tf信息
    //有的时候/tf这个消息的发布者不止一个
    if (msg->transforms[0].header.frame_id == "map")
    {
        geometry_msgs::TransformStamped laser;

        laser = msg->transforms[0];

        float dt;

        dt = (laser.header.stamp.sec - laser_last.header.stamp.sec) + (laser.header.stamp.nsec - laser_last.header.stamp.nsec)/10e9;


        if (dt != 0)
        {
            dt = 0.1;  //SLAM算法发布位置的频率大概为10Hz ， 也可以用上面的dt，但不如这样效果好

            //位置 xy
            pos_drone.position.x = laser.transform.translation.x;
            pos_drone.position.y = - laser.transform.translation.y;

            //速度 vx vy [平滑处理]
            float vel_now[2];

            vel_now[0] = (pos_drone.position.x - pos_drone_last.position.x) / dt;
            vel_now[1] = (pos_drone.position.y - pos_drone_last.position.y) / dt;

            vel_drone.position.x = 0*vel_drone.position.x + 1*vel_now[0];
            vel_drone.position.y = 0*vel_drone.position.y + 1*vel_now[1];

            //储存为上一时刻数据
            pos_drone_last.position.x = pos_drone.position.x;
            pos_drone_last.position.y = pos_drone.position.y;

            laser_last = laser;

            //欧拉角
            float euler_laser[3];
            float q[4];
            q[0] = laser.transform.rotation.w;
            q[1] = laser.transform.rotation.x;
            q[2] = laser.transform.rotation.y;
            q[3] = laser.transform.rotation.z;
            quaternion_2_euler(q, euler_laser);

            //laser系 到 NED系
            euler_laser[0] = euler_laser[0];
            euler_laser[1] = - euler_laser[1];
            euler_laser[2] = - euler_laser[2];

            //转换会四元数形式,并储存到yaw,发送给飞控
            euler_2_quaternion(euler_laser, q);

            yaw.pose.orientation.w = q[0];
            yaw.pose.orientation.x = q[1];
            yaw.pose.orientation.y = q[2];
            yaw.pose.orientation.z = q[3];


            //这个仅做储存用
            euler.position.x = euler_laser[0];
            euler.position.y = euler_laser[1];
            euler.position.z = euler_laser[2];
        }
    }
}
void sonic_cb(const std_msgs::UInt16::ConstPtr& msg)
{
    std_msgs::UInt16 sonic;

    sonic = *msg;

    //位置
    pos_drone.position.z  = - (float)sonic.data / 1000;

    //速度 [平滑处理]
    float vel_now;
    float dt;

    dt = 0.1;

    vel_now = (pos_drone.position.z  - pos_drone_last.position.z ) / dt;

    vel_drone.position.z = 0*vel_drone.position.z + 1*vel_now ;

    pos_drone_last.position.z = pos_drone.position.z;

}

//void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
//{
//    Laser = *msg;
//}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_estimator_laser");
    ros::NodeHandle nh;

    // 【订阅】cartographer估计位置
    ros::Subscriber laser_sub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1000, laser_cb);

    // 【订阅】Lidar数据
   // ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);

    // 【发布】Lidar数据
    //ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/lidar/scan", 100);

    // 【订阅】超声波的数据
    ros::Subscriber sonic_sub = nh.subscribe<std_msgs::UInt16>("/sonic", 100, sonic_cb);

    // 【发布】无人机位置 坐标系 NED系
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Pose>("drone/pos", 10);

    // 【发布】无人机速度 坐标系 NED系
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Pose>("drone/vel", 10);

    // 【发布】无人机位置和偏航角 坐标系 NED系 [借助MOCAP的mavlink消息将外部测到的位置yaw角发布给飞控]
    ros::Publisher yaw_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 100);
    yaw.pose.position.x = 200;
    yaw.pose.position.y = -200;
    // 频率 [50Hz]
    ros::Rate rate(50.0);

    // 启控时间
    ros::Time begin_time = ros::Time::now();


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>储存数据<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    time_t tt = time(NULL);
    tm* t = localtime(&tt);
    char iden_path[256];
    sprintf(iden_path, "/home/nvidia/mavros/data/position-estimator-laser-%d-%02d-%02d_%02d-%02d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min);
    std::ofstream out_data_file(iden_path);

    if (!out_data_file)
    {
        std::cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " pos_drone.x " << " pos_drone.y " << " pos_drone.z " \
                                << " vel_drone.x " << " vel_drone.y " << " vel_drone.z " \
                                << " yaw_laser " \
                                <<std::endl;
    }


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        float cur_time = get_dt(begin_time);

        //回调一次 更新传感器状态
        ros::spinOnce();

        //发布
        pos_pub.publish(pos_drone);
        vel_pub.publish(vel_drone);
        yaw_pub.publish(yaw);
       // lidar_pub.publish(Laser);

        //打印
        printf();

        //储存数据
        save_flight_data(out_data_file,cur_time);


        rate.sleep();

    }

    return 0;

}

void save_flight_data(std::ofstream& out_file, float timenow)
{
    out_file << timenow <<"  "<< pos_drone.position.x <<"  "<< pos_drone.position.y <<"  "<< pos_drone.position.z <<"  "\
                              << vel_drone.position.x <<"  "<< vel_drone.position.y <<"  "<< vel_drone.position.z <<"  "\
                              << euler.position.z * 180/3.1415  <<"  "\
                              << std::endl;
}

//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    //angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
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

void printf()
{

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>Position Estimator<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout.setf(ios::fixed);


    cout << "Position: [X Y Z] : " << " " << fixed <<setprecision(5)<< pos_drone.position.x << " [m] "<< pos_drone.position.y<<" [m] "<< pos_drone.position.z<<" [m] "<<endl;
    cout << "Velocity: [X Y Z] : " << " " << fixed <<setprecision(5)<< vel_drone.position.x << " [m/s] "<< vel_drone.position.y<<" [m/s] "<< vel_drone.position.z<<" [m/s] "<<endl;
    cout << "Yaw_laser:  : " << fixed <<setprecision(5)<< euler.position.z * 180/3.1415 <<"   "<<endl;

}
