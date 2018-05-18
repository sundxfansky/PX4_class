/*
 * position_control.cpp
 *
 * Author: Qyp
 *
 * Time: 2018.3.25
 *
 * 说明: mavros位置控制示例程序
 *      1. 订阅导航信息
 *      2. 位置环串级PID
 *      2. 发布加速度期望值给飞控
 */
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

#include <ros/ros.h>
#include <pid.h>
#include <param.h>

//topic
#include <mavros_msgs/Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
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


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//自定义的Command变量
//相应的命令分别为 待机 起飞 悬停 降落 移动(惯性系ENU) 上锁 移动(机体系)
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

//无人机执行的指令,由其他上层模块发布
mavros_msgs::Command Command_Now;                      //无人机当前执行命令
mavros_msgs::Command Command_Last;                     //无人机上一条执行命令

mavros_msgs::State current_state;                       //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

//无人机的位置和速度,
//从position_estimator.cpp模块中读取(位置和速度来源于自主定位模块,例如激光雷达或VINS);
//也可以直接从飞控中读取(/mavros/local_position/pose /mavros/local_position/velocity)
geometry_msgs::Pose pos_drone;                          //无人机当前位置 姿态
geometry_msgs::Pose vel_drone;                          //无人机当前速度


float PIX_Euler[3];                                     //无人机当前欧拉角(从飞控中读取)
float PIX_Euler_target[3];                              //无人机当前期望欧拉角(从飞控中读取)
float Thrust_target;                                    //期望推力
float Yaw_Locked;                                       //锁定的偏航角(一般锁定为0)

mavros_msgs::PositionTarget pos_setpoint;               //即将发给无人机的控制指令  ----  加速度设定值，偏航角设定值 (发送给飞控,通过setpoint_saw.cpp)



//声明PID类
PID PIDX, PIDY, PIDZ, PIDVX, PIDVY ,PIDVZ;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角
float get_ros_time(ros::Time begin);                                                 //获取ros当前时间
int pix_controller(float cur_time);                                                  //控制程序
void prinft_drone_state(float current_time);                                         //打印无人机状态函数
void prinft_command_state();                                                         //打印控制指令函数
void rotation_yaw(float yaw_angle, float input[2], float output[2]);                 //坐标转换

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pos_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    pos_drone = *msg;
}

void vel_cb(const geometry_msgs::Pose::ConstPtr &msg)
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
//命令回调函数
void Command_cb(const mavros_msgs::Command::ConstPtr& msg)
{
    Command_Now = *msg;
}


void euler_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    float q[4];
    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;
    quaternion_2_euler(q, PIX_Euler_target);

    Thrust_target = msg->thrust;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    // 【订阅】无人机当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 【订阅】指令 - 来自上层模块的指令
    ros::Subscriber Command_sub = nh.subscribe<mavros_msgs::Command>("mavros/Command", 10, Command_cb);

    // 【订阅】无人机当前位置 坐标系:NED系 [室外：GPS，室内：自主定位或mocap等]
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::Pose>("/drone/pos", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:NED系 [室外：GPS，室内：自主定位或mocap等]
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::Pose>("/drone/vel", 10, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:NED系
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, euler_cb);

    // 【订阅】无人机期望欧拉角 坐标系:NED系
    ros::Subscriber euler_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/target_attitude", 10, euler_target_cb);

    // 【发布】加速度期望值 坐标系 NED系
    ros::Publisher accel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    // 【服务】解锁上锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // 【服务】修改系统模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;


    // 频率 [50Hz]
    ros::Rate rate(50.0);


    // 读取参数 PID参数 等
    parameter PIDparam;
    string paraadr("/home/nuc/mavros/param_pid");
    if (PIDparam.readParam(paraadr.c_str()) == 0){
        std::cout<<"read config file error!"<<std::endl;
        return 0;
    }

    // 设置PID参数 比例参数 积分参数 微分参数 微分项滤波增益
    PIDX.setPID(PIDparam.x_p, PIDparam.x_i, PIDparam.x_d, PIDparam.x_f);
    PIDY.setPID(PIDparam.y_p, PIDparam.y_i, PIDparam.y_d, PIDparam.y_f);
    PIDZ.setPID(PIDparam.z_p, PIDparam.z_i, PIDparam.z_d, PIDparam.z_f);
    PIDVX.setPID(PIDparam.vx_p, PIDparam.vx_i, PIDparam.vx_d, PIDparam.vx_f);
    PIDVY.setPID(PIDparam.vy_p, PIDparam.vy_i, PIDparam.vy_d, PIDparam.vy_f);
    PIDVZ.setPID(PIDparam.vz_p, PIDparam.vz_i, PIDparam.vz_d, PIDparam.vz_f);

    // 设置积分上限 控制量最大值 误差死区
    PIDX.set_sat(2, 6, 0.0);
    PIDY.set_sat(2, 6, 0.0);
    PIDZ.set_sat(2, 6, 0.0);
    PIDVX.set_sat(2, 3, 0);
    PIDVY.set_sat(2, 3, 0);
    PIDVZ.set_sat(2, 4, 0);

    // 等待和飞控的连接
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not Connected");
    }

    //连接成功
    ROS_INFO("Connected!!");

    //偏航角锁定值，一般情况下 直接锁死 ； 如果需要用到也可以控
    Yaw_Locked = 0;


    //初始化命令-
    // 默认设置：move模式 子模式：位置控制 起飞到 0,0，-1这个点
    Command_Now.comid = 0;
    Command_Now.command = Move;
    Command_Now.sub_mode = 0;
    Command_Now.pos_sp[0] = 0;          //NED Frame
    Command_Now.pos_sp[1] = 0;          //NED Frame
    Command_Now.pos_sp[2] = -1;         //NED Frame
    Command_Now.vel_sp[0] = 0;          //NED Frame
    Command_Now.vel_sp[1] = 0;          //NED Frame
    Command_Now.vel_sp[2] = 0;          //NED Frame
    Command_Now.yaw_sp = Yaw_Locked;

    //初始化悬停的位置值
    float Hold_position_NED[3];

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

        // 当前时间
        float cur_time = get_ros_time(begin_time);

        switch (Command_Now.command)
        {
        // 待机状态 [默认模式,无人机等待命令,上锁状态,自稳模式]
        case Standby:

            pos_setpoint.type_mask = (2 << 10) | (7 << 3) | (7 << 0);  // 100 000 111 111   加速度 + yaw
            pos_setpoint.acceleration_or_force.x = 0;
            pos_setpoint.acceleration_or_force.y = 0;
            pos_setpoint.acceleration_or_force.z = 0;
            pos_setpoint.yaw = 0*3.1415/180.0;               //注意 : 单位为弧度

            //状态打印
            cout << "Command: [ Standby ] " <<endl;
            prinft_drone_state(cur_time);
            prinft_command_state();
            break;

        // 悬停状态 [无人机悬停于当前点]
        case Hold:
            // 如果上一次执行的命令不是Hold，记录此时进入的飞机位置（即第一次执行hold指令时飞机的位置）
            if (Command_Last.command != 2)
            {
              Hold_position_NED[0] = pos_drone.position.x;
              Hold_position_NED[1] = pos_drone.position.y;
              Hold_position_NED[2] = pos_drone.position.z;
            }

            Command_Now.sub_mode = 0;
            Command_Now.pos_sp[0] = Hold_position_NED[0];
            Command_Now.pos_sp[1] = Hold_position_NED[1];
            Command_Now.pos_sp[2] = Hold_position_NED[2];
            Command_Now.vel_sp[0] = 0;
            Command_Now.vel_sp[1] = 0;
            Command_Now.vel_sp[2] = 0;
            Command_Now.yaw_sp = Yaw_Locked ;

            pix_controller(cur_time);

            prinft_drone_state(cur_time);
            prinft_command_state();

            break;

        // 起飞
        case Takeoff:

        //降落
        case Land:

        //上锁 ： 用于1 紧急上锁 2 任务结束： 安全降落
        case Disarm:
            if(current_state.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);

            }
            if(current_state.mode == "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);
            }


            if (arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }
            //状态打印
            prinft_drone_state(cur_time);
            prinft_command_state();

        //惯性系移动
        case Move:

            Command_Now.yaw_sp = Yaw_Locked ;
            //控制程序
            pix_controller(cur_time);

            //状态打印
            prinft_drone_state(cur_time);
            prinft_command_state();

            break;

        //机体系移动
        //需要先将机体系坐标转化到惯性系
        case Moving_Body:

            //记录进入时,飞机的高度
            static double desire_position_NED[3] = {0, 0, pos_drone.position.z};
            static float desire_velocity_NED[3] = {0, 0, 0};
            static bool pz_is_nzero = 1;

            //只有在comid增加时才会进入解算
            if( Command_Now.comid  !=  Command_Last.comid )
            {
              //xy velocity mode
              if( Command_Now.sub_mode & 0b10 )
              {
                float d_vel_body[2] = {Command_Now.vel_sp[0], Command_Now.vel_sp[1]};         //the desired xy velocity in Body Frame
                float d_vel_ned[2];                                                           //the desired xy velocity in NED Frame
                //机体系到NED系
                rotation_yaw( PIX_Euler[2], d_vel_body, d_vel_ned);
                desire_velocity_NED[0] = d_vel_ned[0];
                desire_velocity_NED[1] = d_vel_ned[1];
              }
              //xy position mode
              else
              {
                float d_pos_body[2] = {Command_Now.pos_sp[0], Command_Now.pos_sp[1]};         //the desired xy position in Body Frame
                float d_pos_ned[2];                                                           //the desired xy position in NED Frame (The origin point is the drone)
                rotation_yaw( PIX_Euler[2], d_pos_body, d_pos_ned);

                desire_position_NED[0] = pos_drone.position.x + d_pos_ned[0];
                desire_position_NED[1] = pos_drone.position.y + d_pos_ned[1];                           //在当前位置上累加

              }

              //z velocity mode
              if( Command_Now.sub_mode & 0b01 )
              {
                desire_velocity_NED[2] = Command_Now.vel_sp[2];
              }
              //z position mode
              else
              {
                if(Command_Now.pos_sp[2] != 0)
                {
                  // the pos_sp[2] is not zero
                  pz_is_nzero = 1;
                  desire_position_NED[2] = pos_drone.position.z + Command_Now.pos_sp[2];
                }
                else
                {
                  //??
                  if( (Command_Now.pos_sp[2] == 0) && (pz_is_nzero == 1) )
                  {
                    // the pos_sp[2] is not zero
                    pz_is_nzero = 0;
                    desire_position_NED[2] = pos_drone.position.z + Command_Now.pos_sp[2];
                  }
                }

              }

            }

            Command_Now.pos_sp[0] = desire_position_NED[0];
            Command_Now.pos_sp[1] = desire_position_NED[1];
            Command_Now.pos_sp[2] = desire_position_NED[2];
            Command_Now.vel_sp[0] = desire_velocity_NED[0];
            Command_Now.vel_sp[1] = desire_velocity_NED[1];
            Command_Now.vel_sp[2] = desire_velocity_NED[2];
            Command_Now.yaw_sp = Yaw_Locked ;

            prinft_drone_state(cur_time);
            prinft_command_state();
            pix_controller(cur_time);

            break;
        }

        Command_Last = Command_Now;

        accel_pub.publish(pos_setpoint);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;

}

//旋转函数 [机体系到NED系]input是机体系,output是惯性系
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

//获取当前时间 单位：秒
float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
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
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>控 制 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int pix_controller(float cur_time)
{
    // 读取控制子模式
    int mode_  = Command_Now.sub_mode;

    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDX.start_intergrate_flag = 1;
    PIDY.start_intergrate_flag = 1;
    PIDZ.start_intergrate_flag = 1;
    PIDVX.start_intergrate_flag = 1;
    PIDVY.start_intergrate_flag = 1;
    PIDVZ.start_intergrate_flag = 1;
    if(current_state.mode != "OFFBOARD")
    {
        PIDX.start_intergrate_flag = 0;
        PIDY.start_intergrate_flag = 0;
        PIDZ.start_intergrate_flag = 0;
        PIDVX.start_intergrate_flag = 0;
        PIDVY.start_intergrate_flag = 0;
        PIDVZ.start_intergrate_flag = 0;
    }
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>位 置 环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // xy通道
    if((mode_>>1) == 0)
    {
        //xy位置控制
        //计算误差
        float error_x = Command_Now.pos_sp[0] - pos_drone.position.x;
        float error_y = Command_Now.pos_sp[1] - pos_drone.position.y;

        //传递误差
        PIDX.add_error(error_x, cur_time);
        PIDY.add_error(error_y, cur_time);

        //计算输出
        PIDX.pid_output();
        PIDY.pid_output();
    }
    else
    {
        //xy速度控制
        PIDX.Output = Command_Now.vel_sp[0];
        PIDY.Output = Command_Now.vel_sp[1];
    }

    // z通道
    if((mode_& 0b01) == 0)
    {
        //z位置控制
        //计算误差
        float error_z = Command_Now.pos_sp[2] - pos_drone.position.z;
        //传递误差
        PIDZ.add_error(error_z, cur_time);
        //计算输出
        PIDZ.pid_output();
    }
    else
    {
        //z速度控制
        PIDZ.Output = Command_Now.vel_sp[2];
    }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>速 度 环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //计算误差
    float error_vx = PIDX.Output - vel_drone.position.x;
    float error_vy = PIDY.Output - vel_drone.position.y;
    float error_vz = PIDZ.Output - vel_drone.position.z;
    //传递误差
    PIDVX.add_error(error_vx, cur_time);
    PIDVY.add_error(error_vy, cur_time);
    PIDVZ.add_error(error_vz, cur_time);
    //计算输出
    PIDVX.pid_output();
    PIDVY.pid_output();
    PIDVZ.pid_output();

    //赋值到期望加速度
    float trim_az = -0.3;      //平衡点推力值 - 平衡推力的选取(手飞大概选取，稍微选小0.01)

    pos_setpoint.type_mask = (2 << 10) | (7 << 3) | (7 << 0);  // 100 000 111 111   加速度 + yaw

    //注意这里我经常遇到一个坑 x和y莫名其妙会交换。。找不到原因
    pos_setpoint.acceleration_or_force.x = PIDVX.Output;   //x轴加速度控制指令
    pos_setpoint.acceleration_or_force.y = PIDVY.Output;   //y轴加速度控制指令
    pos_setpoint.acceleration_or_force.z = (trim_az + PIDVZ.Output);   //Z轴加速度控制指令
    pos_setpoint.yaw = Command_Now.yaw_sp * 3.1415926/180;  //偏航角控制指令

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>打   印   数   据<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Control result [ax ay az]: " << pos_setpoint.acceleration_or_force.x << "  "<< pos_setpoint.acceleration_or_force.y <<" " << pos_setpoint.acceleration_or_force.z <<endl;
    cout << "Control result [psai]: " << pos_setpoint.yaw*180/3.1415 <<endl;

    return 0;
}


void prinft_drone_state(float current_time)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] "<<endl;

    //是否和飞控建立起连接
    if (current_state.connected == true)
    {
        cout << "Connected State: [ "<< "Connected" <<" ]" <<endl;
    }
    else
    {
        cout << "Connected State: [ "<< "Unconnected" <<" ]" <<endl;
    }

    //是否上锁
    if (current_state.armed == true)
    {
        cout << "Connected State: [ "<< "Armed" <<" ]" <<endl;
    }
    else
    {
        cout << "Connected State: [ "<< "DisArmed" <<" ]" <<endl;
    }

    cout << "Mode : [ " << current_state.mode<<" ]" <<endl;

    cout << "Position: [X Y Z] : " << " " << pos_drone.position.x << " [m] "<< pos_drone.position.y<<" [m] "<<pos_drone.position.z<<" [m] "<<endl;
    cout << "Velocity: [X Y Z] : " << " " << vel_drone.position.x << " [m/s] "<< vel_drone.position.y<<" [m/s] "<<vel_drone.position.z<<" [m/s] "<<endl;

    cout << "Attitude: [roll pitch yaw] : " << PIX_Euler[0] * 180/3.1415 <<" [°] "<<PIX_Euler[1] * 180/3.1415 << " [°] "<< PIX_Euler[2] * 180/3.1415<<" [°] "<<endl;

    cout << "Attitude_target: [roll pitch yaw] : " << PIX_Euler_target[0] * 180/3.1415 <<" [°] "<<PIX_Euler_target[1] * 180/3.1415 << " [°] "<< PIX_Euler_target[2] * 180/3.1415<<" [°] "<<endl;

    cout << "Thrust_target[0 - 1] : " << Thrust_target <<endl;

}

void prinft_command_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    switch(Command_Now.command)
    {
    case Standby:
        cout << "Command: [ Standby ] " <<endl;
        break;
    case Hold:
        cout << "Command: [ Hold ] " <<endl;
        break;
    case Move:
        cout << "Command: [ Move ] " <<endl;
        break;
    case Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        break;
    case Land:
        cout << "Command: [ Land ] " <<endl;
        break;
    case Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;
    case Moving_Body:
        cout << "Command: [ Moving_Body ] " <<endl;
        break;
    }

    int sub_mode;
    sub_mode = Command_Now.sub_mode;
    if((sub_mode>>1) == 0) //xy channel
    {
        cout << "Submode: xy position control "<<endl;
        cout << "X_desired : " << Command_Now.pos_sp[0] << " [m]"   <<  "  Y_desired : " << Command_Now.pos_sp[1] << " [m]"   <<endl;
    }
    else{
        cout << "Submode: xy velocity control "<<endl;
        cout << "X_desired : " << Command_Now.vel_sp[0] << " [m/s]" << "  Y_desired : "  << Command_Now.vel_sp[1] << " [m/s]" <<endl;
    }
    if((sub_mode & 0b01) == 0) //z channel
    {
        cout << "Submode:  z position control "<<endl;
        cout << "Z_desired : "<< Command_Now.pos_sp[2] << " [m]" << endl;
    }
    else
    {
        cout << "Submode:  z velocity control -- Z_desired [NED] : "<< Command_Now.vel_sp[2] <<" [m/s]"<<endl;
        cout << "Z_desired : "<< Command_Now.vel_sp[2] << " [m/s]" <<endl;
    }

    cout << "Yaw is locked. " <<endl;
    cout << "Yaw Desired:  "  << Command_Now.yaw_sp << " [°] "  << "Current Yaw: "<< PIX_Euler[2]/ 3.1415926 *180 << " [°]  "<<endl;

}


