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
#include <mavros_msgs/Command.h>
using namespace std;

ros::Publisher move_pub;
// enum Command
// {
//     Standby = 0,
//     Takeoff,
//     Hold,
//     Land,
//     Move,
//     Disarm
// };
mavros_msgs::Command Command_now;
void generate_com(int sub_mode, float state_desired[4]);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    move_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);


    int flag_1;
    cout << "Input the flag:  1 for move_body 0 for move_ned"<<endl;
    cin >> flag_1;
    if (flag_1 == 0)
    {
        Command_now.command = 4;
    }else
    {
        Command_now.command = 6;
    }

    float state_desired[4];
    int sub_mode;
    //----------------------------------
    //input
    while(1)
    {

         cout << "Input the sub_mode:  # 0 for xy/z position control; 3 for xy/z velocity control"<<endl;
         cin >> sub_mode;
         cout << "Input the position value: (x y z yaw)"<<endl;
         cin >> state_desired[0];
         cin >> state_desired[1];
         cin >> state_desired[2];
         cin >> state_desired[3];


        generate_com(sub_mode, state_desired);
        move_pub.publish(Command_now);
        cout << "desired state: [ " << Command_now.pos_sp[0] << "  "<< Command_now.pos_sp[1] << "  "<<Command_now.pos_sp[2] << "  " \
              << Command_now.vel_sp[0]<<"  "<< Command_now.vel_sp[1]<< "  "<< Command_now.vel_sp[2]<<"  " << Command_now.yaw_sp << "]  "  <<endl;
        sleep(3);
    }


}

// float32[3] pos_sp
// float32[3] vel_sp
// float32 yaw_sp
void generate_com(int sub_mode, float state_desired[4]){

    static int comid = 1;
    Command_now.sub_mode = sub_mode; //0b00
    if((sub_mode>>1) == 0) //xy channel
    {
        Command_now.pos_sp[0] = state_desired[0];
        Command_now.pos_sp[1] = state_desired[1];
        Command_now.vel_sp[0] = 0.0;
        Command_now.vel_sp[1] = 0.0;
        cout << "submode: xy position control "<<endl;
    }
    else{
        Command_now.pos_sp[0] = 0.0;
        Command_now.pos_sp[1] = 0.0;

        Command_now.vel_sp[0] = state_desired[0];
        Command_now.vel_sp[1] = state_desired[1];
        cout << "submode: xy velocity control "<<endl;
    }
    if((sub_mode & 0b01) == 0) //z channel
    {
        Command_now.pos_sp[2] = state_desired[2];
        Command_now.vel_sp[2] = 0.0;
        cout << "submode: z position control "<<endl;
    }
    else
    {
        Command_now.pos_sp[2] = 0.0;
        Command_now.vel_sp[2] = state_desired[2];
        cout << "submode: z velovity control "<<endl;
    }
    Command_now.yaw_sp = state_desired[3];
    Command_now.comid = comid;
    comid++;
}
