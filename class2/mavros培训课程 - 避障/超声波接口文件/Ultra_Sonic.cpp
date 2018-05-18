

#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
#include <sstream>
#include "serial.hpp"

int main(int argc, char **argv)
{
	std::cout<<"hello sonic!!"<<std::endl;

	// int RE_length, TR_length;
	ros::init(argc, argv, "talker");
	ros::NodeHandle node;
        ros::Publisher sonic_pub = node.advertise<std_msgs::UInt16>("/sonic", 100);
  	ros::Rate loop_rate(10);

	unsigned short int raw_height, height;

	Serial Sonic_Serial;

        int fd = Sonic_Serial.open_port(0);
	if (fd == -1){
		std::cout<<"open serail fail!"<<std::endl;
		return -1;}

	Sonic_Serial.set_opt(fd, 9600, 8, 'N', 1);

	int rByte;
        unsigned char rx_buffer[10], tr_buffer[10];
	tr_buffer[0] = 0x55;

	std_msgs::UInt16 sonic_info;
	while(1){
		tcflush(fd, TCIFLUSH);
		Sonic_Serial.nwrite(fd, tr_buffer, 1);
		usleep(50000);
		if(Sonic_Serial.nread(fd, rx_buffer, 2) == 2)
		{
			raw_height = (uint8_t)rx_buffer[0]*256+(uint8_t)rx_buffer[1];
			// cout<<"high: " <<(unsigned short int)rx_buffer[0]<<" low: "<< (unsigned short int)rx_buffer[1]<<endl;
		}
		std::cout <<"raw_height: "<< raw_height<<std::endl;

		sonic_info.data = (std::uint16_t)raw_height;
		sonic_pub.publish(sonic_info);

	    ros::spinOnce();
 	    loop_rate.sleep();
	}

	return 0;
}

