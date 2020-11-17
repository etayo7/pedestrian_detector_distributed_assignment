#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h> 
#include <stdlib.h>
#include <sstream>
#include <string>
#include "publishers_arbotix/GetPosition.h"
#include "publishers_arbotix/SetSpeed.h"
#include <iostream>
using namespace std;




































int main(int argc, char **argv) {

// help phrases
	string help_text =
			"\nTerminal\n"
					""
					"valid commands:"
					" rosrun get_3d_point etccc\n";
				

	ros::init(argc, argv, "publisher_point_subs_frame");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1);















	ros::Publisher servo1_pub = n.advertise < std_msgs::Float64
			> ("/servo1/command", 1000);



	ros::ServiceClient servo1_pos_service = n.serviceClient
			< publishers_arbotix::GetPosition > ("/servo1/get_position");
	ros::ServiceClient servo2_pos_service = n.serviceClient
			< publishers_arbotix::GetPosition > ("/servo2/get_position");

	ros::ServiceClient servo1_speed_service = n.serviceClient
			< publishers_arbotix::SetSpeed > ("/servo1/set_speed");
	ros::ServiceClient servo2_speed_service = n.serviceClient
			< publishers_arbotix::SetSpeed > ("/servo2/set_speed");

	float pos = -2.000000;
	bool add = true;
	while (ros::ok()) {
		if (argc > 1) {
			if (!strcmp(argv[1], "help")) {
				ROS_INFO("%s", help_text.c_str());
				return 0;
			} else if (!strcmp(argv[1], "get")) {
				if (!strcmp(argv[2], "pos")) {
					if (!strcmp(argv[3], "1")) {
						publishers_arbotix::GetPosition srv;
						if (servo1_pos_service.call(srv)) {
							ROS_INFO("%f", srv.response.position);
							return 0;
						}
					} else if (!strcmp(argv[3], "2")) {
						publishers_arbotix::GetPosition srv;

						if (servo2_pos_service.call(srv)) {
							ROS_INFO("%f", srv.response.position);
							return 0;
						}
					}

					else if (!strcmp(argv[3], "all")) {
						publishers_arbotix::GetPosition srv;
						publishers_arbotix::GetPosition srv2;
						if (servo1_pos_service.call(srv)) {
							ROS_INFO("Servo 1: %f", srv.response.position);
						}
						if (servo2_pos_service.call(srv2)) {
							ROS_INFO("Servo 2: %f", srv2.response.position);
						}
						return 0;
					}

				}
			}


	
	else {
		ROS_INFO("%s", "Must introduce at least one argument");
    return 0 ;
	}
}
	return 0;
}
