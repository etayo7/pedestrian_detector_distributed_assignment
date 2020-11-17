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
					" mvspeed1 direction speed - Move servo 1 right/left with desired speed\n"
					" mvspeed2 direction speed - Move servo 1 up/down with desired speed\n"
					" get pos id - get a position value from a servo (1, 2, all)\n\n\n"
					"valid parameters\n"
					" pos - current position of a servo, -3 - 3\n";

	ros::init(argc, argv, "publisher");

	ros::NodeHandle n;

	ros::Publisher servo1_pub = n.advertise < std_msgs::Float64
			> ("/servo1/command", 1000);
	ros::Publisher servo2_pub = n.advertise < std_msgs::Float64
			> ("/servo2/command", 1000);

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

			else if (!strcmp(argv[1], "mvspeed1")) {
				publishers_arbotix::SetSpeed srv;
				srv.request.speed = atof(argv[3]);
				if (servo1_speed_service.call(srv)) {
				}

				if (!strcmp(argv[2], "left")) {
					pos = -3.15;
				} else if (!strcmp(argv[2], "right")) {
					pos = 3.15;
				}
				std_msgs::Float64 msg;
				std::stringstream ss;
				ss << argv[1] << pos;
				msg.data = pos;
				servo1_pub.publish(msg);
			}

			else if (!strcmp(argv[1], "mvspeed2")) {
				publishers_arbotix::SetSpeed srv;
				srv.request.speed = atof(argv[3]);
				if (servo2_speed_service.call(srv)) {
				}

				if (!strcmp(argv[2], "down")) {
					pos = -2;
				} else if (!strcmp(argv[2], "up")) {
					pos = 2;
				}
				std_msgs::Float64 msg;
				std::stringstream ss;
				ss << argv[1] << pos;
				msg.data = pos;
				servo2_pub.publish(msg);
			}

			else if (!strcmp(argv[1], "mv")) {
				float freq = 10;
				if (argv[2] != NULL) {
					freq = atof(argv[2]);
				}

				if (pos > 3.15) {
					add = false;
				} else if (pos <= -3.15) {
					add = true;
				}
				ros::Rate loop_rate(freq);

				std_msgs::Float64 msg;

				std::stringstream ss;
				ss << argv[1] << pos;
				msg.data = pos;

				ROS_INFO("%f", msg.data);

				servo1_pub.publish(msg);
				servo2_pub.publish(msg);

				ros::spinOnce();

				loop_rate.sleep();
				if (add) {
					pos = pos + 0.01;
				} else {
					pos = pos - 0.01;
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
