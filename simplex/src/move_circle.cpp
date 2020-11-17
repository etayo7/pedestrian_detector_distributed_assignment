// NODE MOVEMENT TARGETS - Move_circle (Arg 1 = N / Arg 2 = target id / Arg 3 = Radius / Arg 4 = speed) [ROS / c++]

// TFM - Iñigo Etayo
// Directors - Eduardo Montijano, Danilo Tardioli

// Perform a circular movement for the target i (target id), with speed and
// radius as arguments of the node

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <sstream>


int main(int argc, char **argv) {

  // ---------------------------INIT NODE---------------------------------------
  ros::init(argc, argv, "Move_circle"); //to set your node name, later rename with launcher

  //-------------------------- INIT GET PARAMETERS -----------------------------
  // 2 parameters: robot (id of robot to assign), N (number of robots)
  int target, N;
  float radius, speed;
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("N", N, int(40));
  private_node_handle_.param("target", target, int(40));
  private_node_handle_.param("radius", radius, float(40));
  private_node_handle_.param("speed", speed, float(40));

  //-------------------------- INIT STRINGS------- -----------------------------
  std::string str_topic_p = "/robot_" + std::to_string(N + target - 1) + "/cmd_vel";

  //------------------ INIT PUBLISHER AND SUBSCRIBER ROBOTi --------------------
	ros::NodeHandle n; //to initialize this node

  // 1 publisher, one robot to move, cmd_vel
  ros::Publisher str_pub = n.advertise < geometry_msgs::Twist
			> (str_topic_p, 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  ros::spinOnce();

  //-------------------------------INIT MAIN LOOP ------------------------------
  while (ros::ok()) {

    geometry_msgs::Twist movement;

    // -------------------------CALCULATE TRAJECTORY---------------------------
    movement.linear.x = speed;
    movement.angular.z = speed/radius;

    // ---------------------------------PUBLISH---------------------------------
    str_pub.publish(movement);

    // -----------------------------------LOGS----------------------------------
    //printf("ARGUMENTS: \n");

    // --------------------------------END----------------------------------
    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }

  return 0;

}
