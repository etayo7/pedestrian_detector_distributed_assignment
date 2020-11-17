// NODE NETWORK - network (Arg 1 = N) [ROS / c++]

// TFM - Iñigo Etayo
// Directors - Eduardo Montijano, Danilo Tardioli

// Creates a matrix of the communication limits between robots in fuction of
// theris position, if two robots are too far away (more than a defined limit)
// this two robots cannot exchange information

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <sstream>

//Eigen - matrix library
#include <iostream> //IO datos
#include <fstream> //IO datos from files (results)
#include <Eigen/Dense> //Eigen, matrix library
using namespace Eigen;
using Eigen::all; //matrix indexing

//global variables for subscribers
std::vector<geometry_msgs::PoseWithCovariance> pos_t; //subscriber pos targets
void poseTargetMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn); //get pos of targets

int main(int argc, char **argv) {
  // ---------------------------INIT NODE---------------------------------------
  ros::init(argc, argv, "Network"); //to set your node name, later rename with launcher

  int N;
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("N", N, int(40));

  //------------------ INIT PUBLISHER AND SUBSCRIBER ROBOTi --------------------
	ros::NodeHandle n; //to initialize this node
  ros::Publisher net_pub = n.advertise <std_msgs::Int32MultiArray> ("/network", 1000);

  // N subscribers for N pos of the robots
  int N_fix = N+1;
  pos_t.resize(N_fix);
  ros::Subscriber sub_t[N_fix]; // [through TCP]
  std::string str2[N_fix];
  for (int i = 1; i < N_fix; i++) {
    str2[i] = "/robot_" + std::to_string(i-1) + "/amcl_pose";
    sub_t[i] = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(str2[i], 1000, boost::bind(poseTargetMessage, _1, i));
  }

  ros::Rate loop_rate(10);
  int count = 0;
  ros::spinOnce();

  //-------------------------------INIT MAIN LOOP ------------------------------
  while (ros::ok()) {

    // -------------------------DEFINE NETWORK----------------------------------
    std::vector<int> graph((N*N),0);

    MatrixXd vec(2,1);
    float d;
    for (int i = 1; i < N_fix; i++) {
      for (int j = 1; j < N_fix; j++) {
        if (i!=j) {
          vec(0)= pos_t.at(i).pose.position.x - pos_t.at(j).pose.position.x;
          vec(1)= pos_t.at(i).pose.position.y - pos_t.at(j).pose.position.y;
          d = vec.norm();
          if (d <= 3.5) {
            graph.at((j-1)*(N_fix-1) + i-1)=1;
          } else {
            graph.at((j-1)*(N_fix-1) + i-1)=0;
          }
        }
      }
    }

    std_msgs::Int32MultiArray net;
    net.layout.dim.push_back(std_msgs::MultiArrayDimension());
    net.layout.dim[0].size = graph.size();
    net.layout.dim[0].stride = 1;
    net.layout.dim[0].label = "Network";

    net.data.clear();
    net.data.insert(net.data.end(), graph.begin(), graph.end());

     // printf("Network: \n");
     // printf("%i , %i , %i, %i \n",graph.at(0),graph.at(1),graph.at(2),graph.at(3));
     // printf("%i , %i , %i, %i \n",graph.at(4),graph.at(5),graph.at(6),graph.at(7));
     // printf("%i , %i , %i, %i \n",graph.at(8),graph.at(9),graph.at(10),graph.at(11));
     // printf("%i , %i , %i, %i \n \n",graph.at(12),graph.at(13),graph.at(14),graph.at(15));

    // ---------------------------------PUBLISH---------------------------------
    net_pub.publish(net);

    // ------------------------------------END----------------------------------
    ros::spinOnce();

    loop_rate.sleep();
    ++count;

  }

  return 0;

}

void poseTargetMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn) {
  pos_t.at(nn) = msg->pose; //N target pose subscribers
}
