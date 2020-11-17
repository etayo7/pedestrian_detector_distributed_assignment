// NODE RViz Assignment - assign_rviz (Arg 1 = N) [ROS / c++]

// TFM - Iñigo Etayo
// Directors - Eduardo Montijano, Danilo Tardioli

// Creates diferent markers to visualize in RViz the robots, the targets and
// the assignment between robots and targets

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "simplex/simplex.h"
#include "simplex/simData.h"
#include <boost/bind.hpp>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int32MultiArray.h"
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <math.h>


//global variables for subscribers
std::vector<geometry_msgs::PoseWithCovariance> pos_t; //subscriber pos targets
std::vector<geometry_msgs::PoseWithCovariance> pose_r; //subscriber pos targets
std::vector<simplex::simplex> dataRc; //subscriber Msg other robots

//callback functions for subscribers
void poseRobotMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn); //get pos of targets
void poseTargetMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn); //get pos of targets
void getDataMessage(simplex::simplex msg); //get message of other robots

//other functions to create RViz
simplex::simData send_results(int robot, int i_assign, geometry_msgs::PoseWithCovariance pose);
void create_shape(ros::Publisher marker_pub, simplex::simData robot, int id, uint32_t shape, float size, float colour[3], int sign, int assign, simplex::simData target); //create RVIZ


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
// ---------------------------INIT NODE----------------------------------- ----
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
int main(int argc, char **argv) {

  ros::init(argc, argv, "Rviz_markers"); //to set your node name, later rename with launcher for each robot i

  //-------------------------- INIT GET PARAMETERS -----------------------------
  // 3 parameters: robot (id of robot to assign), N (number of robots), policy (which policy to use)
  int N;
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("N", N, int(40));
  int N_fix = N+1; //size N_fix = N+1 (due to start at 0)
  pos_t.resize(N_fix);
  pose_r.resize(N_fix);
  dataRc.resize(N_fix);


    //------------------ INIT PUBLISHER AND SUBSCRIBER ROBOTi --------------------
  	ros::NodeHandle n; //to initialize this node

    // 1 publisher, data send to other robots (topic master)
    ros::Publisher topData_pub = n.advertise < simplex::simplex > ("/topData", 1000);

    // subscriber of the data send by other robots (topic master) [through UDP]
    const auto e = ros::TransportHints().udp();
    ros::Subscriber topData_sub = n.subscribe("/topData", 1000, getDataMessage, e);


    //--------------------------------INIT SUB N robots -------------------------
    // N subscribers for N pos of the targets
    ros::Subscriber sub_tr[N_fix]; // [through TCP]
    std::string str2r[N_fix];
    for (int i = 1; i < N_fix; i++) {
      str2r[i] = "/robot_" + std::to_string(i-1) + "/amcl_pose";
      sub_tr[i] = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(str2r[i], 1000, boost::bind(poseRobotMessage, _1, i));
    }

    //--------------------------------INIT SUB N TARGETS -------------------------
    // N subscribers for N pos of the targets
    ros::Subscriber sub_t[N_fix]; // [through TCP]
    std::string str2[N_fix];
    for (int i = 1; i < N_fix; i++) {
      str2[i] = "/robot_" + std::to_string(N+i-1) + "/amcl_pose";
      sub_t[i] = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(str2[i], 1000, boost::bind(poseTargetMessage, _1, i));
    }

    //--------------------------------PUB RVIZ------------------------------------
    // publisher to create rviz shapes
    ros::Publisher pub_m_r[N_fix];
    ros::Publisher pub_m_t[N_fix];
    std::string str_m_r[N_fix];
    std::string str_m_t[N_fix];
//    pub_m_r = n.advertise < visualization_msgs::Marker > (str_m_r, 10);
        for (int i = 1; i < N_fix; i++) {
          str_m_r[i] = "visualization_marker_r_" + std::to_string(i);
          pub_m_r[i] = n.advertise < visualization_msgs::Marker > (str_m_r[i], 10);
        }
//    pub_m_t = n.advertise < visualization_msgs::Marker > (str_m_t, 10);
        for (int i = 1; i < N_fix; i++) {
          str_m_t[i] = "visualization_marker_t_" + std::to_string(i);
          pub_m_t[i] = n.advertise < visualization_msgs::Marker > (str_m_t[i], 10);
        }

    ros::Rate loop_rate(50);
    int count = 1;


      //----------------------------------------------------------------------------
      //----------------------------------------------------------------------------
      //-------------------------------INIT MAIN LOOP ------------------------------
      //----------------------------------------------------------------------------
      //----------------------------------------------------------------------------

      while (ros::ok()) {

        //------------------------------SHOW RESULTS--------------------------------
        if (count > 3) {
          // RVIZ definitions
          std::vector<simplex::simData> data_t;
          simplex::simData data_r;
          data_t.resize(N_fix);

          int assign_rviz;
          // create results to create rviz
          for (int k = 1; k < N_fix; k++) {
            assign_rviz = dataRc.at(k).assign;

            data_r = send_results(k, assign_rviz, pose_r.at(k));

            uint32_t cube = visualization_msgs::Marker::CUBE;
            uint32_t cylinder = visualization_msgs::Marker::CYLINDER;
            float size = 0.3; //1m3
            float blue[3] = {0.0,0.0,1.0};
            float red[3] = {1.0,0.0,0.0};

            // from PoseCovaraince of targets, to simData
            if (k<2){
                for (int i = 1; i < N_fix; i++) {
                  data_t.at(i).pose = {pos_t.at(i).pose.position.x,pos_t.at(i).pose.position.y,pos_t.at(i).pose.position.z};
                  data_t.at(i).orientation = {0.0,0.0,pos_t.at(i).pose.orientation.z,pos_t.at(i).pose.orientation.w};
                  create_shape(pub_m_t[i], data_t.at(i), N+i, cylinder, size, red, 2, 1, data_t.at(i));
                }
            }

            data_t.at(assign_rviz).pose = {pos_t.at(assign_rviz).pose.position.x,pos_t.at(assign_rviz).pose.position.y,pos_t.at(assign_rviz).pose.position.z};
            data_t.at(assign_rviz).orientation = {0.0,0.0,pos_t.at(assign_rviz).pose.orientation.z,pos_t.at(assign_rviz).pose.orientation.w};

            create_shape(pub_m_r[k], data_r, k, cube, size, blue, 1, assign_rviz, data_t.at(assign_rviz));
          }
          }


        // --------------------------------END--------------------------------------
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
      }
//      myFile.close();
      return 0;
    }



    simplex::simData send_results(int robot, int i_assign, geometry_msgs::PoseWithCovariance pose)
    {
      simplex::simData dataRe;

      dataRe.header.stamp = ros::Time::now();
      dataRe.id = robot;
      dataRe.assign = i_assign;
      dataRe.reach = 7.5;
      dataRe.pose = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
      dataRe.orientation = {0.0 , 0.0, pose.pose.orientation.z, pose.pose.orientation.w};

      return dataRe;
    }


    void create_shape(ros::Publisher marker_pub, simplex::simData robot, int id, uint32_t shape, float size, float colour[3], int sign, int assign, simplex::simData target){

      visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = id;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = robot.pose[0];
        marker.pose.position.y = robot.pose[1];
        marker.pose.position.z = robot.pose[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = robot.orientation[2];
        marker.pose.orientation.w = robot.orientation[3];

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = colour[0];
        marker.color.g = colour[1];
        marker.color.b = colour[2];
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);

        // draw line to assignment
        if (sign < 2) { // 2= target/ 1= correct assign/ 2=incorrect assign
          visualization_msgs::Marker line_strip;
          line_strip.header.frame_id = "/map";
          line_strip.header.stamp = ros::Time::now();
          line_strip.ns = "points_and_lines";
          line_strip.id = id;
          line_strip.action = visualization_msgs::Marker::ADD;
          line_strip.pose.orientation.w = 1.0;
          line_strip.type = visualization_msgs::Marker::LINE_STRIP;
          line_strip.scale.x = 0.1;
          if (sign=1) {
            line_strip.color.r = 0.0;
            line_strip.color.g = 1.0;
            line_strip.color.b = 0.0;
            line_strip.color.a = 1.0;
          } else {
            line_strip.color.r = 1.0;
            line_strip.color.g = 0.0;
            line_strip.color.b = 0.0;
            line_strip.color.a = 1.0;
          }
          geometry_msgs::Point p;
          p.x = robot.pose[0];
          p.y = robot.pose[1];
          p.z = robot.pose[2];
          line_strip.points.push_back(p);
          p.x = target.pose[0];
          p.y = target.pose[1];
          p.z = target.pose[2];
          line_strip.points.push_back(p);
          marker_pub.publish(line_strip);
        }

        //draw line to reach bandwidth
        if (sign < 2) { // 2= target/ 1= correct assign/ 2=incorrect assign
          visualization_msgs::Marker communication;
          communication.header.frame_id = "/map";
          communication.header.stamp = ros::Time::now();
          communication.ns = "basic_shapes";
          communication.id = id+1;
          communication.action = visualization_msgs::Marker::ADD;
          communication.pose.orientation.w = 1.0;
          communication.type = visualization_msgs::Marker::CYLINDER;

          communication.pose.position.x = robot.pose[0];
          communication.pose.position.y = robot.pose[1];
          communication.pose.position.z = robot.pose[2];
          communication.pose.orientation.x = 0.0;
          communication.pose.orientation.y = 0.0;
          communication.pose.orientation.z = 0.0;
          communication.pose.orientation.w = 1.0;

          float reach = 7;
          communication.scale.x = reach;
          communication.scale.y = reach;
          communication.scale.z = 0.1;

          communication.color.r = 1.0;
          communication.color.g = 1.0;
          communication.color.b = 0.0;
          communication.color.a = 0.2;

          communication.lifetime = ros::Duration();

          marker_pub.publish(communication);
        }
    }



    // Calback functions
    void poseRobotMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn) {
      pose_r.at(nn) = msg->pose; //N target pose subscribers
    }
    void poseTargetMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn) {
      pos_t.at(nn) = msg->pose; //N target pose subscribers
    }
    void getDataMessage(simplex::simplex msg) {
      dataRc.at(msg.id) = msg; //robots messages subscribers
    }
