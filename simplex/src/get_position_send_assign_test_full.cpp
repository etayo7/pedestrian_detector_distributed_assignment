// MASTER NODE - Robot_Assign (Arg 1 = robot / Arg 2 = N / Arg 3 = Policy)
//  (Arg 4 = proportional controller / Arg 5 = max angular velocity)
//  (Arg 6 = lab(0)-virtual(1) / Arg 7 = path to record results) [ROS / c++]

// TFM - Iñigo Etayo
// Directors - Eduardo Montijano, Danilo Tardioli

// Given de number of robots of the problem, and the id of the robot to
// compute de assignment, this node read the position of the robot_id and the
// position of all the targets. Then compute the assignment with distributed simplex
// and with the assignment obtained for the robot i to look target j, publish
// angular velocity to the robot i to orientate towards the target using a
// proportional controller.

// To do so this node received data from the other robots to update its own
// and be able to compute the algorithm. This data sent and received is selected
// according to the policy choosen with the initial arguments.

// Policies:
//
//   No: Name:          Size / Selection criteria
//  - 0: Original     - 2N-1 / simplex choose
//  - 1: Assign       - N    / assignment
//  - 2: Assign+Own   - 2N-1 / assign + own rewards
//  - 3: Extra cost   - 3N-1 / assign + min rewards
//  - 4: Extra age    - 3N-1 / assign + min ages
//  - 5: All          - N²   / all

//----------------------------------------------------------------------------
// ---------------------------DECLARATIONS------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "simplex/simplex.h"
#include "simplex/simData.h"
#include "simplex/IntList.h"
#include <boost/bind.hpp>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Int32MultiArray.h"
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <sstream>
#include <cmath>
#include <ctgmath>

//Eigen - matrix library
#include <iostream> //IO datos
#include <fstream> //IO datos from files (results)
#include <Eigen/LU> //linear solver, not needed
#include <Eigen/Dense> //Eigen, matrix library

using namespace Eigen;
using Eigen::all; //matrix indexing

//define simplex data, collects all data of simplex matrix
struct simplexData {
  MatrixXd A;
  MatrixXd b;
  MatrixXd rew;
  MatrixXd rew_old;
  MatrixXd rew_pc;
  MatrixXd age;
  MatrixXd age_old;
  ArrayXf ibasic;
  MatrixXd B;
  MatrixXd xbasic;
  MatrixXd z;
  int Assign;
  bool converge;
  int policy;
  int size;
};

//global variables for subscribers
std::vector<geometry_msgs::PoseWithCovariance> pos_t; //subscriber pos targets
geometry_msgs::PoseWithCovariance pos_r; //subscriber pos robot i
std::vector<simplex::simplex> dataRc; //subscriber Msg other robots
int Net[100]; //{0,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0}; //info sobre el grafo de comunicacion de la red
//int Net[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
ArrayXf msg_count; //to count the number of msg sent and received
simplex::IntList PD_costs;
simplex::IntList PD_angles;

//callback functions for subscribers
void poseRobotMessage(const geometry_msgs::PoseWithCovarianceStamped &msg); //get position of robot i
void poseTargetMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn); //get pos of targets
void getDataMessage(simplex::simplex msg); //get message of other robots
void getNetDefinition(const std_msgs::Int32MultiArray::ConstPtr& array); //get net graph
//callbacks to PD costs
void getCosts(simplex::IntList msg); //get cost of robot i
void getAngles(simplex::IntList msg); //get angle to each target


//declare other functions
simplexData initializeSimplex(simplexData sData, int N); //initialize simplexData for first iteration, and initial basis
float orientation_f(geometry_msgs::PoseWithCovariance robot, geometry_msgs::PoseWithCovariance target); //angle from pos of robot and target assigned
simplexData simplexDist(simplexData data, int robot, int N, ros::Publisher topData_pub); //compute simplex algorithm
void send_data(simplexData data, int robot, int N, ros::Publisher topData_pub, bool send); //send data from robot i to anyone listening
simplexData updateRew(simplexData data, int robot, int N, geometry_msgs::PoseWithCovariance pos_r, std::vector<geometry_msgs::PoseWithCovariance> pos_t); //compute own rewards
simplexData updateMsg(simplexData data, int robot, int N, std::vector<simplex::simplex> dataRc, int Net[]); //update rewards and ages from Msg
simplexData updateAge(simplexData data, int N); //update age and old rewards
int lexratiotest(MatrixXd B, MatrixXd e); //obtain simplex iexit
bool lexlt(MatrixXd v, MatrixXd w); //obtain simplex iexit

//other functions to create RViz
simplex::simData send_results(int robot, int i_assign, geometry_msgs::PoseWithCovariance pose);
void create_shape(ros::Publisher marker_pub, simplex::simData robot, int id, uint32_t shape, float size, float colour[3], int sign, int assign, simplex::simData target); //create RVIZ


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
// ---------------------------INIT NODE---------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
int main(int argc, char **argv) {

  ros::init(argc, argv, "Robot_assign"); //to set your node name, later rename with launcher for each robot i

  //-------------------------- INIT GET PARAMETERS -----------------------------
  // 7 parameters: robot (id of robot to assign), N (number of robots), policy (which policy to use)
  int robot, N, policy, simulation, PD;
  float controller, max_vel;
  simulation = 0;
  std::string path;
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("robot", robot, int(40));
  private_node_handle_.param("N", N, int(40));
  private_node_handle_.param("policy", policy, int(40));
  private_node_handle_.param("controller", controller, float(40));
  private_node_handle_.param("max_vel", max_vel, float(40));
  private_node_handle_.param("simulation", simulation, int(40));
  private_node_handle_.param("path", path, std::string());
  private_node_handle_.param("PD", PD, int(40));
  int N_fix = N+1; //size N_fix = N+1 (due to start at 0)
  pos_t.resize(N_fix);
  dataRc.resize(N_fix);
  PD_costs.data.resize(N);
  PD_angles.data.resize(N);
  msg_count = ArrayXf::Ones(N)*0;
  if (simulation == 0) {
    robot = robot + 1;
  }

  for (int i = 0; i < (N*N); i++)
  {
    Net[i] = 1;
  }

  for (int i = 0; i < (N); i++)
  {
    PD_costs.data.at(i) = 1;
    PD_angles.data.at(i) = 0;
  }

  //-------------------------- INIT STRINGS------- -----------------------------
  std::string str_topic_p = "/robot_" + std::to_string(robot-1) + "/cmd_vel";
  std::string str_topic_s = "/robot_" + std::to_string(robot-1) + "/amcl_pose";

  //------------------ INIT PUBLISHER AND SUBSCRIBER ROBOTi --------------------
	ros::NodeHandle n; //to initialize this node

  // 1 publisher, one robot to move, cmd_vel
  ros::Publisher str_pub = n.advertise < geometry_msgs::Twist > (str_topic_p, 1000);
  // 1 publisher, data send to other robots (topic master)
  ros::Publisher topData_pub = n.advertise < simplex::simplex > ("/topData", 1000);
  // 1 subscriber of the position of roboti [through TCP]
  ros::Subscriber str_sub = n.subscribe(str_topic_s, 1000, poseRobotMessage);

  // subscriber of the data send by other robots (topic master) [through UDP]
  const auto e = ros::TransportHints().udp();
  ros::Subscriber topData_sub = n.subscribe("/topData", 1000, getDataMessage, e);

  //------------------------------INIT SUB positions----------------------------
  // N subscribers for N pos of the targets
  ros::Subscriber sub_t[N_fix]; // [through TCP]
  std::string str2[N_fix];
  for (int i = 1; i < N_fix; i++) {
    str2[i] = "/robot_" + std::to_string(N+i-1) + "/amcl_pose";
    sub_t[i] = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(str2[i], 1000, boost::bind(poseTargetMessage, _1, i));
  }
  //-------------------------------- SUB PD camera -----------------------------
  // subscriber of PedestrianDetector, each cost and angle to move
  std::string PD_str_1 = "/robot_" + std::to_string(robot-1) + "/PD_cost";
  std::string PD_str_2 = "/robot_" + std::to_string(robot-1) + "/PD_angle";
  ros::Subscriber PD_sub_1 = n.subscribe<simplex::IntList>(PD_str_1, 1000, getCosts);
  ros::Subscriber PD_sub_2 = n.subscribe<simplex::IntList>(PD_str_2, 1000, getAngles);

  //--------------------------------SUB NETWORK DEF-----------------------------
  // subscriber of graph net, if not complete connection needed
  ros::Subscriber net_sub = n.subscribe("/network", 1000, getNetDefinition);

  //--------------------------------PUB RVIZ------------------------------------
  // publisher to create rviz shapes
  ros::Publisher pub_m_r;
  ros::Publisher pub_m_t[N_fix];
  std::string str_m_r;
  std::string str_m_t[N_fix];
  str_m_r = "visualization_marker_r_" + std::to_string(robot);
  pub_m_r = n.advertise < visualization_msgs::Marker > (str_m_r, 10);
  if (robot<2){
      for (int i = 1; i < N_fix; i++) {
        str_m_t[i] = "visualization_marker_t_" + std::to_string(i);
        pub_m_t[i] = n.advertise < visualization_msgs::Marker > (str_m_t[i], 10);
      }
  }

  ros::Rate loop_rate(50);
  int count = 1;

  // ------------------------INITIALIZE MATRIX----------------------------------
  simplexData sData;
  sData.policy = policy;
  sData = initializeSimplex(sData, N); //initialize simplex matrix and first Basis

  //initialize data Matrix (dataRc)
  for (int i = 1; i < N_fix; i++) {
    send_data(sData,i,N,topData_pub,false);
  }

  //create file to save results
  //std::string filename = "/home/osboxes/catkin_ws/src/simplex/results/robot_" + std::to_string(robot) + "_results.csv";
  std::string filename = path + "/robot_" + std::to_string(robot) + "_results.csv";
  std::ofstream myFile(filename);

  sleep (5);

  //----------------------------------------------------------------------------
  //----------------------------------------------------------------------------
  //-------------------------------INIT MAIN LOOP ------------------------------
  //----------------------------------------------------------------------------
  //----------------------------------------------------------------------------
  while (ros::ok()) {

    // ------------------------ UPDATE DATA (SIMPLEX) --------------------------
    //update simplex matrix with computed rewards and Msgs
    sData = updateMsg(sData, robot, N, dataRc, Net);
    //update age and old rewards
    sData = updateAge(sData, N);
    //printf("PD= %i \n",PD);
    if (PD == 0) {
      //compute rewards and ages from own poses
      sData = updateRew(sData, robot, N, pos_r, pos_t);
    } else {
      //compute rewards and ages from PD costs
      for (int i = 0; i < N; i++) {
        sData.rew(0,(N*(robot-1))+i)=PD_costs.data.at(i);
        sData.age(0,(N*(robot-1))+i)=0;
      }
    }
    //send data thorugh Msg
    send_data(sData,robot,N,topData_pub,true);

    // --------------------COMPUTE ASSIGNMENT(SIMPLEX)--------------------------
    sData.converge = false;
    while ((not(sData.converge))) {
      //compute simplex algorithm ultil convergence
      sData = simplexDist(sData, robot, N, topData_pub);
    }
    // -------------------------CALCULATE ORIENTATION---------------------------
    float vel_angular;
    if (PD == 0) {
      vel_angular = orientation_f(pos_r, pos_t.at(sData.Assign));
    } else {
      if (std::isnan(PD_angles.data.at(sData.Assign))) {
        vel_angular = 0;
      } else {
        vel_angular = PD_angles.data.at(sData.Assign);
      }
    }

    geometry_msgs::Twist assign;
    // ---------------------PUBLISH ORIENTATION---------------------------------
    float kp = 0.7; // proportional controller (try different const)
    if (std::isnan(vel_angular)) {
      assign.angular.z = 0.0;
    } else {
      if (controller*vel_angular>max_vel) {
        vel_angular=max_vel/controller;
      }
      assign.angular.z = controller*(vel_angular);
    }
    str_pub.publish(assign);
    //------------------------------SHOW RESULTS--------------------------------
    if (count < 0) {
      // RVIZ definitions
      std::vector<simplex::simData> data_t;
      simplex::simData data_r;
      data_t.resize(N_fix);
      // create results to create rviz
      data_r = send_results(robot, sData.Assign, pos_r);

      uint32_t cube = visualization_msgs::Marker::CUBE;
      uint32_t cylinder = visualization_msgs::Marker::CYLINDER;
      float size = 0.3; //1m3
      float blue[3] = {0.0,0.0,1.0};
      float red[3] = {1.0,0.0,0.0};

      // from PoseCovaraince of targets, to simData
      if (robot<2){
          for (int i = 1; i < N_fix; i++) {
            data_t.at(i).pose = {pos_t.at(i).pose.position.x,pos_t.at(i).pose.position.y,pos_t.at(i).pose.position.z};
            data_t.at(i).orientation = {0.0,0.0,pos_t.at(i).pose.orientation.z,pos_t.at(i).pose.orientation.w};
            create_shape(pub_m_t[i], data_t.at(i), N+i, cylinder, size, red, 2, 1, data_t.at(i));
          }
      }
      data_t.at(sData.Assign).pose = {pos_t.at(sData.Assign).pose.position.x,pos_t.at(sData.Assign).pose.position.y,pos_t.at(sData.Assign).pose.position.z};
      data_t.at(sData.Assign).orientation = {0.0,0.0,pos_t.at(sData.Assign).pose.orientation.z,pos_t.at(sData.Assign).pose.orientation.w};

      create_shape(pub_m_r, data_r, robot, cube, size, blue, 1, sData.Assign, data_t.at(sData.Assign));
    }


    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // -----------------------------OUTPUT FILE---------------------------------
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    if (PD==0) {
      myFile << count << "," << sData.policy << "," << sData.Assign << ",";
      for (int i = 0; i < N; i++) {
        myFile << sData.rew(0,(N*(robot-1))+i) << ",";
      }
      myFile << pos_r.pose.position.x << "," << pos_r.pose.position.y << ",";
      myFile << pos_t.at(robot).pose.position.x << "," << pos_t.at(robot).pose.position.y << ",";
      for (int i = 0; i < (2*N-1); i++) {
        myFile << sData.ibasic(i,0) << ",";
      }
      for (int i = 0; i < sData.rew.cols(); i++) {
        myFile << sData.rew(0,i) << ",";
      }
      double secs =ros::Time::now().toSec();
      double secs1 = secs * 100.0;
      myFile << secs << ",";
      myFile << "\n";
    }
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // -----------------------------------LOGS----------------------------------
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // printf("ARGUMENTS: \n");
    // printf("Robot i = %i \n", robot);
    // printf("Num. Robots = %i \n \n", N);
    //
    // // printf("POSITIONS: \n");
    // // printf("Position R %f %f \n", pos_r.pose.position.x, pos_r.pose.position.y);
    // // printf("Position T %f %f \n \n", pos_t.at(sData.Assign).pose.position.x, pos_t.at(sData.Assign).pose.position.y);
    //
    // // printf("ORIENTATIONS: \n");
    // // //printf("Init %f Goal %f \n", angle_init, angle_goal);
    // // printf("Vel. Angular %f\n \n", assign.angular.z);
    // printf("ASSIGNMENT %i \n \n", sData.Assign);
    // printf("robot %i \n \n", robot);
    //
    // // printf("STRINGS: \n");
    // // printf("robot i pub %s \n", str_topic_p.c_str());
    // // printf("robot i sub %s \n", str_topic_s.c_str());
    // // printf("target assign sub %s \n \n", str2[sData.Assign].c_str());
    //
    //  printf("Network: \n");
    //  printf("%i , %i , %i, %i \n",Net[0],Net[1],Net[2],Net[3]);
    //  printf("%i , %i , %i, %i \n",Net[4],Net[5],Net[6],Net[7]);
    //  printf("%i , %i , %i, %i \n",Net[8],Net[9],Net[10],Net[11]);
    //  printf("%i , %i , %i, %i \n \n",Net[12],Net[13],Net[14],Net[15]);
    //
    //  printf("Message: %i \n",dataRc.at(robot).id);
    //  printf("b %i , %i , %i, %i, %i, %i, %i, %i  \n",dataRc.at(robot).b[0],dataRc.at(robot).b[1],dataRc.at(robot).b[2],dataRc.at(robot).b[3],dataRc.at(robot).b[4],dataRc.at(robot).b[5],dataRc.at(robot).b[6],dataRc.at(robot).b[7]);
    //  printf("rew %f , %f , %f, %f, %f  \n",dataRc.at(robot).rew[0],dataRc.at(robot).rew[1],dataRc.at(robot).rew[2],dataRc.at(robot).rew[3],dataRc.at(robot).rew[4]);
    //  printf("age %i , %i , %i, %i, %i  \n \n",dataRc.at(robot).age[0],dataRc.at(robot).age[1],dataRc.at(robot).age[2],dataRc.at(robot).age[3],dataRc.at(robot).age[4]);
    //
    //
    // // -------------------------- LOGS EIGEN -----------------------------------
    //  printf("Matrix: A b r age \n");
    //  //printf("A \n");std::cout << sData.A << std::endl << std::endl;
    //  //printf("b \n");std::cout << sData.b.transpose() << std::endl << std::endl;
    //  printf("rew \n");std::cout << sData.rew << std::endl << std::endl;
    //  printf("age \n");std::cout << sData.age << std::endl << std::endl;
    //  printf("ibasic \n");std::cout << sData.ibasic.transpose() << std::endl << std::endl;
    //  //printf("B \n");std::cout << sData.B << std::endl << std::endl;
    //  //printf("B inv \n");std::cout << sData.B.inverse() << std::endl << std::endl;
    //  printf("xbasic \n");std::cout << sData.xbasic.transpose() << std::endl << std::endl;
    //  printf("z \n");std::cout << sData.z << std::endl << std::endl;
    //
    //  printf("p \n");std::cout << sData.policy << std::endl << std::endl;
    //  printf("s \n");std::cout << sData.size << std::endl << std::endl;
    //  printf("A \n");std::cout << sData.A << std::endl << std::endl;
    //  printf("b \n");std::cout << sData.b << std::endl << std::endl;
    //
    //  printf("Msg Count \n");
    //  printf("Msg Count: \n");std::cout << msg_count << std::endl << std::endl;

    // --------------------------------END--------------------------------------
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  myFile.close();
  return 0;
}

simplexData initializeSimplex(simplexData sData, int N){

  switch (sData.policy) {
    case 0:
      sData.size = 2*N-1;
      break;
    case 1:
      sData.size = N;
      break;
    case 2:
      sData.size = 2*N-1;
      break;
    case 3:
      sData.size = 3*N-1;
      break;
    case 4:
      sData.size = 3*N-1;
      break;
    case 5:
      sData.size = N*N;
      break;
  }

  sData.A = MatrixXd::Zero((2*N),(N*N));
  MatrixXd bigM = MatrixXd::Identity((2*N-1),(2*N-1));
  for (int i = 0; i < N; i++) {
    for (int j=0; j < N; j++) {
      sData.A(i,  (i)*N+j)=1;
      sData.A(N+j,(i)*N+j)=1;
    }
  }
//  sData.A = sData.A.block(0,0,(2*N-1),(N*N));
  sData.A.conservativeResize((2*N-1),(N*N+(2*N-1)));
  sData.A << sData.A.block(0,0,(2*N-1),(N*N)),bigM;
  sData.A(0,0)=1;
  sData.b = MatrixXd::Ones((2*N-1),1);
  sData.rew = MatrixXd::Ones(1,N*N+(2*N-1))*(FLT_MAX); //float max
  sData.rew.block(0,N*N,1,(2*N-1)) = MatrixXd::Ones(1,(2*N-1))*100000; //bigM
  sData.age = MatrixXd::Ones(1,N*N+(2*N-1))*0;
  sData.rew_old = MatrixXd::Ones(1,N*N+(2*N-1))*(FLT_MAX);
  sData.age_old = MatrixXd::Ones(1,N*N+(2*N-1))*0;

  sData.ibasic = ArrayXf::LinSpaced(2*N-1,N*N,N*N+2*N-2);
  sData.B = MatrixXd((2*N-1),(2*N-1));
  sData.B = sData.A(all,sData.ibasic);
  sData.xbasic = MatrixXd((2*N-1),1);
  sData.xbasic = sData.B.inverse()*sData.b;
  sData.z = MatrixXd(1,1);
  sData.z = sData.rew(all,sData.ibasic)*sData.xbasic;
  sData.Assign = 0;

  return sData;
}

simplexData simplexDist(simplexData data, int robot, int N, ros::Publisher topData_pub)
{
  // compute not basic index [inon]
  ArrayXf inon(N*N);
  int j=0;
  for (int i = 0; i < (data.A.cols()); i++) {
    if (not(data.ibasic==i).any()) {
      inon(j) = i;
      j = j+1;
    }
  }

  //compute CRed of inon
  MatrixXd CRed(1,N*N);
  CRed = data.rew_pc(0,inon) - (data.rew_pc(0,data.ibasic)*(data.B.inverse()))*data.A(all,inon);
  std::ptrdiff_t ii, jj;
  float minC = CRed.minCoeff(&ii,&jj);
  printf("Coste \n");std::cout << data.rew << std::endl << std::endl;
  printf("CRed \n");std::cout << CRed << std::endl << std::endl;
  printf("Min \n");std::cout << CRed.minCoeff() << std::endl << std::endl;

  // find min(CRed) -> entering basis index
  if (CRed.minCoeff() < -0.00000001) {
    int ienter = inon(jj);
    data.converge = false;
    data.B = data.A(all,data.ibasic);

    //obtain iexit
    int iexit = data.ibasic(lexratiotest(data.B, data.A(all,ienter)),0);
    int kk;

    //find index of exit
    for (int i = 0; i < 2*N-1; i++) {
      if (data.ibasic(i,0) == iexit) {
        kk = i;
      }
    }

    //enter ienter at iexit position
    data.ibasic(kk,0) = ienter;

    //sort ibasic
    int ib = 0;
    ArrayXf basi = ArrayXf::Ones((2*N-1));
    for (int k = 0; k < data.A.cols(); k++) {
      if ((data.ibasic == k).any()) {
        basi(ib) = k;
        ib = ib+1;
      }
    }
    data.ibasic = basi;

    //update matrix
    data.B = data.A(all,data.ibasic);
    data.xbasic = data.B.inverse()*data.b;
    data.z = data.rew(all,data.ibasic)*data.xbasic;

    printf("ibasic \n");std::cout << data.ibasic << std::endl << std::endl;
    printf("xbasic \n");std::cout << data.xbasic << std::endl << std::endl;

    printf("Enter \n");std::cout << ienter << std::endl << std::endl;
    printf("Exit \n");std::cout << iexit << std::endl << std::endl;

  } else {
    data.z = data.rew(all,data.ibasic)*data.xbasic;
    data.rew_old = data.rew;
    data.age_old = data.age;
    data.converge = true;
  }

  ArrayXf array = ArrayXf::LinSpaced(N,N*(robot-1),N*(robot)-1);
  int base;
  for (int i = 0; i < 2*N-1; i++) {
    base = data.ibasic(i,0);
    if ((array==base).any()&&(data.xbasic(i,0)==1)) {
      data.Assign = base - ((robot-1)*N) + 1;
      i = 2*N-1;
    }
  }

  return data;
}

int lexratiotest(MatrixXd B, MatrixXd e)
{
  MatrixXd b = MatrixXd::Ones(B.rows(),1);
  MatrixXd B_inv = B.inverse();
  MatrixXd Mn(B.rows(),B.rows()+1);
  Mn << B_inv*b , B.inverse();
  MatrixXd Md(B.rows(),1);
  Md << B_inv*e;

  printf("invB \n");std::cout << B_inv << std::endl << std::endl;
  printf("Mn \n");std::cout << Mn << std::endl << std::endl;
  printf("Md \n");std::cout << Md << std::endl << std::endl;

  int iexit = -1;//((B.rows()+1)/2)^2;
  MatrixXd lmin = MatrixXd::Zero(1,B.rows());
  bool lexx = false;
  for (int i = 0; i < Md.rows(); i++) {
    if (Md(i,0)>0) {
      if (iexit == -1) {
        lmin = Mn(i,all)/Md(i,0);
        iexit = i;
        printf("lex \n");std::cout << lmin << "," << iexit << std::endl << std::endl;
      } else {
        lexx = lexlt(Mn(i,all)/Md(i,0),lmin);
        if (lexx) {
          lmin = Mn(i,all)/Md(i,0);
          iexit = i;
          printf("lex \n");std::cout << lmin << "," << iexit << std::endl << std::endl;
        }
      }

    }
  }
  printf("Exit-lex \n");std::cout << iexit << std::endl << std::endl;
  return iexit; //return iexit;
}

bool lexlt(MatrixXd v, MatrixXd w)
{
  MatrixXd u = w-v;
  int j = 0;
  int i = 0;
  bool ans = false;
  while (i < u.cols()) {
    if ((u(0,i)!=0.0)&&(u(0,i)!=(-0.0))) {
      j = i;
      i = u.cols();
      if (u(0,j)>0.000001) {
        ans = true;
      }
    }
    i = i+1;
  }
  printf("ratio \n");std::cout << j << std::endl << std::endl;
  printf("lex-u \n");std::cout << u << std::endl << std::endl;
  return ans;
}

simplexData updateRew(simplexData data, int robot, int N, geometry_msgs::PoseWithCovariance pos_r, std::vector<geometry_msgs::PoseWithCovariance> pos_t)
{
  MatrixXd vec(2,1);
  for (int i = 0; i < N; i++) {
    vec(0)= pos_r.pose.position.x - pos_t.at(i+1).pose.position.x;
    vec(1)= pos_r.pose.position.y - pos_t.at(i+1).pose.position.y;
    data.rew(0,(N*(robot-1))+i)=1-exp(-vec.norm()); //norm or squared norm?
    data.age(0,(N*(robot-1))+i)=0;
  }
  return data;
}

simplexData updateMsg(simplexData data, int robot, int N, std::vector<simplex::simplex> dataRc, int Net[])
{
  for (int i = 1; i < (N+1); i++) {
    //if (true) {
    if (Net[N*(robot-1) + i-1]==1) {
      if (i!=robot) {
        for (int j = 0; j < (data.size); j++) {
          if (data.age(0,(dataRc.at(i).b[j]))>=dataRc.at(i).age[j]) {
            data.rew(0,(dataRc.at(i).b[j])) = dataRc.at(i).rew[j];
            data.age(0,(dataRc.at(i).b[j])) = dataRc.at(i).age[j];
          }
        }
      }
    }
  }
  return data;
}

simplexData updateAge(simplexData data, int N)
{
  MatrixXd onee = MatrixXd::Ones(1,data.age.cols());
  data.age = data.age + onee;
  data.rew_pc = data.rew;
  for (int i = 0; i < (data.rew.cols()-(2*N-1)); i++) {
    if (data.age(0,i)>N) {
      data.rew(0,i) = FLT_MAX; //float max
    } else if (data.age(0,i)>1) {
      data.rew_pc(0,i) = data.rew(0,i) + data.age(0,i)*(data.rew(0,i)-data.rew_old(0,i));
    }
  }
  //data.rew_pc = data.rew;
  return data;
}

void send_data(simplexData data, int robot, int N, ros::Publisher topData_pub, bool send)
{
  simplex::simplex dataSn;

  std::vector<float>   rew_msg (data.size,FLT_MAX);
  std::vector<int32_t> b_msg (data.size,N*N);
  std::vector<int32_t> age_msg (data.size,N+1);

  dataSn.header.stamp = ros::Time::now();
  dataSn.id = robot;

  //update Msg according to policy
  switch (data.policy) {
    case 0:
    {
      for (int i = 0; i < (data.size); i++) {
        b_msg[i] = data.ibasic(i,0);
        rew_msg[i] = data.rew(0,b_msg[i]);
        age_msg[i] = data.age(0,b_msg[i]);
      }
    }
      break;

    case 1:
    {
      int i2 = 0;
      for (int j = 0; j < (2*N-1); j++) {
        if ((data.xbasic(j,0)==1)&&(data.ibasic(j,0)<(N*N))) {
          b_msg[i2] = data.ibasic(j,0);
          rew_msg[i2] = data.rew(0,b_msg[i2]);
          age_msg[i2] = data.age(0,b_msg[i2]);
          i2 = i2+1;
        }
      }
    }
      break;

    case 2:
    {
      int i3 = 0;
      for (int j = 0; j < (2*N-1); j++) {
        if ((data.xbasic(j,0)==1)&&(data.ibasic(j,0)<(N*N))) {
          b_msg[i3]   = data.ibasic(j,0);
          rew_msg[i3] = data.rew(0,b_msg[i3]);
          age_msg[i3] = data.age(0,b_msg[i3]);
          i3 = i3+1;
        }
      }
      ArrayXf array = ArrayXf::LinSpaced(N,N*(robot-1),N*(robot)-1);
      for (int j = 0; j < N; j++) {
        if (array(j)!=(((robot-1)*N)+data.Assign-1)) {
          b_msg[i3]   = array(j);
          rew_msg[i3] = data.rew(0,b_msg[i3]);
          age_msg[i3] = data.age(0,b_msg[i3]);
          i3 = i3+1;
        }
      }
    }
      break;

    case 3:
    {
      int i4 = 0;
      for (int j = 0; j < (2*N-1); j++) {
        if ((data.xbasic(j,0)==1)&&(data.ibasic(j,0)<(N*N))) {
          b_msg[i4]   = data.ibasic(j,0);
          rew_msg[i4] = data.rew(0,b_msg[i4]);
          age_msg[i4] = data.age(0,b_msg[i4]);
          i4 = i4+1;
        }
      }
      MatrixXd rew = data.rew(0,all);
      int base;
      for (int i = 0; i < (2*N-1); i++) {
        if (data.xbasic(i,0)==1) {
          base = data.ibasic(i,0);
          rew(0,base) = 1.0;
        }
      }
      std::ptrdiff_t ii, jj;
      float minC;
      for (int j = 0; j < (2*N-1); j++) {
        minC = rew.minCoeff(&ii,&jj);
        rew(0,jj) = 1.0;
        b_msg[i4]   = jj;
        rew_msg[i4] = data.rew(0,b_msg[i4]);
        age_msg[i4] = data.age(0,b_msg[i4]);
        i4 = i4+1;
      }
    }
      break;

    case 4:
    {
      int i5 = 0;
      for (int j = 0; j < (2*N-1); j++) {
        if ((data.xbasic(j,0)==1)&&(data.ibasic(j,0)<(N*N))) {
          b_msg[i5]   = data.ibasic(j,0);
          rew_msg[i5] = data.rew(0,b_msg[i5]);
          age_msg[i5] = data.age(0,b_msg[i5]);
          i5 = i5+1;
        }
      }
      MatrixXd age = data.age(0,all);
      int base;
      for (int i = 0; i < (2*N-1); i++) {
        if (data.xbasic(i,0)==1) {
          base = data.ibasic(i,0);
          age(0,base) = 100.0;
        }
      }
      std::ptrdiff_t ii, jj;
      float minA;
      for (int j = 0; j < (2*N-1); j++) {
        minA = age.minCoeff(&ii,&jj);
        age(0,jj) = 100.0;
        b_msg[i5]   = jj;
        rew_msg[i5] = data.rew(0,b_msg[i5]);
        age_msg[i5] = data.age(0,b_msg[i5]);
        i5 = i5+1;
      }
    }

      break;

    case 5:
    {
      for (int i = 0; i < (data.size); i++) {
        b_msg[i]   = i;
        rew_msg[i] = data.rew(0,b_msg[i]);
        age_msg[i] = data.age(0,b_msg[i]);
      }
    }

      break;
  }

  dataSn.rew = rew_msg;
  dataSn.b = b_msg;
  dataSn.age = age_msg;
  dataSn.assign = data.Assign;

  dataRc.at(robot)=dataSn;
  if (send) {
    msg_count(robot-1) = msg_count(robot-1) + 1;
    topData_pub.publish(dataSn);
  }
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

float orientation_f(geometry_msgs::PoseWithCovariance robot, geometry_msgs::PoseWithCovariance target)
{
    float PI = 3.1415926535897;
    float a_angle, b_angle, h_angle, sen_angle, cos_angle;
    float angle_goal, angle_init;

    // obtain robot i orientation [Quaternion to Euler angles]
    sen_angle = 2*(robot.pose.orientation.z*robot.pose.orientation.w);
    cos_angle = 1 - 2*(robot.pose.orientation.z*robot.pose.orientation.z);
    angle_init = std::atan2(sen_angle,cos_angle);

    if (angle_init < 0) {
      angle_init = 2*PI + angle_init; // 0º->360º instead of +-180º
    }

    // obtain goal orientation [Trigonometry pos(x,y)]
    a_angle = (target.pose.position.y-robot.pose.position.y);
    b_angle = (target.pose.position.x-robot.pose.position.x);
    h_angle = sqrt((a_angle*a_angle) + (b_angle*b_angle));
    sen_angle = a_angle/h_angle;
    cos_angle = b_angle/h_angle;
    angle_goal = std::atan2(sen_angle,cos_angle);

    if (angle_goal < 0) {
      angle_goal = 2*PI + angle_goal; // 0º->360º instead of +-180º
    }
    float vel_angular = angle_goal-angle_init;
    if (abs(vel_angular) > PI) {
      vel_angular = -vel_angular; // both clockwise direcction
    }
    return vel_angular;
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
void poseRobotMessage(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  pos_r = msg.pose; //i pose subscriber
}
void poseTargetMessage(geometry_msgs::PoseWithCovarianceStampedConstPtr msg, int nn) {
  pos_t.at(nn) = msg->pose; //N target pose subscribers
}
void getDataMessage(simplex::simplex msg) {
  dataRc.at(msg.id) = msg; //robots messages subscribers
  msg_count(msg.id - 1) = msg_count(msg.id - 1) + 1;
}
void getNetDefinition(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	int i = 0;
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Net[i] = *it;
		i++;
	}
	return;
}
void getCosts(simplex::IntList msg)
{
  PD_costs = msg;
}
void getAngles(simplex::IntList msg)
{
  PD_angles = msg;
}
