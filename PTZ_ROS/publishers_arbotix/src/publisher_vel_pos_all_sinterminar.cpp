#include "ros/ros.h"// ros/ros.h is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system.
#include "std_msgs/String.h" //This includes the std_msgs/String message, which resides in the std_msgs package
#include "std_msgs/Float64.h" //This includes the std_msgs/Float64 message, which resides in the std_msgs package ***esto se ha cambiado 
#include <stdlib.h> // C++ library used for certain commans as (!strcmp(argv[1], "get") to compare strings
#include <sstream> //C++ library to use stringstream a buffer to store info
#include <string> // C++ library to use strings
#include "publishers_arbotix/GetPosition.h" //necessary for using the service GetPosition.srv which is in the folder.srv
#include "publishers_arbotix/SetSpeed.h" 
#include <iostream>
using namespace std;


float servo_1_get_pos()
{
	float response;

	ros::ServiceClient servo1_pos_service = n.serviceClient< publishers_arbotix::GetPosition > ("/servo1/get_position");
	
	publishers_arbotix::GetPosition srv; 
	//step one: set a service type [folder where it is::Name of the service]

	if (servo1_pos_service.call(srv)) {
 	//In this case a request from the client is not needed
	//step two: call an specific cliente that use the previous service type	
		response= srv.response.position				
	} 

	//this line print the position on the screen which is get from de service 
	//srv.response.position 
	return response;
	
}


void servo_2_get_pos()
{
	float response;
	ros::ServiceClient servo2_pos_service = n.serviceClient< publishers_arbotix::GetPosition > ("/servo2/get_position");
	publishers_arbotix::GetPosition srv; 
	//step one: set a service type [folder where it is::Name of the service]

	if (servo2_pos_service.call(srv)) {
 	//In this case a request from the client is not needed
	//step two: call an specific cliente that use the previous service type	
		response= srv.response.position	
	}			
 
	//this line print the position on the screen which is get from de service 
	//srv.response.position 
	return response;

}


void servo_1_set_speed()
{
	ros::ServiceClient servo1_speed_service = n.serviceClient< publishers_arbotix::SetSpeed > ("/servo1/set_speed");

	publishers_arbotix::SetSpeed srv;
	srv.request.speed = 1.0;
	//the same as in mvspeed1 but with the second servo
	return 0;
}




void servo_2_set_speed()
{
	ros::ServiceClient servo2_speed_service = n.serviceClient< publishers_arbotix::SetSpeed > ("/servo2/set_speed");
	publishers_arbotix::SetSpeed srv;
	srv.request.speed = 1.0;
	//the same as in mvspeed1 but with the second servo
	return 0;
}




void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	
	float ofset,lado1,x_cam,y_cam,z_cam,depth,angle_tita,angle_servo1,angle_rho,angle_rho1;
	
	ofset=80;

	printf("I heard : %f %f %f %f ", msg->data[0],msg->data[1],msg->data[2],msg->data[3],);
	x_cam=msg.data[0];
	y_cam=msg.data[1];
	z_cam=msg.data[2];
	depth=msg.data[3];

	angle_tita=servo_2_get_pos();
	angle_servo1=servo_1_get_pos();
	void servo_1_set_speed();
	void servo_2_set_speed();

	if (depth != 0.0 or depth == float('NaN'))
	{
		//posicionar el pixel en yaw (horizontal) en el centro de la imagen
		
		//midiendo con calibre se supone que es 7.65
	
		lado1=abs(sin(angle_tita)*ofset) 
		angle_rho=abs(angle_tita)
		angle_rho1=atan(abs(y_cam)/abs(z_cam))

		if ((y_cam < 0.0 and angle_tita < 0.0) or (y_cam > 0.0 and angle_tita >0.0))
		{
			angle_rho2=angle_rho+angle_rho1
		}
		else
		{
			angle_rho2=abs(angle_rho-angle_rho1)
		}
		lado2=math.sqrt(z_cam**2+y_cam**2)
		lado3=math.cos(angle_rho2)*lado2


		if (angle_tita > 0.0)
		{
			//la camara esta mirando hacia arriba
			giro = math.atan(x_cam/(lado3-lado1))
		}
		else
		{
			//la camara esta mirando hacia abajo
			giro = math.atan(x_cam/(lado3-lado1))
		}


		pos_1=angle_servo1+giro

		if pos_1 > 3.15 or pos_1 < -3.15:
			pos_1=0.0
			print("invalid angle for servo_1")

		coef_x=z_cam
		coef_y=-y_cam+ofset
		coef_a=((1-coef_x)**2-ofset**2)
		coef_b=2*(1-coef_x)*coef_y
		coef_c=coef_y**2-ofset**2
		m_1=(-coef_b+math.sqrt(coef_b**2-4*coef_a*coef_c))/(2*coef_a)
		m_2=(-coef_b-math.sqrt(coef_b**2-4*coef_a*coef_c))/(2*coef_a)
		

		if m_1>m_2:
			m=m_2
		else:
			m=m_1
		

		h=coef_y+m*-coef_x
		input_acos=ofset/h

		if input_acos>1.0:
			print("input del arcoseno:",input_acos)
			input_acos=1.0
		
		incremento_pos_2=math.acos(input_acos)

		incremento_pos_2_grados=math.degrees(incremento_pos_2)

		if coef_y < ofset:
			incremento_pos_2=-incremento_pos_2

		
		pos_2=angle_tita+incremento_pos_2
	

		
		pub_1.publish(pos_1) 
		pub_2.publish(pos_2)
	}
	else
	{		
		pub_1.publish(angle_servo1) 
		pub_2.publish(angle_tita)
	}
	

	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);
	ros::spin();
	return 0;
}













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



	ros::init(argc, argv, "publisher"); //to set your publisher file's name

	/*
 	The ros::init() function needs to see argc and argv so that it can perform
      	any ROS arguments and name remapping that were provided at the command line.
       	For programmatic remappings you can use a different version of init() which takes
      	remappings directly, but for most command-line programs, passing argc and argv is
       	the easiest way to do it.  The third argument to init() is the name of the node.
       	You must call one of the versions of ros::init() before using any other
      	part of the ROS system.
     	*/


	ros::NodeHandle n; //to initialize this node



	ros::Publisher servo1_pub = n.advertise < std_msgs::Float64
			> ("/servo1/command", 1000);
	ros::Publisher servo2_pub = n.advertise < std_msgs::Float64
			> ("/servo2/command", 1000);
	//the advertise()function is how you tell ROS that you want to publish on a given topic name
	//ros::Publisher [publisher's name] = n.advertise < std_msgs::[message type] > ("[topic]", [maximum number of messages stored]); 




	ros::ServiceClient servo1_pos_service = n.serviceClient
			< publishers_arbotix::GetPosition > ("/servo1/get_position");
	
	ros::ServiceClient servo2_pos_service = n.serviceClient
			< publishers_arbotix::GetPosition > ("/servo2/get_position");

	ros::ServiceClient servo1_speed_service = n.serviceClient
			< publishers_arbotix::SetSpeed > ("/servo1/set_speed");

	ros::ServiceClient servo2_speed_service = n.serviceClient
			< publishers_arbotix::SetSpeed > ("/servo2/set_speed");

	//This creates a client for the /servo1/get_position service. The ros::ServiceClient object is used to call the service later on.
	//ros::ServiceClient [service's name to call it] = n.serviceClient < [directory's name in which you work]::[file to execute] > ("[name of the service from arbotix that is going to be used]");


	//the client service has the request srv.request.[]
	//the server service has the response srv.response.[]
	


	float pos = -2.000000;
	float vel = 0.0;

	float pos1 = 0.000000;
	float pos2 = 0.000000;
	float vel1 = 0.0;
	float vel2 = 0.0;
	float pos_actual = 4.0;
	bool add = true; 
	bool seguir = true;
	bool seguir_2 = true;
	int respuesta = 1;

	while (ros::ok()) {
		if (argc > 1) {
			if (!strcmp(argv[1], "help")) { //get the position of the servos
				ROS_INFO("%s", help_text.c_str());
				return 0;
			} else if (!strcmp(argv[1], "get")) {
				if (!strcmp(argv[2], "pos")) {
					if (!strcmp(argv[3], "1")) {

		//with those if we compare the input from the keyboard (a string when we execute the node to execute an specific part of the code

						publishers_arbotix::GetPosition srv; 
						//step one: set a service type [folder where it is::Name of the service]

						if (servo1_pos_service.call(srv)) {
 						//In this case a request from the client is not needed
						//step two: call an specific cliente that use the previous service type
						
							ROS_INFO("%f", srv.response.position); 
							//this line print the position on the screen which is get from de service 
							//srv.response.position 
							return 0;
						}
					} else if (!strcmp(argv[3], "2")) {
						publishers_arbotix::GetPosition srv;

						if (servo2_pos_service.call(srv)) {
							ROS_INFO("%f", srv.response.position);//you can see the argument which the server is
   							//going to response in arbotix /catkin_ws/src/publishers_arbotix/srv/GetPosition.srv 
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

			else if (!strcmp(argv[1], "mvspeed1")) { //set a velocity and move the first servo to a fixed position
				publishers_arbotix::SetSpeed srv; 
				srv.request.speed = atof(argv[3]); 
				//transform a string into a number which is used for the request's argument
				//In this case the client need to send a request, the velocity, however there is no response from the server 
				//client
				 
				if (servo1_speed_service.call(srv)) {
				}

				if (!strcmp(argv[2], "left")) {
					pos = -3.15;
				} else if (!strcmp(argv[2], "right")) {
					pos = 3.15;
				}
				//the following commands lines are to broadcast a message on ROS
				std_msgs::Float64 msg; //we create a message type Float64 called msg
				msg.data = pos; //we assign the float pos to the message
				servo1_pub.publish(msg); //Now we actually broadcast the message to anyone who is connected. 
				//we the previous comand we call ros::Publisher [servo1_pub] = n.advertise < std_msgs::Float64 > ("/servo1/
				//command", 1000);
			}

			else if (!strcmp(argv[1], "mvspeed2")) { //set a velocity and move the second servo to a fixed position
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
				//the same as in mvspeed1 but with the second servo
			}

			else if (!strcmp(argv[1], "vel1_pos1")) { //set the velocity and the position for the first servo
				publishers_arbotix::SetSpeed srv;
				
				while(seguir){
			    
					printf("Introduzca la velocidad del servo1 : ");
					scanf("%f",&vel);
					srv.request.speed = vel;
					if (servo1_speed_service.call(srv)) {
					}


					printf("Introduzca la posicion del servo1 [-3,3] : ");
					scanf("%f",&pos);
					std_msgs::Float64 msg;
					msg.data = pos;
					servo1_pub.publish(msg);
					
					printf("Seguir introduciendo datos? [yes=1,no=0]: ");
					scanf("%d",&respuesta);

					if(respuesta == 0 ){
						seguir=false;
					}
				
				}
			return 0;
			}


			else if (!strcmp(argv[1], "vel2_pos2")) { //set the velocity and the position for the second servo
				publishers_arbotix::SetSpeed srv;
				
				while(seguir){
			    
					printf("Introduzca la velocidad del servo2 : ");
					scanf("%f",&vel);
					srv.request.speed = vel;
					if (servo2_speed_service.call(srv)) {
					}


					printf("Introduzca la posicion del servo 2 [-2,2] : ");
					scanf("%f",&pos);
					std_msgs::Float64 msg;
					msg.data = pos;
					servo2_pub.publish(msg);
					
					printf("Seguir introduciendo datos? [yes=1,no=0]: ");
					scanf("%d",&respuesta);

					if(respuesta == 0 ){
						seguir=false;
					}
				
				}
			return 0;
			}

			else if (!strcmp(argv[1], "vel_all_pos_all")) { //set the velocity and the position for both servos
				publishers_arbotix::SetSpeed srv;
				
				while(seguir){
		
					printf("Introduzca la velocidad del servo1 : ");
					scanf("%f",&vel1);
					srv.request.speed = vel1;
					if (servo1_speed_service.call(srv)) {
					}
					
					printf("Introduzca la velocidad del servo2 : ");
					scanf("%f",&vel2);
					srv.request.speed = vel2;
					if (servo2_speed_service.call(srv)) {
					}

					printf("Introduzca la posicion servo1 [-3,3] : ");
					scanf("%f",&pos1);
					std_msgs::Float64 msg1; 
					msg1.data = pos1;
					

					printf("Introduzca la posicion servo2 [-2,2] : ");
					scanf("%f",&pos2);
					std_msgs::Float64 msg2;
					msg2.data = pos2;

					
					servo1_pub.publish(msg1);
					servo2_pub.publish(msg2);

				
					printf("Seguir introduciendo datos? [yes=1,no=0]: ");
					scanf("%d",&respuesta);

					if(respuesta == 0 ){
						seguir=false;
					}
				
				}
			return 0;
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
