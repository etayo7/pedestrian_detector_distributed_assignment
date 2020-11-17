#include "ros/ros.h"//simpre obligatorio ponerlo
#include "std_msgs/String.h" //tipo de mensaje que va a publicar
#include <std_msgs/Float64.h> 
#include <stdlib.h> //libreria necesaria para algunos comandos utilizados de C++
#include <sstream>
#include <string>
#include "publishers_arbotix/GetPosition.h" //esto es para el nodo servicio
#include "publishers_arbotix/SetSpeed.h" //esto es para el nodo servicio
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

	ros::init(argc, argv, "publisher"); //to set your publisher file's name

	ros::NodeHandle n; //to initialize this node

	ros::Publisher servo1_pub = n.advertise < std_msgs::Float64
			> ("/servo1/command", 1000);
	ros::Publisher servo2_pub = n.advertise < std_msgs::Float64
			> ("/servo2/command", 1000);
	//the advertise()function is how you tell ROS that you want to publish on a given topic name
	//ros::Publisher [el publisher] = n.advertise < std_msgs::[tipo de mensaje a publicar] > ("[el topic]", [cuantas 		//publicaciones pueden almacenarse en cola ); 




	ros::ServiceClient servo1_pos_service = n.serviceClient
			< publishers_arbotix::GetPosition > ("/servo1/get_position");
	//to call the service already created as GetPosition in arbotix
	//ros::ServiceClient [service's name to call it] = n.serviceClient < [carpeta donde esta]::[file to execute] > ("[el servicio que utiliza de arbotix]");
	ros::ServiceClient servo2_pos_service = n.serviceClient
			< publishers_arbotix::GetPosition > ("/servo2/get_position");

	ros::ServiceClient servo1_speed_service = n.serviceClient
			< publishers_arbotix::SetSpeed > ("/servo1/set_speed");
	ros::ServiceClient servo2_speed_service = n.serviceClient
			< publishers_arbotix::SetSpeed > ("/servo2/set_speed");

	//los servicios pueden tener preguntar datos o responder
	//srv.request.[]
	//srv.response.[]



	float pos = -2.000000;
	bool add = true; //tipo booleano
	bool seguir = true;
	int respuesta = 1;

	while (ros::ok()) {
		if (argc > 1) {
			if (!strcmp(argv[1], "help")) {
				ROS_INFO("%s", help_text.c_str());
				return 0;
			} else if (!strcmp(argv[1], "get")) {
				if (!strcmp(argv[2], "pos")) {
					if (!strcmp(argv[3], "1")) {
						publishers_arbotix::GetPosition srv; //carpeta donde esta el fichero a ejecutar :: the file's name
						if (servo1_pos_service.call(srv)) { //es para llamar al servicio ros::ServiceClient servo1_pos_service = ...
							ROS_INFO("%f", srv.response.position); //this line print the position on the screen which is get from de service srv.response.position
							//ROS_INFO es como print pero le puesdes pasar como argumento un mensaje o la respuesta de un servicio como lo es "position" en GetPosition.srv
							return 0;
						}
					} else if (!strcmp(argv[3], "2")) {
						publishers_arbotix::GetPosition srv;

						if (servo2_pos_service.call(srv)) {
							ROS_INFO("%f", srv.response.position);//si te metes en el propio servicio dentro de arbotix /catkin_ws/src/publishers_arbotix/srv/GetPosition.srv te da el argumetno con el que responde (position en este caso) 
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

			else if (!strcmp(argv[1], "mvspeed1")) { //strcmp es par comparar strings
				publishers_arbotix::SetSpeed srv; //carpeta donde esta el fichero a ejecutar :: the file's name
				srv.request.speed = atof(argv[3]); //convierte un string en su valor numérico en este caso la velocidad
				//cuando pones como argumento mvspeed1 left 2 -> el dos lo metes como un char, para pasarlo a 					//un float se usa atof 
				if (servo1_speed_service.call(srv)) {
				}//es para llamar al servicio

				if (!strcmp(argv[2], "left")) {
					pos = -3.15;
				} else if (!strcmp(argv[2], "right")) {
					pos = 3.15;
				}
				//the following commands lines are to broadcast a message on ROS
				std_msgs::Float64 msg; //tipo de mensaje en nuestro caso un float
				std::stringstream ss;
				ss << argv[1] << pos;
				msg.data = pos;
				servo1_pub.publish(msg); //hace que el publisher creado publique el mensaje en el topic
				//viene de aqui ros::Publisher [servo1_pub] = n.advertise < std_msgs::Float64 > ("/servo1/command", 1000);
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
				//lo mismo que lo de arriba pero para el servo 2
			}

			else if (!strcmp(argv[1], "vel1_pos1")) {
				publishers_arbotix::SetSpeed srv;
				srv.request.speed = atof(argv[2]);
				if (servo1_speed_service.call(srv)) {
				}
				while(seguir){
			        	//pos = atof(argv[3]);
					
					printf("Introduzca la posicion [-3,3] : ");
					scanf("%f",&pos);
					std_msgs::Float64 msg;
					std::stringstream ss;
					ss << argv[1] << pos;
					msg.data = pos;
					servo1_pub.publish(msg);
					
					printf("Seguir introduciendo datos? [yes=1,no=0]: ");
					scanf("%d",&respuesta);

					if(respuesta == 0 ){
						seguir=false;
					}
				return 0;
				}
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

				ROS_INFO("%f", msg.data); //ros_info es como print pero le puedes pasar un mensaje como parámetro

				servo1_pub.publish(msg);//publica los mensajes en el topic, la primera posición que se va apublicar es -2.0 ya que es la que por defecto tiene la variable "pos" declarada al principio del todo
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
