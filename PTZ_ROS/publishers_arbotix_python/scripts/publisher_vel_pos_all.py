#!/usr/bin/env python
    
#the top command line is used by every python Ros node. 


import rospy  #it is necessary to import rospy if you are writing a Ros Node with python
from std_msgs.msg import Float64 #Float64 is the type of message that you are going to publish in the topic
from publishers_arbotix_python.srv import * # In this node two services are also used, so you have to import them fron the folder srv inside your directory

def set_speed_1_client(): #this function is used to set the servo1's speed 
	
	vel_1 = float(input("Introduzca la velocidad del servo1:")) #it takes a string from the terminal and converts it into a float
	rospy.wait_for_service('/servo1/set_speed') #This is a convenience method that blocks until the service named /servo1/set_speed is available
	
        try:
		set_speed_1 = rospy.ServiceProxy('/servo1/set_speed', SetSpeed) # Next we create a handle for calling the service
		# '/servo1/set_speed' is de server node that we call and 'SetSpeed' is the server type that we have in the folder.srv
		set_speed_1(vel_1)# we execute the call passing the velocity as the argument 
		print "Call success"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

#regarding the Ros tutorials, this part of the code would be the client node of the service, the server node '/servo1/set_speed' it is already created by arbotix and we only have to call it


def set_speed_2_client(): #this function is used to set the servo2's speed 
	
	vel_2 = float(input("Introduzca la velocidad del servo2:"))
	rospy.wait_for_service('/servo2/set_speed')	
	
        try:
		set_speed_2 = rospy.ServiceProxy('/servo2/set_speed', SetSpeed)
		set_speed_2(vel_2)
		print "Call success"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e


def talker_pos_1(): #this function runs the whole node and publish the positions in the topics '/servo[1,2]/command'

	pub_1 = rospy.Publisher('/servo1/command', Float64, queue_size=10)
        #pub_1 = rospy.Publisher('/servo1/command', Float64, queue_size=10) declares that your node is publishing to the /servo1/command topic using the message type Float64
	pub_2 = rospy.Publisher('/servo2/command', Float64, queue_size=10)
	rospy.init_node('talker_pos_1', anonymous=True)
	#rospy.init_node(NAME, ...), is very important as it tells rospy the name of your node
	

	rate = rospy.Rate(10) # 10hz we expect to go through the loop 10 times per second
                           
    	while not rospy.is_shutdown():

		set_speed_1_client() #we run the function which calls the service
		set_speed_2_client()

		pos_1 = float(input("Introduzca la posicion del servo1:"))#take the positions from the terminal
		pos_2 = float(input("Introduzca la posicion del servo2:")) 

		pub_1.publish(pos_1) #publish in the topic the position 
		pub_2.publish(pos_2)

		rate.sleep()

         
if __name__ == '__main__':
	try:
		
        	talker_pos_1()#we run the funcions which runs the whole node
		
	except rospy.ROSInterruptException:
		pass


	#es un nodo de ros que publica en los topics '/servo1/command' ; '/servo2/command'
	#los cuales mueven los motores del ptz
	#tambien se subscrive a los servicios '/servo1/set_speed' y '/servo2/set_speed'
	#para dar consigna de velocidadad a los motores
	








