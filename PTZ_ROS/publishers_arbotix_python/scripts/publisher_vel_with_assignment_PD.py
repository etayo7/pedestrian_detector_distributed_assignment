#!/usr/bin/env python

# PTZ MOVEMENT NODE - talker_pos (Arg 1 = robot id)

# Creator - Inigo Etayo
# Directors - Eduardo Montijano, Danilo Tardioli

# This node with one argument [Arg1 = robot id] receives the relatives coordinates to move
# the PTZ to adjust the image to the assigned target to the robot i. [from Victor Fuertes TFM]

# This node publish to the topics '/servo1/command' and '/servo2/command' which change
# the position of the PTZ. It also calls the services '/servo1/set_speed' and '/servo2/set_speed'
# to choose the desired speed of the rotor, for convenience this speed is already fixed

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from publishers_arbotix_python.srv import *
import math
import time

#to publish the angles
pub_1 = rospy.Publisher('servo1/command', Float64, queue_size=1)
pub_2 = rospy.Publisher('servo2/command', Float64, queue_size=1)

starting_time = time.time()
pre_x = 0.0
pre_z = 0.0
now = rospy.Time(0.0)
t = rospy.Duration(0.5)
pos_max = float(sys.argv[2])


def set_speed_1_client(): #this function is used to set the servo1's speed

	vel_1=0.9
	rospy.wait_for_service('servo1/set_speed') #This is a convenience method that blocks until the service named /servo1/set_speed is available

        try:
		set_speed_1 = rospy.ServiceProxy('servo1/set_speed', SetSpeed) # Next we create a handle for calling the service
		# '/servo1/set_speed' is de server node that we call and 'SetSpeed' is the server type that we have in the folder.srv
		set_speed_1(vel_1)# we execute the call passing the velocity as the argument
		print "Call success 1"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

#regarding the Ros tutorials, this part of the code would be the client node of the service, the server node '/servo1/set_speed' it is already created by arbotix and we only have to call it


def set_speed_2_client(): #this function is used to set the servo2's speed

	vel_2=0.9
	rospy.wait_for_service('servo2/set_speed')

        try:
		set_speed_2 = rospy.ServiceProxy('servo2/set_speed', SetSpeed)
		set_speed_2(vel_2)
		print "Call success 2"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e


def get_pos_2(): #this function is used to get the servo2's actual angle

	rospy.wait_for_service("servo2/get_position")

	try:
		get_position_2 = rospy.ServiceProxy('servo2/get_position', GetPosition)
		print "Call success 3"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

	position=get_position_2().position

	return position

def get_pos_1():

	rospy.wait_for_service("servo1/get_position")

	try:
		get_position_1 = rospy.ServiceProxy('servo1/get_position', GetPosition)
		print "Call success 4"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

	position=get_position_1().position

	return position

def callback(data):

	# from the assingment node we get the 3D coordenates of the point
	x_cam=data.data[0]
	y_cam=data.data[1]
	z_cam=data.data[2]
	depth=data.data[3]
	global pre_x, pre_z
	global now, t
	global pos_max

	# we run the function which calls the service
	set_speed_1_client()
	set_speed_2_client()
	angle_tita=get_pos_2()
	angle_servo1=get_pos_1()
	if (z_cam==0.0)and(depth>0.099)and(depth<0.1001):
		pos_1 = 0.0
		pos_2 = 0.0
		pub_1.publish(pos_1)
		pub_2.publish(pos_2)
	elif depth != 0.0 or depth == float('NaN'):
		if data.data[3] != 0.0:

			#posicionar el pixel en yaw (horizontal) en el centro de la imagen

			#midiendo con calibre se supone que es 7.65
			ofset = 80.0
			lado1=abs(math.sin(angle_tita)*ofset)
			angle_rho=abs(angle_tita)
			angle_rho1=math.atan(abs(y_cam)/abs(z_cam))

			if (y_cam < 0.0 and angle_tita < 0.0) or (y_cam > 0.0 and angle_tita >0.0):
				angle_rho2=angle_rho+angle_rho1
			else:
				angle_rho2=abs(angle_rho-angle_rho1)

			lado2=math.sqrt(z_cam**2+y_cam**2)
			lado3=math.cos(angle_rho2)*lado2


			if angle_tita > 0.0:
				#la camara esta mirando hacia arriba
				giro = math.atan(x_cam/(lado3-lado1))
			else:
				#la camara esta mirando hacia abajo
				giro = math.atan(x_cam/(lado3-lado1))

			#tita es el angulo del servo 2 respecto de la vertical
			#si tita es positivo y x_cam positivo el incremento es positivo
			#si tita es positivo y x_cam negativo el incremento es negativo
			#si tita es negativo y x_cam positivo el incremento es positivo
			#si tita es negativo y x_cam negativo el incremento es negativo
			#no hace falta poner ningun if else porque el signo del giro va acorde
			#con el signo de x_cam y este si es negativo al meterlo en atan sale un giro
			#negativo y viceversa

			pos_1=angle_servo1+giro

			if pos_1 > 3.15 or pos_1 < -3.15:
				pos_1=0.0
				print("invalid angle for servo_1")

			#posicionar el pixel en pitch (vertical) en el centro de la imagen
			#la coordenada y_cam no va a cambiar con el movimiento del servo1
			#la coordenada z_cam aproximamos que tampoco cambia con el anterior movimiento

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

			if coef_y < ofset:
				incremento_pos_2=-incremento_pos_2

			pos_2=angle_tita+incremento_pos_2

			# publish the desired position of the servos if different from previous movement
			secs = rospy.get_rostime() - now
			if (abs(pos_1)-abs(pos_max)) > 0.0:
				pos_1 = np.sign(pos_1)*pos_max
			if secs>t:
				pub_1.publish(pos_1)
				pub_2.publish(pos_2)
			elif (abs(x_cam-pre_x)>40.0) and (abs(z_cam-pre_z)>60.0):
				pub_1.publish(pos_1)
				pub_2.publish(pos_2)
			pre_x = x_cam
			pre_z = z_cam
			now = rospy.get_rostime()
	else:
		now



def talker_pos_1(): #this function runs the whole node and publish the positions in the topics '/servo[1,2]/command'
	rospy.init_node('talker_pos', anonymous=True)
	pub_cost_str = 'chatter';
	img = rospy.Subscriber(pub_cost_str,Float64MultiArray,callback,queue_size=1)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	try:

        	talker_pos_1()#we run the functions which runs the whole node

	except rospy.ROSInterruptException:
		pass
