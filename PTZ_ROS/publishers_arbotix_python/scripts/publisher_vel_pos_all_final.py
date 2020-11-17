#!/usr/bin/env python
    
#the top command line is used by every python Ros node. 


import rospy  #it is necessary to import rospy if you are writing a Ros Node with python
from std_msgs.msg import Float64 #Float64 is the type of message that you are going to publish in the topic
from std_msgs.msg import Float64MultiArray
from publishers_arbotix_python.srv import * # In this node two services are also used, so you have to import them fron the folder srv inside your directory
import math


#to publish the angles 
pub_1 = rospy.Publisher('/servo1/command', Float64, queue_size=1)
pub_2 = rospy.Publisher('/servo2/command', Float64, queue_size=1)




def set_speed_1_client(): #this function is used to set the servo1's speed 
	
	#vel_1 = float(input("Introduzca la velocidad del servo1:")) #it takes a string from the terminal and converts it into a float
	vel_1=1.0
	rospy.wait_for_service('/servo1/set_speed') #This is a convenience method that blocks until the service named /servo1/set_speed is available
	
        try:
		set_speed_1 = rospy.ServiceProxy('/servo1/set_speed', SetSpeed) # Next we create a handle for calling the service
		# '/servo1/set_speed' is de server node that we call and 'SetSpeed' is the server type that we have in the folder.srv
		set_speed_1(vel_1)# we execute the call passing the velocity as the argument 
		print "Call success 1"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

#regarding the Ros tutorials, this part of the code would be the client node of the service, the server node '/servo1/set_speed' it is already created by arbotix and we only have to call it


def set_speed_2_client(): #this function is used to set the servo2's speed 
	
	#vel_2 = float(input("Introduzca la velocidad del servo2:"))
	vel_2=1.0
	rospy.wait_for_service('/servo2/set_speed')	
	
        try:
		set_speed_2 = rospy.ServiceProxy('/servo2/set_speed', SetSpeed)
		set_speed_2(vel_2)
		print "Call success 2"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e


def get_pos_2(): #this function is used to get the servo2's actual angle  

	rospy.wait_for_service("/servo2/get_position")
	
	try:
		get_position_2 = rospy.ServiceProxy('/servo2/get_position', GetPosition)
		print "Call success 3"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

	position=get_position_2().position
	#print (position)

	return position
	# en la carpeta srv tienes los servicios a los que te pudes subscribir
	#en este caso queremos subscribirnos al GetPosition el cual no tiene inputs pero si outputs
	#output=get_position_2(input)
	#en el caso de setspeed es al reves, tiene un input pero no outputs

def get_pos_1():

	rospy.wait_for_service("/servo1/get_position")
	
	try:
		get_position_1 = rospy.ServiceProxy('/servo1/get_position', GetPosition)
		print "Call success 4"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

	position=get_position_1().position
	#print (position)

	return position

def callback(data):

	#from the node get_3dpoint we get the 3D coordenates of the point
	print("I have heard",data.data[0], data.data[1], data.data[2], data.data[3])
	#print("I have heard",data.data)
	
	x_cam=data.data[0]
	y_cam=data.data[1]
	z_cam=data.data[2]
	depth=data.data[3]

	#x_cam = float(input("Introduzca coordenada X:"))
	#y_cam = float(input("Introduzca coordenada Y:"))
	#z_cam = float(input("Introduzca la coordenada Z:"))
	#depth = float(input("Introduzca la profundidad:"))

	set_speed_1_client() #we run the function which calls the service
	set_speed_2_client()
	#angulo del servo dos inicial
	angle_tita=get_pos_2() #lo devuelve en radianes
	#angulo del servo uno inicial
	angle_servo1=get_pos_1()
	#print("angulo tita:", angle_tita)

	if depth != 0.0 or depth == float('NaN'):

		#posicionar el pixel en yaw (horizontal) en el centro de la imagen
		
		#midiendo con calibre se supone que es 7.65
		ofset=80.0
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
		#print("esto es coef_x:",coef_x)
		coef_y=-y_cam+ofset
		#print("esto es coef_y:",coef_y)
		coef_a=((1-coef_x)**2-ofset**2)
		#print("esto es coef_a:",coef_a)
		coef_b=2*(1-coef_x)*coef_y
		#print("esto es coef_b:",coef_b)
		coef_c=coef_y**2-ofset**2
		#print("esto es coef_c:",coef_c)
		m_1=(-coef_b+math.sqrt(coef_b**2-4*coef_a*coef_c))/(2*coef_a)
		#print("esto es m_1:",m_1)
		m_2=(-coef_b-math.sqrt(coef_b**2-4*coef_a*coef_c))/(2*coef_a)
		#print("esto es m_2:",m_2)		

		if m_1>m_2:
			m=m_2
		else:
			m=m_1
		
		#print("esto es m:",m)
		h=coef_y+m*-coef_x
		#print("esto es h:",h)
		#el numero que introduces al arcoseno debe ser  menor que uno 
		#aveces por redondeos sale un poco menos por eso hay que limitarlo
		
		input_acos=ofset/h

		if input_acos>1.0:
			print("input del arcoseno:",input_acos)
			input_acos=1.0
		
		incremento_pos_2=math.acos(input_acos)

		incremento_pos_2_grados=math.degrees(incremento_pos_2)
		#print("esto es el incremento:",incremento_pos_2_grados)
		#print("esto es el angulo actual:",angle_tita)

		if coef_y < ofset:
			incremento_pos_2=-incremento_pos_2

		#print("esto es el angulo a restar o sumar:",incremento_pos_2)
		pos_2=angle_tita+incremento_pos_2
	

		#take the positions from the terminal
		#pos_1 = float(input("Introduzca la posicion del servo1:"))
		#pos_2 = float(input("Introduzca la posicion del servo2:"))
		
		
		#publish in the topic the position 
		pub_1.publish(pos_1) 
		pub_2.publish(pos_2)
	else:
			
		pub_1.publish(angle_servo1) 
		pub_2.publish(angle_tita)
	
	print "hola 3"


	#rate_1.sleep()




def talker_pos_1(): #this function runs the whole node and publish the positions in the topics '/servo[1,2]/command'

	rospy.init_node('talker_pos_1', anonymous=True)
	print "hola 1"
	rospy.Subscriber("chatter",Float64MultiArray,callback,queue_size=1)
	print "hola 2"
	rospy.spin()

if __name__ == '__main__':
	try:
		
        	talker_pos_1()#we run the functions which runs the whole node
		
	except rospy.ROSInterruptException:
		pass

	#es un nodo de ros que publica en los topics '/servo1/command' ; '/servo2/command'
	#los cuales mueven los motores del ptz
	#tambien se subscrive a los servicios '/servo1/set_speed' y '/servo2/set_speed'
	#para dar consigna de velocidadad a los motores
	#en este caso por comodidad la velocidad esta fijada



















