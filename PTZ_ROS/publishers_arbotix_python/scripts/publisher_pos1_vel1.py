#!/usr/bin/env python
    
    
import rospy
from std_msgs.msg import Float64
from publishers_arbotix_python.srv import *

def set_speed_client():
	
	vel_1 = float(input("Introduzca la velocidad del servo1:"))
	rospy.wait_for_service('/servo1/set_speed')	
	
        try:
		set_speed = rospy.ServiceProxy('/servo1/set_speed', SetSpeed)
		set_speed(vel_1)
        	print "call success"
       	except rospy.ServiceException, e:
           	print "Service call failed: %s"%e

def talker_pos():
	pub = rospy.Publisher('/servo1/command', Float64, queue_size=10)
	rospy.init_node('talker_pos', anonymous=True)
	rate = rospy.Rate(10) # 10hz
    	while not rospy.is_shutdown():
		set_speed_client()
		pos_1 = float(input("Introduzca la posicion del servo1:")) 
		pub.publish(pos_1)
		rate.sleep()
         
if __name__ == '__main__':
	try:
		
		
        	talker_pos()
		
	except rospy.ROSInterruptException:
		pass
