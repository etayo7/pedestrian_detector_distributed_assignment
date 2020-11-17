#!/usr/bin/env python
    
    
import rospy
from std_msgs.msg import Float64
    
def talker_pos():
    pub = rospy.Publisher('/servo1/command', Float64, queue_size=10)
    rospy.init_node('talker_pos', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	pos_1 = float(input("Introduzca la velocidad del servo1:")) 
	#rospy.loginfo(pos_1)
	pub.publish(pos_1)
	rate.sleep()
         
if __name__ == '__main__':
       try:
           talker_pos()
       except rospy.ROSInterruptException:
           pass
