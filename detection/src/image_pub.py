#!/usr/bin/env python
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def start_node():
	start_node( rospy.myargv(argv=sys.argv)[1] )
while not rospy.is_shutdown():
	filename = 'DSC_0051.jpg'
	img = cv2.imread(filename)
	#cv2.imshow("image", img)
	cv2.waitKey(2000)
	bridge = CvBridge()
	imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")

if __name__ == '__main__':
    try:
        start_node(rospy.myargv(argv=sys.argv)[1])
    except rospy.ROSInterruptException:
        pass
