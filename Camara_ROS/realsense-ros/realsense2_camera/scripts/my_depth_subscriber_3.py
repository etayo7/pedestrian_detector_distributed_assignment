#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy
import cv2


refPt = []
cropping = False


def convert_color_image(ros_image):
     bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
     try:
     #Convert the depth image using the default passthrough encoding
                color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	
     except CvBridgeError, e:
 	          print e

     (rows,cols,channels)= color_image.shape

     #muestras por pantalla la imagen
     #cv2.imshow("Image window", color_image)
     #con waitkey esperas hasta pintar la siguiente un timepo x si x=0 esperas infinit


     image =cv2.imread(color_image)
     #clone = color_image.copy()
     #cv2.namedWindow("color_image")
     #cv2.setMouseCallback("color_image", click_and_crop)
     cv2.waitKey(0)


     # initialize the list of reference points and boolean indicating
     # whether cropping is being performed or not
     
 
def click_and_crop(event, x, y, flags, param):
	# grab references to the global variables
	global refPt, cropping
 
	# if the left mouse button was clicked, record the starting
	# (x, y) coordinates and indicate that cropping is being
	# performed
	if event == cv2.EVENT_LBUTTONDOWN:
		refPt = [(x, y)]
		cropping = True
 
	# check to see if the left mouse button was released
	elif event == cv2.EVENT_LBUTTONUP:
		# record the ending (x, y) coordinates and indicate that
		# the cropping operation is finished
		refPt.append((x, y))
		cropping = False
 
		# draw a rectangle around the region of interest
		cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
		cv2.imshow("color_image", image)


def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/camera/color/image_raw", Image, convert_color_image, queue_size=1)
	
	
	rospy.spin()

if __name__ == '__main__':
	
	pixel2depth()

	#Es un intento de que pudas seleccionar un pixel en la imagen directamente y no mediante sus coordenadas
