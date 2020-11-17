#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy 
import cv2
import math


pub_talker = rospy.Publisher('chatter', Float64MultiArray, queue_size=1)
u=0
v=0
a=0
b=1

def pixel2depth():
	
	#pub_talker = rospy.Publisher('chatter', Float64MultiArray, queue_size=1)
	rospy.init_node('pixel2depth',anonymous=True)
	print "adios 1"
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, convert_depth_image, queue_size=1)
	rospy.Subscriber("/camera/color/image_raw", Image, convert_color_image, queue_size=1)
	print "adios 2"
	rospy.spin()	

def convert_depth_image( ros_image):

	global u,v,a,b
	print "adios 3"
	bridge = CvBridge()
     	# Use cv_bridge() to convert the ROS image to OpenCV format
     	try:
     	#Convert the depth image using the default passthrough encoding
        	depth_image = bridge.imgmsg_to_cv2(ros_image, "passthrough")

     	except CvBridgeError, e:
 	          print e
     
     	u=int(input("valor de fila:"))	
     	v=int(input("valor de columna:"))
	b=0
	a=0
	

	#rospy.Subscriber("/camera/color/image_raw", Image, convert_color_image, queue_size=1)
	#rospy.Subscriber("/camera/color/image_raw", Image, convert_color_image_2, queue_size=1)

     	#Convert the depth image to a Numpy array
     	depth_array = numpy.array(depth_image, dtype=numpy.float32)
     	depth=depth_array[u][v]

	camera_intrinsics_fx=613.0291137695312	
	camera_intrinsics_fy=613.4072265625

	camera_intrinsics_ppy=250.34303283691406
	camera_intrinsics_ppx=322.4624938964844

	dist_1=u - camera_intrinsics_ppy
	dist_2=v - camera_intrinsics_ppx
	dist_3=math.sqrt(dist_1**2+dist_2**2)
	tita=math.atan(dist_3/camera_intrinsics_fx)

	z = math.cos(tita)*depth
	x = (v - camera_intrinsics_ppx)*z/camera_intrinsics_fx
	y = (u - camera_intrinsics_ppy)*z/camera_intrinsics_fy
	
	print("valor de X:",x, "valor de Y:",y, "valor de Z:",z, "valor de depth:",depth)

	data_to_send = Float64MultiArray()  # the data to be sent
	pos=[0.0,0.0,0.0,0.0]		
	pos[0]=x
	pos[1]=y
	pos[2]=z
	pos[3]=depth
	data_to_send.data = pos

	pub_talker.publish(data_to_send)
	print "adios 4"


def convert_color_image(ros_image):

	#print "adios 5"
	global a,b
	bridge = CvBridge()
	# Use cv_bridge() to convert the ROS image to OpenCV format
	if a==0 and b == 0:
		try:
		#Convert the depth image using the default passthrough encoding
        		color_image = bridge.imgmsg_to_cv2(ros_image,"rgb8")
    		except CvBridgeError, e:
 	      	  print e

		cv2.circle(color_image, (v,u),10,(0,0,0),3)
     		cv2.circle(color_image, (322,250),10,(250,250,250),3)
		cv2.circle(color_image, (320,240),10,(250,0,0),3)
		cv2.imshow("Image window_1", color_image)
		cv2.waitKey(5)	
		a=1
		b=1
		#print "adios 6"

	elif a==1 :
			# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
		#Convert the depth image using the default passthrough encoding
        		color_image = bridge.imgmsg_to_cv2(ros_image,"rgb8")
    		except CvBridgeError, e:
 		        print e

     		cv2.circle(color_image, (322,250),10,(250,250,250),3)
		cv2.circle(color_image, (320,240),10,(250,0,0),3)

     		cv2.imshow("Image window_2", color_image)
		cv2.waitKey(1)	


	#mostrar por pantalla el rgb del punto seleccionado y cel cetro optico
        #color_1 = color_image[322,250]
        #color_2 = color_image[u,v]
        #print ("R_centro opt:",color_1[2],"G_centro opt:",color_1[1],"B_centro opt:",color_1[0])
        #print ("R_pto:",color_2[2],"G_pto:",color_2[1],"B_pto:",color_2[0])
   	


if __name__ == '__main__':
	
	try:
		pixel2depth()
	except rospy.ROSInterruptException:
		pass

	#este programa te da las coordenadas de un pixel de la imagen en 3D
	#utilizando la calibracion que proporciona la propia camara en el topic
	# rostopic echo /camera/color/camera_info



















