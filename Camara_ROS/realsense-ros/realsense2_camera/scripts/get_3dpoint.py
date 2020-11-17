#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import numpy 
import cv2
import math

#topic in which you send the coordinates 3D of the pixel
pub_talker = rospy.Publisher('chatter', Float64MultiArray, queue_size=1)
u=0
v=0
a=0
b=1

def pixel2depth():
	
	#we initialize the node and we subscribe to get the color and depth image
	
	rospy.init_node('pixel2depth',anonymous=True)
	
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, convert_depth_image, queue_size=1)
	rospy.Subscriber("/camera/color/image_raw", Image, convert_color_image, queue_size=1)
	
	rospy.spin()	

def convert_depth_image( ros_image):

	global u,v,a,b
	
	bridge = CvBridge()
     	# Use cv_bridge() to convert the ROS image to OpenCV format
     	try:
     	#Convert the depth image using the default passthrough encoding
        	depth_image = bridge.imgmsg_to_cv2(ros_image, "passthrough")

     	except CvBridgeError, e:
 	          print e
     
	#ask for the pixel that the user want to center
     	u=int(input("valor de fila:"))	
     	v=int(input("valor de columna:"))
	b=0
	a=0
	c=0
	d=0


     	#Convert the depth image to a Numpy array
     	depth_array = numpy.array(depth_image, dtype=numpy.float32)

	#obtain the depth from the pixel (we make the average of the surroundings pixels)
	arrows=5
	colums=arrows
	arrow=u-(arrows-1)/2
	colum=v-(colums-1)/2
	depth_media=numpy.zeros([arrows,colums],dtype=float)
	
	#c y d van a ir desde 0 hasta 2
	for c in range(0,arrows):
		for d in range(0,colums):
     			depth_media[c,d]=depth_array[arrow][colum]
			colum=colum+1
		arrow=arrow+1
		colum=v-(colums-1)/2

	print("esto es depth_media",depth_media)

	camera_intrinsics_fx=613.0291137695312	
	camera_intrinsics_fy=613.4072265625

	camera_intrinsics_ppy=250.34303283691406
	camera_intrinsics_ppx=322.4624938964844

	#el codigo desde dist_1 hasta z_2 no hace falta ya que la depth ya es directamente la coordenada z
	#dist_1=u - camera_intrinsics_ppy
	#dist_2=v - camera_intrinsics_ppx
	#dist_3=math.sqrt(dist_1**2+dist_2**2)
	#tita=math.atan(dist_3/camera_intrinsics_fx)
	#z_2 = math.cos(tita)*depth

	#calculamos la z con el promedio de las z de los pixeles del alrededor mas el pixel objetivo
	
	divisor=0
	sum_depth=0.0
	z=0.0

	for c in range(0,arrows):
		for d in range(0,colums):
			if depth_media[c][d]!="NaN" and depth_media[c][d]!=0.0 :
     				sum_depth=sum_depth+depth_media[c][d]
				divisor=divisor+1
		
	z=sum_depth/divisor

	x = (v - camera_intrinsics_ppx)*z/camera_intrinsics_fx
	y = (u - camera_intrinsics_ppy)*z/camera_intrinsics_fy
	
	#print("valor de X:",x, "valor de Y:",y, "valor de Z:",z, "valor de depth:",z_2)
	print("valor de X:",x, "valor de Y:",y, "valor de Z:",z)

	
	#we send the coordinates through the topic chatter
	data_to_send = Float64MultiArray()  # the data to be sent
	pos=[0.0,0.0,0.0,0.0]		
	pos[0]=x
	pos[1]=y
	pos[2]=z
	#pos[3]=depth
	pos[3]=z

	data_to_send.data = pos

	pub_talker.publish(data_to_send)
	#print "adios 4"


def convert_color_image(ros_image):

	#We show firt the color imagen before the movement of the PTZ with two circles, One its the objetive to center and the other is center of the image, And we show the imagen after the movement with a circle in the center of the image
	global a,b
	bridge = CvBridge()
	# Use cv_bridge() to convert the ROS image to OpenCV format
	if a==0 and b == 0:
		try:
		#Convert the color image using the default passthrough encoding
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

	#this code ask for the pixel that the user want to center in the image, calculate the coordinates 3D and send this info throgh the chatter topic toward the node that calculate the angles.


