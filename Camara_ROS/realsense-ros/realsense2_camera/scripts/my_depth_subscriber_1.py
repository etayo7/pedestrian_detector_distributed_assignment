#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy
import cv2

def convert_depth_image( ros_image):
     bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
     try:
     #Convert the depth image using the default passthrough encoding
                depth_image = bridge.imgmsg_to_cv2(ros_image, "passthrough")

     except CvBridgeError, e:
 	          print e
     #Convert the depth image to a Numpy array
     depth_array = numpy.array(depth_image, dtype=numpy.float32)

     #rospy.loginfo(depth_array)
     #rospy.loginfo("depth esquina 1",depth_array[1][1],"depth esquina 2",depth_array[1][639],"depth esquina 3",depth_array[260][639],"depth esquina 4",depth_array[260][1])
     print("depth esquina 1",depth_array[220][20],"depth esquina 2",depth_array[20][495],"depth esquina 3",depth_array[265][490],"depth esquina 4",depth_array[265][150])
   
def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, convert_depth_image, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	pixel2depth()


	#este programa te convierte una imagen de prorundidad de ros a openCv.
	#para ello nos subscribimos al topic "/camera/aligned_depth_to_color/image_raw" que es publica mensajes de tipo image
	#despues pasamos la imagen cv a una matriz de float depth_array[u][v]
	#el origen esta en la esquina superior izquierda 
	#eje u va de 0 a 479 desde origen hasta esquina inferior iz
	#eje v va de 0 a 639 desde origen hasta esquina sup derecha
