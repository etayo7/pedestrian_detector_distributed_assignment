#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy
import cv2


def convert_color_image(ros_image):
     bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
     try:
     #Convert the depth image using the default passthrough encoding
                color_image = bridge.imgmsg_to_cv2(ros_image,"rgb8")
     except CvBridgeError, e:
 	          print e

     (rows,cols,channels)= color_image.shape
     print("rows: ",rows,"cols: ",cols,channels,"color")

     v=int(input("valor de la columna:"))
     u=int(input("valor de la fila:"))

     #pintar el valor rgb origen en el centro de la imagen
     #color = color_image[x+240,y+320]
     #print ("R:",color[2],"G:",color[1],"B:",color[0])
     
     #pintar el valor rgb del pixel seleccionado
     color = color_image[u,v]
     print ("R:",color[2],"G:",color[1],"B:",color[0])
	

     #para dibujar un circulo en la imagen, en este caso se cogen las coordenadas alreves v y u
     #argumentos: imagen, centro , radio, color, relleno o grosor
     #cv2.circle(color_image, (320,240),50,(250,250,250),-1)
     cv2.circle(color_image, (v,u),10,(250,250,250),3)

     #pintas por pantalla la imagen
     cv2.imshow("Image window", color_image)	
     #con waitkey esperas hasta pintar la siguiente un timepo x si x=0 esperas infinito
     cv2.waitKey(0)



def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/camera/color/image_raw", Image, convert_color_image, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
	
	pixel2depth()


	#este programa te convierte una imagen de prorundidad de ros a openCv.
	#para ello nos subscribimos al topic "/camera/color/image_raw" que es publica mensajes de tipo image
	#despues pasamos la imagen cv a una matriz de float depth_array[u][v]
	#el origen esta en la esquina superior izquierda 
	#eje u va de 0 a 479 desde origen hasta esquina inferior iz
	#eje v va de 0 a 639 desde origen hasta esquina sup derecha
	

	#este codigo se subscribe al topic de imagen de color
 	#esta imagen se pasa a formato opencv
	#se pinta un circulo en la imagen con centro el pixel seleccionado
	
