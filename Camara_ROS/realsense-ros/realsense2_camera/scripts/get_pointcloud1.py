#!/usr/bin/env python

import sys
import time
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Imu as msg_Imu
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import inspect
import ctypes
import struct
import tf

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

bla=0

def pc2_to_xyzrgb(point):
    global bla
	# Thanks to Panos for his code used in this function.
    #print point
    x, y, z = point[:3]
    rgb = point[3]

    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f', rgb)
    i = struct.unpack('>l', s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
	
    #print ('x_coordinate:',x,'y_coordinate:', y,'z_coordinate:',z, r, g, b)
    bla=bla+1
    #print ('loop_number:',bla)
   	
    return x, y, z, r, g, b


class CWaitForMessage:
    global bla  

    def __init__(self, params={}):
        self.result = None

        self.break_timeout = False
        self.timeout = params.get('timeout_secs', -1)
        self.seq = params.get('seq', -1)
        self.time = params.get('time', None)
        self.node_name = params.get('node_name', 'rs2_listener')
        self.bridge = CvBridge()
        self.listener = None

        self.themes = {
                       'pointscloud': {'topic': '/camera/depth/color/points', 'callback': self.pointscloudCallback, 'msg_type': msg_PointCloud2},
                       }

        self.func_data = dict()

    def pointscloudCallback(self, theme_name):
        def _pointscloudCallback(data):
            self.prev_time = time.time()
            #print 'Got pointcloud: %d, %d' % (data.width, data.height)
	    print ('Got pointcloud_1:',data.width)
	    print ('Got pointcloud_2:',data.height)

            self.func_data[theme_name].setdefault('frame_counter', 0)
            self.func_data[theme_name].setdefault('avg', [])
            self.func_data[theme_name].setdefault('size', [])
            self.func_data[theme_name].setdefault('width', [])
            self.func_data[theme_name].setdefault('height', [])
            # until parsing pointcloud is done in real time, I'll use only the first frame.
            self.func_data[theme_name]['frame_counter'] += 1

            if self.func_data[theme_name]['frame_counter'] == 1:
                # Known issue - 1st pointcloud published has invalid texture. Skip 1st frame.
                return

            try:
                points = np.array([pc2_to_xyzrgb(pp) for pp in pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb"))])
		#points = np.array([pc2_to_xyzrgb(pp) for pp in pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb")) if pp[0] > 0])
		
		asd=int(input("final?_1:"))
	
		num_puntos=100
		coord_x_1=np.zeros([num_puntos+1,],dtype=float)
		coord_y_1=np.zeros([num_puntos+1,],dtype=float)
		coord_z_1=np.zeros([num_puntos+1,],dtype=float)

		coord_x_2=np.zeros([num_puntos+1,],dtype=float)
		coord_y_2=np.zeros([num_puntos+1,],dtype=float)
		coord_z_2=np.zeros([num_puntos+1,],dtype=float)

		coord_x_3=np.zeros([num_puntos+1,],dtype=float)
		coord_y_3=np.zeros([num_puntos+1,],dtype=float)
		coord_z_3=np.zeros([num_puntos+1,],dtype=float)

		coord_x_4=np.zeros([num_puntos+1,],dtype=float)
		coord_y_4=np.zeros([num_puntos+1,],dtype=float)
		coord_z_4=np.zeros([num_puntos+1,],dtype=float)

		coord_x_5=np.zeros([num_puntos+1,],dtype=float)
		coord_y_5=np.zeros([num_puntos+1,],dtype=float)
		coord_z_5=np.zeros([num_puntos+1,],dtype=float)

		coord_x_6=np.zeros([num_puntos+1,],dtype=float)
		coord_y_6=np.zeros([num_puntos+1,],dtype=float)
		coord_z_6=np.zeros([num_puntos+1,],dtype=float)

		coord_x_7=np.zeros([num_puntos+1,],dtype=float)
		coord_y_7=np.zeros([num_puntos+1,],dtype=float)
		coord_z_7=np.zeros([num_puntos+1,],dtype=float)

		coord_x_8=np.zeros([num_puntos+1,],dtype=float)
		coord_y_8=np.zeros([num_puntos+1,],dtype=float)
		coord_z_8=np.zeros([num_puntos+1,],dtype=float)

		coord_x_9=np.zeros([num_puntos+1,],dtype=float)
		coord_y_9=np.zeros([num_puntos+1,],dtype=float)
		coord_z_9=np.zeros([num_puntos+1,],dtype=float)

		coord_x_10=np.zeros([num_puntos+1,],dtype=float)
		coord_y_10=np.zeros([num_puntos+1,],dtype=float)
		coord_z_10=np.zeros([num_puntos+1,],dtype=float)

		coord_x_11=np.zeros([num_puntos+1,],dtype=float)
		coord_y_11=np.zeros([num_puntos+1,],dtype=float)
		coord_z_11=np.zeros([num_puntos+1,],dtype=float)

		u=0
		print ("numero de puntos de la nuve",bla)
		while asd==0:
			for u in range(0,1200):
				
				#print (u,': esto es x:',points[u,0]*1000,'esto es y:',points[u,1]*1000,'esto es z:',points[u,2]*1000)
				if u<100:
					print "hola1"
					coord_x_1[u]=points[u*100,0]*1000
					coord_y_1[u]=points[u*100,1]*1000
					coord_z_1[u]=points[u*100,2]*1000
				elif u<200:
					print "hola2"
					coord_x_2[u-100]=points[u*100,0]*1000
					coord_y_2[u-100]=points[u*100,1]*1000
					coord_z_2[u-100]=points[u*100,2]*1000
				elif u<300:
					print "hola3"
					coord_x_3[u-200]=points[u*100,0]*1000
					coord_y_3[u-200]=points[u*100,1]*1000
					coord_z_3[u-200]=points[u*100,2]*1000
				elif u<400:
					print "hola4"
					coord_x_4[u-300]=points[u*100,0]*1000
					coord_y_4[u-300]=points[u*100,1]*1000
					coord_z_4[u-300]=points[u*100,2]*1000
				elif u<500:
					print "hola5"
					coord_x_5[u-400]=points[u*100,0]*1000
					coord_y_5[u-400]=points[u*100,1]*1000
					coord_z_5[u-400]=points[u*100,2]*1000
				elif u<600:
					print "hola6"
					coord_x_6[u-500]=points[u*100,0]*1000
					coord_y_6[u-500]=points[u*100,1]*1000
					coord_z_6[u-500]=points[u*100,2]*1000
				elif u<700:
					print "hola7"
					coord_x_7[u-600]=points[u*100,0]*1000
					coord_y_7[u-600]=points[u*100,1]*1000
					coord_z_7[u-600]=points[u*100,2]*1000
				elif u<800:
					print "hola8"
					coord_x_8[u-700]=points[u*100,0]*1000
					coord_y_8[u-700]=points[u*100,1]*1000
					coord_z_8[u-700]=points[u*100,2]*1000
				elif u<900:
					print "hola9"
					coord_x_9[u-800]=points[u*100,0]*1000
					coord_y_9[u-800]=points[u*100,1]*1000
					coord_z_9[u-800]=points[u*100,2]*1000
				elif u<1000:
					print "hola10"
					coord_x_10[u-900]=points[u*100,0]*1000
					coord_y_10[u-900]=points[u*100,1]*1000
					coord_z_10[u-900]=points[u*100,2]*1000
				elif u*100<bla and u-1000<100:
					print "hola11"
					coord_x_11[u-1000]=points[u*100,0]*1000
					coord_y_11[u-1000]=points[u*100,1]*1000
					coord_z_11[u-1000]=points[u*100,2]*1000
				elif u-1000<100:
					coord_x_11[u-1000]=0.0
					coord_y_11[u-1000]=0.0
					coord_z_11[u-1000]=0.0
			
			
			fig = plt.figure()
			ax = fig.add_subplot(111, projection='3d')
			ax.scatter(coord_x_1, coord_y_1, coord_z_1, c='b', marker='o')
			ax.scatter(coord_x_2, coord_y_2, coord_z_2, c='g', marker='o')
			ax.scatter(coord_x_3, coord_y_3, coord_z_3, c='r', marker='o')
			ax.scatter(coord_x_4, coord_y_4, coord_z_4, c='b', marker='v')
			ax.scatter(coord_x_5, coord_y_5, coord_z_5, c='g', marker='v')
			ax.scatter(coord_x_6, coord_y_6, coord_z_6, c='r', marker='v')
			ax.scatter(coord_x_7, coord_y_7, coord_z_7, c='b', marker='h')
			ax.scatter(coord_x_8, coord_y_8, coord_z_8, c='g', marker='h')
			ax.scatter(coord_x_9, coord_y_9, coord_z_9, c='r', marker='h')
			ax.scatter(coord_x_10, coord_y_10, coord_z_10, c='b', marker='x')
			ax.scatter(coord_x_11, coord_y_11, coord_z_11, c='g', marker='x')

			ax.set_xlabel('X Label')
			ax.set_ylabel('Y Label')
			ax.set_zlabel('Z Label')
			plt.show()
			
			asd=int(input("final?_2:"))

            except Exception as e:
                print(e)
                return
            self.func_data[theme_name]['avg'].append(points.mean(0))
            self.func_data[theme_name]['size'].append(len(points))
            self.func_data[theme_name]['width'].append(data.width)
            self.func_data[theme_name]['height'].append(data.height)
        return _pointscloudCallback

    def wait_for_message(self, params):
	print 'hola2'
        topic = params['topic']
        print 'connect to ROS with name: %s' % self.node_name
        rospy.init_node(self.node_name, anonymous=True)

        rospy.loginfo('Subscribing on topic: %s' % topic)
        self.sub = rospy.Subscriber(topic, msg_Image, self.callback)

        self.prev_time = time.time()
        break_timeout = False
        while not any([rospy.core.is_shutdown(), break_timeout, self.result]):
            rospy.rostime.wallsleep(0.5)
            if self.timeout > 0 and time.time() - self.prev_time > self.timeout:
                break_timeout = True
                self.sub.unregister()

        return self.result


    def wait_for_messages(self, themes):
	print 'hola3'
        # tests_params = {<name>: {'callback', 'topic', 'msg_type', 'internal_params'}}
        self.func_data = dict([[theme_name, {}] for theme_name in themes])

        print 'connect to ROS with name: %s' % self.node_name
        rospy.init_node(self.node_name, anonymous=True)
        for theme_name in themes:
            theme = self.themes[theme_name]
            rospy.loginfo('Subscribing %s on topic: %s' % (theme_name, theme['topic']))
            self.func_data[theme_name]['sub'] = rospy.Subscriber(theme['topic'], theme['msg_type'], theme['callback'](theme_name))

        self.prev_time = time.time()
        break_timeout = False
        while not any([rospy.core.is_shutdown(), break_timeout]):
            rospy.rostime.wallsleep(0.5)
            if self.timeout > 0 and time.time() - self.prev_time > self.timeout:
                break_timeout = True
                self.unregister_all(self.func_data)

        return self.func_data





def main():
    if len(sys.argv) < 2 or '--help' in sys.argv or '/?' in sys.argv:
        print 'USAGE:'
        print '------'
        print 'rs2_listener.py <topic | theme> [Options]'
        print 'example: rs2_listener.py /camera/color/image_raw --time 1532423022.044515610 --timeout 3'
        print 'example: rs2_listener.py pointscloud'
        print ''
        print 'Application subscribes on <topic>, wait for the first message matching [Options].'
        print 'When found, prints the timestamp.'
        print
        print '[Options:]'
        print '-s <sequential number>'
        print '--time <secs.nsecs>'
        print '--timeout <secs>'
        exit(-1)

    wanted_topic = sys.argv[1]

    print('hola1', wanted_topic)	

    msg_params = {}
    for idx in range(2, len(sys.argv)):
        if sys.argv[idx] == '-s':
            msg_params['seq'] = int(sys.argv[idx + 1])
        if sys.argv[idx] == '--time':
            msg_params['time'] = dict(zip(['secs', 'nsecs'], [int(part) for part in sys.argv[idx + 1].split('.')]))
        if sys.argv[idx] == '--timeout':
            msg_params['timeout_secs'] = int(sys.argv[idx + 1])

    msg_retriever = CWaitForMessage(msg_params)
    if '/' in wanted_topic:
        msg_params = {'topic': wanted_topic}
        res = msg_retriever.wait_for_message(msg_params)
        rospy.loginfo('Got message: %s' % res.header)
    else:
        themes = [wanted_topic]
        res = msg_retriever.wait_for_messages(themes)
        print res


if __name__ == '__main__':
    main()

