#!/usr/bin/env python
from __future__ import print_function
import threading
import numpy as np
import rospy
import cv2
import std_msgs.msg
import yaml
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RegionVisualizer(object):

    def __init__(self,default_filename):
        self.node_name = 'region_visualizer'

        self.lock = threading.Lock()
        self.is_first = True 
        self.window_name = 'regions'



        self.region_color = (0,255,0)
        self.led_color = (0,0,255)

        self.region_num_pad = 0,5
        self.region_param_path = '/path_integration_3x3/regions'

        self.default_corner_file = default_filename
        self.region_data_list = self.get_regions_from_server()
        #print(self.region_data_list)

        rospy.init_node(self.node_name)
        self.input_image = rospy.get_param('{}/input_image'.format(self.node_name), '/camera/image_raw')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.input_image, Image, self.on_image_callback)


    def get_regions_from_server(self):
        if not rospy.has_param(self.region_param_path):
            rospy.logerr('region parameters not found on server!!!')
        region_data_list = rospy.get_param(self.region_param_path)
        return region_data_list


    def on_image_callback(self,ros_img):

        with self.lock:
            if self.is_first:
                cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)
                cv2.moveWindow(self.window_name, 100, 100)
                cv2.resizeWindow(self.window_name, 800,600)
                self.is_first = False 

        cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        cv_img_bgr = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)

        for num, region_data in enumerate(self.region_data_list):
            x0, y0 = region_data['x0'], region_data['y0']
            x1, y1 = region_data['x1'], region_data['y1']
            x_text = x0+self.region_num_pad[0]
            y_text = y0-self.region_num_pad[1]
            
            cv2.rectangle(cv_img_bgr, (x0,y0), (x1,y1), self.region_color, 1)
            cv2.putText(cv_img_bgr,'{}'.format(num), (x_text,y_text) , cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.region_color)

            if region_data.has_key('leds'):
                for led_data in region_data['leds']:
                    led_x = led_data['x']
                    led_y = led_data['y']
                    led_w = led_data['w']
                    led_h = led_data['h']
                    led_x0 = led_x - led_w/2
                    led_y0 = led_y - led_h/2
                    led_x1 = led_x + led_w/2
                    led_y1 = led_y + led_h/2
                    cv2.rectangle(cv_img_bgr, (led_x0,led_y0), (led_x1,led_y1), self.led_color, 1)

            
        cv2.imshow(self.window_name,cv_img_bgr)
        cv2.waitKey(1)


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

#----------------------------------------------------------------------------------------
if __name__ == '__main__':

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'region_coords.yaml'
    node = RegionVisualizer(filename)
    node.run()

