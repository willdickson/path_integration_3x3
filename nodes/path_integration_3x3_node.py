#!/usr/bin/env python
from __future__ import print_function
import os
import Queue
import roslib
import rospy
import numpy
import std_msgs.msg
import cv2
import threading

from region_visualizer import RegionVisualizer
from tracking_region import TrackingRegion

from operator import attrgetter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from multi_tracker.msg import Trackedobject, Trackedobjectlist

from path_integration_3x3.msg import LedRegionData
from path_integration_3x3.msg import TrackingRegionData
from path_integration_3x3.msg import PathIntegration3x3Data


class PathIntegrationNode(object):

    def __init__(self,nodenum=1):

        self.objects_queue = Queue.Queue()

        rospy.init_node('path_integration_node')
        self.start_time = rospy.Time.now().to_time()

        # Parameter paths
        self.default_param_path = '/path_integration_3x3/defaults'
        self.region_param_path = '/path_integration_3x3/regions'

        # Read default and region parameters
        self.default_param = self.get_default_param()
        self.region_data_list = self.get_regions_from_server()
        self.region_visualizer = RegionVisualizer(self.region_data_list)
        self.tracking_region_list = [TrackingRegion(i,p) for i,p in enumerate(self.region_data_list)]

        # Subscribe to tracked objects topic
        tracked_objects_topic = '/multi_tracker/{}/tracked_objects'.format(nodenum)
        self.tracked_objects_sub = rospy.Subscriber(
                tracked_objects_topic, 
                Trackedobjectlist, 
                self.tracked_objects_callback
                )

        # Subscribe to camera images
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_sub = rospy.Subscriber('/camera/image_mono', Image, self.image_callback)
        self.data_pub = rospy.Publisher('path_integration_3x3_data', PathIntegration3x3Data, queue_size=10) 

    def get_default_param(self):
        default_param = None
        if rospy.has_param(self.default_param_path):
            default_param = rospy.get_param(self.default_param_path)
        return default_param

    def get_regions_from_server(self):
        # Load from parameter server
        if not rospy.has_param(self.region_param_path):
            rospy.logerr('region parameters not found on server!!!')
        region_data_list = convert_region_data_list(rospy.get_param(self.region_param_path))
        # Set to defaults in missing
        if self.default_param is not None:
            region_data_list_tmp = [dict(self.default_param) for d in region_data_list]
            for region_data_tmp, region_data in zip(region_data_list_tmp, region_data_list):
                region_data_tmp.update(region_data)
            region_data_list = region_data_list_tmp
            #for region_data in region_data_list:
            #    print(region_data)
        return region_data_list

    def image_callback(self,ros_img): 
        cv_img = self.bridge.imgmsg_to_cv2(ros_img,desired_encoding='mono8')
        cv_img_bgr = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        with self.image_lock:
            self.latest_image = cv_img_bgr

    def tracked_objects_callback(self,data):
        number_of_objects = len(data.tracked_objects)
        if number_of_objects > 0: 
            self.objects_queue.put(data.tracked_objects)

    def run(self):

        while not rospy.is_shutdown():
            while (self.objects_queue.qsize() > 0):

                # Process tracked objects
                tracked_objects = self.objects_queue.get()
                self.process_regions(tracked_objects)

                # Visualize regions and objects
                with self.image_lock:
                    self.region_visualizer.update(self.latest_image, tracked_objects)

    def process_regions(self,tracked_objects):

        ros_time_now = rospy.Time.now()
        current_time = ros_time_now.to_time()
        elapsed_time = current_time - self.start_time 

        header = std_msgs.msg.Header()
        header.stamp = ros_time_now

        msg = PathIntegration3x3Data()
        msg.header = header

        # Create message data in region class instances???
        region_data_list = []

        for tracking_region in self.tracking_region_list: 
            region_data = tracking_region.update(elapsed_time, tracked_objects)
            region_data_list.append(region_data)

        for region_data in region_data_list:
            region_msg = TrackingRegionData()
            try:
                region_msg.object = region_data['object'] 
                region_msg.index = region_data['index']
            except KeyError:
                continue
            for led_data in region_data['leds']:
                led_msg = LedRegionData()
                led_msg.contains_object = led_data['contains_object']
                led_msg.activation_count = led_data['activation_count']
                led_msg.index = led_data['index']
                region_msg.led_region_data.append(led_msg)
            msg.tracking_region_data.append(region_msg)
        self.data_pub.publish(msg)
                

# Utility functions
# -------------------------------------------------------------------------------------------------------

def convert_region_data_list(region_data_list):
    return [convert_region_data(r) for r in region_data_list]

def convert_region_data(raw_region_data):
    region_data = dict(raw_region_data)
    region_data['leds'] = convert_led_list(region_data['leds'])
    return region_data

def convert_led_list(led_data_list):
    return [convert_led_data(d) for d in led_data_list]

def convert_led_data(led_data): 
    x = led_data['x']
    y = led_data['y']
    w = led_data['w']
    h = led_data['h']
    x0 = x - w/2
    y0 = y - h/2
    x1 = x + w/2
    y1 = y + h/2
    return {'x0':x0, 'y0':y0, 'x1':x1, 'y1':y1, 'chan': led_data['chan']}



# -------------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    node = PathIntegrationNode()
    node.run()
