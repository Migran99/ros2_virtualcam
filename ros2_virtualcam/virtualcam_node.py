###================================
# Miguel Granero. 2022

#
#This package creates a virtual camera from the images published in a topic.
#

###================================

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyvirtualcam

import os

class VirtualCamNode(Node):
    def __init__(self, node_name='virtualcam'):
        super().__init__(node_name)

        # Args for setting IP/port of AI-deck. Default settings are for when
        # AI-deck is in AP mode.
        self.declare_parameter('cam_width',324)
        width = self.get_parameter('cam_width').value
        self.declare_parameter('cam_height',244)
        height = self.get_parameter('cam_height').value
        self.declare_parameter('cam_fps',20)
        fps = self.get_parameter('cam_fps').value
        self.declare_parameter('topic_name','/aideck/image')
        topic_name = self.get_parameter('topic_name').value

        self.declare_parameter('switch_rb',True)

        self.cvBdg = CvBridge()
        self.cam = pyvirtualcam.Camera(width=width, height=height, fps=fps, device='/dev/video3')

        self.get_logger().info('Using camera device #{}'.format(self.cam.device))

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.topicCB,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Listening to topic: {}'.format(topic_name))


    def topicCB(self, msg):
        self.get_logger().debug('Image received: "{}"'.format(msg.header.stamp))
        if self.get_parameter('switch_rb').value:
            img = cv2.cvtColor(self.cvBdg.imgmsg_to_cv2(msg,desired_encoding="8UC3"),cv2.COLOR_BGR2RGB)
        else:
            img = self.cvBdg.imgmsg_to_cv2(msg,desired_encoding="8UC3")

        self.cam.send(img)

   

def main(args=None):

    rclpy.init(args=args)
    virtualcam = VirtualCamNode('virtualcam')
    rclpy.spin(virtualcam)

    virtualcam.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()