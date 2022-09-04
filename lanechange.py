#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Int32, String
from cv_bridge import Cvbridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from visualization_msgs.msg import Marker

class lidar_receiver:
    def __init__(self):
        # self.lidar_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.mk_sub = rospy.Subscriber("/zed/visualization_marker", Marker, self.callback4)
        self.lc_pub = rospy.Publisher("/lc_marker", Float32, queue_size = 5)
        self.cvbridge = Cvbridge()

    def callback4(self, data):
        AR = data.pose.position.z
        if (data.id == 0) & (1.25 < AR <= 2.0):
            AR_angle = 0.2
        elif (data.id == 1) & (1.0 < AR <=2.25):
            AR_angle = -0.3
        else: AR_angle = 0
        print(AR)
        self.lc_pub.publish(AR_angle)

def run():
    rospy.init_node("steer_sub", anonymous = True)
    lidar_receiver_a = lidar_receiver()
    rospy.spin()

if __name__ == "__main__":
    run()