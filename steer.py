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

dotk = [0 for i in range(1)]

class lidar_receiver:
    def __init__(self):
        self.stop_sub = rospy.Subscriber("/warning", Int32, self.callback)
        self.lineangle_sub = rospy.Subscriber("/lineangle", Float32, self.callbasck2)
        self.msg_sub = rospy.Subscriber("/tflag", String, self.callback3)
        self.lineangle2_sub = rospy.Subscriber("/lineangle2", Float32, self.callbasck4)
        self.lc_sub = rospy.Subscriber("/lc_marker", Float32, self.callback5)
        self.drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_1", AckermannDriveStamped,)
        self.cvbridge = Cvbridge()

    def callback(self, data):
        tmp_data = AckermannDriveStamped()
        if angle >= 0.3:
            speed = 1.1
        else:
            speed = 1.25
        
        if (data.data == 1) | (RG == 'R'):
            # vehicle stop
            tmp_data.drive.speed = 0.0
        else:
            # vehicle move
            tmp_data.drive.speed = speed

        if (AR_angle <= -0.1) | (AR_angle >= 0.1):
            tmp_data.drive.steering_angle = AR_angle

            if (AR_angle >= 0.1):
                dotk[0] = 1
            elif(AR_angle <= -0.1):
                dotk[0] = 0
        else:
            if dotk[0] == 1:
                tmp_data.drive.steering_angle = angle2
            else:
                tmp_data.drive.steering_angle = angle
        
        print(dotk[0])

        self.drive_pub.publish(tmp_data)

    def callback2(self, data):
        global angle
        angle = data.data
        #print(data.data)

    def callback3(self, data):
        global RG
        RG = data.data

    def callback4(self, data):
        global angle2
        angle2 = data.data

    def callback5(self, data):
        global AR_angle
        AR_angle = data.data
        print(AR_angle)

def run():
    rospy.init_node('steer_sub', anonymous = True)
    lidar_receiver_a = lidar_receiver()
    #raew = spin()

if __name__ == "__main__":
    run()