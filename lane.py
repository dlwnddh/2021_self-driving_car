#! /usr/bin/env python

from array import array
import rospy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from cv_bridge import Cvbridge
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

dot1_x = [0 for i in range(8)]
dot1_y = [0 for i in range(8)]
dot2_x = [0 for i in range(8)]
dot2_y = [0 for i in range(8)]

drive_msg = AckermannDriveStamped()

class lidar_receiver:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        self.stop_pub = rospy.Publisher("/check", Float32, queue_size = 5)
        self.lineangle_pub = rospy.Publisher("/lineangle", Float32, queue_size = 5)
        self.drive_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_1", AckermannDriveStamped,)
        self.cvbridge = Cvbridge()

    def callback(self, data):
        frame1 = self.cvbridge.imgmsg_to_cv2(data, "bgr8")

        val = 30
        array = np.full(frame1.shape, (val, val, val), dtype=np.uint8)
        frame = cv2.subtract(frame, array)
        pts1 = np.float32([[160, 300], [560, 300], [700, 400], [40, 400]])
        pts2 = np.float32([[0, 0], [640, 0], [640, 480], [0, 480]])
        for i in pts1:
            pass
            cv2.circle(frame, tuple(i), 5, (255, 255, 0), -1)
        M = cv2.getPerspectiveTransform(pts1, pts2)
        inv_M = cv2.getPerspectiveTransform(pts2, pts1)
        BEV = cv2.warpPerspective(frame, M, (640, 480))
        hsv = cv2.cvtColor(BEV, cv2.COLOR_BGR2HSV)
        lower_th = np.array([23, 100, 100])
        higher_th = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_th, higher_th)
        res = cv2.bitwise_and(BEV, BEV, mask = mask)
        edges = cv2.Canny(mask, 200, 400)

        #cv2.circle(frame, (320, 240), 20, (255, 0, 0), -1)
        #cv2.imshow("frame", frame)
        #cv2.imshow("hsv", hsv)
        cv2.imshow("mask", mask)
        #cv2.imshow("colored_frame", res)
        #cv2.imshow("BEV", BEV)
        #cv2.imshow("edges", edges)
        cv2.waitKey(1)

        x0 = 0
        w0 = 340
        h0 = 60
        y0 = 0

        i = 0

        for j in range(2):
            for i in range(8):
                if (i == 0) & (x0 == 680):
                    x0 = 0

                roi = mask[y0:y0 + h0, x0:x0+w0]
                contours, _ = cv2.findContours(roi, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                #ROI = cv2.drawContours(BEV, countours, -1, (0, 255, 0), 2)

                for contour in contours:
                    if cv2.contourArea(contour) < 700:
                        continue
                
                (x, y, w, h) = cv2.boundingRect(contour)
                cv2.rectangle(BEV, (x + w / 2 - 50 + x0, x + h / 2 - 20 + y0), (x + w / 2 + 50 + x0, x + h / 2 + 20 + y0), (0, 0, 255), 0)

                xx = x + w / 2
                yy = y + h / 2 + y0
                
                if x0 == 0:
                    dot1_x[i] = xx
                    dot1_y[i] = yy
                else:
                    dot2_x[i] = xx
                    dot2_y[i] = yy
            y0 += 50
            if i == 7:
                y0 = 0
            
            x0 += 320

        x_line1 = 0

        if dot2_x[7] > 161:
            x_line2 = dot2_x[7]
        else:
            x_line2 = 161
        
        cv2.line(BEV, (dot1_x[7], dot1_y[7]), (dot1_x[0], dot1_y[0]), (255, 0, 0), 3)
        cv2.line(BEV, (x_line2 + 320, dot2_y[7]), (dot2_x[0] + 320, dot2_y[0]), (255, 0, 0), 3)

        if (dot1_x[7] - dot1_x[0]) == 0:
            slope1 = 0
        else:
            slope1 = (dot1_y[7] - dot1_y[0]) / (dot1_x[7] - dot1_x[0])
        
        if (x_line2 - dot2_x[0]) == 0:
            slope2 = 0
        else:
            slope2 = (dot2_y[7] - dot2_y[0]) / (x_line2 - dot2_x[0])
        
        # print(dot1_x[7] - x_line1)
        # print("---------------------")
        # print(dot2_x)

        if (680 - dot2_x[0]) > 585:
            angle = 0.4
        else:
            if (680 - dot2_x[0]) > 500:
                angle = 0.07
            else:
                if (slope2 >= 7) & ((dot1_x[7] - x_line1) >= 120):
                    angle = -0.06
                else:
                    angle = 0
        
        cv2.imshow("BEV", BEV)
        self.lineangle_pub.publish(angle)

def run():
    rospy.init_node("steer_sub", anonymous = True)
    lidar_receiver_a = lidar_receiver()
    rospy.spin()

if __name__ == "__main__":
    run()