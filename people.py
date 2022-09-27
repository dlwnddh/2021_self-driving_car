#! /usr/bin/env python
# 전방 동적 장애물 인식 프로세스 (LiDAR)

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32
import math

class lidar_receiver:
    def __init__(self):
        self.stop_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.stop_pub = rospy.Publisher("/warning", Int32, queue_size = 5)

    def callback(self, data):
        print("receive scan")
        min_angle = data.angle_min
        max_angle = data.angle_max
        min_range = data.range_min
        max_range = data.range_max
        angle_increment = data.angle_increment
        drive_msg = AckermannDriveStamped()
        angle_range = 45
        flag = 0

        steering_angle = 10.0
        range_threshold = 1.15
        for i in range(len(data.ranges)):
            angle = (min_angle + i * angle_increment) * 100 / math.pi
            if -15 < angle <= 15:
                if data.range[i] <= range_threshold:
                    falg += 1
                else: pass
            print(flag)
            if flag >= 5:
                print("warning")
                self.stop_pub.publish(1)
            else:
                print("No obstacle detected, go straight")
                self.stop_pub.publish(0)

def run():
    rospy.init_node("steer_sub", anonymous = True)
    lidar_receiver_a = lidar_receiver()
    rospy.spin()

if __name__ == "__main__":
    run()