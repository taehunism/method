#!/usr/bin/env python3
# -*- coding : utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class GapFollow:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

    def scan_callback(self, scan):
        if len(scan.ranges) == 0:
            rospy.loginfo("No LiDAR Input")
            return

        # 스캔 범위 설정
        left_start, left_end = 660, 900
        right_start, right_end = 180, 420
        left_values = []
        right_values = []
        # 거리 데이터 수집
        left_values = [scan.ranges[i] for i in range(left_start, left_end) if scan.ranges[i] != 0 and not np.isinf(scan.ranges[i])]
        right_values = [scan.ranges[i] for i in range(right_start, right_end) if scan.ranges[i] != 0 and not np.isinf(scan.ranges[i])]

        # 중간값 계산
        left_distance = np.median(left_values) if left_values else 0.0
        right_distance = np.median(right_values) if right_values else 0.0
        
        # 측정된 거리를 이용한 운전 각도 계산
        side = left_distance - right_distance
        print(f"side : {side}")
        if side > 0 :
            steer = side * 0.11
        elif side < 0 :
            steer = side * 0.11
        else :
            steer = 0.0
        #steer = np.clip(steer, -0.34, 0.34)

        # 운전 명령 생성
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 2.0
        drive_msg.drive.steering_angle = steer

        # 운전 명령 발행
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        rospy.init_node("gap_follower")
        print("Gap follow method Initialized")
        gap_follow = GapFollow()
        rospy.spin()  # 루프를 통해 콜백 호출
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated by ROS")
