#!/usr/bin/env python3
# -*- coding : utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
# turn_left : (+), turn_right : (-)

class WallFollow:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size = 10)

    def scan_callback(self, scan):
        if len(scan.ranges) == 0:
            rospy.loginfo("No LiDAR Input")
            return
    
        left_values = []
        front_values = []
        right_values = []

        for i in range(660, 900):
            if scan.ranges[i] != 0 and not math.isinf(scan.ranges[i]):
                left_values.append(scan.ranges[i])

        for i in range(500, 600):
            if scan.ranges[i] != 0 and not math.isinf(scan.ranges[i]):
                front_values.append(scan.ranges[i])

        for i in range(180, 420):
            if scan.ranges[i] != 0 and not math.isinf(scan.ranges[i]):
                right_values.append(scan.ranges[i])
                

        if left_values:
            left_avg = sum(left_values) / len(left_values)
        
        if front_values:
            front_avg = sum(front_values) / len(front_values)
        
        if right_values:
            right_avg = sum(right_values) / len(right_values)
            
        drive_msg = AckermannDriveStamped()
        print(f"len : {len(scan.ranges)}")
        print(f"right : {right_avg}")
        #print(f"left : {left_avg}")
        
        if right_avg > 1.4 :  #left pose
            steer= -right_avg*1.1
            speed = 2.0 * front_avg * 0.5      
                                                                  
        elif right_avg < 0.7:   #right pose
            steer = right_avg*1.1
            speed = 2.0 * front_avg * 0.5         
        else: 
            steer = 0.0               
            speed = 2.0 * front_avg * 0.7          
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer
        #print(f"steer_angle : {}")
        self.drive_pub.publish(drive_msg)
       

if __name__ == '__main__':
    try:
        rospy.init_node("wall_follower")
        print("wall follow method Initialized")
        wall_follow = WallFollow()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()
        
    except KeyboardInterrupt:
        rospy.loginfo("Node Start Failed")
