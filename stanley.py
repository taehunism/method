#!/usr/bin/env python3
# -*- coding : utf-8 -*-

#define import library
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#import module
from path_polyfit import Path_Polyfit
path = Path_Polyfit()
# defined drive message
drive_msg = AckermannDriveStamped()
SPEED = 2.0
#class 
class StanleyNode():
    def __init__(self):
        self.drive_pub = rospy.Publisher("drive", AckermannDriveStamped, queue_size = 10)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scanCallback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.poseCallback)
        self.global_ref = np.genfromtxt('/home/catkin_ws/src/f1tenth_simulator/node/CSV FILE NAME', delimiter=',', skip_header = 1, usecols=(0,1))
        
    def scanCallback(self, scan):
        global drive_msg
        if len(scan.ranges) == 0:
            rospy.loginfo("No LiDAR Input")
            return
        # lidar range 
        left_start, left_end = 660, 900
        right_start, right_end = 180, 420
        front_start, front_end = 420, 660
        # lidar data list
        left_values = []
        right_values = []
        front_values = []
        # lidar filtering
        left_values = [scan.ranges[i] for i in range(left_start, left_end) if scan.ranges[i] != 0 and not np.isinf(scan.ranges[i])]
        right_values = [scan.ranges[i] for i in range(right_start, right_end) if scan.ranges[i] != 0 and not np.isinf(scan.ranges[i])]
        front_values = [scan.ranges[i] for i in range(front_start, front_end) if scan.ranges[i] != 0 and not np.isinf(scan.ranges[i])]
        # filter development
        if left_values:
            left_avg = sum(left_values) / len(left_values)
        
        if front_values:
            front_avg = sum(front_values) / len(front_values)
        
        if right_values:
            right_avg = sum(right_values) / len(right_values)
        #----------------------------------------------------------
        side = left_avg - right_avg
        if front_avg < 3.3:
            drive_msg.drive.speed = 0.5
            drive_msg.drive.steering_angle = side
            self.drive_pub.publish(drive_msg)
        
    def poseCallback(self, data):
        global path, drive_msg, SPEED
        cur_vel = data.twist.twist.linear.x
        poses = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
        pose_x = poses[0] #Cartesian coordinates of the vehicle in the global frame
        pose_y = poses[1] #Cartesian coordinates of the vehicle in the global frame
        
        # robot angle Finder
        quat = data.pose.pose.orientation
        quat_x = quat.x
        quat_y = quat.y
        quat_z = quat.z
        quat_w = quat.w   #vehicle's orientation in the global frame
        (roll, pitch, yaw) = euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])
        print('heading : ', yaw)

        # local coordinate transform
        local_ref = copy.deepcopy(self.global_ref)
        local_ref[:,0] = math.cos(yaw)*(self.global_ref[:,0]- pose_x) + math.sin(yaw)*(self.global_ref[:,1] - pose_y)
        local_ref[:,1] = -math.sin(yaw)*(self.global_ref[:,0]- pose_x) + math.cos(yaw)*(self.global_ref[:,1] - pose_y)

        # path generate
        path(local_ref[:,0], local_ref[:,1], yaw)
        lookahead_idx = path.search_points()
        goal = path.path_generate(lookahead_idx)
        steer = path.stanley_controller(goal, cur_vel)

        #publish
        drive_msg.drive.speed = SPEED * (1-steer)
        drive_msg.drive.steering_angle = steer

        self.drive_pub.publish(drive_msg)
    
if __name__ == "__main__":
    try:
        rospy.init_node("stanley_node")
        rate = rospy.Rate(10)
        stanley = StanleyNode()
        rospy.spin()
        rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("failed")