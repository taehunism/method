#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import time
class PurePursuitDriver:

    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance
        self.steering_angle = 0

    def find_lookahead_idx(self, pose_x, pose_y, ref):
        start = time.time()

        #find the lookahead index directly in front of the car
        self.lookahead_distance = 1.5

        distances = np.sqrt((ref[:, 0]) ** 2 + (ref[:, 1]) ** 2)
        #distances = np.sqrt((ref[:, 0] - pose_x) ** 2 + (ref[:, 1] - pose_y) ** 2)
        #min_idx = np.argmin(distances)       ##find index of minimum value
        min_idx = np.argmin(distances)       ##too slow function fucking 
        lookahead_idx = min_idx
        
        while True:
            lookahead_idx += 1
            
            if lookahead_idx >= len(ref):
                lookahead_idx = 0

            dist_to_lookahead = np.hypot(ref[lookahead_idx, 0], ref[lookahead_idx, 1])
                     
            if dist_to_lookahead >= self.lookahead_distance:   ##if waypoint is farther than the lookahead distance
                break

        return lookahead_idx

    def pure_pursuit_control(self, pose_x, pose_y, pose_theta, ref):
        # Find the lookahead point on the reference trajectory
        self.lookahead_distance = 1.3 # default l_h_d

        lookahead_idx = self.find_lookahead_idx(pose_x, pose_y, ref)
        lookahead_point = ref[lookahead_idx]  

        target_angle = math.atan2(lookahead_point[1], lookahead_point[0])

        point_dist = np.hypot(lookahead_point[1],lookahead_point[0])
        
        alpha = target_angle - pose_theta      ##Look ahead heading(theta)

        # Ensure the angle is within the range of [-pi, pi]
        
        while alpha > np.pi:
            alpha -=  2 * np.pi
        while alpha < -np.pi:
            alpha +=  2 * np.pi
        
        #print('Look ahead heading(theta)2: ',alpha)
        # Compute the steering angle

        self.steering_angle = math.atan2(2.0 * math.sin(target_angle), self.lookahead_distance)
        # self.steering_angle = math.atan2(2.0 * math.sin(target_angle), self.lookahead_distance)
        speed = 0.5

        # 필요에 따라 속도를 다르게 조절할 수 있습니다.
        # if abs(self.steering_angle) > 0.8:
        #     self.lookahead_distance = 1.5
        #     speed = 3.0

        # if abs(self.steering_angle) < 0.2:
        #     self.steering_angle = 0.0
        # if (lookahead_idx > 175 and lookahead_idx < 245):
        #     speed = 10.0
        # if (lookahead_idx > 360 and lookahead_idx < 50):
        #     speed = 10.0
        #if (lookahead_idx > 180 and lookahead_idx < 330):
        #    speed = 4.0
        #    self.lookahead_distance = 2.0
        #    if (lookahead_idx>195 and lookahead_idx < 200):
        #        speed = 3.0


        if (abs(self.steering_angle) < 0.2):
            self.steering_angle = 0.5 * self.steering_angle

        print('lookahead index', lookahead_idx)
        print('steering angle: ',self.steering_angle)
        print('------------------------------------')
  
        return speed, self.steering_angle, lookahead_idx



