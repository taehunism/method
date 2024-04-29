#!/usr/bin/env python3
# -*- coding : utf-8 -*-
#define import library
import math
import copy
import sys
import numpy as np
from numpy.polynomial import Polynomial
#class 
class Path_Polyfit():
    def __init__(self, local_x, local_y, yaw):
        self.x = local_x
        self.y = local_y
        self.yaw = yaw
        self.lookahead_distance = 1.0
        self.poly_coefficients = []

    def search_points(self):
        distances = np.hypot(self.x, self.y)
        min_idx = np.argmin(distances) # returned index number
        lookahead_idx = min_idx # index circle structure ex) a to z to a to z
        
        while True:
            lookahead_idx += 1
            if lookahead_idx >= len(self.local_x):
                lookahead_idx = 0

            dist_to_lookahead = np.hypot(self.x[lookahead_idx], self.y[lookahead_idx])
                     
            if dist_to_lookahead >= self.lookahead_distance:   #prevent over index
                break

        return lookahead_idx

    def path_generate(self, lookahead_idx):
        #pick up closest points 
        closest_points_x = self.x[lookahead_idx-2:lookahead_idx+2]
        closest_points_y = self.y[lookahead_idx-2:lookahead_idx+2]
        #polyfit
        self.poly_coefficients = Polynomial.fit(closest_points_x, closest_points_y, 3).convert().coef
        #poly1d
        polynomial = np.poly1d(self.poly_coefficients)
        #curvature
        curvature = abs(6 * self.poly_coefficients[2]) # 2-order derivate -> curvature feature
        #tuning parameter for lookahead distance
        if curvature >= 0.2:
            self.lookahead_distance = 0.5
        else:
            self.lookahead_distance = 1.0
        
        goal = polynomial(self.lookahead_distance)

        print('------------------------------------')
        print(f"lookahead index : {lookahead_idx}")
        print(f"curvature : {curvature}")
        print('------------------------------------')

        return goal
    
    def stanley_controller(self, goal, cur_vel):
        w1 = 0.8 # heading term
        w2 = 0.7 # CTR term

        c0 = goal # lateral error
        c1 = self.poly_coefficients[1] # heading error
        # case 1. lower heading error
        steer = w1 * math.atan2(c1,1) + math.atan2(w2 * c0, cur_vel)
        
        # else:
        #     steer = w1*math.atan2(c1, cur_vel) + math.atan2(w2*c0, cur_vel)

        return steer