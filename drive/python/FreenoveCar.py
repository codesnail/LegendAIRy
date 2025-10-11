# -*- coding: utf-8 -*-
"""
Created on Mon Nov 27 19:23:50 2023

@author: ahmed
"""

import json
import numpy as np
from util import polar2cart, euclidean_distance, get_turn_angle, update_heading
from SkidSteerArdu import SkidSteerArdu
import traceback

class LrModel:
    def __init__(self, model: dict()):
        self.intercept = model['Intercept']
        self.slope = model['power_level']
        
class Controller:
    def __init__(self):
        self.prev_error = None
        self.sum_error = None
        f = open('freenove_calibration.txt', 'r')
        vel_model = json.load(f)
        f.close()
        
        f = open('freenove_calibration_turns.txt', 'r')
        turn_model = json.load(f)
        f.close()
        
        self.right_turn_model = LrModel(turn_model['turn_model_right'])
        self.left_turn_model = LrModel(turn_model['turn_model_left'])
        self.vel_model = LrModel(vel_model['vel_model'])
        self.turn_power = 110 # default power level for turning
        self.vel_power = 110 # default power level for velocity

    # MPC controller
    def get_turn_params(self, heading_error):
        '''
        This method returns the turn parameters to close the heading_error gap:
        Returns:
            A dict of: pwr_l (power for left motors), pwr_r (power for right motors), time (time for which to supply the powers)
        Input:
            heading_error - difference between car's heading and target angle'
        '''
        
        direction = 0
        
        if abs(heading_error) >= 0.0001:
            direction = heading_error/abs(heading_error)
        # Negative heading error means turn clockwise = forward spin on left wheel, reverse spin on right wheel
        # Positive heading error means turn anti clockwise = reverse of above spins
        
        print("heading error, direction: ", heading_error, direction)
        
        turn_rate = 0
        turn_time = 0
        
        if direction == -1: # clockwise/right turn model
            # turn rate is radians/sec
            turn_rate = self.right_turn_model.intercept + self.right_turn_model.slope * self.turn_power
            
        elif direction == 1: # anti clockwise/left turn model
            turn_rate = self.left_turn_model.intercept + self.left_turn_model.slope * self.turn_power
        
        turn_time = abs(heading_error / turn_rate)
                
        return {'pwr_l': -self.turn_power * direction, 'pwr_r': self.turn_power * direction, 'time': turn_time}

    def get_move_time(self, dist_error_cm):
        '''
        Given a distance indicated by a distance eror from current position
        to desired position, returns the time in secs to move the wheels at the 
        set power level indicated by self.vel_power. Call get_turn_params first
        to make sure the car is facing the desired direction.
        
        Input:
            dist_error - Difference between car's position and target position.
                         The car should be facing the same direction as the target
                         position.
            
        Returns:
            Time for which to apply self.vel_power power to the motors to travel
            the required distance
        '''
        dist_error = dist_error_cm / 100 
        vel = self.vel_model.intercept + self.vel_model.slope * self.vel_power
        dt = abs(dist_error / vel)
        return dt
        
        
class Sonar:
    def __init__(self):
        self.first_sense = True
        self.scan_points = []
        self.theta = 0 #sonar angle (radians)
        self.r = 0 #sonar distance
        self.angle = -1 #sonar angle in degrees to track movement
        self.prev_angle = -2 #sonar angle in degrees to track movement
        self.scan_complete = False # When a scan is completed to find obstacles
        
class WayPoints:
    def __init__(self):
        self.path = []
        self.wp_idx = 0
        self.wpx = 0
        self.wpy = 0
    
    def reset(self):
        self.path = []
        self.wp_idx = 0
        self.wpx = 0
        self.wpy = 0

class FreenoveCar(SkidSteerArdu):
    
    def __init__(self):
        super().__init__()
        self.x = 0.00 # cm
        self.y = 0.00 # cm
        self.prev_x = 0
        self.prev_y = 0
        self.heading = np.pi/2 # straight in front is 90 degrees
        self.axle_length = 0.1016 # meters
        self.prev_time = 0
        self.controller = Controller()
        self.prev_dist = 1000
        self.prev_way_point = None
        self.delta_x = 0
        self.delta_y = 0
        self.way_points = WayPoints()
        self.sonar = Sonar()
        self.planner = None
        self.yaw = 0
        self.gyro_z = 0
        
    def updatePosition(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading
        
    
    def getNextWayPoint(self):
        
        next_wp = []
        # Get the next way point that is at least a min distance away
        # from current location of robot
        for i in range(self.way_points.wp_idx, len(self.way_points.path)):
            
            self.way_points.wp_idx += 1
            
            if euclidean_distance(self.way_points.path[i], [self.x, self.y]) >= 15:
                next_wp = self.way_points.path[i:]
                break
            
        return next_wp
    
    def calculateErrors(self, way_point):
        err_x    = way_point[0] - self.x
        err_y    = way_point[1] - self.y
        err_dist = np.sqrt(err_y**2 + err_x**2) 
        
        #err_heading = self.heading - (np.pi/2 - np.arctan2(err_y, err_x))
        err_heading = get_turn_angle(self.heading, self.x, self.y, way_point[0], way_point[1])
            
        print("err_dist, err_heading: ", err_dist, err_heading)
        
        return err_dist, err_heading
      
    def move(self, way_points, discard_buffer=True):
        '''
        Moves the car from current x,y position to new position provided
        in way_points.
        
        Parameters
        ----------
        way_points : list
            The next list of x,y positions to move to. The first way point is 
            retrived out of the list and the car is moved to it.
            
        discard_buffer: boolean
            Indicates whether the read buffer should be kept around
            after the move or discarded. Set to True in case of synchronous
            read, False in case of async read.

        '''
        
        # The following logic should probably be in Arduino for
        # faster processing but keeping it here for now for quick 
        # experimentation
        
        # Get next way point
        # Determine error from current position to way point, these will be 2 
        # errors: err_distance and err_heading
        # Use these errors to determine how much power to apply to the left and 
        # right wheels and for how much time
        
        wp = way_points[0]
        errors = self.calculateErrors(wp)
        
        err_dist, err_heading = errors
        
        turn_params = self.controller.get_turn_params(err_heading)
        
        # TODO: Update the below cut offs to handle negative power
        pwr_l = turn_params['pwr_l']
        pwr_r = turn_params['pwr_r']
        dt    = turn_params['time']
            
        print("pwr_l, pwr_r, dt = ", pwr_l, pwr_r, dt)
        
        if (pwr_l != 0 and dt > 0.2):
            super().move(pwr_l, pwr_r, dt, discard_buffer)
            #_, _, _, gyro_z, yaw = super().read_event_move()
            #self.yaw = yaw
            #self.gyro_z = gyro_z
            self.heading = update_heading(self.heading, err_heading)
        
        move_time = self.controller.get_move_time(err_dist)
        if move_time > 0.5:
            print("err_dist, move_time: ", err_dist, move_time)
            super().move(self.controller.vel_power, self.controller.vel_power * (.87), move_time, discard_buffer)
            #_, _, _, _, _ = super().read_event_move()
            self.x = self.x + np.cos(self.heading) * err_dist
            self.y = self.y + np.sin(self.heading) * err_dist
        
        #print("x, y, heading, gyro_z, yaw: ", self.x, self.y, self.heading, self.yaw, self.gyro_z)
    
    def set_planner(self, planner):
        self.planner = planner
        self.planner.set_car = self
    
    def scan(self, stepSize=1, angle=0):
        if(self.sonar.first_sense):
            self.sonar.angle = 0
            res = super().sense(stepSize, angle)
            self.sonar.first_sense = False
        
        else:
            res = super().event_ultrasonic()
            
        #print("res = ", res)
        if res != None and res[0] != None:
            self.sonar.prev_angle = self.sonar.angle
            self.sonar.angle, self.sonar.r = res
            self.sonar.theta = np.deg2rad(self.sonar.angle)
            x, y = polar2cart(self.sonar.theta, self.sonar.r)
            
            # add all readings to a list
            self.sonar.scan_points.append([x,y])
        
        if self.sonar.angle == 180:
            super().stop()
            super().emptyBuffer()
            #self.sonar.first_sense = True
            #self.sonar.angle = 0
        
        return self.sonar.theta, self.sonar.r
    
    def handle_ultrasonic(self, str_event):
        parts = str_event.split(',')
        theta = int(parts[0].split(':')[1])
        r = float(parts[1].split(':')[1])
        
        self.sonar.prev_angle = self.sonar.angle
        self.sonar.angle, self.sonar.r = theta, r
        self.sonar.theta = np.deg2rad(self.sonar.angle)
        x, y = polar2cart(self.sonar.theta, self.sonar.r)
        
        # add all readings to a list
        self.sonar.scan_points.append([x,y])
        
        if self.sonar.angle == 180:
            super().stop()
            
    def drive(self, path):
        self.way_points.path = path
        self.way_points.wp_idx = 0 # reset index
        
        print("wp = ", self.way_points.path[:10])
        next_wp = self.getNextWayPoint()
        print("next_wp = ", next_wp[:10])
        
        end_of_path = False
        
        if(len(next_wp) == 0):
            self.stop()
            end_of_path = True;
            print("End of path")
            
        else:
            self.move(next_wp)
            
        if self.way_points.wp_idx == len(path) - 1:
            self.way_points.path = []
            end_of_path = True
        
        return end_of_path    

def test():
    try:
        car = FreenoveCar()
        
        while(True):
            value = input("Enter next way points as x y (or q to quit): ")
            
            if(value == 'q'):
                break
            
            l = value.split()
            print(l)
            if len(l) != 2:
                print("Invalid input")
                continue
            
            way_points = [[float(x) for x in l]]
            print(way_points)
            car.move(way_points)
    except:
        traceback.print_exc()
    
    car.stop()

def test1():
    car = FreenoveCar()
    
    #path = [[0, 40], [40, 40], [40, 0], [0, 0], [0, 10]]
    path = [[0, 0], [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], [0, 8], [0, 9], [0, 10], [0, 11], [0, 12], [0, 13], [0, 14], [0, 15], [0, 16], [0, 17], [0, 18], [0, 19],  [0, 20], [0, 21], [0, 22], [0, 23], [0, 24], [0, 25], [0, 26], [0, 27], [0, 28], [0, 29], [0, 30], [0, 31], [0, 32], [1, 32], [2, 32], [3, 32], [4, 32], [5, 32], [6, 32], [7, 32], [8, 32]]
    try:
        end_of_path = False
        
        while not end_of_path:
            end_of_path = car.drive(path)
            
    except:
        traceback.print_exc()
        
    car.stop()
    
def test2():
    car = FreenoveCar()
    
    way_points = [[0, 40], [40, 40], [40, 0], [0, 0], [0, 10]]
    
    for i in range(2):
        for res in car.scan_180():
            print(res)
        
        #car.updatePosition(0.00, 0.00, 1.57) # x, y, heading
        
        for res in car.move_for_time(way_points, 1):
            print(res)
    
    car.stop()
    
def test3():
    wps = [[1,30], [10,60], [20,90], [30,120], [40,150]]
    car = FreenoveCar()
    check_dist = car.reached(wps[0])
    print(check_dist)
    
test()