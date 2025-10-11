# -*- coding: utf-8 -*-
"""
Created on Mon Nov 27 19:23:50 2023

@author: ahmed
"""

import serial
import json
import time
import numpy as np
from util import polar2cart, euclidean_distance, rotate, translate
import traceback

class SkidSteerArdu:
    
    def __init__(self, port='COM7'):
        # The bluetooth port on my computer is COM7, you will need
        # to update this to the port you are using to communicate
        # with your board
        self.ser = serial.Serial(port = port, baudrate=9600,
                                   bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE)

        self.x = 0.00 # cm
        self.y = 0.00 # cm
        self.axle_length = 0.1016 # meters
        self.prev_time = 0
    
    def __del__(self):
        self.ser.close()

    def recvFromArduino(self):
        startMarker = '<'
        endMarker = '>'
        
        res = ""
        x = b'z' # any value that is not an end- or startMarker
        byteCount = -1 # to allow for the fact that the last increment will be one too many
        
        if self.ser.in_waiting <= 0:
            return None
        
        #while self.ser.in_waiting <= 0: # and byteCount < 50: # wait until there is something to read
        #    pass
        
        #if x.decode() == 'd':
        #    return "deserialize error"
        
        # wait for the start character
        while  x.decode() != startMarker and x.decode() != '': 
            #print("waiting... ", x)
            x = self.ser.read()
        
        # save data until the end marker is found
        while x.decode() != endMarker and x.decode() != '':
            if x.decode() != startMarker:
              res = res + x.decode()
              byteCount += 1
            x = self.ser.read()
        
        return res.strip("<, >")
    
    def extractValue(self, pair):
        return pair.split(':')[1]
      
    def move(self, pwr_l, pwr_r, dt=0, discard_buffer=True):
        '''
        Parameters
        ----------
        dt - Length of time to provide the power. 0 for no time boundary predefined and will be controlled outside.

        '''
        
        # The following logic will ideally be in Arduino but
        # keeping it here for now for quick experimentation
        
                
        command = {
            "mode": 3,
            "pos": [0, 0, 1.57],
            #"wp": way_points, # [[x1, y1], [x2, y2], [x3, y3]]
            "pwr": [pwr_l, pwr_r]
        }
        
        cmdStr = '<' + json.dumps(command) + '>'
        
        i = self.ser.write(cmdStr.encode()) 
        #start_time = time.time()
        print("move command sent...")
        
        if dt > 0: # If a time length is defined, stop after that
            time.sleep(dt)
           # time_elapsed = 0
            #while time_elapsed <= dt:
            #    i = self.ser.write(cmdStr.encode())
            #    time_elapsed = time.time() - start_time
            print("stopping after {} secs.".format(dt))
            self.stop()
            
            if discard_buffer: # Set this to false when we read all inout from the buffer asynchronously
            
                self.emptyBuffer() # maybe its not stopping because the Serial buffer
                                   # is full. So try empty it after each move command
                                   # After experimenting, it looks like this was the reason.
                                   # Tried successuflly the following loop twice ( sually it 
                                   # fails to stop after one iteration of this):
                                   #    30 30, 30 70, 70 70, 70 0, 0 0, 0 10
                                   # This was the issue! Confirmed, now I can stop the car
                                   # consistently!
    
    def read_event_move(self):
        res = None
        #if self.ser.in_waiting > 0:
        '''
        while not res.startswith("wp"):
            res = self.recvFromArduino() # str(s.readline().strip()) #
            if res == None:
                #self.prev_time = 0
                #return None, None, None, None
                break
        '''
        res = self.recvFromArduino()
        
        #print("read_event_move() - prev_time: ", self.prev_time, res)
        #if res != None and res == "": #.startswith("deserialize"):
            #self.prev_time = cur_time
            #return None, None, None, None
        
        if res == "deserialize error":
            self.prev_time = 0
            print("deserialize error...")
            return None, None, None
        
        if res == None:
            self.prev_time = 0
        
        if res == "" or (res != None and not res.startswith("wp")):
            res = None
            
        #if res == None:
            #return None, None, None, None
        
        ax = 0
        ay = 0
        az = 0
        
        if res != None:
            pairs = res.split(',')
            
            if(len(pairs) > 13):
                #target_x = float(pairs[0].split(':')[1])
                #target_y = float(pairs[1].split(':')[1])
                ax = -float(self.extractValue(pairs[2]))
                ay = -float(self.extractValue(pairs[3]))
                #az = float(self.extractValue(pairs[4]))
                gz = float(self.extractValue(pairs[7]))
                yaw    = float(self.extractValue(pairs[8]))
                #pwr_l  = int(  self.extractValue(pairs[9]))
                #pwr_r  = int(  self.extractValue(pairs[10]))
                dt     = int(  self.extractValue(pairs[11])) # msecs
                an     = int(  self.extractValue(pairs[12]))
                dist   = float(self.extractValue(pairs[13]))
            
                return an, dist, dt, gz, yaw, ax, ay
        
        return -1, -1, -1, -1, -1, -1, -1
    
    def sense(self, stepSize=1, angle=90):
        command = {
            "mode": 9,
            "step": stepSize, # set to 0 if not scanning
            "an": angle       
        }
        
        cmdStr = '<' + json.dumps(command) + '>'
        i = self.ser.write(cmdStr.encode())
        
        return self.event_ultrasonic()
        
    def event_ultrasonic(self):
        res = ""
        #if self.ser.in_waiting > 0:
        
        res = self.recvFromArduino() # str(s.readline().strip()) #
        #print("res = ", res)
        if res == "deserialize error" or res == None:
            return None, None
        
        if res == "" or (res != None and not res.startswith("an")):
            return None, None
        
        #print("sense(): ", res)
        parts = res.split(',')
        angle = int(parts[0].split(':')[1])
        dist = float(parts[1].split(':')[1])
        
        return angle, dist
        
        
    def stop(self):
        command = {
            "mode": 4
        }

        cmdStr = '<' + json.dumps(command) + '>'
        i = self.ser.write(cmdStr.encode())
        self.prev_time = 0
        
        #an, dist, dt, gyro_z, yaw = self.read_event_move()
        
    def emptyBuffer(self):
        while self.ser.in_waiting > 0:
            res = self.recvFromArduino()
            
    