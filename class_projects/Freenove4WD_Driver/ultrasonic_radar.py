# -*- coding: utf-8 -*-
"""
Created on Thu May 25 00:05:46 2023

@author: ahmed
"""

import serial
import traceback
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import KalmanFilter
import math
import time

# Initialize the figure and axes
fig = plt.figure(figsize=(6, 3))
ax = fig.add_subplot(111, polar=True)

# Set the maximum value for the radar plot
max_value = 300

# track object only if it is within this distance (cm)
tracking_threshold = 70

# Set the initial position of the moving point
theta = np.deg2rad(0)  # Angle in radians
r = 0  # Distance from the origin

# Create the radar plot
dist_ticks = np.linspace(0, 300, 6)
ax.set_ylim(0, max_value)
ax.set_yticks(dist_ticks)
ax.set_yticklabels(['0', '', '120', '', '240', ''])

# Set the plot limits to display a 180-degree plot on top
ax.set_theta_zero_location("E")  # Set zero degrees to the right (East)
ax.set_theta_direction(1)  # Set the direction to clockwise

# Set the theta ticks and labels for a 180-degree plot on top
theta_ticks = np.linspace(0, np.pi, 5)  # Set the number of theta ticks (e.g., 5 ticks)
theta_labels = ['0°', '45°', '90°', '135°', '180°']  # Set the labels for the theta ticks
ax.set_xticks(theta_ticks)
ax.set_xticklabels(theta_labels)

text = ax.text(0, 0, 'dist: ')

# Show grid lines
ax.grid(True)

# Set the plot to display only 0 to 180 degrees instead of full circle
plt.xlim(np.pi, 0)

point = ax.scatter(theta, r, color='green', alpha=0.6, s=10)
kf_point = ax.scatter(theta, r, color='red', alpha=0.6, s=10)

reverse = False

# The bluetooth port on my computer is COM7, you will need
# to update this to the port you are using to communicate
# with your board
s = serial.Serial(port = "COM7", baudrate=9600,
                           bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE)

end_sonar_read = False

kf = KalmanFilter.KalmanFilter()


def polar2cart(theta, r):
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    x = int(r * cos_theta)
    y = int(r * sin_theta)
    return x, y

def cart2polar(x, y):
    rho = math.sqrt(x**2 + y**2)
    phi = math.atan2(y, x)
    return phi, rho


# Read the sonar from serial port
def read_sonar():
    global theta, r, s, end_sonar_read, prev_xy, prevScanCount, scanCount
    
    # read until quit command is issued from main thread
    while(not end_sonar_read):
        res = str(s.readline().strip())
        l = res.split(',')
        angle = int(l[0][8:])
        r = float(l[1][10:].lstrip().split(' ')[0]) 
        theta = np.deg2rad(angle)
            
        x, y = polar2cart(theta, r)
        
        # Track object if it is within a certain distance
        # Keep only one object within this distance for simplicity
        if(r <= tracking_threshold):
            # TODO: Add your code here for tracking the object
            # and using KalmanFilter
            
            # End Code
            print(x, y, round(kf.x[0,0],2), round(kf.x[1,0],2), round(kf.x[2,0],2), round(kf.x[3,0],2))
    
    
# Function to update the moving point on the radar plot
def update_point(frame):
    
    # Update the position of the moving point
    global theta, r, point, text, kf_point
    
    # Update the position of the scatter plot point
    point.set_offsets([(theta, r)])
    phi, rho = cart2polar(kf.x[0,0], kf.x[1,0])
    kf_point.set_offsets([(phi, rho)])
    text.set_text("dist: " + str(r))
    #print(theta, r),
    return point, kf_point, text,
    

# send '9' to Arduino to kick off the 2-D KF program
i = s.write(str(9).encode()) 

# create a thread to read sonar output from Arduino
t1 = threading.Thread(target=read_sonar)
t1.start()

# Initialize the animation
ani = animation.FuncAnimation(fig, update_point, interval=20, blit=True)

# Start the animation loop
plt.show()

resp = input("Enter 'q' to quit: ")
if(resp == 'q'):
    i = s.write(str(4).encode())
    end_sonar_read = True

t1.join()
    
s.close()
