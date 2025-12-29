#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from typing import Tuple

import numpy as np


class KalmanFilter:
    """
    Simple Kalman Filter with fading memory control.
    """
    
    def __init__(self, initial_x: int = 0, initial_y: int = 0):
        """
        Initialize the Kalman filter
        :param initial_x: the initial x location.
        :param initial_y: the initial y location.
        """
        
        # TODO: Add code here
        
        # transition function
        self.f = None
        
        # process noise covariance matrix
        self.q = None
    
        # measurement function
        self.h = None
    
        # measurement noise
        self.r = None
    
        # state
        self.x = None
        
        # covariance 
        self.p = None
        
        # identity
        self.i = None
        
        # fading memory control
        self.alpha_decay = 1 #1.10
        
        
    def measurement_update(self, z_x: int, z_y: int, r: float = 0) -> Tuple[int, int]:
        """
        Update the filter with the measurement.
        :param z_x: the measurement in x.
        :param z_y: the measurement in y.
        :return: the estimated x, estimated y
        """
        
        # TODO: Add code here
        pass
    
    def prediction_update(self, dt):
        """
        Update the filter with the prediction.
        :param z_x: the measurement in x.
        :param z_y: the measurement in y.
        :return: the estimated x, estimated y
        """
        
        # TODO: Add code here
        pass
