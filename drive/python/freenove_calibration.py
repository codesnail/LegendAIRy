# -*- coding: utf-8 -*-
"""
Created on Mon Dec  9 22:12:29 2024

@author: Yaser
"""
import time
import numpy as np
from SkidSteerArdu import SkidSteerArdu
import traceback
import matplotlib.pyplot as plt
import seaborn as sns
import statsmodels.formula.api as smf
import pandas as pd
import json
import keyboard

def calibrate_turn(car, direction=1):
    DIST_THRESH = 20
    TIME_THRESH = 1.5
    turn_rates = []
    extra_turn_rates = []
    
    num_iters = 3
    dt = 0
    
    speeds = [100, 110, 120, 130, 140, 150]
    reverse_speed = speeds[0]
    
    for speed in speeds:
        
        for i in range(num_iters):
            curr_time = time.time()
            car.move(speed*direction, -speed*direction)
            
            for c in range(100000): # as long as there is input to read
                an, dist, _, _, _, _, _ = car.read_event_move()
                                    
                end_time = time.time() # sec
                dt_temp = end_time - curr_time
                
                if((dist != -1) and (dist <= DIST_THRESH)) and (dt == 0 or dt_temp > TIME_THRESH):
                    car.stop()
                    car.emptyBuffer()
                    print("### dist < threhsold so stopping...")
                    
                    print("dist, dt, dt_temp: ", dist, dt, dt_temp)
                    dt = dt_temp
                    if i == 0 and speed == speeds[0]: # first turn
                        turn_rates.append([speed, np.pi/dt])  # first time the obstacle should be at 180 degrees
                    else:
                        turn_rates.append([speed, 2 * np.pi/dt]) # second and subsequent times, the car will need to turn 360 degrees to see the obstacle again
                        
                    # At higher speeds the car surpasses the object
                    # so turn the car back until it is facing the object before the next iteration
                    
                    # Read the dist from sonar
                    an, dist = car.sense()
                    
                    while(dist == None):
                        an, dist = car.event_ultrasonic()
                    
                    # If distance > threshold then the car is not 
                    # facing the reference object, in which case
                    # turn it back until it faces it
                    if dist > DIST_THRESH:
                        curr_time = time.time()
                        car.move(-reverse_speed*direction, reverse_speed*direction)
                        
                        for c in range(100000): # as long as there is input to read
                            an, dist, _, _, _, _, _ = car.read_event_move()
                            
                            end_time = time.time() # sec
                            stopping_time = end_time - curr_time
                            
                            if((dist != -1) and (dist <= DIST_THRESH)):
                                car.stop()
                                car.emptyBuffer()
                                extra_turn_rates.append([reverse_speed, stopping_time])
                                break
                    else:
                        extra_turn_rates.append([reverse_speed, 0])
                    
                    break
    
    return turn_rates, extra_turn_rates

def calibrate_velocity(car):
    STOP_DIST = 25
    num_iters = 2
    dt = 0
    
    vel = []
    imu_data = []
    ref_dist = 0
    fwrd_pwr_ratio_L = 1
    fwrd_pwr_ratio_R = 1
    
    rvrs_pwr_ratio_L = 1
    rvrs_pwr_ratio_R = 1
    
    for pwr in [90, 100, 110, 120, 130, 140, 150]:
        
        an, dist = car.sense()
        
        while(dist == None):
            an, dist = car.event_ultrasonic()
            print("dist = ", dist)
        
        ref_dist = dist
        car.stop()
        car.emptyBuffer()
        
        for i in range(num_iters):
            
            x = input("Enter r to decrease power on right side, l to decrease power on left, c to continue: ")
            if x == 'r':
                if pwr < 0: # this means the previous run was with pwr > 0, so adjust forward pwr ratios
                    # this will take effect the next time we move forward
                    fwrd_pwr_ratio_R = fwrd_pwr_ratio_R - 0.05
                    pwrR = pwr * fwrd_pwr_ratio_R
                elif pwr > 0:
                    rvrs_pwr_ratio_R = rvrs_pwr_ratio_R - 0.05
                    pwrR = pwr * rvrs_pwr_ratio_R
            elif x == 'l':
                if pwr < 0: # this means the previous run was with pwr > 0, so adjust forward pwr ratios
                    fwrd_pwr_ratio_L = fwrd_pwr_ratio_L - 0.05
                    pwrL = pwr * fwrd_pwr_ratio_L
                elif pwr > 0:
                    rvrs_pwr_ratio_L = rvrs_pwr_ratio_L - 0.05
                    pwrL = pwr * rvrs_pwr_ratio_L
            elif x == 'c':
                pass
            
            curr_time = time.time()
            
            if pwr < 0:
                # use the reverse power ratio adjusted from last time
                pwrL = pwr * rvrs_pwr_ratio_L
                pwrR = pwr * rvrs_pwr_ratio_R
            elif pwr > 0:
                pwrL = pwr * fwrd_pwr_ratio_L
                pwrR = pwr * fwrd_pwr_ratio_R
                
            car.move(pwrL, pwrR)
            imu_rows = []
            for c in range(100000): # as long as there is input to read
                end_an, new_dist, dt, gz, yaw, ax, ay = car.read_event_move()
                print("new_dist = ", new_dist)
                imu_rows.append([dt, gz, yaw, ax, ay])
                if (new_dist != -1) and ( 
                        (pwr > 0 and new_dist <= STOP_DIST) 
                        or (pwr < 0 and new_dist >= ref_dist)):
                    end_time = time.time() # sec
                    car.stop()
                    dt = end_time - curr_time
                    vel.append([abs(pwr), abs(dist-new_dist)/dt])
                    imu_data.append(imu_rows)
                    imu_rows.clear()
                    pwr = -pwr
                    ref_dist = dist
                    dist = new_dist
                    car.emptyBuffer()
                    break
                
    return vel, fwrd_pwr_ratio_L, fwrd_pwr_ratio_R, rvrs_pwr_ratio_L, rvrs_pwr_ratio_R, imu_data

def calibrate_velocity2(car):
    STOP_DIST = 25
    num_iters = 2
    dt = 0
    
    vel = []
    ref_dist = 0
    
    for pwr in [90, 100, 110, 120, 130, 140, 150]:
        
        an, dist = car.sense()
        
        while(dist == None):
            an, dist = car.event_ultrasonic()
            print("dist = ", dist)
        
        ref_dist = dist
        car.stop()
        car.emptyBuffer()
        
        for i in range(num_iters):
            curr_time = time.time()
            car.move(pwr, pwr)
            
            for c in range(100000): # as long as there is input to read
                end_an, new_dist, _, _, _, _, _ = car.read_event_move()
                print("new_dist = ", new_dist)
                
                if (new_dist != -1) and ( 
                        (pwr > 0 and new_dist <= STOP_DIST) 
                        or (pwr < 0 and new_dist >= ref_dist)):
                    end_time = time.time() # sec
                    car.stop()
                    dt = end_time - curr_time
                    vel.append([abs(pwr), abs(dist-new_dist)/dt])
                    pwr = -pwr
                    ref_dist = dist
                    dist = new_dist
                    car.emptyBuffer()
                    break
    return vel

def calibrate_imu():
    f = open('imu_readings.txt', 'r')
    lines = f.readlines()
    print(lines)
    total_gyz = 0
    count = 0
    for imu in lines:
        readings = imu.strip().split(',')
        for val in readings:
            if val.startswith('gyz'):
                gyz = val.split(':')[1]
                total_gyz += float(gyz)
                print(gyz)
                count += 1
        #print(reading)
    
    avg_gyz_rest = total_gyz / count
    print("avg_gyz_rest: ", avg_gyz_rest)
    f.close()

def run_calibrate_turn(direction=1):    
    car = SkidSteerArdu()
    try:
        turn_rates, extra_turn = calibrate_turn(car, direction)
        print("turn rate (rad/sec): ")
        print(turn_rates)
        turn_rates_deg_msec = [[speed, rate * (180/np.pi) * (1/1000)] for speed, rate in turn_rates]
        print("turn rate (deg/msec): ")
        print(turn_rates_deg_msec)
        return turn_rates, extra_turn
    
    except:
        traceback.print_exc()
        car.stop()
    
    return None

def run_calibrate_vel():    
    car = SkidSteerArdu()
    try:
        pwr_vel, pwr_ratio_FwL, pwr_ratio_FwR, pwr_ratio_RvL, pwr_ratio_RvR, imu_data = calibrate_velocity(car)
        print("vel (cm/sec): ")
        print(pwr_vel)
        vel_mps = [[pwr, vel / 100] for pwr, vel in pwr_vel]
        print("vel (m/sec): ")
        print(vel_mps)
        if len(vel_mps)%2 == 0:
            return vel_mps, pwr_ratio_FwL, pwr_ratio_FwR, pwr_ratio_RvL, pwr_ratio_RvR
    except:
        traceback.print_exc()
        car.stop()
    
    return None

def remove_outliers_iqr(data, column, threshold=0.2):
    q1 = data[column].quantile(0.30)
    q3 = data[column].quantile(0.70)
    iqr = q3 - q1
    print("q1, q3 = ", q1, q3)
    lower_bound = q1 - threshold * iqr
    upper_bound = q3 + threshold * iqr
    return data[(data[column] >= lower_bound) & (data[column] <= upper_bound)]


def remove_outliers_zscore(data, column, threshold=0.5): #1.07):
    z_scores = np.abs((data[column] - data[column].mean()) / data[column].std())
    return data[(z_scores < threshold)]

def create_lr_model(data, remove_outliers=False):    
    power_level = [x for x,y in data]
    ang_vel =    [y for x,y in data]
    
    X = np.array([power_level, ang_vel]).transpose()
    
    #print(X)
    #plt.scatter(X[:, 0], X[:, 1])
    #plt.show()
    
    df = pd.DataFrame(X, columns = ['power_level', 'velocity'])
    if remove_outliers:
        #df = remove_outliers_iqr(df, 'velocity')
        df = remove_outliers_zscore(df, 'velocity')
        
    model = smf.ols('velocity ~ power_level', data=df).fit()
    
    sns.regplot(x='power_level', y='velocity', data=df)
    
    '''
    #sm.qqplot(res, stats.t, fit=True, line="s")
    print("res = ", type(res), res)
    df_no_outliers = df.where(res < 0.15)
    
    model = smf.ols('velocity ~ power_level', data=df_no_outliers).fit()
    
    sns.regplot(x='power_level', y='velocity', data=df_no_outliers)
    '''
    
    res = model.resid
    print("res = ", type(res), res)
    
    plt.show()
    
    return model.params

def calibrate():
    
    lr_model_dict = dict()
    input("Place object about 15 cm BEHIND the car, then press enter")
    
    turn_directions = [1, -1]
    
    for direction in turn_directions:
        data = run_calibrate_turn(direction)
        #data = [[100, 0.8213263853902903], [100, 0.7342320763017197], [100, 0.6851843326846826], [110, 1.0117631592167236], [110, 1.0152597711952578], [110, 1.02503248411181], [120, 1.3517520637901386], [120, 1.3711327300292355], [130, 1.7145111203363397], [130, 1.6764205533002137], [130, 1.7645133563600246], [140, 2.015886330407387], [140, 1.9439417922682496], [150, 2.38678569806777], [150, 2.3776173588300007], [150, 2.296185067580511]]  
        #data = data, None # just to make 'data' a tuple so it is same as the one returned by the actual method
        print(data)
        if data == None:
            print("Calibration didn't run properly, please try again.")
        else:
            turn_data, extra_turn = data
            model_params = create_lr_model(turn_data, remove_outliers=False)
            print(model_params)
            key_turn = 'turn_model'
            key_stop_time = 'stop_time'
            if direction == 1:
                key_turn += '_right'
                key_stop_time += '_right'
            elif direction == -1:
                key_turn += '_left'
                key_stop_time += '_left'
                
            lr_model_dict[key_turn] = model_params.to_dict()
            #lr_model_dict[key_stop_time] = extra_turn
    
    if lr_model_dict:
        f = open('freenove_calibration_turns.txt', 'w')
        json.dump(lr_model_dict, f)
        f.close()
     
    lr_model_dict.clear()
    resp = input("Place obstacle atleast 1 meter away, then press enter. Or q to quit: ")
    if resp == 'q':
        return
    data = run_calibrate_vel()
    #data = [[90, 0.28160426416679435], [90, 0.25635981394940144], [100, 0.3303139717831327], [100, 0.2923517968173589], [110, 0.38732069492552773], [110, 0.34256951759406196], [120, 0.4474300864394937], [120, 0.38295596178586544], [130, 0.46567005592178257], [130, 0.7811446996820031], [140, 0.8719087969884062], [140, 0.5974554094570063], [150, 0.8619491386272541], [150, 0.8029889912327418]]
    #data = data, None, None, None, None
    if data == None:
        print("Calibration didn't run properly, please try again.")
    else:
        vel_mps, pwr_ratio_FwL, pwr_ratio_FwR, pwr_ratio_RvL, pwr_ratio_RvR = data
        vel_mps = [[abs(pwr), vel] for pwr, vel in vel_mps]
        print(vel_mps)
        model_params = create_lr_model(vel_mps, remove_outliers=True)  
        print(model_params)
        lr_model_dict['vel_model'] = model_params.to_dict()
        lr_model_dict['pwr_ratio_FwL'] = pwr_ratio_FwL
        lr_model_dict['pwr_ratio_FwR'] = pwr_ratio_FwR
        lr_model_dict['pwr_ratio_RvL'] = pwr_ratio_RvL
        lr_model_dict['pwr_ratio_RvR'] = pwr_ratio_RvR
        
        f = open('freenove_calibration_vel.txt', 'w')
        json.dump(lr_model_dict, f)
        f.close()

calibrate()
#data = run_calibrate_turn()
#data = run_calibrate_vel()
#calibrate_imu()

