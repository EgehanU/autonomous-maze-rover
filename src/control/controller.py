from __future__ import annotations

import time
import math
from src.config.config import *
from src.control.odometry_and_control import (
    send_command, read_data, position, previous_position, normalize_angle, 
    last_command, get_theta_delta_degrees, reset_position, update_speed
)

"""
    The file for correcting the over rotations, this code has two different purpose:
    1. Making a certain degree of rotations
    2. After a straight command sometimes rover tilts towards right or left, the correction_straight make it straight again.
"""
correction_pwm: int = 120
delay: float = 0.1



def degrees_to_radians(degrees: float) -> float:
    return degrees * (math.pi/180)

def radians_to_degrees(radians: float) -> float:
    return radians * (180/math.pi)
"""
def rotate(angle_degrees):
    # First get close the the target angle then start calibrating with a controller
    initial_theta = position["theta"]
    target_theta = normalize_angle(initial_theta + degrees_to_radians(angle_degrees))
    
    print(f"Starting rotation from {radians_to_degrees(initial_theta):.2f} to {radians_to_degrees(target_theta):.2f}")

    if angle_degrees > 0:
        rotation_command = MOVE_COUNTER_CLOCKWISE
    else:
        rotation_command = MOVE_CLOCKWISE
    
    send_command(rotation_command, turning_speed=100)
    
    while True:
        read_data()  
        current_angle = position["theta"]
        remaining_angle = normalize_angle(target_theta - current_angle)
        
        remaining_degrees = radians_to_degrees(remaining_angle)
        
        if remaining_degrees > 180:
            remaining_degrees -= 360
        elif remaining_degrees < -180:
            remaining_degrees += 360
            
        #print(f"Current:{radians_to_degrees(current_angle):}, Remaining: {remaining_degrees:}")
        
        if abs(remaining_degrees) < 10:
            break
            
        time.sleep(0.05) 
    
    send_command(MOVE_STOP)
    time.sleep(0.1)
    
    # Fine tuning for small adjustment
    fine_tune_speed = 98
    
    while fine_tune_speed> 70:
        read_data()
        current_angle = position["theta"]
        remaining_angle = normalize_angle(target_theta-current_angle)
        remaining_degrees = radians_to_degrees(remaining_angle)
        
        if remaining_degrees > 180:
            remaining_degrees -= 360
        elif remaining_degrees < -180:
            remaining_degrees += 360
            
        #print(f"Current: {radians_to_degrees(current_angle):}, Remaining: {remaining_degrees:}")
        
        # If close within 2 degrre, then done
        if abs(remaining_degrees) < 2:
            break    
    
        if remaining_degrees > 0:
            fine_cmd = MOVE_COUNTER_CLOCKWISE
        else:
            fine_cmd = MOVE_CLOCKWISE
        
        send_command(fine_cmd, turning_speed=fine_tune_speed)
        time.sleep(0.05)
        send_command(MOVE_STOP)
        time.sleep(0.1)
        
        fine_tune_speed -= 1
        
    send_command(MOVE_STOP)
    read_data()  # One final update to get accurate position
    
    final_angle = position["theta"]
    actual_rotation = radians_to_degrees(normalize_angle(final_angle - initial_theta))
    
    #print(f"Rotation complete. Target: {angle_degrees:}, Actual: {actual_rotation:}")
    
    return actual_rotation
"""
def rotate(angle_degrees: float, timeout_seconds: float = 10) -> float:
    # Same rotate function however with a time limit
    
    start_time = time.time()
    
    # First get close to the target angle then start calibrating with a controller
    initial_theta = position["theta"]
    target_theta = normalize_angle(initial_theta + degrees_to_radians(angle_degrees))
    
    
    if angle_degrees > 0:
        rotation_command = MOVE_COUNTER_CLOCKWISE
    else:
        rotation_command = MOVE_CLOCKWISE
        
    send_command(rotation_command, turning_speed=100)
    
    while True:
        #  timeout added
        if time.time() - start_time > timeout_seconds:
            send_command(MOVE_STOP)
            read_data()
            final_angle = position["theta"]
            actual_rotation = radians_to_degrees(normalize_angle(final_angle - initial_theta))
            return actual_rotation
        
        read_data()
        current_angle = position["theta"]
        remaining_angle = normalize_angle(target_theta - current_angle)
        remaining_degrees = radians_to_degrees(remaining_angle)
        
        if remaining_degrees > 180:
            remaining_degrees -= 360
        elif remaining_degrees < -180:
            remaining_degrees += 360
            
        if abs(remaining_degrees) < 10:
            break
            
        time.sleep(0.05)
    
    send_command(MOVE_STOP)
    time.sleep(0.1)
    
    fine_tune_speed = 98
    while fine_tune_speed > 70:
        if time.time() - start_time > timeout_seconds:
            send_command(MOVE_STOP)
            read_data()
            final_angle = position["theta"]
            actual_rotation = radians_to_degrees(normalize_angle(final_angle - initial_theta))
            return actual_rotation
        
        read_data()
        current_angle = position["theta"]
        remaining_angle = normalize_angle(target_theta - current_angle)
        remaining_degrees = radians_to_degrees(remaining_angle)
        
        if remaining_degrees > 180:
            remaining_degrees -= 360
        elif remaining_degrees < -180:
            remaining_degrees += 360
            
        # If close within 2 degrees, then done
        if abs(remaining_degrees) < 2:
            break
            
        if remaining_degrees > 0:
            fine_cmd = MOVE_COUNTER_CLOCKWISE
        else:
            fine_cmd = MOVE_CLOCKWISE
            
        send_command(fine_cmd, turning_speed=fine_tune_speed)
        time.sleep(0.05)
        send_command(MOVE_STOP)
        time.sleep(0.1)
        fine_tune_speed -= 1
    
    send_command(MOVE_STOP)
    read_data()  # One final update to get accurate position
    final_angle = position["theta"]
    actual_rotation = radians_to_degrees(normalize_angle(final_angle - initial_theta))
    
    rotation_time = time.time() - start_time
    return actual_rotation
def rotate_90_cw() -> float:
    return rotate(-90)

def rotate_90_ccw() -> float:
    return rotate(90)

"""def correction_straight():
    global correction_pwm
    correction_pwm = 100
    
    read_data()
    current_angle_deg = radians_to_degrees(position["theta"])
    
    
    while (correction_pwm > 74) and (abs(current_angle_deg) > 2):
        if current_angle_deg < 0:
            send_command(MOVE_COUNTER_CLOCKWISE, turning_speed=correction_pwm)
        else:
            send_command(MOVE_CLOCKWISE, turning_speed=correction_pwm)
            
        time.sleep(0.05)
        send_command(MOVE_STOP)
        time.sleep(0.1)
        
        correction_pwm = correction_pwm - 2
        read_data()
        current_angle_deg = radians_to_degrees(position["theta"])
        #print(f"Straightening - Current angle: {current_angle_deg:}")
    
    send_command(MOVE_STOP)
    read_data()
    final_angle = radians_to_degrees(position["theta"])
    
    #print(f"Final angle: {final_angle:}")
"""
def correction_straight(target_angle: float | None = None) -> float:
    global correction_pwm
    correction_pwm = 100

    read_data()
    
    if target_angle is None:
        target_angle = position["theta"]
    
    current_angle = position["theta"]
    angle_diff = normalize_angle(current_angle - target_angle)
    angle_diff_deg = math.degrees(angle_diff)
    
    while (correction_pwm > 74) and (abs(angle_diff_deg) > 2):
        if angle_diff_deg < 0:
            send_command(MOVE_COUNTER_CLOCKWISE, turning_speed=correction_pwm)
        else:
            send_command(MOVE_CLOCKWISE, turning_speed=correction_pwm)
            
        time.sleep(0.05)
        send_command(MOVE_STOP)
        time.sleep(0.1)
        
        correction_pwm = correction_pwm - 2
        
        read_data()
        current_angle = position["theta"]
        angle_diff = normalize_angle(current_angle - target_angle)
        angle_diff_deg = math.degrees(angle_diff)
        
    
    send_command(MOVE_STOP)
    read_data()
    final_angle = math.degrees(position["theta"])
    #print(f"Target angle: {math.degrees(target_angle):}, Final angle: {final_angle:}")
    
    return final_angle


def move_forward_cm(distance_cm: float = 55, speed: int = 130) -> None:
    read_data()
    x_start = position["x"]
    y_start = position["y"]

    send_command(MOVE_FORWARD, speed=speed)

    while True:
        time.sleep(0.05)
        read_data()
        dx = position["x"] - x_start
        dy = position["y"] - y_start
        travelled = math.sqrt(dx**2 + dy**2)

        if travelled >= distance_cm:
            break

    send_command(MOVE_STOP)
    time.sleep(0.1)
    correction_straight()
    read_data()




