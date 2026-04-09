from __future__ import annotations

from typing import Any, TypedDict

import smbus
import time
import struct
import math
from src.config.config import *
import matplotlib.pyplot as plt


class PositionState(TypedDict, total=False):
    x: float
    y: float
    theta: float
    stop_flag: bool


device_speed: int = 100
device_turning_speed: int = 100
bus: Any = smbus.SMBus(I2C_BUS)

ANGLE_CALIBRATION: float = 90 / 110
# Position tracking
position: PositionState = {"x": 0.0, "y": 0.0, "theta": 0.0, "stop_flag": False}
previous_position: PositionState = {"x": 0.0, "y": 0.0, "theta": 0.0}
prev_left_encoder: float | None = None
prev_right_encoder: float | None = None
initialized: bool = False
last_command: int | None = None
distance_travelled: float = 0.0

ir_readings: list[int | float | None] = []
IR_SAMPLES: int = 5
filtered_ir_distance: float | None = 0.0
SET_DISTANCE_FLAG: int = 9
SET_TARGET_DISTANCE: int = 10
stop_flag: bool = False

encoder_history: dict[str, list[float]] = {"left": [], "right": []}
HISTORY_SIZE: int = 3
last_update_time: float | None = None


def normalize_angle(angle: float) -> float:
    # Convert to range 0, 2pi
    angle = angle % (2 * math.pi)
    
    # Convert to range -pi, pi
    if angle > math.pi:
        angle -= 2 * math.pi
    
    return angle

def calculate_median(values: list[int | float | None]) -> float | None:
    sorted_values = sorted(values)
    length = len(sorted_values)
    if length == 0:
        return None  
    if length % 2 == 1:
        return sorted_values[length // 2]
     
    middle_right = length // 2
    middle_left = middle_right - 1
    return (sorted_values[middle_left] + sorted_values[middle_right]) / 2

def encoder_to_distance(encoder_value: float) -> float:
    # Convert encoder value to distance in cm
    return (encoder_value / PULSES_PER_REV) * CM_PER_ROTATION

def send_command(
    command: int,
    speed: int | None = None,
    turning_speed: int | None = None,
    servo_angle: int | None = None,
) -> None:
    global device_speed, device_turning_speed, last_command
    if speed is None:
        speed = device_speed
    if turning_speed is None:
        turning_speed = device_turning_speed
    
    # Update last command for tracking movement type
    if command in [MOVE_FORWARD, MOVE_BACKWARD, MOVE_CLOCKWISE, MOVE_COUNTER_CLOCKWISE]:
        last_command = command
    
    try:
        # For servo angle command, angle is the 4th byte
        if command == SET_SERVO_ANGLE and servo_angle is not None:
            bus.write_i2c_block_data(ARDUINO_ADDRESS, command, [speed, turning_speed, servo_angle])
            #print(f"Sent servo angle command:{command}, angle: {servo_angle}")
        else:
            bus.write_i2c_block_data(ARDUINO_ADDRESS, command, [speed, turning_speed])
            #print(f"Sent command: {command}, speed: {speed}, turn: {turning_speed}")
    except Exception as e:
        print("Failed to send command:", e)




def read_data() -> (
    tuple[int | None, float | None, float | None]
    | tuple[None, None, None, None]
):
    global distance_travelled, stop_flag, prev_left_encoder, prev_right_encoder, position, previous_position, initialized, ir_distance
    
    try:
        data = bus.read_i2c_block_data(ARDUINO_ADDRESS, 0, 11)
        ir_distance = (data[0] << 8) | data[1]  # in cm
        position["stop_flag"] = bool(data[10])
        #print("data[10] =", data[10])
        #print("position stop_flag =", position["stop_flag"])
        
        # Get encoder values
        right_encoder = struct.unpack('<f', bytes(data[2:6]))[0]  # Little endian 
        left_encoder = struct.unpack('<f', bytes(data[6:10]))[0]  
        
        # Convert to distances
        right_cm = encoder_to_distance(right_encoder)
        left_cm = encoder_to_distance(left_encoder)
        
        #print(f"IR Distance: {ir_distance} cm")
        #print(f"Right encoder: {right_encoder:.2f} ({right_cm:.2f} cm)")
        #print(f"Left encoder: {left_encoder:.2f} ({left_cm:.2f} cm)")
        
        # In case this is the first read
        if not initialized:
            prev_right_encoder = right_encoder
            prev_left_encoder = left_encoder
            initialized = True
            #print("Initialized encoders")
        else:
            previous_position["x"] = position["x"]
            previous_position["y"] = position["y"]
            previous_position["theta"] = position["theta"]
            
            # Calculate change in encoder values, after storing previous
            right_delta = right_encoder - prev_right_encoder
            left_delta = left_encoder - prev_left_encoder
            
            right_delta_cm = encoder_to_distance(right_delta)
            left_delta_cm = encoder_to_distance(left_delta)
            
            # Displacement
            dist_delta = (right_delta_cm + left_delta_cm) / 2
            
            
            # CCW positive, CW negative
            raw_angle_delta = (right_delta_cm - left_delta_cm) / WHEEL_BASE
            angle_delta = raw_angle_delta * ANGLE_CALIBRATION
            
            # Update position 
            if abs(angle_delta) < 0.01:  # Nearly straight motion
                delta_y = dist_delta * math.cos(position["theta"])
                delta_x = dist_delta * math.sin(position["theta"])
                new_theta = position["theta"]

            # if last_command == MOVE_FORWARD:
            #     delta_y = dist_delta * math.cos(position["theta"])
            #     delta_x = dist_delta * math.sin(position["theta"])
            #     new_theta = position["theta"]
            else:
                # Robot moved in an arc
                # Calculate radius of turn
                radius = (WHEEL_BASE / 2) * (right_delta_cm + left_delta_cm) / (right_delta_cm - left_delta_cm)
                
                delta_y = radius * (math.sin(position["theta"] + angle_delta) - math.sin(position["theta"]))
                delta_x = radius * (math.cos(position["theta"]) - math.cos(position["theta"] + angle_delta))
                new_theta = position["theta"] + angle_delta
            
            position["x"] += delta_x
            position["y"] += delta_y
            
            # Normalize the angle to -pi to +pi 
            position["theta"] = normalize_angle(new_theta)
            
            #print(f"Movement: Linear={dist_delta:.2f}cm, Angular={math.degrees(angle_delta):.2f}°")
            #print(f"Angle change: Previous={math.degrees(previous_position['theta']):.2f}° → Current={math.degrees(position['theta']):.2f}°")
            
            # Update encoder references for next calculation
            prev_right_encoder = right_encoder
            prev_left_encoder = left_encoder
            
        #print(f"Position: ({position['x']:.2f}, {position['y']:.2f}, {math.degrees(position['theta']):.2f}°)")
        return ir_distance, right_encoder, left_encoder
    
    except Exception as e:
        print("Failed to read data:", e)
        return None, None, None, None


def read_filtered_data() -> float | None:
    global filtered_ir_distance
    i = 0
    while i < IR_SAMPLES:
        dist, _, _ = read_data()
        ir_readings.append(dist)
        time.sleep(0.2)
        i = i+1
    
    filtered_ir_distance = calculate_median(ir_readings)
    ir_readings.clear()
    #print(f"Position: ({position['x']:.2f}, {position['y']:.2f}, {math.degrees(position['theta']):.2f}°)")
    return filtered_ir_distance 

def reset_position() -> None:
    global position, previous_position
    previous_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
    position = {"x": 0.0, "y": 0.0, "theta": 0.0}
    print("Position reset to (0,0,0)")

def reset_encoders() -> None:
    global position, previous_position, prev_left_encoder, prev_right_encoder, initialized
    
    send_command(RESET_ENCODERS)
    time.sleep(0.2)  
    
    # Reset 
    previous_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
    position = {"x": 0.0, "y": 0.0, "theta": 0.0}
    prev_left_encoder = None
    prev_right_encoder = None
    initialized = False
    
    # Read once to initialize
    read_data()
    print("Encoders and position reset to zero")



def move_for_time(command: int, duration_seconds: float) -> None:
    #print(f"Moving for {duration_seconds} seconds")
    
    send_command(command)
    
    # Monitor position while moving
    start_time = time.time()
    while time.time() - start_time < duration_seconds:
        time.sleep(0.2)  
        read_data()
    
    send_command(MOVE_STOP)
    time.sleep(0.1)
    read_data()


def update_speed(new_speed: int | None = None, new_turn: int | None = None) -> None:
    global device_speed, device_turning_speed
    send_command(MOVE_STOP)
    time.sleep(0.2)
    if new_speed is None and new_turn is None:  # To write the values manually, use function with no arguments
        new_speed = int(input("Enter new forward/backward speed 0-255: "))
        new_turn = int(input("Enter new turning speed 0-255: "))
    if new_speed is not None and (0 <=new_speed<= 255):
        device_speed = new_speed
    if new_turn is not None and (0 <= new_turn<= 255):
        device_turning_speed = new_turn
        print(f"Updated speeds->Straight: {device_speed}, Turn: {device_turning_speed}")
  


def get_theta_delta_degrees() -> float:
    current_theta_deg = math.degrees(position["theta"])
    previous_theta_deg = math.degrees(previous_position["theta"])
    
    delta = current_theta_deg - previous_theta_deg
 
    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360
        
    return delta




def print_position() -> None:
   read_data()
   print(f"The location is {position['x']:.1f}, {position['y']:.1f}, {math.degrees(position['theta']):.4f}°")
   print("Stop flag:", position["stop_flag"])


def move_for_time_with_logging(
    command: int, duration_seconds: float
) -> tuple[list[float], list[float | None], list[float | None]]:
    send_command(command)
    
    start_time = time.time()
    timestamps = []
    left_encoders = []
    right_encoders = []

    while time.time() - start_time < duration_seconds:
        _, right_encoder, left_encoder = read_data()
        timestamps.append(time.time() - start_time)
        left_encoders.append(left_encoder)
        right_encoders.append(right_encoder)
        time.sleep(0.1)
    
    send_command(MOVE_STOP)
    time.sleep(0.1)
    read_data()

    return timestamps, left_encoders, right_encoders


def plot_encoders(
    timestamps: list[float],
    left_encoders: list[float | None],
    right_encoders: list[float | None],
    title: str = "Encoder Values Over Time",
) -> None:
    plt.figure()
    plt.plot(timestamps, left_encoders, label='Left Encoder')
    plt.plot(timestamps, right_encoders, label='Right Encoder')
    plt.xlabel("Time (s)")
    plt.ylabel("Encoder Value")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()
