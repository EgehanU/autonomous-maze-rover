from __future__ import annotations

import math
import time
from src.control.odometry_and_control import position, read_data, reset_encoders
from src.sensors.imu_filtered import (
    get_heading,
    reset_heading,
    start_heading_tracker,
    stop_heading_tracker,
)

FUSION_ALPHA: float = 0.7  # 0.7 = favor encoders, 0.3 for IMU heading

def convert_imu_to_encoder_style(imu_heading_deg: float) -> float:
    if imu_heading_deg > 180:
        return imu_heading_deg - 360  # CW negative
    else:
        return imu_heading_deg        # ccw positive

def normalize_angle_to_encoder_range(angle_deg: float) -> float:
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg


def fuse_headings(encoder_theta_rad: float, imu_heading_deg: float) -> float:
    encoder_deg = math.degrees(encoder_theta_rad)
    encoder_deg = normalize_angle_to_encoder_range(encoder_deg)

    imu_encoder_style_deg = convert_imu_to_encoder_style(imu_heading_deg)

    # Compute difference and fuse
    diff = imu_encoder_style_deg - encoder_deg
    diff = normalize_angle_to_encoder_range(diff)

    fused_deg = encoder_deg + (1 - FUSION_ALPHA) * diff
    fused_deg = normalize_angle_to_encoder_range(fused_deg)

    return fused_deg


def get_fused_heading() -> float:
    imu_heading = get_heading()  # IMU heading is between 0 and 360
    
    # Fuse the headings
    fused_heading = fuse_headings(position["theta"], imu_heading)
    
    return fused_heading

def get_fused_position() -> float:
    read_data()  
    imu_heading = get_heading()  
    
    # Get fused heading
    fused_heading_deg = fuse_headings(position["theta"], imu_heading)
    fused_heading_rad = math.radians(fused_heading_deg)
    
    #return {"x": position["x"],"y": position["y"],"theta": fused_heading_rad,"theta_deg": fused_heading_deg}
    return fused_heading_deg
def heading_difference(start: float, end: float) -> float:
    diff = (end - start + 360) % 360
    if diff > 180:
        diff -= 360
    return diff

def reset_fused_system() -> None:
    reset_encoders()
    reset_heading()
    print("Fused system reset")

def start_fused_system() -> bool:
    return start_heading_tracker()

def stop_fused_system() -> bool:
    return stop_heading_tracker()
