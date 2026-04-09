from __future__ import annotations

from typing import Any

import smbus
import time
import math
import numpy as np
import numpy.typing as npt
import threading

# The imu for getting the heading angle

Quaternion = npt.NDArray[np.float64]

I2C_BUS: int = 3
MPU6500_ADDR: int = 0x68

# MPU6500 Register addr
MPU_PWR_MGMT_1: int = 0x6B
MPU_CONFIG: int = 0x1A
MPU_GYRO_CONFIG: int = 0x1B
MPU_ACCEL_CONFIG: int = 0x1C
MPU_ACCEL_XOUT_H: int = 0x3B
MPU_GYRO_XOUT_H: int = 0x43
MPU_WHO_AM_I: int = 0x75

# Configuration values, most senstivive ones are used, robot is not fast enough for me to use less sensitive addr
GYRO_SCALE_250: int = 0x00
ACCEL_SCALE_2G: int = 0x00


# Scales used as most sensitive, but if robot is too fast the others can be tried. As for mars rover, the most sensitive ones works
GYRO_SCALE: int = GYRO_SCALE_250
ACCEL_SCALE: int = ACCEL_SCALE_2G


current_heading: float = 0.0
gyro_sensitivity: float = 131.0  #For most sensitive , 32768 / 250
accel_sensitivity: float = 16384.0 # 32768 / 2g
last_time: float | None = None
gyro_offset_x: float = 0.0
gyro_offset_y: float = 0.0
gyro_offset_z: float = 0.0

# Filter parameters
madgwick_beta: float = 0.1  # Madgwick filter gain
mahony_kp: float = 2.0      # Mahony prop gain
mahony_ki: float = 0.1      #Mahony integral gain
quaternion: Quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # Initial quaternion, w,x,y,z
integral_error: npt.NDArray[np.float64] = np.array([0.0, 0.0, 0.0], dtype=float)  #for mahony integral term

FILTER_TYPE: str = "MADGWICK"  # "MADGWICK" or "MAHONY" MADDGWICK gives better results

# threading
running: bool = False
heading_thread: threading.Thread | None = None
initialized: bool = False
bus: Any = None

calibration_flag: bool = False

def init_mpu6500() -> bool:

    global bus, initialized

    try:
        bus = smbus.SMBus(I2C_BUS)
    except Exception as e:
        print("Error")
        return False

    try:
        # resetting and waking up the module
        bus.write_byte_data(MPU6500_ADDR, MPU_PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        bus.write_byte_data(MPU6500_ADDR, MPU_PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # Configure gyro and acc
        bus.write_byte_data(MPU6500_ADDR, MPU_GYRO_CONFIG, GYRO_SCALE)
        bus.write_byte_data(MPU6500_ADDR, MPU_ACCEL_CONFIG,ACCEL_SCALE)
        bus.write_byte_data(MPU6500_ADDR, MPU_CONFIG, 0x03)  # arounf 44Hz

        global gyro_sensitivity, accel_sensitivity
        
        gyro_sensitivity = 131.0
        accel_sensitivity = 16384.0
        
        initialized = True
        return True

    except Exception as e:
        print("Error initilaizing")
        return False

def read_accel_data() -> tuple[float, float, float]:
    data = bus.read_i2c_block_data(MPU6500_ADDR, MPU_ACCEL_XOUT_H , 6)
    # Converting the raw values to 16 bit and converting it to g

    accel_x = (data[0] << 8) | data[1]
    if accel_x > 32767: accel_x -= 65536
    accel_y = (data[2] << 8) | data[3]
    if accel_y > 32767: accel_y -= 65536
    accel_z = (data[4]<< 8) | data[5]
    if accel_z > 32767: accel_z -= 65536
    
    accel_x = accel_x / accel_sensitivity
    accel_y = accel_y / accel_sensitivity
    accel_z= accel_z / accel_sensitivity
    return accel_x, accel_y, accel_z


def read_gyro_data() -> tuple[float, float, float]:

    data = bus.read_i2c_block_data(MPU6500_ADDR, MPU_GYRO_XOUT_H, 6)
    gyro_x = (data[0] << 8) | data[1]
    if gyro_x > 32767: gyro_x -= 65536
    gyro_y = (data[2] << 8) | data[3]
    if gyro_y > 32767: gyro_y -= 65536
    gyro_z = (data[4] << 8) | data[5]
    if gyro_z > 32767: gyro_z -= 65536
   
    gyro_x = gyro_x / gyro_sensitivity - gyro_offset_x
    gyro_y = gyro_y / gyro_sensitivity - gyro_offset_y
    gyro_z = gyro_z / gyro_sensitivity - gyro_offset_z
    return gyro_x, gyro_y, gyro_z


def calibrate_gyroscope() -> bool:
    global gyro_offset_x, gyro_offset_y, gyro_offset_z

    print("\nCalibrating, don't move")

    samples = 0
    sum_x, sum_y, sum_z = 0, 0, 0

    start_time = time.time()
    while time.time() - start_time < 5:
        gyro_x, gyro_y, gyro_z = read_gyro_data()

        if gyro_x is not None:
            # remove offsets temporarly
            gyro_x += gyro_offset_x
            gyro_y += gyro_offset_y
            gyro_z += gyro_offset_z

            sum_x += gyro_x
            sum_y += gyro_y
            sum_z += gyro_z
            samples += 1

            time.sleep(0.01)

    if samples > 0:
        gyro_offset_x = sum_x / samples
        gyro_offset_y = sum_y / samples
        gyro_offset_z = sum_z / samples

        return True
    else:
        print("\nCalibration failed.")
        return False

def quaternion_to_euler(q: Quaternion) -> tuple[float, float, float]:

    # Normalize quaternion, then calculate x axis , y and z axis rotations (roll, ptich and yaw)
    norm = np.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
    q = q/norm

    w,x, y, z = q


    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x +y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)


    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    # Normalize to between 0 and 360
    while yaw < 0: yaw+= 360
    while yaw >= 360: yaw -= 360

    return roll, pitch, yaw

def madgwick_update(ax: float, ay: float, az: float, gx: float, gy: float, gz: float, dt: float) -> None:
    # Madgwick algorithm without a magnometer, using gradient decent
    global quaternion

    # Convert gyro values to radians
    gx = math.radians(gx)
    gy = math.radians(gy)
    gz = math.radians(gz)

    norm =math.sqrt(ax *ax+ay*ay+az*az) #normalize acc
    if norm == 0: return

    ax /= norm
    ay /= norm
    az /= norm

    q1, q2, q3, q4 = quaternion  # w,x,y,z

    #correction step
    F = [2 * (q2 * q4 - q1 * q3) - ax,
        2 * (q1 * q2 + q3 * q4) - ay,
        2 * (0.5 - q2 * q2 - q3 * q3) - az]
    

    J = [ [-2 * q3, 2 * q4, -2 * q1, 2 * q2],
        [2 * q2, 2 * q1, 2 * q4, 2 * q3],
        [0, -4 * q2, -4 * q3, 0]]

    
    gradient = [0, 0, 0, 0]
    for i in range(3):
        for j in range(4):
            gradient[j] += J[i][j] * F[i]

    # Normalizing the gradient
    norm = math.sqrt(sum(g*g for g in gradient))
    if norm == 0: return

    for i in range(4):
        gradient[i] /= norm

    # quar rate of change
    qDot = [
        -0.5 * (q2 * gx + q3 * gy + q4 * gz),
        0.5 * (q1 * gx + q3 * gz - q4 * gy),
        0.5 * (q1 * gy - q2 * gz + q4 * gx),
        0.5 * (q1 * gz + q2 * gy - q3 * gx)
    ]

    # Apply feedback from acc
    for i in range(4):
        qDot[i] -= madgwick_beta * gradient[i]

    #integrate to get new quat
    for i in range(4):
        quaternion[i] += qDot[i] * dt

    norm = math.sqrt(sum(q*q for q in quaternion))   # Normalize quat
    quaternion /= norm

def mahony_update(ax: float, ay: float, az: float, gx: float, gy: float, gz: float, dt: float) -> None:
    global quaternion, integral_error

    gx = math.radians(gx)
    gy = math.radians(gy)
    gz = math.radians(gz)


    # Normalize acc
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0: return

    ax /= norm
    ay /= norm
    az /= norm
    q1, q2, q3, q4= quaternion 

    # Estimated direction of gravity from quaternion
    vx = 2 * (q2 * q4 - q1 * q3)
    vy = 2 * (q1 * q2 + q3 * q4)
    vz = q1 * q1 - q2 * q2 - q3 *q3 +q4 * q4

    # the cross product of the estimated and measured direction is the error
    ex = ay * vz - az * vy
    ey = az *vx - ax * vz
    ez = ax * vy - ay * vx

    # Integral error multiplied by Ki
    if mahony_ki>0:
        integral_error[0] += ex * dt * mahony_ki
        integral_error[1] += ey * dt * mahony_ki
        integral_error[2] += ez * dt * mahony_ki
    else:
        integral_error = np.array([0.0, 0.0, 0.0])


    # Apply feedback terms
    gx += mahony_kp * ex + integral_error[0]
    gy += mahony_kp * ey + integral_error[1]
    gz += mahony_kp * ez + integral_error[2]

    # Integrate rate of change of quat
    gx *= 0.5
    gy *= 0.5
    gz *= 0.5

    qa = q1
    qb = q2
    qc = q3
    qd = q4

    q1 += (-qb * gx - qc * gy - qd * gz) * dt
    q2 += (qa * gx + qc * gz - qd * gy) * dt
    q3 += (qa * gy - qb * gz + qd * gx) * dt
    q4 += (qa * gz + qb * gy - qc * gx) * dt
    # Normalize quaternion
    quaternion[0] = q1
    quaternion[1] = q2
    quaternion[2] = q3
    quaternion[3] = q4

    norm = math.sqrt(sum(q*q for q in quaternion))
    quaternion /= norm


def update_heading() -> float:
    global current_heading, last_time, quaternion

    # Read sensor data
    accel_x, accel_y, accel_z = read_accel_data()
    gyro_x, gyro_y, gyro_z = read_gyro_data()

    if gyro_z is None or (FILTER_TYPE != "SIMPLE" and accel_z is None):
        return current_heading

    current_time = time.time()

    # Initialize if first reading
    if last_time is None:
        last_time = current_time
        return current_heading
    # time diff
    dt = current_time - last_time

    # Update heading
    
    if FILTER_TYPE == "MADGWICK":
        madgwick_update(accel_x, accel_y, accel_z, gyro_x,gyro_y, gyro_z, dt)
        _, _, yaw = quaternion_to_euler(quaternion)
        current_heading = yaw

    elif FILTER_TYPE == "MAHONY":
        mahony_update(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt)
        _, _, yaw = quaternion_to_euler(quaternion)
        current_heading = yaw


    # Update last time

    last_time = current_time

    return current_heading

def reset_heading() -> bool:
    global current_heading, quaternion

    if FILTER_TYPE in ["MADGWICK", "MAHONY"]:
        # Get the current roll and pitch
        roll, pitch, _ = quaternion_to_euler(quaternion)

        # Reset only the yaw component
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)

        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)


        # Set yaw to zero , respectively w,x,y,z,
        quaternion[0] = cr * cp  
        quaternion[1] = sr * cp  
        quaternion[2] = cr * sp  
        quaternion[3] = 0        

        # Normalize
        norm = math.sqrt(sum(q*q for q in quaternion))
        quaternion /= norm
    else:
        current_heading = 0.0

    print("Heading reset to 0.0°")
    return True

def heading_thread_function() -> None:
    global running

    while running:
        update_heading()
        time.sleep(0.01)  


def start_heading_tracker() -> bool:
    global running, heading_thread, calibration_flag

    if not initialized:
        if not init_mpu6500():
            print("Failed to initialize MPU6500")
            return False
    
    if calibration_flag is False:
        calibrate_gyroscope()
        calibration_flag = True

    #update thread

    running = True
    heading_thread = threading.Thread(target=heading_thread_function)
    heading_thread.daemon = True
    heading_thread.start()

    time.sleep(1)

    return True

def stop_heading_tracker() -> bool:
    global running
    running = False
    if heading_thread:
        heading_thread.join(timeout=1.0)

    return True

def get_heading() -> float:
    return current_heading
