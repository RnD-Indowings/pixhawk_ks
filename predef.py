import math
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np
from pymavlink import mavutil
from math import radians, cos, sin, sqrt, atan2, asin, degrees
import time

vehicle = connect('127.0.0.1:14569', wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    
    while not vehicle.is_armable:
        print(" Waiting for vehicle to become armable...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude}")
        if altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def set_velocity_body(vx, vy, vz, yaw_rate=0.1):
    print(f"Setting velocity - vx: {vx}, vy: {vy}, vz: {vz}, yaw_rate: {yaw_rate}")
    print(vehicle.location.global_relative_frame.alt)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_NED,  
        0b0000111111000111, 
        0, 0, 0,  
        vx, vy, vz, 
        0, 0, 0, 
        0, yaw_rate  #
    )
    vehicle.send_mavlink(msg)
    

def get_distance_meters(target_location):
    """
    Get the 3D distance (including altitude difference) from the current location to the target location.
    """
    current_location = vehicle.location.global_relative_frame
    dlat = target_location.lat - current_location.lat
    dlon = target_location.lon - current_location.lon
    dalt = target_location.alt - current_location.alt
    horizontal_distance = math.sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5
    distance = math.sqrt(horizontal_distance**2 + dalt**2)
    return distance

def calculate_yaw_to_target(target_location):
    current_location = vehicle.location.global_relative_frame
    dlat = target_location.lat - current_location.lat
    dlon = target_location.lon - current_location.lon
    yaw = math.atan2(dlon, dlat) * (180 / math.pi)  # Convert to degrees
    return yaw

def get_current_yaw():
    """
    Get the current yaw angle of the UAV.
    """
    return vehicle.heading

def condition_yaw(degree, relative=True):
    """
    Rotate the UAV to a specific yaw angle.
    """
    if relative:
        is_relative = 1  
    else:
        is_relative = 0 
    print(f"Adjusting yaw by {degree} degrees")
    
    if degree >= 0:
        direction = 1  # clockwise
    else:
        direction = -1  # counterclockwise
        degree = abs(degree)

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        degree,  # Yaw in degrees
        0,  # Yaw speed (deg/s)
        direction,  # Direction: 1 cw, -1 ccw
        is_relative,  # Relative or absolute
        0, 0, 0
    )
    vehicle.send_mavlink(msg)

def align_heading_to_target(target_location):
    """
    Align UAV's yaw to face the target location.
    """
    current_yaw = get_current_yaw() 
    target_yaw = calculate_yaw_to_target(target_location)
    
    relative_yaw = target_yaw - current_yaw
    if relative_yaw > 180:
        relative_yaw -= 360
    elif relative_yaw < -180:
        relative_yaw += 360
    
    print(f"Current yaw: {current_yaw}, Target yaw: {target_yaw}, Adjusting yaw by: {relative_yaw}")
    condition_yaw(relative_yaw)

def adjust_course_to_target(target_location):
    """
    Moves the drone towards the target at full speed (20 m/s) while dynamically adjusting altitude
    to ensure impact exactly at the target location.
    """
    distance = get_distance_meters(target_location)

    vx = 15  # Maintain full speed
    current_altitude = vehicle.location.global_relative_frame.alt
    target_altitude = target_location.alt

    # Calculate time to impact at full speed
    time_to_target = distance / vx  

    if time_to_target > 0:  
        vz = (target_altitude - current_altitude) / time_to_target  # Adjust altitude smoothly
    else:
        vz = 0  # Avoid division by zero
    
    print(f"Adjusting course - vx: {vx}, vz: {vz}, distance: {distance}")
    set_velocity_body(vx, 0, -vz)  

 

def trigger_final_mechanism():
    """
    Simulate triggering the final mechanism (e.g., kinetic strike).
    """
    print("Final strike initiated!")

try:
    arm_and_takeoff(100) 
    target_location = LocationGlobalRelative(-35.36216065, 149.16122379, 1)  # Example target
    
    align_heading_to_target(target_location)  # Align the UAV to face the target
    time.sleep(5)
    
    while True:
        distance_to_target = get_distance_meters(target_location)
        altitude_error = abs(vehicle.location.global_relative_frame.alt - target_location.alt)

        print(f"Distance: {distance_to_target}, Altitude Error: {altitude_error}")

        if distance_to_target < 0.5 and altitude_error < 0.5:  # Ensure precision before striking
            trigger_final_mechanism()
            break
        
        adjust_course_to_target(target_location)

        

finally:
    print("Closing vehicle connection")
    vehicle.close()
