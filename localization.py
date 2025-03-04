import sys
import numpy as np
from pymavlink import mavutil
import time
import threading
import os
import math
from connect import connect_to_px4, get_gps_data
from dronekit import connect, VehicleMode, LocationGlobalRelative
# Connecting to PX4
connection_gps = connect_to_px4('udp:127.0.0.1:14550')

CHANNEL_9_THRESHOLD = 1500 
RADIUS_EARTH = 6378  # Mean radius of the Earth in km
RADIUS_EARTH_METERS = RADIUS_EARTH * 1000 
terminal_opened = False
lock = threading.Lock()
threading.daemon = True

#GPS
def get_gps_data(connection):
    connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1)
    msg = connection.recv_match(type='GPS_RAW_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7  # Convert raw GPS data to degrees
        lon = msg.lon / 1e7
        return lat, lon
    else:
        return None, None
    
#FOV - (OPTIONAL)
def calculate_fov(sensor_width, sensor_height, focal_length_x, focal_length_y):
    fov_x = 2 * math.atan((sensor_width / 2) / focal_length_x)
    fov_y = 2 * math.atan((sensor_height / 2) / focal_length_y)
    return fov_x, fov_y

#INCLINED DISTANCE CALC
def calculate_inclined_distance(altitude, pitch_angle):
    alpha = 90 - pitch_angle  # Inclined angle (alpha) is 90 - pitch angle
    alpha_rad = math.radians(alpha)
    Y = altitude / math.sin(alpha_rad)
    return Y
#HORIZONTAL DISTANCE CALCULATION
def calculate_horizontal_distance(altitude, Y):
    # Calculate X using the Pythagorean theorem: X = sqrt(Y^2 - altitude^2)
    X = math.sqrt(Y**2 - altitude**2)
    return X

#MAIN
def main():


    # 2. FOR CONNECTING THE DRONE AND GETTING THE LIVE GPS LAT LON OF THE DRONE + GETTING YAW and ALT
    connection_string = 'udp:127.0.0.1:14550'
    px4_connection = connect_to_px4(connection_string)
    vehicle = connect('127.0.0.1:14569', wait_ready=True)
    
    
    if px4_connection:
        # Get the current GPS position of the drone once
        latitude, longitude = get_gps_data(px4_connection)
        if latitude is not None and longitude is not None:
            print(f"Current GPS Coordinates: Latitude = {latitude}, Longitude = {longitude}")


        # 3. FOV EXTRACTION
        '''focal_length_x = 2.1
        focal_length_y = 2.1
        sensor_width = 3.2  # Example value (in mm)
        sensor_height = 2.6
        fov_x, fov_y = calculate_fov(sensor_width, sensor_height, focal_length_x, focal_length_y)
        print(f"Camera FOV: Horizontal = {math.degrees(fov_x):.2f}°, Vertical = {math.degrees(fov_y):.2f}°")'''

        # 4. DISTANCE CALC Y and X
        #gimbal = vehicle.gimbal
        #pitch_angle = gimbal.pitch
        #print(f" Pitch: {pitch_angle}°")
        pitch_angle = 45 
        print(f" Pitch: {pitch_angle}°")
        altitude = 110
        #altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude}")
        Y = calculate_inclined_distance(altitude, pitch_angle)
        print(f"Inclined Distance (Y) : {Y:.2f} meters")
        X = calculate_horizontal_distance(altitude, Y)
        print(f"Horizontal Distance (X): {X:.2f} meters")

        #5. CONVERSION
        yaw = vehicle.heading
        print(f"Yaw:{yaw}")
        uav_lat_rad = math.radians(latitude)
        uav_lon_rad = math.radians(longitude)

        delta_e = X * math.sin(math.radians(yaw))  # Eastward displacement
        delta_n = X * math.cos(math.radians(yaw))  # Northward displacement  

    # Calculate the target's latitude and longitude from the displacement
        delta_lat = delta_n / RADIUS_EARTH_METERS  # Change in latitude (in radians)
        delta_lon = delta_e / (RADIUS_EARTH_METERS * math.cos(uav_lat_rad))  # Change in longitude (in radians)

    # Calculate target latitude and longitude in radians
        target_lat_rad = uav_lat_rad + delta_lat
        target_lon_rad = uav_lon_rad + delta_lon

    # Convert the target's latitude and longitude back to degrees
        target_lat = math.degrees(target_lat_rad)
        target_lon = math.degrees(target_lon_rad)

        #target_lat, target_lon, target_alt = calculate_target_lat_lon(latitude, longitude, altitude, Y, X, yaw, pitch_angle)
        print(f"Target Latitude: {target_lat:.6f}")
        print(f"Target Longitude: {target_lon:.6f}")

        #6. THREADING
        global terminal_opened
        with lock:
            if terminal_opened:
                return
            terminal_opened = True
            time.sleep(1)
            command = f'lxterminal -e "python3 old_pixhawk_KS.py {target_lat} {target_lon} {altitude}; bash"'
            os.system(command)
            return

        
        

if __name__ == '__main__':
    main()
