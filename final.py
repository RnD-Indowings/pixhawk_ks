import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from math import radians, cos, sin, sqrt, atan2, asin, degrees
import threading
from localization import main
import os
import sys
import time

# Global variables for midpoints
midpoint_lat = None
midpoint_lon = None

# Function to calculate the new coordinates after moving a certain distance from the target
def calculate_new_position_from_target(lat1, lon1, lat, lon, distance_meters):
    earth_radius = 6371000  # Radius of the Earth in meters

    # Convert latitude and longitude from degrees to radians
    lat1_rad = radians(lat1)
    lon1_rad = radians(lon1)
    target_lat_rad = radians(lat)
    target_lon_rad = radians(lon)

    # Difference in coordinates (target to current)
    delta_lat = target_lat_rad - lat1_rad
    delta_lon = target_lon_rad - lon1_rad

    # Haversine formula to calculate the distance between two points on the Earth
    a = sin(delta_lat / 2) ** 2 + cos(lat1_rad) * cos(target_lat_rad) * sin(delta_lon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = earth_radius * c  # Distance in meters

    # the bearing (direction) from the current position to the target
    y = sin(delta_lon) * cos(target_lat_rad)
    x = cos(lat1_rad) * sin(target_lat_rad) - sin(lat1_rad) * cos(target_lat_rad) * cos(delta_lon)
    bearing = atan2(y, x)

    # Calculate the new position by moving the specified distance in the forward direction from the target
    new_distance = distance_meters / earth_radius  # Convert distance to radians
    new_lat_rad = asin(sin(target_lat_rad) * cos(new_distance) +
                       cos(target_lat_rad) * sin(new_distance) * cos(bearing))
    new_lon_rad = target_lon_rad + atan2(sin(bearing) * sin(new_distance) * cos(target_lat_rad),
                                         cos(new_distance) - sin(target_lat_rad) * sin(new_lat_rad))

    # Convert the new latitude and longitude back to degrees
    new_lat = degrees(new_lat_rad)
    new_lon = degrees(new_lon_rad)

    return new_lat, new_lon

async def print_current_speed(drone, file):
    async for velocity_ned in drone.telemetry.velocity_ned():
        speed = sqrt(velocity_ned.north_m_s**2 + velocity_ned.east_m_s**2)
        file.write(f"Current speed: {speed:.2f} m/s\n")

async def run():
    if len(sys.argv) != 4:
        print("Usage: python localization.py <target_lat> <target_lon> <altitude>")
        sys.exit(1)

    # Access the command-line arguments
    lat = float(sys.argv[1])  # Argument 1
    lon = float(sys.argv[2])  # Argument 2  
    altitude = float(sys.argv[3])
    print(f"Received Coordinates: ({lat} {lon} {altitude} )")

    # Open the drone.txt file in append mode
    with open('drone.txt', 'a') as file:

        drone = System()
        await drone.connect(system_address="serial:///dev/ttyACM0:1152000")
        speed_task = asyncio.ensure_future(print_current_speed(drone, file))

        print("-- Waiting for drone to reach sufficient altitude")
        async for position in drone.telemetry.position():
            if position.relative_altitude_m > 0:  # Adjust based on your required altitude
                file.write("-- Drone reached desired altitude\n")
                break

        await asyncio.sleep(10)  # Additional stabilization after reaching altitude

        # Target locations
        target_altitude = -altitude -5  # in meters

        # Get current position
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg

        # Calculate the new position (100 meters forward from the target)
        new_lat, new_lon = calculate_new_position_from_target(current_lat, current_lon, lat, lon, 100)
        file.write(f"New coordinates (100 meters forward from target): Latitude = {new_lat}, Longitude = {new_lon}\n")

        # Mission items
        mission_items = [
            MissionItem(new_lat, new_lon, target_altitude, 40, True, float('nan'), float('nan'),
                        MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'),
                        float('nan'), MissionItem.VehicleAction.NONE),
        ]

        mission_plan = MissionPlan(mission_items)

        file.write("-- Uploading mission\n")
        await drone.mission.upload_mission(mission_plan)

        file.write("-- Starting mission\n")
        await drone.mission.start_mission()
        file.write("-- Mission started\n")

        await drone.param.set_param_float("MPC_XY_CRUISE", 50.0)  # Desired cruising speed
        await drone.param.set_param_float("MPC_XY_VEL_MAX", 50.0)  # Maximum XY velocity

        file.write("-- Printing distance to target and altitude in real-time\n")
        distance_task = asyncio.ensure_future(print_distance_to_target(drone, lat, lon, file))

        print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone, file))
        running_tasks = [print_mission_progress_task, distance_task, speed_task]
        termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks, file))

        await termination_task

async def print_distance_to_target(drone, lat, lon, file):
    """
    Continuously print the distance to the target and current altitude.
    """
    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        current_alt = position.relative_altitude_m  # Altitude relative to takeoff point

        # Calculate distance to target
        distance = haversine(current_lat, current_lon, lat, lon)

        file.write(f"Distance to target: {distance:.2f} meters | Altitude: {current_alt:.2f} meters\n")

async def print_mission_progress(drone, file):
    async for mission_progress in drone.mission.mission_progress():
        file.write(f"Mission progress: {mission_progress.current}/{mission_progress.total}\n")

async def observe_is_in_air(drone, running_tasks, file):
    was_in_air = False
    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return

if __name__ == "__main__":
    asyncio.run(run())
