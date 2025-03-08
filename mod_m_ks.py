import time
import argparse
import threading
from pymavlink import mavutil

# Parse command-line arguments for target coordinates
parser = argparse.ArgumentParser(description="Parse target latitude, longitude")
parser.add_argument("target_lat", type=float, help="Target latitude")
parser.add_argument("target_lon", type=float, help="Target longitude")
args = parser.parse_args()

# Target coordinates
target_lat = args.target_lat
target_lon = args.target_lon
target_alt = 2  # Set a low altitude for waypoint navigation

# Connection string
connection_string = "127.0.0.1:14550"

# Connect to the drone
def connect_drone():
    connection = mavutil.mavlink_connection(connection_string)
    connection.wait_heartbeat()
    print("Connected to the drone.")
    return connection

# Set GUIDED mode
def set_guided_mode(connection):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        1, 4, 0, 0, 0, 0, 0
    )
    print("Setting GUIDED mode...")
    time.sleep(2)

# Arm the drone
def arm_drone(connection):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming the drone...")

    while not connection.recv_match(type='HEARTBEAT', blocking=True).base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        time.sleep(1)
    print("Drone armed.")

# Takeoff and wait until altitude is reached
def takeoff_to_altitude(connection, altitude):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )
    print(f"Taking off to {altitude} meters.")

    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # Convert mm to meters
            print(f"Current altitude: {current_alt:.2f}m")

            if current_alt >= altitude * 0.95:  # Allow slight tolerance
                print("Takeoff complete.")
                break
        time.sleep(1)

# Change drone speed
def change_speed(connection, speed):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
        0, speed, -1, 0, 0, 0, 0
    )
    print(f"Changing speed to {speed} m/s.")

# Print live speed in a separate thread
def print_live_speed(connection, log_file="mission_log_full_speed.txt"):
    with open(log_file, "w") as file:
        while True:
            msg = connection.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            if msg:
                log_entry = f"Current Speed: {msg.groundspeed:.2f} m/s\n"
                print(log_entry.strip())  # Print to console
                file.write(log_entry)  # Write to file
                file.flush()  # Force flush to ensure data is written
            time.sleep(0.5)


# Navigate to target waypoint
def go_to_waypoint(connection, lat, lon, alt):
    connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10, connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000,  # Ignore velocities and acceleration
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0,  # No velocity set
            0, 0, 0,  # No acceleration
            0, 0  # No yaw
        )
    )
    print(f"Navigating to Target: Lat {lat}, Lon {lon}, Alt {alt}m")

# Monitor if the drone has reached the target
def wait_until_reached(connection, lat, lon, threshold=5, log_file="mission_log.txt"):
    with open(log_file, "w") as file:
        while True:
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                distance = ((current_lat - lat) ** 2 + (current_lon - lon) ** 2) ** 0.5 * 111139  # Approx. meters
                print(f"Current Position: ({current_lat:.6f}, {current_lon:.6f}) | Distance to Target: {distance:.2f}m  ")
                log_entry = f"Current Position: ({current_lat:.6f}, {current_lon:.6f}) | Distance to Target: {distance:.2f}m \n"
                file.write(log_entry)
                
                if distance <= threshold:
                    print("Target reached.")
                    file.write("Target reached.\n")
                    break
                
            time.sleep(1)
            #file.write(f"{current_lat:.6f}, {current_lon:.6f} | Distance to Target: {distance:.2f}m | Speed: {msg.groundspeed:.2f} m/s\n")
# Main execution
def main():
    print(f"Received Coordinates: ({target_lat}, {target_lon})")

    connection = connect_drone()
    
    # Start live speed monitoring
    speed_thread = threading.Thread(target=print_live_speed, args=(connection,), daemon=True)
    speed_thread.start()

    set_guided_mode(connection)
    arm_drone(connection)
    takeoff_to_altitude(connection, 40)  # Takeoff to 40m
    change_speed(connection, 17)  # Set speed to 17 m/s
    go_to_waypoint(connection, target_lat, target_lon, target_alt)  # Navigate to target
    wait_until_reached(connection, target_lat, target_lon)  # Wait for arrival

    print("Mission complete.")

if __name__ == "__main__":
    main()
