import time
import math
from pymavlink import mavutil

# Connect to the drone (Replace with your connection string)
connection_string = "udp:127.0.0.1:14550"  # SITL Example
# connection_string = "/dev/serial0"  # For real hardware (Raspberry Pi or Pixhawk)

# Start MAVLink connection
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print("Connected to the drone.")

# Target speed in m/s
TARGET_SPEED = 15.0  

def set_guided_mode():
    """ Puts the drone in GUIDED mode """
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode
    )
    print("Mode set to GUIDED")

def arm_and_takeoff(altitude=10):
    """ Arms the drone and makes it take off """
    print("Arming drone...")
    master.arducopter_arm()
    time.sleep(2)
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    print(f"Taking off to {altitude}m")
    time.sleep(5)

def get_current_position():
    """ Retrieves the current GPS position """
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    lat = msg.lat / 1e7  # Convert to decimal degrees
    lon = msg.lon / 1e7
    return lat, lon

def send_velocity_command(vx, vy, vz=0):
    """ Sends velocity command in the LOCAL_NED frame """
    master.mav.set_position_target_local_ned_send(
        0,  # Timestamp (not used)
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # Mask to ignore position, use only velocity
        0, 0, 0,  # Position (ignored)
        vx, vy, vz,  # Velocity (m/s)
        0, 0, 0,  # Acceleration (ignored)
        0, 0  # Yaw (ignored)
    )

def move_to_waypoint(target_lat, target_lon):
    """ Moves the drone towards the waypoint at 15m/s """
    while True:
        # Get current position
        current_lat, current_lon = get_current_position()

        # Compute direction
        dx = target_lat - current_lat
        dy = target_lon - current_lon
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.00001:  # Threshold to stop when close
            print("Reached the waypoint.")
            send_velocity_command(0, 0)  # Stop movement
            break

        # Normalize direction and apply speed
        velocity_x = (dx / distance) * TARGET_SPEED
        velocity_y = (dy / distance) * TARGET_SPEED

        send_velocity_command(velocity_x, velocity_y)
        time.sleep(0.2)  # Small delay to allow continuous movement

if __name__ == "__main__":
    set_guided_mode()
    arm_and_takeoff(altitude=10)

    target_lat = 37.7749  # Example waypoint latitude
    target_lon = -122.4194  # Example waypoint longitude

    move_to_waypoint(target_lat, target_lon)
