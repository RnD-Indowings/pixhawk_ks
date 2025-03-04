
from pymavlink import mavutil

# Connect to Pixhawk
def connect_to_px4(connection_string='serial:///dev/tty2:921600'):
    """
    Establishes a connection to the Pixhawk autopilot using MAVLink.

    Args:
        connection_string (str): MAVLink connection string for the Pixhawk. 
                                 Example: 'serial:/dev/ttyUSB0:57600' for USB, 
                                          'udp:127.0.0.1:14550' for SITL.

    Returns:
        mavutil.mavlink_connection: A MAVLink connection object or None if connection fails.
    """
    try:
        print("Connecting to PX4...")
        connection = mavutil.mavlink_connection(connection_string)
        connection.wait_heartbeat()  # Wait for a heartbeat to confirm connection
        print("Connection established! Heartbeat received from PX4.")
        return connection
    except Exception as e:
        print(f"Error: Could not connect to PX4. {e}")
        return None

# Fetch GPS data from Pixhawk
def get_gps_data(connection_gps):
    """
    Retrieves the current GPS latitude and longitude from Pixhawk.

    Args:
        connection (mavutil.mavlink_connection): A valid MAVLink connection to Pixhawk.

    Returns:
        tuple: A tuple containing latitude and longitude in decimal degrees (lat, lon).
               Returns (None, None) if GPS data is unavailable.
    """
    try:
        # Wait for the GLOBAL_POSITION_INT message (contains GPS data)
        msg = connection_gps.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            # Convert lat/lon from integer to decimal degrees
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            return lat, lon
        else:
            print("No GPS data received.")
    except Exception as e:
        print(f"Error retrieving GPS data: {e}")
    return None, None

# Main function to demonstrate connection and GPS data retrieval
if __name__ == "__main__":
    # Replace with your connection string
    connection_string = '127.0.0.1:14550'

    # Connect to Pixhawk
    pixhawk_connection = connect_to_px4(connection_string)

    if pixhawk_connection:
        while True:
            # Fetch GPS coordinates
            latitude, longitude = get_gps_data(pixhawk_connection)
            if latitude is not None and longitude is not None:
                print(f"Current GPS Coordinates: Latitude = {latitude}, Longitude = {longitude}")
            else:
                print("Unable to retrieve GPS coordinates.")