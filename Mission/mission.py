import asyncio
import cv2
from basic import Drone
from path import *
from detection import detectionClassify
from align import align_to_target
from log import *

# ========== Geofence Parameters ==========
geofence = [
    (15.3677317, 75.1253137),  # Bottom-Left
    (15.3673024, 75.1252842),  # Bottom-Right
    (15.3672791, 75.1259252),   # Top-Right 
    (15.3676723, 75.1259440),  # Top-Left
] # Change values as required

x_divisions = 5
y_divisions = 4
altitude = 15  # in meters
mavsdk_address = "udp://:14540"
mode = 'Herelink'
drone = None
address = {'Herelink': 'udpout:192.168.0.1:14550', 
           'Telemetry': 'COM3',
           'Onboard_USB': '/dev/ttyUSB0',
           'Onboard_UART': '/dev/ttyAMA0',
           'Onboard_Serial': '/dev/ttySerial0'}
camera = "udp://@:1234"

async def execute_mission():
    try:
        if mode not in address:
            raise ValueError(f"Invalid mode: {mode}. Available modes: {list(address.keys())}")
        pymavlink_address = address[mode]
        drone = Drone()
        await drone.connect(mavsdk_address, pymavlink_address)
        await drone.arm()
        await drone.takeoff(altitude)

        wp = generate_waypoints(geofence, x_divisions, y_divisions)

        await detectionClassify(camera, drone)

        boustrophedon_path(drone, wp, altitude)
        await asyncio.sleep(1)

        from align import target
        if target:
            print("Moving to target...")
            await drone.go_to_location(target['latitude'], target['longitude'], altitude)
            await align_to_target(drone)
            await asyncio.sleep(2)
            print("Mission completed successfully.")
        else:
            print("No target detected.")    
    except KeyboardInterrupt:
        logging.error(f"Mission interrupted by user.")
    except Exception as e:
        logging.error(f"Mission execution failed: {e}")
    finally:
        if drone:
            await drone.set_mode('RTL')
            print("Returning to Launch...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(execute_mission())