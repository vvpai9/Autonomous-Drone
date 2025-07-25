import asyncio
from mavsdk import System
from pymavlink import mavutil

class Drone:
    def __init__(self):
        self.drone = System()
        self.master = None

    async def connect(self, mavsdk_address="udp://:14540", pymavlink_address="udp:127.0.0.1:14550"):
        print(f"Connecting to drone on {pymavlink_address}...")
        await self.drone.connect(system_address=mavsdk_address)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
        self.master = mavutil.mavlink_connection(pymavlink_address)  
        self.master.wait_heartbeat()
        print("Connected to drone.")
        await asyncio.sleep(2)

    async def set_mode(self, mode='GUIDED'):
        """Set flight mode using direct MAVLink command"""
        # Common ArduPilot modes
        modes = {
            'STABILIZE': 0,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'CIRCLE': 7,
            'LAND': 9,
            'BRAKE': 17,
            'THROW': 18,
            'GUIDED_NOGPS': 20,
            'SMART_RTL': 21
        }
        
        if mode in modes:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                modes[mode]
            )
            print(f"Mode set to {mode}")
            await asyncio.sleep(1)
        else:
            print(f"Unknown mode: {mode}")

    async def get_position(self):
        """Get current GPS position"""
        async for position in self.drone.telemetry.position():
            return {
                'latitude': position.latitude_deg,
                'longitude': position.longitude_deg,
                'altitude_rel': position.relative_altitude_m   # Above takeoff point
            }

    async def get_altitude(self):
        """Get current altitude"""
        async for position in self.drone.telemetry.position():
            return position.relative_altitude_m

    async def arm(self):
        await self.drone.action.arm()
        await asyncio.sleep(2)

    async def disarm(self):
        await self.drone.action.disarm()
        await asyncio.sleep(2)

    async def takeoff(self, altitude=15.0):
        await self.set_mode('GUIDED')
        await self.drone.action.set_takeoff_altitude(altitude)
        print(f"Taking off to {altitude} meters...")
        await self.drone.action.takeoff()
        while True:
            alt = await self.get_altitude()
            print(f"Current altitude: {alt} m")
            if alt >= altitude * 0.95:
                print("Reached target altitude.")
                break
            await asyncio.sleep(1)

    async def land(self):
        await self.drone.action.land()
        print("Landing...")
        while True:
            alt = await self.get_altitude()
            print(f"Current altitude: {alt} m")
            if alt <= 0.1:
                print("Landed safely.")
                break
            await asyncio.sleep(1)

    async def go_to_location(self, lat, lon, alt, yaw=0.0):
        await self.drone.action.goto_location(lat, lon, alt, yaw)

    def send_ned_velocity(self, velocity_x=0, velocity_y=0, velocity_z=0, duration=0):
        for _ in range(duration):
            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                0b10111000111,
                0, 0, 0,
                velocity_x, velocity_y, velocity_z,
                0, 0, 0,
                0, 0
            )
            asyncio.run(asyncio.sleep(1))

    def drop_payload(self, servo_channel=9, pwm_value=1000):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_channel,
            pwm_value,
            0, 0, 0, 0, 0
        )