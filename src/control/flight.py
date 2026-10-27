import asyncio

from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed

from src.utils import config


class FlightController:
    """Thin async wrapper over the MAVSDK calls the loop needs."""

    def __init__(self, connection_url=config.CONNECTION_URL,
                 takeoff_altitude=config.SAFE_ALTITUDE):
        self.connection_url = connection_url
        self.takeoff_altitude = takeoff_altitude
        self.drone = System()
        self.offboard_active = False

    async def connect(self):
        await self.drone.connect(system_address=self.connection_url)
        print("Waiting for drone connection...")

        # Blocks until the EKF reports a usable global position. No timeout.
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok:
                break

        print("Position estimate OK")

    async def takeoff(self):
        print(f"Launching to {self.takeoff_altitude} m")
        await self.drone.action.set_takeoff_altitude(self.takeoff_altitude)
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await asyncio.sleep(config.TAKEOFF_SETTLE_TIME)

    async def start_offboard(self):
        # PX4 wants a setpoint before offboard will start.
        await self.set_velocity(0.0, 0.0, 0.0)
        await self.drone.offboard.start()
        self.offboard_active = True

    async def set_velocity(self, vx, vy, yaw_rate):
        """Body-frame velocity setpoint in m/s, yaw rate in deg/s. Vertical is held at zero."""
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, 0.0, yaw_rate)
        )

    async def velocity_ned(self):
        """One fresh NED velocity sample; the telemetry stream is opened per call."""
        async for velocity in self.drone.telemetry.velocity_ned():
            return (velocity.north_m_s, velocity.east_m_s, velocity.down_m_s)
        return (0.0, 0.0, 0.0)

    async def stop_offboard(self):
        if not self.offboard_active:
            return
        try:
            await self.drone.offboard.stop()
        except Exception as error:
            print(f"Offboard stop failed: {error}")
        finally:
            self.offboard_active = False

    async def land(self):
        await self.drone.action.land()
