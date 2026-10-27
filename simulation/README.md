# Simulation

Everything in this repo talks to PX4 over MAVSDK at `udpin://0.0.0.0:14550`
(`CONNECTION_URL` in `src/utils/config.py`). That is the address PX4 SITL uses
when it is started with the default MAVLink endpoint, so simulation and real
hardware differ only in where the camera frames come from and which autopilot
you are connected to.

## SITL

```bash
# Terminal 1 - PX4 with the Gazebo classic airframe (adjust to your install)
cd PX4-Autopilot
make px4_sitl gazebo-classic

# Terminal 2 - inspect the connection before running the pipeline
python -m mavsdk.tools.mavsdk_shell   # or QGroundControl on the same port
```

Once the EKF reports a position estimate, the pipeline can start:

```bash
python -m src.main
```

The pipeline does not know whether it is in SITL or on hardware. It waits for
`is_global_position_ok`, arms, takes off to `SAFE_ALTITUDE`, waits 10 s, then
starts offboard control.

## Camera source in SITL

Two options, both configured through `VIDEO_SOURCE`:

1. A virtual camera device fed by the simulator (v4l2loopback on Linux, OBS
   virtual camera on Windows) so the pixel path is identical to the aircraft.
2. A recorded clip, by constructing `Camera(source="path/to/clip.mp4")` instead
   of using the integer index. Useful for repeatable perception tests, but the
   MAVSDK side still needs a live SITL instance.

## What SITL does not tell you

- The depth threshold in `src/utils/config.py` was calibrated on some recording set
  that is not part of this repository. Simulated textures do not have the same depth
  distribution, and the map is normalised per frame, so the same threshold can mean
  something different.
- Timing is optimistic. SITL runs on a workstation; the flight computer usually
  does not.
- Offboard failsafe behaviour differs from a real failsafe, because there is no
  physical airframe to lose.

Use SITL to check wiring, frame conventions, setpoint signs and the takeoff to
landing sequence. Use flight logs for anything about control quality.
