# Flying the simulator's physics with real ArduPilot (SITL)

The browser simulator flies a built-in *approximation* of ArduPlane's QuadPlane
controller. For the most faithful practice, you can let the **real ArduPilot**
firmware fly the same physics model. ArduPilot SITL has a "JSON" backend: it
sends servo PWM values over UDP and an external program returns the vehicle state.
`tools/sitl-bridge.mjs` is that program.

> Status: the bridge is tested against a mock ArduPilot client (`test/sitl.test.mjs`),
> not yet against a live ArduPilot build. Treat this as the starting point for Phase 2.

## 1. Install ArduPilot SITL (Windows 10 → WSL2)

1. Install WSL2 with Ubuntu (`wsl --install -d Ubuntu` in an admin PowerShell, then reboot).
2. In Ubuntu:
   ```bash
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   Tools/environment_install/install-prereqs-ubuntu.sh -y
   . ~/.profile
   ```
3. Install Node.js 18+ in Ubuntu and copy or clone this repository there.

## 2. Run

Terminal 1 (this repository):
```bash
node tools/sitl-bridge.mjs --wind 8 --wind-from 0 --gust 2 --turbulence light
```

Terminal 2 (ardupilot):
```bash
Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON:127.0.0.1 \
  --add-param-file=/path/to/ClaudeFlightSim/sitl/quadplane-rescue.parm --console --map
```

The bridge resets its physics when ArduPilot restarts and resends a reply when
ArduPilot repeats a frame.

## 3. Use your radio

With the RadioMaster Pocket in **USB Joystick (HID)** mode it is an ordinary
Windows game controller. Connect Mission Planner to the SITL vehicle (UDP 14550;
from WSL2 you may need MAVProxy `--out` to the Windows host IP) and enable its
joystick feature to send RC overrides. Map the flight-mode and arm switches the
same way as on the real aircraft.

## Channel map

| Servo | Function | ArduPilot SERVOx_FUNCTION |
|---|---|---|
| 1 | aileron (both, opposite) | 4 |
| 2 | V-tail left | 79 |
| 3 | pusher throttle | 70 |
| 4 | V-tail right | 80 |
| 5–8 | lift motors 1–4 | 33–36 |
| 9 | flotation release | 28 (gripper) |

Override with `--map 1=aileronLeft,10=aileronRight` and reverse surfaces with
`--reverse 2,4` if a surface moves the wrong way (check with the SITL servo
output view: positive roll command must give right-wing-down).

## What the bridge sends

`timestamp`, `imu.gyro` (body rad/s), `imu.accel_body` (specific force, m/s²),
`position` (NED from the start point), `velocity` (NED), `quaternion` (body→NED),
`airspeed` (pitot) and `rng_1` (downward rangefinder).
