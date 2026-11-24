# QuadPlane Rescue Sim

A browser flight simulator for the lifeguard-rescue QuadPlane. You fly it with
your RadioMaster Pocket, using ArduPlane-style modes (QLOITER, QHOVER, FBWA,
CRUISE, AUTO, RTL, QRTL, QLAND, MANUAL) over a beach with a swimmer 800 m offshore.

The physics uses your design data directly:
- the flow5 stability derivatives;
- the mass and inertia;
- the lift-motor, pusher and battery figures.

It reproduces your flow5 dynamic modes to within 1–2 % (see [Validation](#validation)).

## Run it (Windows 10)

1. Download **`ClaudeFlightSim.html`** from this branch. On GitHub: open the file → **Download raw file**.
2. Double-click it. It opens in Chrome or Edge, with nothing to install and no internet needed.
3. Click **Help** (or press F1) for the controls.

You can fly with the keyboard straight away. To use the radio, connect it as described below.

## Connect the RadioMaster Pocket

1. Power the radio on and plug it into the PC with a USB-C **data** cable.
2. On the radio's USB pop-up choose **USB Joystick (HID)**. Windows sees it as a game controller; you can check
   with `joy.cpl`.
3. In the simulator, move a stick (browsers only report a joystick after it moves), then open **Controller**
   (or press G) and run the **calibration wizard**. It detects:
   - the sticks;
   - the arm switch;
   - the 3-position flight-mode switch;
   - the RTL/QLAND switch;
   - the payload-release switch.
4. Defaults follow your planned layout:
   - mode switch = QLOITER / FBWA / AUTO;
   - second switch = off / RTL / QLAND;
   - QHOVER, CRUISE and MANUAL on the **H**, **C** and **M** keys.

   Any mode can be moved between a switch position and a key on the same screen. Calibration is saved in the
   browser; you can also export it to a file.

EdgeTX's classic joystick mode maps CH1–CH8 to the joystick axes. With your ArduPilot channel order (AETR plus the
switches on CH5–CH8) the defaults may already be right; the wizard sorts out any differences.

## Your first flight

1. Start in **QLOITER** with the camera in **line-of-sight**: you stand behind the pad, as you will on the beach.
2. Throttle low, flip **arm**, then raise the throttle above the centre to climb. Centre the throttle to hold height.
3. Climb to about 20 m, point into the wind and switch to **FBWA** with the throttle high. Watch the transition bar:
   the lift motors hold height until the airspeed reaches 20 m/s, then ramp down.
4. Fly some circuits. Keep the airspeed above about 20 m/s. Below 16 m/s **Q_ASSIST** steps in; the stall is 16.7 m/s.
5. Slow down, switch back to **QLOITER**, and expect it to coast and balloon upward. Then descend and land on the pad
   (touchdown under 2.5 m/s).
6. Try **AUTO** to watch the full rescue mission, then fly it yourself: reach the swimmer within 60 s and release
   the float (**payload switch** or **B**) at 5 m above the water.

## Controls

| Input | Action |
|---|---|
| Arm switch / Space | Arm / disarm (throttle low to arm) |
| Mode switch, RTL/QLAND switch | Flight modes (configurable) |
| H, C, M, 1–6 | QHOVER, CRUISE, MANUAL, QLOITER, FBWA, AUTO, RTL, QLAND, QRTL |
| Payload switch / B | Release the flotation device |
| Keyboard flying | Arrows = pitch/roll, A/D = yaw, W/S = throttle, X = 50 %, Z = 0 |
| V | Camera: line-of-sight → chase → FPV |
| Mouse wheel, + / − | Zoom (LOS), distance (chase), tilt (FPV) |
| O | Aircraft and swimmer markers |
| P, T, R | Pause, time ×0.5/×1/×2, reset |
| G, Tab, F1 | Controller setup, settings, help |

The **Settings** menu covers:
- wind: speed, direction, gusts and turbulence, up to your 10 m/s design case;
- air temperature or your 1.083 kg/m³ design density;
- a drag multiplier and sensor errors;
- the rescue scenario;
- every ArduPilot parameter the flight controller uses;
- the camera and start options.

## What the simulator found about the design

These come from the simulator, so they are only as good as its inputs (see the assumptions below).

1. **Q_ASSIST_SPEED 16 m/s is below the 16.7 m/s stall speed.** In FBWA the aircraft can stall before the lift
   motors help. 17–18 m/s is safer.
2. **The 60 s target holds in calm air (swimmer reached at 46 s) but only just in the 10 m/s onshore design wind
   (59.6 s).** The float goes in at about 62 s in calm air and 89 s in that headwind, because the back-transition,
   braking and descent to 5 m take 25–30 s.
3. **Back-transition from 26.7 m/s needs about 250 m.** With ArduPilot's default Q_TRANS_DECEL of 2.0 the AUTO
   approach overshoots the swimmer by about 60 m. With 1.5 m/s² (the simulator's default) it stops within a few
   metres in calm air, and about 30 m past in the gusty 10 m/s onshore case. Braking harder makes the wing balloon
   upward.
4. **The pusher is overpowered for its motor when static.** Full throttle on the ground draws about 111 A (the
   motor is rated 85 A) and spins the APC 15×8E to about 10,800 rpm (APC's limit is 9,667 rpm).
   THR_MAX 80 % gives 53 N at about 74 A and 8,700 rpm. A 0–20 m/s transition then takes about 4 s plus the ramp.
5. **Hovering in wind costs much less power.** Holding position in a 10 m/s wind takes about 750 W versus 1150 W
   in calm air, thanks to rotor translational lift and wing lift.
6. **Strong crosswind takes away heading control in hover.** Differential-thrust yaw on a large quadplane is weak
   (under about 1.2 N·m) compared with the tail's weathervane moment. ArduPilot's mixer gives up yaw first, as it
   should; consider Q_WVANE_ENABLE.
7. **Altitude reference.** The pad is about 1 m above the sea, so "5 m" relative altitude is about 6 m above the
   water. The rescue mission converts the 5 m drop height to the right relative altitude; a real mission must do the
   same, or use a rangefinder.
8. **Line of sight.** At 800 m the 1.34 m wingspan is a speck. The simulator shows that honestly, with optional
   markers and zoom; FPV or AUTO is the realistic way to fly that leg.

## Validation

| | Simulator | Your flow5 data |
|---|---|---|
| Short period | 0.930 Hz, ζ 0.466 | 0.93 Hz, ζ 0.46 |
| Phugoid | 11.5 s | 11.4 s |
| Roll time constant | 0.226 s | 0.23 s |
| Dutch roll | 0.528 Hz, ζ ≈ 0 | 0.53 Hz, ζ ≈ 0 |
| Spiral | doubles in 27.2 s | ~27 s |
| Stall | 16.6 m/s | 16.7 m/s |
| Hover / cruise power | 1205 W / 365 W | 1205 W / 365 W |

`npm run validate` prints the full report. [docs/MODEL.md](docs/MODEL.md) explains the model, every estimate, and how
each number was checked.

### Estimates I had to make

Change any of these in `src/core/params.js`; the model recalibrates at start-up.
- X3525 winding resistance and no-load current;
- lift-rotor solidity;
- motor, prop and ESC inertia, and the ESC current limit;
- battery internal resistance;
- skid geometry;
- fuselage diameter;
- post-stall flat-plate coefficients;
- negative stall CL;
- whether the 32 N maximum thrust was quoted at sea-level density (I assumed yes).

## Limitations

- The built-in flight controller **approximates** ArduPlane's QuadPlane code. The mode logic, limits and parameter
  names follow ArduPilot, but it is not the firmware. For exact behaviour, fly the same physics with real ArduPilot
  SITL: see [docs/SITL.md](docs/SITL.md) (Phase 2, bridge included).
- It does not model ground effect on the wing, propwash over the tail, rotor flapping moments, ESC desync or EKF
  failures.

## Development

```bash
npm install
npm test            # physics validation + closed-loop flight tests + SITL bridge test
npm run validate    # validation report
npm run build       # rebuilds ClaudeFlightSim.html and dist/
npm run dev         # http://localhost:8000 with rebuild on save
node tools/sitl-bridge.mjs   # ArduPilot SITL JSON backend (see docs/SITL.md)
```

Code layout:
- `src/core/`: physics (no browser code, also runs in Node);
- `src/fc/`: flight controller;
- `src/sim/`: simulation loop, scenario and analysis;
- `src/app/`: browser UI;
- `tools/`: build, validation, SITL bridge;
- `test/`: tests.
