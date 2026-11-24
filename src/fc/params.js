// ArduPlane / QuadPlane parameters used by the built-in flight controller.
// Names follow ArduPilot; units are SI (m, m/s, deg, s) unless the name says
// otherwise. Values are ArduPilot defaults, except where the owner's design
// data sets them (marked [USER]) or this airframe needs something else ([SIM]).

export const DEFAULT_FC_PARAMS = {
  // ---- speeds
  ARSPD_FBW_MIN: 20.0, // [USER] transition / minimum fixed-wing speed (1.2 Vs)
  ARSPD_FBW_MAX: 30.0,
  AIRSPEED_CRUISE: 20.8, // [USER]
  DASH_SPEED: 26.7, // [USER] rescue dash speed (used by the mission)

  // ---- QuadPlane
  Q_ASSIST_SPEED: 16.0, // [USER] (note: below the 16.7 m/s stall speed)
  Q_ASSIST_ANGLE: 30,
  Q_ASSIST_DELAY: 0.5,
  Q_TRANSITION_MS: 1500, // [SIM] lift-motor ramp-down after reaching ARSPD_FBW_MIN
  Q_TRANS_FAIL: 0, // s, 0 = disabled
  Q_TRAN_PIT_MAX: 3,
  Q_TRANS_DECEL: 1.5, // [SIM] m/s^2 planned deceleration (ArduPilot default 2.0 overshoots ~60 m on this airframe)
  Q_BCK_PIT_LIM: 10, // deg max nose-up during back-transition braking
  Q_ANGLE_MAX: 30,
  Q_LOIT_SPEED: 5.0,
  Q_LOIT_ACC_MAX: 2.5,
  Q_LOIT_BRK_ACCEL: 2.5,
  Q_WP_SPEED: 5.0,
  Q_WP_SPEED_UP: 2.5,
  Q_WP_SPEED_DN: 1.5,
  Q_PILOT_SPD_UP: 2.5,
  Q_PILOT_SPD_DN: 1.5,
  Q_LAND_SPEED: 0.5,
  Q_LAND_FINAL_ALT: 6,
  Q_RTL_ALT: 15,
  Q_YAW_RATE_MAX: 60, // deg/s
  Q_A_INPUT_TC: 0.15,
  Q_FWD_THR_USE: 1, // 0 off, 1 position modes (QLOITER/QRTL/AUTO), 2 all VTOL modes
  Q_FWD_PIT_LIM: 3,
  Q_FWD_THR_GAIN: 2.0,
  Q_WVANE_ENABLE: 0, // weathervane into wind in position modes
  Q_WVANE_GAIN: 1.0,
  Q_M_SPIN_ARM: 0.1,
  Q_M_SPIN_MIN: 0.15,
  Q_M_SPIN_MAX: 0.98,
  Q_RTL_MODE: 1, // RTL flies fixed-wing home, then VTOL lands

  // ---- fixed wing
  THR_MAX: 80, // [SIM] % - keeps the pusher near its 85 A rating when static
  THR_MIN: 0,
  LIM_ROLL_CD: 45, // deg (name kept for familiarity)
  LIM_PITCH_MAX: 20,
  LIM_PITCH_MIN: -25,
  STAB_PITCH_DOWN: 2,
  STALL_PREVENTION: 1,
  RLL2SRV_TCONST: 0.5,
  PTCH2SRV_TCONST: 0.5,
  YAW2SRV_DAMP: 1.0, // yaw damper gain scale (0 disables)
  YAW2SRV_RLL: 1.0, // turn-coordination feed-forward
  NAVL1_PERIOD: 14,
  NAVL1_DAMPING: 0.75,
  WP_RADIUS: 40,
  RTL_ALTITUDE: 30, // m (ALT_HOLD_RTL)
  TECS_CLMB_MAX: 5,
  TECS_SINK_MAX: 5,
  TECS_TIME_CONST: 5,
  FBWB_CLIMB_RATE: 2,

  // ---- arming / failsafe
  LAND_DISARMDELAY: 2,
  BATT_LOW_VOLT: 21.0, // 3.5 V/cell loaded
  BATT_CRT_VOLT: 20.4, // 3.4 V/cell
  BATT_FS_LOW_ACT: 0, // 0 warn only, 1 RTL, 2 QLAND
  BATT_FS_CRT_ACT: 0,
  FS_LONG_ACTN: 1, // radio loss: 0 continue, 1 RTL, 2 QLAND
  FS_LONG_TIMEOUT: 1.5,
};

/** Human-readable descriptions for the settings panel. */
export const PARAM_INFO = {
  ARSPD_FBW_MIN: 'Transition complete / min fixed-wing airspeed (m/s)',
  ARSPD_FBW_MAX: 'Max fixed-wing airspeed (m/s)',
  AIRSPEED_CRUISE: 'Cruise airspeed (m/s)',
  DASH_SPEED: 'Rescue mission dash airspeed (m/s)',
  Q_ASSIST_SPEED: 'Lift motors assist below this airspeed (m/s, 0 = off)',
  Q_ASSIST_ANGLE: 'Assist if attitude error exceeds (deg)',
  Q_TRANSITION_MS: 'Lift-motor ramp-down time after transition airspeed (ms)',
  Q_TRANS_DECEL: 'Planned deceleration for VTOL approaches (m/s²)',
  Q_BCK_PIT_LIM: 'Max nose-up pitch while braking after back-transition (deg)',
  Q_ANGLE_MAX: 'Max VTOL lean angle (deg)',
  Q_LOIT_SPEED: 'QLOITER max horizontal speed (m/s)',
  Q_WP_SPEED: 'VTOL navigation speed (m/s)',
  Q_PILOT_SPD_UP: 'Pilot max climb rate in Q modes (m/s)',
  Q_PILOT_SPD_DN: 'Pilot max descent rate in Q modes (m/s)',
  Q_LAND_SPEED: 'Final landing descent rate (m/s)',
  Q_RTL_ALT: 'QRTL altitude (m)',
  Q_YAW_RATE_MAX: 'Max pilot yaw rate in Q modes (deg/s)',
  Q_FWD_THR_USE: 'Use pusher in VTOL modes (0 off, 1 position modes, 2 all)',
  Q_WVANE_ENABLE: 'Weathervane into wind in position modes (0/1)',
  THR_MAX: 'Max pusher throttle (%)',
  LIM_ROLL_CD: 'Max fixed-wing bank angle (deg)',
  LIM_PITCH_MAX: 'Max fixed-wing pitch up (deg)',
  LIM_PITCH_MIN: 'Max fixed-wing pitch down (deg)',
  YAW2SRV_DAMP: 'Yaw damper gain (0 = off, shows the neutral Dutch roll)',
  RTL_ALTITUDE: 'Fixed-wing RTL altitude (m)',
  BATT_LOW_VOLT: 'Low-battery voltage (V)',
  BATT_CRT_VOLT: 'Critical-battery voltage (V)',
  BATT_FS_LOW_ACT: 'Low-battery action (0 warn, 1 RTL, 2 QLAND)',
  BATT_FS_CRT_ACT: 'Critical-battery action (0 warn, 1 RTL, 2 QLAND)',
  FS_LONG_ACTN: 'Radio-loss action (0 continue, 1 RTL, 2 QLAND)',
};
