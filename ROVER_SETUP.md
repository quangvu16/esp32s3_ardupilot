# Rover parameter setup

Parameter setup for the RC car this firmware drives (ESP32-S3 `esp32s3devkit` board,
steering servo + ESC, MPU9250 IMU, ZED-F9P GPS, LD14 LiDAR), in the order to do it.
Parameter names are those of ArduRover 4.6. Wiring is in the [README](README.md#wiring).

## 0. Connect

1. Power the ESP32-S3. It creates its own WiFi network **`ardupilot123`** (password `ardupilot123`).
2. Join that network from your PC. In Mission Planner choose **TCP** → `192.168.4.1`, port `5760`.
3. **Config → Full Parameter List → Load from file** → [`esp32s3_rover_ld14_f9p.parm`](esp32s3_rover_ld14_f9p.parm)
   → **Write Params**. Reboot the board and reconnect.

## 1. Mandatory hardware (Setup → Mandatory Hardware)

| Step | What to do |
| --- | --- |
| Frame type | Already set: `FRAME_CLASS=1` (Rover) |
| Accel calibration | Put the board **fixed in the car** first, then do the 6-position calibration. If the board isn't mounted arrow-forward and flat, set `AHRS_ORIENTATION` before calibrating. |
| Compass | Run the calibration by rotating the car in every direction. If no compass is found (many "MPU9250" boards are really an MPU6500), see [step 5](#5-heading-if-theres-no-working-compass). |
| Radio calibration | Move every stick and switch to its ends. Check that channel 1 = steering, channel 3 = throttle, channel 8 = mode switch. |
| Flight modes | Already set on channel 8: Manual / Auto / RTL. Add **Hold** as a safe position, e.g. `MODE2=4`. |

## 2. Steering and throttle checks (wheels off the ground, Manual mode)

| Problem | Fix |
| --- | --- |
| Stick right turns the wheels left | `SERVO1_REVERSED=1` |
| Car isn't straight at centre stick | Change `SERVO1_TRIM` in small steps (around 1500) |
| Steering servo hits its end stops | Narrow `SERVO1_MIN` / `SERVO1_MAX` until it doesn't strain |
| Throttle forward drives backwards | `SERVO3_REVERSED=1` |
| Wheels creep at centre throttle | Calibrate the ESC's neutral (most car ESCs learn it at power-up with the throttle at centre). Keep `SERVO3_TRIM=1500` |
| Motor needs a lot of throttle before it moves | Raise `MOT_THR_MIN` (%) until a small stick movement just starts the car rolling |
| Car accelerates too hard | `MOT_SLEWRATE` (%/s, e.g. 100) |

**Arming:** throttle at centre (zero) and steering **full right for 2 seconds**. Disarm with
full left. Do your first arm with the wheels in the air.

## 3. Failsafes (set before driving on the ground)

| Parameter | Value | Why |
| --- | --- | --- |
| `FS_THR_ENABLE` | `1` | React when the RC signal is lost |
| `FS_THR_VALUE` | `910` | Set your **receiver's own failsafe** so channel 3 drops below this (e.g. 900 µs) when the transmitter is off |
| `FS_GCS_ENABLE` | `1` | React when the WiFi link to Mission Planner drops |
| `FS_ACTION` | `2` (Hold) | Stop. `1` = RTL, but only use that after GPS works well |
| `FS_TIMEOUT` | `1.5` | Seconds before the failsafe triggers |
| `FS_CRASH_CHECK` | `2` | Hold and disarm if the car is stuck against something |

Test it: arm with the wheels off the ground, switch the transmitter off, and check that the
wheels stop.

## 4. GPS (ZED-F9P)

Already set in the parameter file. Check that Mission Planner shows `3D Fix` outdoors. For RTK,
use **Setup → Optional Hardware → RTK/GPS Inject → NTRIP** (caster, port, usually 2101,
mountpoint, user, password). The status goes 3D Fix → RTK Float → RTK Fixed. ArduPilot sets
up the F9P itself because `GPS_AUTO_CONFIG=1`.

## 5. Heading if there's no working compass

If the compass is missing, or upset by the motor and its wires:

```
COMPASS_ENABLE = 0
EK3_SRC1_YAW   = 8     # GSF: heading estimated from GPS while the car moves
```

The car must drive straight a few metres after arming before Auto mode has a good heading.
With a working compass, keep `COMPASS_ENABLE=1` and mount the board away from the motor and
battery wires.

## 6. Battery monitor (optional)

Connect the battery through a resistor divider to GPIO5 (analog pin 1), keeping the pin below 3.1 V:

```
BATT_MONITOR    = 3     # analog voltage only
BATT_VOLT_PIN   = 1
BATT_VOLT_MULT  = ...   # adjust until Mission Planner's voltage matches a multimeter
BATT_LOW_VOLT   = ...   # e.g. 3.5 V x number of cells
BATT_FS_LOW_ACT = 2     # Hold
```

## 7. Tuning (outdoors, open space)

Work in this order. Mission Planner's **Config → Tuning** page and the live graphs
(Ctrl-F → Tuning) show requested vs actual values. Logs need `LOG_BACKEND_TYPE=2`
(log over MAVLink), since there's no SD card.

1. **Cruise throttle.**
   - Set `RC7_OPTION=50` (Learn Cruise) on a switch.
   - Drive straight in Manual at a steady speed, then flip the switch. `CRUISE_SPEED` and
     `CRUISE_THROTTLE` are set from that run.
2. **Turn rate (Acro mode, e.g. `MODE3=1`).**
   - `TURN_RADIUS`: your car's tightest turn in metres (default 0.9).
   - `ATC_STR_RAT_MAX`: highest turn rate you want, in °/s (e.g. 120).
   - `ATC_STR_RAT_FF`: raise it until the actual turn rate follows the requested rate. Then
     add a little `ATC_STR_RAT_P` (~0.1–0.2) and `ATC_STR_RAT_I` (similar).
3. **Speed control (Acro mode).**
   - Start with `ATC_SPEED_P=0.2` and `ATC_SPEED_I=0.2`. Raise them until the speed holds
     without surging.
   - `ATC_ACCEL_MAX` and `ATC_DECEL_MAX` in m/s², e.g. 1.0 for gentle driving.
4. **Navigation (Auto mode).**
   - `WP_SPEED`: driving speed in m/s.
   - `WP_RADIUS`: how close counts as "arrived" (≥ `TURN_RADIUS`; with RTK, 0.5 m works).
   - `WP_ACCEL`: 0 = use `ATC_ACCEL_MAX`.
   - If it cuts corners or swings wide, look first at `TURN_RADIUS` and `ATC_STR_ANG_P`.

## 8. Obstacle avoidance with the LiDAR (last)

1. Put an object in front of the car. In the proximity view (Ctrl-F → Proximity), set
   `PRX1_YAW_CORR` so the object shows at 0°.
2. Then:
   ```
   AVOID_ENABLE = 3     # fence + proximity: stop or slow before obstacles
   AVOID_MARGIN = 0.5   # m
   OA_TYPE      = 1     # BendyRuler: drive around obstacles in Auto (optional)
   ```
3. Test slowly in Manual first: the car should refuse to drive into a box.

## Checklist before every drive

- [ ] Mode switch on Manual or Hold
- [ ] GPS 3D Fix (or RTK) and EKF status green
- [ ] Receiver failsafe tested
- [ ] Battery charged, and the car on the ground only once directions are confirmed
