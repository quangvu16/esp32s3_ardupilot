# ESP32-S3 Rover: user guide

**English** | [Tiếng Việt](GUIDE.vi.md)

How to use this repository's ArduRover on an RC car, from the first connection to zig-zag, patrol
and the LiDAR. Building, flashing, wiring and the full parameter list are in the
[README](../README.md); this guide is about driving.

There are four ground station apps with the same core: Android, Windows, iPhone/iPad and Mac.

![Android app](../android_gcs/docs/screens.svg)

<details><summary>Windows, iPhone and Mac apps</summary>

![Windows app](../android_gcs/desktop/docs/screens.svg)

![iPhone app](../apple_gcs/docs/screens-ios.svg)

![Mac app](../apple_gcs/docs/screens-mac.svg)

</details>

## Contents

1. [What you have](#1-what-you-have)
2. [Connect](#2-connect)
3. [Drive by hand](#3-drive-by-hand)
4. [Missions](#4-missions)
5. [Zig-zag](#5-zig-zag)
6. [Patrol with the LiDAR](#6-patrol-with-the-lidar)
7. [Fences](#7-fences)
8. [LiDAR behaviours: Avoid, Follow, Warn](#8-lidar-behaviours-avoid-follow-warn)
9. [RTK GPS](#9-rtk-gps)
10. [Troubleshooting](#10-troubleshooting)
11. [Before every drive](#11-before-every-drive)
12. [Practise without the car](#12-practise-without-the-car)
13. [Indoors without GPS (LiDAR SLAM)](#13-indoors-without-gps-lidar-slam)

## 1. What you have

| Part | What it is | Where |
| --- | --- | --- |
| Firmware | ArduRover 4.6.3 for the ESP32-S3 with the LD14 LiDAR, ZED-F9P RTK, object avoidance and fences | [Releases](https://github.com/quangvu16/esp32s3_ardupilot/releases), README [Flash](../README.md#flash) |
| Android app | Phone ground station: map, missions, zig-zag, patrol, fences, LiDAR, parameters, calibration, RTK | [android_gcs](../android_gcs/README.md), APK in the `gcs-v…` releases |
| Windows app | The same on a PC (no calibration or RTK: use Mission Planner for those). `Esp32RoverGcs.exe` with Java inside | [android_gcs/desktop](../android_gcs/desktop/README.md), *Windows app (exe)* workflow / releases |
| iPhone, iPad and Mac app | One SwiftUI app: the Android app's tabs, calibration and RTK included; arrow keys drive on the Mac | [apple_gcs](../apple_gcs/README.md), built with Xcode |
| Mission Planner | Still works: TCP `192.168.4.1:5760` | [ardupilot.org](https://ardupilot.org/planner/) |

The board's WiFi takes one ground station at a time comfortably. Close the others before driving.

### Install the apps

| App | How |
| --- | --- |
| Android | Download the APK from the latest `gcs-v…` release, open it on the phone and allow *install unknown apps*. |
| Windows | `Esp32RoverGcs-<version>.exe` installs for your user (Start menu *ESP32 Rover* and a desktop shortcut); or unzip `…-windows-portable.zip` and run `Esp32RoverGcs\Esp32RoverGcs.exe`. No Java needed. First run: SmartScreen *More info → Run anyway*; firewall: allow *Private networks*. |
| Mac | Open `apple_gcs/Esp32RoverGcs.xcodeproj` in Xcode, choose **My Mac**, **Product → Run**. No Apple account needed. The first time, macOS may block it: System Settings → Privacy & Security → **Open Anyway**. |
| iPhone / iPad | Xcode → Settings → Accounts: add your Apple ID. Project → Signing & Capabilities → Team: *your name (Personal Team)*. Connect the iPhone by cable, turn on Settings → Privacy & Security → **Developer Mode**, choose it in Xcode and **Run**. On the iPhone: Settings → General → VPN & Device Management → your Apple ID → **Trust**. With a free Apple ID, run it from Xcode again every 7 days. |

Details: [apple_gcs/README](../apple_gcs/README.md), [desktop README](../android_gcs/desktop/README.md).

## 2. Connect

1. Power the car. The ESP32-S3 creates the WiFi network **`ardupilot123`** (password `ardupilot123`)
   within about 10 seconds.
2. Join it from the phone or the PC.
   - **Android** says the network has no internet: choose *stay connected*. Mobile data stays on
     for maps and RTK; the app sends MAVLink over the WiFi by itself.
   - **Windows** and **Mac**: the WiFi has no internet either; that's normal.
   - **iPhone / iPad**: stay on the network when iOS says it has no internet. The first time the app
     opens, allow **Local Network**; without it the app can't reach `192.168.4.1` (Settings → Privacy
     & Security → Local Network to change it later). Mobile data stays on for the map and RTK.
3. Open the app. It connects to TCP `192.168.4.1:5760` by itself. The first item in the status bar
   turns green.

First time only: load [`esp32s3_rover_ld14_f9p.parm`](../esp32s3_rover_ld14_f9p.parm) (Params →
Load), write it, reboot, then do the accelerometer, compass and radio calibration (Android: Setup
tab; or Mission Planner). The order and the checks are in the README's [Set up](../README.md#set-up).

## 3. Drive by hand

- **Arm**: the Arm button (Android: long-press to force), or the RC transmitter (throttle centre,
  steering full right for 2 s). Disarm the same way, or full left.
- **Modes**: Manual (raw sticks), Acro (sticks = speed and turn rate; zero brakes), Hold (stop),
  Auto (mission), Guided (go to a point), RTL (back home).
- **Without a transmitter**: the **Drive** stick of the Android and iPhone apps (Data tab → Drive), or
  the arrow keys / W A S D of the Windows app (tick *Drive with arrow keys*) and the Mac app (open
  **Drive**). All send stick overrides that work in Manual, Acro and Steering; *Power* caps them.
  Letting go sends zero; on a PC or Mac, held keys are dropped when the window loses the focus.
  If the app stops sending, the car falls back to the transmitter after `RC_OVERRIDE_TIME` (3 s).
- **Guided**: switch to Guided, then tap (click) the map where the car should go. (iPhone and Mac:
  the map's *Tap* menu chooses what a tap does: drive here, add a waypoint, or move home.)

Do the first drive with the wheels off the ground.

## 4. Missions

On the **Plan** tab: tap (click) the map to add waypoints, drag them to move them, tap one to edit it
(wait time, loiter, return to launch, speed change, order). **Write** sends the plan to the car,
**Read** fetches the car's. **Start Auto** runs it from the first item. Mission Planner
`.waypoints` files can be loaded and saved.

Settings that matter: `WP_SPEED` (m/s), `WP_RADIUS` (how close counts as arrived: at least
`TURN_RADIUS`; 0.5-1 m with RTK), `TURN_RADIUS` (the car's tightest turn).

## 5. Zig-zag

Copter has a ZigZag mode for spraying and mowing; Rover doesn't, so the apps make it a mission: the
car drives from A to B, steps sideways, drives back, and so on, in Auto.

1. **Set A and B**: drive the car to one end and press **A = car here**, then to the other end and
   **B = car here** (Plan → **Zig zag…**). Or long-press (right-click) the Plan map and choose
   *Zig-zag point A/B here*. The purple dashed line is A→B.
2. **Settings**, named after Copter's `ZIGZ_` parameters:

   | Setting | Meaning |
   | --- | --- |
   | Line spacing | Distance between lines (`ZIGZ_SIDE_DIST`) |
   | Lines | How many lines (`ZIGZ_LINE_NUM`) |
   | Side | Right or left of A→B (`ZIGZ_DIRECTION`) |
   | Wait at ends | Seconds at the end of each line (`ZIGZ_WP_DELAY`) |
   | Run-out | Extra length past both ends, so the car turns outside the area |

3. **Make lanes** replaces the plan with the lanes. **Write**, then **Start Auto**.

A car can't U-turn in less than about 2 × `TURN_RADIUS`. With a smaller spacing, give it a run-out
of about `TURN_RADIUS`: it then swings round outside the area and is back on the line before the
area starts. Line 1 starts at A itself (no run-out behind the car).

Tested in SITL: 4 lines of 20 m, 4 m apart, driven within 0.14 m of each line; with 1 m spacing
(less than the 1.8 m U-turn) every line was still driven, within 0.5 m.

## 6. Patrol with the LiDAR

Patrol drives the plan round and round in Auto, and ArduPilot's **BendyRuler** steers round
whatever the LiDAR sees on the way. The loop is a `DO_JUMP` at the end of the mission, so the car keeps
patrolling on its own even if the phone or PC disconnects. **Hold** stops it.

1. **Make a plan**: waypoints, or a [zig-zag](#5-zig-zag). Keep every waypoint further than
   `OA_MARGIN_MAX` (2 m) from obstacles: BendyRuler can't reach a waypoint inside its margin and
   then wanders, in SITL even into an obstacle.
2. Plan → **Patrol…**: number of laps (0 = forever) and a wait at the end of each lap.
3. **Set up BendyRuler** (once). It writes the settings below. The `OA_BR_` parameters only exist
   after a reboot with `OA_TYPE` 1, so it takes **two presses**: the first writes `OA_TYPE` and
   reboots the board; after it reconnects, Refresh the parameters and press it again. The dialog
   says *BendyRuler ready* when everything is set.

   | Setting | Value | Why |
   | --- | --- | --- |
   | `OA_TYPE` | 1 | BendyRuler (3 = with Dijkstra's round fences) |
   | `OA_MARGIN_MAX` | 2 m | Distance kept from obstacles. 3 m left no way between obstacles 8 m apart |
   | `WP_RADIUS` | 1 m | Below the margin, or the car cuts corners into what it avoids |
   | `AVOID_MARGIN` | 1 m | ArduPilot's simple avoidance as a last stop. The default 2 m stops the car before BendyRuler can steer |
   | `WP_SPEED` | 1 m/s | At 2 m/s a car's turn carried it into an obstacle before it could react |
   | `OA_BR_LOOKAHEAD` | 5 m | How far ahead BendyRuler looks; under the LD14's 8 m range |
   | `OA_DB_DIST_MAX` | 8 m | The LD14's range |

4. **Make patrol**, **Write**, **Start Auto**. The HUD shows *Lap n/N*.

The LiDAR must be set up first (`PRX1_TYPE` 19, `PRX1_YAW_CORR` checked: a box in front shows at the
top of the LiDAR tab).

**Tested in SITL** with its simulated LiDAR and a field of 1 m-radius posts 10 m apart: a 2-lap patrol
whose first leg runs straight through three posts (without BendyRuler the car drives through them).
With the settings above the car drove round the posts, 0.5-1.6 m from them at the closest, and looped
into lap 2. **But it can get stuck:** in one run it came off a waypoint heading straight at a post,
simple avoidance stopped it 1 m short, and inside BendyRuler's 2 m margin BendyRuler found no way out,
so it stayed there. A car can't turn away on the spot. So:

- watch a patrol, and take over (Hold, then Manual) if the car stops in front of something;
- don't route the car straight at an obstacle from close by (a waypoint a few metres in front of one);
- keep waypoints in open space, further than `OA_MARGIN_MAX` from anything.

## 7. Fences

Plan → **Fence**. **+ Stay-in** / **+ Stay-out**, then tap the corners (3 or more); **+ Circle**,
then its centre and radius. Drag corners to move them; tap a corner for more. **Write** sends the
fence, **Enable…** turns it on (`FENCE_ENABLE`, `FENCE_TYPE`, `FENCE_ACTION` = Hold). The car then
won't arm outside a stay-in zone or inside a stay-out zone, and stops (Hold) if it crosses a fence.

The board stores about 80 corners. Fences are what Dijkstra's path planning (`OA_TYPE` 2 or 3)
plans around: keep `OA_MARGIN_MAX` above `WP_RADIUS`, or the car cuts a corner into the fence.

## 8. LiDAR behaviours: Avoid, Follow, Warn

The **LiDAR** tab shows the LD14's scan from above (firmware v4.6.3-esp32s3-3 or newer) and runs
the three behaviours of the hoverboard robot's `hoverboard_laser` package on the car:

| Behaviour | What it does |
| --- | --- |
| **Avoid** | Drives forward and steers away from what is in the front, left and right sectors |
| **Follow** | Follows the nearest object ahead, keeping the set distance |
| **Warn** | Beeps (and vibrates the phone) when something ahead is too close; doesn't drive |

Choose one, **Start**. Avoid and Follow drive with stick overrides: arm the car and use **Acro**
(the stick is a speed and zero brakes; in Manual the car coasts). The Drive stick / arrow keys pause
them; **Stop**, switching behaviour, leaving the app or losing the scan stops the car at once.

- Set `PRX1_TYPE` 19 and check `PRX1_YAW_CORR` first: put a box in front of the car; it must show at
  the top of the scan.
- Keep `AVOID_MARGIN` (ArduPilot's own stopping distance, README step 8) below the follow distance,
  or ArduPilot holds the car back. With the patrol settings (`AVOID_MARGIN` 1 m), set Follow's keep
  distance to 1.3 m or more.
- Start at the default speed (0.15) and raise it only once the car reacts in time.

These behaviours drive from the app. For avoiding obstacles on a mission, use BendyRuler
([patrol](#6-patrol-with-the-lidar)), which runs on the car itself.

## 9. RTK GPS

Android, iPhone and Mac: Setup → **RTK / NTRIP**: caster, port (2101), mountpoint, user,
password → **Start NTRIP**. On an iPhone keep *Over mobile data* on while the phone is on the car's
WiFi (that WiFi has no internet).
 On a PC: Mission Planner → Setup → Optional Hardware → RTK/GPS Inject. The GPS status goes
3D fix → RTK Float → RTK Fixed. With RTK Fixed, `WP_RADIUS` 0.5 m is realistic.

## 10. Troubleshooting

| Problem | What to check |
| --- | --- |
| The app doesn't connect | The phone or PC is on `ardupilot123`; the address is TCP `192.168.4.1:5760`; no other ground station is connected. iPhone: *Local Network* allowed for the app. The status says *no answer: is this device on the car's WiFi?* when it can't reach the board |
| iPhone app won't open after a week | Apps signed with a free Apple ID expire after 7 days: run it from Xcode again |
| Windows: "Windows protected your PC" | The exe isn't signed with a paid certificate: **More info → Run anyway** |
| Arming refused | The Log tab shows the reason (`PreArm: …`). Fences: the car must be inside its stay-in zones |
| Stick / arrow keys do nothing | Mode must be Manual, Acro or Steering; `SYSID_MYGCS` must be 255; the car must be armed |
| LiDAR tab shows no scan | Firmware v4.6.3-esp32s3-3 or newer, `PRX1_TYPE` 19, `SERIAL2_PROTOCOL` 11, LD14 TX on GPIO41 |
| `OA_BR_` parameters missing | They appear only after a reboot with `OA_TYPE` 1; Refresh the parameters after it reconnects |
| Car cuts corners into a fence | Raise `OA_MARGIN_MAX` above `WP_RADIUS` |
| Car stops in Hold on its own | A fence breach (`FENCE_ACTION`), a failsafe (`FS_…`) or the end of the mission: see the Log tab |

## 11. Before every drive

- [ ] Mode switch on Manual or Hold
- [ ] GPS 3D fix (or RTK) and a sensible position on the map
- [ ] Transmitter failsafe tested
- [ ] LiDAR: a box in front shows at the top of the scan
- [ ] First run of anything new: wheels off the ground, then slowly on the ground

## 12. Practise without the car

[`apple_gcs/tools/fake_rover.py`](../apple_gcs/tools/fake_rover.py) is a pretend ArduRover for
trying any of the apps at the desk. It needs only Python 3:

```bash
python3 apple_gcs/tools/fake_rover.py
```

Then point the app at TCP **`127.0.0.1:5760`** (Setup, or the Windows app's address box). It sends a
position in a park with RTK Fixed, a LiDAR scan with four posts around home, 23 parameters, and
answers arm, modes, the Drive stick, Guided, missions and fences: Auto drives the waypoints and stops
in Hold. It is a toy model, not a simulator of the real car; for that use ArduPilot SITL.

## 13. Indoors without GPS (LiDAR SLAM)

The car can drive missions indoors with no GPS: it maps the room with the LD14 and finds itself in
that map (about ±10 cm in tests), and the EKF fuses this with the IMU.

1. Load [`esp32s3_rover_ld14_indoor.parm`](../esp32s3_rover_ld14_indoor.parm) after the usual file,
   and set `PRX1_SLAM_LAT` and `PRX1_SLAM_LON` to your building.
2. Reboot with the car on a start mark, always facing the same way. Wait for `SLAM: tracking`.
3. Drive slowly in Manual first, then Auto.

How it works, the settings, accuracy, outdoor RTK tuning and troubleshooting are in
[Sensor fusion: IMU + LiDAR + GPS](SENSOR_FUSION.md).
