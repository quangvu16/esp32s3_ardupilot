# ESP32 Rover GCS: Android app

A small [Mission Planner](https://github.com/ArduPilot/MissionPlanner) for the phone, made for
this repository's ArduRover on the ESP32-S3. It speaks MAVLink 2 to the board over its WiFi
(TCP `192.168.4.1:5760`), and also works with any other ArduRover or with SITL.

Native Kotlin, no libraries: the MAVLink code, the map and the NTRIP client are part of the app.
Android 8.0 (API 26) or newer.

## Connect

1. Power the board. On the phone, join the WiFi network **`ardupilot123`** (password `ardupilot123`).
   Android says it has no internet. Choose **stay connected**.
2. Open the app. It connects to TCP `192.168.4.1:5760` by itself and reconnects if the link drops.
   The first pill at the top turns green with the address.

Mobile data stays on. The app sends MAVLink through the board's WiFi, and fetches map tiles
and NTRIP corrections over mobile data.

## Screens

| Tab | What it does | Mission Planner equivalent |
| --- | --- | --- |
| **Data** | HUD (horizon, heading, speed, mode, armed, GPS, battery, waypoint distance). **Arm** (long press: force arm), mode buttons, and a map with the car, its trail, home and the mission. Tap the map in Guided mode to drive there. Long press for *Drive here*, *Set home here* or *Add waypoint*. **Drive** opens a phone stick (RC override over MAVLink, power limit 10-100%). | Flight Data |
| **Plan** | Tap the map to add waypoints, drag them to move, tap one to edit it (waypoint, loiter time, RTL, speed change, order, delete). **Read** / **Write** to the vehicle, **Start Auto**, mission speed. Load and save Mission Planner `.waypoints` files. | Flight Plan |
| **Params** | Full parameter list with search. Tap to edit: **Write now**, or **Stage** several and write them together. **Load file** stages only the values that differ from the vehicle (e.g. [`esp32s3_rover_ld14_f9p.parm`](../esp32s3_rover_ld14_f9p.parm)). **Save file**, **Reboot**. | Config → Full Parameter List |
| **Setup** | Connection (TCP / UDP). 6-position accelerometer calibration, or level only. Compass calibration. Radio bars and calibration (writes `RCn_MIN/MAX/TRIM`). LiDAR proximity view. RTK over NTRIP. Set home, reboot, clear the mission. | Initial Setup, Optional Hardware |
| **Log** | Every message from the autopilot (`STATUSTEXT`), newest first. | Messages tab |

### RTK / NTRIP

Setup → RTK / NTRIP: caster, port (usually 2101), mountpoint, user and password, then **Start NTRIP**.
The app pulls RTCM3 from the caster over mobile data. It checks each frame's CRC and sends it to the
ZED-F9P as `GPS_RTCM_DATA`, splitting frames over 180 bytes. Every 10 s it sends the car's
position to the caster as GGA, which VRS casters need. The status line lists the RTCM message
types received. The GPS pill goes `3D fix` → `RTK Float` → `RTK Fixed`.

### Driving from the phone

**Drive** sends `MANUAL_CONTROL`. ArduRover reads it as an RC override: steering on `y`,
throttle on `z`. It only works in modes that use the sticks (Manual, Acro, Steering…), and
`SYSID_MYGCS` must be 255, the default. Releasing the stick sends zeros. If the app stops
sending, the override ends after `RC_OVERRIDE_TIME` (default 3 s) and your RC transmitter is in
control again.

Use **Power** to limit the stick. Test with the wheels off the ground first.

## Build

Built on this PC with no Android Studio. The toolchain is in `~/Android`: JDK 17, the SDK and
Gradle 8.7. The Gradle cache already holds every plugin, so the build works offline.

```bash
cd android_gcs
export JAVA_HOME=~/Android/jdk ANDROID_HOME=~/Android/Sdk
~/Android/gradle-8.7/bin/gradle --no-daemon --offline assembleRelease
# -> app/build/outputs/apk/release/app-release.apk  (signed with the debug key)
```

`./gradlew` also works where gradle.org (or the Tencent mirror it is set to) can be reached.
Create `local.properties` with `sdk.dir=<path to the Android SDK>`, or set `ANDROID_HOME`.

## Install

- **USB:** turn on Developer options → USB debugging, then
  `adb install -r app/build/outputs/apk/release/app-release.apk`
- **Without a cable:** copy the APK to the phone, open it, and allow "install unknown apps".

## How it was tested

The MAVLink, parameter, mission and command code (`Mavlink.kt`, `Link.kt`, `Vehicle.kt`,
`Ntrip.kt`) was compiled for the desktop JVM, with small stand-ins for the Android classes it
uses, and run against ArduRover 4.6.3 SITL built from this tree. These worked over TCP 5760:

- heartbeat and telemetry
- reading all 1283 parameters, single and batch writes, and a refused unknown name
- a 5-item mission written and read back identical
- Guided mode, arming, and driving to a tapped point
- Auto mode, with the mission running
- driving with `MANUAL_CONTROL`, then disarming
- both calibration flows: the accel position prompts, and compass progress
- splitting and fragmenting RTCM3

UDP listen mode on 14550 was also tested: heartbeat, all parameters, and a mode change.
The screens were not tested in that run; they need a phone or an emulator.

## Files

| File | Contents |
| --- | --- |
| `Mavlink.kt` | v1/v2 parser, v2 encoder, X.25 CRC, message ids, CRC extras and lengths (from ArduPilot 4.6's generated headers) |
| `Link.kt` | TCP and UDP links. The socket is bound to the WiFi network so it does not go out over mobile data |
| `Vehicle.kt` | Telemetry state, parameter / mission / command protocols with retries, calibrations, RTCM injection |
| `Ntrip.kt` | NTRIP 1.0 client, RTCM3 frame splitter (CRC-24Q), GGA sentence |
| `MapView.kt` | OpenStreetMap / Esri satellite tiles (disk cache), vehicle, trail, home, mission, waypoint dragging |
| `HudView.kt`, `RadarView.kt`, `JoystickView.kt` | HUD, LiDAR proximity view, drive stick |
| `MainActivity.kt` | The five tabs |
