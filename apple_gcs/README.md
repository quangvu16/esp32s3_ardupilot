# ESP32 Rover GCS: iPhone, iPad and Mac app

The Android app's ground station as one native SwiftUI app for **iPhone, iPad (iOS 17+) and Mac
(macOS 14+)**. It speaks MAVLink 2 to this repository's ArduRover on the ESP32-S3 over the board's
WiFi (TCP `192.168.4.1:5760`), and also works with SITL or any other ArduRover.

No libraries: the MAVLink code, the NTRIP client and the LiDAR behaviours are Swift ports of the
Android app's shared core (`android_gcs/core`), the map is Apple's MapKit.

![iPhone app: Data, Plan, LiDAR and Setup tabs](docs/screens-ios.svg)

![Mac app: Data, Plan and LiDAR tabs](docs/screens-mac.svg)

The drawings are made by `python3 docs/gen_app_screens.py` (from the repository root) from the app's
layout. On an iPhone the tab bar shows Data, Plan, LiDAR, Setup and **More** (Params, Log); iPad and
Mac show all six.

| Tab | What it does |
| --- | --- |
| **Data** | HUD (horizon, heading, speed, mode, GPS accuracy, waypoint distance), **Arm** (with *Force arm*), mode buttons, map with the car, its trail, home, mission and fence. Tapping the map drives there in Guided (or adds a waypoint, or moves home: *Tap* menu). **Drive** opens a stick (RC override with `MANUAL_CONTROL`, power limit). On the Mac the arrow keys / WASD drive too. |
| **Plan** | Tap the map to add waypoints; tap one in the list to edit it (wait, loiter, RTL, speed, jump), swipe to delete, drag to reorder. **Read** / **Write**, **Start Auto**, mission speed, Mission Planner `.waypoints` load/save. **Zig zag…** (lanes from A and B) and **Patrol…** (loop with `DO_JUMP`, *Set up BendyRuler*). **Fence**: stay-in / stay-out polygons and circles, a return point, Read / Write / Enable, storage used. |
| **Params** | Full list with search; write now or stage several; load a `.parm` file (stages only what differs), save one, reboot. |
| **Setup** | TCP / UDP connection, accelerometer (6 positions or level), compass calibration, radio bars and calibration, LiDAR proximity sectors, RTK over NTRIP (over mobile data on an iPhone), set home, reboot, clear mission. |
| **LiDAR** | The LD14 scan from above (tap to zoom 2 / 4 / 8 m) and hoverboard_laser's **Avoid**, **Follow** and **Warn**, with their settings. |
| **Log** | Every `STATUSTEXT` from the autopilot, newest first. |

Safety, the same as the Android and Windows apps: letting go of the stick sends zero three times;
a LiDAR behaviour stops the car when the scan is more than 1 s old; leaving the app (or closing the
Mac window) stops any behaviour and the stick; on the Mac, held keys are forgotten when the window
loses the focus. ArduPilot keeps the last override for `RC_OVERRIDE_TIME` (3 s) if the link drops,
so set `FS_GCS_ENABLE` (README step 3).

## Build

You need a Mac with **Xcode 15 or newer** (free, Mac App Store) and the iOS platform installed
(Xcode → Settings → Components).

1. Open `apple_gcs/Esp32RoverGcs.xcodeproj` in Xcode.
2. **Mac:** choose the destination **My Mac** in the toolbar, then **Product → Run**. It is signed to
   run locally; no Apple account needed.
3. **iPhone simulator:** choose any iPhone simulator, then **Product → Run**.

After adding or removing a `.swift` file, run `python3 tools/gen_xcodeproj.py` to rewrite the project.

### On your iPhone with a free Apple ID

1. Xcode → **Settings → Accounts → +** → Apple ID, and sign in.
2. Click the project in the navigator → target **Esp32RoverGcs** → **Signing & Capabilities** → **Team**:
   choose *your name (Personal Team)*. If Xcode says the bundle identifier is taken, change
   `com.quangvu16.Esp32RoverGcs` to something of your own.
3. Connect the iPhone with a cable, unlock it and tap **Trust**. On the iPhone, turn on
   **Settings → Privacy & Security → Developer Mode** (it restarts).
4. Choose the iPhone as the destination and **Product → Run**.
5. The first time, the iPhone refuses to open an app from an unknown developer: **Settings → General →
   VPN & Device Management → your Apple ID → Trust**.

An app signed with a free Apple ID stops opening after **7 days**; run it from Xcode again to renew it.
A paid Apple Developer account ($99/year) gives a year and TestFlight.

## Connect

1. Power the board, join its WiFi **`ardupilot123`** (password `ardupilot123`). The iPhone says the
   network has no internet: stay on it.
2. Open the app. The first time, allow **Local Network** access when iOS asks: without it the app
   can't reach `192.168.4.1`. (Settings → Privacy & Security → Local Network, if you said no.)
3. It connects to TCP `192.168.4.1:5760` by itself and reconnects if the link drops.

Notes:

- **NTRIP over mobile data.** On an iPhone the NTRIP connection is made over the mobile network
  (*Over mobile data*, on by default), since the board's WiFi has no internet. Turn it off when the
  phone is on a WiFi that does have internet (SITL at home, for example).
- **UDP listen** (the board's `HAL_ESP32_WIFI 2` mode broadcasts to 14550) works on the Mac. On an
  iPhone, receiving broadcasts needs Apple's multicast entitlement, so use the board's default TCP.
- The map needs internet for its tiles. On the board's WiFi an iPhone fetches them over mobile data;
  map areas seen before stay cached.

## Files

| File | |
| --- | --- |
| `Core/Mavlink.swift` | Framing, CRC, the message table (checked against pymavlink) |
| `Core/Link.swift` | TCP and UDP on Network.framework |
| `Core/Vehicle.swift` | Telemetry, commands, parameters, mission and fence transfers |
| `Core/Lidar.swift` | Scan, hoverboard_laser behaviours, the driver that stops on a stale scan |
| `Core/Plan.swift` | Fences, zig-zag, patrol, lap counter |
| `Core/Ntrip.swift` | NTRIP 1 client and RTCM3 framing |
| `AppModel.swift` | App state, settings, events, files |
| `Views/` | The six tabs, the map, HUD and stick |
| `tools/gen_xcodeproj.py` | Writes `Esp32RoverGcs.xcodeproj` |
| `tools/build.sh` | Builds for the Mac and the iOS Simulator into `build.log`; `--watch` rebuilds on changes |
| `tools/fake_rover.py` | A pretend ArduRover on TCP `127.0.0.1:5760` for trying the apps without the car ([guide §12](../docs/GUIDE.md#12-practise-without-the-car)) |
