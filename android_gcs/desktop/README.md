# ESP32 Rover GCS for Windows

![Windows app: Fly, Plan and LiDAR tabs](docs/screens.svg)

The drawing is made by `python3 docs/gen_app_screens.py` (from the repository root) from the app's layout.

The desktop version of the [Android app](../README.md), for a Windows PC (or any computer with
Java 17). It has the same core as the phone app: the same MAVLink, missions, zig-zag, patrol,
fences and LiDAR behaviours, so they work the same way. Calibration and RTK/NTRIP are not here:
use Mission Planner for those on a PC.

## Install and run

**Easiest: the `.exe`, with Java inside** (no Java install). From the
[releases](https://github.com/quangvu16/esp32s3_ardupilot/releases) (or the *Windows app (exe)*
workflow's artifacts), take one of:

- `Esp32RoverGcs-<version>.exe`: installer. Installs for your Windows user (no administrator
  needed), with a Start-menu entry (*ESP32 Rover*) and a desktop shortcut.
- `Esp32RoverGcs-<version>-windows-portable.zip`: no install. Unzip anywhere and run
  `Esp32RoverGcs\Esp32RoverGcs.exe`.

The first time, Windows SmartScreen may say *"Windows protected your PC"* (the app isn't signed
with a paid certificate): **More info → Run anyway**. If Windows Firewall asks, allow **Private
networks**.

**Or the jar**, if the PC has Java 17 or newer: put `Esp32RoverGcs.jar` and `Esp32RoverGcs.bat` in
one folder and double-click `Esp32RoverGcs.bat` (or `java -jar Esp32RoverGcs.jar`).

Then join the board's WiFi **`ardupilot123`** (password `ardupilot123`). The app connects to TCP
`192.168.4.1:5760` by itself. The address is at the top left; ArduPilot SITL is `127.0.0.1:5760`.

Settings (connection, map type, zig-zag points, LiDAR settings) are remembered per Windows user.
Map tiles are cached in `%USERPROFILE%\.esp32gcs\tiles`.

## Tabs

| Tab | What it does |
| --- | --- |
| **Fly** | HUD, mode buttons, Arm/Disarm, the map with the car, its trail, home, the mission, fences and the LiDAR scan. Click the map in Guided to drive there; right-click to drive there or set home. **Drive with arrow keys** (or W A S D) sends stick overrides, like the phone's Drive stick, capped by *Power*. |
| **Plan** | Click the map to add waypoints, drag them, click one to edit it; the list on the right (double-click to edit). **Read** / **Write**, **Speed…**, **Zig zag…**, **Patrol…**, **Start Auto**, Mission Planner `.waypoints` files. **Fence** switches to fence editing: **+ Stay-in**, **+ Stay-out**, **+ Circle**, **Read**, **Write**, **Enable…**. Right-click the map for zig-zag points A and B. |
| **LiDAR** | The LD14 scan seen from above (click it to zoom) and the `hoverboard_laser` behaviours: **Avoid**, **Follow**, **Warn** (beeps), with their settings. Holding an arrow key pauses the behaviour. |
| **Params** | All parameters with search. Edit the *To write* column, then **Write**. Load and save `.parm` files; **Load** keeps only the values that differ from the vehicle. **Reboot**. |
| **Log** | Messages from the autopilot, newest first. |

How zig-zag, patrol, fences and the LiDAR behaviours work, and the settings that matter, is in the
[guide](../../docs/GUIDE.md).

## Build

From `android_gcs/`, on the PC used for the Android app (offline Gradle cache):

```bash
export JAVA_HOME=~/Android/jdk
~/Android/gradle-8.7/bin/gradle --no-daemon --offline :desktop:fatJar
# -> desktop/build/libs/Esp32RoverGcs-<version>.jar  (rename it to Esp32RoverGcs.jar next to dist/Esp32RoverGcs.bat)
```

On any machine with internet, `./gradlew :desktop:fatJar` works too (`GCS_DESKTOP_ONLY=1` skips
the Android app, so no Android SDK is needed).

The `.exe` files are made on Windows by `.github/workflows/windows-gcs.yml`: it builds the jar,
wraps it with `jpackage` (JDK 17) into `Esp32RoverGcs.exe` with its own Java runtime and the icon in
`desktop/packaging/`, starts it once as a smoke test, then builds the installer. Pushing a tag
`gcs-win-v<version>` publishes both as a release. The shared code is in
`android_gcs/core`; the desktop window is `desktop/src/main/kotlin/.../desktop/` (Swing, no
libraries besides Kotlin).

## How it was tested

The core is the one the Android app uses and was tested against ArduRover SITL (see the
[Android README](../README.md#how-it-was-tested)). The desktop window itself was run against SITL on
Linux, drawing each tab off-screen to check it shows the live vehicle, the map, the scan and the
plans; it has not been tried on Windows yet.
