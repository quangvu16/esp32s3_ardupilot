# Indoor navigation in the apps (Nav2 style)

**English** | [Tiếng Việt](INDOOR_NAV.vi.md)

The **Indoor** tab in all four apps (Android, Windows, iPhone/iPad, Mac) drives the car round a building
with no GPS, using only its LiDAR and IMU. It follows the design of ROS 2's navigation stack,
**Nav2** [1], and the Automatic Addison Jazzy tutorial [2], but it needs no ROS and no extra computer:
everything runs in the app.

![The Indoor tab in the simulator: the saved map, the safety margin, the live scan, the path and the patrol waypoints](indoor_nav_sitl.png)

*The Windows app's Indoor tab during the simulator test: black = walls in the saved map, blue
shading = the safety margin round them, red dots = the LiDAR scan now, blue line = the planned path,
orange = patrol waypoints, blue triangle = the car.*

- [1. What it does](#1-what-it-does)
- [2. Before you start](#2-before-you-start)
- [3. Step by step](#3-step-by-step)
- [4. How each part works](#4-how-each-part-works)
- [5. How the app drives the car](#5-how-the-app-drives-the-car)
- [6. Safety](#6-safety)
- [7. Tests and results](#7-tests-and-results)
- [8. Limits and troubleshooting](#8-limits-and-troubleshooting)
- [9. Questions to go deeper](#9-questions-to-go-deeper)
- [Sources](#sources)

## 1. What it does

| You do | The app does (Nav2's part) |
| --- | --- |
| **Map new**, drive round the room, **Save map** | Builds a map from the LiDAR scans and the car's pose, and saves it as a ROS map (PGM image + YAML) (*map_saver*) |
| **Load map** | Loads it and finds the car in it (*map_server*, *AMCL*) |
| Tap the map | Plans a path round the walls and drives it (*planner*, *controller*) |
| **Tap: add waypoint**, **Patrol** | Visits the waypoints in order, round and round (*waypoint_follower*) |
| Someone walks in front | Marks them on the map, stops or plans round them (*costmap obstacle layer*, *collision checking*) |
| The car gets stuck | Clears the obstacle marks, waits, backs up, tries again (*behaviour tree recoveries*) |
| **STOP** | Zero speed, then Hold |

**Why in the app and not on the ESP32?** Planning and localisation need memory and CPU that the
ESP32-S3 already uses for flying the car. A phone has plenty. The car still does the fast work: its EKF
fuses the IMU with its own LiDAR SLAM 50 times a second ([sensor fusion guide](SENSOR_FUSION.md)), and the
app tells it where to go.

## 2. Before you start

1. Firmware with the LiDAR SLAM (the *lidar-slam* branch or newer) and these parameters. They are
   all in [`esp32s3_rover_ld14_indoor.parm`](../esp32s3_rover_ld14_indoor.parm):

   | Parameter | Value | Why |
   | --- | --- | --- |
   | `PRX1_SLAM` | 1 | The car knows where it is indoors (its own frame) |
   | `PRX1_SCAN_FULL` | 1 | Scans at 1° instead of 5° to the app: sharper maps. The app offers a **Use 1° scans** button if it is 0 |
   | `EK3_SRC1_POSXY`, `_YAW` | 6, 6 | The EKF uses the SLAM for position and heading |
   | `WP_SPEED` | 1 | Top speed in Guided mode, so also for the app (its own speed setting is lower, 0.4 m/s) |
   | `FS_GCS_ENABLE`, `FS_ACTION` | 2, 2 | Link to the app lost → Hold (section 6) |

2. A free space: start with the car in the middle of a room with walls within 8 m.

## 3. Step by step

1. **Connect** and open the **Indoor** tab.
2. **Map new.** Put the car on its start mark, facing the way it will always face at the start. Drive
   slowly (walking pace, ≤ 0.5 m/s) round the room in Manual mode, or with the stick on the Data tab.
   Go along every wall once. The map grows on screen: black = walls, white = free, grey = not seen yet.
   *Why slowly?* Each scan is matched to where the car was. At walking pace the error is a few
   centimetres; fast turns smear the walls.
3. **Save map** with a name, for example `living_room`. The map is kept in the app (Android: the app's
   files; Windows: `%APPDATA%\Esp32RoverGcs\maps`; iPhone/Mac: Application Support), as
   `living_room.pgm` + `.yaml` + `.json`. They open in RViz or Nav2 too.
4. **Next time: Load map.** If the car is on its start mark, the app already knows where it is. If
   not, use **Set pose**: on the phone, press and hold where the car is, then drag the way it faces. On
   Windows, right-drag. On iPhone/Mac, switch **Set pose** on, then drag. Or use **Find me**, which
   searches the whole map, then drive a metre so it can confirm.
   *Example:* "pose ±0.06 m, match 95%" under the map means it is sure. Particles (small blue dots)
   show while it is unsure.
5. **Arm**, then **tap the map** where the car should go, and confirm. The app plans a path (blue line)
   and drives it in Guided mode.
6. **Waypoints and patrol.** Switch to **Tap: add waypoint**, tap a few places (P1, P2, P3…), then
   **Patrol**: the car visits them in order, waits 3 s at each, and starts again. **Waypoints…** lists
   them (go to one, delete one). They are saved with the map.
7. **STOP** at any time: zero speed, then Hold.

## 4. How each part works

### Map (Nav2: map_saver / map_server)

Every LiDAR ray lowers the "occupied" score of the cells it passes through (free) and raises the
score of the cell it ends in (a wall). This is log-odds ray tracing.

- *Why not just draw the points?* One wrong point would become a wall for ever. With scores, a wall
  needs to be seen several times, and a person who walked past is erased by later rays going through
  where they stood.
- The pose for each ray comes from the car's own LiDAR SLAM, at the moment that degree was
  measured. One rotation takes 0.17 s, and the car moves during it.
- A ray with no return clears only 3 m, and never through a known wall. *Why?* Black or shiny walls can
  give no return.

**Saved as** ROS map_server files [3]: a PGM image (254 free, 0 occupied, 205 unknown) and a YAML file
(resolution 0.1 m, origin, thresholds 0.65 / 0.196).

### Localisation (Nav2: AMCL)

Each boot, the car's own frame starts again where it stands, so the app must find where that frame
sits in the saved map. It uses a particle filter (AMCL [4]): 600 guesses ("particles") of the car's
place.

1. Every guess moves with the car's own motion, plus a little noise.
2. Every guess is scored: put the scan there, and see how close its points land to walls on the map.
   The app uses Nav2's *likelihood field* model: a precomputed distance-to-wall for every cell, so
   scoring is one table look-up per point.
3. Bad guesses die out and good ones are copied (resampling).

The result is a small correction: map ← car frame. Nav2 calls it map → odom.

| Setting | Nav2 default | Here | Why different |
| --- | --- | --- | --- |
| particles | 500–2000 | 600 | the car's own odometry (SLAM) is good |
| beams | 60 | 72 | all the 5° directions |
| sigma_hit | 0.2 m | 0.15 m | the LD14 is accurate to ~2 cm |
| update every | 0.25 m / 0.2 rad | 0.10 m / 0.10 rad | slow car, small rooms |
| odometry noise α | 0.2 | 0.1 | SLAM odometry drifts little |

*Example:* the car starts 40 cm and 8° away from its usual mark. AMCL corrects that in about 2 m of
driving (offline test: error 8 cm, 0.6°).

### Costmap (Nav2: costmap_2d)

A grid of costs from 0 to 254, rebuilt about 5 times a second:

- **Static layer:** the walls of the saved map.
- **Obstacle layer:** what the LiDAR sees now (people, boxes, a closed door), out to 3.5 m. It is
  cleared when later rays see through it, or after 8 s.
- **Inflation:** cost rises near anything, as Nav2's formula: 252·e^(−k·(d − r)) [5]. Here *r* is
  the car radius (0.20 m) and *k* is the cost scaling (4). The margin reaches 0.55 m from walls.

*Why inflate?* The planner treats the car as a dot. Inflation makes the dot "fat" (cost 253 means
touching) and makes paths prefer the middle of a corridor.
*Example:* in a 1.2 m doorway the path goes through the middle, where the cost is lowest.

### Path (Nav2: planner, Smac 2D)

A* search on the costmap, in 8 directions. Each step costs its length × (1 + 2 × cost / 252), so it
avoids squeezing past walls when there is room [6]. The zig-zag grid path is then smoothed and given
every 10 cm. The path is planned again **every second** (Nav2's default behaviour tree), so new
obstacles are planned round.

### Following the path (Nav2: Regulated Pure Pursuit)

Steer towards a point 0.35–0.9 m ahead on the path (further when faster). Turn curvature =
2·y / distance². Then the speed is **regulated** (reduced) [7]:

- on tight curves (radius below 0.9 m);
- near obstacles (from the costmap);
- when arriving (the last 0.6 m);
- and limited to the steering's tightest turn (Rover's `TURN_RADIUS`).

The arc ahead (1.5 s) is checked against the costmap; if it would hit something, the car stops.

*Why RPP and not DWB or MPPI?* Nav2 lists RPP for Ackermann (car-like) robots. DWB is for robots that
turn on the spot. MPPI needs thousands of trial paths per step [8]. A car **can't turn on the spot**:
when the path starts behind it, the app does a 3-point turn (reverse with the wheels turned, then
forward).

### Recoveries and patrols (Nav2: bt_navigator, waypoint_follower)

If the car can't make progress (0.5 m in 10 s), or no path exists, the app tries in turn: clear the
obstacle layer → wait 5 s → back up 0.3 m. It does this up to 6 times (Nav2's default tree [9]). A
patrol then skips that waypoint and goes on (`stop_on_failure` false); a single goal gives up and
holds.

## 5. How the app drives the car

The app sends **SET_POSITION_TARGET_LOCAL_NED** 10 times a second in Guided mode: a forward speed
and a turn rate in the car's frame (type_mask 1511). This is the MAVLink version of Nav2's
`cmd_vel` [10].

- *Why not send positions?* With a speed and a turn rate the app keeps control of slowing down and
  collision checks.
- *Why is this safe?* Rover stops by itself if these messages stop for 3 s [10].

## 6. Safety

| What happens | What the app / car does |
| --- | --- |
| No pose or no scan for 1 s | Stops: zero speed, then Hold |
| Unsure where it is for 2 s (particles spread > 0.25 m, or < 40% of the scan on walls) | Stops: "lost in the map" |
| Someone changes the mode (RC switch, another app) | Stops navigating |
| The arc ahead hits something | Stops, then plans round it |
| The app freezes or the WiFi drops | Rover stops after 3 s without speed commands. With `FS_GCS_ENABLE` it goes to Hold when the heartbeats stop (`FS_GCS_TIMEOUT`, 5 s by default) [11] |
| Always | Keep the RC transmitter in hand: switching to Manual or Hold overrides the app |

Start with the default 0.4 m/s and raise it only when everything behaves.

## 7. Tests and results

**Offline** (`android_gcs/core/src/test/.../NavOfflineTest.kt`): a simulated car (turn radius 0.9 m)
and LD14 in a 10 × 8 m room.

| Test | Result |
| --- | --- |
| Map: occupied cells on real walls | 98% |
| PGM + YAML + JSON save and load | identical |
| AMCL from a start 0.4 m / 8° off | error 8 cm, 0.6° |
| Find me (whole map, no hint) | error 2 cm, 86 ms |
| To a goal behind the car (3-point turn) | arrived, never closer than 0.6 m to a wall |
| Past a box that is not on the map | planned round it, never closer than 0.5 m |
| Patrol A → B → C twice | done, never closer than 0.26 m |

**Simulator** (`Tools/lidar_slam_test/sitl_nav.py`): the real firmware in ArduPilot SITL (simulated
LD06 in the room, no GPS, no compass) and the apps' real navigation code over MAVLink. It maps the
room, saves, reboots the autopilot, loads the map, drives to B, then patrols A → B → C. The app's
pose in the map against the simulator's truth: about **13–15 cm RMS**. Run it yourself:

```bash
./waf configure --board sitl && ./waf rover
# compile android_gcs/core (main + test) with kotlinc, then:
python3 Tools/lidar_slam_test/sitl_nav.py --classpath core.jar:test.jar:kotlin-stdlib.jar
```

## 8. Limits and troubleshooting

| Problem | Why | What to do |
| --- | --- | --- |
| "Waiting for the car's position (EKF on the LiDAR SLAM)" | The EKF isn't using the SLAM yet (just after boot), or `PRX1_SLAM` is 0 | Wait a few seconds after boot; check the indoor parameters |
| Map has smeared, doubled walls | Drove too fast, or bumped the car | Map new, slower |
| "Not sure where the car is" | Not on the start mark | Set pose, or Find me and drive a metre |
| "lost in the map" while driving | Furniture moved a lot, or many people | Set pose again; make a new map if the room changed |
| Car stops with "obstacle ahead" in a narrow gap | The gap is narrower than 2 × car radius + margin | Lower "Car radius" or "Keep away" in Settings, carefully |
| Car swings wide in turns | Its tightest turn (`TURN_RADIUS`) is larger than the path's curves | Set `TURN_RADIUS` to the real value; leave more space |
| Nothing happens on tap | Not armed, or not localised | The status line says which |

## 9. Questions to go deeper

- Why does AMCL need the car to move before it can be sure? What does a still car not tell it?
- Why does the app plan again every second instead of once? What would go wrong with a person
  walking past?
- If inflation is too big, what happens in a doorway? And if it is too small?
- Why is a speed + turn-rate command safer than sending the next point?
- Why does Pure Pursuit cut corners at a long lookahead, and wobble at a short one?
- When would you move localisation onto the ESP32 itself (send it the saved map)? What would
  you gain, and what would it cost?
- What would change for a skid-steer robot that can spin on the spot?

## Sources

1. Nav2 (ROS 2 navigation stack): https://github.com/ros-navigation/navigation2 · docs https://docs.nav2.org
2. Automatic Addison, *Autonomous Navigation for a Mobile Robot Using ROS 2 Jazzy*: https://automaticaddison.com/autonomous-navigation-for-a-mobile-robot-using-ros-2-jazzy/
3. ROS map_server map format (YAML + PGM, which Nav2 keeps): https://robo-marc.github.io/navigation_documents/map_server.html
4. Nav2 AMCL configuration: https://docs.nav2.org/configuration/packages/configuring-amcl.html
5. Nav2 inflation layer: https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
6. Nav2, *Setting up navigation plugins* (planners: NavFn, Smac 2D, Smac Hybrid-A*; controllers and the robots each suits): https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/
7. Nav2 Regulated Pure Pursuit: https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html
8. As [6]: RPP is listed for "Differential, Ackermann, Legged" robots, DWB for "Differential, Omnidirectional"
9. Nav2 default behaviour tree (replanning at 1 Hz, recoveries): https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
10. ArduPilot Rover, Guided mode commands (velocity targets, 3 s timeout): https://ardupilot.org/dev/docs/mavlink-rover-commands.html
11. ArduPilot Rover failsafes (GCS failsafe): https://ardupilot.org/rover/docs/rover-failsafes.html
