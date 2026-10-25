# Tests of the shared core

Plain `main` programs (no test framework needed), compiled with `kotlinc`:

```bash
cd android_gcs
kotlinc core/src/main/kotlin -d core.jar
kotlinc core/src/test/kotlin -cp core.jar -d test.jar
# indoor navigation, offline: simulated car and LiDAR in a room (exit code 0 = pass)
java -cp core.jar:test.jar:$KOTLIN_HOME/lib/kotlin-stdlib.jar org.ardupilot.esp32gcs.nav.NavOfflineTestKt
# indoor navigation against ArduPilot SITL (starts SITL itself; build it first with ./waf rover)
python3 ../Tools/lidar_slam_test/sitl_nav.py --classpath core.jar:test.jar:$KOTLIN_HOME/lib/kotlin-stdlib.jar
```

`NavOfflineTest` maps a 10 m × 8 m room from known poses, saves and loads it (PGM + YAML + JSON),
finds the car with AMCL from a wrong start and with no hint at all, drives to a goal behind the car
(3-point turn) and past a box that is not on the map, and runs a patrol twice round.

`NavSitlTest` (run by `sitl_nav.py`) does the same with the real firmware in SITL over MAVLink: maps
the room on an Auto lap, saves the map, reboots the autopilot, loads the map, drives to B and patrols
A → B → C. `sitl_nav.py` then compares the app's pose with the simulator's truth from the log.
`desktop/src/test/.../IndoorShots.kt` runs it and saves pictures of the Windows app's Indoor tab.
