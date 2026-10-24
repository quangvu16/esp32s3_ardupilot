ESP32 Rover GCS for Windows: the ground station for this repository's ArduRover on the ESP32-S3,
built by the "Windows app (exe)" workflow. **No Java install needed**: Java is inside.

**Download one of:**

- `Esp32RoverGcs-<version>.exe`: installer. Installs for your Windows user (no administrator
  needed) and adds a Start-menu entry (*ESP32 Rover*) and a desktop shortcut.
- `Esp32RoverGcs-<version>-windows-portable.zip`: no install. Unzip anywhere (a USB stick is fine)
  and run `Esp32RoverGcs\Esp32RoverGcs.exe`.

The first time, Windows SmartScreen may say *"Windows protected your PC"*, because the app is not
signed with a paid code-signing certificate. Click **More info → Run anyway**. Windows Firewall may
ask about network access: allow **Private networks** so the app can reach the board.

Join the board's WiFi **`ardupilot123`**; the app connects to TCP `192.168.4.1:5760` by itself.

Changes in 1.3:
- The last "stop" (zero-stick) command is sent before the connection closes, so closing the
  window or Disconnect stops the car at once instead of after `RC_OVERRIDE_TIME`.
- A stale LiDAR scan stops a LiDAR behaviour with three zero commands, not one.
- Keyboard driving: held keys are forgotten when the window loses the focus, so a key released
  in another window can't leave the car driving.
