# esp32s3_ardupilot

ArduPilot (ArduRover 4.6.3) for the **ESP32-S3** (`esp32s3devkit` board) driving an
**RC car** (steering servo + ESC, PPM/SBUS receiver) with an MPU9250 IMU on I2C, and:

- **LDROBOT LD14 / LD14P 360° LiDAR** as a proximity sensor — new `PRX1_TYPE = 19` (LD14)
- **u-blox ZED-F9P RTK GPS**, with NTRIP corrections sent from Mission Planner over WiFi

Based on the upstream [ArduPilot](https://github.com/ArduPilot/ardupilot) `ArduPilot-4.6`
branch at commit `913e171eca7a37549d9f02e442698edfd4490435`. The upstream README is kept
as [README_ArduPilot.md](README_ArduPilot.md).

## Changes from upstream

| File | Change |
| --- | --- |
| `libraries/AP_Proximity/AP_Proximity_LD06.{h,cpp}` | LD14 model: the LD06 packet parser plus the LDROBOT SDK ranging-centre angle correction (the LD14 is a triangulation sensor, its raw angles are off by up to ~7°), 8 m max range, no confidence filter |
| `libraries/AP_Proximity/AP_Proximity.{h,cpp}`, `AP_Proximity_Params.cpp` | `PRX_TYPE` 19 = LD14 |
| `libraries/AP_HAL_ESP32/boards/esp32s3devkit.h` | UART2 (RX GPIO41, TX GPIO42) added so SERIAL2 exists; MPU9250 IMU moved from SPI to I2C (0x68) and SPI removed, since GPIO33-37 belong to the octal PSRAM on N8R8/N16R8 modules; SD card disabled, since on the S3 it takes GPIO14/15/2 (RC input, I2C SCL, analog pin 3); ADC channel names updated for ESP-IDF 5.3 (`ADC1_CHANNEL_n` → `ADC_CHANNEL_n`), without which the board does not compile |
| `esp32s3_rover_ld14_f9p.parm` | Parameters for the RC car (servo, ESC, receiver, modes), the LiDAR, the F9P and RTK injection |
| `esp32s3-env.sh` | ESP-IDF environment for an S3-only toolchain install |

## Wiring

| Device | Pin | ESP32-S3 | ArduPilot port |
| --- | --- | --- | --- |
| Steering servo | signal | GPIO11 | SERVO1 |
| ESC | signal | GPIO9 | SERVO3 |
| (spare outputs) | | GPIO10, 8, 7, 6 | SERVO2, 4, 5, 6 |
| RC receiver | PPM or SBUS out | GPIO14 | RCIN |
| MPU9250 IMU | SDA / SCL | GPIO16 / GPIO15 | I2C bus 0, address 0x68 |
| BMP280 baro (optional) | SDA / SCL | GPIO16 / GPIO15 | I2C bus 0, address 0x77 |
| LD14 LiDAR | TX | GPIO41 | SERIAL2 (UART2) |
| LD14 LiDAR | RX | not connected | |
| ZED-F9P | UART1 TX | GPIO17 | SERIAL3 (UART1) |
| ZED-F9P | UART1 RX | GPIO18 | SERIAL3 (UART1) |
| all | GND | GND | |

- Power the servo and receiver from the ESC's BEC; join its ground to the ESP32's. Never power servos from the ESP32.
- Analog inputs (for `BATT_VOLT_PIN` etc.): analog pins 1-4 = GPIO5, 4, 2, 1, through a divider to below 3.1 V.
- No SD card: parameters are stored in the ESP32's flash. `LOG_BACKEND_TYPE=2` streams logs over MAVLink.
- Power the LiDAR (5 V, ~200 mA) from the robot's 5 V rail, not from the DevKit's 5V pin while on USB.
- ZED-F9P: leave **D_SEL** open/high. Pulled low it selects SPI and turns UART1 off; ArduPilot has no SPI GPS support.
- SERIAL1 is MAVLink over WiFi (Mission Planner connects to the board's access point).

## Build

```bash
git clone https://github.com/quangvu16/esp32s3_ardupilot.git
cd esp32s3_ardupilot
git submodule update --init --depth 1 --recursive modules/waf modules/mavlink \
    modules/DroneCAN/DSDL modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard modules/DroneCAN/pydronecan

# ESP-IDF v5.3 at the commit ArduPilot pins (Tools/scripts/esp32_get_idf.sh), shallow
mkdir -p modules/esp_idf && cd modules/esp_idf
git init -q && git remote add origin https://github.com/espressif/esp-idf.git
git fetch --depth 1 origin cc3203dc4f087ab41b434afff1ed7520c6d90993 && git checkout -q FETCH_HEAD
git submodule update --init --depth 1 components/bootloader/subproject/components/micro-ecc/micro-ecc \
    components/mbedtls/mbedtls components/lwip/lwip components/esp_wifi/lib components/esp_phy/lib \
    components/esp_coex/lib components/heap/tlsf components/spiffs/spiffs components/json/cJSON
python3 tools/idf_tools.py install xtensa-esp-elf esp-rom-elfs ninja
python3 tools/idf_tools.py install-python-env
~/.espressif/python_env/idf5.3_py3.10_env/bin/pip install empy==3.3.4 pexpect future
cd ../..

source esp32s3-env.sh          # instead of modules/esp_idf/export.sh (which wants every chip's tools)
./waf configure --board esp32s3devkit
./waf rover
```

`install-python-env` needs Python's `venv`/`ensurepip` (`sudo apt install python3-venv`).
Adjust the `idf5.3_py3.10_env` name in `esp32s3-env.sh` if your Python is not 3.10.

## Flash

Output in `build/esp32s3devkit/esp-idf_build/` (8 MB flash, DIO, 80 MHz):

| File | Offset |
| --- | --- |
| `bootloader/bootloader.bin` | `0x0` |
| `partition_table/partition-table.bin` | `0x10000` |
| `ardupilot.bin` | `0x20000` |

```bash
./waf rover --upload
# or
cd build/esp32s3devkit/esp-idf_build
python -m esptool --chip esp32s3 -b 460800 write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x0 bootloader/bootloader.bin 0x10000 partition_table/partition-table.bin 0x20000 ardupilot.bin
```

## Set up

Full step-by-step parameter guide (calibration, failsafes, tuning, avoidance):
**[ROVER_SETUP.md](ROVER_SETUP.md)**. In short:

1. Connect Mission Planner over WiFi, Config → Full Parameter List → Load from file →
   `esp32s3_rover_ld14_f9p.parm` → Write Params, reboot.
2. Object straight ahead of the robot; in the proximity view (Ctrl-F → Proximity) adjust
   `PRX1_YAW_CORR` until it shows at 0°.
3. Setup → Optional Hardware → RTK/GPS Inject → NTRIP: caster, port (usually 2101),
   mountpoint, user, password → Connect. GPS status goes 3D Fix → RTK Float → RTK Fixed.
4. Only then enable obstacle avoidance (`AVOID_ENABLE`, commented out in the .parm file).

## License

GPLv3, the same as ArduPilot — see [LICENSE](LICENSE).
