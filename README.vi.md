# esp32s3_ardupilot

[English](README.md) | **Tiếng Việt**

ArduPilot (ArduRover 4.6.3) cho **ESP32-S3** (board `esp32s3devkit`) điều khiển một
**xe RC** (servo lái + ESC, bộ thu PPM/SBUS), với IMU MPU9250 qua I2C, và:

- **LiDAR 360° LDROBOT LD14 / LD14P** làm cảm biến tiệm cận — kiểu mới `PRX1_TYPE = 19` (LD14)
- **GPS RTK u-blox ZED-F9P**, nhận dữ liệu hiệu chỉnh NTRIP do Mission Planner gửi qua WiFi

Dựa trên nhánh `ArduPilot-4.6` của [ArduPilot](https://github.com/ArduPilot/ardupilot) gốc,
tại commit `913e171eca7a37549d9f02e442698edfd4490435`. README gốc của ArduPilot được giữ
lại trong [README_ArduPilot.md](README_ArduPilot.md).

## Thay đổi so với bản gốc

| Tệp | Thay đổi |
| --- | --- |
| `libraries/AP_Proximity/AP_Proximity_LD06.{h,cpp}` | Kiểu LD14: dùng bộ giải mã gói tin của LD06, cộng thêm phép hiệu chỉnh góc theo tâm đo của SDK LDROBOT (LD14 là cảm biến tam giác lượng, góc thô lệch tới ~7°), tầm xa tối đa 8 m, không lọc theo độ tin cậy |
| `libraries/AP_Proximity/AP_Proximity.{h,cpp}`, `AP_Proximity_Params.cpp` | `PRX_TYPE` 19 = LD14 |
| `libraries/AP_HAL_ESP32/boards/esp32s3devkit.h` | Thêm UART2 (RX GPIO41, TX GPIO42) để có SERIAL2; IMU MPU9250 chuyển từ SPI sang I2C (0x68) và bỏ SPI, vì GPIO33-37 thuộc về PSRAM octal trên module N8R8/N16R8; tắt thẻ SD, vì trên S3 thẻ SD chiếm GPIO14/15/2 (đầu vào RC, I2C SCL, chân analog 3); đổi tên kênh ADC cho ESP-IDF 5.3 (`ADC1_CHANNEL_n` → `ADC_CHANNEL_n`), nếu không board sẽ không biên dịch được |
| `esp32s3_rover_ld14_f9p.parm` | Tham số cho xe RC (servo, ESC, bộ thu, chế độ lái), LiDAR, F9P và truyền dữ liệu RTK |
| `esp32s3-env.sh` | Môi trường ESP-IDF cho bộ công cụ chỉ cài riêng cho S3 |

## Đấu dây

![Sơ đồ đấu dây: ESP32-S3 DevKitC-1 với servo, ESC, bộ thu RC, MPU9250, ZED-F9P, LiDAR LD14 và nguồn](docs/wiring.svg)

(Nhãn trong sơ đồ bằng tiếng Anh; bảng bên dưới có đầy đủ thông tin bằng tiếng Việt.)

| Thiết bị | Chân | ESP32-S3 | Cổng ArduPilot |
| --- | --- | --- | --- |
| Servo lái | tín hiệu | GPIO11 | SERVO1 |
| ESC | tín hiệu | GPIO9 | SERVO3 |
| (ngõ ra dự phòng) | | GPIO10, 8, 7, 6 | SERVO2, 4, 5, 6 |
| Bộ thu RC | ngõ ra PPM hoặc SBUS | GPIO14 | RCIN |
| IMU MPU9250 | SDA / SCL | GPIO16 / GPIO15 | I2C bus 0, địa chỉ 0x68 |
| Cảm biến áp suất BMP280 (tùy chọn) | SDA / SCL | GPIO16 / GPIO15 | I2C bus 0, địa chỉ 0x77 |
| LiDAR LD14 | TX | GPIO41 | SERIAL2 (UART2) |
| LiDAR LD14 | RX | không nối | |
| ZED-F9P | UART1 TX | GPIO17 | SERIAL3 (UART1) |
| ZED-F9P | UART1 RX | GPIO18 | SERIAL3 (UART1) |
| tất cả | GND | GND | |

- Cấp nguồn cho servo và bộ thu từ BEC của ESC; nối mass (GND) của nó với ESP32. Không bao giờ cấp nguồn servo từ ESP32.
- Ngõ vào analog (cho `BATT_VOLT_PIN`...): chân analog 1-4 = GPIO5, 4, 2, 1, qua cầu chia áp để dưới 3,1 V.
- Không có thẻ SD: tham số được lưu trong flash của ESP32. `LOG_BACKEND_TYPE=2` gửi log qua MAVLink.
- Cấp nguồn cho LiDAR (5 V, ~200 mA) từ đường 5 V của xe, không dùng chân 5V của DevKit khi đang cắm USB.
- ZED-F9P: để chân **D_SEL** hở/mức cao. Nếu kéo xuống thấp, module chuyển sang SPI và tắt UART1; ArduPilot không hỗ trợ GPS qua SPI.
- SERIAL1 là MAVLink qua WiFi (Mission Planner kết nối vào điểm phát WiFi của board).

## Biên dịch

```bash
git clone https://github.com/quangvu16/esp32s3_ardupilot.git
cd esp32s3_ardupilot
git submodule update --init --depth 1 --recursive modules/waf modules/mavlink \
    modules/DroneCAN/DSDL modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard modules/DroneCAN/pydronecan

# ESP-IDF v5.3 tại commit mà ArduPilot chỉ định (Tools/scripts/esp32_get_idf.sh), tải nông
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

source esp32s3-env.sh          # thay cho modules/esp_idf/export.sh (vốn đòi công cụ của mọi loại chip)
./waf configure --board esp32s3devkit
./waf rover
```

`install-python-env` cần `venv`/`ensurepip` của Python (`sudo apt install python3-venv`).
Sửa tên `idf5.3_py3.10_env` trong `esp32s3-env.sh` nếu Python của bạn không phải 3.10.

## Nạp firmware

### Cần chuẩn bị

- Board ESP32-S3 DevKit và cáp USB **truyền dữ liệu** (một số cáp chỉ để sạc).
- Firmware từ trang [Releases](https://github.com/quangvu16/esp32s3_ardupilot/releases).
  Lần nạp đầu tiên chỉ cần tệp `ardurover-esp32s3devkit-factory.bin`.
- `esptool`, công cụ nạp của Espressif: `pip install esptool` (mọi hệ điều hành có Python 3).
  Hoặc nạp ngay trên trình duyệt, không cần cài gì (cách C bên dưới).

Dùng cổng có ghi **UART** trên DevKit (cạnh chip chuyển USB-serial CP2102 hoặc CH343): nạp
ổn định và hiện được thông báo khởi động của ArduPilot. Cổng ghi **USB** cũng nạp được, nhưng
console của ArduPilot không nằm ở cổng đó.

| Tệp trong Release | Địa chỉ nạp | Khi nào |
| --- | --- | --- |
| `ardurover-esp32s3devkit-factory.bin` | `0x0` | Lần nạp đầu; gồm bootloader + bảng phân vùng + firmware trong một tệp |
| `ardurover-esp32s3devkit.bin` | `0x20000` | Cập nhật (chỉ firmware) |
| `bootloader.bin`, `partition-table.bin` | `0x0`, `0x10000` | Chỉ khi nạp riêng ba phần |

Bố cục flash: 8 MB, DIO, 80 MHz. Tham số được lưu ở `0x320000`, nằm sau firmware, nên nạp
tệp nào ở trên cũng **giữ nguyên tham số**. Chỉ `erase_flash` mới xóa chúng.

### A. Linux

```bash
# 1. cho phép user mở cổng serial (làm một lần; sau đó đăng xuất rồi đăng nhập lại)
sudo usermod -aG dialout $USER

# 2. kiểm tra tệp đã tải
sha256sum -c SHA256SUMS --ignore-missing

# 3. cắm cổng UART và tìm tên cổng: thường là /dev/ttyUSB0 (hoặc /dev/ttyACM0)
ls /dev/ttyUSB* /dev/ttyACM*

# 4. chỉ lần nạp đầu: xóa toàn bộ flash (xóa luôn tham số cũ)
python3 -m esptool --chip esp32s3 --port /dev/ttyUSB0 erase_flash

# 5. nạp
python3 -m esptool --chip esp32s3 --port /dev/ttyUSB0 -b 460800 \
    write_flash 0x0 ardurover-esp32s3devkit-factory.bin
```

### B. Windows

1. Nếu máy không nhận board, cài driver USB-serial: CP210x (Silicon Labs) hoặc CH343 (WCH),
   tùy chip nằm cạnh cổng UART.
2. Cắm cổng UART. Xem số cổng **COM** trong Device Manager → Ports (COM & LPT).
3. Mở Command Prompt trong thư mục chứa các tệp đã tải (thay `COM5` bằng cổng của bạn):
   ```bat
   certutil -hashfile ardurover-esp32s3devkit-factory.bin SHA256
   python -m esptool --chip esp32s3 --port COM5 erase_flash
   python -m esptool --chip esp32s3 --port COM5 -b 460800 write_flash 0x0 ardurover-esp32s3devkit-factory.bin
   ```
   So mã của certutil với dòng tương ứng trong `SHA256SUMS`. Bỏ qua `erase_flash` nếu muốn giữ
   tham số.

### C. Nạp trên trình duyệt (không cần cài đặt)

1. Mở https://espressif.github.io/esptool-js/ bằng **Chrome hoặc Edge**.
2. Baudrate `460800` → **Connect** → chọn cổng serial của board.
3. Flash Address `0x0`, chọn `ardurover-esp32s3devkit-factory.bin` → **Program**.
   (Bấm **Erase Flash** trước nếu muốn bắt đầu sạch.)

### Cập nhật về sau

Chỉ nạp firmware. Tham số và hiệu chuẩn được giữ nguyên:

```bash
python3 -m esptool --chip esp32s3 --port /dev/ttyUSB0 -b 460800 write_flash 0x20000 ardurover-esp32s3devkit.bin
```

### Kiểm tra hoạt động

1. Bấm **RST** (hoặc rút ra cắm lại).
2. Trong khoảng 10 giây sẽ xuất hiện mạng WiFi **`ardupilot123`**. Kết nối Mission Planner bằng
   TCP tới `192.168.4.1:5760` rồi làm tiếp phần **Cài đặt tham số** bên dưới.
3. Muốn xem thông báo khởi động, mở cổng UART ở tốc độ 115200:
   ```bash
   python3 -m serial.tools.miniterm /dev/ttyUSB0 115200     # Ctrl+] để thoát
   ```

### Xử lý sự cố

| Hiện tượng | Cách xử lý |
| --- | --- |
| `Failed to connect to ESP32-S3: No serial data received` | Đưa board vào chế độ nạp bằng tay: giữ **BOOT**, bấm rồi nhả **RST**, nhả **BOOT**, rồi chạy lại lệnh. Bấm **RST** sau khi nạp xong. |
| `Permission denied: '/dev/ttyUSB0'` | Linux: `sudo usermod -aG dialout $USER`, rồi đăng xuất và đăng nhập lại. |
| Không thấy cổng serial | Thử cáp khác (phải truyền được dữ liệu) và cổng USB còn lại. Trên Windows cài driver CP210x/CH343. |
| Cổng đang bận | Đóng Mission Planner, Arduino IDE, miniterm hoặc chương trình khác đang dùng cổng. |
| Nạp bị dừng giữa chừng | Giảm tốc độ: `-b 115200`. |
| Board khởi động lại liên tục, hoặc không có WiFi sau khi nạp | Chạy `erase_flash`, rồi nạp lại tệp factory. |
| Mission Planner không kết nối được | Máy tính phải đang ở WiFi `ardupilot123`. Dùng TCP, không phải UDP, tới `192.168.4.1` cổng `5760`. |

### Nạp bản tự biên dịch

Sau khi chạy `./waf rover` (xem phần **Biên dịch**), cắm board vào cổng UART:

```bash
./waf rover --upload
# hoặc
cd build/esp32s3devkit/esp-idf_build
python -m esptool --chip esp32s3 -b 460800 write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB \
  0x0 bootloader/bootloader.bin 0x10000 partition_table/partition-table.bin 0x20000 ardupilot.bin
```

## Cài đặt tham số

Các tham số ArduRover cho xe RC, theo đúng thứ tự cần cài. Tên tham số là của ArduRover 4.6;
cách đấu dây xem phần **Đấu dây** ở trên. Tên menu trong Mission Planner được giữ bằng tiếng
Anh, đúng như trên màn hình.

### 0. Kết nối

1. Cấp nguồn cho ESP32-S3. Board tự phát mạng WiFi **`ardupilot123`** (mật khẩu `ardupilot123`).
2. Cho máy tính kết nối vào mạng đó. Trong Mission Planner chọn **TCP** → `192.168.4.1`, cổng `5760`.
3. **Config → Full Parameter List → Load from file** → [`esp32s3_rover_ld14_f9p.parm`](esp32s3_rover_ld14_f9p.parm)
   → **Write Params**. Khởi động lại board rồi kết nối lại.

### 1. Phần cứng bắt buộc (Setup → Mandatory Hardware)

| Bước | Việc cần làm |
| --- | --- |
| Loại khung | Đã cài sẵn: `FRAME_CLASS=1` (Rover) |
| Hiệu chuẩn gia tốc kế | Trước tiên gắn **cố định board trên xe**, sau đó hiệu chuẩn 6 tư thế. Nếu board không nằm phẳng với mũi tên hướng về phía trước, cài `AHRS_ORIENTATION` trước khi hiệu chuẩn. |
| La bàn | Hiệu chuẩn bằng cách xoay xe theo mọi hướng. Nếu không tìm thấy la bàn (nhiều board "MPU9250" thực ra là MPU6500), xem bước 5. |
| Hiệu chuẩn tay điều khiển | Gạt mọi cần và công tắc tới hết hành trình. Kiểm tra kênh 1 = lái, kênh 3 = ga, kênh 8 = công tắc chế độ. |
| Chế độ lái | Đã cài sẵn trên kênh 8: Manual / Auto / RTL. Thêm **Hold** làm vị trí an toàn, ví dụ `MODE2=4`. |

### 2. Kiểm tra lái và ga (nhấc bánh xe khỏi mặt đất, chế độ Manual)

| Hiện tượng | Cách sửa |
| --- | --- |
| Gạt cần sang phải nhưng bánh xe quay sang trái | `SERVO1_REVERSED=1` |
| Xe không đi thẳng khi cần lái ở giữa | Chỉnh `SERVO1_TRIM` từng chút một (quanh 1500) |
| Servo lái bị kẹt ở hai đầu hành trình | Thu hẹp `SERVO1_MIN` / `SERVO1_MAX` cho tới khi servo không còn gồng |
| Đẩy ga tới nhưng xe chạy lùi | `SERVO3_REVERSED=1` |
| Bánh xe nhích khi ga ở giữa | Hiệu chuẩn điểm trung tính của ESC (đa số ESC xe RC tự học điểm này khi bật nguồn với cần ga ở giữa). Giữ `SERVO3_TRIM=1500` |
| Phải đẩy ga nhiều thì động cơ mới chạy | Tăng `MOT_THR_MIN` (%) cho tới khi chỉ cần gạt nhẹ là xe bắt đầu lăn bánh |
| Xe tăng tốc quá gắt | `MOT_SLEWRATE` (%/giây, ví dụ 100) |

**Arm (kích hoạt):** cần ga ở giữa (bằng 0) và cần lái **gạt hết sang phải trong 2 giây**.
Disarm bằng cách gạt hết sang trái. Lần arm đầu tiên hãy nhấc bánh xe lên khỏi mặt đất.

### 3. Failsafe (cài trước khi chạy dưới đất)

| Tham số | Giá trị | Lý do |
| --- | --- | --- |
| `FS_THR_ENABLE` | `1` | Phản ứng khi mất tín hiệu RC |
| `FS_THR_VALUE` | `910` | Cài **failsafe của chính bộ thu** sao cho kênh 3 xuống dưới mức này (ví dụ 900 µs) khi tắt tay điều khiển |
| `FS_GCS_ENABLE` | `1` | Phản ứng khi mất kết nối WiFi với Mission Planner |
| `FS_ACTION` | `2` (Hold) | Dừng xe. `1` = RTL (quay về), chỉ dùng khi GPS đã hoạt động tốt |
| `FS_TIMEOUT` | `1.5` | Số giây trước khi failsafe kích hoạt |
| `FS_CRASH_CHECK` | `2` | Dừng và disarm nếu xe bị kẹt vào vật cản |

Thử nghiệm: arm khi bánh xe đang nhấc khỏi mặt đất, tắt tay điều khiển, và kiểm tra bánh xe dừng lại.

### 4. GPS (ZED-F9P)

Đã cài sẵn trong tệp tham số. Kiểm tra Mission Planner báo `3D Fix` khi ở ngoài trời. Với RTK,
dùng **Setup → Optional Hardware → RTK/GPS Inject → NTRIP** (caster, cổng thường là 2101,
mountpoint, tên đăng nhập, mật khẩu). Trạng thái sẽ chuyển 3D Fix → RTK Float → RTK Fixed.
ArduPilot tự cấu hình F9P vì `GPS_AUTO_CONFIG=1`.

### 5. Hướng xe khi không có la bàn hoạt động

Nếu không có la bàn, hoặc la bàn bị nhiễu bởi động cơ và dây điện:

```
COMPASS_ENABLE = 0
EK3_SRC1_YAW   = 8     # GSF: ước lượng hướng từ GPS khi xe đang chạy
```

Sau khi arm, xe phải chạy thẳng vài mét thì chế độ Auto mới có hướng chính xác. Nếu la bàn
hoạt động tốt, giữ `COMPASS_ENABLE=1` và gắn board xa động cơ và dây pin.

### 6. Theo dõi pin (tùy chọn)

Nối pin qua cầu chia áp bằng điện trở vào GPIO5 (chân analog 1), giữ điện áp ở chân dưới 3,1 V:

```
BATT_MONITOR    = 3     # chỉ đo điện áp analog
BATT_VOLT_PIN   = 1
BATT_VOLT_MULT  = ...   # chỉnh cho tới khi điện áp trên Mission Planner khớp với đồng hồ đo
BATT_LOW_VOLT   = ...   # ví dụ 3,5 V x số cell
BATT_FS_LOW_ACT = 2     # Hold
```

### 7. Tinh chỉnh (ngoài trời, nơi rộng rãi)

Làm theo thứ tự sau. Trang **Config → Tuning** và đồ thị trực tiếp (Ctrl-F → Tuning) của
Mission Planner cho thấy giá trị yêu cầu so với giá trị thực tế. Muốn có log thì cần
`LOG_BACKEND_TYPE=2` (log qua MAVLink), vì không có thẻ SD.

1. **Ga hành trình.**
   - Gán `RC7_OPTION=50` (Learn Cruise) cho một công tắc.
   - Chạy thẳng ở chế độ Manual với tốc độ đều, rồi bật công tắc. `CRUISE_SPEED` và
     `CRUISE_THROTTLE` được cài theo lần chạy đó.
2. **Tốc độ quay vòng (chế độ Acro, ví dụ `MODE3=1`).**
   - `TURN_RADIUS`: bán kính vòng cua hẹp nhất của xe, tính bằng mét (mặc định 0,9).
   - `ATC_STR_RAT_MAX`: tốc độ quay lớn nhất bạn muốn, tính bằng °/giây (ví dụ 120).
   - `ATC_STR_RAT_FF`: tăng dần cho tới khi tốc độ quay thực tế bám theo tốc độ yêu cầu. Sau đó
     thêm một chút `ATC_STR_RAT_P` (~0,1–0,2) và `ATC_STR_RAT_I` (tương tự).
3. **Điều khiển tốc độ (chế độ Acro).**
   - Bắt đầu với `ATC_SPEED_P=0.2` và `ATC_SPEED_I=0.2`. Tăng dần cho tới khi tốc độ giữ ổn
     định mà không bị giật.
   - `ATC_ACCEL_MAX` và `ATC_DECEL_MAX` tính bằng m/s², ví dụ 1,0 để chạy êm.
4. **Dẫn đường (chế độ Auto).**
   - `WP_SPEED`: tốc độ chạy, m/s.
   - `WP_RADIUS`: khoảng cách được coi là "đã tới" điểm (≥ `TURN_RADIUS`; với RTK, 0,5 m là được).
   - `WP_ACCEL`: 0 = dùng `ATC_ACCEL_MAX`.
   - Nếu xe cắt góc hoặc ôm cua quá rộng, trước hết hãy xem `TURN_RADIUS` và `ATC_STR_ANG_P`.

### 8. Tránh vật cản bằng LiDAR (làm sau cùng)

1. Đặt một vật phía trước xe. Trong màn hình tiệm cận (Ctrl-F → Proximity), chỉnh
   `PRX1_YAW_CORR` sao cho vật hiển thị ở 0°.
2. Sau đó:
   ```
   AVOID_ENABLE = 3     # hàng rào + tiệm cận: dừng hoặc giảm tốc trước vật cản
   AVOID_MARGIN = 0.5   # m
   OA_TYPE      = 1     # BendyRuler: lái vòng qua vật cản ở chế độ Auto (tùy chọn)
   ```
3. Thử chậm ở chế độ Manual trước: xe phải từ chối lao vào một cái hộp.

### Danh sách kiểm tra trước mỗi lần chạy

- [ ] Công tắc chế độ ở Manual hoặc Hold
- [ ] GPS 3D Fix (hoặc RTK) và trạng thái EKF màu xanh
- [ ] Đã thử failsafe của bộ thu
- [ ] Pin đã sạc đầy, và chỉ đặt xe xuống đất khi đã xác nhận đúng chiều lái và chiều ga

## Giấy phép

GPLv3, giống ArduPilot — xem [LICENSE](LICENSE).
