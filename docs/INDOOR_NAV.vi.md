# Tự hành trong nhà trên ứng dụng (kiểu Nav2)

[English](INDOOR_NAV.md) | **Tiếng Việt**

Tab **Indoor** có trên cả bốn ứng dụng (Android, Windows, iPhone/iPad, Mac). Nó giúp xe chạy trong nhà
mà không cần GPS, chỉ dùng LiDAR và IMU. Thiết kế theo bộ điều hướng **Nav2** của ROS 2 [1] và bài
hướng dẫn Jazzy của Automatic Addison [2], nhưng không cần ROS và không cần thêm máy tính: mọi thứ
chạy ngay trong ứng dụng.

![Tab Indoor trong trình mô phỏng: bản đồ đã lưu, vùng an toàn, vòng quét LiDAR, đường đi và các điểm tuần tra](indoor_nav_sitl.png)

*Tab Indoor của ứng dụng Windows trong bài thử mô phỏng:*

- đen = tường trên bản đồ đã lưu
- tô xanh = vùng an toàn quanh tường
- chấm đỏ = vòng quét LiDAR hiện tại
- đường xanh = đường đi đã lập
- cam = điểm tuần tra
- tam giác xanh = chiếc xe

- [1. Nó làm gì](#1-nó-làm-gì)
- [2. Trước khi bắt đầu](#2-trước-khi-bắt-đầu)
- [3. Từng bước](#3-từng-bước)
- [4. Từng phần hoạt động thế nào](#4-từng-phần-hoạt-động-thế-nào)
- [5. Ứng dụng lái xe thế nào](#5-ứng-dụng-lái-xe-thế-nào)
- [6. An toàn](#6-an-toàn)
- [7. Thử nghiệm và kết quả](#7-thử-nghiệm-và-kết-quả)
- [8. Giới hạn và xử lý sự cố](#8-giới-hạn-và-xử-lý-sự-cố)
- [9. Câu hỏi để hiểu sâu hơn](#9-câu-hỏi-để-hiểu-sâu-hơn)
- [Nguồn tham khảo](#nguồn-tham-khảo)

## 1. Nó làm gì

| Bạn làm | Ứng dụng làm (phần tương ứng của Nav2) |
| --- | --- |
| **Map new**, lái quanh phòng, **Save map** | Vẽ bản đồ từ các lần quét LiDAR và tư thế xe, lưu dạng bản đồ ROS (ảnh PGM + YAML) (*map_saver*) |
| **Load map** | Nạp bản đồ và tìm vị trí xe trong đó (*map_server*, *AMCL*) |
| Chạm vào bản đồ | Lập đường tránh tường và lái theo (*planner*, *controller*) |
| **Tap: add waypoint**, **Patrol** | Đi qua các điểm theo thứ tự, vòng đi vòng lại (*waypoint_follower*) |
| Có người đi ngang trước xe | Đánh dấu lên bản đồ, dừng hoặc lập đường vòng qua (*lớp vật cản costmap*, *kiểm tra va chạm*) |
| Xe bị kẹt | Xoá dấu vật cản, chờ, lùi lại, thử lại (*các bước phục hồi của behaviour tree*) |
| **STOP** | Tốc độ về 0, rồi chuyển Hold |

**Sao chạy trên ứng dụng mà không chạy trên ESP32?** Lập đường và định vị cần bộ nhớ và CPU, mà
ESP32-S3 đang dùng để điều khiển xe. Điện thoại thì dư sức. Xe vẫn làm phần việc nhanh: EKF của xe kết
hợp IMU với LiDAR SLAM 50 lần/giây ([hướng dẫn kết hợp cảm biến](SENSOR_FUSION.vi.md)), còn ứng dụng
bảo xe đi đâu.

## 2. Trước khi bắt đầu

1. Firmware có LiDAR SLAM (nhánh *lidar-slam* trở về sau) với các tham số sau, đều có trong
   [`esp32s3_rover_ld14_indoor.parm`](../esp32s3_rover_ld14_indoor.parm):

   | Tham số | Giá trị | Vì sao |
   | --- | --- | --- |
   | `PRX1_SLAM` | 1 | Xe biết mình ở đâu trong nhà (theo khung của chính nó) |
   | `PRX1_SCAN_FULL` | 1 | Gửi vòng quét 1° thay vì 5° cho ứng dụng: bản đồ nét hơn. Nếu đang là 0, ứng dụng có nút **Use 1° scans** |
   | `EK3_SRC1_POSXY`, `_YAW` | 6, 6 | EKF lấy vị trí và hướng từ SLAM |
   | `WP_SPEED` | 1 | Tốc độ tối đa ở chế độ Guided, tức cũng là tối đa cho ứng dụng (cài đặt của ứng dụng thấp hơn, 0,4 m/s) |
   | `FS_GCS_ENABLE`, `FS_ACTION` | 2, 2 | Mất kết nối với ứng dụng → Hold (mục 6) |

2. Chỗ trống: lúc đầu, đặt xe giữa một phòng có tường trong phạm vi 8 m.

## 3. Từng bước

1. **Kết nối**, mở tab **Indoor**.
2. **Map new.** Đặt xe lên vạch xuất phát, quay đúng hướng mà lần nào xe cũng quay khi xuất phát. Lái
   chậm (tốc độ đi bộ, ≤ 0,5 m/s) quanh phòng ở chế độ Manual, hoặc bằng cần lái ở tab Data. Đi dọc mỗi
   bức tường một lần. Bản đồ lớn dần trên màn hình: đen = tường, trắng = trống, xám = chưa thấy.
   *Vì sao phải chậm?* Mỗi lần quét được khớp với vị trí xe lúc đó. Ở tốc độ đi bộ sai số chỉ vài cm;
   quay nhanh làm tường bị nhoè.
3. **Save map**, đặt tên, ví dụ `phong_khach`. Bản đồ lưu trong ứng dụng (Android: thư mục của ứng dụng;
   Windows: `%APPDATA%\Esp32RoverGcs\maps`; iPhone/Mac: Application Support) thành các file
   `phong_khach.pgm`, `.yaml` và `.json`. Mở được bằng RViz hoặc Nav2.
4. **Lần sau: Load map.** Nếu xe đứng trên vạch xuất phát, ứng dụng đã biết xe ở đâu. Nếu không, dùng
   **Set pose**: trên điện thoại, nhấn giữ ở chỗ xe đứng rồi kéo theo hướng xe quay. Trên Windows, kéo
   bằng chuột phải. Trên iPhone/Mac, bật **Set pose** rồi kéo. Hoặc bấm **Find me**: ứng dụng tìm trên
   cả bản đồ, rồi lái xe một mét để nó xác nhận.
   *Ví dụ:* dòng "pose ±0.06 m, match 95%" dưới bản đồ nghĩa là ứng dụng đã chắc chắn. Khi chưa chắc,
   ứng dụng hiện các "hạt" (chấm xanh nhỏ).
5. **Arm**, rồi **chạm vào bản đồ** chỗ muốn xe đến và xác nhận. Ứng dụng lập đường (đường xanh) và lái
   theo ở chế độ Guided.
6. **Điểm và tuần tra.** Chuyển sang **Tap: add waypoint**, chạm vài chỗ (P1, P2, P3…), rồi bấm
   **Patrol**: xe đi qua lần lượt, chờ 3 giây ở mỗi điểm, rồi lặp lại. **Waypoints…** liệt kê các điểm
   (đi đến một điểm, xoá một điểm). Các điểm được lưu cùng bản đồ.
7. **STOP** bất cứ lúc nào: tốc độ về 0, rồi Hold.

## 4. Từng phần hoạt động thế nào

### Bản đồ (Nav2: map_saver / map_server)

Mỗi tia LiDAR làm giảm điểm "có vật" ở các ô nó đi qua (trống), và tăng điểm ở ô nó dừng (tường). Đó là
dò tia log-odds.

- *Sao không vẽ thẳng các điểm?* Một điểm sai sẽ thành tường mãi mãi. Dùng điểm số thì một bức tường
  phải được thấy nhiều lần mới hiện, và người đã đi qua sẽ bị xoá khi các tia sau xuyên qua chỗ họ đứng.
- Tư thế dùng cho mỗi tia lấy từ LiDAR SLAM của xe, đúng lúc góc đó được đo. Một vòng quét mất 0,17 s,
  và xe di chuyển trong lúc đó.
- Tia không có phản hồi chỉ xoá trống 3 m, và không bao giờ xuyên qua tường đã biết. *Vì sao?* Tường
  đen hoặc bóng có thể không phản hồi.

**Lưu** theo định dạng map_server của ROS [3]: ảnh PGM (254 trống, 0 có vật, 205 chưa biết) và file YAML
(độ phân giải 0,1 m, gốc toạ độ, ngưỡng 0,65 / 0,196).

### Định vị (Nav2: AMCL)

Mỗi lần khởi động, khung toạ độ của xe bắt đầu lại từ chỗ nó đứng, nên ứng dụng phải tìm xem khung đó
nằm đâu trên bản đồ đã lưu. Nó dùng bộ lọc hạt (AMCL [4]): 600 "hạt", mỗi hạt là một phỏng đoán vị trí xe.

1. Mỗi hạt di chuyển theo chuyển động của xe, cộng thêm chút nhiễu.
2. Mỗi hạt được chấm điểm: đặt vòng quét vào vị trí đó, xem các điểm rơi gần tường trên bản đồ đến
   đâu. Ứng dụng dùng mô hình *likelihood field* của Nav2: khoảng cách đến tường của mỗi ô được tính
   sẵn, nên chấm một điểm chỉ cần tra một bảng.
3. Hạt kém bị loại, hạt tốt được nhân bản (resampling).

Kết quả là một phép hiệu chỉnh nhỏ: map ← khung của xe. Nav2 gọi là map → odom.

| Thiết lập | Mặc định Nav2 | Ở đây | Vì sao khác |
| --- | --- | --- | --- |
| số hạt | 500–2000 | 600 | quãng đường đo từ SLAM của xe đã tốt |
| số tia | 60 | 72 | đủ mọi hướng 5° |
| sigma_hit | 0,2 m | 0,15 m | LD14 chính xác khoảng 2 cm |
| cập nhật mỗi | 0,25 m / 0,2 rad | 0,10 m / 0,10 rad | xe chậm, phòng nhỏ |
| nhiễu chuyển động α | 0,2 | 0,1 | SLAM ít trôi |

*Ví dụ:* xe xuất phát lệch 40 cm và 8° so với vạch quen. AMCL sửa trong khoảng 2 m (bài thử offline:
sai 8 cm, 0,6°).

### Costmap (Nav2: costmap_2d)

Một lưới chi phí từ 0 đến 254, dựng lại khoảng 5 lần mỗi giây:

- **Lớp tĩnh:** tường trên bản đồ đã lưu.
- **Lớp vật cản:** những gì LiDAR đang thấy (người, thùng, cửa đóng) trong 3,5 m. Bị xoá khi các tia
  sau nhìn xuyên qua, hoặc sau 8 giây.
- **Lớp phồng (inflation):** chi phí tăng khi gần vật, theo công thức Nav2: 252·e^(−k·(d − r)) [5].
  Trong đó *r* là bán kính xe (0,20 m), *k* là hệ số (4). Vùng an toàn rộng 0,55 m tính từ tường.

*Sao phải "phồng"?* Bộ lập đường coi xe như một chấm. Lớp phồng biến chấm thành "béo" (chi phí 253 nghĩa
là chạm), và khiến đường đi ưu tiên giữa hành lang.
*Ví dụ:* qua cửa rộng 1,2 m, đường đi giữa cửa, nơi chi phí thấp nhất.

### Lập đường (Nav2: planner, Smac 2D)

Tìm kiếm A* trên costmap theo 8 hướng. Mỗi bước tốn chiều dài × (1 + 2 × chi phí / 252), nên tránh lách
sát tường khi còn chỗ rộng [6]. Đường zig-zag theo ô lưới được làm mượt và chia mỗi 10 cm. Đường được lập
lại **mỗi giây** (theo behaviour tree mặc định của Nav2), nên có vật cản mới thì đi vòng.

### Đi theo đường (Nav2: Regulated Pure Pursuit)

Lái hướng về một điểm trên đường cách xe 0,35–0,9 m (đi nhanh thì nhìn xa hơn). Độ cong = 2·y /
khoảng cách². Rồi tốc độ được **điều tiết** (giảm) [7]:

- ở khúc cua gắt (bán kính dưới 0,9 m);
- gần vật cản (theo costmap);
- khi sắp đến nơi (0,6 m cuối);
- và giới hạn theo vòng quay hẹp nhất của bánh lái (`TURN_RADIUS` của Rover).

Cung đường phía trước (1,5 s) được kiểm tra trên costmap; nếu sắp chạm thì xe dừng.

*Sao dùng RPP mà không dùng DWB hay MPPI?* Nav2 xếp RPP cho robot kiểu ô tô (Ackermann). DWB dành cho
robot quay tại chỗ được. MPPI cần thử hàng nghìn đường mỗi bước [8]. Ô tô **không quay tại chỗ được**:
khi đường bắt đầu phía sau xe, ứng dụng quay đầu 3 bước (lùi với bánh lái bẻ, rồi tiến).

### Phục hồi và tuần tra (Nav2: bt_navigator, waypoint_follower)

Nếu xe không tiến được (0,5 m trong 10 s), hoặc không có đường, ứng dụng lần lượt thử: xoá lớp vật cản
→ chờ 5 s → lùi 0,3 m. Làm vậy tối đa 6 lần (theo cây mặc định của Nav2 [9]). Khi đang tuần tra, ứng
dụng bỏ qua điểm đó và đi tiếp (`stop_on_failure` false); với một đích đơn lẻ thì bỏ cuộc và giữ yên.

## 5. Ứng dụng lái xe thế nào

Ứng dụng gửi **SET_POSITION_TARGET_LOCAL_NED** 10 lần mỗi giây ở chế độ Guided: tốc độ tiến và tốc độ
quay trong khung của xe (type_mask 1511). Đây là phiên bản MAVLink của `cmd_vel` trong Nav2 [10].

- *Sao không gửi toạ độ?* Gửi tốc độ và tốc độ quay thì ứng dụng tự quyết việc giảm tốc và kiểm tra va
  chạm.
- *Sao an toàn?* Rover tự dừng nếu các lệnh này ngừng 3 giây [10].

## 6. An toàn

| Chuyện xảy ra | Ứng dụng / xe làm gì |
| --- | --- |
| Không có tư thế hoặc vòng quét trong 1 s | Dừng: tốc độ 0, rồi Hold |
| Không chắc mình ở đâu trong 2 s (hạt toả rộng > 0,25 m, hoặc < 40% điểm quét trúng tường) | Dừng: "lost in the map" |
| Ai đó đổi chế độ (công tắc RC, ứng dụng khác) | Ngừng điều hướng |
| Cung đường phía trước sắp chạm vật | Dừng, rồi lập đường vòng |
| Ứng dụng treo hoặc mất WiFi | Rover dừng sau 3 s không có lệnh tốc độ. Với `FS_GCS_ENABLE`, xe chuyển Hold khi mất heartbeat (`FS_GCS_TIMEOUT`, mặc định 5 s) [11] |
| Luôn luôn | Cầm tay điều khiển RC: chuyển Manual hoặc Hold là ghi đè ứng dụng |

Bắt đầu với 0,4 m/s mặc định; chỉ tăng khi mọi thứ ổn.

## 7. Thử nghiệm và kết quả

**Offline** (`android_gcs/core/src/test/.../NavOfflineTest.kt`): xe mô phỏng (bán kính quay 0,9 m) và LD14
trong phòng 10 × 8 m.

| Bài thử | Kết quả |
| --- | --- |
| Bản đồ: ô "có vật" nằm đúng trên tường thật | 98% |
| Lưu và nạp PGM + YAML + JSON | giống hệt |
| AMCL khi xuất phát lệch 0,4 m / 8° | sai 8 cm, 0,6° |
| Find me (cả bản đồ, không gợi ý) | sai 2 cm, 86 ms |
| Đến đích phía sau xe (quay đầu 3 bước) | đến nơi, không bao giờ gần tường dưới 0,6 m |
| Vòng qua một thùng không có trên bản đồ | lập đường vòng, không bao giờ gần dưới 0,5 m |
| Tuần tra A → B → C hai vòng | xong, không bao giờ gần dưới 0,26 m |

**Trình mô phỏng** (`Tools/lidar_slam_test/sitl_nav.py`): firmware thật trong ArduPilot SITL (LD06 mô
phỏng trong phòng, không GPS, không la bàn) và mã điều hướng thật của ứng dụng qua MAVLink. Bài thử vẽ
bản đồ, lưu, khởi động lại autopilot, nạp bản đồ, đi đến B, rồi tuần tra A → B → C. Tư thế ứng dụng tính
được so với thực tế mô phỏng: khoảng **13–15 cm RMS**.

## 8. Giới hạn và xử lý sự cố

| Vấn đề | Vì sao | Làm gì |
| --- | --- | --- |
| "Waiting for the car's position (EKF on the LiDAR SLAM)" | EKF chưa dùng SLAM (vừa khởi động), hoặc `PRX1_SLAM` = 0 | Chờ vài giây sau khi khởi động; kiểm tra tham số trong nhà |
| Bản đồ có tường nhoè, tường đôi | Lái quá nhanh, hoặc xe bị va | Map new, chạy chậm hơn |
| "Not sure where the car is" | Xe không ở vạch xuất phát | Set pose, hoặc Find me rồi lái một mét |
| "lost in the map" khi đang chạy | Đồ đạc dời nhiều, hoặc đông người | Set pose lại; phòng thay đổi thì vẽ bản đồ mới |
| Xe dừng "obstacle ahead" ở khe hẹp | Khe hẹp hơn 2 × bán kính xe + khoảng an toàn | Giảm "Car radius" hoặc "Keep away" trong Settings, cẩn thận |
| Xe ôm cua rộng | Vòng quay hẹp nhất của xe (`TURN_RADIUS`) lớn hơn khúc cua của đường | Đặt `TURN_RADIUS` đúng thực tế; chừa nhiều chỗ hơn |
| Chạm vào bản đồ không có gì xảy ra | Chưa arm, hoặc chưa định vị xong | Dòng trạng thái cho biết lý do |

## 9. Câu hỏi để hiểu sâu hơn

- Vì sao AMCL cần xe di chuyển mới chắc chắn được? Xe đứng yên thì thiếu thông tin gì?
- Vì sao ứng dụng lập lại đường mỗi giây thay vì một lần? Nếu có người đi ngang thì sao?
- Lớp phồng quá lớn thì ở cửa ra vào chuyện gì xảy ra? Quá nhỏ thì sao?
- Vì sao lệnh tốc độ + tốc độ quay an toàn hơn gửi điểm kế tiếp?
- Vì sao Pure Pursuit nhìn xa thì cắt cua, nhìn gần thì lắc?
- Khi nào nên chuyển việc định vị vào chính ESP32 (gửi bản đồ đã lưu xuống xe)? Được gì, mất gì?
- Với robot bánh xích quay tại chỗ được thì điều gì thay đổi?

## Nguồn tham khảo

1. Nav2 (bộ điều hướng ROS 2): https://github.com/ros-navigation/navigation2 · tài liệu https://docs.nav2.org
2. Automatic Addison, *Autonomous Navigation for a Mobile Robot Using ROS 2 Jazzy*: https://automaticaddison.com/autonomous-navigation-for-a-mobile-robot-using-ros-2-jazzy/
3. Định dạng bản đồ map_server của ROS (YAML + PGM, Nav2 giữ nguyên): https://robo-marc.github.io/navigation_documents/map_server.html
4. Cấu hình AMCL của Nav2: https://docs.nav2.org/configuration/packages/configuring-amcl.html
5. Lớp inflation của Nav2: https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
6. Nav2, *Setting up navigation plugins* (planner: NavFn, Smac 2D, Smac Hybrid-A*; controller và loại robot phù hợp): https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/
7. Regulated Pure Pursuit của Nav2: https://navigation.ros.org/configuration/packages/configuring-regulated-pp.html
8. Như [6]: RPP dành cho robot "Differential, Ackermann, Legged", DWB cho "Differential, Omnidirectional"
9. Behaviour tree mặc định của Nav2 (lập lại đường 1 Hz, phục hồi): https://github.com/ros-navigation/navigation2/blob/main/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
10. ArduPilot Rover, lệnh ở chế độ Guided (lệnh vận tốc, hết hạn sau 3 s): https://ardupilot.org/dev/docs/mavlink-rover-commands.html
11. ArduPilot Rover, failsafe (GCS failsafe): https://ardupilot.org/rover/docs/rover-failsafes.html
