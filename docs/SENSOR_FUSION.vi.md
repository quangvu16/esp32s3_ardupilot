# Kết hợp cảm biến: IMU + LiDAR + GPS

[English](SENSOR_FUSION.md) | **Tiếng Việt**

Cách xe kết hợp ba cảm biến vị trí (IMU MPU9250, LiDAR LD14 và GPS ZED-F9P) thành một ước lượng
vị trí tốt hơn. Hướng dẫn cũng trình bày tính năng mới **LiDAR SLAM**, giúp xe tự hành trong nhà
mà không cần GPS.

- [1. Vì sao phải kết hợp cảm biến?](#1-vì-sao-phải-kết-hợp-cảm-biến)
- [2. ArduPilot kết hợp thế nào: bộ lọc EKF](#2-ardupilot-kết-hợp-thế-nào-bộ-lọc-ekf)
- [3. Điểm mới: LiDAR SLAM](#3-điểm-mới-lidar-slam)
- [4. Trong nhà không GPS: cài đặt](#4-trong-nhà-không-gps-cài-đặt)
- [5. Độ chính xác bao nhiêu?](#5-độ-chính-xác-bao-nhiêu)
- [6. Ngoài trời với GPS RTK](#6-ngoài-trời-với-gps-rtk)
- [7. Trong nhà và ngoài trời trên một công tắc (nâng cao)](#7-trong-nhà-và-ngoài-trời-trên-một-công-tắc-nâng-cao)
- [8. Giới hạn và xử lý sự cố](#8-giới-hạn-và-xử-lý-sự-cố)
- [9. Thử trên máy tính](#9-thử-trên-máy-tính)
- [10. Câu hỏi để hiểu sâu hơn](#10-câu-hỏi-để-hiểu-sâu-hơn)
- [Nguồn tham khảo](#nguồn-tham-khảo)

## 1. Vì sao phải kết hợp cảm biến?

Không cảm biến nào giỏi mọi thứ. Mỗi cái sai theo một kiểu riêng, nên cái này bù cho cái kia.

| Cảm biến | Giỏi | Kém | Ví dụ |
| --- | --- | --- | --- |
| **IMU** (MPU9250: con quay + gia tốc kế) | Rất nhanh (1000 lần/giây) và mượt; đo góc quay gần như hoàn hảo trong 1–2 giây | Bị trôi. Vị trí chỉ từ gia tốc kế sai hàng mét sau vài giây; hướng từ con quay trôi vài độ mỗi phút | Trong thử nghiệm bên dưới, chỉ dùng con quay thì hướng đã trôi **54°** sau 3 phút |
| **GPS** (ZED-F9P) | Vị trí tuyệt đối ở mọi nơi ngoài trời; RTK Fixed ~1–2 cm | Chỉ 5–10 lần/giây, trễ ~0,1 s, không có tín hiệu trong nhà, dưới mái hay tán cây | Trong nhà kho: không có fix |
| **LiDAR** (LD14) | Thấy tường 360°, 6 vòng/giây, sai số ±1–2 cm | Chỉ thấy vật gần (8 m); giữa bãi trống không có gì để đo | Hành lang, phòng: rất tốt. Sân bóng: không có gì để đo |
| **La bàn** (trong MPU9250) | Hướng tuyệt đối | Bị động cơ, dây pin, sắt thép làm lệch (nhất là trong nhà) | Cạnh kệ sắt có thể lệch 30° hoặc hơn |

**Vì sao kết hợp?** IMU lấp các khoảng trống *giữa* các lần đo chậm. Các phép đo tuyệt đối chậm
(GPS ngoài trời, LiDAR trong nhà) giữ cho IMU không trôi.

*Ví dụ:* ở 1 m/s, GPS cập nhật mỗi 0,2 s để lại khoảng trống 20 cm. IMU lấp đầy mượt mà, nên bộ lái
có vị trí mới ở mỗi vòng lặp chính (hàng trăm lần mỗi giây), chứ không phải 5 lần mỗi giây.

*Nguồn:* ArduPilot, "Extended Kalman Filter (EKF)" [1].

## 2. ArduPilot kết hợp thế nào: bộ lọc EKF

ArduPilot dùng **bộ lọc Kalman mở rộng (EKF3)**. Nó lặp lại hai bước:

1. **Dự đoán** bằng IMU: "Tôi đang ở đây, chạy nhanh thế này, quay thế này, vậy giờ tôi ở *kia*."
   Mỗi lần dự đoán, độ bất định tăng lên.
2. **Hiệu chỉnh** mỗi khi có phép đo: "GPS (hoặc LiDAR) nói tôi lệch trái 8 cm." EKF dời ước lượng
   một phần quãng đó. Dời bao nhiêu tùy mức tin vào mỗi bên, và độ bất định giảm xuống.

**Sao không dùng luôn phép đo mới nhất?** Vì phép đo nào cũng có nhiễu. EKF cân phép đo với dự đoán
theo độ bất định của từng bên.

*Ví dụ:* GPS báo ±2 cm (RTK Fixed) còn dự đoán ±10 cm thì EKF gần như theo GPS. GPS báo ±2 m
(không RTK) thì EKF chỉ dời một chút.

**Phép đo đến trễ.** Một lần fix GPS hay một vòng quét LiDAR mô tả quá khứ, cách đây 50–200 ms.
EKF giữ một đoạn lịch sử ngắn và hợp nhất mỗi phép đo đúng thời điểm nó được đo, rồi chạy lại IMU
đến hiện tại. Vì vậy dấu thời gian và tham số độ trễ rất quan trọng (mục 4).

**Cảm biến nào dùng cho việc gì** do các tham số *nguồn* quyết định [2]:

| Tham số | Ý nghĩa | Ngoài trời (GPS) | Trong nhà (LiDAR SLAM) |
| --- | --- | --- | --- |
| `EK3_SRC1_POSXY` | vị trí ngang | 3 = GPS | **6 = ExternalNav** |
| `EK3_SRC1_VELXY` | vận tốc ngang | 3 = GPS | 0 = không |
| `EK3_SRC1_POSZ` | độ cao | 1 = Baro hoặc 3 = GPS | 6 = ExternalNav |
| `EK3_SRC1_YAW` | hướng | 1 = La bàn hoặc 8 = GSF | **6 = ExternalNav** |

"ExternalNav" là vị trí và hướng do một nguồn bên ngoài cung cấp cho EKF. Thường đó là máy tính
đồng hành (camera, ROS). **Trong firmware này, đó là LiDAR SLAM chạy ngay trên ESP32-S3.**

## 3. Điểm mới: LiDAR SLAM

**SLAM** = định vị và lập bản đồ đồng thời: xe vẽ bản đồ các bức tường *và* tìm vị trí của mình
trong bản đồ đó, cùng một lúc.

**Sao LiDAR thường chưa đủ?** Trước đây LiDAR chỉ dùng để *tránh* vật cản (`PRX1_TYPE` 19). Nó
cho biết "có vật cách 1,2 m bên trái", nhưng không bao giờ nói "bạn đang ở (3,4; 2,1)". SLAM biến
chính các lần quét đó thành vị trí.

Cách hoạt động, mỗi vòng quét một lần (khoảng 6 lần/giây). Mã nguồn nằm trong
`libraries/AP_Proximity/LidarSlam.cpp` và `AP_Proximity_LD06.cpp`:

1. **Thu** một vòng quét: mỗi độ một điểm, tối đa 360 điểm.
2. **Nắn thẳng (deskew).** Xe di chuyển trong 0,17 s của một vòng, nên hình quét bị cong. Con quay
   (góc quay) và tốc độ xe (tiến tới) được dùng để dời từng điểm về vị trí lẽ ra thấy ở cuối vòng.
   *Ví dụ:* ở 1 m/s và quay 60°/s, điểm đầu vòng lệch 17 cm và 10°. Không nắn thì tường trông bị cong.
3. **Khớp** hình quét với bản đồ (khớp quét Gauss-Newton, phương pháp của Hector SLAM [3]). Nó trượt
   và xoay hình quét đến khi các điểm nằm trên những bức tường đã có trong bản đồ. Việc khớp đi từ
   thô đến mịn trên ba độ phân giải: 40 cm, rồi 20 cm, rồi 10 cm. Góc quay của con quay từ lần quét
   trước là điểm xuất phát.
   *Vì sao ba độ phân giải?* Bản đồ thô kéo được hình quét lệch tới 30 cm. Bản đồ mịn cho chi tiết
   tới từng xăng-ti-mét.
4. **Cập nhật bản đồ** bằng hình quét, nhưng chỉ sau khi xe đi được 30 cm hoặc quay 7°.
   *Sao không cập nhật mỗi lần quét?* Thêm cùng một cảnh nhiều lần, mỗi lần một sai số nhỏ, làm
   tường bị nhòe.
5. **Đưa tư thế cho EKF** dưới dạng ExternalNav: vị trí, hướng, thời điểm quét và độ bất định. Độ bất
   định tăng khi ít điểm rơi trúng tường đã biết (chất lượng `Q`).

Các bước 2–4 chạy trong luồng riêng nên không làm chậm vòng lặp chính (khoảng 15–30 ms mỗi lần quét
trên ESP32-S3). Bản đồ 20 m × 20 m với ô 10 cm, tốn 53 KB RAM.

**Kết hợp trong một câu:** **IMU** dự đoán ở mỗi vòng lặp, **con quay** nắn thẳng mỗi lần quét và cho
điểm xuất phát khi khớp, **LiDAR SLAM** hiệu chỉnh vị trí *và* hướng 6 lần/giây. Ngoài trời, **GPS**
làm việc hiệu chỉnh đó.

## 4. Trong nhà không GPS: cài đặt

1. **Nạp** firmware build từ nhánh này (README, mục *Build* và *Flash*).
2. **Nạp tham số:** trước hết `esp32s3_rover_ld14_f9p.parm` (nếu chưa nạp), sau đó
   [`esp32s3_rover_ld14_indoor.parm`](../esp32s3_rover_ld14_indoor.parm). Rồi đặt:
   - `PRX1_SLAM_LAT`, `PRX1_SLAM_LON`: tọa độ tòa nhà (Google Maps → chuột phải → tọa độ). Đây là gốc
     của EKF, để bản đồ và nhiệm vụ hiện đúng chỗ.
     *Vì sao cần?* Không có GPS thì EKF không có gốc, và Auto, Guided, RTL sẽ không chạy.
   - `PRX1_SLAM_X`, `PRX1_SLAM_Y`: vị trí LiDAR so với bo ESP32, đơn vị mét (phía trước, bên phải).
     *Ví dụ:* LiDAR ở trước bo 12 cm thì `PRX1_SLAM_X` = 0,12.
3. **Khởi động lại.** Đặt xe lên **vạch xuất phát**, luôn quay cùng một hướng, và để yên khoảng 5 s.
   Trong tab Log (hoặc Messages của Mission Planner) sẽ thấy:
   ```
   SLAM: map 20 m, 0.10 m cells, 51 KB
   SLAM: started, heading 0
   SLAM: tracking
   SLAM: EKF origin 21.028511 105.804817
   EKF3 IMU0 is using external nav data
   ```
   *Vì sao phải cùng vạch xuất phát?* Bản đồ được vẽ mới ở mỗi lần khởi động, bắt đầu từ chỗ xe đứng.
   Chỗ đó là (0, 0), và "hướng bắc" là hướng xe quay mặt. Nhiệm vụ lưu hôm qua chỉ đến đúng chỗ hôm
   nay nếu xe xuất phát từ cùng vị trí đó.
4. **Chạy thử đầu tiên ở Manual,** thật chậm. Xem biểu tượng xe trên bản đồ: nó phải đi theo xe.
   Đẩy xe một vòng quanh phòng để bản đồ phủ cả phòng.
5. **Rồi Auto:** vẽ một nhiệm vụ ngắn trong phòng (tab Plan), arm, chọn Auto.

Những gì cần xem trong log (bản tin `SLAM`; đặt `LOG_BACKEND_TYPE`=2 để gửi log qua MAVLink):

| Trường | Ý nghĩa | Tốt |
| --- | --- | --- |
| `X`, `Y`, `Yaw` | vị trí (m về phía bắc và đông so với điểm xuất phát) và hướng | đi theo xe |
| `Q` | tỉ lệ điểm quét rơi trúng tường đã biết | > 0,8 |
| `M` | 1 = hình quét khớp được với bản đồ | luôn là 1 |
| `Map` | 1 = lần quét này được thêm vào bản đồ | thỉnh thoảng |
| `Drop` | số lần quét bị bỏ qua vì bộ khớp còn bận | gần 0 |
| `Age` | ms từ cuối lần quét đến EKF | < 80 |

**Vì sao `EK3_FLOW_DELAY` 100?** Không có GPS, cửa sổ nhìn lại của EKF chỉ 60 ms. Tư thế SLAM đến
40–80 ms sau lần quét, nên EKF sẽ hợp nhất nó như thể nó mới hơn thực tế. Đặt độ trễ optical-flow
(không dùng tới) thành 100 ms để mở rộng cửa sổ. *Ví dụ:* ở 1 m/s, sai 40 ms về thời gian làm vị trí
chậm 4 cm.

## 5. Độ chính xác bao nhiêu?

Thử trong trình mô phỏng của ArduPilot (SITL): firmware thật, LiDAR LD06 mô phỏng trong một phòng
10 m × 8 m có một thùng, một tủ, một ghế và một đoạn tường, không GPS, không la bàn. Xe chạy 3 vòng
ở chế độ Auto, tốc độ 0,6 m/s:

![Thử SITL: đường đi thật và đường EKF trong phòng, và sai số vị trí theo thời gian](slam_sitl.svg)

| Thử nghiệm | Sai số vị trí RMS | Lớn nhất | Sai số hướng | Lần quét khớp được |
| --- | --- | --- | --- | --- |
| SITL, 3 vòng Auto, đầu ra EKF ([`sitl_indoor.py`](../Tools/lidar_slam_test/sitl_indoor.py)) | **10–12 cm** | 20 cm | < 1° | 100 % |
| Offline, 3 phút chạy vòng, nhiễu và mất điểm giống LD14, con quay lệch 0,3°/s ([`test_slam.cpp`](../Tools/lidar_slam_test/test_slam.cpp)) | 7,7 cm | 14 cm | 1,8° | 100 % |
| Như trên, 10 phút (14 vòng) | 8,2 cm | 14 cm | 2,3° | 100 % |

**Ý nghĩa:** khoảng ±10 cm ở mọi chỗ trong phòng, và sai số **không** tăng theo thời gian hay quãng
đường, vì lần quét nào cũng khớp với cùng một bản đồ.
*So sánh:* chỉ dùng con quay và tốc độ bánh (dead reckoning), hướng trôi ~0,3°/s, nên sau 10 phút xe
tưởng hướng bắc lệch 180°.

**Cẩn thận với các con số này:** trình mô phỏng sạch hơn phòng thật. LD14 thật sẽ thấy người đi lại,
kính, bề mặt đen, chân ghế. Lúc đầu nên chờ đợi kết quả kém hơn một chút, và kiểm tra bằng log của
chính bạn (ví dụ: lái về vạch xuất phát rồi đọc `X`, `Y`).

## 6. Ngoài trời với GPS RTK

Ngoài trời GPS làm việc hiệu chỉnh, nên không cần SLAM. Tắt bằng `PRX1_SLAM` 0 (giải phóng khoảng
65 KB RAM). Tránh vật cản bằng LiDAR vẫn hoạt động. Với ZED-F9P ở RTK Fixed, các tham số sau giúp
EKF tin GPS đúng mức nó đáng được tin:

| Tham số | Giá trị | Vì sao | Nguồn |
| --- | --- | --- | --- |
| `EK3_SRC1_POSXY` / `VELXY` | 3 / 3 | vị trí và vận tốc từ GPS | [2] |
| `EK3_POSNE_M_NSE` | 0,1 (mặc định 0,5) | Đây là sai số vị trí GPS *nhỏ nhất* mà EKF giả định. EKF dùng độ chính xác do GPS báo nếu nó lớn hơn, nên hạ giá trị này cho phép tính đến 1–2 cm của RTK. *Ví dụ:* để mặc định thì RTK Fixed vẫn bị coi là ±50 cm | [1], `AP_NavEKF3_PosVelFusion.cpp` |
| `GPS1_POS_X`, `_Y`, `_Z` | vị trí ăng-ten so với bo, m (trước, phải, xuống) | Nếu không đặt, khi xe quay thì ăng-ten dịch ngang và EKF tưởng xe bị trượt. *Ví dụ:* ăng-ten sau bo 20 cm sẽ văng 20 cm khi quay 90° | [4] |
| `EK3_SRC1_YAW` | 1 (la bàn) hoặc **8 (GSF)** | GSF tính hướng từ vận tốc GPS khi xe chạy, không cần la bàn. Hợp khi động cơ làm nhiễu la bàn của MPU9250 | [5] |
| `WP_RADIUS` | 0,5 | Với vị trí ±2 cm thì 0,5 m là thực tế | README, mục *Tuning* |

*Sao không dùng SLAM ngoài trời?* Ở chỗ trống, LiDAR có ít tường trong phạm vi 8 m nên khớp kém, và
xe sẽ ra khỏi bản đồ 20 m chỉ sau vài giây. Ở đó GPS mới là cảm biến tuyệt đối phù hợp.

## 7. Trong nhà và ngoài trời trên một công tắc (nâng cao)

EKF giữ được tối đa ba *bộ nguồn* và chuyển giữa chúng khi đang chạy [2]. Nạp file ngoài trời, rồi đặt:

```
PRX1_SLAM       1       # SLAM luôn chạy (khởi động lại)
EK3_SRC1_POSXY  3       # bộ 1 = ngoài trời: vị trí, vận tốc GPS; hướng la bàn hoặc GSF
EK3_SRC1_VELXY  3
EK3_SRC1_YAW    1       # hoặc 8
EK3_SRC2_POSXY  6       # bộ 2 = trong nhà: vị trí và hướng từ LiDAR SLAM
EK3_SRC2_VELXY  0
EK3_SRC2_POSZ   6
EK3_SRC2_YAW    6
EK3_FLOW_DELAY  100
RC9_OPTION      90      # công tắc 3 nấc trên kênh 9: thấp = bộ 1, giữa = bộ 2
```

Nếu EKF đã có vị trí GPS khi SLAM bắt đầu (bật ngoài trời, chờ có fix, rồi mới chạy vào nhà), điểm
(0, 0) của bản đồ đặt tại vị trí đó, nên hai bộ nguồn thống nhất. Nếu SLAM bắt đầu trước, hai bộ lệch
nhau đúng bằng khoảng cách từ gốc GPS đến điểm xuất phát.
*Vì sao là nâng cao?* Ở cửa ra vào, bạn chuyển bộ nguồn khi hai bộ có thể lệch nhau vài chục cm. Hãy
thử chậm, và chỉ chuyển khi xe đứng yên.

## 8. Giới hạn và xử lý sự cố

| Vấn đề | Vì sao | Làm gì |
| --- | --- | --- |
| `SLAM: out of memory` | Bản đồ không vừa RAM (không dùng PSRAM) | Giảm `PRX1_SLAM_SIZE` (15) hoặc tăng ô (`PRX1_SLAM_CELL` 0,15) |
| `SLAM: lost` | Hình quét không còn khớp: xe bị nhấc lên, quay quá nhanh, hoặc phần lớn là người hay kính | Chuyển Hold, lái tay về chỗ đã biết. Không hồi phục thì khởi động lại trên vạch xuất phát |
| Vị trí nhảy trong hành lang dài | Hành lang trơn trông giống nhau ở mọi mét, nên SLAM không biết đã đi bao xa | Thêm vật để nhìn (thùng, khung cửa), hoặc chạy chậm |
| Xe ra khỏi mép bản đồ | Bản đồ chỉ phủ `PRX1_SLAM_SIZE`/2 quanh điểm xuất phát | Xuất phát ở giữa khu vực, hoặc tăng `PRX1_SLAM_SIZE` |
| Hướng lệch sau khi quay rất nhanh | Con quay cho điểm xuất phát khi khớp, nhưng quay hàng trăm °/s làm hình quét cong quá mức nắn được | Giới hạn tốc độ quay, ví dụ `ATC_STR_RAT_MAX` 90 |
| `Drop` tăng liên tục | Bộ khớp chạy lâu hơn một vòng quét | Tăng ô, giảm bản đồ |
| Bản đồ vẽ lại mỗi lần khởi động | Thiết kế vậy: bản đồ nằm trong RAM | Luôn xuất phát từ cùng một vạch |

## 9. Thử trên máy tính

Hai bài thử, không cần xe (Linux, hoặc WSL trên Windows):

```bash
# 1. Riêng SLAM: LD14 mô phỏng trong một phòng, ~10 s
cd Tools/lidar_slam_test
g++ -O2 -std=c++17 -I../../libraries/AP_Proximity test_slam.cpp \
    ../../libraries/AP_Proximity/LidarSlam.cpp -o test_slam && ./test_slam 600

# 2. Toàn bộ firmware trong trình mô phỏng ArduPilot: 3 vòng Auto, không GPS
cd ../..
./waf configure --board sitl && ./waf rover
pip install pymavlink
python3 Tools/lidar_slam_test/sitl_indoor.py --laps 3
python3 Tools/lidar_slam_test/plot_run.py /tmp/slam_sitl_*/logs/00000001.BIN run.svg
```

`SIM_ROOM=1` (do `sitl_indoor.py` tự đặt) thay bãi cột của SITL bằng căn phòng.

## 10. Câu hỏi để hiểu sâu hơn

- Vì sao sai dấu thời gian chỉ 50 ms đã đáng kể ở 1 m/s, nhưng không sao khi xe đứng yên?
- Nếu EKF tin SLAM nhiều hơn (sai số nhỏ hơn), xe chính xác hơn hay chỉ "giật" hơn? Xem điều đó
  trong log thế nào?
- Vì sao SLAM sửa được độ trôi hướng, còn IMU không tự sửa được?
- Trong phòng tròn, hoặc hành lang dài trơn, việc khớp sẽ ra sao? Hướng nào không đo được, và vì sao?
- Vì sao thêm mọi lần quét vào bản đồ lại làm bản đồ *tệ hơn*?
- Ngoài trời, vì sao vận tốc GPS chỉ cho hướng (GSF) khi xe đang chạy?
- Có người đi cạnh xe, sao SLAM không tưởng bức tường di chuyển? (Gợi ý: một người chiếm bao nhiêu
  trong 360 điểm?)
- Khi nào đáng lưu bản đồ vào flash để giữ qua lần khởi động sau? Có thể sai ở đâu?

## Nguồn tham khảo

1. ArduPilot, *Extended Kalman Filter (EKF)*: https://ardupilot.org/rover/docs/common-apm-navigation-extended-kalman-filter-overview.html
2. ArduPilot, *EKF Source Selection and Switching* (`EK3_SRCx_*`, ExternalNav = 6, `RCx_OPTION` 90): https://ardupilot.org/rover/docs/common-ekf-sources.html
3. S. Kohlbrecher, O. von Stryk, J. Meyer, U. Klingauf, *A Flexible and Scalable SLAM System with Full 3D Motion Estimation*, IEEE SSRR 2011 (Hector SLAM): https://doi.org/10.1109/SSRR.2011.6106777
4. ArduPilot, *Sensor Position Offset Compensation*: https://ardupilot.org/rover/docs/common-sensor-offset-compensation.html
5. ArduPilot, *Compass-less Operation* (`EK3_SRC1_YAW` = 8, GSF): https://ardupilot.org/rover/docs/common-compassless.html
6. Mã nguồn ArduPilot trong repo này: `libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp` (`writeExtNavData`: dấu thời gian, cửa sổ nhìn lại), `AP_NavEKF3_core.cpp` (cửa sổ trễ), `AP_NavEKF3_PosVelFusion.cpp` (ngưỡng nhiễu GPS)
7. Tài liệu phát triển và SDK LDROBOT LD14 (tốc độ quét, tầm đo, định dạng gói), như dùng trong `AP_Proximity_LD06.cpp`
