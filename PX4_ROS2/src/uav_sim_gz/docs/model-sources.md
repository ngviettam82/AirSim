# Nguồn số liệu & giới hạn của mô hình

> **Nguyên tắc:** dự án này hướng tới bay thật, nên **số không có nguồn thì không được duyệt**. Số nào chưa xác minh phải ghi rõ là chưa xác minh, thay vì điền đại cho đủ chỗ.
>
> Đây chính là tiêu chuẩn ta đã áp cho bản nộp của đồng nghiệp, nên bản của ta chịu cùng tiêu chuẩn.
>
> Cập nhật: 2026-08-10 · Gazebo Harmonic (Sim 8.14.0) · PX4 v1.15.4 · ROS2 Humble

---

## 1. Thông số nào nằm ở file nào

| Thông số | File | Thẻ XML |
|---|---|---|
| Khối lượng, quán tính thân | `models/uav0_frame/` → `model://x500` | `<inertial><mass>`, `<inertia>` |
| Hằng số motor kT, kM, ω_max | như trên (trong `x500`) | `<plugin ...MulticopterMotorModel>` |
| Hình học rotor, phân bổ điều khiển | `airframes/4100_gz_uav0` (kế thừa `4001_gz_x500`) | `CA_ROTOR*`, `CA_ROTOR_COUNT` |
| Ga hover khởi tạo | `airframes/4001_gz_x500` của PX4 | `MPC_THR_HOVER` |
| Vị trí lắp cảm biến | `models/uav0_nav/`, `models/uav0_full/` | `<include><pose relative_to="base_link">` |
| Tầm đo & nhiễu rangefinder | `models/sensor_lidar_down/model.sdf` | `<lidar><range>`, `<noise>` |
| Độ phân giải & nhiễu camera dưới | `models/sensor_camera_down/model.sdf` | `<camera><image>`, `<noise>` |
| Camera trước RGB + depth | `models/sensor_camera_front/`, `models/sensor_depth_front/` — **của dự án** (mesh mượn `OakD-Lite`) | `<sensor type="camera">`, `type="depth_camera"` |
| Bật/tắt GPS, baro, mag mô phỏng | `airframes/4001_gz_x500` | `SENS_EN_GPSSIM`, `SENS_EN_BAROSIM`, `SENS_EN_MAGSIM` |
| Trọng lực, timestep, engine vật lý | `worlds/uav_arena.sdf` | `<physics>`, `<gravity>` |
| Từ trường, toạ độ địa lý | `worlds/uav_arena.sdf` | `<magnetic_field>`, `<spherical_coordinates>` |

---

## 2. Số liệu và nguồn

### 2.1 Thân drone — ✅ có nguồn, ⚠️ nhưng KHÔNG phải drone của dự án

Toàn bộ kế thừa model `x500` của PX4 v1.15.4, đọc trực tiếp từ `PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf` và `x500/model.sdf` (verify 2026-08-03, đối chiếu lại 2026-08-10).

| Thông số | Giá trị | Đơn vị |
|---|---|---|
| `mass` | 2.0 | kg |
| `ixx` = `iyy` | 0.02166666 | kg·m² |
| `izz` | 0.04 | kg·m² |
| `motorConstant` (kT) | 8.54858e-06 | N·s²/rad² |
| `momentConstant` (kM) | 0.016 | m |
| `maxRotVelocity` | 1000.0 | rad/s |
| `timeConstantUp` / `Down` | 0.0125 / 0.025 | s |
| `rotorDragCoefficient` | 8.06428e-05 | — |
| `rollingMomentCoefficient` | 1e-06 | — |
| `MPC_THR_HOVER` | 0.60 | — |

> 🔴 **Đây là số MƯỢN.** Drone thật của dự án sẽ do team phần cứng vẽ CAD. Mọi kết luận về lực đẩy, thời gian đáp ứng, biên điều khiển hiện chỉ đúng với x500. Phải đo lại từ CAD + bench test trước khi bay thật (P11).

### 2.2 World — ✅ có nguồn

Kế thừa nguyên khối từ `PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf` (v1.15.4).

| Thông số | Giá trị | Ghi chú |
|---|---|---|
| Physics engine | `ode` | PX4 chọn |
| `max_step_size` | 0.004 s (250 Hz) | Chuẩn PX4 cho multicopter |
| `real_time_update_rate` | 250 | — |
| `gravity` | `0 0 -9.8` | m/s² |
| `magnetic_field` | `6e-06 2.3e-05 -4.2e-05` | T, ứng với toạ độ dưới |
| Toạ độ gốc | 47.3980°N, 8.5462°E, cao độ 0 | Zurich — mặc định của PX4 |
| `atmosphere` | `adiabatic` | — |
| Gió | **không bật** | Wind system chưa nạp |

> ⚠️ Toạ độ gốc là của PX4 (Zurich). Từ trường khai trong world khớp với toạ độ đó. **Đổi sang toạ độ Việt Nam thì phải đổi luôn `magnetic_field`**, nếu không EKF2 sẽ lệch heading.

### 2.3 Cảm biến ta tự thêm — ⚠️ TOÀN BỘ LÀ PLACEHOLDER

**Chưa chọn thiết bị thật.** Các số dưới đây là giá trị hợp lý để mô phỏng chạy được, **không lấy từ datasheet nào**. Phải thay bằng số của thiết bị thật khi chốt mua.

#### `sensor_lidar_down` (rangefinder hướng xuống)

| Thông số | Giá trị | Cơ sở |
|---|---|---|
| `mass` | 0.010 kg | ⚠️ ước lượng, chưa xác minh |
| `ixx`/`iyy` | 1.354e-06 | ✅ tính từ công thức khối hộp `m(b²+c²)/12` với 35×35×20 mm |
| `izz` | 2.042e-06 | ✅ như trên |
| `range.min` | 0.05 m | ✅ chọn theo **khoảng hở càng đáp đo được 0.177 m** (biên 3.5×) |
| `range.max` | 12.0 m | ⚠️ chưa xác minh |
| `resolution` | 0.01 m | ⚠️ chưa xác minh |
| `noise stddev` | 0.01 m | ⚠️ chưa xác minh |
| `update_rate` | 20 Hz | ⚠️ chưa xác minh |
| Hướng | pitch +1.5708 rad | ✅ **verify bằng thực nghiệm**: probe static ở z=3.0 đọc đúng 3.000 m; pitch âm nhìn lên trời |

#### `sensor_camera_down` (camera hướng xuống)

| Thông số | Giá trị | Cơ sở |
|---|---|---|
| `mass` | 0.020 kg | ⚠️ ước lượng |
| Quán tính | 3.0e-06 cả 3 trục | ✅ công thức khối lập phương 30 mm |
| `horizontal_fov` | 1.2217 rad (70°) | ⚠️ chưa xác minh |
| Độ phân giải | 640×480 | ⚠️ chưa xác minh |
| `noise stddev` | 0.007 | ⚠️ chưa xác minh |
| `update_rate` | 30 Hz | ⚠️ chưa xác minh |

#### Camera trước — `sensor_camera_front` + `sensor_depth_front` (của dự án)

Trước 2026-08-13 `uav0_full` lấy nguyên `model://OakD-Lite` của PX4 (RGB **1920×1080@30**, depth 640×480@30). ❌ **Đã bỏ:** đó là mặc định của PX4, không phải lựa chọn của ta, và đo được rằng **RGB lớn tick cùng nhịp với depth** kéo mean RTF xuống **0,701** — dưới ngưỡng 0,95 cần để tin một phép đo bay. Chi tiết truy nguyên: [`../../../docs/package-status.md`](../../../docs/package-status.md) §4.

Nay là hai mô-đun của dự án, **cùng 640×480 @ 15 Hz**, hfov 1.274 rad, depth clip 0.2–19.1 m:

| Số | Nguồn | Độ tin |
|---|---|---|
| mass 0.061 kg, hình dáng (mesh) | mượn `model://OakD-Lite` của PX4 | ✅ có nguồn |
| 640×480 @ 15 Hz | **chọn theo phép đo RTF**, không theo datasheet | ⚠️ ràng buộc tính toán, không phải thông số thiết bị |
| hfov, clip | giữ nguyên của OakD-Lite | ✅ có nguồn |

⚠️ **Chưa chắc là camera dự án sẽ dùng.** Và lưu ý: độ phân giải hiện tại được chọn vì **máy chạy sim**, nên khi thay thiết bị thật thì phải đặt lại theo datasheet **rồi đo lại RTF**, đừng chép số này sang.

`sensor_depth_front` khai `mass 0.001 kg` (gần như 0) vì đây là **cặp stereo trên cùng board** với RGB — khối lượng thật đã tính hết vào `sensor_camera_front` (0.061 kg), không phải bỏ sót.

#### Marker ArUco (`scripts/generate_aruco_marker.py`)

Dictionary chọn **`DICT_4X4_50`**, không phải 7×7 hay lớn hơn như hay thấy trong tài liệu ArUco: ở độ cao bay của dự án, **số pixel/ô quyết định giải mã được hay không, không phải code space**. Camera dưới 640 px ngang trên FOV 70°, ở 2.5 m ra ~183 px/m; marker 0.5 m ⇒ ~91 px, chia cho lưới 6×6 ô của 4×4 (~15 px/ô) so với lưới 9×9 của 7×7 (~10 px/ô). 50 ID là dư so với nhu cầu dự án. Bảng theo quy ước bệ đáp: board 1 m, marker 0.5 m ở giữa (viền trắng bắt buộc — ArUco cần "quiet zone" quanh viền đen).

> **Ảnh hưởng của khối lượng cảm biến thêm vào:** `uav0_nav` thêm 0.030 kg, `uav0_full` thêm 0.091 kg trên nền 2.0 kg → **+1.5%** và **+4.6%**. Chưa hiệu chỉnh `MPC_THR_HOVER`; PX4 tự ước lượng lại ga hover khi bay. Bài bay hồi quy vẫn PASS 3/3 với cả hai biến thể nên hiện chưa cần can thiệp.

---

## 3. Yếu tố KHÔNG được mô hình + biên an toàn khuyến nghị

Đây là phần quan trọng nhất của tài liệu này: **những gì mô phỏng đang nói dối**.

| Yếu tố | Trạng thái | Hệ quả khi ra bay thật | Biên khuyến nghị |
|---|---|---|---|
| **Gió** | ❌ không bật | Sim không có nhiễu loạn nào. Bám điểm trong sim luôn đẹp hơn thật. | Đừng lấy sai số bám điểm trong sim làm chỉ tiêu nghiệm thu. Bật Wind system và thử lại trước P11. |
| **Ground effect** | ❌ không mô hình | Lực nâng thật tăng khi gần đất (dưới ~1 bán kính cánh). Hạ cánh thật sẽ "nổi" hơn sim. | Giữ biên cho pha hạ cánh cuối; đừng tune gain hạ cánh chỉ bằng sim. |
| **Downwash / blade flapping** | ❌ không mô hình | Bay gần vật cản hoặc gần drone khác sẽ khác sim. | Tăng khoảng cách an toàn tối thiểu khi bay thật. |
| **Sụt áp pin theo tải** | ❌ không mô hình | Lực đẩy khả dụng giảm dần trong chuyến bay thật. | Đặt ngưỡng battery failsafe bảo thủ; không dùng thời gian bay của sim để lập kế hoạch. |
| **Mật độ không khí theo độ cao / nhiệt độ** | ❌ `atmosphere adiabatic` nhưng không nối vào lực đẩy | Bay cao hoặc trời nóng → lực đẩy thật kém hơn. | Giữ biên lực đẩy; kiểm tại độ cao và nhiệt độ thật của bãi bay. |
| **Rangefinder KHÔNG vào EKF2** | 🔴 khác biệt kiến trúc | Trên drone thật rangefinder thường nuôi EKF2 → **hành vi ước lượng độ cao khác hẳn**. | Phải kiểm lại riêng trên HITL ở P11. Không giả định P4 tuned trong sim sẽ đúng khi rangefinder vào EKF2. |
| **"VIO" là ground truth** | 🔴 lý tưởng hoá | Không nhiễu, không trôi, không mất bám, không trễ. Thuật toán ăn nguồn này sẽ **trông giỏi hơn thực tế rất nhiều**. | Trước khi tin bất kỳ kết quả P4 nào, phải tiêm nhiễu + độ trễ + sự kiện mất bám. |
| **Optical flow** | ❌ chưa có nguồn dữ liệu | Plugin của PX4 chưa tích hợp ở v1.15.4. | `optical_flow_adapter_node` chưa test được. Cân nhắc tự sinh flow từ camera dưới. |
| **Nhiễu IMU/GPS/Mag** | ⚠️ dùng mặc định của PX4/x500 | Chưa hiệu chỉnh theo cảm biến thật. | Đo nhiễu của IMU/GPS thật rồi nạp lại trước P11. |
| **Độ trễ & băng thông cảm biến** | ⚠️ một phần | Đo được 13–18 Hz ảnh so với 30 Hz cấu hình. Drone thật đi qua USB/MIPI với đặc tính trễ khác. | Thiết kế P5 chịu được tần số ảnh thấp và biến thiên. |
| **Rung động khung** | ❌ không mô hình | Rung thật làm hỏng IMU và làm mờ ảnh. | Không có cách bù trong sim; phải kiểm trên phần cứng. |
| **Va chạm mềm / biến dạng càng đáp** | ❌ thân cứng tuyệt đối | — | Ngưỡng phát hiện tiếp đất trong sim lạc quan hơn thật. |

---

## 4. Thí nghiệm định lượng đã chạy

| # | Thí nghiệm | Kết quả | Ngày |
|---|---|---|---|
| E-RTF | Real-time factor theo biến thể, đo bằng chênh lệch sim-time / real-time trong 20 s | `uav0` 1.000 · `uav0_nav` 1.000 · `uav0_full` 1.000 | 2026-08-10 |
| E-GPU | Ma trận cấu hình render | env mặc định → **segfault**; ép NVIDIA → 1.000; software GL → 0.014 | 2026-08-10 |
| E-ORIENT | Hướng cảm biến, probe **static** ở z=3.0 trên nền phẳng | lidar 3.000 m · depth 3.000/3.000 m toàn khung · camera pitch+ thấy đất, pitch− thấy trời | 2026-08-10 |
| E-MOUNT | Vị trí link sau khi neo `relative_to="base_link"` | `base_link` z=0.24 → cảm biến z=0.19, đúng offset 0.05 m | 2026-08-10 |
| E-RANGE | Rangefinder suốt một chuyến bay (cất cánh 2.5 m) | **2197/2197 mẫu hữu hạn**, min 0.135 m, max 2.885 m, span 2.750 m | 2026-08-10 |
| E-REG | Bài bay hồi quy M5 | `uav0` **PASS 3/3** · `uav0_nav` **PASS 3/3** · thêm 3 chuyến đơn PASS | 2026-08-10 |

### Chưa chạy (nợ)

- **E1 — Hover thrust:** đối chiếu ga hover đo được với dự đoán từ khối lượng và kT (sai số mục tiêu < 5–10%). Chỉ có ý nghĩa thật sự khi đã có thân drone thật.
- **E3 — Quán tính:** đổi `ixx/iyy` ±30%, đo thời gian đáp ứng roll/pitch, kiểm đúng xu hướng.
- **E9 — Nhiễu cảm biến:** tăng nhiễu IMU/GPS, so vị trí ước lượng với ground truth, xem sai số tăng nhưng không phân kỳ.

Ba thí nghiệm này để lại cho lúc có thân drone thật — chạy trên x500 thì chỉ nghiệm thu lại x500, không nói gì về drone của dự án.

---

## 5. Tài liệu tham chiếu

| Nguồn | Dùng cho | Truy cập |
|---|---|---|
| `PX4-Autopilot/Tools/simulation/gz/models/x500*` (v1.15.4) | Thông số thân, motor | 2026-08-03, đối chiếu 2026-08-10 |
| `PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf` (v1.15.4) | Physics, gravity, toạ độ | 2026-08-10 |
| `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` | Phân bổ điều khiển, `SENS_EN_*` | 2026-08-10 |
| `PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp` (v1.15.4) | Danh sách cảm biến PX4 thật sự nhận; cơ chế spawn 1-lần-1-giây | 2026-08-10 |
| `PX4-Autopilot/src/modules/simulation/gz_bridge/gz_env.sh.in` | Cách `GZ_SIM_RESOURCE_PATH` được nối thêm | 2026-08-10 |
| `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator` | Luồng khởi động Gazebo, `PX4_GZ_STANDALONE` | 2026-08-10 |

Toàn bộ tra cứu đọc **trực tiếp từ mã nguồn đã cài trong máy**, không lấy từ web — nên phiên bản chắc chắn khớp với thứ đang chạy.
