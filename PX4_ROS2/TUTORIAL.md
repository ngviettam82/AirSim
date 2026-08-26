# Hướng dẫn sử dụng — dùng từng package, khi nào và để làm gì

> **Tài liệu này viết cho người đã đọc [`USER-GUIDE.md`](USER-GUIDE.md)** và bây giờ muốn *chạy*
> chứ không phải *hiểu*. Nếu bạn chưa đọc file kia, ít nhất hãy đọc mục 2 và mục 4 của nó —
> không có ranh giới `px4_msgs` và xương sống lệnh thì mọi lệnh dưới đây chỉ là gõ theo.

---

## Mục lục

1. [Chuẩn bị và bay lần đầu](#1-chuẩn-bị-và-bay-lần-đầu)
2. [Hai tầng khởi động — thứ tự là bắt buộc](#2-hai-tầng-khởi-động--thứ-tự-là-bắt-buộc)
3. [Bảng công tắc launch](#3-bảng-công-tắc-launch)
4. [Dùng từng package](#4-dùng-từng-package)
5. [Công thức cho việc hay gặp](#5-công-thức-cho-việc-hay-gặp)
6. [Chạy test và cổng](#6-chạy-test-và-cổng)

---

## 1. Chuẩn bị và bay lần đầu

### 1.1 Bố trí cây thư mục

| Vai | Đường dẫn |
|---|---|
| Cây nguồn chuẩn (nơi bạn **sửa**) | `C:\code\PX4_ROS2` |
| Cây build (nơi colcon **chạy**) | `~/PX4_ROS2` trong WSL |
| PX4-Autopilot | `~/PX4-Autopilot` (v1.15.4) |

`~/PX4_ROS2/scripts` là **symlink** trỏ về cây Windows, nên script luôn đồng bộ. `src/` là **bản
sao thật** và phải được đồng bộ khi bạn sửa mã.

### 1.2 Build

```bash
cd ~/PX4_ROS2
colcon build                     # lần đầu: build cả 12 package
source install/setup.bash
```

Một package thôi:

```bash
bash scripts/sync_build_package.sh uav_navigation           # đồng bộ + build
bash scripts/sync_build_package.sh uav_navigation --test    # kèm chạy test
```

> ⚠️ **Mọi terminal mới đều phải `source ~/PX4_ROS2/install/setup.bash`.** Chỉ `start_sim.sh` tự
> source. Quên nó sẽ ra `Package 'uav_bringup' not found`.

### 1.3 Bay lần đầu — ba lệnh

```bash
# Terminal 1 — THẾ GIỚI mô phỏng (Gazebo + PX4 SITL + agent uXRCE-DDS + cầu)
cd ~/PX4_ROS2 && UAV_MODEL=uav0_nav bash scripts/start_sim.sh

# Terminal 2 — PHẦN MỀM tự chủ
source ~/PX4_ROS2/install/setup.bash
ros2 launch uav_bringup sim.launch.py
```

Rồi bay bài hồi quy chuẩn (arm → cất cánh → bay tới → hạ cánh → disarm, ba lần):

```bash
bash scripts/run_m5_regression.sh
```

Muốn xem bằng mắt: `bash scripts/start_sim.sh gui`.
Dừng mọi thứ: `bash scripts/stop_sim.sh`.

---

## 2. Hai tầng khởi động — thứ tự là bắt buộc

Đây là chỗ người mới vấp nhiều nhất.

| Tầng | Lệnh | Nó là gì | Trên máy bay thật? |
|---|---|---|---|
| **Thế giới** | `scripts/start_sim.sh` | Gazebo, PX4 SITL, MicroXRCEAgent (udp4:8888), cầu Gazebo→ROS | ❌ **Không có bản tương ứng.** Vai trò này do bộ điều khiển bay và cảm biến thật đảm nhiệm |
| **Tự chủ** | `ros2 launch uav_bringup sim.launch.py` | 12 package của dự án | ✅ Cùng bộ node, chạy qua `real.launch.py` |

> 🔴 **Thế giới TRƯỚC, phần mềm SAU.** `sim.launch.py` đặt `use_sim_time:=true`, và `/clock` do
> cầu Gazebo phát. Launch trước là để mọi node đứng chờ một cái đồng hồ không bao giờ tới.
>
> Nếu chạy hoàn toàn không có sim: thêm `use_sim_time:=false`.

### Biến môi trường của `start_sim.sh`

| Biến | Mặc định | Ý nghĩa |
|---|---|---|
| `UAV_MODEL` | `uav0` | `uav0` · `uav0_nav` · `uav0_nav_indoor` · `uav0_track` · `uav0_full` |
| `UAV_WORLD` | suy từ model | `*_indoor` → `uav_arena_indoor`; còn lại → `uav_arena` |
| `UAV_BRIDGE_IMAGES` | — | truyền vào tham số `images:` của file launch cầu |
| `UAV_BRIDGE_CONFIG` | — | đè đường dẫn yaml của cầu |
| `UAV_DDS_PROFILE` | — | `none` để không dùng profile Fast DDS large-samples |

Log: `/tmp/agent.log` · `/tmp/gz.log` · `/tmp/px4.log` · `/tmp/bridge.log`.

### Thang bốn tầng model drone — chọn cái nhẹ nhất đủ dùng

| Model | Cảm biến | Dùng khi |
|---|---|---|
| `uav0` | không | Test bay thuần, M5, bất cứ gì không cần nhìn. **Nhẹ nhất** |
| `uav0_nav` | camera dưới + lidar dưới | Điều hướng, marker, độ cao |
| `uav0_track` | camera trước RGB + depth | Bám mục tiêu, trích vật cản |
| `uav0_full` | cả bốn | Bài cần đủ giác quan |

> 🔑 **Số cảm biến render cùng một tick quyết định tốc độ mô phỏng, chứ không phải cảm biến nào.**
> Đừng dùng `uav0_full` cho một bài chỉ cần `uav0`.

### `real.launch.py` — trơ theo thiết kế

```bash
ros2 launch uav_bringup real.launch.py                  # KHỞI ĐỘNG KHÔNG GÌ CẢ, và ghi log lý do
ros2 launch uav_bringup real.launch.py reviewed:=true   # mới thật sự chạy
```

Cờ `reviewed` không phải nghi thức: ba quyết định từ các giai đoạn trước được ghi **thành văn**
vào chính file đó. Hai file launch khác nhau **đúng ba tham số**, và sự khác biệt đó **được một
test khẳng định**:

| Chỉ sim | Chỉ real | Chung |
|---|---|---|
| `localization_params` · `require_obstacle_feed` | `reviewed` | 12 tham số còn lại |

---

## 3. Bảng công tắc launch

```bash
ros2 launch uav_bringup sim.launch.py <tên>:=<giá trị> ...
```

| Tham số | Mặc định | Bật cái gì | Khi nào bạn đổi nó |
|---|---|---|---|
| `uav_id` | `uav0` | Tiền tố `/uav/<id>/…` trên mọi node | Nhiều drone |
| `use_sim_time` | `true` | Theo `/clock` | `false` khi chạy không có sim |
| `perception` | **`false`** | 5 node perception + `world_model_node` | Cần marker/vật cản/bám mục tiêu. **Tốn render** |
| `navigation` | `true` | `route_planner_node` + `local_planner_node` (cố vấn) | `false` để bay không có cố vấn |
| `navigator` | `true` | `navigator_action_server_node` | `false` khi tự phát setpoint |
| `control_authority` | `true` | Trọng tài quyền | Hầu như không bao giờ |
| `safety` | `true` | `safety_supervisor_node` | Hầu như không bao giờ |
| `safety_enforcement` | `true` | INHIBIT/HOLD **có hiệu lực thật** | `false` để **quan sát** safety mà không cho nó hành động |
| `mission` | **`false`** | `mission_executor_node` | Chạy mission BT |
| `blackbox` | `true` | `rosbag_manager_node` | `false` khi không cần ghi |
| `diagnostics` | `true` | `diagnostics_node` (đèn go/no-go) | Hầu như không bao giờ |
| `event_log` | `true` | `event_logger_node` | Hầu như không bao giờ |
| `require_obstacle_feed` | `false` (sim) | Planner **từ chối lập kế hoạch** khi không có nguồn vật cản | `true` để bay đúng cấu hình phía real |
| `localization_params` | yaml ship sẵn | Trỏ localization sang file cấu hình khác | Thí nghiệm tiêm nhiễu |

> ⚠️ **`perception` và `mission` mặc định TẮT.** Muốn dùng phải truyền tường minh.

---

## 4. Dùng từng package

### 4.1 `uav_interfaces` — từ vựng

**Khi nào:** mỗi lần bạn sắp publish, subscribe, gọi hay phục vụ bất cứ thứ gì trong stack; và
mỗi lần bạn cần biết một trường nghĩa là gì hay đơn vị nào.

```bash
ros2 interface list | grep uav_interfaces
ros2 interface show uav_interfaces/msg/ControlCommand
ros2 interface show uav_interfaces/action/GotoPose
```

Đẩy một lệnh bằng tay để thử bench:

```bash
ros2 topic pub -r 10 /uav/uav0/control/cmd_test uav_interfaces/msg/ControlCommand \
  '{header: {frame_id: odom}, uav_id: uav0, control_mode: 1, position: {x: 0.0, y: 0.0, z: 2.0}}'
```

**Phải biết:** thêm trường thì **chỉ thêm vào cuối** (additive-only). Hai "foot-gun" **cố ý
không có** trong API: `Arm.srv` **không có** tuỳ chọn force-arm, và `Disarm.srv` **chỉ dùng dưới
đất**, không có kill trên không — cắt động cơ khi đang bay là rơi tự do; nút cắt khẩn phải là một
công tắc RC phần cứng độc lập.

---

### 4.2 `uav_px4_backend` — cầu nối PX4

**Khi nào:** câu hỏi dính tới bộ điều khiển bay. *"Arm thế nào?"* · *"Sao không vào được
offboard?"* · *"`/state/odometry_raw` từ đâu ra?"* · *"Đẩy VIO vào EKF2 kiểu gì?"*

Năm node, chạy qua bringup. Chạy lẻ một node:

```bash
ros2 run uav_px4_backend px4_state_adapter_node --ros-args -p uav_id:=uav0 -p use_sim_time:=true
```

**Tham số hay chỉnh** (`src/uav_bringup/config/backend_params.yaml`):

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `px4_command_gateway_node.stream_rate_hz` | `20.0` | Nhịp cặp setpoint + offboard. Sàn của PX4 là 2 Hz; biên này **cố ý rộng** |
| `px4_command_gateway_node.command_timeout_sec` | `0.5` | Lệnh cũ hơn mức này ⇒ **ngừng phát hoàn toàn** |
| `offboard_session_manager_node.priming_cycles` | `10` | Số setpoint phải phát **trước khi** xin đổi chế độ |
| `offboard_session_manager_node.auto_engage` | `true` | `false` để bắt buộc một tác nhân ngoài ra lệnh offboard |
| `px4_external_odometry_node.require_valid_localization` | `true` | Không đẩy gì vào EKF2 nếu localization không đáng tin |

> 🔴 **Bẫy QoS phải nhớ.** Mọi subscription `/fmu/out/*` dùng `KeepLast(10) + BestEffort +
> TransientLocal`. PX4 phát best-effort qua uXRCE-DDS, nên một subscription **Reliable sẽ không
> bao giờ khớp** — DDS đơn giản là không kết nối: không lỗi, không cảnh báo, không dòng log.
> Triệu chứng **giống hệt** "PX4 không chạy".

---

### 4.3 `uav_localization` — drone đang ở đâu

**Khi nào:** *"Drone nghĩ nó đang ở đâu, và tin được bao nhiêu?"* Một planner đang vật lộn với
pose nhảy hoặc trễ; bạn cần biết nguồn nào đang hoạt động.

Sáu node lên cùng nhau qua bringup. Xem nguồn nào đang thắng:

```bash
ros2 topic echo /uav/uav0/state/estimator_source          # latched, chỉ phát khi ĐỔI
ros2 topic echo /uav/uav0/state/localization_status
ros2 topic echo /uav/uav0/diagnostics/localization        # 1 Hz, bằng chứng sức khoẻ
```

> ⚠️ `estimator_source` là **TransientLocal và chỉ phát khi đổi**. Subscribe bằng QoS
> **Volatile** sẽ không thấy gì suốt cả chuyến bay và tưởng là hỏng.

**Ba điều phải biết khi dùng:**

1. Mux **chọn**, không **trộn** ⇒ đừng đòi nó lọc bớt sai số của một nguồn.
2. Nó chọn theo 1-sigma **tự khai** ⇒ nguồn trôi mà khai đẹp vẫn thắng; ca đó do
   `localization_health_node` bắt.
3. Sau khi mất hết nguồn, mốc neo còn sống thêm `source_timeout_sec`. Nháy ngắn thì được làm mượt;
   mất lâu thì thả mốc và pose kế tiếp **được phép nhảy** — cố ý, vì làm mượt sau một cú mất tín
   hiệu thật là che giấu nó.

---

### 4.4 `uav_perception` — drone nhìn thấy gì

**Khi nào:** *"Drone THẤY gì?"* Hạ cánh theo marker không tìm ra marker; local planner được cho
ăn vật cản không tồn tại hoặc thiếu vật cản có thật.

```bash
ros2 launch uav_bringup sim.launch.py perception:=true
```

Kiểm từng khối:

```bash
bash scripts/verify_marker_detector.sh        # bay lơ lửng trên marker ở hai cao độ
bash scripts/verify_obstacle_extractor.sh     # spawn hộp, đo hình học
bash scripts/verify_target_tracker.sh
bash scripts/verify_camera_health.sh          # kèm tiêm lỗi: giết cầu ảnh
```

**Phải biết khi dùng:**

- **Package này KHÔNG có driver camera, có chủ đích.** Trong sim, cầu của `uav_sim_gz` đã phát
  đúng tên topic mà driver thật sẽ dùng ⇒ không node nào ở đây phân biệt được sim với real.
- Mỗi node có tham số `camera`: `marker_detector_node` mặc định **`down`**,
  `obstacle_extractor_node` và `object_detector_node` mặc định **`front`**.
- `marker_detector_node` **từ chối** ước lượng pose cho tới khi `camera_info` có `fx, fy, cx, cy`
  đều `> 0`. Một `CameraInfo` chưa tới đọc ra toàn số 0, và `solvePnP` sẽ vui vẻ trả về một pose
  **trông y hệt mọi pose khác** mà không có nghĩa gì.
- `camera_health_node` **không được cho biết nhịp khung mong đợi** — nó suy từ khoảng cách nhỏ
  nhất giữa hai `header.stamp`. Đổi `<update_rate>` trong SDF thì nó tự theo, không phải sửa tham
  số ở đâu cả.
- **Ảnh phẳng chỉ bao giờ ra WARN, không bao giờ ra ERROR.** Camera chĩa lên trời quang thật sự
  cho ảnh phẳng, và không phép thử nào trên riêng bức ảnh phân biệt nổi nó với cảm biến chết.

---

### 4.5 `uav_world_model` — quan sát thành sự thật thế giới

**Khi nào:** khi một consumer cần câu trả lời ở **khung thế giới** thay vì khung camera.

Node này lên cùng cờ `perception:=true`.

```bash
ros2 topic echo /uav/uav0/world/obstacle_map_local
ros2 topic echo /uav/uav0/world/semantic_landmarks
ros2 topic echo /uav/uav0/world/target_state
```

**Phải biết:** đây là **nơi duy nhất** đổi khung optical → odom. Đừng thêm phép đổi thứ hai ở bất
kỳ đâu. Một odometry có hiệp phương sai vị trí **toàn 0 hoặc âm bị TỪ CHỐI** — 0 nghĩa là "chắc
chắn tuyệt đối", mà không có phép đo nào như vậy.

---

### 4.6 `uav_navigation` — biến mục tiêu thành chuyển động

**Khi nào:** *"Máy bay di chuyển kiểu gì?"* Bạn đang viết mission và cần biết gọi action nào, hay
goal của bạn bị từ chối vì sao.

Bảy action, gọi trực tiếp được:

```bash
ros2 action send_goal /uav/uav0/planning/takeoff \
  uav_interfaces/action/Takeoff '{target_altitude: 2.5}' --feedback

ros2 action send_goal /uav/uav0/planning/goto_pose \
  uav_interfaces/action/GotoPose \
  '{target: {header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0, z: 2.5}}}}' --feedback

ros2 action send_goal /uav/uav0/planning/land uav_interfaces/action/Land '{}'
```

Xem kế hoạch hiện hành và lời khuyên tránh vật cản:

```bash
ros2 topic echo /uav/uav0/planning/trajectory     # latched — người đọc vào muộn vẫn thấy
ros2 topic echo /uav/uav0/planning/route
ros2 topic echo /uav/uav0/planning/avoidance
```

**Phải biết khi dùng — sáu luật:**

1. **Một nhiệm vụ tại một thời điểm**, và chỉ nhận từ `IDLE` hoặc `HOLDING`. Không hàng đợi, không
   tự chiếm quyền. Muốn ngắt: **cancel rồi gửi goal mới**.
2. **`Recover` là action DUY NHẤT chiếm quyền được.** Nhiệm vụ đang chạy kết thúc với
   `ABORTED_SAFETY`, cố ý **không** phải `CANCELED` — vì không ai bảo caller đó dừng.
3. **Cancel không buông tay lái.** Setpoint **đóng băng tại chỗ** (không giật về vị trí đo được —
   đó sẽ là một bậc thang), máy bay tự khép nốt vài centimét, và **dòng 20 Hz vẫn chạy**.
4. **Cancel trong lúc LAND bị TỪ CHỐI.** Đã lệnh hạ cánh thì autopilot sở hữu máy bay.
5. **Offboard TRƯỚC, arm SAU.** `Takeoff` neo setpoint tại pose hiện tại → chạy dòng 20 Hz → **xoá
   trạng thái offboard cũ** → chờ `STATE_ACTIVE` → mới gọi arm → mới leo.
6. **Chỉ có MỘT publisher của `/planning/trajectory`**, và nó latched. Đó là lý do không có
   `recovery_planner_node` hay `trajectory_generator_node` riêng: hai người viết vào một topic
   latched sẽ cho người đọc vào muộn **hai "kế hoạch hiện hành" mâu thuẫn**.

---

### 4.7 `uav_control_authority` — ai đang lái

**Khi nào:** *"Ai đang thật sự lái, và vì sao setpoint của tôi không ra được đầu kia?"*

```bash
ros2 topic echo /uav/uav0/control/authority          # latched
ros2 topic echo /uav/uav0/control/command_selected
ros2 topic info /uav/uav0/control/command_selected --verbose   # phải thấy ĐÚNG 1 publisher
```

**Tham số hay chỉnh:**

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `source_timeout_sec` | `0.20` | Cửa sổ "còn sống" của một kênh |
| `release_dwell_sec` | `0.20` | Thời gian im lặng liên tục trước khi **nhả** quyền xuống |
| `odom_frame` | `odom` | `ControlCommand` khác khung bị **DROP** |

**Phải biết:** đi **lên** thì tức thì (bản tin ưu tiên cao hơn chiếm quyền ngay khi tới), đi
**xuống** thì cần `release_dwell_sec` im lặng liên tục. Và **rác không phải bằng chứng của sự
sống** — kênh phát NaN/sai khung/quá cũ ở 20 Hz không bao giờ thắng quyền, không bao giờ giữ được
quyền.

Bộ lọc nội dung chạy **trước** mọi thứ khác: `control_mode` không hỗ trợ → DROP + WARN; giá trị
không hữu hạn ở trường mà chế độ đang dùng thật sự đọc → DROP + ERROR; `header.frame_id` khác
`odom_frame` → DROP + ERROR; `header.stamp` cũ hơn `max_command_age_sec` → DROP.

---

### 4.8 `uav_safety` — giám sát an toàn cấp cao

**Khi nào:** *"Vì sao máy bay ngừng nhận lệnh?"* · *"Mã vi phạm này nghĩa là gì?"* · *"Xoá một
fault đã latch kiểu gì?"*

```bash
ros2 topic echo /uav/uav0/safety/state          # latched, 5 Hz + phát ngay khi đổi
ros2 topic echo /uav/uav0/safety/violations
ros2 service call /uav/uav0/safety/clear_fault uav_interfaces/srv/ClearFault '{}'
```

Quan sát mà **không** cho nó hành động:

```bash
ros2 launch uav_bringup sim.launch.py safety_enforcement:=false
# xác nhận: /control/cmd_safety phải có 0 publisher
ros2 topic info /uav/uav0/control/cmd_safety --verbose
```

**Taxonomy đúng ba hành động — không có hành động thứ tư:**

| Hành động | Nó làm gì | Mã nào |
|---|---|---|
| **REPORT** | Phát vi phạm + sự kiện. **Không bao giờ** chạm quyền điều khiển | 8 mã |
| **HOLD** | Giành quyền, phát một pose **đóng băng**, latch tới khi `ClearFault` | `OBSTACLE_TOO_CLOSE` |
| **INHIBIT** | Giành quyền rồi **không phát gì cả**, latch tới khi `ClearFault` | `BLIND_COMMAND` · `BATTERY_PX4_NO_ACTION` · `FRAME_MISMATCH` |

> 🔴 **`ClearFault` phải POLL.** Gọi một lần rồi kết luận là sai — nó cần `clear_stability_sec`
> liên tục sạch mới thật sự nhả.

**Phải biết:** package này **không bao giờ** ra lệnh land, RTL hay climb; không bao giờ gọi action
của navigator; không arm/disarm/set_mode. Mất định vị ⇒ nó **im lặng có chủ đích** (INHIBIT) để
failsafe lõi PX4 tiếp quản và **trả quyền cho phi công**.

---

### 4.9 `uav_mission` — nhiệm vụ nhiều bước

**Khi nào:** bạn muốn drone làm một công việc **nhiều bước có tên**, không phải một action lẻ.

```bash
ros2 launch uav_bringup sim.launch.py mission:=true     # MẶC ĐỊNH TẮT — phải truyền
# chờ dòng log: mission_executor_node ready for uav0

ros2 action send_goal /uav/uav0/mission/execute_mission \
  uav_interfaces/action/ExecuteMission '{mission_id: indoor_patrol}' --feedback

ros2 topic echo /uav/uav0/mission/status     # latched, 2 Hz + ngay khi đổi trạng thái
ros2 topic echo /uav/uav0/mission/events

ros2 service call /uav/uav0/mission/pause  uav_interfaces/srv/PauseMission  '{}'
ros2 service call /uav/uav0/mission/resume uav_interfaces/srv/ResumeMission '{}'
ros2 service call /uav/uav0/mission/abort  uav_interfaces/srv/AbortMission  '{}'
```

Ba mission ship sẵn: `indoor_patrol` · `follow_target` · `inspect_point`
(`src/uav_mission/config/missions/*.xml`).

**Phải biết:**

- **Mission không bao giờ điều khiển máy bay.** Bốn điều cấm: không `ControlCommand`, không
  `clear_fault`, không `arm/disarm/set_mode`, không `SetControlAuthority`.
- **Tối đa MỘT goal navigator** tồn tại, node-wide, vĩnh viễn. Không hàng đợi: broker bận thì yêu
  cầu bị **từ chối**.
- **Tối đa MỘT `ExecuteMission`.** Goal thứ hai bị từ chối ngay, không xếp hàng, không huỷ cái
  đang chạy.
- **`Takeoff` và `Finish` do node tự thêm**, không nằm trong file XML. Mỗi mission là
  `Takeoff → thân XML → Finish`, với `Finish` = `GotoPose(home) → Land`.
- **`AbortMission` KHÔNG hạ cánh** — nó dừng mission và **giữ vị trí**.
- **Guard an toàn nằm trong code, thân mission nằm trong XML.** `MissionGuard` là decorator C++
  đánh giá `mission_policy` mỗi tick 10 Hz; nó **không phải** một thẻ XML và **không thể bị sửa
  mất** bằng cách sửa file mission.

---

### 4.10 `uav_observability` — bằng chứng

**Khi nào:** *"Chuyến bay đó thật sự đã xảy ra chuyện gì?"* hoặc *"Máy bay này có đủ điều kiện
cất cánh ngay bây giờ không?"*

Cả ba node **mặc định BẬT**.

```bash
ros2 topic echo /uav/uav0/state/system_health      # go_no_go: GO | NO_GO | UNKNOWN
ros2 topic echo /uav/uav0/diagnostics/aggregated
```

> 🔴 **Đọc đèn go/no-go phải qua `scripts/preflight_light.sh`**, đừng `ros2 topic echo --once`
> trần — topic này latched và một lượt echo trần dễ đọc trúng mẫu cũ.

**Phải biết:**

- **`go_no_go` có BA giá trị.** Bất kỳ mục được tính nào ở `UNKNOWN` làm cả phán quyết thành
  `UNKNOWN` (không phải `NO_GO`). Ngược lại, một `ERROR` **hoặc một `WARN`** được tính là đủ thành
  `NO_GO` — không cần tới `ERROR`. Chỉ một bảng sạch hoàn toàn mới cho `GO`.
  Coi *"khác GO"* là `NO_GO` sẽ **phá huỷ** đúng thứ phân biệt *"đo được là xấu"* với *"không đo
  được"*.
- **Hộp đen không bao giờ chặn được cất cánh.** Hai nguồn chẩn đoán do blackbox sở hữu **không bao
  giờ** được tính vào `go_no_go`.
- **Pha bay là đại lượng ĐO ĐƯỢC**, không phải tham số đặt tay — suy từ `/state/vehicle`, cần
  `armed` **và** `connected`.
- Đọc hộp đen bằng `sqlite3` (và `reindex` nếu cần), không phải bằng công cụ khác.

---

### 4.11 `uav_sim_gz` — thế giới mô phỏng

**Khi nào:** đổi thứ drone bay trong đó hoặc thứ nó mang: thêm/dời cảm biến, thêm biến thể model,
chọn world, nối một topic cảm biến mới qua ROS, đăng ký airframe PX4 mới.

Chỉ chạy cầu (Gazebo đã lên sẵn):

```bash
ros2 launch uav_sim_gz gz_bridge.launch.py model:=uav0_nav
```

Đăng ký airframe + world với PX4:

```bash
bash src/uav_sim_gz/scripts/install_to_px4.sh ~/PX4-Autopilot
```

**Phải biết:**

- Cầu là **tầng driver cảm biến** của mô phỏng và là **nguồn duy nhất của `/clock`**. Tên topic
  phía ROS **cố ý** là tên mà driver thật sẽ phát.
- **Point cloud CỐ Ý không được bridge.** Nó mang cùng thông tin với ảnh depth trong khi tranh
  băng thông: có cloud thì depth chỉ giao được 18% số khung; bỏ cloud thì lên 42%.
- **"VIO" trong sim là GROUND TRUTH của Gazebo**, không phải thị giác. Không nhiễu, không trôi,
  không mất bám, không trễ. Thuật toán nào ăn nó cũng sẽ trông đẹp hơn thực tế.
- **Rangefinder không bao giờ tới được EKF2 của PX4.** `gz_bridge` của PX4 v1.15 không có distance
  sensor. Nó chỉ tồn tại ở phía ROS.
- 🔴 **Cảm biến render đọc `<visual>`, vật lý đọc `<collision>`.** Không mesh visual nào được đi
  kèm khối va chạm nhỏ hơn nó — nếu không, drone **thấy** tường và **bay xuyên**.
  `test/sdf_invariants.py` canh luật này trên mọi world và model.

---

## 5. Công thức cho việc hay gặp

### 5.1 "Máy bay không arm"

```bash
ros2 topic echo /uav/uav0/backend/offboard_status   # phải tới STATE_ACTIVE trước khi arm
ros2 topic echo /uav/uav0/state/vehicle             # armed / connected / flight_mode
ros2 topic echo /uav/uav0/safety/state              # có latch nào đang giữ không
grep -iE "arm|preflight|reject" /tmp/px4.log | tail -20   # LÝ DO của chính PX4
```

Thứ tự đúng luôn là **offboard trước, arm sau**. Nếu `offboard_status` không lên `STATE_ACTIVE`,
kiểm dòng setpoint có đang chạy không — `priming_cycles` (10) setpoint phải được phát **trước**
khi xin đổi chế độ.

### 5.2 "Setpoint của tôi không ra tới PX4"

Lần theo xương sống lệnh, từ dưới lên:

```bash
ros2 topic hz /fmu/in/trajectory_setpoint              # gateway có phát không
ros2 topic hz /uav/uav0/control/command_selected       # trọng tài có chọn ai không
ros2 topic echo /uav/uav0/control/authority            # ai đang giữ quyền
ros2 topic hz /uav/uav0/control/cmd_mission            # navigator có phát không
```

Bốn nguyên nhân theo thứ tự hay gặp: (1) một latch của safety đang giữ quyền; (2) lệnh bị bộ lọc
nội dung DROP (sai `frame_id`, có NaN, quá cũ); (3) một nguồn ưu tiên cao hơn đang giữ; (4) dòng
lệnh của bạn chậm hơn `command_timeout_sec` = 0,5 s.

### 5.3 "Thêm một cảm biến vào drone"

1. Tạo model cảm biến trong `src/uav_sim_gz/models/sensor_<tên>/model.sdf`
2. `<include>` nó vào biến thể drone + thêm `<joint>` cố định
3. Bridge:
   - **không phải ảnh** → thêm mục vào `config/bridge_<model>.yaml`
   - **là ảnh** → thêm cặp `(gz topic, ROS topic)` vào `MODEL_IMAGE_TOPICS` trong
     `launch/gz_bridge.launch.py`
4. Đặt tên topic ROS **đúng tên mà driver thật sẽ dùng**
5. `bash src/uav_sim_gz/scripts/install_to_px4.sh` nếu có airframe mới
6. Kiểm bất biến: `python3 -m pytest src/uav_sim_gz/test/test_sdf_invariants.py`

### 5.4 "Thêm một mission"

1. Viết `src/uav_mission/config/missions/<tên>.xml` — chỉ dùng thẻ trong danh sách trắng
2. Đăng ký trong `mission_registry`
3. Đừng thêm `Takeoff` hay `Land` vào XML — node tự bọc
4. Đừng thêm guard vào XML — guard nằm trong code và **không sửa được từ file mission**
5. `colcon test --packages-select uav_mission`

### 5.5 "Đổi một ngưỡng an toàn"

Ngưỡng nằm trong `src/uav_safety/config/safety_params.yaml`, và **mỗi mặc định đều kèm lý do**
ngay trong `failsafe_policy.hpp`. Đọc lý do trước khi đổi số. Sau khi đổi:

```bash
colcon test --packages-select uav_safety
bash scripts/verify_safety.sh               # chuỗi cắt end-to-end trong sim
```

### 5.6 "Bay đúng cấu hình phía real, trong sim"

```bash
ros2 launch uav_bringup sim.launch.py \
  require_obstacle_feed:=true perception:=true safety_enforcement:=true
```

Đây là cấu hình gần `real.launch.py` nhất mà vẫn chạy trong mô phỏng.

---

## 6. Chạy test và cổng

### 6.1 Test đơn vị

```bash
colcon test --packages-select uav_navigation
colcon test-result --verbose
```

> ⚠️ Một số package cần package khác **được build trong cùng lượt** vì test của chúng link vào
> node thật: `uav_mission` cần `uav_control_authority`; `uav_safety` cũng vậy.

Vài test dùng `ROS_DOMAIN_ID` riêng (đã khai trong `CMakeLists.txt`) để lưu lượng giả của chúng
không đụng domain đang chạy: `uav_navigation` → 92 · `uav_perception` → 93 · `uav_mission` → 98 ·
`uav_observability` → 99.

### 6.2 Kiểm tra tĩnh — không cần sim

```bash
bash scripts/check_px4_msgs_boundary.sh      # ranh giới px4_msgs trên toàn cây
python3 scripts/check_sim_real_parity.py     # sim và real có cùng bộ node không
bash scripts/check_doc_links.sh              # link tài liệu
bash scripts/audit_comments.sh               # kiểm kê ghi chú
```

### 6.3 Bài bay kiểm chứng

```bash
bash scripts/run_m5_regression.sh            # hồi quy nền: arm→takeoff→goto→land ×3
bash scripts/verify_<tên>.sh                 # 26 script, mỗi cái chứng minh một khối
```

### 6.4 🔑 `selftest_*` — thói quen đáng học nhất của dự án này

```bash
bash scripts/selftest_sdf_invariants.sh      # 14 ca
bash scripts/selftest_px4_msgs_boundary.sh   # 11 ca
bash scripts/selftest_workspace_verdict.sh   # 11 ca, hai chiều
bash scripts/selftest_sim_real_parity.sh      # 10 ca
```

Chúng **không kiểm sản phẩm**. Chúng đưa một đột biến **cùng hình dạng** với thứ cần canh vào,
rồi đòi bộ kiểm tra phải **đỏ**.

> Một cổng chưa từng chứng minh mình biết nói **KHÔNG** thì màu xanh của nó không chứng minh gì.

Khi bạn viết một bộ kiểm tra mới, hãy viết luôn `selftest_` của nó. Đó là quy ước quan trọng nhất
trong repo này.

---

## Đi đâu tiếp

| Cần gì | Đọc gì |
|---|---|
| Hiểu kiến trúc và cách đọc mã | [`USER-GUIDE.md`](USER-GUIDE.md) |
| Lý do thiết kế từng msg/srv/action | [`docs/interface-contract-v0.1.md`](docs/interface-contract-v0.1.md) |
| Chính sách QoS — 4 nhóm, topic nào latched | [`docs/qos-policy-v1.md`](docs/qos-policy-v1.md) |
| Bẫy công cụ / môi trường đã trả giá | [`docs/ops-playbook.md`](docs/ops-playbook.md) |
| Trạng thái & giả định từng package | [`docs/package-status.md`](docs/package-status.md) |
| Sim chứng minh được gì, và không được gì | [`docs/sim-boundary-statement.md`](docs/sim-boundary-statement.md) |
| Trước khi nghĩ tới bay thật | [`docs/preflight-checklist.md`](docs/preflight-checklist.md) |
