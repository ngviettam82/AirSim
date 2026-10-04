# CLAUDE.md — UAV Simulation & Autonomy Framework (PX4 + ROS2 + Gazebo)

> File này giữ **bức tranh tổng quan & ổn định** của dự án: mục tiêu, kiến trúc, bản đồ package, quy ước. **Cố ý KHÔNG chứa chi tiết hiện thực hay tiến độ** — những thứ đó thay đổi liên tục và sẽ làm file này phồng lên rồi lạc hậu. Chi tiết nằm ở các file riêng bên dưới.
>
> **⏩ ĐẦU mỗi context:** đọc file này + `.claude/memory.md`. **⏹ CUỐI mỗi context:** chạy checklist đóng phiên ở `.claude/rules/session-protocol.md` (R22) — ngưỡng context, số liệu suy lại từ công cụ, 1 dòng changelog.

## 🗺️ Bản đồ tài liệu — tìm gì ở đâu


> 📌 **Bản giao này không kèm thư mục `.claude/`.** Những chỗ bên dưới nhắc tới
> `.claude/...` được giữ lại dưới dạng tên gọi, không phải liên kết.

| Cần biết | Đọc file |
|---|---|
| 🆕 **NGƯỜI MỚI NHẬN DỰ ÁN BẮT ĐẦU TỪ ĐÂY** — kiến trúc, xương sống lệnh, chuỗi cảm nhận, thứ tự đọc từng package | [`USER-GUIDE.md`](USER-GUIDE.md) |
| 🆕 **Cách DÙNG từng package** — khi nào dùng, chạy thế nào, tham số hay chỉnh, công thức việc hay gặp | [`TUTORIAL.md`](TUTORIAL.md) |
| **Package nào xong/chưa, bẫy & giả định từng package** | [`docs/package-status.md`](docs/package-status.md) |
| Cách dùng navigator action + 3 điều phải biết trước khi sửa | [`src/uav_navigation/README.md`](src/uav_navigation/README.md) |
| Lý do thiết kế của msg/srv/action | [`docs/interface-contract-v0.1.md`](docs/interface-contract-v0.1.md) |
| **Chính sách QoS — 4 nhóm, danh sách topic latched, bẫy "sai một nét là im lặng"** | [`docs/qos-policy-v1.md`](docs/qos-policy-v1.md) |
| Cách khởi động sim & chạy bài bay hồi quy | [`src/uav_bringup/README.md`](src/uav_bringup/README.md) |
| **🔧 SỔ TAY VẬN HÀNH — mọi bẫy công cụ/môi trường đã trả giá (triệu chứng → nguyên nhân → cách đúng)** | [`docs/ops-playbook.md`](docs/ops-playbook.md) ← **đọc khi gặp lỗi lạ về công cụ** |
| Thế giới mô phỏng: 5 world, thang model, cầu Gazebo→ROS, **9 bẫy đã trả giá** | [`src/uav_sim_gz/README.md`](src/uav_sim_gz/README.md) |
| Nguồn từng con số vật lý + yếu tố KHÔNG mô hình + biên an toàn | [`src/uav_sim_gz/docs/model-sources.md`](src/uav_sim_gz/docs/model-sources.md) |
| **Vì sao giữ Gazebo, vì sao Google 3D Tiles bị cấm, và cách chữa "world không đáng tin"** | [`docs/sim-fidelity-decision.md`](docs/sim-fidelity-decision.md) |
| Chỗ điền thông số drone thật khi team phần cứng giao CAD | [`src/uav_sim_gz/docs/physics-parameters.md`](src/uav_sim_gz/docs/physics-parameters.md) |
| Hợp đồng của mọi nguồn định vị (không pose nào thiếu độ tin cậy) | [`src/uav_localization/README.md`](src/uav_localization/README.md) |
| Perception: **vì sao package này KHÔNG có driver camera**, sức khoẻ luồng ảnh, marker ArUco | [`src/uav_perception/README.md`](src/uav_perception/README.md) |
| World model: đổi observation sang frame odom, hợp đồng `/world/*` | [`src/uav_world_model/README.md`](src/uav_world_model/README.md) |
| Trọng tài quyền điều khiển: single-writer `command_selected`, latch, QoS | [`src/uav_control_authority/README.md`](src/uav_control_authority/README.md) |
| **Safety: taxonomy 3 hành động, enforcement, ClearFault phải POLL** | [`src/uav_safety/README.md`](src/uav_safety/README.md) |
| Mission BT: 4 điều cấm, NavGoalBroker, guard-trong-code, 5 lệch có chủ đích | [`src/uav_mission/README.md`](src/uav_mission/README.md) |
| **Observability: 7 điều PHẢI biết trước khi sửa** — hộp đen `sqlite3` (+`reindex`), 🔴 đọc đèn go/no-go phải qua `scripts/preflight_light.sh`, KHÔNG `ros2 topic echo --once` trần | [`src/uav_observability/README.md`](src/uav_observability/README.md) |
| **🧭 TUYÊN BỐ RANH GIỚI sim↔real** — sim đã chứng minh gì (20) và **KHÔNG bao giờ chứng minh được gì (65, 6 nhóm)** · bảng *đọc nhầm ↔ sự thật* · nhóm **B-e = nợ còn sống trong code SẼ BAY** | [`docs/sim-boundary-statement.md`](docs/sim-boundary-statement.md) ← **đọc trước khi tin bất kỳ câu "đã kiểm chứng" nào** |
| **📋 PHÂN LOẠI DÒNG CHƯA PHỦ (cổng `G-SIM S3`)** — 5 loại + điều kiện bằng chứng từng loại · 🔴 **cổng không hỏi phần trăm, hỏi "còn dòng nào chưa ai nhìn không"** · cổng tự bắt **luật chết** | [`docs/coverage-classification.md`](docs/coverage-classification.md) |
| **🛫 CHECKLIST TIỀN BAY** — 8 cổng chặn cứng · 9 mục kiểm chạy lại trước bay · 3 quyết định `real.launch.py` thừa kế · trình tự hiện trường · ô chữ ký | [`docs/preflight-checklist.md`](docs/preflight-checklist.md) ← **đọc trước khi nghĩ tới bay thật** |
| **Buổi giảng R18 cho P5+P6** — kiến trúc, 5 bài toán then chốt, 2 bài học chẩn đoán, câu hỏi tự kiểm | [`docs/lecture-p5-p6.md`](docs/lecture-p5-p6.md) |
| **Buổi giảng R18 TỔNG P7+P8+P9** — 3 tầng quyền/an toàn/mission, chuỗi cắt end-to-end, 4 ca chẩn đoán, R24–R35 | [`docs/lecture-p7-p8-p9.md`](docs/lecture-p7-p8-p9.md) |
| **Cách CHẠY nền AirSim** (Unreal → PX4 → ROS2) — 3 cuộc hội thoại, flow 4 bước, sự cố | [`docs/run-airsim-ga0.md`](docs/run-airsim-ga0.md) |
| **🔴 BÀI HỌC: dựng world 3D Bách Khoa — đã ĐÓNG & CẤT, đọc trước khi định dựng world từ ảnh** | [`docs/lesson-bk-world-attempt.md`](docs/lesson-bk-world-attempt.md) |
| **Tám nguồn trong `dóc/pdf/` là bài nào** — tên file → trích dẫn thật, giấy phép, và nguồn gốc thiết kế A*/xoắn ốc | [`docs/references.md`](docs/references.md) |

> # ⚠️ AN TOÀN SINH MẠNG — NGUYÊN TẮC TỐI CAO (đọc trước mọi thứ)
> **Đây là dự án THỰC TẾ. UAV sẽ bay thật — KHÔNG phải mô phỏng để "bay chơi".** Mô phỏng chỉ là bước đệm; đích đến là **real flight**. Vì vậy **nguyên tắc sim-to-real là quan trọng nhất**: mọi thứ làm trong sim phải hướng tới chạy đúng & an toàn trên phần cứng thật.
>
> Chỉ cần tôi hoặc bạn **bất cẩn, mất tập trung, hay giảm chất lượng công việc**, thì khi ra real flight **người thật có thể gặp nguy hiểm**. Do đó: làm **thật kỹ lưỡng, cẩn trọng, có lương tâm nghề nghiệp**; thà chậm mà chắc; **không đoán bừa** ở phần sát an toàn (offboard timing, frame, failsafe, control authority, QoS); luôn verify thật (không "chắc là ổn"). Đây là chuẩn mực xuyên suốt toàn dự án.

---

## 1. Bối cảnh & Mục tiêu dự án

**Đề tài:** Ứng dụng mô phỏng phương tiện bay không người lái (UAV/drone) trong môi trường **Gazebo**, sử dụng **ROS2** để xây dựng hệ thống **tự động hóa** cho UAV.

**Ý tưởng cốt lõi:**
- Xây dựng một **framework ROS2 module hóa** để điều khiển UAV tự động, chạy được cả trong **mô phỏng (Gazebo + PX4 SITL)** lẫn trên **drone thật (PX4 FC)** mà không phải viết lại logic.
- PX4 là flight stack (autopilot). ROS2 lo lớp autonomy cấp cao: perception, localization, planning, mission, safety.
- Cầu nối PX4 ↔ ROS2 là **uXRCE-DDS** (Micro XRCE-DDS Agent ↔ uXRCE-DDS Client trong PX4), trao đổi qua `px4_msgs`.

**Nguyên tắc thiết kế quan trọng (rút ra từ tài liệu):**
- **Cô lập `px4_msgs`:** chỉ package backend duy nhất (`uav_px4_backend`) được phép dùng `px4_msgs` trực tiếp. Mọi node khác chỉ dùng **message nội bộ** (`uav_interfaces`) để tránh phụ thuộc lung tung và giảm lỗi khi PX4 đổi API.
- **Tách bạch theo trách nhiệm:** mỗi package lo một mối quan tâm (perception, localization, navigation, mission, safety, control authority...).
- **Sim/real parity:** cùng codebase, chỉ khác launch file (`sim.launch.py` vs `real.launch.py`).
- **Namespace theo drone:** mọi topic đặt dưới `/uav/<id>/...` để sẵn sàng multi-drone.
- **Service = hành động ngắn (blocking); Action = hành động dài** (có feedback/cancel).
- Hướng tới hỗ trợ **RL** về sau (world model tạo observation là điểm nhấn được ghi chú trong tài liệu).

---

## 2. Trạng thái

> 📌 **Không ghi tiến độ ở đây** — nó lạc hậu ngay. Nguồn duy nhất:
> - **Cột mốc, quyết định, nợ kỹ thuật, điểm tiếp tục** → `.claude/memory.md`
> - **Package nào xong/chưa + bẫy từng package** → [`docs/package-status.md`](docs/package-status.md)

---

## 3. Kiến trúc tổng quan (conceptual pipeline)

Từ sơ đồ `dóc/px4_ros2_arch.drawio.png` — luồng dữ liệu ở mức khái niệm:

```
   Perception node ──/data/perception_data──┐
                                            ▼
   Mission layer ⇄(req-res)⇄ Planning executor ⇄/cmd/action,/cmd/pose⇄ Vehicle interface layer
                                            ▲                                    │ px4_msgs
   Estimation node ─/data/estimation_data───┘                                    ▼
                                                              Micro XRCE-DDS Agent
                                                                     │ DDS transport
                                                                     ▼
                                                              uXRCE-DDS Client
                                                                     │
                                                                     ▼
                                                        PX4 SITL / FC ⇄ Gazebo / Real drone
```

**Ánh xạ sơ đồ khái niệm → package thực tế (mục 4):**
| Khối trong sơ đồ | Package chi tiết |
|---|---|
| Mission layer | `uav_mission` |
| Planning executor | `uav_navigation` |
| Perception node | `uav_perception` (+ `uav_world_model`) |
| Estimation node | `uav_localization` |
| Vehicle interface layer | `uav_px4_backend` |
| (điều phối quyền điều khiển) | `uav_control_authority` |
| (giám sát an toàn) | `uav_safety` |

Sơ đồ là bản rút gọn; tài liệu Word là thiết kế chi tiết và **là nguồn chuẩn** cho cấu trúc package/node dưới đây.

---

## 4. Thiết kế chi tiết các package & node

Namespace quy ước: `<id>` = định danh drone (vd `uav0`). Mọi topic dạng `/uav/<id>/...`.

### 4.1 `uav_interfaces` — Interface nội bộ (nền tảng, mọi package phụ thuộc)
Chứa toàn bộ msg/srv/action nội bộ (KHÔNG dùng `px4_msgs`).

- **Messages:** `VehicleState`, `VehicleHealth`, `LocalizationStatus`, `MissionStatus`, `MissionEvent`, `SafetyState`, `ControlAuthority`, `MarkerObservation` (QR/ArUco), `TargetTrack` (bám mục tiêu), `ObstacleArray`, `Path3D`, `TrajectoryPoint`, `Trajectory3D`, `ControlCommand`.
- **Services (hành động ngắn):** `Arm`, `Disarm`, `SetFlightMode`, `SetControlAuthority`, `LoadMission`, `PauseMission`, `ResumeMission`, `AbortMission`, `ClearFault`.
- **Actions (hành động dài):** `ExecuteMission`, `Takeoff`, `Land`, `HoldPosition`, `GotoPose`, `FollowPath`, `TrackTarget`, `InspectMarker`, `Recover`.

### 4.2 `uav_px4_backend` — Cầu nối PX4 (package DUY NHẤT dùng `px4_msgs`)

Người phiên dịch hai chiều giữa hệ nội bộ và bộ điều khiển bay. 5 node:

| Node | Trách nhiệm |
|---|---|
| `px4_state_adapter_node` | Đọc trạng thái PX4 → msg nội bộ |
| `px4_command_gateway_node` | Lệnh nội bộ → setpoint PX4; service arm/disarm/set_mode |
| `offboard_session_manager_node` | Giữ nhịp offboard ≥2Hz, priming, bật offboard an toàn |
| `px4_frame_bridge_node` | Phát TF `odom → base_link` |
| `px4_external_odometry_node` | Đẩy định vị ngoài (VIO/mocap) vào bộ ước lượng PX4 |

> 📌 **Chi tiết hiện thực, quy tắc vận hành (NaN, ngừng phát khi lệnh cũ, không giành quyền khỏi autopilot), các giả định & bẫy đã biết** → [`docs/package-status.md`](docs/package-status.md).

### 4.3 `uav_localization` — Vị trí hiện tại của drone (nguồn định vị cho planner & mission)
- **`gps_adapter_node`** — Bay ngoài trời/GPS tốt. Out: `/uav/<id>/localization/gps_odometry`, `/gps_status` (kèm fix quality).
- **`vio_adapter_node`** — Trong nhà / GPS kém, dùng VIO (camera+IMU+SLAM: vd RealSense T265, VINS-Fusion, ORB-SLAM3). Out: `/localization/vio_odometry`, `/vio_status`.
- **`optical_flow_adapter_node`** — Camera nhìn xuống + rangefinder → local motion estimate. Out: `/localization/flow_odometry`, `/flow_status`.
- **`rangefinder_adapter_node`** — Đo khoảng cách xuống đất/vật cản gần; hỗ trợ optical flow, precision landing, safety bay thấp. Out: `/localization/range`, `/range_status`.
- **`localization_mux_node`** — Chọn nguồn định vị hợp lệ nhất → định vị chính thức toàn hệ thống. In: **gps + vio** odometry + status (optical flow phát *vận tốc*, không phải nguồn vị trí — xem README của package). Out: `/uav/<id>/state/odometry_fused`, `/state/localization_status`, `/state/estimator_source`. ⚠️ **Nó CHỌN một nguồn, không trộn** — đầu ra là nguồn thắng cộng bù liên tục khi chuyển; hệ quả cho người dùng đầu ra ở [`docs/package-status.md`](docs/package-status.md) §5.
- **`localization_health_node`** (optional) — Đánh giá chất lượng định vị (mất dữ liệu, tần số, nhảy vị trí, covariance tăng). Out: `/state/localization_health`, `/diagnostics/localization`.

### 4.4 `uav_perception` — Quan sát & mô tả môi trường
- 🔴 **KHÔNG có `camera_front_node` / `camera_down_node` trong sim (lệch thiết kế gốc, đã duyệt).** Cầu của `uav_sim_gz` đã phát đúng trên tên topic mà driver thật sẽ dùng (R7), nên thêm node chuyển tiếp là **trùng vai**. Driver camera thật thuộc **P11**. Lý do đầy đủ → [`src/uav_perception/README.md`](src/uav_perception/README.md).
- **`camera_health_node`** — Giám sát từng luồng ảnh, suy nhịp nguồn từ khoảng cách nhỏ nhất giữa hai dấu thời gian nên **phát hiện được mất khung mà không cần biết trước nhịp**. Out: `/diagnostics/perception`, `/state/camera_health`.
- **`marker_detector_node`** — Phát hiện QR/ArUco, ước lượng pose so với camera. In: `/perception/down/image_raw`. Out: `/perception/markers` (vd: marker ID 17, cách drone 1.2m, xoay 30°).
- **`object_detector_node`** — Nhận dạng vật thể tổng quát (người, xe, hộp hàng). Out: `/perception/detections`.
- **`target_tracker_node`** — Ổn định target, duy trì ID, ước lượng vận tốc mục tiêu. Out: `/perception/target_track`.
- **`obstacle_extractor_node`** — depth/lidar/point cloud → thông tin vật cản (có/không, ở đâu, khoảng cách, kích thước). Out: `/perception/obstacles_local`.

### 4.5 `uav_world_model` — Tổng hợp world model hoàn chỉnh
- **`world_model_node`** — Đổi observation từ frame camera → frame map/odom (**quan trọng cho RL**), lưu target/marker/obstacle map.
  - In: `/state/odometry_fused`, `/perception/markers`, `/perception/target_track`, `/perception/obstacles_local`.
  - Out: `/world/target_state`, `/world/obstacle_map_local`, `/world/semantic_landmarks`, `/world/mission_reference`.

### 4.6 `uav_navigation` — "Muốn tới mục tiêu thì bay thế nào" (trajectory)
- **`navigator_action_server_node`** — Cổng action chính cho mission gọi: Takeoff, GotoPose, FollowPath, TrackTarget, HoldPosition, Land, Recover.
- **`route_planner_node`** — Lập tuyến (waypoint 1→2→3, đi vòng khu cấm) → sinh path cho local planner.
- **`local_planner_node`** — Dùng vị trí + world model + obstacle local + target state để tránh vật cản, sửa quỹ đạo khi target di chuyển, giữ khoảng cách an toàn.
- **`trajectory_generator_node`** — Path/goal → trajectory mượt gửi backend. Out: `/uav/<id>/planning/trajectory`.
- **`recovery_planner_node`** — Sinh trajectory recovery khi lỗi (giữ vị trí, lên cao an toàn, về home, failsafe).

### 4.7 `uav_control_authority` — Ai đang có quyền điều khiển drone (chống xung đột lệnh)
- **`control_authority_manager_node`** — Nhận lệnh từ nhiều nguồn, chỉ cho **1 nguồn** điều khiển tại 1 thời điểm.
  - In: `/uav/<id>/control/cmd_safety`, `/control/cmd_operator`, `/control/cmd_mission`, `/control/cmd_test`.
  - Out: `/uav/<id>/control/command_selected`, `/control/authority`.
  - **Priority:** `safety/emergency` > `operator override` > `mission planner`.

### 4.8 `uav_safety` — An toàn cấp cao (KHÔNG thay thế failsafe lõi của PX4)
- **MỘT node (`safety_supervisor_node`) + thư viện ROS-free `failsafe_policy`** — lệch thiết kế gốc 2-node **có chủ đích** (P8, 2026-08-21): 0 hop trễ trên đường cứu mạng, diệt chế độ hỏng "policy chết mà supervisor sống"; luật ghim bằng bảng test.
- Giám sát liên tục: mất localization, lệnh hạ nguồn cũ, obstacle quá gần, battery, offboard unhealthy, sensor/frame. Out: `/safety/state`, `/safety/violations`, `/uav/<id>/control/cmd_safety`, srv `/safety/clear_fault`.
- **Taxonomy hành động đúng 3** (policy chủ dự án ký 2026-08-21): `REPORT` · `HOLD` (đóng băng tại pose tốt + latch) · `INHIBIT` (latch + im lặng có chủ đích → PX4 failsafe lõi + trả quyền pilot). 🔴 **Mất localization: KHÔNG auto-land, KHÔNG bay mù** — safety không bao giờ phát setpoint trên pose không tin được; land/RTL/climb là việc mission (P9) điều phối qua navigator, không phải của safety.
- Hợp đồng chi tiết → `docs/interface-contract-v0.1.md` §2.18; trạng thái/bẫy → `docs/package-status.md` §10.

### 4.9 `uav_mission` — Logic cấp cao nhất
- 🔴 **MỘT node (`mission_executor_node`) + 3 lib ROS-free (`mission_policy`, `mission_registry`, `nav_goal_broker`)** — lệch thiết kế gốc 3-node **có chủ đích** (P9, chủ dự án duyệt D-1 2026-08-22, tiền lệ P8): executor và bt_runner dùng chung toàn bộ state nên tách là đặt một hop DDS vào giữa máy trạng thái; registry không sở hữu state sống lâu nên hạ xuống lib.
- Node: action server `ExecuteMission` + BehaviorTree.CPP v3.8 (guard an toàn trong **code**, thân mission trong **XML** `config/missions/*.xml` — file text không gỡ được nhánh an toàn) + `NavGoalBroker` (tối đa MỘT goal navigator, mãi mãi) + 4 service Load/Pause/Resume/Abort. Out: `/uav/<id>/mission/status`, `/mission/events`. 3 mission: `indoor_patrol`, `follow_target`, `inspect_point` — cả 3 đã bay end-to-end (M9).
- Mission **không tự bay**: thuần action/service client của navigator; 4 điều CẤM (không ControlCommand · không clear_fault/latch · không arm/disarm/set_mode · không SetControlAuthority). Hợp đồng → `docs/interface-contract-v0.1.md` §2.19; trạng thái/bẫy → `docs/package-status.md` §11.

### 4.10 `uav_bringup` — Khởi động hệ thống đồng bộ
`sim.launch.py` (mô phỏng) và `real.launch.py` (drone thật) — **cùng bộ node, khác cấu hình** (R7). Kèm bài bay hồi quy.

> 📌 Cách dùng, phân biệt sim vs real, ngưỡng bài test → [`src/uav_bringup/README.md`](src/uav_bringup/README.md).

### 4.11 `uav_sim_gz` — Thế giới mô phỏng
World, model drone, mô-đun cảm biến, **và cầu nối Gazebo→ROS**. Cầu này đóng vai driver cảm biến của thế giới mô phỏng và là nguồn duy nhất của `/clock` — nên nó thuộc lớp mô phỏng, **không** có bản sao trên drone thật.

> ⚠️ Đừng nhầm với `uav_world_model` (§4.5): đây là thế giới drone **bay trong đó**; kia là thế giới drone **tin là có** và chạy cả trên phần cứng thật.

### 4.12 `uav_observability` — Tầng BẰNG CHỨNG (3 node + 3 lib ROS-free)
Không phải tầng an toàn — là tầng **bằng chứng**: trong sim, chuyến bay hỏng chạy lại được; trên drone thật thì không. Thiếu nó, mọi năng lực chẩn đoán của dự án vô dụng ngoài đời vì không còn dữ liệu để nhìn.

| Node | Vai | Lib ROS-free |
|---|---|---|
| `rosbag_manager_node` | Hộp đen: ghi ~46 topic bằng subscription generic (**không include header msg nào**), tự dọn bag cũ theo ngân sách đĩa | `bag_retention` |
| `diagnostics_node` | Sức khoẻ tổng + `/state/system_health` với `go_no_go` ∈ {GO, NO_GO, **UNKNOWN**} | `staleness_board` |
| `event_logger_node` | Dòng thời gian sự kiện hợp nhất → **file JSONL**, không publish lên bus | `event_ledger` |

🔴 **Năm nguyên tắc chi phối** (chủ dự án ký 2026-08-23): **O1** hộp đen KHÔNG được là tải trên đường bay — observability hỏng ⇒ mất bằng chứng, **không được mất chuyến bay** (3 process riêng, BestEffort mặc định, mọi lỗi ghi = WARN + degrade, không throw) · **O2** không tạo nguồn sự thật thứ hai (timeline là file, `/mission/events` vẫn là kênh sự kiện duy nhất trên dây) · **O3** "không đo được" ≠ OK · **O4** mọi sample-and-hold mang tuổi · **O5** sim/real parity.

> 📌 Hợp đồng đầy đủ → `docs/interface-contract-v0.1.md` §2.20; trạng thái/bẫy → `docs/package-status.md`; lý lẽ thiết kế + kết quả cổng → `.claude/plan/P10-observability.md`.

---

## 5. Quy ước & Kiến thức nền cần nhớ

- **Topic naming:** `/uav/<id>/<domain>/<name>` — domain: `state`, `localization`, `perception`, `world`, `planning`, `control`, `safety`, `mission`, `backend`, `diagnostics`. PX4 giữ nguyên `/fmu/in/*`, `/fmu/out/*`.
- **Ranh giới `px4_msgs`:** chỉ `uav_px4_backend`. Vi phạm ranh giới này là lỗi thiết kế.
- **Offboard control:** PX4 yêu cầu stream `OffboardControlMode` **≥ 2Hz** trước & trong khi offboard; `offboard_session_manager_node` chịu trách nhiệm nhịp này.
- **Frame:** ROS (ENU / FLU) vs PX4 (NED / FRD) — luôn convert qua `px4_frame_bridge_node`, không convert rải rác.
- **Service vs Action:** ngắn/blocking → service; dài/có feedback+cancel → action.
- **Ghi chú trong code:** tối đa **1 dòng, ≤10 từ, tiếng Anh**, chỉ khi lý do KHÔNG hiển nhiên (R16). Lý do dài → tài liệu riêng, không nhét vào code. **5 ngoại lệ hợp lệ** (đơn vị/frame trong `.msg`, cảnh báo R0 trong yaml, header script, số cụ thể trong test, `<description>` của model) → `.claude/memory.md` §1.
- **Namespace C++:** using-**declaration** từng kiểu; **không** `using namespace`; giữ nguyên `std::`/`rclcpp::` (tiền tố là thông tin, không phải nhiễu) → `.claude/rules/cpp-namespaces.md`.
- **Localization mux** là điểm hợp nhất: planner/mission chỉ đọc `/state/odometry_fused`, không tự chọn nguồn.
- **Stack công nghệ (ĐÃ CHỐT & đang cài):** **ROS2 Humble + Gazebo Harmonic + PX4 v1.15 + uXRCE-DDS**, Ubuntu 22.04 (chạy trên **WSL2** tạm thời, dual-boot sau), `ros-humble-ros-gzharmonic`, `px4_msgs`/`px4_ros_com`, BehaviorTree.CPP (cho mission BT). Nền tảng dev & workflow chi tiết ở memory §5.
  - 🎯 **Phân vai nền mô phỏng (chốt 2026-08-13):** **Gazebo lo ROS2 / framework / logic điều khiển** — đây là chỗ nó thật sự mạnh và chuyển giao sang đời thật **~90–95%**. **Thế giới 3D độ thật cao** (phục vụ nhận dạng bằng DNN) thì **nghiên cứu nền Unreal / Cosys-AirSim riêng**, chưa bắt đầu. ⚠️ Khi đánh giá nền mới, hai câu hỏi quyết định **trước** cả chuyện ảnh đẹp: *xuất được **nhãn ground-truth theo instance** không?* và *nối được **PX4 v1.15** không?*
  - ⚠️ **Sim KHÔNG phải chứng chỉ bay được.** Không có con số "sim tốt → thực tế 70–80%": tỉ lệ chuyển giao khác nhau cực lớn theo tầng (logic ~90–95% · động lực/tuning trung bình · **định vị & thị giác thấp**). Sim là **bộ lọc bug**. Chi tiết số liệu → `.claude/memory.md`.
  - **Lựa chọn Gazebo đã được tái thẩm định 2026-08-12** (nghiên cứu 13 agent) → **giữ nguyên**. Isaac Sim và Unreal đều không đi được với codebase này; Google Photorealistic 3D Tiles **bị cấm** dùng cho nhận dạng vật thể. Lý do đầy đủ → [`docs/sim-fidelity-decision.md`](docs/sim-fidelity-decision.md).
- 🔴 **Số đo an toàn phải suy từ đại lượng ĐƯỢC PHÁT, không từ trường nội bộ mô tả nó.** Ca thật (lõi quỹ đạo P6.2, 2026-08-17): `peakAcceleration()` báo **0,897 m/s²** trong khi gia tốc suy từ chính chuỗi vị trí gửi ra dây là **55,7 m/s²** — trường nội bộ tính đạo hàm *giải tích trong lòng mỗi đoạn* nên mù hoàn toàn với bậc thang tại chỗ nối. **Một con số an toàn nói dối nguy hơn không có con số nào.** → Mọi cổng/test về giới hạn phải đo từ thứ thật sự đi ra (setpoint đã phát, ảnh đã nhận, khoảng cách đã đo), không từ biến trung gian tiện tay.
- 🔴 **Một test "nhạy tải" hầu như luôn là test đang ĐO SAI ĐẠI LƯỢNG, không phải sản phẩm chập chờn.** Bốn ca thật trong một ngày (2026-08-25): `CutChain.Item8` đếm mẫu theo **lúc nhận** thay vì `header.stamp` (lúc phát) · `MissionExecutor.AuthoritySeize…` đo ân hạn từ **lời gọi hàm của chính nó** trong khi node coi *hết tươi* cũng là mất quyền · `StaleRouteFixture` có tiền đề **dựa vào may** (tuyến phát mỗi 200 ms, ngưỡng 20 ms ⇒ **10%/lượt**) · `LOCALIZATION_JUMP` áp biên động học lên dòng do nhiễu chi phối mà bỏ qua độ bất định **do chính bản tin khai**. → **Phản xạ đúng khi thấy test đỏ lúc máy bận: hỏi "đại lượng này được neo vào cái gì?" TRƯỚC khi nghi ngờ sản phẩm.** 🔴 **Cấm chữa bằng retry hay nới ngưỡng** — cả hai đều giấu mất câu trả lời.
- 🔴 **Cảm biến render đọc `<visual>`, KHÔNG đọc `<collision>`.** `gpu_lidar` và depth camera bắn tia vào visual (Harmonic không có lidar CPU). → **Không mesh visual nào được đi kèm khối va chạm nhỏ hơn nó** — nếu không, drone *thấy* tường, *bay xuyên*, test tránh vật cản đạt trong sim rồi gây tai nạn thật.

---

## 6. Nguồn tài liệu

- `dóc/Tài liệu thiết kế đầy đủ cho framework PX4 - ROS2.docx` — thiết kế chi tiết package/node/topic (**nguồn chuẩn**).
- `dóc/px4_ros2_arch.drawio.png` — sơ đồ kiến trúc khái niệm.

---

## 7. Cách làm việc (working agreement)

- Người dùng (chủ dự án) + Claude (chuyên gia) cùng phát triển. Claude được **chủ động đề xuất cải tiến** framework, nhưng bám thiết kế trong `dóc/` làm mốc.
- Tài liệu mới sẽ được người dùng thêm vào `dóc/`.
- Ngôn ngữ làm việc: **tiếng Việt**.
- **Phân vai file (theo yêu cầu người dùng):** `CLAUDE.md` chỉ là bức tranh dự án. Memory / skills / handoff / task-log sẽ tạo ở file riêng **khi được yêu cầu**, không dồn vào đây.

---

## 8. Ý tưởng cải tiến đề xuất (Claude — để thảo luận, chưa chốt)

> Ghi lại để bàn sau; chưa đưa vào thiết kế chính.
- **`uav_interfaces` versioning:** thêm header chuẩn (stamp, frame_id, `uav_id`) đồng nhất cho mọi msg để dễ log/replay và RL.
- **Diagnostics chuẩn hóa:** dùng `diagnostic_msgs` + `diagnostic_updater` để `safety_supervisor` và `localization_health` nói chung một ngôn ngữ.
- **QoS policy rõ ràng:** sensor data (BestEffort) vs command/state (Reliable) — cần định nghĩa sớm vì PX4 uXRCE-DDS nhạy với QoS.
- **Sim-first CI:** một mission smoke-test tối thiểu (arm → takeoff → goto → land) chạy trong Gazebo để bảo vệ regression.
- **RL interface:** tách một adapter mỏng từ `world_model` xuất observation/action theo chuẩn Gym để không trộn RL vào node runtime.
