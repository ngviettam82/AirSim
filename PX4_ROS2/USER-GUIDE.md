# Hướng dẫn đọc hiểu dự án — UAV Autonomy Framework (PX4 + ROS2 + Gazebo)

> **Tài liệu này viết cho người vừa nhận dự án.** Nó trả lời ba câu, theo thứ tự: *dự án này là
> gì*, *dữ liệu chảy qua nó thế nào*, và *mở file nào trước*. Cách **dùng** từng package nằm ở
> [`TUTORIAL.md`](TUTORIAL.md).
>
> Đọc hết file này mất khoảng 30 phút và sau đó bạn đọc được mã nguồn. Đọc lướt mục 1–4 mất
> 10 phút và sau đó bạn nói chuyện được về kiến trúc.

---

## 1. Dự án này là gì

Một **framework ROS2 module hoá để một UAV tự bay**: nhận nhiệm vụ cấp cao ("tuần tra các điểm
này", "bám mục tiêu kia"), tự lập đường, tự tránh vật cản, tự giám sát an toàn, và đẩy setpoint
xuống bộ điều khiển bay PX4 qua uXRCE-DDS.

- **PX4** lo phần bay: ổn định, điều khiển vòng trong, failsafe lõi.
- **ROS2** lo phần tự chủ: perception, định vị, lập kế hoạch, mission, an toàn cấp cao.
- **Gazebo** lo phần thế giới: vật lý, cảm biến render, ground truth.

Cùng một mã nguồn chạy cả trên mô phỏng lẫn phần cứng thật; khác nhau **chỉ ở file launch và
tham số**, không ở logic.

---

## 2. Ý tưởng kiến trúc phải hiểu trước tiên

Nếu bạn chỉ nhớ được một điều từ tài liệu này, hãy nhớ điều này:

> ### 🔴 Đúng **MỘT** package được phép `#include <px4_msgs/...>`: `uav_px4_backend`.
>
> Mười một package còn lại nói bằng **`uav_interfaces`** — bộ msg/srv/action nội bộ.

Vì sao nó quan trọng đến thế:

| Hệ quả | Ý nghĩa thực tế |
|---|---|
| Đổi/nâng cấp autopilot không lan quá backend | PX4 đổi API ⇒ sửa 1 package, không phải 12 |
| Mọi package khác **không biết** nó đang bay PX4 | Cùng mã chạy được với autopilot khác |
| Ranh giới **kiểm tự động được** | `bash scripts/check_px4_msgs_boundary.sh` chứng minh trên toàn cây |

Có đúng một ngoại lệ được khai báo trong toàn repo (một script probe cố tình kiểm PX4 **bên
dưới** stack của ta), nằm ở `scripts/px4_msgs_boundary_allowlist.txt`.

**Hệ quả thứ hai, ít hiển nhiên hơn:** vì `uav_interfaces` không phụ thuộc `px4_msgs`, nó phải
tự định nghĩa mọi thứ — và nó chọn **hệ quy chiếu ROS**, không phải hệ của PX4.

| | ROS (mọi thứ trong `uav_interfaces`) | PX4 (chỉ bên trong backend) |
|---|---|---|
| Hệ thế giới | **ENU** — X đông, Y bắc, **Z lên** | NED — X bắc, Y đông, **Z xuống** |
| Hệ thân | **FLU** — X trước, Y trái, Z lên | FRD — X trước, Y phải, Z xuống |

Phép đổi giữa hai hệ nằm **đúng một chỗ**: thư viện `frame_conversions` trong `uav_px4_backend`.
Không có phép đổi hệ nào rải rác nơi khác. Đơn vị: **mét, m/s, m/s², radian, giây, volt** — không
có biến thể độ hay centimét ở bất kỳ đâu.

---

## 3. Bản đồ 12 package — bốn tầng

```
                      ┌─────────────────────────────────────────┐
   TẦNG NHIỆM VỤ      │  uav_mission        (BehaviorTree)      │
                      └──────────────────┬──────────────────────┘
                                         │ ROS2 action
                      ┌──────────────────▼──────────────────────┐
   TẦNG CHUYỂN ĐỘNG   │  uav_navigation     (7 action, setpoint)│
                      └──────────────────┬──────────────────────┘
                                         │ ControlCommand
                      ┌──────────────────▼──────────────────────┐
   TẦNG QUYỀN & AN    │  uav_control_authority   uav_safety     │
   TOÀN               └──────────────────┬──────────────────────┘
                                         │ command_selected
                      ┌──────────────────▼──────────────────────┐
   TẦNG CẦU NỐI       │  uav_px4_backend    ← DUY NHẤT chạm PX4 │
                      └──────────────────┬──────────────────────┘
                                         │ uXRCE-DDS
                                    PX4 SITL / FC

   NUÔI DỮ LIỆU CHO CÁC TẦNG TRÊN:
     uav_localization  → drone đang ở đâu, tin được bao nhiêu
     uav_perception    → drone NHÌN thấy gì (khung ảnh camera)
     uav_world_model   → điều đó nghĩa là gì trong hệ thế giới
     uav_observability → chuyến bay đã xảy ra chuyện gì (bằng chứng)

   NỀN:
     uav_interfaces    → từ vựng chung (21 msg · 9 srv · 9 action)
     uav_bringup       → khởi động cả hệ (sim.launch.py / real.launch.py)
     uav_sim_gz        → thế giới mô phỏng (chỉ sim, không có bản trên máy bay thật)
```

| Package | Một câu | Node | Lib ROS-free |
|---|---|---|---|
| `uav_interfaces` | Từ vựng chung, không phụ thuộc `px4_msgs` | 0 | — |
| `uav_px4_backend` | Cầu nối hai chiều với PX4; **duy nhất** chạm `px4_msgs` | 5 | 2 |
| `uav_localization` | Mọi cảm biến vị trí → **một** odometry được phép tin | 6 | 11 |
| `uav_perception` | Ảnh → marker, vật cản, mục tiêu (khung **optical**) | 5 | 6 |
| `uav_world_model` | Quan sát khung camera → **sự thật khung thế giới** | 1 | 5 |
| `uav_navigation` | Mục tiêu → dòng setpoint 20 Hz, né vật cản | 3 | 10 |
| `uav_control_authority` | Ai đang thật sự lái — trọng tài 4 nguồn | 1 | 2 |
| `uav_safety` | 12 bộ dò, đúng 3 phản ứng: REPORT · HOLD · INHIBIT | 1 | 2 |
| `uav_mission` | BehaviorTree điều phối các action của navigator | 1 | 5 |
| `uav_observability` | Hộp đen · đèn go/no-go · dòng thời gian sự kiện | 3 | 6 |
| `uav_bringup` | Hai file launch + bài bay hồi quy | 0 | — |
| `uav_sim_gz` | World, model drone, cầu Gazebo→ROS, nguồn `/clock` | 1 | 1 |

---

## 4. 🔴 Xương sống lệnh — từ mục tiêu tới động cơ

Đây là con đường quan trọng nhất trong dự án. Mọi thứ khác phục vụ nó.

```
   mission_executor_node
          │  ROS2 action (GotoPose, Takeoff, Land, ...)
          ▼
   navigator_action_server_node          ← nơi DUY NHẤT sinh setpoint trong uav_navigation
          │  ControlCommand @ 20 Hz
          ▼
   /uav/uav0/control/cmd_mission
          │
          │   /control/cmd_safety    (ưu tiên 4 — cao nhất)
          │   /control/cmd_operator  (ưu tiên 3)
          │   /control/cmd_mission   (ưu tiên 2)   ← navigator viết vào đây
          │   /control/cmd_test      (ưu tiên 1 — thấp nhất, dùng cho bài test)
          ▼
   control_authority_manager_node        ← trọng tài, NGƯỜI VIẾT DUY NHẤT của topic dưới
          │
          ▼
   /uav/uav0/control/command_selected
          │
          ▼
   px4_command_gateway_node              ← package DUY NHẤT chạm px4_msgs
          │
          ├──▶ /fmu/in/offboard_control_mode   (nhịp "còn sống" ≥ 2 Hz)
          └──▶ /fmu/in/trajectory_setpoint     (setpoint thật)
                     │  uXRCE-DDS
                     ▼
                PX4 SITL / Flight Controller
```

### Bốn luật của con đường này — hiểu sai là rơi máy bay

**(a) Ưu tiên quyết định bởi TOPIC, không bởi nội dung bản tin.**
`SAFETY(4) > OPERATOR(3) > MISSION(2) > TEST(1)`. Nếu trường `source` trong bản tin mâu thuẫn với
topic nó tới, trọng tài **đóng dấu lại** theo topic, ghi ERROR, rồi vẫn chuyển tiếp — nó không tin
lời tự khai của bản tin. Thứ tự này được mã hoá **ngay trong giá trị số** của
`ControlAuthority.msg`, nên so sánh quyền chỉ là so sánh số nguyên; luật *"safety thắng operator
thắng mission"* nằm trong kiểu dữ liệu và **không thể hiện thực sai**.

**(b) Trọng tài KHÔNG BAO GIỜ tự chế setpoint.**
Khi mọi nguồn im lặng, nó phát **không gì cả** và `active_source` thành `NONE`. Sự im lặng đó là
cố ý: gateway có timeout 0,5 s của riêng nó, và PX4 sẽ vào failsafe. Một trọng tài tự chế hover sẽ
giữ máy bay lơ lửng vô hạn trong khi phần điều khiển đã chết.

**(c) Lệnh cũ nghĩa là IM LẶNG, không phải giữ lệnh cũ.**
Nếu không có `ControlCommand` nào trong `command_timeout_sec` (0,5 s), gateway **ngừng phát cả
nhịp offboard lẫn setpoint**, nhường cho failsafe lõi của PX4. Nó không bao giờ giữ mục tiêu cuối
và không bao giờ tự chế hover.

**(d) `OffboardControlMode` và `TrajectorySetpoint` luôn đi thành CẶP trong cùng một tick, hoặc
không đi cái nào.** Báo chế độ mà không có setpoint là đẩy PX4 vào offboard với không gì để bám —
tức trượt thẳng vào failsafe.

> 🔑 **Trường ngoài chế độ đang dùng được điền `NaN`, không phải `0`.** PX4 đọc `NaN` là *"đừng
> điều khiển trục này"*; `0` nghĩa là *"về gốc toạ độ"* — một mệnh lệnh hoàn toàn khác và tệ hơn
> nhiều.

---

## 5. Chuỗi định vị — từ cảm biến tới **một** pose

```
   gps_adapter_node        ──┐
   vio_adapter_node        ──┤
                             ├──▶ localization_mux_node ──▶ /uav/uav0/state/odometry_fused
   rangefinder_adapter_node ─┘ (không vào mux: đo ĐỘ CAO,      │
   optical_flow_adapter_node   không phải vị trí)               │
                                                                ▼
                              navigation · mission · perception · world_model · safety · backend
                              (10 nơi trên 6 package đọc topic này)
```

`/state/odometry_fused` là **điểm hợp nhất của cả hệ**. Planner và mission không bao giờ tự chọn
nguồn định vị — chúng chỉ đọc topic này.

### Ba điều phải biết về mux

1. **Nó CHỌN một nguồn, nó KHÔNG trộn.** Không có bộ lọc, không trung bình có trọng số, không
   Kalman. Đầu ra = pose của nguồn thắng + một lượng bù liên tục đang tan dần. ⇒ Độ chính xác của
   đầu ra **không bao giờ tốt hơn** nguồn được chọn. Đòi nó "lọc bớt sai số của một nguồn" là đòi
   sai chỗ.
2. **Nó chọn theo 1-sigma mà mỗi nguồn TỰ KHAI**, không theo sự thật đo được. Một nguồn trôi mà
   vẫn khai sai số nhỏ sẽ tiếp tục thắng — ca đó do `localization_health_node` bắt (kiểm bất đồng
   giữa các nguồn), không phải do mux.
3. **Nguồn không khai nổi sai số của chính nó bị coi như hỏng.** `SourceChannel` từ chối phát
   odometry khi chất lượng là `QUALITY_BAD` hoặc `QUALITY_UNKNOWN` — nó phát status với
   `is_valid=false` kèm lý do. *Không pose nào được đi ra mà thiếu độ tin cậy.*

**Khi chuyển nguồn:** độ lệch giữa hai nguồn được hấp thụ vào một offset để pose công bố **không
nhảy bậc** (REP-105). Hệ quả phải biết: trong cửa sổ đó `odometry_fused` **cố ý trễ** so với nguồn
và trượt về với tốc độ `continuity_decay_rate_mps` (0,5 m/s).

**Chuyển lên thì thận trọng, chuyển xuống thì tức thì:** một nguồn đối thủ phải tốt hơn 30%
(`switch_margin` 0,7) và giữ được 1,0 s mới được thay. Nhưng nếu nguồn đang dùng **chết**, chuyển
ngay lập tức — quán tính không bao giờ được phép giữ máy bay trên một nguồn đã biết là chết.

---

## 6. Chuỗi cảm nhận — từ điểm ảnh tới sự thật thế giới

```
   Gazebo  ──▶  gz_bridge / image_bridge  ──▶  /uav/uav0/perception/{front,down}/image_raw
                (uav_sim_gz)                   /uav/uav0/perception/front/depth_image
                                               /uav/uav0/perception/front/camera_info
                                               /clock          ← nguồn DUY NHẤT của đồng hồ sim
                                                     │
              ┌──────────────────────────────────────┤
              ▼                  ▼                   ▼
   marker_detector_node   obstacle_extractor   camera_health_node
   (camera DƯỚI)          (depth camera TRƯỚC)  (giám sát 3 luồng)
              │                  │
              ▼                  ▼
   /perception/markers   /perception/obstacles_local ──▶ target_tracker_node
              │                  │                              │
              └──────────────────┴──────────────────────────────┘
                                 │  TẤT CẢ vẫn ở khung OPTICAL
                                 ▼
                        world_model_node       ← nơi DUY NHẤT đổi optical → odom
                                 │
                                 ▼
                   /world/obstacle_map_local · /world/semantic_landmarks
                   /world/target_state · /world/mission_reference
                                 │
                                 ▼
                   route_planner · local_planner · safety · navigator
```

### Hai luật của chuỗi này

**(a) Mọi thứ `uav_perception` phát ra đều ở khung OPTICAL** — X phải, Y xuống, Z dọc trục ống
kính — và `header.frame_id` nói đúng như vậy (ví dụ `uav0/camera_down_optical`). Phần còn lại của
hệ nói khung thân (X trước, Y trái, Z lên). Hai khung lệch nhau một phép quay mà **không trình
biên dịch nào bắt được**.

**(b) `uav_world_model` là nơi DUY NHẤT đổi khung.** Detector cố ý không tự đổi. Đừng thêm phép
đổi thứ hai ở bất kỳ đâu.

**Ba điều world_model làm mà bạn cần biết:**

- Nó ghép mỗi quan sát với pose **tại chính dấu thời gian của quan sát đó**, không phải pose mới
  nhất. Ảnh về thưa và không đều; "mới nhất ghép mới nhất" sai ngay khi máy bay chuyển động.
- **Không bao giờ trung bình các lần nhìn thấy lặp lại.** Trung bình sẽ làm độ bất định co lại
  theo `1/√N` — một lời nói dối, vì sai số trội là trôi định vị, thứ tương quan mạnh theo thời
  gian: 100 lần nhìn chia sẻ gần như cùng một sai số pose.
- Mọi độ bất định công bố là **tổng bình phương của bốn thành phần**: perception, định vị (trục
  **xấu nhất** của hiệp phương sai fused), ghép cặp (`|dt| × tốc độ`), và tuổi. Theo cấu trúc, kết
  quả **không bao giờ** là `-1` và **không bao giờ** là `0`.

---

## 7. Quy ước phải biết trước khi mở bất kỳ file nào

### 7.1 Tên topic

```
/uav/<id>/<domain>/<name>          <id> mặc định = uav0
```

`<domain>` ∈ `state` · `localization` · `perception` · `world` · `planning` · `control` ·
`safety` · `mission` · `backend` · `diagnostics`.

Riêng PX4 giữ nguyên `/fmu/in/*` và `/fmu/out/*` — và chỉ `uav_px4_backend` được chạm vào chúng.

### 7.2 Service hay Action?

| | Dùng khi | Ví dụ |
|---|---|---|
| **Service** | Hành động **ngắn**, chặn, xong là xong | `Arm`, `Disarm`, `SetFlightMode`, `ClearFault` |
| **Action** | Hành động **dài**, cần feedback và huỷ được | `Takeoff`, `GotoPose`, `FollowPath`, `ExecuteMission` |

### 7.3 Luật header

Bản tin **được phát** bắt đầu bằng `std_msgs/Header header` + `string uav_id`.
Bản tin **phần tử** nằm trong mảng (`Obstacle`, `SemanticLandmark`, `TrajectoryPoint`) **cố ý
không có** — chúng thừa kế dấu thời gian và khung từ bản tin bao ngoài.

### 7.4 Khuôn "node mỏng + thư viện ROS-free"

Đây là khuôn lặp lại khắp dự án và là chìa khoá để đọc mã nhanh:

> **Logic nằm trong thư viện ROS-free** (không `#include <rclcpp/...>`), **node chỉ đấu dây**.

Ví dụ: `uav_safety` có 1 node + thư viện `failsafe_policy`; `uav_control_authority` có 1 node +
`authority_arbiter`; `uav_mission` có 1 node + `mission_policy`/`mission_registry`/`nav_goal_broker`.

Hệ quả cho người đọc: **muốn hiểu luật, đọc thư viện; muốn hiểu đấu dây, đọc node.** Và các thư
viện đó unit-test được mà không cần dựng ROS.

### 7.5 Quy ước tiền tố của 110 script

| Tiền tố | Số | Nó làm gì |
|---|---|---|
| `verify_*` | 26 | Chứng minh **một node/package** chạy thật trong sim |
| `run_*` | 18 | Chạy một **cổng** hoặc một chiến dịch đo |
| `selftest_*` | 12 | 🔑 Kiểm **chính bộ kiểm tra** — nó có biết nói KHÔNG không |
| `gate_*` | 8 | Cổng nghiệm thu |
| `check_*` | 6 | Kiểm tra tĩnh (không cần chạy sim) |
| `measure_*` | 6 | Đo một đại lượng |
| `diagnose_*` | 5 | Công cụ chẩn đoán |

> 🔑 **`selftest_*` là quy ước đáng học nhất.** Chúng không kiểm sản phẩm — chúng đưa một đột biến
> **cùng hình dạng** với thứ cần canh vào, rồi đòi bộ kiểm tra phải **đỏ**. Một cổng chưa từng
> chứng minh mình biết nói KHÔNG thì màu xanh của nó không có nghĩa gì.

---

## 8. Thứ tự đọc từng package

Đọc theo thứ tự này thì mỗi package đều dựa trên cái đã hiểu ở package trước.

### Bước 1 — Nền

**`uav_interfaces`** *(không có README — đọc contract)*
1. `docs/interface-contract-v0.1.md` §1 (sáu nguyên tắc), §2.11 (bảng `frame_id` từng bản tin)
2. `msg/ControlAuthority.msg` — thứ tự ưu tiên nằm trong chính giá trị số
3. `msg/ControlCommand.msg` — hợp đồng "chế độ nào đọc trường nào"
4. `msg/ResultCode.msg` — bảng kết quả dùng chung cho cả 9 action
5. `test/test_interface_conventions.py` — cảnh sát quy ước, chạy trong ctest

**`uav_px4_backend`** *(không có README — đọc mã)*
1. `package.xml` — khai ranh giới R1 trong ba dòng
2. `include/uav_px4_backend/frame_conversions.hpp` → `src/frame_conversions.cpp` — từ vựng
   ENU/FLU ↔ NED/FRD mà mọi node dưới đây dùng
3. `src/px4_state_adapter_node.cpp` — nửa **vào** của cầu
4. `src/px4_command_gateway_node.cpp` — nửa **ra** của cầu
5. `src/offboard_session_manager_node.cpp` — máy trạng thái 5 pha vào offboard an toàn

### Bước 2 — Trạng thái

**`uav_localization`** → `README.md`, rồi `include/.../source_channel.hpp` (*"không pose nào
thiếu độ tin cậy"* viết thành mã), rồi `src/localization_mux_node.cpp`.

**`uav_world_model`** → `README.md` (đặc biệt bảng nêu nguồn của **từng hằng số hình học**), rồi
`config/world_model_params.yaml`.

### Bước 3 — Cảm nhận & thế giới mô phỏng

**`uav_perception`** → `README.md`, bắt đầu ở mục *"package này KHÔNG có driver camera"*, rồi
`config/perception_params.yaml`.

**`uav_sim_gz`** → `README.md` §1 (thang bốn tầng model drone), §3 (bảng topic gz→ROS), §3b
(các world), §5 (mười cái bẫy đã trả giá).

### Bước 4 — Chuyển động

**`uav_navigation`** — package lớn nhất. `README.md` (850 dòng) là lời giải thích tốt nhất về
package này; mục 1–5 cho actions, máy trạng thái, luật nhận goal, chuỗi engagement và mô hình
đồng thời.

### Bước 5 — Quyền & an toàn

**`uav_control_authority`** → `README.md` §1 (bảng bốn nguồn) và §2 (sáu điều phải biết trước khi
sửa), rồi `include/.../authority_arbiter.hpp` — cả hợp đồng nằm trong ghi chú header.

**`uav_safety`** → `README.md` (bảng taxonomy), rồi `include/.../failsafe_policy.hpp` — 12 mã,
mọi giá trị mặc định kèm **lý do**.

### Bước 6 — Nhiệm vụ & bằng chứng

**`uav_mission`** → `README.md` (4 điều cấm, hình dạng 1-node/3-lib có chủ đích, luật *"guard
trong code, thân mission trong XML"*), rồi ba file mission XML — chúng ngắn và là bức tranh rõ
nhất về việc một mission thật sự làm gì.

**`uav_observability`** → `README.md`, cụ thể bảng *"7 điều PHẢI biết trước khi sửa"*.

### Bước 7 — Ráp lại

**`uav_bringup`** → `README.md`, rồi `launch/node_manifest.py` (60 dòng dữ liệu thuần — **toàn bộ
tập node của cả hệ trong một màn hình**), rồi `launch/sim.launch.py`.

---

## 9. Mười lăm điều bất ngờ nhất trong dự án này

Gom lại đây vì mỗi cái đều là một quyết định thiết kế mà đọc mã không đoán ra được.

| # | Điều bất ngờ |
|---|---|
| 1 | **Safety không bao giờ tự hạ cánh.** Mất định vị ⇒ nó **im lặng có chủ đích** (INHIBIT), để failsafe lõi PX4 tiếp quản và **trả quyền cho phi công**. Nó không bao giờ phát setpoint trên một pose không tin được. |
| 2 | **Safety không bao giờ ra lệnh land/RTL/climb.** Hạ cánh hay về nhà là việc của mission, đi qua navigator. |
| 3 | **Đúng 4 trong 12 mã của safety là "latch"** (giành quyền được): `BLIND_COMMAND`, `BATTERY_PX4_NO_ACTION`, `FRAME_MISMATCH` → INHIBIT; `OBSTACLE_TOO_CLOSE` → HOLD. Tám mã còn lại chỉ **báo cáo**. |
| 4 | **Trọng tài không tự chế setpoint.** Mọi nguồn im ⇒ nó phát **không gì cả**. |
| 5 | **Rác không phải bằng chứng của sự sống.** Một kênh phát NaN/sai khung ở 20 Hz **không** được tính là "còn sống": nó không bao giờ thắng quyền và không bao giờ giữ được quyền. |
| 6 | **Navigator nhận đúng MỘT nhiệm vụ tại một thời điểm**, và chỉ từ trạng thái đã ổn định (IDLE hoặc HOLDING). Không hàng đợi, không tự động chiếm quyền. |
| 7 | **`Recover` là action DUY NHẤT được chiếm quyền** giữa chừng. |
| 8 | **Huỷ (cancel) không buông tay lái.** Setpoint bị **đóng băng tại chỗ** và dòng 20 Hz **vẫn chạy**. Ngừng dòng khi đang bay là mất offboard. |
| 9 | **Huỷ trong lúc LAND bị TỪ CHỐI.** Khi đã lệnh hạ cánh, autopilot sở hữu máy bay. |
| 10 | **Offboard TRƯỚC, arm SAU.** Ngược lại là arm một máy bay chưa ai điều khiển. |
| 11 | **Mission không bao giờ điều khiển máy bay** — 4 điều cấm: không `ControlCommand`, không `clear_fault`, không `arm/disarm/set_mode`, không `SetControlAuthority`. |
| 12 | **Tối đa MỘT goal navigator tồn tại**, node-wide, vĩnh viễn — bảo đảm bởi `NavGoalBroker`. |
| 13 | **`AbortMission` không hạ cánh** — nó dừng mission và **giữ vị trí**. |
| 14 | **`go_no_go` có BA giá trị**, không phải hai: `GO` · `NO_GO` · **`UNKNOWN`**. Coi *"khác GO"* là `NO_GO` sẽ phá huỷ đúng thứ phân biệt *"đo được là xấu"* với *"không đo được"*. |
| 15 | **Hộp đen không bao giờ chặn được cất cánh.** Hai nguồn chẩn đoán do blackbox sở hữu **không bao giờ** được tính vào `go_no_go`. |

---

## 10. Đi đâu tiếp

| Cần gì | Đọc gì |
|---|---|
| **Cách dùng từng package** | [`TUTORIAL.md`](TUTORIAL.md) |
| Bức tranh dự án, quy ước, stack công nghệ | [`CLAUDE.md`](CLAUDE.md) |
| Lý do thiết kế của từng msg/srv/action | [`docs/interface-contract-v0.1.md`](docs/interface-contract-v0.1.md) |
| Chính sách QoS — 4 nhóm, danh sách topic latched | [`docs/qos-policy-v1.md`](docs/qos-policy-v1.md) |
| Bẫy công cụ / môi trường đã trả giá | [`docs/ops-playbook.md`](docs/ops-playbook.md) |
| Trạng thái & giả định từng package | [`docs/package-status.md`](docs/package-status.md) |
| Sim chứng minh được gì và **không** chứng minh được gì | [`docs/sim-boundary-statement.md`](docs/sim-boundary-statement.md) |
| Checklist trước khi nghĩ tới bay thật | [`docs/preflight-checklist.md`](docs/preflight-checklist.md) |
| Tám nguồn tham khảo trong `dóc/pdf/` là bài nào | [`docs/references.md`](docs/references.md) |
