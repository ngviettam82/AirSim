# Interface Contract v0.1 — `uav_interfaces`

> Nơi chứa **lý do thiết kế** của hợp đồng giao tiếp. Code chỉ giữ ghi chú tối giản (R16); mọi giải thích dài nằm ở đây.
>
> Cập nhật: 2026-08-20 · Trạng thái: phần lõi điều khiển/an toàn đã build sạch; phần quan sát/báo cáo đang chờ bàn giao. Bổ sung 2026-08-15: 5 quy ước frame/field chốt (§2.7–§2.11). Bổ sung 2026-08-20: §2.16 hợp đồng arbitration (P7) + §2.11 `ControlCommand`/`ControlAuthority` từ planned → chạy thật; cùng ngày sửa §2.16(c)/(g) theo review B2 (liveness chỉ tính message hợp lệ, gỡ cơ chế đếm demote thừa) — interface v0.1 vẫn ĐÓNG BĂNG, không đổi/xoá trường nào. Bổ sung 2026-08-24 (P10.9b): §2.20 thiết kế lại ngữ nghĩa `go_no_go` (pha đo được từ `armed`, waiver per-child, luật never-waive NW1/NW2) + §2.18 thêm 1 dòng ràng buộc ngược — vẫn KHÔNG thêm/đổi/xoá msg/srv nào, chỉ đổi param/KeyValue của node quan sát.

---

## 1. Nguyên tắc nền

| # | Nguyên tắc | Vì sao |
|---|---|---|
| 1 | **Không phụ thuộc `px4_msgs`** | Chỉ `uav_px4_backend` được chạm API của bộ điều khiển bay. Đổi autopilot không phải sửa cả hệ (R1). |
| 2 | **Mọi hình học ở hệ ROS: ENU / FLU** | ENU = X đông, Y bắc, Z **lên**. Chuyển sang NED/FRD **chỉ** ở `px4_frame_bridge_node` (R5). Convert rải rác = lỗi dấu, lan toàn hệ. |
| 3 | **Đơn vị SI tuyệt đối** | mét, m/s, m/s², **radian**, giây, Volt. Không độ, không cm. |
| 4 | **Header chuẩn** | Mọi `.msg` gửi đi mở đầu bằng `std_msgs/Header header` + `string uav_id`. Message thành phần và `.srv` **không** có. |
| 5 | **Không mảng đa chiều** | DDS hay lỗi. Cần ma trận → mảng phẳng + ghi kích thước. |
| 6 | **Không đặt giá trị mặc định trong interface** | Mặc định thuộc về phía gọi. Interface chỉ định nghĩa hình dạng dữ liệu. |

**Vì sao dùng `std_msgs/Header` thay vì tự tạo kiểu header riêng:** để hai nhánh công việc (lõi điều khiển & quan sát) **không phụ thuộc nhau** — người viết message quan sát không phải chờ file header của người kia tồn tại.

---

## 2. Quyết định thiết kế đáng chú ý

### 2.1 `ResultCode.msg` — bảng mã lỗi dùng chung
Message chỉ chứa hằng số, **không bao giờ publish**. Cả 9 action dùng chung bộ mã này thay vì mỗi action tự nghĩ mã riêng.

**Lý do:** khi tầng nhiệm vụ (Behavior Tree) phải phân nhánh theo loại lỗi, một bảng mã thống nhất là điều kiện bắt buộc. 9 bảng mã rời rạc sẽ khiến logic xử lý lỗi không viết nổi.

*Đây là bổ sung ngoài thiết kế gốc → tổng số message 15 → 16.*

### 2.2 `ControlAuthority` — mã hoá thứ tự ưu tiên vào giá trị số
```
SOURCE_NONE=0 < SOURCE_TEST=1 < SOURCE_MISSION=2 < SOURCE_OPERATOR=3 < SOURCE_SAFETY=4
```
Giá trị số **trùng với thứ tự ưu tiên**, nên so quyền chỉ là phép so sánh số — không cần bảng tra, không hiện thực sai được. Luật *an toàn > người vận hành > nhiệm vụ* (R6) nằm luôn trong kiểu dữ liệu.

> ⚠️ **Không đổi giá trị số** nếu chưa rà toàn hệ thống.

### 2.3 `ControlCommand` — chọn chế độ tường minh thay vì NaN
Bộ điều khiển bay dùng quy ước "trường khác 0 đầu tiên quyết định loại setpoint", trường thừa điền NaN. Quy ước đó **dễ sai khi quên gán giá trị**.

Bản nội bộ có `control_mode` nói rõ trường nào có nghĩa:

| `control_mode` | Trường được đọc |
|---|---|
| `MODE_POSITION` | `position`, `yaw` |
| `MODE_VELOCITY` | `velocity`, `yaw_rate` |
| `MODE_ACCELERATION` | `acceleration`, `yaw` |
| `MODE_ATTITUDE` | `orientation`, `thrust` |
| `MODE_BODY_RATE` | `body_rate`, `thrust` |

`MODE_UNKNOWN` là giá trị không hợp lệ — lớp cầu nối **phải từ chối**, không được đoán.

**Nhịp phát:** lệnh này phải phát liên tục **≥ 2 Hz** (thực tế 10–20 Hz). Ngắt nhịp → bộ điều khiển bay thoát chế độ tự hành → kích hoạt failsafe. `offboard_session_manager_node` chịu trách nhiệm giữ nhịp.

### 2.4 Hai "foot-gun" cố ý KHÔNG đưa vào API

| Thứ bị loại | Lý do |
|---|---|
| Tuỳ chọn **arm cưỡng bức** trong `Arm.srv` | Bỏ qua kiểm tra trước bay là nguyên nhân tai nạn phổ biến. Cần force-arm để thử nghiệm → dùng phần mềm mặt đất hoặc console autopilot, không đi qua API hệ thống. |
| **Tắt động cơ khi đang bay** trong `Disarm.srv` | Tắt động cơ trên không = rơi tự do. Ngắt khẩn cấp phải là **công tắc phần cứng RC độc lập**, không qua phần mềm (phần mềm có thể treo). |

`Disarm` chỉ được phép thành công khi phương tiện **đã ở dưới đất**.

### 2.5 `Recover.action` — mất định vị thì trả quyền, KHÔNG tự hạ cánh
🔴 **Đây là điểm sát sinh mạng nhất của hợp đồng này.**

Phản xạ tự nhiên khi mất định vị là cho hạ cánh tự động. Nhưng **khi đã mất định vị thì phương tiện không biết mình đang ở đâu** — "hạ cánh tự động" có thể hạ trúng người hoặc chướng ngại vật.

Bằng chứng thực nghiệm trong tài liệu tham khảo của dự án nghiêng về **trả quyền cho người lái** (`TYPE_HANDOVER_TO_PILOT`). Điều này **mâu thuẫn với `CLAUDE.md §4.8`** (đang ghi "mất localization → land") → mục đó cần được rà lại.

Việc ánh xạ *loại sự cố → phương án khôi phục* phải do người phụ trách chốt và ghi lại, **không để mặc định ngầm trong code**.

🔴 **Ngữ nghĩa `TYPE_HOLD` (chốt 2026-08-20, E2):** đây là loại khôi phục **DUY NHẤT không đọc pose**, nên
`navigator_action_server_node` (a) **không** chặn nó ở cổng độ tin cậy định vị, (b) **tắt hai phép kiểm
định vị** trong bộ dò lỗi cho riêng nó, và (c) coi là **xong theo bằng chứng dòng lệnh** — setpoint đứng
yên **và** stream còn chảy đủ `arrival_settle_sec` — chứ **không** theo `khoảng cách(pose, điểm giữ)`.
Nếu không: đúng lúc mất định vị, tầng an toàn (P7–P8) sẽ đọc được *"khôi phục THẤT BẠI"* ở đúng nút leo
thang, mà đường leo thang còn lại chỉ còn `TYPE_LAND`/`TYPE_HANDOVER_TO_PILOT` — ngược đúng giáo lý của
mục này. Bằng chứng "biết mà vẫn giữ" phải nằm trong `result.message`, không được giấu.

### 2.6 `Path3D` vs `Trajectory3D`

| | Nội dung | Ai sinh ra |
|---|---|---|
| `Path3D` | Chỉ **hình học** — đi qua những đâu | Bộ hoạch định tuyến |
| `Trajectory3D` | Có thêm **thời gian & vận tốc** — khi nào tới đâu | Bộ sinh quỹ đạo |

`Path3D` dùng `geometry_msgs/Pose[]` (vị trí + hướng trong một kiểu) thay vì tách hai mảng song song vị trí/góc — để không bao giờ lệch chỉ số giữa hai mảng.

`TrajectoryPoint` là **message thành phần**, luôn nằm trong `Trajectory3D`, nên không mang header riêng.

⚠️ **`Trajectory3D` mô tả HÌNH DẠNG + NHỊP DANH NGHĨA, không phải một lịch theo giờ tường** (chốt
2026-08-18 khi có publisher đầu tiên). Đồng hồ thực thi thuộc về executor và nó **đóng băng khi dây
xích kẹp** — xem §2.14.

**Chốt 2026-08-19 (Đ1, thuần bổ sung):** cả `Path3D` và `Trajectory3D` thêm `uint8 plan_state`
(hằng số `PLAN_STATE_UNKNOWN=0 / VALID=1 / NO_GOAL=2 / WITHDRAWN=3 / FAILED=4`) + `string reason`.
`Trajectory3D` thêm thêm `uint32 sequence` (tăng mỗi bản kế hoạch mới, để so được hai quỹ đạo kế
tiếp). `is_valid` **giữ nguyên** — `true` ⇔ `plan_state == PLAN_STATE_VALID`, chỉ để không phá code
đang đọc `is_valid`. Ba nghĩa cũ mà `is_valid=false` từng gánh chung, giờ tách theo `route_planner_node`
(publisher hiện có của `Path3D`):

| Nghĩa cũ (gánh chung trong `is_valid=false`/`planner_name`) | `plan_state` mới | Khi nào |
|---|---|---|
| "chưa có kế hoạch" — thiếu dữ liệu đầu vào để lập kế hoạch | `PLAN_STATE_NO_GOAL` | chưa có `odometry_fused` (chưa có goal thì node im lặng, không publish — không phải lỗi) |
| "lập kế hoạch thất bại" — có đủ đầu vào nhưng không ra được kế hoạch dùng được | `PLAN_STATE_FAILED` | obstacle map bắt buộc mà cũ/thiếu; A* không tìm được đường; spline tighten phá map |
| "kế hoạch bị thu hồi" — một kế hoạch từng hợp lệ bị rút lại | `PLAN_STATE_WITHDRAWN` | chưa có publisher nào phát ra trạng thái này (dự phòng cho mission huỷ goal giữa chừng) |

🔵 **`PLAN_STATE_WITHDRAWN` dành cho P9 (`uav_mission`) — v0.1 chưa có emitter nào phát hằng số này**
(chốt 2026-08-20, đóng phần mơ hồ của nợ N4). Hằng số vẫn giữ nguyên trong `.msg`, không xoá — chỉ ghi
rõ chưa ai dùng, để consumer không đoán nhầm là "đã hiện thực nhưng chưa test".

`planner_name` **trở lại đúng vai ban đầu** — tên thuật toán (`"a_star"` / `"a_star:projected"`),
rỗng khi thuật toán chưa từng chạy. Lý do thất bại giờ nằm ở `reason`, không còn mượn `planner_name`.

**Chốt 2026-08-19 (thuần bổ sung, lượt hai):** `Path3D` thêm `builtin_interfaces/Time goal_stamp` —
`header.stamp` của `/planning/route_goal` mà bản tuyến này trả lời, bằng 0 khi node chưa có goal nào.

🔴 **Vì sao cần:** trước đó consumer chỉ lọc được theo **thời điểm nhận**. `route_planner_node` phát
**5 Hz**, nên một bản tuyến tính cho goal **TRƯỚC** vẫn hạ cánh **tới 200 ms sau** lúc goal mới được
hỏi — và được nhận nhầm. Cùng loại lỗi với *"im lặng không được đọc thành CLEAR"* (§2.15): **thứ tự
thời gian không phải danh tính.** Navigator đối chiếu `goal_stamp` với câu hỏi nó vừa gửi, ở **cả hai**
chỗ: lúc chờ tuyến (`requestRoute`) và lúc ghép lại tuyến sau một escape (`routeTailAfter`). Không
khớp ⇒ coi như **chưa có tuyến**, không phải "có tuyến lạ".

⚠️ Publisher nào không điền trường này thì `goal_stamp` = 0 ⇒ navigator **không bao giờ khớp** ⇒ rơi
về đường thẳng và **nói ra** (`"route planner silent"`). Đó là hướng fail-safe có chủ ý, nhưng ai viết
publisher `Path3D` mới thì phải điền.

### 2.7 `Obstacle.size` — FULL EXTENT, không phải bán kính

Chốt 2026-08-15. `size` (`geometry_msgs/Vector3`) là **kích thước toàn phần** theo 3 trục — hộp 1×1×2 m → `size=(1,1,2)`. Consumer muốn bán kính né tránh (P6 local planner, P5.6 world_model) tự lấy `size/2`.

**Vì sao phải chốt:** comment cũ *"bounding extents, m"* mơ hồ — không nói rõ full hay half-extent. `P5.4 obstacle_extractor_node` (bên ghi) và `P6 local_planner_node` (bên đọc) là hai node khác nhau, viết ở hai thời điểm khác nhau; nếu hiểu sai chiều nhau thì vật cản trong bản đồ **nhỏ đi một nửa** → va chạm. Đây là lý do trường này từng chặn cả P5.4 (xem `.claude/memory.md` mục nợ P5.4).

### 2.8 `GotoPose.frame_id` — chỉ nhận `"odom"` (v0.1)

Chốt 2026-08-15. Server `GotoPose` **CHỈ** chấp nhận `frame_id == "odom"`. Rỗng hoặc bất kỳ giá trị nào khác → **REJECT goal ngay khi nhận**, không đoán, không tự quy đổi.

**Vì sao v0.1 chỉ có `"odom"`:** hệ thống hiện chưa có map/SLAM — `"odom"` là frame duy nhất tồn tại và có ý nghĩa. Khi có TF `map → odom` (relocalization/SLAM) thì mở rộng nhận thêm `"map"` và convert qua TF ở lớp navigation — action server vẫn không được tự đoán.

**Vì sao REJECT chứ không "cứ dùng luôn":** sai frame ở action điều hướng = bay lạc vị trí (🔴 sát an toàn theo R0) — `GotoPose` là action lái drone thẳng tới toạ độ, không có lớp nào ở sau chặn lại nếu frame sai.

✅ **Enforcement ĐÃ HIỆN THỰC ở P6** (đóng 2026-08-20): `goal_admission.cpp` từ chối bằng `RejectReason::INVALID_FRAME` (`:146`, `:201`); ghim bằng `test_goal_admission.cpp:352` ("foreign frame") và `test_navigator_action_server_node.cpp:1461` (`GotoInAForeignFrameIsRejectedWhileTheSameGoalInOdomIsAccepted` — có **đối chứng dương**: cùng goal ở `odom` phải được nhận).

### 2.9 `InspectMarker.marker_frame_id` (result) — luôn `"odom"`

Chốt 2026-08-15. Trường `marker_frame_id` trong **result** luôn là `"odom"`. Server tự transform pose marker (quan sát được ở frame camera quang học, xem `MarkerObservation` ở bảng §2.11) sang `odom` trước khi trả kết quả — caller (mission/BT) không phải tự lo transform. Cùng lý do v0.1 với §2.8: chỉ `"odom"` có ý nghĩa ở giai đoạn hiện tại.

### 2.10 `Land.marker_id` — bị bỏ qua khi không precision landing

Chốt 2026-08-15. Khi `use_precision_landing = false`, server **không đọc/validate** `marker_id` — trường chỉ có ý nghĩa khi `use_precision_landing = true`. Không cần giá trị sentinel (vd `-1`) cho trường hợp tắt; caller để giá trị gì cũng được, server phải bỏ qua hoàn toàn.

### 2.11 Frame conventions — `frame_id` cho từng msg có `Header`

Chốt 2026-08-15, sau khi **grep publisher thật trong `src/`** (không suy đoán). Nguyên tắc: **codify hành vi đang chạy thật**; msg chưa có publisher thì ghi quy ước dự kiến và đánh dấu **planned**.

| Msg | `frame_id` | Trạng thái | Bằng chứng |
|---|---|---|---|
| `VehicleState` | *(rỗng)* | ✅ chạy thật, khớp quy ước | `px4_state_adapter_node.cpp::publishVehicleState()` không set `frame_id` |
| `VehicleHealth` | *(rỗng)* | ✅ chạy thật, khớp quy ước | `px4_state_adapter_node.cpp::publishHealth()` không set `frame_id` |
| `OffboardStatus` | *(rỗng)* | ✅ chạy thật, khớp quy ước | `offboard_session_manager_node.cpp::publishStatus()` không set `frame_id` |
| `MarkerObservation` | `"<uav_id>/camera_down_optical"` | ✅ chạy thật, khớp quy ước | `marker_detector_node.cpp`, param `optical_frame` (mặc định `<uav_id>/camera_down_optical`) |
| `LocalizationStatus` | ⚠️ **`"odom"`** (GPS/VIO/Fused) **hoặc `"base_link"`** (optical-flow) | ✅ chạy thật, **KHÔNG khớp giả định "rỗng" ban đầu** | xem §2.11.1 ngay dưới |
| `ObstacleArray` | `"<uav_id>/camera_front_optical"` trên `/perception/*`; `"odom"` trên `/world/*` (§2.13) | ✅ chạy thật 2026-08-15 — `obstacle_extractor_node.cpp` | trên `/perception/*` là frame quang học camera trước, stamp = stamp ảnh nguồn |
| `TargetTrack` | tên frame chứa pose target (giá trị cụ thể chưa chốt) | ⏳ planned — P5.5 chưa build | comment sẵn trong `.msg`: *"frame_id names the frame pose is expressed in"* |
| `ControlCommand` | `"odom"` | ✅ chạy thật 2026-08-20 — `navigator_action_server_node` phát trên `cmd_mission`; `control_authority_manager_node` validate `frame_id == odom_frame` trước khi cho qua bất kỳ kênh nào (§2.16d) | `px4_command_gateway_node.cpp` vẫn **không đọc** `header.frame_id` — kiểm tra nằm ở trọng tài, không phải gateway |
| `Trajectory3D` | `"odom"` | ✅ chạy thật 2026-08-18 — `navigator_action_server_node.cpp::toMessage()` | publisher DUY NHẤT là navigator; QoS **latched**, xem §2.14 |
| `Path3D` | `"odom"` (suy ra) | ⏳ planned — chưa có publisher, chờ P6 | cùng lý do §2.8 |
| `MissionStatus` | *(rỗng)* | ⏳ planned — chưa có publisher, chờ P9 `uav_mission` | không có trường hình học |
| `MissionEvent` | *(rỗng)* | ⏳ planned — chưa có publisher, chờ P9 | không có trường hình học |
| `SafetyState` | *(rỗng)* | ⏳ planned — chưa có publisher, chờ P8 `uav_safety` | không có trường hình học |
| `ControlAuthority` | *(rỗng)* | ✅ chạy thật 2026-08-20 — `control_authority_manager_node` phát trên `/control/authority` (khi đổi + nhịp tim 2 Hz, §2.16b) | không có trường hình học |
| `SemanticLandmarkArray` | `"odom"` | ⏳ planned — publisher là `world_model_node` (P5.6) | frame cố định theo hợp đồng §2.13 |
| `TargetState` | `"odom"` | ⏳ planned — publisher là `world_model_node` (P5.6) | frame cố định theo hợp đồng §2.13 |

**4 message KHÔNG có `Header` (ngoài bảng trên):** `ResultCode` (chỉ hằng số, không bao giờ publish), `TrajectoryPoint` (nested trong `Trajectory3D`), `Obstacle` (nested trong `ObstacleArray`), `SemanticLandmark` (nested trong `SemanticLandmarkArray`) — các message nested "kế thừa" frame/timestamp từ message cha, đã ghi rõ ở dòng đầu mỗi file.

#### 2.11.1 ⚠️ Phát hiện lệch quy ước: `LocalizationStatus.frame_id`

Giả định ban đầu (trước audit): msg trạng thái phi không gian — không có trường x/y/z — thì để `frame_id` **rỗng**. `LocalizationStatus` không có trường hình học nên rơi vào nhóm này theo giả định đó. Nhưng grep code thật (2026-08-15) cho thấy **không rỗng**:

- `source_channel.cpp` (dùng chung bởi `gps_adapter_node`, `vio_adapter_node`, `localization_mux_node`) set `status.header.frame_id = options_.odom_frame` — tham số mặc định `"odom"`.
- `optical_flow_adapter_node.cpp` (viết riêng, không qua `SourceChannel`, vì flow chỉ đo vận tốc chứ không có pose) set `status.header.frame_id = base_frame_` — tham số mặc định `"base_link"`.

Đây **không phải ngẫu nhiên**: mỗi nguồn đang gán `LocalizationStatus.frame_id` **trùng** với `frame_id`/`child_frame_id` của message hình học song hành trên topic anh em — `nav_msgs/Odometry.header.frame_id = "odom"` cho GPS/VIO/Fused; `geometry_msgs/TwistWithCovarianceStamped.header.frame_id = "base_link"` cho optical-flow (flow đo vận tốc **thân**, "odom" không có nghĩa cho phép đo đó). Đây là một quy ước mạch lạc, nhất quán nội bộ — chỉ khác với giả định "rỗng" đặt ra ban đầu.

`test_source_channel.cpp` chỉ pin `"odom"`/`"base_link"` trên `nav_msgs/Odometry` (dòng test `EXPECT_EQ(odometry_[0].header.frame_id, "odom")`), **không** assert trên `LocalizationStatus` — hành vi trên `LocalizationStatus` là thật (deterministic theo default tham số) nhưng chưa được test khoá lại.

**Đã xử lý hôm nay:** đổi comment trong `.msg` thành `# frame_id: source-dependent, see contract` — không khẳng định rỗng (sai sự thật), không tự quyết cách sửa (ngoài phạm vi audit hôm nay). Chủ dự án chọn 1 trong 2 hướng:
1. **Chốt hành vi hiện tại làm quy ước chính thức** (sửa bảng trên từ "KHÔNG khớp" thành quy tắc mới: "frame_id mirror theo topic anh em"), hoặc
2. **Sửa code trả `frame_id` rỗng** cho khớp giả định ban đầu (đổi 2 file `.cpp`, ngoài phạm vi hôm nay vì chỉ được đụng comment).

### 2.12 `position_uncertainty` — độ bất định vị trí theo mét (duyệt 2026-08-15)

Thêm `float32 position_uncertainty  # 1-sigma, m` vào `MarkerObservation`, `Obstacle`, `TargetTrack`. Lý do: cổng S6 của P5 đòi `/world/*` mang độ bất định **theo mét** (= perception ⊕ định vị, plan P5 §0.2); `confidence` không đơn vị nên không cộng được với sai số định vị. Tên trường theo tiền lệ `LocalizationStatus.position_uncertainty` (cùng nghĩa 1-sigma m) — không dùng `position_stddev` (tên tầng hiện thực C++ của `uav_localization`).

Quy tắc sentinel — theo idiom `kUnknownCovariance = -1.0` sẵn có của `uav_localization`:
- **Tầng `/perception/*` (quan sát thô):** `-1` được phép — detector v0.1 có thể chưa ước lượng được.
- **Tầng `/world/*` (`world_model_node` phát):** **cấm `-1`** — tối thiểu luôn cộng được phần sai số định vị từ `odometry_fused`; phần perception là `-1` thì thay bằng sàn đo được của detector (ghi nguồn số).
- `0` nghĩa là "chính xác tuyệt đối" — không bao giờ hợp lệ cho phép đo thật (cùng bẫy đã ghi ở quy tắc covariance của `uav_px4_backend`).

### 2.13 Lớp `/world/*` dùng message RIÊNG (duyệt 2026-08-15)

`world_model_node` (P5.6) **không republish** message perception lên `/world/*`. Lý do: frame của message perception gắn theo **type** ("frame_id names the observing camera") — cùng type mà frame đổi theo topic là phá bảng §2.11. Cùng họ tiền lệ `Path3D` vs `Trajectory3D` (§2.6): quan sát thô và ước lượng world là hai tầng ngữ nghĩa.

| Topic `/uav/<id>/world/…` | Kiểu | Ghi chú |
|---|---|---|
| `semantic_landmarks` | `SemanticLandmarkArray` (mới, chứa `SemanticLandmark[]`) | frame `"odom"`; mỗi landmark mang `position_uncertainty` + `time_since_seen_sec` (tuổi quan sát — ràng buộc plan P5 §0.1b) |
| `target_state` | `TargetState` (mới) | frame `"odom"`; status values theo `TargetTrack`; nghĩa `time_since_seen_sec` + chính sách chọn track ngay dưới |
| `obstacle_map_local` | `ObstacleArray` (tái dùng) | comment header đã nới: sensing frame **hoặc** `"odom"` trên `/world/*`; `Obstacle` là hình học thuần nên không dính frame-theo-type |
| `mission_reference` | `geometry_msgs/PoseStamped` | kiểu chuẩn, không cần msg mới |

*(Lệch nhỏ so với phê duyệt gốc "2 msg mới": cần **3 file** vì landmark theo mẫu component+array giống `Obstacle`/`ObstacleArray` — bản đồ phát theo lô, consumer không phải tự tích luỹ trạng thái.)*

**`TargetState.time_since_seen_sec` = TUỔI QUAN SÁT, không phải tuổi message (siết 2026-08-22, bug N1):** `= TargetTrack.time_since_seen_sec` (tracker khai lúc đóng dấu) **+** `(lúc phát − header.stamp của TargetTrack)` — **cộng dồn**, không thay thế, không lấy `max`. Đọc là *"đã bao lâu rồi mục tiêu KHÔNG được nhìn thấy"*; `header.stamp` của `TargetState` vẫn là publish time (§2.11) nên **không** suy được độ tươi từ nó. Cùng tuổi đó dùng để nới `position_uncertainty` và để `target_forget_sec` quên mục tiêu. *(Trước bản vá, trường này bị ghi đè bằng tuổi đường truyền: tracker báo 3,032 s, world model phát 0,972 s trên cùng dòng dữ liệu.)*

**Chính sách chọn track — BÁM DÍNH `track_id` (siết 2026-08-22, bug N3):** `target_tracker_node` phát **mỗi track một message trên cùng topic** (kể cả track LOST bị thải), nên `/world/target_state` phải chọn, không được lấy "message cuối". Luật: (1) lần đầu chọn track **tốt nhất trong lô cùng `header.stamp`** — thứ tự hơn thua *không-LOST trước · `time_since_seen` nhỏ hơn · `track_id` nhỏ hơn* (thứ tự này khiến kết quả **độc lập với thứ tự message tới**); (2) sau đó **chỉ track đó cập nhật**, track khác bị bỏ qua và đếm vào `unselected_track` của log định kỳ; (3) đổi track chỉ khi track đang bám **LOST** hoặc **im lặng > `target_track_switch_after_sec`** (mặc định 1,0 s; giữ ≥ 2× chu kỳ phát `target_track`) — **track LOST đến sau không bao giờ cướp được track sống**; (4) mỗi lần đổi phát một WARN `target selection switched to track <id>` (interface đóng băng, không thêm trường msg). Consumer phải coi `track_id` đổi là **cạnh gián đoạn** của ước lượng, không phải chuyển động của mục tiêu. *(Trước bản vá, track cuối lô thắng tuỳ tiện: `track_id=1` lọt 7/4870 mẫu, gây 2 cú nhảy ước lượng 1,92 và 1,94 m.)*

🔴 **Nguồn phát đã siết thêm ở tầng `target_tracker_node` (bug #10, G-M4.4, 2026-08-23) — chữ, không đổi trường:** "mỗi track một message" ở trên **chỉ còn đúng cho track đã CONFIRMED** (xác nhận M-của-N). Trước bản vá này, track mới publish ngay từ khớp đầu tiên — nhiễu 1-khung đẻ track KHÔNG-LOST liên tục, mỗi track ma vừa sinh thắng luật (3) ở trên (không LOST → được chọn ngay khi track cũ vượt `target_track_switch_after_sec`) dù không phải mục tiêu thật, gây **18 lần switch** trong một chặng không còn vật thật (xem `docs/package-status.md` §6). Sau bản vá, track chỉ xuất hiện trên topic sau khi đã khớp ổn định — luật (1)–(4) ở trên **không đổi logic**, chỉ nay chọn giữa một tập track đáng tin hơn.

### 2.14 `/planning/trajectory` — topic LATCHED, một publisher, và `start_time` chỉ là danh nghĩa (chạy thật 2026-08-18)

`navigator_action_server_node` là publisher đầu tiên và **duy nhất** của `Trajectory3D` (P6.2/T1.3).

| Điều | Chốt | Vì sao |
|---|---|---|
| **QoS** | `KeepLast(1)` + `Reliable` + **`TransientLocal`** | Ngữ nghĩa là *"kế hoạch đang có hiệu lực"*, không phải dòng sự kiện. Node vào muộn phải thấy ngay. **Depth > 1 là nguy hiểm chứ không an toàn hơn**: reader vào muộn sẽ nhận cả những kế hoạch đã chết, theo thứ tự, và consumer ngây thơ hành động theo cái cũ trước |
| **Đúng MỘT publisher** | vĩnh viễn | Latched + hai writer ⇒ reader vào muộn nhận **hai** "kế hoạch hiện hành" mâu thuẫn, không có thứ tự giữa chúng. Đây là lý do P6.3–P6.6 **không** được đẻ `trajectory_generator_node` riêng: planner nạp waypoint cho navigator, navigator vẫn là publisher duy nhất |
| **Thu hồi** | `is_valid=false` + `points` rỗng | Phát khi task kết thúc / cancel / fault, khi rơi về carrot, và **một bản lúc node khởi động** — để phân biệt được "chưa có kế hoạch" với "navigator không chạy" |
| 🔴 **`start_time`** | **danh nghĩa** | Đồng hồ kế hoạch **đóng băng khi dây xích kẹp** (navigator dừng lấy mẫu khi máy bay tụt lại). Ai tính *"giờ này drone phải ở đâu"* bằng `start_time + time_from_start` sẽ phân kỳ khỏi setpoint thật, càng lâu càng lệch — và nếu đó là `uav_safety` thì nó kích failsafe nhầm đúng lúc máy bay đang vật lộn |
| **Không nằm trên đường điều khiển** | | Thứ lái máy bay là `ControlCommand` trên `/control/command_selected`. `Trajectory3D` để quan sát / log / mission đọc. Vòng dây xích cần **pose đo được** nên nó phải ở cùng executor với bộ phát setpoint |

✅ **Hai trong ba khoảng trống đã đóng 2026-08-19 (Đ1)** — `plan_state` + `reason` + `sequence` thêm
vào cả `Path3D`/`Trajectory3D`, xem §2.6. ✅ **Nửa còn lại đóng 2026-08-20:** navigator (publisher duy
nhất của `Trajectory3D`) nay SET đủ trường — `toMessage()` phát `VALID` + `sequence` tăng mỗi kế
hoạch mới; `publishNoTrajectory(state, reason)` phân biệt `NO_GOAL` (khởi động / goal kết thúc /
`use_trajectory=false`) với `FAILED` (build hỏng, `reason` = lý do build), và **giữ nguyên `sequence`**
— bản thu hồi nói về số phận của kế hoạch hiện hành, không phải kế hoạch mới. Test khóa:
`TheGotoPlanIsPublishedAndThenRetired` + `AGoalTooFarToPlanFallsBackToTheCarrotAndSaysWhy`.

⏳ **Một khoảng trống còn lại:**
1. `points[].yaw` là yaw **bám hướng đi**; đoạn bàn giao sang yaw đích ở cuối chuyến chưa được ghi lại vào lưới đã publish ⇒ dự đoán hướng mũi từ lưới sẽ sai trong những giây cuối.

---

### 2.15 `AvoidanceAdvice` — lời khuyên có trạng thái, KHÔNG phải lệnh (P6.4, duyệt 2026-08-19)

**Vai:** `local_planner_node` phát trên `/uav/<id>/planning/avoidance`; **navigator vẫn là nơi duy nhất
phát setpoint** (quyết định kiến trúc P6.4 — `plan/P6-navigation.md` §2g).
Msg này **khuyên**, không ra lệnh — cùng nếp `localization_health`: *đưa bằng chứng, không tự chữa*.

| Quyết định | Vì sao |
|---|---|
| **Có `advice` (CLEAR/ESCAPE/HOLD) + `reason` ngay từ ngày đầu** | 🔑 Đây là chỗ **không lặp lại nợ của `Path3D`/`Trajectory3D`**, vốn thiếu trường trạng thái nên lý do thất bại phải đi nhờ `planner_name` và `is_valid` gánh ba nghĩa (§2.6, §2.14). Rẻ nhất là làm đúng từ msg đầu tiên |
| `reason` **luôn** điền, kể cả khi `CLEAR` | Chuỗi rỗng không phân biệt được *"không có gì để nói"* với *"quên điền"* |
| `escape_point` = **NaN** trừ khi `ADVICE_ESCAPE` | `(0,0,0)` trong `odom` là **chỗ cất cánh** — một toạ độ trông hợp lệ. Cùng khuyết tật #11 của lõi quỹ đạo (§2.14) |
| Có **cả** `clearance_m` **và** `checked_horizon_m` | Một con số hở mà không kèm *"nhìn xa tới đâu"* thì **không đọc được**: hở 5 m trên tầm nhìn 2 m không nói lên điều gì. Cùng bài học *"tỉ lệ % không kèm cửa sổ thời gian thì không phải số đo"* |
| `map_fresh` + `map_age_sec`, `inf` khi **chưa từng** nhận | Bản đồ **rỗng** và bản đồ **chết** trông y hệt nhau nếu chỉ đọc nội dung — độ tươi phải là kênh riêng (giáo lý `lastInputAge()` của P6.3) |
| `frame_id` **luôn** `"odom"` | Khớp §2.11 và `GotoPose.frame_id` (§2.8) |

🔴 **Ràng buộc cho phía tiêu thụ:** **im lặng KHÔNG được đọc thành `CLEAR`.** Navigator phải bắt bằng
timeout riêng trên topic này — kiến trúc advisor đẻ ra chế độ hỏng *"advisor chết mà navigator vẫn bay"*
mà kiến trúc cũ không có.

---

### 2.16 Hợp đồng arbitration — `uav_control_authority` (P7, chốt 2026-08-20)

**Vai:** một trọng tài duy nhất chọn **1 trong 4** nguồn lệnh cho `/control/command_selected`, áp thứ
tự ưu tiên đã mã hoá trong `ControlAuthority` (§2.2) bằng số học thuần — không thêm message/trường
mới, chỉ định nghĩa **luật vận hành** trên `ControlCommand`/`ControlAuthority` đã có. Chi tiết thiết kế
+ lý lẽ đầy đủ → `plan/P7-control-authority.md`; số đo/mốc
đóng từng bước → `.claude/changelog.md` (grep `"P7."`).

#### a) Bốn kênh vào — một topic = một mức ưu tiên

| Topic | Priority | QoS | Ai phát |
|---|---|---|---|
| `/uav/<id>/control/cmd_safety` | 4 SAFETY | Reliable · KeepLast(10) · Volatile | `uav_safety` (P8) — chưa có publisher |
| `/uav/<id>/control/cmd_operator` | 3 OPERATOR | như trên | teleop/GCS (P11) — chưa có publisher |
| `/uav/<id>/control/cmd_mission` | 2 MISSION | như trên | `navigator_action_server_node` |
| `/uav/<id>/control/cmd_test` | 1 TEST | như trên | probe bay hồi quy — chỉ sim (`enable_test_source`) |

#### b) Ba topic ra

| Topic | Kiểu | QoS | Ghi chú |
|---|---|---|---|
| `/uav/<id>/control/command_selected` | `ControlCommand` | Reliable · KeepLast(10) · Volatile | 🔴 **SINGLE WRITER vĩnh viễn** — trước P7.3 navigator phát thẳng vào đây (quyết định P6 #1); từ P7.3 chỉ trọng tài được ghi |
| `/uav/<id>/control/authority` | `ControlAuthority` | Reliable · KeepLast(1) · TransientLocal | "ai đang cầm lái" — phát **khi đổi** + nhịp tim **2 Hz** (hằng hợp đồng, không phải tham số) |
| `/uav/<id>/diagnostics/control_authority` | `diagnostic_msgs/DiagnosticArray` | Reliable · KeepLast(10) | nhịp **1 Hz** (hằng hợp đồng); mọi số đo phụ (drop, publisher trùng, release) đi đường này, không lẫn vào `authority`. **Y15 (P8)**: status "arbitration" mang 6 key/value: `latch_active`, `latch_level`, `latch_effective`, `latch_ever_alive`, `latch_age_sec`, `inhibit_active`; khi `inhibit_active=true` thì level cả status này lên **ERROR** mỗi chu kỳ (R26 — bù cho im lặng vô hạn có chủ đích của INHIBIT). **N-c (P10.8a)**: status riêng "control_authority: clock regressions" mang key `clock_regressions` (đếm dồn, không reset) — level lên **ERROR** khi > 0, không thoái lui im lặng. **C4 (P10.8b)**: `latch_age_sec` đọc **NaN** (không phải số âm) khi đồng hồ lùi trong lúc latch đang giữ — xem §2.16i |
| `/uav/<id>/control/clear_safety_latch` | `uav_interfaces/srv/ClearFault` (tái dùng, không thêm interface mới) | mặc định service | **P8 R2**: đường DUY NHẤT gỡ latch SAFETY. Nằm trong `io_group_` (R24). Người gọi hợp lệ duy nhất là `uav_safety` sau khi `ClearFault` của chính nó đã chấp nhận — trọng tài không tự phán điều kiện, chỉ thi hành; mọi lần gọi log ERROR |

#### c) Rơ-le thuần — KHÔNG BAO GIỜ tự sáng tác setpoint

| Luật | Vì sao |
|---|---|
| Trọng tài không bao giờ tự phát `ControlCommand` của riêng nó | nó chỉ cho qua / chặn / đóng dấu lại `source` — không kẹp, không mượt, không nội suy hình học |
| Mọi nguồn chết (`active_source = SOURCE_NONE`) ⇒ **không publish gì** trên `command_selected` | im lặng là **CÓ CHỦ ĐÍCH**: `px4_command_gateway_node` tự đệm `command_timeout_sec` rồi ngừng phát → PX4 tự failsafe. Trọng tài tự sinh hover thay vào đó sẽ giữ offboard sống khi KHÔNG AI lái — chế độ hỏng tệ nhất có thể thiết kế ra |
| 🔴 **"Một nguồn CÒN SỐNG" chỉ được suy từ message ĐÃ QUA bộ lọc (d), không phải mọi message đến** (B2, chốt 2026-08-20) | một kênh phát toàn lệnh dị dạng ở tần số cao **không được** đọc thành "kênh này còn sống" — nếu không, nó có thể cướp quyền bằng ưu tiên, giữ một latch hiệu lực vĩnh viễn, hoặc giữ quyền vô hạn dù nội dung không dùng được. Đây là chốt chặn thứ hai sau (d), không phải phần thừa |

#### d) Bộ lọc hợp lệ trước khi cho qua

| Kiểm | Xử lý | Vì sao |
|---|---|---|
| Trường của `control_mode` đang dùng không hữu hạn (NaN/inf) | DROP | setpoint NaN tới PX4 = "trục này thả tự do", không ai đỡ |
| `header.frame_id != odom_frame` | DROP | sai frame ở đường lái trực tiếp nguy hơn im lặng |
| `control_mode` ngoài {POSITION, VELOCITY, ACCELERATION} | DROP | gateway không hiện thực các mode còn lại |
| Tuổi `header.stamp` quá `max_command_age_sec` (khi `check_command_age=true`) | DROP | chặn bơm dữ liệu cũ; liveness tính theo **thời điểm ĐẾN**, tuổi này là kiểm riêng trên **nội dung** — cổng DUY NHẤT chặn nội dung cũ trên toàn tuyến (gateway hạ nguồn chỉ đo thời điểm đến, không đọc `header.stamp`). **C4 (P10.8b, §2.16i):** đồng hồ lùi (`now < stamp`) đọc thành tuổi **+inf** ⇒ luôn DROP, không phải "chắc chắn tươi" |
| **Bốn kiểm trên áp dụng ĐỒNG NHẤT trên cả 4 kênh, kể cả SAFETY** | | Q2 đã chốt: setpoint dị dạng không ai đỡ tốt hơn PX4 failsafe, kể cả khi nó tự xưng là lệnh cứu mạng |
| `msg.source` ≠ mức của topic đang phát | **KHÔNG drop** — đóng dấu lại (`source` = mức của topic) | quyền do **TOPIC** quyết, không do nhãn tự khai; kênh SAFETY không được im chỉ vì một nhãn sai |

#### e) Hysteresis — lên ngay, xuống có dwell

| Chiều | Luật | Vì sao |
|---|---|---|
| **LÊN** (nguồn ưu tiên cao hơn xuất hiện) | chiếm quyền **ngay ở message đầu tiên**, không chờ | làm chậm SAFETY dù một tick là phản bội thứ tự ưu tiên (§2.2) |
| **XUỐNG** (nhả cho nguồn thấp hơn) | chỉ khi nguồn đang giữ **im lặng liên tục** `release_dwell_sec` | chống giật khi một nguồn rung nhẹ nhịp phát |

#### f) Latch (`SetControlAuthority`) — AS-BUILT

| Luật | Vì sao |
|---|---|
| Sàn latch **chỉ hiệu lực khi bên giữ đã publish ít nhất một lần** | latch cấp xong mà chưa ai phát **không loại bất kỳ nguồn nào** — sửa chế độ hỏng "operator giành quyền rồi cầm nhầm cần" từng khoá một mission đang bay tốt |
| 🔴 **Ngoại lệ SAFETY (P8 R2, chốt 2026-08-20)**: mức SAFETY hiệu lực **NGAY**, không cần bằng chứng sống, và **không GRACE-expiry** khi chưa từng sống | policy-1 (P8): SAFETY không được phép phát setpoint trên pose hỏng ⇒ không thể "chứng minh đang lái" như 3 mức kia — bắt nó chứng minh sẽ làm INHIBIT (im lặng có chủ đích) bất khả thi |
| RELEASE (`requested_source = SOURCE_NONE`) **bị TỪ CHỐI** khi latch đang giữ ở mức SAFETY | chỉ service **`clear_safety_latch`** (kiểu `ClearFault`, P8) mới gỡ được latch SAFETY; mọi yêu cầu release — thành công hay bị từ chối — đều là sự kiện đáng chú ý (log ERROR + đếm trong diagnostics) |
| Ba đường thu hồi bắt buộc cho OPERATOR/MISSION/TEST (không đổi bởi P8): (1) release tường minh, (2) latch cấp mà bên giữ **chưa từng publish** quá `latch_grace_sec`, (3) bên giữ **đã từng sống rồi im** quá `latch_timeout_sec` | thiếu một đường là latch trở thành khoá chết máy bay |
| SAFETY luôn thắng một latch OPERATOR bằng publish | latch dựng **SÀN**, không dựng **TRẦN** — nguồn cao hơn mức latch không bao giờ bị chặn |

**Mức nào cần bằng chứng sống, mức nào không (P8 R2):**

| Mức latch | Cần `latch_ever_alive_` để hiệu lực? | GRACE-expiry khi chưa từng sống? | Vì sao |
|---|---|---|---|
| SAFETY | **Không** — hiệu lực ngay lúc cấp | **Không** — persist vô hạn cho tới `clear_safety_latch` / PX4 failsafe / RC (P11) | INHIBIT là im lặng có chủ đích (policy-1), không phải lỗi cần tự huỷ |
| OPERATOR / MISSION / TEST | **Có** — `latchEffective()` false tới khi có 1 message hợp lệ | **Có** — tự huỷ sau `latch_grace_sec` nếu chưa từng sống | Y4 giữ nguyên: latch "giành quyền rồi cầm nhầm cần" không được khoá một mission đang bay tốt |

🔴 **R5 (chốt 2026-08-21, sửa lại kết luận trên):** một latch SAFETY **đã từng sống rồi mới im**
KHÔNG tự huỷ sau `latch_timeout_sec` nữa — SAFETY miễn trừ khỏi CẢ HAI nhánh (GRACE lẫn TIMEOUT), không
chỉ nhánh "chưa từng sống". Lý do: HOLD (P8.5) khiến SAFETY thực sự publish rồi chủ động ngừng khi
escalate sang INHIBIT — im lặng có chủ đích đó không được đọc nhầm thành "bên giữ đã chết" (phát hiện
thật ở G-S1 #8: latch tự thu hồi ở +5s sau khi HOLD ngừng stream, đúng lúc `command_selected` có thể
quay lại chọn MISSION dù safety vẫn tưởng mình đang giữ quyền). `latch_age_sec` (Y15) tăng vô hạn trên
`latch_level=SAFETY` là tín hiệu giám sát bù cho trường hợp safety node chết mà vẫn giữ latch.

#### g) Kênh toàn-rác không giữ được quyền (Y5 — cơ chế cấu trúc từ B2, chốt 2026-08-20)

🔵 **Sửa 2026-08-20:** bản đầu (P7.1b) hiện thực (g) bằng một bộ đếm riêng (`demote_after_bad_ticks`
— hạ cấp sau N lệnh dị dạng liên tiếp). Review B2 chỉ ra: một khi (c)/(e)/(f) đều chỉ đọc mốc thời
gian của message ĐÃ QUA bộ lọc (d) — đúng luật vừa thêm ở mục (c) — thì một kênh toàn rác **không
bao giờ được coi là còn sống** để bắt đầu, nên **không cần bộ đếm riêng nữa**: bảo chứng dưới đây vẫn
đúng y nguyên, chỉ đổi từ "một cơ chế đếm cần giữ đồng bộ với luật khác" thành "hệ quả tự nhiên của
luật (c)". Bộ đếm đã bị GỠ khỏi tham số/API.

| Luật | Vì sao |
|---|---|
| Kênh **đang giữ quyền** mà nội dung liên tục bị bộ lọc (d) chặn ⇒ tự rơi ra sau `release_dwell_sec`, giống hệt như im lặng thật | rác không cập nhật mốc "còn sống" (c) nên hysteresis (e) đối xử với nó y hệt một kênh đã ngừng phát — không cần phân biệt "im lặng" với "phát mà không dùng được" bằng hai cơ chế riêng |
| Một kênh 100% rác **không bao giờ thắng được ai** bằng ưu tiên, kể cả kênh SAFETY toàn rác so với MISSION hợp lệ | (c): nó chưa từng được coi là còn sống, nên không lọt vào danh sách ứng viên của luật ưu tiên (§2.2) |
| Một lệnh hợp lệ lập tức khôi phục kênh về đúng luật ưu tiên bình thường (e), không có "phạt vĩnh viễn" | một lỗi thoáng qua không được biến thành mất quyền vĩnh viễn — đây là hệ quả tự nhiên của (e)/(c), không phải một nhánh xử lý riêng |

#### h) Quy tắc ghép cặp tham số (validate lúc khởi động — sai thì từ chối chạy)

| Ràng buộc | Vì sao |
|---|---|
| `source_timeout_sec ≤ 0,6 × downstream_command_timeout_sec` | `downstream_command_timeout_sec` là **bản sao** của `command_timeout_sec` bên gateway (0,5 s) — trọng tài không thấy được tham số thật của gateway |
| `release_dwell_sec ≥ source_timeout_sec` | ngược lại, kênh vừa bị nhả và `SOURCE_NONE` có thể rung qua lại ở đúng biên chung |
| `release_dwell_sec + 2 × chu_kỳ_giám_sát + 1/stream_hz ≤ 0,8 × downstream_command_timeout_sec` | tổng thời gian bàn giao chiều XUỐNG (worst-case) không được ăn hết đệm của gateway — nếu vượt, gateway ngừng phát *giữa* một cú bàn giao hợp lệ |
| `odom_frame` của trọng tài **phải trùng** `odom_frame` của navigator | hai tham số độc lập ở hai package, **không có kiểm runtime nào ràng chúng lại** — mục checklist thủ công khi đổi một trong hai |
| `check_command_age` (bật/tắt kiểm tuổi) là **công tắc RIÊNG** | cấm lấy `max_command_age_sec = 0` ngầm định làm công tắc tắt giới hạn — một giới hạn bị vô hiệu hoá âm thầm nguy hơn không có giới hạn |

#### i) Đồng hồ SIM lùi — N-c (P10.8a) + C4 (P10.8b), chốt 2026-08-23, Q-P10-7

🔴 **C4 (review đóng phase P10.8b):** N-c (bản đầu) chỉ route được **4/6** phép trừ thời gian thật sự
tồn tại trong `authority_arbiter` — bỏ sót guard tuổi **NỘI DUNG** lệnh (mục (d) ở trên,
`check_command_age`/`max_command_age_sec`) và `latchAgeSec()` (chỉ báo Y15, mục (f) trên). Trước C4,
một đồng hồ lùi khiến guard tuổi nội dung thành **no-op im lặng** (lệnh cũ lọt qua như mới nhất, không
đếm `clock_regressions_`) — nguy hiểm hơn các trường hợp N-c đã đóng vì đây là **cổng DUY NHẤT** chặn
nội dung cũ trên toàn tuyến (gateway hạ nguồn chỉ đo thời điểm đến, không đọc `header.stamp`). Đã sửa
cùng cơ chế, cùng hàm.

| Luật | Vì sao |
|---|---|
| Mọi phép trừ `now − stamp` trong `authority_arbiter` — liveness, tuổi thô, 2 nhánh hết hạn latch, dwell release (N-c), **PLUS guard tuổi nội dung lệnh và `latchAgeSec()` (C4)** — đi qua **một** hàm `ageSecOrInf()`: `now < stamp` (đồng hồ lùi, vd replay/loop bag, hoặc publisher quên `use_sim_time`) ⇒ tuổi **không đo được** (`+inf`, cùng giá trị đã dùng cho "chưa từng nhận") | tuổi âm không bao giờ được đọc thành "chắc chắn còn sống"/"chắc chắn còn mới" — R30, cùng phán quyết đã dùng cho `staleness_board` của `uav_observability` |
| Mỗi lần phát hiện `now < stamp` đếm vào `clock_regressions_` (đếm dồn, expose diagnostics — bảng (b) ở trên) | không đo được ≠ OK; một đồng hồ lùi phải **thấy được**, không được thoái lui im lặng |
| Hệ quả: kênh không LIVE ⇒ rơi về `NONE` ⇒ im lặng trên `command_selected` ⇒ gateway hết hạn ⇒ PX4 failsafe | đây là "LÀM ÍT ĐI", không phải chế độ hỏng mới — cùng đường an toàn mục (c) đã mô tả cho mọi trường hợp mọi nguồn chết |
| 🔴 **C4: guard tuổi nội dung lệnh** — `now < header.stamp` ⇒ tuổi `+inf` ⇒ luôn `> max_command_age_sec` ⇒ **DROP (STALE)** | trước C4: `now − stamp` âm không bao giờ `> max_command_age_sec` ⇒ lệnh không đo được tuổi lọt qua như tươi nhất, không để lại dấu vết nào (không đếm `clock_regressions_`) |
| 🔴 **C4: `latchAgeSec()`** — `now < latch_granted_at` ⇒ trả **NaN** (không phải `+inf`, không phải số âm) | đây là số **hiển thị chẩn đoán** (Y15, không so sánh ngưỡng trong code), nên "không đo được" phải đọc **rõ ràng** là NaN (R30) — trả `+inf` sẽ lặng lẽ sắp xếp thành "tuổi lớn nhất có thể", trả số âm (hành vi trước C4) đọc nhầm thành "rất mới", ngược hẳn mục đích của chỉ báo "latch_age_sec tăng vô hạn = safety node chết" |
| Latch OPERATOR/MISSION/TEST **bị nhả** khi đồng hồ lùi (`GRACE_EXPIRED` nếu chưa từng sống, `TIMED_OUT` nếu đã từng sống) — **CHẤP NHẬN** | tuổi latch cũng không đo được được nữa; giữ latch hiệu lực trên một phép đo không tin được nguy hơn nhả nó |
| 🔴 **Latch SAFETY tuyệt đối KHÔNG bị ảnh hưởng** | hai nhánh `latch_level_ == kSourceSafety` trong `processLatchExpiry()` return **TRƯỚC** khi chạm `ageSecOrInf()` — R5 (§2.16f) nguyên vẹn, không đổi bởi N-c lẫn C4 |
| `onTick()`'s dwell-release trên `active_source_` (khác cơ chế latch) đối xử với SAFETY giống mọi kênh — vốn đã vậy TỪ TRƯỚC N-c | một rewind có thể khiến `active_source_` tạm về `NONE` một tick dù floor SAFETY còn nguyên; đó là im lặng an toàn (không kênh thấp hơn lọt qua được floor), không phải mất quyền — SAFETY tự chiếm lại ngay ở message kế tiếp (lên-ngay, không dwell) |

### 2.17 Quy ước twist tri-state trên `odometry_fused` (chốt 2026-08-21, P8)

`nav_msgs/Odometry.twist.covariance[0]` (phương sai `v_x`, hàng-chính 6×6) mang **ba** trạng thái, không phải hai. Đây là hợp đồng của **mọi** `nav_msgs/Odometry` do `uav_localization` phát — trước hết là `/uav/<id>/state/odometry_fused`.

| `twist.covariance[0]` | Nghĩa | Consumer phải làm gì |
|---|---|---|
| **`-1`** | **KHÔNG có twist** — nguồn không đo vận tốc | Không đọc `twist.twist` |
| **`0`** | **Có twist, sai số CHƯA KHAI** | Dùng được vận tốc, nhưng **cấm** coi là chính xác; không cân nó với nguồn khác |
| **`> 0`** | **Đã khai** — giá trị là **phương sai**, `1-sigma = sqrt(...)` | Dùng bình thường; nhớ lấy căn |
| NaN · `±inf` · âm khác `-1` | không đọc được | Xử như **KHÔNG có twist**; cấm "sửa" thành một con số |

🔴 **Consumer PHẢI dùng `uav_localization/twist_reading.hpp`** (`readTwistTrust`, `statedTwistStddev`) — **cấm đọc thô rồi coi `0` là hoàn hảo**. Mặc định của ROS là covariance toàn `0`, nên một nguồn *quên khai* trông y hệt một nguồn *đo được sai số bằng 0*: đó là cùng lớp lỗi đã trả giá ở `px4_external_odometry_node` (§2.12, quy tắc `-1`). Header là **header-only, ROS-free** và đã cài sẵn theo `uav_localization`; consumer tự `#include` khi cần, package này không ép ai phụ thuộc.

**Vì sao GIỮ tri-state thay vì đổi hợp đồng** (chủ dự án chốt 2026-08-21): bắt mọi adapter khai `twist_stddev` thật sẽ **xoá mất phân biệt "chưa khai" và "đã khai"** — lúc đó một con số bịa ra sẽ đi vào chỗ hiện đang trung thực nói *"tôi chưa biết"*. Ba trạng thái là **sự thật**, hai trạng thái là tiện lợi. Giá phải trả là consumer buộc phải qua helper, và đó là cái giá rẻ hơn.

Ai đang ghi giá trị này: `source_channel.cpp` (`!twist_known → -1` · `isUsable(stddev) → stddev²` · còn lại giữ `0`); ai đang đọc: `localization_mux_node.cpp:168` (`covariance[0] >= 0.0`). Test `test_twist_reading.cpp` ghim đúng hai dòng đó, nên helper lệch khỏi code thật là test đỏ.

🔴 **Vì sao mux được phép đọc thô mà không mâu thuẫn dòng in đậm ở trên (làm rõ 2026-08-24):** phép đọc thô `>= 0.0` **chỉ khác helper ở hàng 4** — nó coi `+inf` là "đã khai" trong khi hợp đồng bảo phải xử như *không có twist*. Nay lỗ đó bị bịt **ở phía GHI**: nhánh twist của `source_channel.cpp` dùng chung `isUsable()` (`isfinite && >= 0`) với pose/heading, nên **publisher của ta không bao giờ phát được covariance non-finite** — `+inf`/`NaN` rơi về `0` = *"có twist, sai số chưa khai"* (giá trị vận tốc vẫn tốt, chỉ độ bất định là chưa biết). ⚠️ **Trước bản vá đây là lỗ THẬT, không phải giả thuyết:** test đỏ-trước cho thấy `covariance[0]` ra dây đúng bằng `inf` (`test_source_channel.cpp` · `ANonFiniteTwistStddevIsPublishedAsUnstatedNotAsInfinity`, 2 nhánh `+inf` và `NaN`).
→ **Ràng buộc vẫn giữ nguyên hiệu lực cho nguồn NGOÀI** (VIO/mocap của bên thứ ba ở P11, bag replay, drone khác): những nguồn đó không đi qua `source_channel.cpp` nên hàng 4 vẫn sống, và consumer đọc thẳng `nav_msgs/Odometry` từ nguồn lạ **PHẢI** qua helper.

### 2.18 Hợp đồng giám sát an toàn — `uav_safety` (P8, chốt 2026-08-21)

Lý lẽ thiết kế đầy đủ → `.claude/plan/P8-safety.md`; trạng thái/bẫy → `docs/package-status.md` §10. Ở đây chỉ ghi phần **consumer/operator phải biết**:

| Giao diện | Hợp đồng |
|---|---|
| `/uav/<id>/safety/state` (`SafetyState`) | Reliable/KeepLast(1)/**TransientLocal** — subscriber đến muộn nhận ngay trạng thái mới nhất. 5 Hz + on-change. `recommended_action` ∈ {`none`, `report`, `hold`, `inhibit`} khi enforcement bật; khi `safety_enforcement:=false` hai giá trị dry-run thay chỗ hold/inhibit: `would_hold (dry-run, enforcement_enabled=false)` · `would_inhibit (dry-run, enforcement_enabled=false)` — consumer parse action phải so **tiền tố**, không so chuỗi đầy đủ *(N-review-2, 2026-08-22)* |
| `/uav/<id>/safety/violations` (`DiagnosticArray`) | Reliable/KeepLast(20), **chỉ phát khi có cạnh** (vào/ra violation); mỗi status = mã + measured/threshold/action. Im lặng = không có cạnh mới, KHÔNG có nghĩa "sạch" — trạng thái tổng đọc ở `safety/state` |
| `/uav/<id>/control/cmd_safety` (`ControlCommand`) | Chỉ phát khi **HOLDING** (policy 3): `MODE_POSITION`, frame `odom`, `source=SOURCE_SAFETY`, pose **đóng băng** (không nội suy/trôi), **restamp mỗi tick** @ `hold_stream_hz` 20. INHIBIT **không phát gì** — cắt qua latch trọng tài. `enforcement_enabled=false` ⇒ publisher **không tồn tại** (đảm bảo cấu trúc, kiểm bằng `ros2 topic info`) |
| `/uav/<id>/safety/clear_fault` (`ClearFault`) | 🔴 **Caller PHẢI POLL** — `clear_stability_sec` (3,0 s) tích luỹ **qua các lần gọi**, không qua đồng hồ nội bộ; gọi 1 lần sau khi chờ đủ lâu vẫn bị từ chối (`stable_for=0.000`). Từ chối luôn kèm mã + số đo + ngưỡng trong `remaining_faults`. Ba đường từ chối bắt buộc: không-đo-được (R27-1) · nguyên nhân còn sống · bước nhảy bàn giao > `handover_jump_limit_m` 0,80 |
| Latch SAFETY (trọng tài) | **R5:** một khi đã cấp, KHÔNG tự hết hạn qua bất kỳ nhánh nào (grace lẫn timeout) — đường gỡ DUY NHẤT là `clear_safety_latch` (chỉ `uav_safety` gọi, sau khi `ClearFault` của nó được chấp nhận). Mức OPERATOR/MISSION/TEST giữ nguyên Y4 + `latch_timeout_sec`. Giám sát holder-chết qua Y15 `latch_age_sec` (§2.16f) |
| Taxonomy hành động (đã ký, đúng 3) | `REPORT` (violation + event) · `HOLD` (chiếm quyền, pose đóng băng, latch) · `INHIBIT` (latch, im lặng có chủ đích → PX4 failsafe + pilot). **KHÔNG land/RTL/climb** — safety không bao giờ gọi `set_mode`/arm/disarm/action navigator |
| Hệ quả cho navigator | Safety KHÔNG cancel goal đang chạy — navigator giữ goal suốt thời gian mất quyền và goal **tự chảy lại** sau ClearFault (đo thật G-S3-HOLD: goal Goto gốc tiếp tục, goal mới gửi trong lúc đó bị REJECT `busy`) |
| **Y4 (chốt 2026-08-21):** 5 boolean tóm tắt của `SafetyState` | `localization_ok` = không LOCALIZATION_INVALID/STALE/JUMP · `offboard_link_ok` = không OFFBOARD_UNHEALTHY · `battery_ok` = không BATTERY_WARN/CRITICAL/PX4_NO_ACTION · `obstacle_clear` = không OBSTACLE_TOO_CLOSE · `command_fresh` = **độ tươi THẬT của `command_selected`** (`command_selected_age_sec ≤ selected_stale_sec` 0,5) — **KHÔNG còn là proxy `!BLIND_COMMAND`** (một lệnh có thể tươi mà BLIND_COMMAND vẫn latch vì lý do khác, và ngược lại) |
| **Biên phát hiện FRAME_MISMATCH** *(N-review-1, 2026-08-22)* | Đường "drops tăng bền" chỉ bắt được kênh sai frame **một phần** (lệnh sai frame xen lệnh hợp lệ). Kênh sai frame **100%** thì mọi msg bị trọng tài DROP ⇒ topic hạ nguồn im ⇒ `command_fresh` sập ở 0,5 s **trước** `frame_mismatch_grace_sec` 2,0 s — FRAME_MISMATCH có thể **không bao giờ latch**; kết cục vẫn an toàn qua đường offboard-mất → PX4 failsafe. Đừng đọc hợp đồng này thành "sai frame thì chắc chắn latch FRAME_MISMATCH". Cờ "drops tăng" hết hạn sau 2× `arbiter_diagnostics_period_copy_sec` khi diagnostics trọng tài chết (R32) |
| **B5 (chốt 2026-08-21):** vị ngữ armed cho 4 mã LATCHING | `BLIND_COMMAND`/`BATTERY_PX4_NO_ACTION`/`OBSTACLE_TOO_CLOSE`(HOLD)/`FRAME_MISMATCH` chỉ được phép **LATCH** (chiếm quyền) khi `armed && flight_mode==OFFBOARD && command_fresh` — đo từ `VehicleState`/`command_selected` thật, tuổi kiểm theo B2. **Fail-OPEN**: `vehicle_state` cũ/không đo được KHÔNG được coi là "đã xác nhận đậu bãi" để tắt bảo vệ — chỉ xác nhận DƯƠNG TÍNH disarmed/không-OFFBOARD mới tắt được. 8 mã REPORT-only KHÔNG bị gate — vẫn báo cáo dù đang đậu bãi |
| 🔴 **Ràng buộc ngược từ `uav_observability` (P10.9b, chốt 2026-08-24)** | `DiagnosticStatus.name` của từng child trên `/uav/<id>/diagnostics/safety`, và KeyValue `action` của chúng, nay là **GIAO DIỆN ĐỐI NGOẠI**: `diagnostics_node` chấm go/no-go theo từng child và phủ quyết miễn-trừ theo `action` (contract §2.20's NW1/NW2). Đổi tên một child hoặc đổi từ vựng `action` = đổi hợp đồng, phải sửa `uav_observability/config/preflight_waivers.yaml` cùng lượt — nếu không, `waiver_unmatched` trên `/state/system_health` sẽ khác 0 và test `ShippedWaiverYamlMatchesRealSafetyChildNames` (`test_diagnostics_node.cpp`) sẽ đỏ. **Thêm mã vi phạm mới ⇒ mặc định ĐƯỢC TÍNH** ở P10 (fail-loud), không cần sửa gì ở đây để giữ an toàn — chỉ cần sửa nếu mã đó cần một hàng miễn-trừ theo pha. `uav_safety` **không** phải phát thêm trường nào cho P10 |

### 2.19 Hợp đồng điều phối mission — `uav_mission` (P9, chốt 2026-08-22)

Lý lẽ thiết kế đầy đủ → `.claude/plan/P9-mission.md` §4. Ở đây chỉ ghi phần **consumer/operator phải biết**:

| Giao diện | Hợp đồng |
|---|---|
| `/uav/<id>/mission/status` (`MissionStatus`) | Reliable/KeepLast(1)/**TransientLocal** — subscriber đến muộn nhận ngay trạng thái mới nhất. 2 Hz + on-change |
| `/uav/<id>/mission/events` (`MissionEvent`) | Reliable/KeepLast(50)/**Volatile** — **chỉ phát khi có cạnh** (vào/ra một trạng thái), KHÔNG phát nhịp đều. Volatile (không TransientLocal như `status`) vì đây là **luồng cạnh lịch sử**, không phải trạng thái hiện tại: subscriber đến muộn cần *tất cả* cạnh kể từ lúc kết nối để dựng lại lịch sử đúng thứ tự, không phải chỉ cạnh gần nhất — TransientLocal chỉ giữ được sample cuối nên sẽ làm subscriber muộn hiểu sai chuỗi sự kiện. Im lặng trên `events` **KHÔNG** có nghĩa "sạch" — trạng thái tổng đọc ở `mission/status`, cùng nếp với `safety/violations` (§2.18) |
| Số lượng goal `ExecuteMission` | **Tối đa MỘT goal tồn tại tại một thời điểm** (`NavGoalBroker`, P9-mission.md §1). Goal thứ 2 gửi khi đang có goal chạy ⇒ **REJECT** ngay, không hàng đợi, không huỷ goal cũ để nhường goal mới |
| `mission_id` trong goal `ExecuteMission` | **Rỗng = đang chấp nhận** (đại diện goal hiện đang chạy, không phải "goal trống"); một `mission_id` **khác** goal đang chạy ⇒ **REJECT** — không có cơ chế đổi mission giữa chừng, phải abort/complete goal cũ trước |
| 🔴 `uav_mission` KHÔNG subscribe `/uav/<id>/planning/trajectory` | Nợ kiến trúc đang mở: chuyển giao yaw giữa hai trajectory kế tiếp (P6.2) chưa có lời giải chốt. Mission chỉ nói chuyện với navigator qua 7 action (Takeoff/Land/GotoPose/HoldPosition/FollowPath/TrackTarget/Recover) + service Load/Pause/Resume/Abort — **không** đọc trực tiếp trajectory để tự quyết logic, tránh kế thừa nợ đó vào một tầng cao hơn trước khi nó được giải |
| `MissionStatus.last_result_code` / `last_reason` / `goal_id` (thuần bổ sung P9) | `last_result_code` dùng chung bảng hằng `ResultCode.msg`; **luôn điền** khi `state ∈ {PAUSED, ABORTED}`, giữ nguyên giá trị cũ khi state khác (không reset về 0 mỗi tick). `goal_id` là UUID hex của goal `ExecuteMission` đang chạy; rỗng khi `state == IDLE` |
| `ResultCode.ABORTED_LOW_BATTERY=12` | Mã riêng cho nhánh pin WARN kết thúc sớm (P9-mission.md §2 mục 3: `GotoPose(home) → Land`). **Cấm mượn `ABORTED_SAFETY`** cho lý do pin — hai nguyên nhân khác nhau (mất quyền điều khiển vs. hết năng lượng an toàn), gộp chung sẽ làm log/diagnostics không phân biệt được nguyên nhân gốc |
| `follow_target`'s goal `TrackTarget.target_id` (G-M4.4, chốt 2026-08-23) | KHÔNG BAO GIỜ mang `-1` ra dây dù mission-level filter là `-1` ("bám bất kỳ") — mission neo `target_id` = track_id `TargetSeen` vừa thấy tại thời điểm dispatch, giữ nguyên suốt vòng đời goal đó, và neo LẠI id mới sau mỗi lần tái-bắt qua search; xem `uav_mission/README.md` mục 2 |
| `follow_target`'s search-attempts budget (G-M4.4b, chốt 2026-08-23) | Ngân sách theo **loss episode** (mỗi lần vào `search_when_lost`, không phải mỗi LOST_TARGET thô), sống sót qua nháy `TargetSeen` một khung, chỉ reset khi bám ổn định thật ≥ `track_progress_reset_sec`; cạn `search_attempts_max` (mặc định 2) ⇒ `Recover(CLIMB)` rồi `ABORTED_LOST_TARGET` — không bao giờ `ABORTED_TIMEOUT`; xem `uav_mission/README.md` mục 2 |
| `follow_target`'s `search_when_lost` phải có tái-bắt cuối (bug #12, chốt 2026-08-23) | Bay hết 2 điểm search KHÔNG đồng nghĩa "tìm thấy" — con cuối `target_visible_recheck` (`TargetSeen`) phải xác nhận thật; recheck fail (ngân sách còn) ⇒ retry, KHÔNG kết thúc mission; recheck/search_when_lost SUCCESS cũng KHÔNG BAO GIỜ đọc là mission SUCCEEDED (follow_target không có trạng thái "xong" hợp lệ) — xem `uav_mission/README.md` mục 2 |
| 🔴 **Giới hạn vận hành của filter `-1` ("bám bất kỳ") — ĐÃ CHỨNG MINH trên dây 2026-08-23** | Khi mục tiêu chỉ định biến mất mà vùng bay còn vật khác đủ điều kiện bám (kích thước/khoảng cách trong tầm tracker), mission với filter `-1` sẽ **lặng lẽ tái-bắt và bám vật đó** thay vì báo mất mục tiêu (đo thật: 58 s bám cột trụ ổn định, 0 cảnh báo, sau khi hộp mục tiêu bị gỡ). Đúng hợp đồng, KHÔNG phải lỗi — nhưng **bay thật với nhiệm vụ đòi bám đúng MỘT vật thì phải truyền `target_id` cụ thể**, và người thiết kế mission phải khảo sát vùng bay có vật bám-được nào khác không. Chuỗi lost/search/recover chỉ chạy trọn khi THẬT SỰ không còn gì bám được |
| 🔴 **B1 (2026-08-23, sửa lại sau chuyến chẩn đoán G-M1): authority-seize giữa **GotoHome/Recovering** ⇒ PAUSE; riêng **LANDING thì KHÔNG** | Ở `kFinishGotoHome`/`kRecovering`: cancel goal + PAUSED, nhớ đúng leg qua `paused_from_phase_`, `ResumeMission` redispatch đúng leg; `last_result_code` đọc `ABORTED_NO_AUTHORITY`. 🔴 **`kFinishLanding` được MIỄN guard này** (đo thật: PX4 LAND native ⇒ không ai stream `cmd_mission` ⇒ trọng tài báo `SOURCE_NONE` — trạng thái an toàn ĐÚNG DỰ KIẾN, không phải bị giành; và cancel-khi-LANDING bị navigator từ chối nên PAUSED lúc đó là trạng thái mồ côi). Seize thật giữa lúc hạ cánh: safety/PX4 xử ở tầng dưới, mission không pause — vị ngữ an toàn kèm tiền đề pha, cùng họ B5-P8. Chỉ `AbortMission` của operator được honor ở mọi pha |
| 🔴 **B2 (2026-08-23, review round 1): `MarkerSeen`/`TargetSeen` R32 — tuổi CẢ tin nhắn, không chỉ từng landmark** | `latestLandmark()`/`latestTarget()` nay đòi CẢ 2: (a) landmark/target riêng lẻ còn tươi (`time_since_seen_sec` cũ), VÀ (b) chính tin nhắn `/world/semantic_landmarks`/`/world/target_state` phải đến trong `2 × world_model_publish_period_copy_sec` (mặc định 0,1 s — copy `publish_rate_hz=10.0` của `uav_world_model`). world_model "chết" (ngừng publish) ⇒ landmark cuối cùng KHÔNG còn đọc là "thấy" dù `time_since_seen_sec` của nó không bao giờ tự tăng nữa |
| 🟡 **Y6 (2026-08-23): battery WARN trong lúc PAUSED phát `EVENT_PAUSED` bổ sung** | Logic bay KHÔNG đổi (PAUSED = không tự bay vì lý do pin). Khi battery rơi WARN/unknown NGAY TRONG lúc đã PAUSED (vì lý do khác), node phát thêm 1 `MissionEvent` (tái dùng `EVENT_PAUSED`, description phân biệt rõ nội dung — `uav_interfaces` không có type "cảnh báo" riêng) đúng 1 lần mỗi đợt liên tục (edge-latch, không lặp 10 Hz) |
| 🟢 **Tiêu chí chọn khi `marker_id=-1` ("bất kỳ") — công khai (green item, 2026-08-23)** | `latestLandmark(-1)` chọn landmark **MỚI thấy nhất** (`time_since_seen_sec` nhỏ nhất) trong số các landmark còn tươi, không còn là "khớp đầu tiên trong mảng" (thứ tự mảng không có hợp đồng) |

---

### 2.20 Hợp đồng quan sát & bằng chứng — `uav_observability` (P10, chốt 2026-08-23)

Lý lẽ thiết kế đầy đủ → `.claude/plan/P10-observability.md`. **P10 KHÔNG thêm msg/srv nào** — mọi giao diện dùng `diagnostic_msgs` chuẩn. Ở đây chỉ ghi phần **consumer/operator phải biết**:

| Giao diện | Hợp đồng |
|---|---|
| `/uav/<id>/state/system_health` (`DiagnosticStatus`) | Reliable/KeepLast(1)/**TransientLocal** — đọc một phát bằng script, subscriber muộn vẫn nhận. 1 Hz + on-change của `go_no_go`. KeyValue: `go_no_go` · `unknown_count` · `error_count` · `warn_count` · `stale_count` · `worst_item` · `clock_regressions` · `gate_mode` · `gate_mode_source` · `waived_count` · `waiver_unmatched` · `blocking` · `stamp_sec` · `wall_stamp_sec` (4 mới: P10.9b) |
| 🔴 `go_no_go` có **BA** giá trị, không phải hai | `GO` ⟺ mọi mục đang được tính đều OK **và** không mục nào UNKNOWN/STALE · có mục UNKNOWN ⇒ **`UNKNOWN`** (KHÔNG phải `NO_GO`, cũng KHÔNG phải `GO`) · còn lại ⇒ `NO_GO`. "Chưa đo được" phải phân biệt được với "đo được và xấu" (R30) — consumer nào coi `!= GO` là `NO_GO` sẽ mất đúng phân biệt đó. Sub-B **WARN** (nội dung một nguồn gộp, vd `diagnostics/safety` đang WARN) **cũng** ép `NO_GO` (sửa 2026-08-23, review đóng phase: trước đó lọt qua như GO, trái đúng dòng luật này) |
| 🔴 **`DiagnosticStatus` không có header — R32 phải tự thi hành bằng payload** (vá 2026-08-23) | `stamp_sec` (đồng hồ node, có thể là sim time) + `wall_stamp_sec` (UTC thật) là thời điểm **đánh giá lần cuối**, không phải thời điểm publish — `/clock` đứng (Gazebo pause/chết) ⇒ tick dừng ⇒ mẫu TransientLocal cuối nằm mãi, nhưng 2 trường này KHÔNG tự tăng nữa. Consumer **phải** từ chối mẫu khi tuổi **NGOÀI KHOẢNG** `0 ≤ (now − wall_stamp_sec) ≤ 2 × publish_period_sec` (đọc là "không đo được", không phải GO/NO_GO cuối cùng) — 🔴 **hai phía, không chỉ phía trên** (vá N8, P10-gate-debt review round 2, 2026-08-24): tuổi ÂM (companion lệch NTP/đồng hồ lùi) cũng phải bị từ chối, không riêng tuổi quá lớn; `preflight_light.py`/`o4_report.py` dùng chung `scripts/gate_freshness.py`'s `is_fresh()` để hai nơi không lệch luật. `wall_stamp_sec` cho phép phát hiện chính `/clock` đứng (sim time đứng nhưng wall time vẫn trôi ⇒ tự tố cáo). **Đọc đèn PHẢI qua `scripts/preflight_light.sh`** (tự kiểm tuổi này, cả hai phía) — **KHÔNG** dùng `ros2 topic echo --once` trần, có thể đọc mẫu latched của một node đã chết (P10.9b) |
| 🔴 **`gate_mode` (P10.9b, vá review lượt 2 2026-08-24: N1/N2/N3) nay là ĐẠI LƯỢNG ĐO ĐƯỢC, không còn param ghi tay** | Suy từ `/uav/<id>/state/vehicle` (`armed` **VÀ `connected`**), mẫu phải **tươi** trong `always_timeout_sec` của chính hàng `state/vehicle` (0,3 s — một nguồn số duy nhất, KHÔNG có bản sao thứ hai) VÀ đến từ **đúng MỘT** subscriber (N2: "state/vehicle" là hàng sentinel, `onVehicleState()` tự nuôi cadence của chính nó — trước đây có 2 subscriber độc lập trên cùng topic, arrival có thể lệch nhau, đóng lại bằng cấu trúc thay vì đua thời gian). 🔴 **N1: mất `connected` ⇒ strict `flight` NGAY, không đọc thành disarm** — `px4_state_adapter_node.cpp` chỉ gán `armed` **bên trong** `if (connected)`; mất liên lạc PX4 vẫn publish đều (topic tươi) nhưng `armed` rơi về mặc định `false` — không có điều kiện `connected`, đây từng đọc thành hạ pha "đo được" xuống PREFLIGHT giữa lúc bay. Nâng pha (`preflight→flight`) **TỨC THÌ**; hạ pha (`flight→preflight`) phải **dwell 7 tick eval liên tiếp** (0,35 s ở mặc định `report_period_sec=0,05s` — V4 (review lượt 3, thay N3): so với `2/expected_hz` là **thước đo SAI** dù đã nâng 3→5 theo N3 trước đó — grace THẬT của `state/vehicle` là chính `always_timeout_sec` của nó (0,3 s, lớn hơn `2/expected_hz`=0,2 s), nên dwell 0,25 s vẫn để lọt một mẫu cũ đủ xác nhận hạ pha; `declareAndValidateParams()` nay ép buộc `kPreflightDwellTicks×report_period_sec > always_timeout_sec(state/vehicle)`, từ chối khởi động nếu cấu hình vi phạm, R33). Không đo được (chưa từng nhận, hoặc quá cũ, hoặc mất `connected`) ⇒ **strict `flight`** và `gate_mode_source=unmeasured_strict` ⇒ **không bao giờ GO** khi không đo được pha. Bảng canh vẫn chia 2 nhóm: `always` (phải chảy cả khi đậu bãi) và `flight_only` (im lặng **hợp lệ** khi chưa bay). `preflight` ⇒ nhóm `flight_only` vẫn báo trong `aggregated` nhưng **KHÔNG tính vào go/no-go**; `flight` ⇒ tính. Cùng họ bài học R35/B5-P8: một tín hiệu đúng, đọc trong pha nó mang nghĩa khác, là guard cắn nhầm. 🔴 **KHÔNG dùng `in_air`** dù `VehicleState` có trường này — `px4_state_adapter_node.cpp`: `in_air = takeoff_time > 0` **không reset sau khi hạ cánh** (nợ #2 của P8) ⇒ sau chuyến bay ĐẦU TIÊN, `in_air` kẹt `true` vĩnh viễn; đưa nó vào vị ngữ pha sẽ tái tạo lại chính bug đang sửa (đèn kẹt `FLIGHT` mãi, waiver tiền bay không bao giờ áp dụng lại được). Nợ: muốn dùng `in_air` phải đóng nợ #2 của P8 trước |
| 🔴 **`gate_mode_override` (param, `auto` \| `preflight` \| `flight`) thay `gate_mode` cũ** | Luật cũ "flip via set_parameter at runtime" **đã bị thay** — nó từng là một lỗ mù trong lúc bay (nghĩa là ai đó có thể ghi tay `gate_mode=preflight` trong lúc đang bay và im lặng tắt go/no-go thật). Override giờ **CHỈ ĐƯỢC SIẾT, không bao giờ được NỚI**: `effective_phase = max(đo_được, override)` — `auto` và `preflight` không bao giờ ép được gì (cả hai chỉ "không nới"), chỉ `flight` có tác dụng (ép `flight` bất kể đo được gì) |
| 🔴 **`blocking` (KeyValue mới) — danh sách item đang chặn GO** | Tối đa **5** tên (định dạng `<source>:<child>` hoặc tên item Sub-A thường), sắp **NẶNG-TRƯỚC** (ERROR > STALE/UNKNOWN > WARN, cùng thứ tự `severityRank` quyết định `worst_item`), nối bằng `", "`; còn thừa ⇒ nối thêm `" (+N more)"`. Chỉ liệt item đang **counted** và không OK — item bị waive/owner-tắt/summary không xuất hiện. Chuỗi rỗng `""` khi mọi thứ OK |
| 🔴 **Sub-B chấm theo TỪNG CHILD, không lấy `max` toàn source** (P10.9b) | `worst_item`/`blocking`/`/diagnostics/aggregated` gọi mỗi child bằng `<source>:<child>` — **`<child>` là tên WIRE THẬT, KHÔNG rút gọn** (vd `diagnostics/safety:safety: OFFBOARD_UNHEALTHY` — giữ nguyên tiền tố `"safety: "` mà `safety_supervisor_node.cpp` tự đặt, vì D0 baseline capture nạp waiver đúng chuỗi này; rút gọn sẽ tạo một tầng chuyển đổi không cần thiết và có thể lệch khỏi dữ liệu D0 thật). Child là **summary/rollup đã CHỨNG MINH** của một source (field cấu hình `diag_source_summary_child`, hiện chỉ khai cho `diagnostics/safety` = `"safety"` — `failsafe_policy.cpp`'s `overall_level = std::max(...)` **chỉ trên các mã đang VIOLATING**, đã đọc code xác nhận là tập con của các child đã đếm riêng, G8) **không bao giờ được đếm theo level**. 5 source còn lại **CHƯA có chứng minh này** nên `summary_child=""` — các entry dạng rollup của chúng (vd `camera_health_node`'s `"perception: cameras"`) vẫn là child THƯỜNG (đếm bình thường) — bảo thủ có chủ đích (không kém an toàn hơn, chỉ chưa gọn), không phải một giả định chung cho mọi source |
| 🔴 **LUẬT KHÔNG BAO GIỜ MIỄN-TRỪ (NW1/NW2, P10.9b)** | Child mang `action` (KeyValue) tiền tố ∈ `{hold, inhibit, would_hold, would_inhibit}` **VÀ** `level==ERROR` ⇒ **LUÔN ĐƯỢC ĐẾM**, mọi pha, mọi cấu hình, không hàng miễn-trừ nào và không param nào tắt được (NW1). Child trên source `action_aware` (hiện CHỈ `diagnostics/safety` — 5 source còn lại không publish KeyValue `action` nên đặt `action_aware=false`, tránh biến "không áp dụng" thành "fail-closed sai chỗ") với `level==ERROR` mà thiếu/không phân giải được `action` cũng **LUÔN ĐƯỢC ĐẾM** (NW2, fail-closed khi P8 đổi từ vựng). `action` so bằng **TIỀN TỐ**, không so chuỗi đầy đủ (`"would_hold (dry-run, ...)"` phải khớp `hold`). **Action-class MỘT CHIỀU: chỉ để PHỦ QUYẾT, không bao giờ để CẤP PHÉP** — `action=report` không tự miễn-trừ, nó chỉ *không kích NW*, vẫn phải qua bảng miễn-trừ như bình thường |
| 🔴 **Bảng miễn-trừ `config/preflight_waivers.yaml` (P10.9b)** | Đúng HAI giá trị `when` hợp lệ: `preflight` (`effective_phase==PREFLIGHT`) và `perception_off` (`perception_enabled_==false`, **độc lập với pha**). Thêm giá trị thứ ba là sửa hợp đồng, không phải sửa cấu hình. Mỗi hàng bắt buộc `waiver_reason` không rỗng và **phải được chủ dự án ký** — thêm một hàng là quyết định ngang cấp sửa hợp đồng. Hàng trỏ vào summary/rollup child (dòng trên) ⇒ **node từ chối khởi động**. `sim.launch.py` và (P11) `real.launch.py` nạp **CHUNG MỘT FILE** (R7) — không có bảng riêng theo môi trường. **Mặc định của mọi mã KHÔNG liệt kê là ĐƯỢC ĐẾM** (fail-loud) — P8 thêm mã mới không cần sửa gì ở P10 để giữ an toàn |
| 🔴 **`waiver_unmatched` — phát hiện DRIFT P8↔P10** | Mỗi hàng waiver mà (source,child) của nó **chưa từng thấy** trên dây kể từ khi node khởi động (bất kể `when` có khớp hay không) sẽ được đếm vào `waiver_unmatched`; khác 0 nghĩa là P8 đã đổi tên child hoặc `action`, hàng waiver đó mồ côi — đọc là cảnh báo cấu hình lệch, không phải lỗi bay |
| 🔴 **Cửa sổ mù mở-nguồn → ARM, nghĩa vụ quy trình (P10.9b)** | Với vị ngữ đang được miễn-trừ, tồn tại khoảng hẹp từ lúc subsystem khởi động tới lúc ARM mà nó có thể hỏng đúng theo cách vị ngữ đó đo (subsystem còn sống, còn publish, nhưng sai). Miễn-trừ `preflight` bay hơi trong ≤1 tick eval khi `armed` lật (G4 chỉ trễ chiều HẠ, chiều NÂNG tức thì). **Checklist tiền bay P11 BẮT BUỘC đọc lại đèn SAU khi ARM và TRƯỚC khi takeoff** — cùng họ nghĩa vụ Y3 (latch tức thì sau arm, §2.18). Đèn **KHÔNG BAO GIỜ** được nối vào bất kỳ đường tự động arm/takeoff nào — vai trò của nó là tư vấn cho con người |
| 🔴 **CHƯA CHỐT, không được che (P10.9b)**: `ESTIMATOR_INPUT_INVALID` trong world GPS-denied | `evalEstimatorInputInvalid()` (`failsafe_policy.cpp:542`) đọc `gps_ok` như một số hạng AND cùng `imu_ok`/`barometer_ok`/`magnetometer_ok` — xác nhận đúng bằng đọc code trực tiếp. Ở `uav_arena_indoor` (GPS-denied theo thiết kế) mã này ERROR **vĩnh viễn**, kể cả đang bay (không chỉ tiền bay) ⇒ **không có `when` nào của bảng miễn-trừ diễn tả đúng** ("preflight" sẽ sai vì cũng ẩn nó lúc đang bay; "perception_off" không liên quan). **Cố ý KHÔNG miễn-trừ ở tầng observability** — miễn-trừ theo môi trường sẽ giấu luôn hỏng GPS thật khi bay ngoài trời. Câu hỏi của P8/backend (vd tách `gps_required` theo world), chưa chạy lại trên world indoor trong phiên P10.9b này — chờ chủ dự án quyết |
| 🔴 **Cấm nới cổng bằng waiver/owner (P10.9b, `o4-gate` (c))** | `perception:=true` trong sim hiện **PHẢI đọc `NO_GO`** vì cầu camera thật sự mất khung (nợ #10). Cổng `o4-gate` (c) khẳng định `worst_item` chứa `diagnostics/perception` — **cấm** làm xanh bằng cách thêm hàng miễn-trừ cho child perception hoặc đổi `owner`. Khi nợ #10 được vá, vòng này đỏ và ép cập nhật kỳ vọng — đó là hành vi ĐÚNG |
| 🔴 **`owner` (mục/nguồn thuộc subsystem TÙY CHỌN) — 2 luật KHÁC NHAU, không đối xứng** (vá 2026-08-23, review đóng phase) | Mỗi mục `always`/`flight_only`/`diag_source` có `owner` ∈ `{""(mặc định), "perception", "blackbox"}`. `owner="perception"` ⇒ tính vào go/no-go **CHỈ KHI** param `perception_enabled` (launch truyền từ cờ `perception`) đúng — sai (`perception:=false`) thì mục đó vẫn hiện trong `aggregated` (`counted=false`) nhưng KHÔNG cộng `unknown_count`/`stale_count`. `owner="blackbox"` (`diagnostics/observability`, `diagnostics/observability_events`) thì **KHÔNG BAO GIỜ** tính vào go/no-go — bất kể `blackbox_enabled` — vì O1: sức khoẻ hộp đen không được là điều kiện cất cánh. Trước fix này: cấu hình `perception:=false` khiến `world/semantic_landmarks`/`diagnostics/perception` UNKNOWN vĩnh viễn ⇒ `go_no_go=UNKNOWN` mãi mãi, không bao giờ phản ánh lỗi thật sau đó (mất on-change); và đĩa hộp đen đầy có thể ép `NO_GO` dù không liên quan an toàn bay |
| `/uav/<id>/diagnostics/aggregated` (`DiagnosticArray`) | 1 Hz — mọi mục canh + N status gộp từ `/diagnostics/*` (nay **6** nguồn, thêm `diagnostics/observability_events`) + 1 status `system`. Nguồn gộp không đến trong `sub_b_stale_after_sec` (mặc định 3 s) ⇒ đóng góp **UNKNOWN**, không giữ giá trị cuối (R32) |
| `/uav/<id>/diagnostics/observability` · `/uav/<id>/diagnostics/observability_events` | Sức khoẻ của **chính** hộp đen / event logger: `state` ∈ {`RECORDING`, `DEGRADED`, `DISABLED_NO_SPACE`}, `bytes_written`, `topics_failed`, `write_errors`, `bags_deleted`, `disk_free_bytes` / `lines_written`, `lines_dropped`, `fsync_errors`, `file_path` |
| 🔴 Dòng thời gian sự kiện là **FILE, không phải topic** | `event_logger_node` **KHÔNG publish sự kiện nào lên bus** — chỉ ghi `<log_root>/<uav_id>_<UTC>_events.jsonl`, một dòng một sự kiện, 9 khoá cố định `t_sim·t_wall·seq·src·field·from·to·level·detail`. `/mission/events` vẫn là kênh sự kiện mission **duy nhất** trên dây; JSONL là **dẫn xuất**, không phải nguồn sự thật thứ hai. Không consumer runtime nào được phụ thuộc vào nó |
| Sự kiện `level ≥ ERROR` trong JSONL | Đã `fsync` xuống đĩa **trước khi callback trả về** — thứ duy nhất được bảo đảm sống sót qua một lần process chết; các mức thấp hơn fsync theo chu kỳ `fsync_period_sec` (mặc định 2 s) |
| 🔴 **Hộp đen KHÔNG bao giờ publish lên đường bay** | Publisher của `rosbag_manager_node` chỉ gồm `/diagnostics/observability` (+ `/rosout`, `/parameter_events`) — kiểm được bằng `ros2 node info`, đã chứng minh ở P10.6. Observability hỏng ⇒ mất bằng chứng, **không được** mất chuyến bay |
| 🔴 **Bag sau sự cố PHẢI `ros2 bag reindex` trước khi đọc** (G-O1, đo thật) | `metadata.yaml` chỉ được ghi lúc writer đóng **sạch**; process bị giết giữa chừng ⇒ `ros2 bag info` báo *"Could not find metadata"*. Với `sqlite3` (**format mặc định**, chốt bằng thực nghiệm G-O1) `reindex` phục hồi **10/10** lượt, mất trung vị **0,032 s** dữ liệu cuối. ⚠️ Với `mcap` thì **mất TOÀN BỘ bag** (0/10 — plugin Humble 0.15.16 từ chối mở khi thiếu footer, không có quét tuyến tính) — đó là lý do format mặc định là sqlite3, không phải mcap |
| 🔴 Giới hạn hiệu lực của kết luận trên (R31) | `kill -9` mô phỏng **process chết**, KHÔNG mô phỏng **mất điện thật** (page cache vẫn được kernel flush). Không được đọc thành "bag an toàn khi mất điện" |
| 🔴 Teardown: recorder cần **SIGINT**, không SIGKILL | Giết cứng ⇒ bag không đóng sạch ⇒ phải reindex. `scripts/stop_sim.sh` đã có khối `pkill -INT` riêng cho `uav_observability` chạy **trước** vòng SIGTERM chung (bẫy thật P10.6: script vốn thiếu hẳn pattern này, để 3 node mồ côi mọi lần teardown) |
| Chất lượng dữ liệu trong bag (Q-P10-2) | Recorder subscribe **BEST_EFFORT** mặc định (trừ nhóm cứu-mạng và nhóm TransientLocal dùng Reliable) — đổi lấy **zero backpressure** lên publisher luồng bay. Hệ quả: bag **có thể thiếu mẫu** dưới tải; bag là bằng chứng chẩn đoán, **không phải** nguồn đếm chính xác tuyệt đối |
| Giới hạn đã biết — topic TransientLocal | Recorder vào muộn **có thể hụt mẫu latched cuối** (race discovery rosbag2 #967, không có cách khắc chế 100% trong Humble). Giảm thiểu: `rosbag_manager_node` xếp **đầu tiên** trong launch |
| Retention đĩa | Xoá theo **nguyên thư mục bag**, cũ nhất trước, **không bao giờ** xoá bag đang ghi; hết ứng viên mà vẫn quá ngưỡng ⇒ đóng writer + `DISABLED_NO_SPACE` (không im lặng). ⚠️ Giới hạn: **không tự khôi phục ghi** khi đĩa được giải phóng sau đó — phải khởi động lại node |
| 🔴 **Retention hardening (vá 2026-08-23, review đóng phase, 4 lỗ)** | (1) Đo `disk_free_bytes`/mtime lỗi ⇒ **NaN**, không phải `0.0` — "chưa đo được" không bao giờ đọc thành "chắc chắn đầy"/"chắc chắn cũ nhất" (R30); tiêu chí `max_total_bytes` (đo độc lập từ dung lượng từng bag) vẫn có hiệu lực. (2) So khớp `active_path` chuẩn hoá lexically (chống bag đang ghi lọt danh sách xoá vì `bag_root` có `/` cuối). (3) `onRetentionTick()` return sớm khi không có bag đang ghi (`DISABLED_NO_SPACE`/`DEGRADED`) — trước đây chạy tiếp với `active_path=""`, biến MỌI bag cũ (kể cả bằng chứng chuyến trước) thành ứng viên xoá. (4) Ứng viên xoá phải khớp tiền tố `<uav_id>_` **và** chứa `metadata.yaml`/`*.db3`/`*.mcap` — một thư mục lạ dưới `bag_root` (log khác, thư mục scratch) không bao giờ bị đụng. Thêm: từ chối khởi động nếu `bag_root` là `$HOME` hoặc `/` |
| 🔴 **`uav_id` sai namespace trong yaml ⇒ từ chối khởi động** (C5, vá 2026-08-23) | Mọi topic `/uav/*` trong `topics:` phải khớp `"/uav/" + uav_id + "/"` (miễn trừ `/tf`, `/tf_static`, `/fmu/*`) — trước đây `create_generic_subscription` không đòi publisher tồn tại nên yaml trỏ nhầm drone (`uav_id:=uav1` mà `topics:` vẫn `/uav/uav0/...`) sẽ subscribe "thành công", báo `RECORDING`, mà bag chỉ có `/tf`+`/fmu/*` — mất trắng bằng chứng đúng chuyến cần điều tra, đèn vẫn xanh |
| `storage_id` mặc định TRONG CODE (VÀNG#8, vá 2026-08-23) | Đổi từ `mcap` sang `sqlite3` khớp yaml + G-O1 — yaml quên khai `storage_id` (vd config mới cho `real.launch.py`, P11) trước đây sẽ **im lặng rơi về `mcap`** (format đã đo mất 100% bag khi tắt máy không sạch). Có `RCLCPP_WARN` nếu ai chủ động chọn `mcap` |
| ⚖️ Ghi `/fmu/*` — ranh giới R1 (Q-P10-1, chủ dự án ký) | Hợp lệ **cùng họ ngoại lệ R25**: recorder ghi **chuỗi byte đã serialize**, tên kiểu là **dữ liệu trong yaml**, không import/link/đọc trường nào. 4 điều kiện cứng: (1) `package.xml`/CMake không có `px4_msgs`; (2) whitelist tường minh trong yaml sau cờ `record_px4_topics`, không wildcard; (3) mã nguồn không chứa chuỗi `px4` — **ngoại lệ duy nhất là định danh cờ `record_px4_topics` + comment**; (4) thiếu typesupport lúc chạy ⇒ degrade có báo cáo (`topics_failed`), không throw |
| Mặc định theo môi trường (D-4, O5) | `sim.launch.py`: `blackbox`/`diagnostics`/`event_log` mặc định **true**. `real.launch.py`: `blackbox` mặc định **false** cho tới khi P11 review độc lập ngân sách đĩa/IO onboard (nếp `safety_enforcement`) |
| 🔴 **Bug đã vá (P10.8c): `offered_qos_profiles` bị bỏ trống** | `rosbag_manager_node.cpp` tạo `TopicMetadata` chỉ set `.name/.type/.serialization_format`, để trống `.offered_qos_profiles` ⇒ `metadata.yaml` không mang durability cho topic nào ⇒ `ros2 bag play` phát mọi topic **Volatile** bất kể QoS gốc ⇒ subscriber TransientLocal từ chối mẫu latched. **Đã vá**: encode qua `rosbag2_transport::Rosbag2QoS`'s `YAML::convert` (cùng bộ mã hoá mà `ros2 bag play` dùng để giải mã ngược) — không tự chế chuỗi YAML tay. Test tất định: `RosbagManagerNode.OfferedQosProfilesEncodesDurabilityPerTopic` (ghi 1 topic `reliable_tl` + 1 topic `be`, đọc lại `metadata.yaml`, khẳng định durability đúng RIÊNG từng topic — đối chứng dương topic `be` KHÔNG mang `transient_local`). Xác nhận thật trên bag bay (indoor_patrol, sau vá): 3 số đo lõi (authority edges, mission/events, odometry_fused) khớp 100% + **6/8** topic `reliable_tl` khớp 100% reference=live (authority 305/305, estimator_source 1/1, planning/trajectory 32/32, safety/state 757/757, mission/status 160/160, world/mission_reference 1/1) |
| 🔴 **P10.8c chỉ vá MỘT nửa — C3 (review đóng phase, vá 2026-08-23): giá trị ghi là QoS SUBSCRIBER, không phải QoS PUBLISHER thật** | Vá P10.8c ở trên chỉ sửa "trường trống" — giá trị GHI vẫn là `subscribed_qos` (QoS mà chính recorder subscribe, vd `be` theo Q-P10-2 cho `cmd_mission`/`cmd_operator`/`cmd_safety`/`cmd_test`/`diagnostics/control_authority`), KHÔNG phải QoS mà publisher THẬT offer (các node sản xuất `ControlCommand` này offer **Reliable**, mặc định `rclcpp::QoS(N)`). Hệ quả đo được thật trên dây (§ dòng G-O3(c) dưới): `ros2 bag play` dựng lại publisher BestEffort cho `cmd_mission` ⇒ subscriber Reliable (arbiter) nhận **0 mẫu**, log `RELIABILITY_QOS_POLICY` incompatible từ giây đầu tới cuối phiên. **Đã vá**: `writer_->create_topic()` dời sang **lần message ĐẦU TIÊN thật sự nhận được** cho mỗi topic (không còn tại thời điểm subscribe, lúc gần như chưa publisher nào lên — P10.6 cố tình xếp recorder đầu tiên để thắng race #967) — tại đó `get_publishers_info_by_topic()` chắc chắn thấy được publisher vừa gửi, dùng đúng QoS **THẬT** của nó; fallback về `subscribed_qos` (kèm `RCLCPP_WARN` nói rõ là suy đoán) chỉ khi 0 hoặc >1 publisher phát hiện được. Test tất định: `RosbagManagerNode.OfferedQosReflectsRealPublisherNotOurOwnBestEffortSubscription` (recorder subscribe `be`, publisher thật `reliable` — khẳng định `offered_qos_profiles` ghi lại **Reliable**, không phải `be`). Đánh đổi đã ghi nhận: một topic **0 traffic suốt chuyến bay** giờ không xuất hiện trong `metadata.yaml` nữa (trước đây luôn có mặt, 0 message) — chấp nhận được, cùng ý nghĩa thực tế (không có gì để phát lại) |
| 🟡 **2 topic `reliable_tl` FAILED TO MEASURE trên bag indoor (không phải lỗi vá)** | `gnss_origin` (0 mẫu) và `/tf_static` (0 mẫu) không có dữ liệu để kiểm phát lại — KHÔNG do bug QoS. Đã xác nhận nguyên nhân bằng thực nghiệm: (a) `gnss_origin` chỉ được `px4_state_adapter_node` phát khi PX4 EKF có tham chiếu GPS toàn cục — world indoor (`uav_arena_indoor`, GPS-denied theo thiết kế) không bao giờ kích hoạt; bay lại M5 nhanh trên world ngoài trời `uav_arena`/`uav0_nav` **xác nhận có đúng 1 mẫu** trong bag, chứng minh publisher hoạt động đúng, chỉ là world indoor không tạo ra dữ liệu để kiểm. (b) `/tf_static` **không có publisher nào trong toàn bộ codebase** (grep xác nhận: `px4_frame_bridge_node` chỉ dùng `TransformBroadcaster` động → `/tf`, không có `StaticTransformBroadcaster` nào ở đâu) — 0 mẫu trên MỌI bag, mọi world, không liên quan P10.8c. **Nợ mới, ngoài phạm vi P10.8c:** thêm publisher `/tf_static` (P10.9/P11, khi có frame tĩnh cần công bố, vd sensor mount) |
| 🔴 **Giới hạn đã biết (P10.8c): `ros2 bag play --loop` KHÔNG phải phép đo hợp lệ cho "N-c bắt được regression thật"** | Cơ chế N-c (`ageSecOrInf`/`clock_regressions_`, §5) tự nó ĐÚNG — kiểm chứng tất định trên node thật (`o3_clock_regression_synth.py`, `clock_regressions` 0→26 khi cố ý cho một kênh im lặng qua một cú lùi đồng hồ thật, tái lập được 2 lần), **không đổi bởi phần dưới**. ⇒ Hệ quả cổng: `verify_observability.sh` dùng vòng **`o3-synth`** (chạy `o3_clock_regression_synth.py` trên node thật) làm bằng chứng tất định chính thức thay cho tiêu chí `clock_regressions>0` qua `--loop`. 🔴 **Vá 2026-08-24 (N4, P10-gate-debt review round 2):** `round_o3_loop_report`'s check cũ #4 (`--loop`: `clock_regressions>0`) và #5 (không `--loop`: `clock_regressions==0`) đã **RÚT khỏi verdict** — dòng CHỐT bên dưới (G-O3(c)) cho thấy cả hai đo trúng nhiễu độ phân giải công cụ replay, không phải cơ chế; giờ chỉ còn in `[INFO]`, không PASS/FAIL. Verdict của vòng này chỉ còn dựa check #1 (o3-synth) + check #2/#3 (đối chứng dương traffic tới được trọng tài) |
| 🔴 **KẾT LUẬN NGUYÊN NHÂN GỐC CỦA `--loop` Ở TRÊN ĐÃ SAI — điều tra lại 2026-08-23 (review đóng phase, R35), sửa bằng chứng cụ thể** | Kết luận cũ ("MISSION tự nhả quyền qua `release_dwell_sec` trước khi bag kết thúc ⇒ không còn kênh LIVE để bắt regression") đã bị THAY vì đọc lại `~/gate_logs/o3_play_loop.log` + `o3_ca_loop.log` (log gốc của chính phép đo G-O3(c), chưa từng đọc kỹ trước đó). Bằng chứng thật trong 2 log: (1) `o3_play_loop.log` — **NGAY GIÂY ĐẦU TIÊN của phiên `--loop`** (`t=113.27`), `rosbag2_player` báo `New subscription discovered ... requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY` cho **CẢ 4** topic `cmd_mission`/`cmd_operator`/`cmd_safety`/`cmd_test` VÀ `diagnostics/control_authority` — đúng chính bug C3 (recorder ghi `offered_qos_profiles=be` cho các topic này thay vì Reliable thật, nên `ros2 bag play` dựng publisher BestEffort, không tương thích với subscription Reliable mặc định của arbiter) — nghĩa là **arbiter KHÔNG HỀ nhận một cmd_mission nào trong suốt phiên `--loop`** (từ giây đầu tới lúc SIGINT ở `t=384`), không phải chỉ "không nhận đúng lúc đồng hồ lùi". (2) `o3_ca_loop.log` — arbiter tự báo lỗi `2 publishers on /uav/uav0/control/command_selected, only this node may write it` LẶP LẠI **~60 lần** suốt phiên (mỗi ~2-3 s, từ `t=114` tới `t=264`): `command_selected`/`authority` cũng NẰM TRONG bag (yaml ghi cả 2), nên `ros2 bag play` tự tạo publisher THỨ HAI ghi đè lên đúng topic exclusive-writer mà arbiter đang giữ — bất kỳ đầu dò nào theo dõi `active_source` qua `/control/authority` trong phiên này rất có thể đang thấy **chính bag phát lại dữ liệu cũ của nó**, không phải quyết định thời gian thực của arbiter sống. Cô lập bằng cơ chế, không suy đoán: arbiter (do QoS chặn) không có cmd_mission thật để xử lý ⇒ `active_source` CỦA ARBITER lẽ ra đứng yên NONE cả phiên; dao động NONE↔MISSION "khớp 2 cạnh gốc" quan sát được nhiều khả năng là replay của chính `/control/authority` đã ghi, không phải arbiter đang giữ-rồi-nhả quyền qua `release_dwell_sec` như kết luận cũ khẳng định. **Kết luận:** nguyên nhân gốc THẬT của `clock_regressions=0` trong ca `--loop` cụ thể này là **arbiter chưa từng có cmd_mission LIVE để mất** (do C3, suốt phiên, không chỉ tại thời điểm lùi đồng hồ) + tín hiệu quan sát bị nhiễm bởi publisher thứ hai từ chính bag — KHÔNG phải vì mission đã nhả quyền "bình thường". Sau khi vá C3, `cmd_mission`/`command_selected`/`authority` sẽ replay đúng QoS — **cổng G-O3(c) CẦN CHẠY LẠI** (bay + capture bag + `--loop` lại) để xác nhận nguyên nhân/kết quả mới; ngoài phạm vi phiên sửa lỗi P10-review này (đòi hỏi bay sim + capture đầy đủ). Tới lúc đó: đừng trích dẫn kết luận cũ ở đây làm căn cứ cho bất kỳ quyết định nào — coi là **CHƯA CHỐT** |
| 🔴 **P10.9a (2026-08-24) — G-O3(c) CHẠY LẠI sau khi vá double-writer, vẫn CHƯA CHỐT nhưng với bằng chứng mới, khác gốc** | `scripts/verify_observability.sh`'s `o3_run_authority_watch` đổi sang `ros2 bag play --topics cmd_mission cmd_safety cmd_operator cmd_test` (loại `command_selected`/`authority` khỏi replay — hết double-writer). Thêm đối chứng dương **`ever_saw_mission`** trước khi tin bất kỳ số `clock_regressions` nào (R27-1/R27-3) | **Giả thuyết C3 (arbiter mù hoàn toàn vì QoS/double-writer) đã LOẠI dứt điểm bằng số**: cả lượt `--loop` lẫn không-`--loop` đều có `ever_saw_mission=true` (arbiter thật sự nhận cmd_mission, `active_source→MISSION` quan sát được nhiều lần). **Nhưng lộ MỘT bất thường khác, chưa từng thấy khi bị 2 bug cũ che khuất**: đối chứng dương "không `--loop` ⇒ `clock_regressions==0`" tự THẤT BẠI — đo ra **1505** (không phải 0), tăng đều ~20/s suốt phiên, cùng nhịp với cmd_mission (~20 Hz) — gần như MỖI message đều bị tính là một regression. Lượt `--loop` cũng tăng đều cùng nhịp (**4957** sau ~166 s), KHÔNG có bước nhảy riêng biệt tại điểm wrap để phân biệt với lượt không-loop. ⇒ Hiện tượng **không đặc thù cho `--loop`** — xảy ra với bất kỳ phiên `ros2 bag play --clock` nào một khi cmd_mission chảy, khác hẳn cơ chế "wrap tại một cú lùi đồng hồ" mà cổng này định đo. Nghi phạm hàng đầu (CHƯA đo xác nhận, chỉ đọc code): lệch domain giữa `/clock` do `ros2 bag play --clock` tổng hợp lại và `stamp_sec` nhúng trong từng `ControlCommand` (`control_authority_manager_node.cpp:47`) — chi tiết + đường điều tra tiếp theo → `docs/ops-playbook.md` §18 dòng cuối. **Kết luận G-O3(c): giữ nguyên CHƯA CHỐT** — nhưng đã loại được giả thuyết C3, và đã cô lập một biến số MỚI (replay-clock vs stamp_sec) cần điều tra trước khi `--loop` (hay bất kỳ phép đo qua `ros2 bag play --clock` nào đo `clock_regressions`) có thể dùng làm bằng chứng cho cơ chế N-c trên dây. Cổng tất định `o3-synth` (không đi qua `ros2 bag play`) không bị ảnh hưởng, PASS không đổi (0→26 khi cố ý im lặng qua cú lùi thật) — vẫn là bằng chứng chính thức duy nhất cho cơ chế N-c |
| ✅ **G-O3(c) ĐÃ CHỐT 2026-08-24 — nguyên nhân là ĐỘ PHÂN GIẢI CỦA CÔNG CỤ REPLAY, không phải lỗi hệ** | **Số đo chốt hạ:** `/clock` do `ros2 bag play --clock` tổng hợp là **tick RỜI RẠC ~40,03 Hz (chu kỳ ~25 ms)**, không đồng bộ pha với lịch phát message; `cmd_mission` chảy ~20 Hz (50 ms) ⇒ khi subscriber `use_sim_time` đọc `now()`, mẫu `/clock` mới nhất **thường chưa tick qua** `header.stamp` của message đang xử lý ⇒ `now < stamp` GIẢ. Đo trực tiếp 30 mẫu liên tiếp: **29/30 (96,7%) âm**, biên độ **−0,000 → −0,023 s** (trung bình −0,0123 s) — **khớp đúng chu kỳ tick 25 ms**. 🔴 **Giả thuyết "lệch domain" (nghi phạm hàng đầu ở dòng trên) ĐÃ BỊ LOẠI bằng ground-truth**: đọc thẳng bag bằng `rosbag2_py` (không qua `bag play`), `recv_time` (do `rosbag_manager_node` ghi bằng `now()`) và `header.stamp` (do navigator gán bằng `now()`) **giống nhau tới micro-giây: `diff = 0,000000` trên 15/15 mẫu** ⇒ cùng domain sim-time, không hề lệch trong dữ liệu gốc; và `/clock` khi replay chạy đúng khoảng bag (14,65 → 219,8 s, khớp `bag info`). **KHÔNG ảnh hưởng bay thật** (3 bằng chứng độc lập): M5/G-M1 PASS · ground-truth `recv−stamp=0` chứng minh khi bay, cả hai đầu lấy từ **một luồng `/clock` sống liên tục** (Gazebo phát trực tiếp) · cơ chế lỗi nằm ở **cách công cụ replay tự dựng `/clock`**, không tồn tại khi hệ chạy live. 🔴 **KHÔNG được sửa `ageSecOrInf()` để "tha" cho nhiễu này** — dung sai 0 là ĐÚNG cho vận hành thật; nới nó ra là làm yếu chính cơ chế N-c đang bảo vệ bay thật. **Hệ quả cho mọi phép đo tương lai:** bất kỳ tiêu chí nào so sánh thời gian với **dung sai 0** đều KHÔNG đo được qua `ros2 bag play --clock` (độ phân giải công cụ 25 ms lớn hơn tín hiệu cần đo); `o3-synth` (không đi qua bag play) là cách đúng, giữ nguyên |

---

## 3. Kiểm kê hiện trạng

**Hoàn thiện 2026-08-07: 17 msg + 9 srv + 9 action**, build sạch 0 warning.
**Mở rộng 2026-08-15 (chủ dự án duyệt): 20 msg** — thêm trường `position_uncertainty` vào 3 msg (§2.12) + 3 msg lớp world `SemanticLandmark`/`SemanticLandmarkArray`/`TargetState` (§2.13).
**Mở rộng 2026-08-19 (chủ dự án duyệt): 21 msg** — thêm `AvoidanceAdvice` cho P6.4 (§2.15). Thuần bổ sung, không đổi/xoá trường cũ nào.
**Cùng ngày (Đ1, `P6-completion-run.md`):** vẫn **21 msg** — không thêm msg mới, chỉ thêm trường `plan_state`/`reason` vào `Path3D` + `Trajectory3D` (thêm cả `sequence` vào `Trajectory3D`), xem §2.6.
**Mở rộng 2026-08-22 (P9.1, D-3, chủ dự án duyệt): vẫn 21 msg** — không thêm msg mới, chỉ thêm trường `last_result_code`/`last_reason`/`goal_id` vào `MissionStatus` + hằng `ABORTED_LOW_BATTERY=12` vào `ResultCode`, xem §2.19. Thuần bổ sung, không đổi/xoá trường cũ nào.

| Nhóm | Loại | Tên |
|---|---|---|
| **Lõi điều khiển & an toàn** | msg (10) | `ResultCode`, `VehicleState`, `ControlAuthority`, `ControlCommand`, `SafetyState`, `TrajectoryPoint`, `Trajectory3D`, `Path3D`, `AvoidanceAdvice` (§2.15), `OffboardStatus` |
| | srv (4) | `Arm`, `Disarm`, `SetFlightMode`, `SetControlAuthority` |
| | action (9) | `Takeoff`, `Land`, `GotoPose`, `HoldPosition`, `FollowPath`, `TrackTarget`, `InspectMarker`, `Recover`, `ExecuteMission` |
| **Quan sát & báo cáo** | msg (8) | `MarkerObservation`, `TargetTrack`, `Obstacle`, `ObstacleArray`, `MissionStatus`, `MissionEvent`, `LocalizationStatus`, `VehicleHealth` |
| | srv (5) | `LoadMission`, `PauseMission`, `ResumeMission`, `AbortMission`, `ClearFault` |
| **World model (§2.13)** | msg (3) | `SemanticLandmark`, `SemanticLandmarkArray`, `TargetState` |

Nhiều hơn thiết kế gốc 2 msg, đều có lý do: `ResultCode` (§2.1) và `OffboardStatus` (thiết kế có topic `/backend/offboard_status` nhưng chưa định nghĩa kiểu).

⚠️ **`LocalizationStatus` và `VehicleHealth` là hai message sát an toàn nhất của nhóm quan sát** — lớp an toàn sẽ đọc chúng để quyết định failsafe. `LocalizationStatus.is_valid = false` nghĩa là **cấm bay tự hành dựa trên nguồn này**.

---

## 4. Quy tắc thay đổi hợp đồng

- Hợp đồng này **đóng băng ở v0.1** sau khi nhận đủ phần bàn giao. *(Mở rộng thuần-bổ-sung 2026-08-15 §2.12–§2.13, 2026-08-19 `AvoidanceAdvice` §2.15 + `plan_state`/`reason`/`sequence` §2.6 — đều do chủ dự án duyệt, không đổi/xoá trường cũ nào.)*
- Đổi tên trường / đổi giá trị hằng số = **thay đổi phá vỡ tương thích** → phải rà mọi package đang dùng.
- Thêm trường mới ở cuối message thường an toàn hơn đổi trường cũ.
- Mọi thay đổi ở `ControlCommand`, `ControlAuthority`, `Recover` **phải được review theo tiêu chí an toàn** trước khi merge.
