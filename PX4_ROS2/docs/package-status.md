# Trạng thái hiện thực từng package

> Chi tiết **đã làm gì / chưa làm gì / bẫy đã biết** của từng package. `CLAUDE.md` chỉ giữ bản đồ tổng quan; mọi chi tiết hiện thực nằm ở đây.
>
> Cập nhật: **2026-08-24** · Tiến độ cột mốc & quyết định: `.claude/memory.md`
> Vì sao giữ Gazebo & cách chữa "world không đáng tin": [`sim-fidelity-decision.md`](sim-fidelity-decision.md)
> Vì sao nhánh world 3D Bách Khoa bị đóng & cất: [`lesson-bk-world-attempt.md`](lesson-bk-world-attempt.md)

---

## Tổng quan 12 package

| # | Package | Trạng thái | Ghi chú |
|---|---|---|---|
| 1 | `uav_interfaces` | ✅ **Xong** | 21 msg + 9 srv + 9 action. **47 case / 1 target** (luật ba loại message, danh sách ngoại lệ tự canh chính nó) |
| 2 | `uav_px4_backend` | ✅ **Xong** | 5/5 node, C++. **57 case / 2 target**. 🆕 **2026-08-26 (S3):** `test_frame_conversions` **13 → 22 case** — bốn hàm trên **đường VẬN TỐC và TỐC ĐỘ XOAY** (`nedToEnuVector` · `enuToNedArray(Vector3)` · `fluToFrdArray` · `yawRateEnuToNed`) **chưa từng được ctest gọi**; mọi case cũ chỉ phủ đường **vị trí**. Sai dấu vận tốc nguy hiểm ngang sai dấu vị trí |
| 3 | `uav_bringup` | ✅ **Dùng được** | `sim.launch.py` + smoke-test; `real.launch.py` để P11. **23 case / 2 target**. 🆕 2026-08-25: M5 nay **đọc `/safety/state`** và **mã an toàn chưa ai viết lý do thì CHẶN một PASS**; arm chờ được trong ngân sách và **in thời gian chờ mỗi chuyến** (§3) |
| 4 | `uav_sim_gz` | ✅ **Dùng được** | **5 world** — mặc định là **`uav_arena`** — + 3 biến thể model + bridge + 2 bộ sinh world (OSM · ảnh khảo sát). **11 case / 1 target** (bất biến SDF: collision **bao hàm** visual) |
| 5 | `uav_localization` | ✅ **Xong (P4)** | 6 node: rangefinder · GPS · VIO · optical flow · mux · health. **115 case / 12 target**. 🆕 2026-08-25: tách **`localization_health_ros`** (thư viện, để test lái được node thật) và **sửa luật phát hiện nhảy pose** — cộng nhiễu do nguồn tự khai, thêm trạng thái *không phán được* (§5). Bay được cả ngoài trời lẫn **trong nhà không GPS**. Còn **3 nợ nhỏ** (§5) |
| 6 | `uav_perception` | ✅ **Dùng được (P5)** | 5 node: camera_health · marker · obstacle (+ bộ lọc mặt phẳng nền vòng 1 + bù nghiêng attitude vòng 2 + cổng độ-sâu-dưới-camera vòng 3, 2026-08-22) · tracker (+ xác nhận track M/N chặn bug #10 + **bù ego-motion (liên kết trong odom) chặn churn gốc rễ**, 2026-08-23) · object_detector (khung DNN, không tuning). **88 case / 5 target** + cổng sim tĩnh PASS + xác nhận trên dây thật (kể cả lát cắt bay động). ⏳ Cổng động D1 chặn RTF. **Cố ý KHÔNG có driver camera** — xem §6 |
| 7 | `uav_world_model` | ✅ **Dùng được (P5.6)** | `world_model_node`, **83 case / 5 target**, cổng S5/S6 PASS + **cổng tổng (b) PASS 2026-08-16** (mount camera trước xác nhận đúng); 2 bug R0 của `/world/target_state` đóng 2026-08-22 — xem §7 |
| 8 | `uav_navigation` | ✅ **P6 ĐÓNG TRỌN 2026-08-20** · 🆕 vá tiền đề `StaleRouteFixture` 2026-08-25 (§8) | `navigator_action_server_node` đủ **7 action** (Takeoff·GotoPose·HoldPosition·Land·FollowPath·TrackTarget·Recover), GotoPose bay **quỹ đạo B-spline**, phát `/planning/trajectory` + carrot-publish. **274 case / 11 target** (mốc đóng phase 265/11 · +G-N5 bay lại ở chiến dịch P9 · +3 test 2026-08-24: guard độ tươi route · hình lõm `CupFixture` · đối chứng reactive-only). Cổng: **G-N1 8/8** · **G-N2 15/15** · **G-N3 ĐÓNG** · **G-N4b** · **G-N5** · **G-N6 ×2**. 🆕 **2026-08-26:** +2 case nhánh **từ chối** của advisor (costmap không hợp lệ · vị trí không hữu hạn) · vá tiền đề `NavigatorAdviceFixture` bằng **nhân chứng kích thích** (đếm advice thật sự phát; timer probe đói ⇒ `FAILED TO MEASURE` thay vì buộc tội node). 🔴 **NỢ MỚI #19: `Costmap` suy độ tươi từ SAI đại lượng** — xem `memory.md` §7 mục 19 và ranh giới **B-65**. ⚠️ **Trần tốc độ hạ còn 0,55 / 0,45 m/s** — suy từ ngân sách dây xích, xem §8 |
| 9 | `uav_control_authority` | ✅ **P7.0–P7.5 đóng; C4 (P10.8b) đóng** | Single-writer trên `command_selected`, navigator đã vào bringup, G-CA2 PASS. **98 case / 2 target**. §9 dưới. 🆕 **2026-08-26:** **hai** fixture nhạy tải của package này đã truy tới gốc — **cả hai là lỗi phép đo, trọng tài đúng luật cả hai lần**. `OnlyTheHigherPriority…`: tiền đề *"hai nguồn cùng sống"* **chưa bao giờ được dựng** (5/6 lượt đỏ là đua lúc khởi động, 2 lượt gap MISSION **2,45–2,51 s** ≫ dwell 0,20 s, nhân chứng probe ≤0,027 s mọi lượt). `ALatchThatNeverPublishes…`: `active_source=NONE` khi luồng bơm của chính test đói quá `source_timeout_sec`. Vá bằng hai **hàm luật thuần** (`countPriorityViolations` · `countUnjustifiedNone`) neo `header.stamp` + chốt nhân chứng probe + **7 ca luật đối chứng, trong đó 3 ca BẮT BUỘC kết tội** |
| 10 | `uav_safety` | ✅ **P8 ĐÓNG TRỌN 2026-08-21** (sổ nợ = 0 từ 2026-08-22) · 🆕 vá phán quyết `CutChain.Item8` 2026-08-25 (§10) | 🔴 sát an toàn — INHIBIT + HOLD enforcement THẬT đều đã bật (mặc định sim); latch SAFETY không tự hết hạn (R5). **123 case / 3 target**, 0 lỗi — §10 dưới. 🆕 **2026-08-26 (S3):** `test_failsafe_policy` **94 → 97 case** — ba nhánh cắt chưa từng chạy: hai đường phát hiện offboard **cùng lúc** · leo thang **HOLD→INHIBIT khi còn đang engage** · điều kiện clear của battery. 🪤 Hai hợp đồng dễ đọc sót, đã trả giá khi viết: `clearFault` **phải POLL** qua `clear_stability_sec`, và nguồn INHIBIT chắc chắn là **battery critical + PX4 không hành động** |
| 11 | `uav_mission` | ✅ **P9 ĐÓNG TRỌN 2026-08-23** · 🟠 **nợ #15 THU HẸP 2026-08-25** | `mission_executor_node` (BT.CPP thật) + `NavGoalBroker` + 3 lib ROS-free. **125 case / 5 target**. 🆕 **2026-08-26 (S3): `NavGoalBroker` từ 0 → 13 case unit test riêng** — trước đó nó **chưa hề có test của mình**, dù nhiệm vụ duy nhất là giữ bất biến *"tối đa MỘT goal navigator, mãi mãi"*; các nhánh tồn tại **chỉ để sống sót qua callback lạc/muộn** chưa từng chạy. Thêm `ASustainedAuthorityLapsePausesTheMission` — tái hiện **cơ chế** (bơm im 3,5 s ⇒ PAUSED + `kResultAbortedNoAuthority`) thay vì chờ hiện tượng. 9/9 cổng bay + M5 PASS, 2 lượt review sạch — §11 dưới. ✅ **Cả hai ca nhạy tải của package này đã truy tới gốc và vá, có đối chứng**: `TerminalStatusGoalIdMatchesSettlingGoal_*` là **lỗi sản phẩm thật** (mẫu terminal thiếu `goal_id`, vá bằng `settling_goal_id_`) · `AuthoritySeizeSustainedCancelsAndPauses` là **lỗi phép đo** (đo ân hạn từ **lời gọi hàm của chính test**, trong khi executor coi *authority hết tươi 1,0 s* cũng là mất quyền ⇒ thread phát của fixture đói là ân hạn đã chạy trước). Luật neo tách thành hàm thuần `authorityHeldEnd()` + **4 case `AuthorityGraceAnchor`** ghim cả ca luồng im 3 s. 🔴 **Nợ #15 vẫn MỞ ở cấp workspace** — xem `memory.md` §7 nợ #15 và #16 |
| 12 | `uav_observability` | ✅ **P10 ĐÓNG TRỌN 2026-08-24** | 3 node vào `sim.launch.py` (`blackbox`/`diagnostics`/`event_log`, mặc định **true**), recorder xếp đầu danh sách node. **163 case / 6 target**, 0 lỗi, 2 lượt liên tiếp. **4 cổng PASS (G-O1/G-O2/G-O3/G-O4)** · 3 lượt review, 15 CHẶN tìm-và-vá, lượt 3 "KHÔNG CÒN CHẶN". **G-O3(c) ĐÃ CHỐT 2026-08-24 — nguyên nhân là ĐỘ PHÂN GIẢI CỦA CÔNG CỤ REPLAY, không phải lỗi hệ** (contract §2.20). Hộp đen mặc định `sqlite3` (mcap mất 100% bag khi bị giết). Nợ còn mở: `O4_ALLOWED_WORST` quá hẹp ở (b) · **G-O4 (g)/(g2) SITL bay-thật chưa chạy (có chủ đích)** · nợ #10 camera bridge — §12 dưới |

Phụ thuộc bên thứ 3 đã cài trong workspace: `px4_msgs` (nhánh `release/1.15`), `px4_ros_com` (dùng `frame_transforms`).

---

## 1. `uav_interfaces` ✅

**21 msg + 9 srv + 9 action**, build sạch 0 warning.

Lý do thiết kế, quy ước header, các quyết định an toàn → [`interface-contract-v0.1.md`](interface-contract-v0.1.md).

Nhiều hơn thiết kế gốc 2 msg, đều có lý do: `ResultCode` (bảng mã lỗi dùng chung 9 action) và `OffboardStatus` (thiết kế có topic nhưng thiếu kiểu).

---

## 2. `uav_px4_backend` ✅

Package **duy nhất** được dùng `px4_msgs` (R1). Viết bằng **C++**. **48 case / 2 target** (2026-08-25).

### 🔴 P11.6 — stack KHÔNG được giành máy bay khỏi tay pilot (vá 2026-08-25)

**Lỗi chỉ tồn tại ở đời thật.** RC override đưa PX4 vào chế độ tay (`POSCTL`/`MANUAL`/`ALTCTL`/
`POSITION_SLOW`/`ACRO`/`STAB`). `autopilotOwnsTheFlight()` trả **false** cho các chế độ đó — **cố ý, và có
test ghim** (`PilotModesLeaveControlAvailable`), vì **mọi chuyến bay đều engage offboard từ trạng thái mặt
đất `POSCTL`**; sửa hàm đó là làm hỏng mọi chuyến bay. Hệ quả cũ:

1. `STATE_ACTIVE` mất offboard vào chế độ tay ⇒ `STATE_FAULT`, *"autopilot left offboard mode
   unexpectedly"* — **quy kết autopilot cho một hành động có chủ đích của con người**.
2. Nguy hơn: `readyToEngage()` = `auto_engage_ && !autopilotOwnsTheFlight() && primed` ⇒ **true** trong khi
   pilot đang lái. Luồng setpoint đứt rồi phát lại (FAULT→IDLE→STREAMING) là stack **gọi `requestOffboard()`
   và giành lại máy bay**.

**Vá — trong ranh giới chủ dự án ký** (chỉ tham số PX4 + hành vi của ta, **không đụng nguồn PX4**):
`pilotIsFlying(nav_state)` (predicate mới, **không phải** nghịch đảo của hàm cũ — hai hàm trả lời hai câu
khác nhau) + chốt `updatePilotOverride(latched, nav_state, arming_state)`:

```
DISARMED        -> false   // mặt đất là POSCTL/MANUAL ở MỌI chuyến bay; chốt ở đây = không bao giờ engage
ARMED + tay lái -> true
ngược lại       -> giữ nguyên   // KHÔNG rơi chốt khi nav_state rời chế độ tay -- đó đúng lúc auto_engage sẽ cắn
```

Ghi chú của chính hàm cũ nói *"Never take control from a landing, RTL, or failsafe"* — thiếu đúng chữ
**pilot**. **8 test mới** (3 predicate + 5 chốt). **M5 3/3, RTF 1,000** — cũng là bằng chứng chốt không
chặn engage bình thường, rủi ro lớn nhất của luật này.

🔴 **Vòng kiểm SITL CHẶN, không phải vì lỗi:** rig **không tạo được** đồng thời *"đã arm"* và *"PX4 ở chế
độ tay"* — armed thì PX4 từ chối POSCTL; PX4 nhận POSCTL thì arm bị từ chối (*"Resolve system health
failures first"*). RC override **theo định nghĩa** là hành động của tay điều khiển, và không có radio thì
không dựng lại được: **cùng cổng chặn với P11.5**. Giả thuyết *"PX4 từ chối vì thiếu RC"* **đã bị bác bỏ
bằng đo** (disarmed thì POSCTL được nhận). Cổng `scripts/verify_pilot_override.sh` giữ trong cây, chạy khi
có radio. ⚠️ **Giới hạn đã biết của chốt:** nó đóng khi *thấy* chế độ tay trong `/fmu/out/vehicle_status`,
mà topic đó phát **~2 Hz** — một cú cướp quyền ngắn hơn một mẫu sẽ vô hình. RC override thật giữ máy bay
lâu hơn 500 ms rất nhiều, nên đây là **mép đã ghi**, không phải mối nguy sống; là thứ đầu tiên phải kiểm
khi có phần cứng.

| Node | Việc |
|---|---|
| `px4_state_adapter_node` | `/fmu/out/*` → `/state/vehicle`, `/state/odometry_raw`, `/state/health_px4`, `/state/gnss_fix`, `/state/gnss_origin` |
| `px4_frame_bridge_node` | Phát TF `odom → base_link` |
| `px4_command_gateway_node` | `ControlCommand` → `/fmu/in/*`; service `arm`/`disarm`/`set_mode` |
| `offboard_session_manager_node` | Giữ nhịp ≥2Hz, priming, bật offboard an toàn |
| `px4_external_odometry_node` | Đẩy VIO/mocap vào bộ ước lượng PX4 |

### Hai thư viện dùng chung

| Thư viện | Chứa gì | Vì sao tách |
|---|---|---|
| `frame_conversions` | Phép đổi ENU/FLU ↔ NED/FRD | Cố ý **không** phụ thuộc `px4_msgs` |
| `px4_interop` | Bảng mode PX4↔nội bộ, hằng số main/sub mode, `autopilotOwnsTheFlight`, `allFinite`, timestamp micro-giây, dựng `VehicleCommand` | Trước đây **rải rác trùng lặp ở 4 node** |

**Vì sao `px4_interop` ra đời (2026-08-10):** hàm `toInternalFlightMode()` từng tồn tại **hai bản y hệt** ở state adapter và command gateway. Đó đúng là loại bảng đã gây ra lỗi `RETURN` → ACRO: sửa một bản quên bản kia thì **trạng thái báo về và lệnh gửi đi nói hai chuyện khác nhau**. Nay một nhà, có **13 unit test** ghim từng con số theo `px4_custom_mode.h`, gồm một test tên thẳng `ReturnIsNotAcro`.

### Thư viện `frame_conversions` — lệch thiết kế gốc (đã duyệt, R19)

`CLAUDE.md` thiết kế `px4_frame_bridge_node` là node làm việc chuyển frame. Thực tế:
- Phép chuyển nằm ở **thư viện gọi trực tiếp trong process** — tránh thêm chặng truyền tin trên đường lệnh sát an toàn. Logic vẫn ở **đúng một chỗ** → giữ tinh thần R5.
- Node chỉ lo **phát TF**.
- Phép toán **tái sử dụng `px4_ros_com::frame_transforms`** (chính thức của PX4), không tự viết ma trận quay.
- **14 unit test** chống *gọi sai chiều* — vì phép biến đổi là involution, trình biên dịch không thể bắt lỗi này.

### Quy tắc vận hành quan trọng

- **Trường ngoài `control_mode` phải điền `NaN`** — PX4 hiểu NaN = "không điều khiển trục này". Điền 0 nghĩa là "về gốc toạ độ", hoàn toàn khác.
- **Lệnh cũ → ngừng phát.** Nhường failsafe của PX4 thay vì bám một mục tiêu không còn ai giám sát.
- **Mọi service chờ autopilot xác nhận**, không tin lệnh đã gửi là đã có hiệu lực.
- **Không giành quyền khi autopilot đang cầm lái** (`AUTO_LAND`, `AUTO_RTL`, `AUTO_TAKEOFF`, `AUTO_MISSION`, `PRECLAND`, `DESCEND`, `TERMINATION`, hoặc failsafe).
- **Mỗi lần vào offboard đều priming lại** — bộ đếm reset khi rời `ACTIVE`.
- **`OffboardControlMode` và `TrajectorySetpoint` phải đi thành CẶP.** Công bố chế độ mà không kèm setpoint = PX4 vào offboard nhưng **không có gì để bám** → trượt vào failsafe. Gateway phát cả hai trong cùng một nhịp timer, hoặc không phát gì cả.
- **`/state/vehicle` phát theo TIMER, không theo lúc nhận tin PX4.** Nhờ vậy **mất kết nối PX4 tự nó là một bản tin** (`connected=false`). Nếu phát theo lúc nhận thì mất PX4 = mất luôn topic, và hạ nguồn không phân biệt được với "node đã chết".
- **Mốc thời gian gửi vào PX4 phải quy đổi qua `px4::Px4ClockOffset`** — hai đồng hồ có thể lệch ~1,79 tỉ giây. EKF2 xét vision **theo dấu thời gian** và **vứt im lặng** mẫu ngoài bộ đệm; lệnh offboard thì không dính vì PX4 xét chúng **theo lúc nhận** — nên lỗi này ẩn được rất lâu. Bộ ước lượng lấy **giá trị lệch LỚN NHẤT** trong cửa sổ 2 s vì trễ đường truyền chỉ có thể làm số đo nhỏ đi. Số đo thật → §5 "Lỗi thật đã sửa: mốc thời gian gửi vào PX4".
- **`px4_external_odometry_node` ăn thẳng nguồn vision, KHÔNG ăn `odometry_fused`** — `odometry_fused` dựng một phần từ chính EKF2; trả nó lại là đóng vòng lặp tự xác nhận (§5).
- **Phương sai là số KHÔNG DẤU** → đổi ENU→NED cho covariance chỉ **hoán vị x↔y**, tuyệt đối không đảo dấu z. Và **phương sai 0 là lời khai "pose này không thể sai"** — không biết thì phải gửi `NaN`, không được gửi 0.
- 🔑 **Vào offboard TRƯỚC, arm SAU** (sửa 2026-08-10). Trước đó `readyToEngage()` đòi `armed_`, và điều đó **khoá chết chuyến bay trong nhà**: không arm được vì PX4 khởi động vào LOITER mà LOITER đòi **global position** (đo được `mode_req_global_position=56`, đúng bit LOITER), còn offboard thì đang chờ armed. Ngoài trời không lộ vì GPS thoả LOITER. Thứ tự mới cũng là thứ tự PX4 dùng trong ví dụ chính thức, và OFFBOARD chỉ cần local position.
- **Chế độ AUTO của PX4 cần cặp main+sub mode.** Chỉ điền main là chọn sai chế độ. Số lấy từ `PX4-Autopilot/src/modules/commander/px4_custom_mode.h`.

### 🔴 QoS với `/fmu/*` — sai một nét là KHÔNG NHẬN ĐƯỢC GÌ, mà không có lỗi

Mọi subscriber `/fmu/out/*` trong package dùng **`KeepLast(10)` + `BestEffort` + `TransientLocal`**.

**Vì sao phải đúng từng nét:** PX4 (qua uXRCE-DDS) phát **best effort**. Một subscription khai `Reliable` **không bao giờ khớp** với publisher đó — DDS lặng lẽ không nối, không ném lỗi, không cảnh báo. Triệu chứng nhìn từ ngoài y hệt "PX4 chưa chạy" hoặc "agent chưa lên": `ros2 topic list` **vẫn thấy topic**, `ros2 topic echo` **không ra gì**.

🪤 Khi nghi mất topic `/fmu/*`, **nghi QoS trước tiên**: `ros2 topic info -v <topic>` để xem QoS **cả hai đầu**, rồi `ros2 topic hz`. Đây là bẫy tốn thời gian nhất của lớp cầu nối, và nó không để lại dấu vết trong log.

Chiều `/fmu/in/*` (publisher của ta) dùng QoS mặc định (reliable, depth 10) — đã chạy thật xuyên suốt M4/M5.

### `px4_command_gateway_node` — mô hình đồng thời (ĐỌC TRƯỚC KHI SỬA NODE NÀY)

Node này là node duy nhất trong package chạy `MultiThreadedExecutor`, với **hai callback group** cố ý tách:

| Group | Chứa | Vì sao phải tách |
|---|---|---|
| `subscription_group_` (mutually exclusive) | sub `vehicle_status`, sub `command_selected`, **timer phát setpoint** | `latest_command_` bị subscription **ghi** và timer **đọc**. Cùng một group loại trừ lẫn nhau ⇒ hai callback không bao giờ chạy song song ⇒ **không cần khoá** |
| `service_group_` (mutually exclusive) | `arm` · `disarm` · `set_mode` | Ba service này **chờ autopilot xác nhận**, ngủ tới vài giây. Để chung group với timer là **chặn nhịp setpoint** → tụt dưới 2 Hz → **rớt offboard giữa lúc bay** |

⇒ Hệ quả bắt buộc: `armed_` và `nav_state_` **phải là `std::atomic`** vì chúng đi **qua ranh giới hai group**. Đổi group, gộp group, hay đổi executor mà không rà lại đúng hai điều trên là đưa data race vào đường lệnh sát an toàn.

### GNSS đi ra ngoài bằng kiểu ROS chuẩn (2026-08-10)

`uav_localization` cần GPS, nhưng đọc thẳng `/fmu/out/vehicle_gps_position` sẽ **vỡ R1**. Nên state adapter phơi hai topic:

| Topic | Kiểu | Nội dung |
|---|---|---|
| `/state/gnss_fix` | `sensor_msgs/NavSatFix` | lat/lon/alt + covariance từ `eph`/`epv` + `fix_type` đã quy đổi |
| `/state/gnss_origin` | `sensor_msgs/NavSatFix` | **gốc EKF2** (`ref_lat/lon/alt`), latched (transient_local) |

**Vì sao phải có topic gốc:** GPS cho lat/lon, mux cần mét. Muốn đổi thì phải có mốc. Nếu adapter tự chọn mốc riêng, nó nằm trong frame của riêng nó và mux sẽ thấy hai nguồn lệch nhau một hằng số **mãi mãi mà không nguồn nào sai**. Bám gốc EKF2 thì khi gốc đó reset, `odometry_raw` và GPS nhảy **cùng nhau** nên vẫn khớp. Đo thật: lệch so với ground truth Gazebo chỉ **(−0.025, +0.044) m**.

**`toNavSatStatus()` cố ý bảo thủ ở hai hàng:** `FIX_TYPE_2D` → `STATUS_NO_FIX` (fix 2D không có độ cao) và `FIX_TYPE_EXTRAPOLATED` → `STATUS_NO_FIX` (ngoại suy là dead-reckoning, không phải phép đo). Cả hai đều không phải vị trí được phép bay theo.

### 🔴 PX4 tự tiêm ground-truth vào EKF2 sau lưng ta (phát hiện & đóng 2026-08-16)

**Triệu chứng:** `odometry_fused − odometry_raw` khi drone đậu = `(+0,284 · +0,348 · +0,228) m`. Bật hồ sơ vision-primary thì **ngang neo tới 1 mm nhưng z đứng yên ở +0,206…+0,228 m**, đúng bằng `body_offset_z = 0,24`.

**Nguyên nhân gốc:** plugin `OdometryPublisher` trong `models/uav0_{nav,track,full}/model.sdf` phát **hai** topic Gazebo: `/model/<m>_0/odometry` (cầu của ta dùng) **và** `/model/<m>_0/odometry_with_covariance`. PX4 tự đăng ký cái thứ hai (`GZBridge.cpp:194`, callback `:632`) rồi **publish thẳng vào uORB `vehicle_visual_odometry`** (`GZBridge.cpp:703`). Tức là EKF2 nhận **hai luồng external vision**: của PX4 (gốc model, không có lever arm) và của `px4_external_odometry_node` (đã cộng `body_offset_z`), lệch nhau đúng 0,240 m. Bằng chứng: trong ulog `vehicle_visual_odometry` **xen kẽ đúng hai giá trị** `+0,0130` và `−0,2270` (NED); chạy sim **không có node ROS nào** thì topic vẫn chạy 33,4 Hz với **duy nhất** giá trị gốc-model.

**Hệ quả an toàn (R0):** mọi bài kiểm chứng "mất GPS / VIO suy giảm" của P4 đều **bị vô hiệu** — dù ta bịt nguồn định vị phía ROS, PX4 vẫn được nuôi bằng ground-truth hoàn hảo. Sim-to-real: drone thật không có luồng này.

**Cách chữa:** đổi tên topic hiệp phương sai trong 3 file SDF (`<odom_covariance_topic>`), giữ nguyên `/model/<m>_0/odometry` nên cầu ROS và mọi thứ hạ nguồn không đổi.

**Đo sau khi chữa** (`uav0_nav`, đậu, 25 s): `vehicle_visual_odometry` chỉ còn **một** giá trị · `EKF2_z − EV_z = +0,0029 m` · `fused − raw = (−0,0002 · +0,0006 · +0,0035) m` · `cs_baro_hgt` 98,9% · `Ready for takeoff!`.

> 🪤 `Preflight Fail: ekf2 missing data` **là thông báo lúc khởi động**, không phải lỗi chặn arm: `estimatorCheck.cpp:105` in nó khi chưa copy được `estimator_status`, tức "chờ bộ ước lượng khởi tạo". Nó xuất hiện ở **mọi** lần chạy, kể cả lần kết thúc bằng `Ready for takeoff!`. Đừng đọc nó là "không arm được".
>
> 🪤 **ECL_INFO/ECL_WARN của EKF2 KHÔNG ra `/tmp/px4.log`.** Lần chạy có 71 lần `reset_pos_to_vision` mà console không một dòng "EV position fusion". Muốn xem quyết định của EKF2 thì đọc ulog (`estimator_event_flags`, `estimator_status_flags`), không grep console.

### ✅ ĐÓNG 2026-08-25 (P11.0, cổng G-E1) — **NỢ NÀY LÀ HIỆN VẬT ĐO, KHÔNG PHẢI LỖI SẢN PHẨM**

🔴 **Kết luận: EKF2 KHÔNG khởi động lại EV aiding mỗi giây. Chưa bao giờ.** Con số "71 lần/70 s, chu kỳ
đúng 1,004 s" đếm **số BẢN TIN**, mà bản tin thì được **phát lại**, không phải sinh mới.

**Cơ chế** — `EKF2.cpp:1129` (v1.15.4):

```cpp
} else if ((_last_event_flags_publish != 0) && (timestamp >= _last_event_flags_publish + 1_s)) {
        // continue publishing periodically
        _estimator_event_flags_pub.update();     // ← phát LẠI bản tin cũ, y nguyên mọi bit
}
```

Khi **không** có sự kiện mới, EKF2 phát lại bản tin trước đó **mỗi giây, vĩnh viễn**, mọi bit latched còn
nguyên, chỉ đổi dấu thời gian. Chu kỳ **1,004 s** chính là nhịp `+ 1_s` cộng jitter — nó là thuộc tính của
**bộ phát log**, không phải của external vision.

**Đại lượng phân xử:** `information_event_changes` là **bộ đếm**, EKF2 chỉ tăng khi thật sự có sự kiện;
bản phát lại giữ nguyên giá trị cũ.

| Nhánh (`scripts/gate_e1_ev_reset.sh`) | Sự kiện thật | Tần suất | Bản tin mang bit | `cs_ev_pos` |
|---|---|---|---|---|
| **A** nominal — `uav0_nav` đậu, **đúng cấu hình gốc của nợ** | **3**, hết trong **9,0 s đầu** / 129,5 s | **0,023 /s** | 121 / 132 | 89,7% |
| **B** tiêm lỗi (treo/thả EV publisher 0,6 s / 1,0 s) | **113** | **0,859 /s** | 147 / 212 | 60,1% |

**Đối chứng độc lập — topic khác, code path khác** (`vehicle_local_position`, bộ đếm EKF2 tăng bên trong
đúng hàm mà nợ nói chạy 71 lần):

| | nhánh A | nhánh B |
|---|---|---|
| `xy_reset_counter` | **2** / 129,5 s | **112** / 131,9 s |
| `vxy_reset_counter` | **2** | **112** |

⇒ Khẳng định thứ hai của nợ — *"bộ đếm reset của `vehicle_odometry` tăng liên tục"* — **cũng sai**.

🔑 **Vì sao tin được nhánh A:** vì nhánh B chứng minh **cùng thiết bị đo, cùng rig, trả được phán quyết
ngược lại theo yêu cầu**. Tách biệt **37 lần**. Không có nhánh B thì 0,023 /s chỉ là một con số chưa ai
thấy nó biết nói "có".

🔑 **Mâu thuẫn đã nằm sẵn trong hồ sơ nợ mà không ai đọc ra:** chính nó ghi `cs_ev_{pos,hgt,vel,yaw}` =
**100%** ở trạng thái ổn định. **Nếu fusion thật sự tắt-bật mỗi giây thì các cờ đó không thể 100%.** Cờ
control-status là **trạng thái được lấy mẫu**, bit sự kiện là **latched** — nửa đáng tin là cờ.

🔑 **Cùng họ với ca `peakAcceleration()`** (CLAUDE.md §5): một trường **mô tả** đại lượng không phải là
**đại lượng**. Ở đây bit latched mô tả sự kiện; bộ đếm mới là sự kiện.

🔴 **Quét toàn bộ 391 ulog lịch sử dự án** (`scripts/sweep_ev_event_rates.py`): **không một log nào** đạt
dải "thật" (≥0,5 /s); max 0,376 /s; p50 0,028. Năm log thấp nhất là hiện vật bắt tận tay —
`events=0, latched_msgs=22, msgs=22`: **0 sự kiện thật, 22 bản tin đều nói "đã reset"** ⇒ đếm bản tin ra
đúng "22 reset / 21 s ≈ 1/s", **đúng con số và đúng nhịp mà nợ đã ghi**.

**KHÔNG sửa gì:** không tham số PX4, không hành vi publish của ta, không nguồn PX4 — đúng ranh giới chủ dự
án ký 2026-08-25. Thứ được sửa là **hồ sơ**.

<details><summary>Hồ sơ cũ của nợ (giữ để truy nguyên)</summary>

### 🟠 (CŨ) Nợ mở: EKF2 khởi động lại toàn bộ EV aiding đúng 1 lần/giây

🪤 **Đọc DẤU THỜI GIAN của cờ, đừng đọc phần trăm.** `cs_ev_pos` "80,2% true" và `cs_fake_pos` "17,6%" **không phải rớt lúc chạy** — `estimator_status_flags` phát đều 1,00 s và trải cả log, nên phần trăm chỉ nói: EV tắt trong **cửa sổ 8,2→22,3 s trước khi stack ROS lên**. `fake_pos` chỉ fuse t=9,9→22,3 s rồi thôi. Ở trạng thái ổn định (log 14_48_54): **`cs_ev_{pos,hgt,vel,yaw}` = 100%, `cs_baro_hgt` = 100%, `cs_fake_pos` = 0%.** Tôi đã báo động nhầm một lần vì đọc phần trăm — công cụ `tools/probe_estimator_aiding.py` nay in luôn cửa sổ thời gian để không lặp lại.

**Tật thật còn lại:** `starting_vision_{pos,vel,yaw}_fusion` + `reset_{pos,vel}_to_vision` + `reset_hgt_to_ev` cùng bắn **71 lần/70 s, chu kỳ đúng 1,004 s** ⇒ cả bốn nguồn EV tắt rồi bật lại mỗi giây. Hệ quả: trạng thái + hiệp phương sai vị trí/vận tốc/độ cao bị khởi tạo lại 1 Hz, bộ đếm reset của `vehicle_odometry` tăng liên tục, và trong lúc BAY thì vận tốc bị ép về giá trị EV mỗi giây.

**Đã loại bằng số đo, đừng thử lại:** cổng innovation (`ev_hpos/ev_vpos/ev_hvel/ev_vvel` test ratio median 0,000, max 0,876 < 1) · variance NaN (EKF2 thay bằng `EKF2_EVP_NOISE`; đo `position_variance[2]` = 0,0100 hữu hạn) · đứt luồng EV (`vision_data_stopped` = **0**; khoảng cách mẫu p99 44 ms; mốc thời gian lệch −4…−8 ms so với đồng hồ EKF2, chỉ 2 lần nhảy lùi) · vận tốc giả 0 (đậu nên truth cũng 0; innovation ≤ 6 mm/s).

**"EV data too fast" cũng đã bị loại — hai lần độc lập.** (a) Số học: `EKF2_PREDICT_US` 10000 + `EKF2_DELAY_MAX` 200 ⇒ `_imu_buffer_length` = 20, `_obs_buffer_length` = min(round(300/10), 20) = **20** ⇒ `_min_obs_interval_us` ≈ 200000/19 ≈ **10,5 ms**, trong khi mẫu của ta cách nhau 28 ms. (b) Thực nghiệm: hạ nhịp EV **33 Hz → 10 Hz** (`max_publish_rate_hz`) thì nhịp reset **không đổi** — 71 lần/70,3 s so với 85 lần/84,1 s, đều đúng 1,00 s. ⇒ tật **không phụ thuộc nhịp phát**. Đã hoàn nguyên về 0 (không throttle); VIO thật chạy 20–30 Hz nên hạ nhịp chỉ mất dữ liệu chứ không được gì.

**Vướng ở đâu:** cần `estimator_aid_src_ev_pos.{fused,time_last_fuse}` để biết nhánh nào, mà EKF2 advertise topic đó **muộn** nên logger không bắt được — thử `px4-logger stop/start` sau khi EV đã lên vẫn không có. Hướng kế còn lại: replay ulog qua `ekf2`, hoặc cho bay có giám sát và đo ảnh hưởng thật (reset vận tốc 1 Hz lúc bay).

🔑 **Hậu xét 2026-08-25:** cả hai hướng trên đều **không cần thiết**, và cái "vướng" ở trên là dấu hiệu
đáng lẽ phải đọc được sớm hơn — thứ chứng minh 71 sự kiện lại **không tìm thấy ở đâu trong ulog**, vì
71 sự kiện đó không tồn tại. Replay được chuẩn bị (module `src/modules/replay/ReplayEkf2.cpp` có sẵn,
`make px4_sitl_default replay=<ulog>` chạy được) nhưng **không phải chạy**: một bộ đếm đã có sẵn trong
chính bản tin đang đếm sai trả lời xong câu hỏi trong một lượt đọc log.

</details>

### Phân biệt hai tình huống mất offboard

| Tình huống | Trạng thái báo | Ý nghĩa |
|---|---|---|
| Autopilot **chủ động** tiếp quản (hạ cánh, về nhà) | `STREAMING` | Bình thường |
| Mất offboard **ngoài ý muốn** | `FAULT` | Bất thường, cần phản ứng |

Gộp chung sẽ khiến `uav_safety` (P8) kích hoạt khôi phục giữa mỗi lần hạ cánh êm.

### 🔴 Hợp đồng battery NaN + bẫy sentinel PX4 (B2-backend, 2026-08-21)

- **`toBatteryHealth()` là hàm thuần trong `px4_interop`** (có test hợp đồng riêng — tiêm lại lỗi cũ thì 3/5 test đỏ): `battery_remaining` là **NaN** khi (a) chưa nhận `battery_status` nào, VÀ (b) khi PX4 gửi **sentinel "unknown"**. Consumer (safety) phải kiểm NaN + tuổi dữ liệu (R30), không được đọc mặc định.
- 🪤 **Sentinel "unknown" của PX4 `BatteryStatus` KHÔNG phải NaN:** `remaining = -1` · `voltage_filtered_v = 0`. Code cũ đọc `-1 < 0,10` thành CRITICAL giả **bất cứ lúc nào PX4 mất ước lượng SOC** (không riêng lúc boot). Nay lọc ngoài `[0,1]` → NaN. ⚠️ `current = 0` giữ **pass-through** (0 A hợp lệ khi chưa arm) — quy ước cần chốt lại nếu về sau có consumer dùng dòng điện để suy hao pin.
- **`overall_level = LEVEL_UNKNOWN` khi `failsafe_flags` không về mà `vehicle_status` về** — các cờ bool `imu/gps/baro/mag_ok` mặc định `true` là ngữ nghĩa không đổi được ở backend (bool không có "chưa biết"); backend phát mức tổng trung thực, **safety phải gate các cờ sensor bằng `overall_level`** (đã làm ở addendum checkpoint 2026-08-21: `LEVEL_UNKNOWN` → CANNOT_MEASURE). Hợp đồng NaN ghi thêm 1 dòng trong `VehicleHealth.msg`.

### ⚠️ Giả định & xấp xỉ đang có hiệu lực

- **`in_air` suy từ `takeoff_time > 0`** — PX4 không phơi `vehicle_land_detected`. **Sai sau khi hạ cánh.** *(P8 né nợ này bằng vị ngữ thay thế `armed && OFFBOARD && command_fresh` — plan P8 §2.)*
- **Cờ sức khoẻ cảm biến suy từ `failsafe_flags`** — đó là cờ *hợp lệ của bộ ước lượng*, không phải sức khoẻ cảm biến thô. `magnetometer_ok` mượn `attitude_invalid` làm đại diện.
- **Không làm** `/state/battery` và `/state/flight_mode` riêng — đã nằm trong `VehicleHealth` và `VehicleState`; tách ra là hai nguồn cho cùng một dữ liệu.
- ⚠️ **`z` của `odometry_raw` phụ thuộc hồ sơ EKF2 (cập nhật 2026-08-16):** mặc định nó là độ cao so với **gốc EKF2** (đặt lúc khởi tạo), **không phải so với mặt đất** — đo 2026-08-10: `base_link` cao 0.227 m, PX4 báo 0.1135 m, ta báo 0.1138 m, rangefinder 0.165 m khớp hình học tới 2 mm. **Nhưng trên hồ sơ vision-primary** (airframe `4102`, `EKF2_HGT_REF 3` + `EKF2_GPS_CTRL 8`) EKF2 neo cả ba trục vào nguồn vision, nên `odometry_raw` mang **đúng datum của `odometry_fused`** — xem mục dưới.
- **Chế độ `MODE_ATTITUDE` và `MODE_BODY_RATE` chưa hiện thực.** `ControlCommand` có khai báo chúng nhưng gateway **từ chối tường minh** thay vì im lặng. *(Trước 2026-08-10 nó công bố chế độ với PX4 rồi không gửi setpoint nào — đủ để PX4 vào offboard mà không có gì để bám.)*
- ✅ **Đã sửa (2026-08-10):** dịch vụ chờ xác nhận nay ngủ bằng `get_clock()->sleep_for()`, và cả 3 timer chạy theo đồng hồ ROS thay vì đồng hồ thực. `use_sim_time` đã bật — xem [`../src/uav_bringup/README.md`](../src/uav_bringup/README.md) mục "Ba điều phải biết".

---

## 3. `uav_bringup` ✅

Chi tiết cách dùng, phân biệt `sim.launch.py` vs `real.launch.py`, ngưỡng của bài smoke-test → [`../src/uav_bringup/README.md`](../src/uav_bringup/README.md).

Tóm tắt: `sim.launch.py` khởi động 5 node backend; `test/smoke_flight.py` chạy `arm→takeoff→goto→land→disarm` với mã thoát 0/1.

### 🔴 M5 siết hai chỗ, 2026-08-25 (P12.6) — cả hai đều từ một chuyến bay hỏng thật

**(a) Arm chờ được, và thời gian chờ HIỆN RA.** Chuyến 2/3 của lượt M5 ngày 25-08 báo
`arm rejected: autopilot did not arm within timeout`. Mở hộp đen PX4 (`07_32_24.ulg`, `pyulog`) thì
**không phải flake**:

| Đo được | |
|---|---|
| `reset_count_quat` | 0 → 2, lần hai tại **t = 49,2 s — đang armed, giữa chuyến** |
| `estimator_status.mag_test_ratio` | **phi hữu hạn** (9 mẫu) tại đúng **t = 49,2–49,3 s** |
| Luật chặn của PX4 | `!isArmed() && mag_test_ratio > COM_ARM_EKF_YAW (0,5)` ⇒ `inf > 0,5` |
| Yaw **thật** so với ground truth Gazebo | lệch tối đa **1,67°**, 15 s cuối tối đa **1,46°** |
| `pre_flt_fail_innov_heading` | **0/8127 mẫu** — chưa từng bật |

⇒ PX4 chặn arm vì **tỉ số đổi mới la bàn nổ số**, không phải vì yaw sai. Máy bay thật sự chưa sẵn sàng, nên
bỏ cuộc sau **một** lần gọi là khuyết tật của **bài test**, không phải của máy bay. `armWithinBudget()` chờ
trong ngân sách 120 s, và báo cáo in `arm=%.1f s over %d try(s)` mỗi chuyến — **một chuyến cần một phút để
arm là một phát hiện**, không phải thứ được hấp thụ im lặng. 🔴 **KHÔNG đụng `COM_ARM_EKF_YAW`.**
`px4_command_gateway_node` không cần sửa: nó vốn đã chờ `armed_` thật rồi mới trả lời, không bao giờ giả định.

**(b) Mã an toàn chưa ai viết lý do thì CHẶN một PASS.** Trước đó `safetyUnexplainedCodes()` chỉ in
`⚠️ UNEXPLAINED` rồi cho đỗ — và đó chính là kẽ hở giữ `LOCALIZATION_JUMP` sống sót nhiều ngày (§5).
Nay `safetyBlockers()` gộp cả hai điều kiện; ghim bằng 3 case mới trong
`test_smoke_flight_safety_verdict.py` (**9 → 12 case**), gồm cả chiều ngược: một mã **đã có** lý do thì
**không** chặn, để luật mới không thành cái cớ từ chối mọi chuyến bay.

---

## 4. `uav_sim_gz` ✅

> **🆕 2026-08-26 — `uav0_full_rgbd` không còn là model đo, nó đã được đấu dây.**
> `bridge_uav0_full_rgbd.yaml` + một mục trong `MODEL_IMAGE_TOPICS`. Tên topic ROS **giống hệt**
> `uav0_full` (R7: trên bridge không ai được phân biệt nổi). Tên topic gz đã kiểm **thật** bằng một
> world tối giản, không cần PX4: `rgbd_camera` phát `/uav0/rgbd_front/{image, depth_image,
> camera_info, points}` — `points` **cố ý không bridge**, cùng lý do đã ghi ở §3.
> **Nghiệm thu trước khi chuyển cổng D3 sang nó:** camera health OK cả 3 luồng, **0 mất khung**, khe
> dài nhất 0,04–0,07 s, và **ERROR cả 3** khi giết bridge ảnh · hình học vật cản ở 3 m lệch **≤ 0,003 m**
> trên khoảng cách/rộng/cao, tin cậy 1,00 trên 221 khung.
> 🔴 **Nhưng lý do gộp đã bị đo bác một nửa:** *"gộp xoá sạch đuôi RTF"* chỉ đúng khi drone **ĐẬU**.
> Khi bay, field-mean 0,907 (gộp) so 0,931 (không gộp) — nằm trong dải cũ, **không cải thiện đo được**.
> Thứ mở được D3 là **phép đo** (xem `memory.md` §7 mục 23), không phải cảm biến.
> ⚠️ Banner của `sensor_rgbd_front` từng ghi *"tầm xa front RGB là chỗ nhận marker"* — **SAI, đã sửa**:
> marker detector mặc định camera **`down`** (far 100, không đụng), obstacle extractor đọc front
> **depth** (đã clip 19,1 m), và người đọc pixel front RGB duy nhất là HOG người-đi-bộ trong một thế
> giới **không có model người**.

Cách dùng, thang model, danh sách topic bridge, và **8 bẫy đã trả giá** → [`../src/uav_sim_gz/README.md`](../src/uav_sim_gz/README.md).
Nguồn từng con số + yếu tố không mô hình + biên an toàn → [`../src/uav_sim_gz/docs/model-sources.md`](../src/uav_sim_gz/docs/model-sources.md).

Tóm tắt: world `uav_arena` (kế thừa nguyên khối physics của PX4) + thang model 4 tầng `uav0_frame → mô-đun cảm biến → uav0 / uav0_nav / uav0_full` + cầu Gazebo→ROS. Hồi quy M5 **PASS 3/3 trên cả `uav0` và `uav0_nav`**, RTF 1.000.

### Năm world — dùng cái nào (chốt 2026-08-13)

| World | Vai trò |
|---|---|
| **`uav_arena`** | ✅ **Mặc định** của `start_sim.sh`. 12 kB, RTF 0,999, M5 PASS 3/3 — nhẹ, đủ dùng, đã kiểm chứng |
| `uav_arena_indoor` | Bay không GPS / không la bàn (đường cơ sở của P4.8) |
| `uav_arena_outdoor` | Đường cơ sở hồi quy |
| `uav_arena_vn` | Sinh từ OSM ở toạ độ Bách Khoa — ⏸ tạm gác |
| `uav_arena_bk` | Sinh từ ảnh khảo sát thật — ⏸ **đóng & cất**, vẫn bay được |

### Hai nhánh world đã đóng lại — đọc trước khi định dựng world mới

- **`uav_arena_vn` + bộ sinh OSM** (`tools/fetch_osm_features.py` + `tools/vn_world_gen.py`) — sinh world cho **bất kỳ toạ độ nào**: nhà theo footprint thật (SDF `<polyline>`), đường, mảng nền, cây tổng hợp, + manifest ground-truth. Bay ở toạ độ Bách Khoa: RTF 0,982, PASS 3/3. 🔴 **Không đáng tin: 93% chiều cao là bịa** (cả khuôn viên chỉ **1/1271** nhà có `height` trong OSM). Bộ sinh vẫn chạy tốt, giữ vai trò phủ sóng nơi khác.
- **`uav_arena_bk` + 9 công cụ đo** (`tools/bk_world_gen.py`, `measure_buildings.py`, `extract_trees.py`, `check_world_invariants.py`…) — dựng từ **997 ảnh khảo sát thật**: 40 nhà + 459 cây, **499 nhãn**, collision ⊇ visual 959/959, M5 PASS 3/3. 🔴 **Đóng & cất 2026-08-13** — không phải vì hỏng, mà vì **độ thật thị giác** không đủ cho nhận dạng DNN, do dữ liệu **100% nadir**. Bài học đầy đủ: [`lesson-bk-world-attempt.md`](lesson-bk-world-attempt.md).
- 🔑 **Đo được và phải nhớ:** chiều cao nhà dựng từ bộ ảnh sai **±1–1,5 m**, KHÔNG phải ±0,2 m (cổng G1‴ trượt: bias −1,15 m + scatter 0,92 m). Biên an toàn đặt ở **inflation của planner**, không nướng vào world.

**Phân vai từ nay:** Gazebo lo **ROS2 / framework / logic điều khiển**; thế giới 3D độ thật cao thì nghiên cứu **nền Unreal (Cosys-AirSim)** riêng, chưa bắt đầu. Chi tiết: [`sim-fidelity-decision.md`](sim-fidelity-decision.md).

### 🔴 Quy tắc vận hành: chỉ tin số đo bay khi RTF ≥ ~0,95

Thí nghiệm một biến (`scripts/compare_flight_accuracy.sh`, cùng model, chỉ đổi world): RTF 0,986 và RTF 0,999 cho sai số ngang **không phân biệt được** (0,02–0,14 m); còn cấu hình RTF **0,17–0,78** cho **0,32–0,37 m** — xấu hơn 3×.

**Vì sao:** vật lý và cảm biến theo sim time nên không bỏ bước, **nhưng vòng điều khiển thì có** — chặng DDS/uXRCE-DDS giữa ROS và PX4 chạy theo **đồng hồ THẬT**, nên RTF tụt biến một độ trễ cố định thành độ trễ **lớn hơn tính theo giờ mô phỏng**. → Hồi quy M5 dùng `uav0`; bài cần lidar (`uav0_nav`) thì kết quả bám điểm phải **dán nhãn "đo dưới RTF thấp"**.

### 🔬 Đuôi RTF — CHẨN ĐOÁN XONG 2026-08-25 (P12.5): không có "khựng sâu", chỉ có chi phí TUẦN HOÀN

`scripts/diagnose_rtf_stalls.sh` + `scripts/rtf_stall_verdict.py`. Hai nhánh mỗi model (**A** sim trần ·
**B** sim + đủ stack + `perception:=true`), đọc `/world/uav_arena/stats` và suy RTF từ `sim_time`/`real_time`
**trong chính bản tin** thay vì tin trường báo sẵn.

| model | cảm biến render | chu kỳ trùng tick | A: quãng dưới 0,95× | A hụt | B: quãng dưới 0,95× | B hụt | khoảng cách (A) |
|---|---|---|---|---|---|---|---|
| `uav0` | **không có** | — | **0 / 584 (0,0%)** | **0,00 s** | **0 / 585 (0,0%)** | **0,00 s** | — |
| `uav0_track` | camera_front 15 · depth_front 15 | 66,7 ms | **0 / 585 (0,0%)** | **0,00 s** | 1 / 585 (0,2%) | 0,01 s | — |
| `uav0_nav` | lidar_down 20 · camera_down 30 | 100 ms | 5 / 585 (0,9%) | 0,03 s | 8 / 586 (1,4%) | 0,05 s | p50 **6,55 s** |
| **`uav0_full`** | **cả bốn** | **200 ms** | **215 / 877 (24,5%)** | **1,98 s** | **290 / 877 (33,1%)** | **3,14 s** | p50 **0,21 s** |

*(A = sim trần · B = sim + đủ stack + `perception:=true`. 60 s/nhánh cho ba model đầu, 90 s cho `uav0_full`.)*

**Bốn kết luận, cả bốn là số đo:**

1. 🔴 **Không có cú khựng nào.** Bản tin stats về đều như máy đếm nhịp — `uav0` p50 = p95 = **max = 0,104 s**;
   `uav0_full` p50 0,103 s **max 0,122 s**. Không có gì bị chặn lâu, chưa bao giờ.
2. 🔴 **`real_time_factor` là đại lượng của MỘT BƯỚC.** Cùng bản tin báo min **0,151** trong khi RTF suy từ
   hai đồng hồ của chính nó là **0,748** — chênh ~5×. Mọi mô tả *"đuôi khựng sâu hiếm"* dựng trên `min` của
   trường đó, và **mô tả đó sai**. Nó đã dẫn P12.5 đi bác nhầm một giả thuyết đúng.
3. 🔴 **Thiếu hụt quy trọn về render cảm biến, và nó TUẦN HOÀN.** Khoảng cách p50 **0,21 s** trên `uav0_full`
   khớp số học 4 cảm biến trùng tick mỗi **200 ms**; `uav0_nav` (chu kỳ trùng 100 ms = đúng bằng cửa sổ lấy
   mẫu) không có chu kỳ nào nhìn thấy được, khoảng cách p50 **6,55 s**. Stack ROS thêm ~0,03 RTF nhưng
   **không tạo cơ chế**: nhánh B trên `uav0` hụt đúng **0,00 s** dù chạy đủ node và perception.
4. 🔴 **Chi phí là SIÊU CỘNG, và đó mới là đòn bẩy.** Từng cặp gần như miễn phí — `uav0_track` **0,0%**,
   `uav0_nav` **0,9%** — mà cả bốn cho **24,5%**, tức **~27× tổng các phần**. ⇒ **Không cảm biến nào là thủ
   phạm.** Đại lượng quyết định là **số cảm biến render TRONG CÙNG MỘT TICK**, và chi phí theo nó là phi
   tuyến.

⇒ **Hai đòn bẩy, cả hai tác động lên đúng đại lượng đó**, và cả hai đều **chưa chạy**:
gộp `camera_front` + `depth_front` (cùng 15 Hz, **cùng pose**) thành một `rgbd_camera` đưa tick nặng nhất
từ 4 xuống **3** lượt render; **lệch pha** đưa nó xuống **1**. Lệch pha mạnh hơn hẳn, nhưng SDF không có núm
pha nên phải tìm cách khác (đổi `update_rate` là **đổi điều kiện đo** — R31/B-19, không được phép).
⚠️ **Không dự đoán con số RTF cho phương án nào** khi chưa đo — đúng cái lỗi đã làm hỏng vòng chẩn đoán trước.

### ✅ Đã đo lại RTF trên RTX (2026-08-13) — nỗi lo chuyển chỗ, không biến mất

`scripts/measure_rtf_models.sh`, world `uav_arena`, 150 mẫu tức thời/biến thể, renderer xác nhận là **RTX 5060**:

| Model | Cảm biến render | **p50** qua các lần chạy | min–max quan sát được |
|---|---|---|---|
| `uav0` | 0 | **1,000** (1 lần) | 0,965–1,029 |
| `uav0_nav` | 2 | **1,000 · 1,000** (2 lần) | 0,942–1,187 |
| `uav0_full` | 8 | **0,626 · 0,322 · 0,428** (3 lần) | 0,128–5,522 |

- ✅ **Con số 0,762 của `uav0_nav` là do render bằng CPU, đã bác bỏ.** Trên RTX nó là 1,000, **lặp lại được ở hai lần chạy độc lập** — README §7 (08-10) đúng. Hồi quy M5 và G2 chạy trên `uav0_nav` là **hợp lệ**.
- 🟠 **`uav0_full` — model P5 cần — ban đầu sụp còn mean 0,50–0,67.** Đã truy nguyên và cải thiện lên **mean 0,79–0,86** (xem mục dưới), vẫn **chưa qua 0,95**.
- ⚠️ **Còn mở — phép đo trung bình theo thời gian:** các số trên là **mẫu tức thời**. Đo trung bình (sim-time/wall-time trên cửa sổ 60 s) **thất bại 2 lần** trong `measure_rtf_models.sh` — `ros2 topic echo /clock --once` không trả gì trong ngữ cảnh script. Lần đầu còn **im lặng in ra 0.000** do `awk` nhận một dòng rỗng; đã sửa để báo `unusable` thay vì bịa số. **Không thử lần 3 (R10).** ✅ **Đã có cách khác chạy được:** `g2_fused_accuracy.py` tự đo tỉ lệ này **từ bên trong node** (`sim_elapsed/wall_elapsed`) và cho số tin cậy — dùng đường đó khi cần.

### 🔬 Truy nguyên RTF của `uav0_full` (2026-08-13) — bốn giả thuyết, ba cái SAI

Công cụ: [`tools/probe_sensor_cost.py`](../src/uav_sim_gz/tools/probe_sensor_cost.py) (cảm biến đơn lẻ, Gazebo thuần) · [`scripts/measure_bridge_cost.sh`](../scripts/measure_bridge_cost.sh) (cắt lớp cầu ROS) · [`scripts/verify_uav0_full.sh`](../scripts/verify_uav0_full.sh) (nghiệm thu).

| Giả thuyết | Phán quyết |
|---|---|
| "Độ phân giải RGB 1080p là chi phí" | ❌ **SAI.** RGB 1920×1080@30 **một mình** = mean 0,993 vs nền 0,992 — gần như miễn phí |
| "Point cloud là luồng nặng nhất nên là thủ phạm" | ❌ **SAI.** Bỏ cloud khỏi cầu: 0,821 → 0,807, **không đổi** |
| "Hạ nhịp sẽ cứu được" | ❌ **SAI.** Hạ 15 → 10 Hz cho **0,692, tệ hơn** |
| **"Nhiều cảm biến render tick cùng nhau gây tranh chấp"** | ✅ **ĐÚNG** |

🔑 **Bằng chứng chốt hạ — chi phí KHÔNG cộng theo pixel:**

| Probe | mean |
|---|---|
| RGB 1080p@30 một mình | **0,993** |
| Depth 640×480@30 một mình | 0,938 |
| RGB **480p** + depth 480p @30 cùng nhau | **0,997** |
| RGB **1080p** + depth @30 cùng nhau *(đúng cấu hình OakD-Lite)* | **0,701** |

Một camera lớn **một mình** không tốn gì; **cùng tick với depth** thì bước mô phỏng khựng. Gazebo render trên một luồng.

**Đã sửa:** `uav0_full` thôi kế thừa `OakD-Lite` của PX4 (1920×1080@30 — thông số của PX4, chưa từng là lựa chọn của ta), thay bằng hai mô-đun của dự án: [`sensor_camera_front`](../src/uav_sim_gz/models/sensor_camera_front/model.sdf) và [`sensor_depth_front`](../src/uav_sim_gz/models/sensor_depth_front/model.sdf), **cả hai 640×480 @ 15 Hz**. Kèm hai lợi ích phụ: topic đi đúng quy ước `uav0/camera_front/image` (bản cũ dùng `/camera` nên `camera_info` rơi ra **gốc namespace** — đúng bẫy #6 trong README), và mesh vẫn dùng lại của PX4 nên hình dáng không đổi.

**Kết quả:** mean **0,50–0,67 → 0,79–0,86** (3 lần chạy: 0,863 / 0,790 / 0,840), cảm biến **PASS 3/3** với dữ liệu thật.

🔴 **VẪN CHƯA QUA 0,95, và tinh chỉnh tiếp sẽ không tới.** Chi phí biên của hai cảm biến trước **trong hệ đầy đủ** là ~0,15 và không co theo độ phân giải/nhịp:

| Cấu hình | mean |
|---|---|
| `uav0_nav` (2 cảm biến render), stack thật | 0,999 |
| Probe (2 cảm biến y hệt, **không PX4/cầu**) | 0,997 |
| `uav0_full` (4 cảm biến), stack thật, đã tối giản | **0,79–0,86** |

→ Phần thiếu là **tranh chấp + tải CPU của cả hệ**, không phải khối lượng dữ liệu. Hai đường còn lại: **tách biến thể theo bài test** (đã là nếp của dự án: `uav0` cho M5, `uav0_nav` khi cần lidar) hoặc **dual-boot**.

### ✅ Nợ #10 ĐÓNG (P5.2, 2026-08-13) — ảnh mất 46–84%, và đã có địa chỉ

Đo bằng consumer **C++** ([`src/image_rate_probe.cpp`](../src/uav_sim_gz/src/image_rate_probe.cpp), chạy qua [`scripts/measure_image_rates.sh`](../scripts/measure_image_rates.sh)). Nó suy **tần số nguồn** từ khoảng cách **nhỏ nhất** giữa hai dấu thời gian, nên biết được cảm biến phát bao nhiêu **kể cả khi khung bị rớt** — không phụ thuộc consumer nhanh hay chậm.

| Luồng | Nguồn phát | Về tới ROS | Tỉ lệ | Mất | Đường đi |
|---|---|---|---|---|---|
| `front/image_raw` | 15,6 Hz | 6,0–8,4 | 38–54% | 139–241 | `ros_gz_image` |
| **`front/camera_info`** | 15,6 Hz | **15,2** | **97%** | **0** | `parameter_bridge` |
| `front/depth_image` | 15,6 Hz | 2,5–6,5 | 16–42% | 224–282 | `ros_gz_image` |
| `down/image_raw` | 31,3 Hz | 15,1–16,3 | 48–52% | 293–400 | `ros_gz_image` |
| **`down/camera_info`** | 31,3 Hz | **30,3** | **97%** | **0** | `parameter_bridge` |

🔑 **Đối chứng chốt hạ:** `camera_info` **cùng cảm biến, cùng nhịp** nhưng đi `parameter_bridge` → **mất 0 khung**. ⇒ Cảm biến phát đủ (nguồn suy ra 15,6/31,3 Hz đúng cấu hình), consumer Python **không** phải thủ phạm (C++ cũng thấy mất). Thủ phạm là **đường ảnh**.

🔑 **Cơ chế: TRẦN THÔNG LƯỢNG CHUNG, các luồng giành nhau — không phải một luồng hỏng.**

| Cấu hình | front RGB | depth | down RGB | **tổng bản tin/giây** | RTF mean |
|---|---|---|---|---|---|
| 1 node cầu, có cloud *(ban đầu)* | 6,2 | 2,5 | 15,1 | **23,8** | 0,706 |
| 3 node cầu, có cloud | 8,4 | 2,8 | 16,3 | **27,5** | 0,738 |
| 3 node cầu, bỏ cloud | 6,0 | 6,5 | 15,2 | **27,7** | 0,775 |
| ✅ **cấu hình chốt** (3 node, cloud bỏ hẳn) | 5,5 | **7,0** | **17,2** | **29,7** | **0,877** |

Tổng gần như bất biến ~**24–30 bản tin/giây**. Bỏ cloud **không tạo thêm dung lượng**, nó **chuyển** dung lượng sang depth (18% → 45%) và camera dưới.

**✅ Đã sửa (giữ lại):** `gz_bridge.launch.py` nay chạy **một node `ros_gz_image` cho mỗi luồng** thay vì một node gánh cả ba. Bằng chứng nó có tác dụng: khoảng cách trung vị của `front/image_raw` từ **132 ms (= 2× chu kỳ, cách một bỏ một)** về đúng **68 ms (= 1× chu kỳ)**, tỉ lệ 39% → 54%, RTF +0,03.

🔴 **RÀNG BUỘC THIẾT KẾ CHO P5 — ngân sách ảnh ~28 bản tin/giây, phải chia có chủ đích:**
- Camera **dưới** đang đặt 30 Hz và ăn **hơn nửa ngân sách một mình** (giao 15 Hz). Nó phục vụ optical flow của P4 → hạ nhịp phải kiểm chứng lại P4.5/G6, không tự ý sửa.
- ✅ **Point cloud đã BỎ khỏi cầu (chốt 2026-08-13).** Nó chở lại đúng thông tin của ảnh depth mà giành mất băng thông của chính ảnh đó. P5 **chiếu ngược trong ROS** từ `depth_image` + `camera_info` — cách drone thật làm. Kết quả: depth 18% → **45%**, RTF **0,706 → 0,877**.
- Đừng thiết kế perception dựa trên nhịp cấu hình trong SDF — **dựa trên nhịp GIAO ĐƯỢC**.

🔑 **RTF lúc ĐẬU khác RTF lúc BAY — đo được, phải nhớ:** `uav0_nav` đậu là 1,000, nhưng dưới **stack đầy đủ 11 node khi bay** (đo bởi chính bài G2) chỉ còn **0,906–0,953** — sát ngưỡng 0,95. Nghĩa là mọi con số RTF đo lúc đậu đều **lạc quan hơn thực tế**.

🔴 **Và "chạy lại trên máy rảnh" KHÔNG cứu được — đã đo, giả định bị bác (2026-08-19).** Chạy
`rerun_p4_gates.sh` trên máy **thật sự rảnh** (WSL vừa boot, 16 nhân, 21/23 GB trống, không việc nào
khác): indoor M5 mean **0,918** · G2 điều kiện A **0,828** · điều kiện B **0,877** ⇒ cả hai vẫn tự
trả `NOT TRUSTED` **lần thứ ba liên tiếp**. Chi phí là của **cả stack khi bay**, không phải tải ngoài.
🔑 **Hệ quả cho mọi phase sau: đừng lên lịch "chạy lại lúc máy rảnh".** Lý lẽ an toàn phải đặt ở cổng
**tất định** (mẫu `run_mux_fidelity.sh` — đo được, và còn bắt ra một lỗi thật cùng ngày); cổng **bay**
chỉ còn vai **xác nhận có dán nhãn**. Đây là căn cứ khiến G-N4 của P6.4 được chia hai tầng ngay từ
lúc thiết kế.

✅ **ĐÃ SỬA** (kiểm lại 2026-08-25): `src/uav_sim_gz/README.md` §7 nay gạch bỏ con số cũ "RTF 1.000 cho cả ba biến thể" và thay bằng hàng đo lại 2026-08-13 — `uav0` 1.000 · `uav0_nav` 0.999 · **`uav0_full` 0.79–0.86**, kèm cảnh báo dùng **mean chứ không p50**. Dòng "Cần sửa" cũ đã nằm lại ở đây sau khi việc đã xong; một ghi chú *cần-sửa* trỏ vào thứ đã sửa là rác tồn đọng — nó khiến người sau làm lại việc không cần làm. Cùng lý lẽ với cổng bắt entry allowlist mục nát trong `check_px4_msgs_boundary.sh`.

### 🏁 NGUYÊN NHÂN GỐC TÌM RA 2026-08-24 tối — **vách là SEGMENT SHARED-MEMORY của Fast DDS, không phải cảm biến**

Mọi `/dev/shm/fastrtps_*` đo được **đúng 549 408 B** (mặc định nguyên bản của thư viện). Ranh giới
giao/không-giao rơi **chính xác** vào đó:

| Bản tin | Byte | Lọt segment 549 408? | Tỉ lệ giao |
|---|---|---|---|
| `camera_info` (~300 B, qua `parameter_bridge`) | ~300 | ✅ | 97% |
| down 320×240 RGB8 | 230 400 | ✅ | **97%, mất 0** |
| depth 320×240 F32 | 307 200 | ✅ | **97%, mất 0** |
| down/front 640×480 RGB8 | 921 600 | ❌ | 31–53% |
| depth 640×480 F32 | 1 228 800 | ❌ | 25–30% |

**Cơ chế:** sample không lọt segment thì rớt sang UDP, bị cắt 14–19 datagram vào bộ đệm mặc định
`net.core.rmem_default` = 208 KB; mất **một** mảnh là mất **cả** khung. `net.core.rmem_max` đã là 4 MiB
⇒ nhân không chặn, chỉ là Fast DDS không xin thêm.

⇒ **Toàn bộ tranh luận A/B/D/E/F là chọn cách NÉ một cái vách dời được.** Không cấu hình nào trong số đó
là câu trả lời đúng — chúng chỉ là các kiểu hy sinh khác nhau.

**Bản vá:** [`src/uav_bringup/config/fastdds_large_samples.xml`](../src/uav_bringup/config/fastdds_large_samples.xml)
— segment SHM 16 MiB + bộ đệm UDP 4 MiB, **khai CẢ HAI transport** (SHM *và* UDPv4). 🔴 Cố ý **không**
SHM-only: `fastdds_shm_only.xml` là ngõ cụt đã đo, nó cắt sạch `/fmu/out/*`.

**Kết quả — cấu hình A NGUYÊN BẢN, không đụng một cảm biến nào** (`scripts/run_dds_transport_trial.sh`):

| Luồng | Trước | Sau |
|---|---|---|
| `front/depth_image` | 4,8 Hz (30%) · mất 594 · gap max **1256 ms** | **15,2 Hz (97%) · mất 0 · 68 ms** |
| `down/image_raw` | 15,9 (51%) · mất 828 · 464 ms | **30,3 (97%) · mất 0 · 36 ms** |
| `front/image_raw` | 5,3 (34%) · mất 585 · 988 ms | **15,2 (97%) · mất 0 · 68 ms** |
| TỔNG | 24,80 msg/s · 23,12 MiB/s | **57,33 msg/s · 54,55 MiB/s** (2,36×) |
| RTF mean | 0,889 | **0,963** |

Bằng chứng profile nạp thật: segment đo được **16 974 368 B** (16 MiB + metadata), port file 124 080 B
thay vì 52 400 B. 🔑 **97% là sàn của chính probe, không phải mất mát** — `camera_info` (300 byte, chưa
từng bị chết đói) cũng đọc đúng 97%, vì probe suy `source_hz` từ khoảng cách nhỏ nhất. **97% kèm `lost 0`
nghĩa là mọi khung đều tới.** Đừng đuổi theo 3% còn thiếu.

### ✅ NỢ #10 ĐÓNG 2026-08-24 — bốn cổng, rồi mới nối vào build

**Cổng R0** (`scripts/gate_r0_dds_profile.sh` + `scripts/gate_r0_verdict.py`) — câu hỏi: profile có làm
hỏng đường tới flight controller không? Hai nhánh **control (transport gốc) / profile**, cả hai chạy
`uav0_full` **dưới stack perception đầy đủ** (điều kiện sản xuất, cũng là điều kiện khắc nghiệt hơn).

| topic | control | profile | tỉ lệ |
|---|---|---|---|
| `vehicle_odometry` | 93,45 Hz | 89,55 Hz | **0,958** |
| `vehicle_status` | 1,85 Hz | 1,78 Hz | **0,962** |
| `sensor_combined` | 93,45 Hz | 89,64 Hz | **0,959** |

(chuẩn-hoá theo RTF; gap tệ nhất **tốt hơn** dưới profile ở cả ba: 0,682/1,347/0,678 s so với
0,702/1,378/0,700 s. RTF 0,761 → 0,791. Segment đo được 16 974 368 B vs 549 408 B.)

🔑 **Hai điều cổng này làm khác `measure_image_budget.sh`** — bản cũ chỉ chờ topic **xuất hiện**, mà
*"nhìn thấy được" ≠ "khoẻ"*, đúng chỗ bản SHM-only đã chết:
1. **Chuẩn-hoá theo RTF.** PX4 phát theo thời gian **sim**, nên ở RTF 0,89 vs 0,96 nhịp wall chênh 8% dù
   transport y hệt. So nhịp wall là **tính công cho profile phần RTF nó tình cờ cải thiện**.
2. **Cổng phép đo trước cổng đối tượng (R27-1).** Profile Fast DDS nạp lỗi thì **im lặng** ⇒ mỗi nhánh
   đọc lại segment thật sự được cấp; nhánh control phải là 549 408, nhánh profile phải ≥ 16 MiB. Không
   thoả ⇒ **FAILED TO MEASURE**, không kết luận.

🪤 **Nhánh đối chứng đã bắt được một sai trong chính cổng** — sàn tuyệt đối ban đầu đặt trên **nhịp wall**
(`vehicle_status ≥ 1,5 Hz`, lấy từ đường cơ sở M2 rig rảnh RTF 1,000). Control đọc **1,476 Hz** dù
transport hoàn toàn lành ⇒ **sàn đó đang đo RTF, không đo link**. Phát biểu lại trên nhịp chuẩn-hoá
(control xác nhận: 91,64 / 1,82) — tiền lệ G-S3-B1, **không phải nới cổng**: link đứt hay mất gói vẫn
kéo tụt nhịp chuẩn-hoá y hệt. **Gap tệ nhất giữ đơn vị wall** — khựng là khựng.

**Ba cổng sau, chạy dưới profile** (`scripts/run_no10_closeout.sh`, tự tái phán xử R0 trước khi chạy):

| Cổng | Kết quả |
|---|---|
| `verify_obstacle_extractor` (uav0_full) | **PASS** — 171 khung, hộp 1,000×0,600 m đo **0,997×0,594**, khoảng cách 2,817 vs 2,800 ⇒ sai ≤ **6 mm** |
| `verify_marker_detector` (uav0_nav) | **PASS** — h=2,5 sai +0,035 m (spread 0,072) · h=3,5 sai +0,070 (0,061) · confidence 1,00 |
| **M5 3/3 (R14, uav0)** | **PASS** — alt_err 0,06–0,21 m · horiz 0,04–0,09 m · **0 vi phạm** · RTF mean **0,996** |

🔑 **Đây là lý lẽ mạnh nhất cho việc sửa TRANSPORT thay vì hạ độ phân giải:** phương án hạ 320×240 từng
đo hộp 1,000×0,600 m thành **0,578×0,298 m** (trượt cổng hình học). Sửa transport giữ nguyên hình học.
🔑 **M5 cũng đóng chiều mà cổng R0 không nhìn thấy**: R0 đo `/fmu/out/*`; luồng setpoint offboard chạy
chiều ngược `/fmu/in/*`, và một chuyến arm→takeoff→goto→land hoàn tất 3 lần là bằng chứng chiều đó sống —
PX4 rơi khỏi offboard ngay khi luồng tụt dưới 2 Hz.

**Nối vào build (2026-08-24):** `start_sim.sh` **và** `sim.launch.py` cùng nạp profile từ
`install/uav_bringup/share/uav_bringup/config/`. Phải là **cả hai**: export trong `start_sim.sh` nằm ở
**shell con** nên chỉ tới được agent + Gazebo + bridge, không tới stack — mà cổng lại đo với profile áp
cho *mọi* tiến trình. Ship khác điều kiện đã đo là **R31**. Cửa thoát `UAV_DDS_PROFILE=none` để nhánh
control của cổng R0 còn chạy lại được — *cổng không chạy lại được là cổng mục*.

**Kiểm chứng nối dây + hệ quả dây chuyền** (`scripts/verify_no10_wiring.sh`, **không** export tay):
segment **16 974 368 B** ⇒ nối dây thật; và đèn go/no-go với `perception:=true` đọc
**`GO age=0,898s gate_mode=preflight(measured) blocking=''`**, `worst_item='state/odometry_fused'`.
🔑 **Đây là bằng chứng end-to-end không ai giả được**: `config/preflight_waivers.yaml` **cố ý không nạp**
3 hàng camera và tự ghi lý do — *"perception:=true đúng ra đọc NO_GO cho tới khi nợ #10 được vá"*. Đèn
xanh **với bảng waiver nguyên vẹn** nghĩa là khung ảnh thật sự tới, không phải thứ gì được miễn trừ.

⚠️ **Hệ quả cho mọi phép đo ngân sách ảnh về sau:** mặc định hôm nay là transport **LARGE**. Mọi số
A/B/C/D/E/F ghi **trước 2026-08-24** đo trên transport **gốc** và **không so sánh trực tiếp được**; muốn
tái lập phải đặt `UAV_DDS_PROFILE=none`. `measure_image_budget.sh` nay **luôn in segment + transport
đang hiệu lực** để chỗ này không thành so sánh táo-với-cam trong im lặng.

⚠️ **Chưa trả lời (chuyển sang P11):** profile này đang là **sim-only**, nhưng vách 549 408 B là mặc định
của **thư viện Fast DDS**, không phải của mô phỏng — **camera thật cũng phát ảnh 640×480+ qua chính nó**.
Xem `../.claude/plan/P11-real-flight-readiness.md` §3(a).

### 🔴 ĐO LẠI CÓ STACK PERCEPTION — 2026-08-24 chiều: kết luận "trần là số bản tin" CHỈ ĐÚNG TRÊN RIG RẢNH

`measure_image_budget.sh PERCEPTION=1`, cửa sổ 60 s, **mỗi lượt xác nhận kích thích trên dây** cho cả
down lẫn depth (MiB/msg phải khớp SDF **đã cài**). Bốn cấu hình, cùng một phiên:

| | front/image_raw | **front/depth_image** | down/image_raw | tổng msg/s | MiB/s | RTF mean |
|---|---|---|---|---|---|---|
| **A** đang ship | 5,3 (34%) | **4,8 (30%) · mất 594 · gap max 1256 ms** | 15,9 (51%) · mất 828 · 464 ms | 24,80 | 23,12 | 0,889 |
| **B** down 320×240 | 5,3 (34%) | 3,9 (25%) · mất 638 · 1252 ms | **30,3 (97%) · mất 0 · 36 ms** | 37,40 | 15,03 | **0,981** |
| **D** depth 320×240 | 5,3 (34%) | **15,2 (97%) · mất 0 · 68 ms** | 16,5 (53%) · 296 ms | 35,10 | 22,40 | 0,943 |
| **E** cả hai 320×240 | 6,7 (43%) | 15,2 (97%) · 68 ms | 30,3 (97%) · 36 ms | **50,27** | 16,39 | 0,960 |

🔑 **Quy luật: luồng nào được thu nhỏ thì luồng đó lên 97% / mất 0 khung.** Chi phí **mỗi bản tin** quyết
định luồng có qua cầu hay không. **Dưới stack KHÔNG có trần số-bản-tin** — tổng chạy từ 24,80 tới 50,27
tuỳ cấu hình. Kết luận buổi sáng cùng ngày (*"trần là số bản tin ~50, hạ nhịp là đòn duy nhất"*) đo trên
**rig rảnh** và **chỉ có hiệu lực ở đó** — đúng cảnh báo R31 ghi kèm số đo đó, nay đã cắn thật.

🔴 **Và phát hiện nặng nhất không nằm ở A/B:** `front/depth_image` — luồng nuôi **tránh vật cản** — chạy
**30% (4,8/15,6 Hz), mất 594 khung/phút, mù tệ nhất 1,256 s**. Không ai biết vì con số **7,0 Hz** ở bảng
trên đo **2026-08-13**, còn `obstacle_extractor_node` mãi **2026-08-16** mới có ⇒ 7,0 là nhịp depth **khi
chưa ai subscribe**, chưa từng là điểm thiết kế được kiểm chứng. Ngưỡng THẬT nằm trong code:
`map_timeout_sec` = `obstacle_timeout_sec` = **3,0 s** ⇒ 1,25 s là **42% ngân sách**, giống nhau ở A và B.

### 🔴 D BỊ LOẠI BẰNG CỔNG SẢN PHẨM — gom cụm vỡ ở nửa độ phân giải (`verify_depth_halfres_cost.sh`)

| depth | vật cản/khung | sai khoảng cách | sai rộng | sai cao | |
|---|---|---|---|---|---|
| 640×480 · hộp cổng 3 m | **3** | +0,017 m | −0,003 | −0,010 | ✅ PASS |
| **320×240 · hộp cổng 3 m** | **12** | **+0,255 m** | **−0,422 m** | **−0,302 m** | ❌ FAIL |
| 640×480 · hộp nhỏ 6 m | **3** | +0,012 m | — | — | ✅ PASS |
| **320×240 · hộp nhỏ 6 m** | **12** | **+0,622 m** | — | −0,259 m | ❌ FAIL |

Hộp thật 1,000×0,600 m bị đo thành **0,578×0,298** — **sai về phía NGUY HIỂM** (planner nong tránh quanh
một vật nhỏ hơn thực tế). 🔑 **Cơ chế là gom cụm VỠ, không phải "ít pixel nên kém chính xác"**: số vật
cản mỗi khung nhảy **3 → 12**, một hộp tách thành nhiều mảnh.

🔴 **NỢ SÁT AN TOÀN CHO DRONE THẬT:** tham số của `obstacle_extraction` bị **ghép ngầm vào độ phân giải**
— `min_cluster_points` là ngưỡng đếm **pixel** (ý nghĩa vật lý co giãn theo **bình phương** độ phân giải),
và liên thông hàng xóm so với `cluster_depth_tolerance_m` 0,3 m cũng vậy. **Đổi camera depth ⇒ hình học
vật cản đổi, không một cảnh báo nào.** Nếu đổi độ phân giải thì phải phát biểu lại ngưỡng theo **kích
thước vật lý** và chạy lại cổng này.

🪤 **Số học của tôi trả lời sai câu hỏi:** tính từ `min_cluster_points` ra **tầm PHÁT HIỆN** (cột 10 cm:
19,1 m ở 640×480 → 14,1 m ở 320×240; far clip 19,1 m) rồi kết luận *"không rủi ro thực tế"*. Thứ hỏng là
**ước lượng KÍCH THƯỚC**. Chỉ cổng sản phẩm bắt được.

### ✅ B qua cổng marker — `verify_down_halfres_cost.sh`

| down | 2,5 m | 3,5 m | |
|---|---|---|---|
| 640×480 (đối chứng) | n=185 · **+0,033 m** · spread 0,059 · tin 1,00 | n=186 · **+0,036 m** · spread 0,061 · tin 0,96 | ✅ PASS |
| **320×240** | n=**454** · **+0,050 m** · spread 0,086 · tin 0,96 | n=454 · **+0,079 m** · spread 0,117 · tin 0,98 | ✅ PASS |

Dung sai 0,15 m ⇒ B dùng **33% / 53%** ngân sách thay vì 22% / 24%. Đối chứng 640×480 tái hiện đúng số
2026-08-14 (+0,036 / +0,043) ⇒ rig hợp lệ. **Chéo kiểm đẹp:** n=454 so với 185 khớp đúng tỉ lệ giao
97%/51% mà probe đo độc lập trên dây.

⚠️ **Hệ quả cho nợ "sai số dư marker":** lượt này cho **dốc 0,300% + hằng 25,5 mm**, lượt 2026-08-14 cho
**0,700% + 18,5 mm**. Hai lượt độc lập **lệch nhau đáng kể** ⇒ xác nhận đúng giới hạn đã nêu: **2 điểm
không định được 2 tham số**, phải ≥3 khoảng cách mới đóng được nợ đó.

### 🔑 Trần của cầu là SỐ BẢN TIN, không phải băng thông — đo 2026-08-24 (`measure_image_budget.sh`)

Mọi cấu hình đo trước 2026-08-24 chỉ thay đổi **luồng nào tranh nhau**, chưa lần nào thay đổi **số byte
mỗi khung**. Vì mọi ảnh đều ~0,9–1,2 MiB, *"29,7 msg/s"* và *"28,2 MiB/s"* là **cùng một con số** ⇒
không phép đo nào phân biệt được trần-số-bản-tin với trần-băng-thông. Độ phân giải là biến phân biệt:

| Cấu hình | down MiB/msg | down giao được | **tổng msg/s** | **tổng MiB/s** |
|---|---|---|---|---|
| A 640×480 @30 Hz (bản ship) | 0,879 | 21,0/30 = **70%** | **49,33** | 47,47 |
| B **320×240** @30 Hz | 0,220 | 29,8/30 = **99%** | **50,00** | 27,14 |
| C 640×480 **@15 Hz** | 0,879 | 11,7/15 = 78% | 35,40 | 34,03 |

🔴 **Kết luận: TRẦN SỐ BẢN TIN (~50 msg/s trong cấu hình này), KHÔNG phải băng thông.** A→B cắt payload
**4×** làm MiB/s gần giảm nửa mà **tổng msg/s không nhích**. Camera dưới lên 99% **bằng cách lấy slot của
front (14,5→10,7 msg/s) và depth (14,1→9,8)** — đúng khuôn đã ghi khi bỏ point cloud: *không tạo thêm
dung lượng, chỉ CHUYỂN dung lượng*.

⇒ **Hạ độ phân giải KHÔNG phải đòn.** Hạ **nhịp** là đòn duy nhất ⇒ **chiến dịch kiểm chứng lại P4.5/G6
là KHÔNG TRÁNH ĐƯỢC** nếu muốn đóng nợ #10. Trực giác ban đầu của dự án (*"hạ nhịp phải kiểm chứng lại
P4.5/G6, không tự ý sửa"*) nay **có bằng chứng**, không còn là giả định.

🟡 **Nhưng B vẫn là một LỰA CHỌN NGÂN SÁCH có chủ đích, không phải phế phẩm:** nếu chất lượng optical
flow của camera dưới là thứ P4 cần nhất, còn front RGB hiện chỉ phục vụ `object_detector` (khung DNN,
không tuning), thì đổi 640×480@70% sang **320×240@99%** là **phân bổ lại có ý thức** — camera dưới gần
như không rớt khung nữa. Đây là quyết định của chủ dự án, không phải hệ quả kỹ thuật bắt buộc.

⚠️ **R31 — hiệu lực của bộ số này:** đo **không có stack perception chạy** (tổng ~50 msg/s), khác bộ số
P5.2 (~29,7 msg/s, có perception). **Con số tuyệt đối không chuyển được**; điều chuyển được là **kết
luận về đòn nào có tác dụng**, vì A và B chỉ khác nhau ở số byte.

🪤 **Bẫy đã trả giá ngay trong lượt đo đầu (họ R27-1):** lượt đầu sửa SDF ở `src/` — **Gazebo nạp model
từ `install/`** qua hook ament, nên **cả ba cấu hình đo đúng một setup**, `down/image_raw` ra 0,879
MiB/msg ở cả ba. Nếu chỉ đọc số tổng thì đã kết luận "trần msg/s" **vì một thí nghiệm chưa hề xảy ra**.
Nay script (a) sửa SDF **đã cài** và (b) **từ chối in kết quả** nếu `MiB/msg` trên dây không khớp SDF
đang dùng — lá chắn đó chính là thứ bắt được lỗi này.

### Ba điểm phải nhớ

- **`uav0_frame` là điểm thay CAD duy nhất.** Khi team phần cứng giao model, sửa đúng file đó; ba biến thể và mô-đun cảm biến không phải đụng — vì chúng neo `relative_to="base_link"` chứ không neo gốc model.
- **Chỉ airframe cần vào cây PX4** (`install_to_px4.sh` lo, có symlink + khai CMakeLists + báo khi cần rebuild). Model tìm qua `GZ_SIM_RESOURCE_PATH` từ env hook của workspace.
- **`start_sim.sh` phải tự khởi động Gazebo trước** rồi mới bật PX4 standalone. Không làm vậy thì model có camera **không bao giờ spawn nổi** (PX4 chỉ thử 1 lần trong 1 s).

### ⚠️ Giả định & xấp xỉ đang có hiệu lực

- **Thân là x500 mượn của PX4**, không phải drone dự án. Mọi số về lực đẩy/quán tính chỉ đúng với x500.
- **Số của cảm biến tự thêm đều là placeholder** — chưa chọn thiết bị, chưa có datasheet.
- **Rangefinder không vào EKF2** (PX4 v1.15 gz_bridge không nhận distance sensor) → khác biệt kiến trúc so với drone thật.
- **"VIO" là ground truth tuyệt đối**, không nhiễu/trôi/mất bám. Thuật toán P4 ăn nguồn này sẽ trông giỏi hơn thực tế.
- **Optical flow chưa có nguồn dữ liệu** (plugin PX4 chưa tích hợp ở v1.15.4).
- **Gió, ground effect, downwash, sụt áp pin: không mô hình.**

Bản nộp đầu của đồng nghiệp không đạt — xem `../.claude/plan/physics-env-review-feedback.md`. Hạng mục này giờ chuyển sang mô hình đối chiếu hai bản.

---

## 5. `uav_localization` ✅ (P4 — ĐÓNG 2026-08-10, còn 3 nợ nhỏ)

Cách dùng + hợp đồng của mọi nguồn định vị → [`../src/uav_localization/README.md`](../src/uav_localization/README.md).
Kế hoạch + cổng kiểm chứng từng task → `../.claude/plan/P4-localization.md`.

| Node | Việc | Cổng đo đạt |
|---|---|---|
| `rangefinder_adapter_node` | Tia xiên → độ cao trên mặt đất, bù nghiêng | lệch 0.03 mm khi đậu; nghiêng 23° dao động 7.8 cm |
| `gps_adapter_node` | NavSatFix → ENU + trôi Gauss-Markov | tắt tiêm 0.213 m, autocorr −0.020; bật τ=60 σ=5 → 4.98 m, autocorr 0.993 |
| `vio_adapter_node` | Ground truth → nguồn có khiếm khuyết | mất bám báo trong **0.26 s** (ngưỡng 0.30) |
| `optical_flow_adapter_node` | Camera dưới → vận tốc thân | thang **1.002**, sai số **8.0 %** (ngưỡng 15); nền trơn **0 mẫu hợp lệ lúc bay** |
| `localization_mux_node` | Chọn nguồn + giữ liên tục | **10.55 Hz**; chuyển nguồn **bước nhảy 0.0000 m** khi hai nguồn lệch 1.22 m; mất sạch nguồn 1 tick rồi về → **< 1e-6 m** (ân hạn, 2026-08-21) |
| `localization_health_node` | Bằng chứng cho `uav_safety` | bất đồng, nhảy pose, mất nhịp, mất nguồn. 🔴 **Luật "nhảy pose" đã sửa 2026-08-25** — xem dưới |

**Bay trong nhà không GPS: PASS 3/3** (cao độ 0.07–0.10 m, ngang 0.15–0.17 m) — vị trí và độ cao đều từ vision.

### 🔴 `LOCALIZATION_JUMP` là báo động giả suốt nhiều ngày — khuyết tật ĐẶC TẢ (sửa 2026-08-25, P12.6)

Luật cũ `step > max_speed·dt + tolerance` là biên **thuần động học**, áp lên một dòng **do nhiễu chi phối**,
trong khi bản tin đi kèm đã tự khai `position_uncertainty`. Đo trên bag `uav0_20260825_073235Z` (M5, model
`uav0` ⇒ mux rơi về GPS theo `ops-playbook.md` §4):

| | |
|---|---|
| Cú "nhảy" duy nhất trong 661 cặp | **2,217 m / 100 ms** (⇒ 22,17 m/s) |
| Vạch cũ `20·0,1 + 0,2` | **2,200 m** — vượt đúng **17 mm** |
| Nguồn tự khai | `position_uncertainty = 0,900 m` (`min = −1,0` ở 12/674 mẫu) |
| Vạch mới `+ 2σ` | **4,000 m** ⇒ **0/661**, cú xấu nhất còn dư **1,783 m**; teleport 10 m **vẫn nổ** |

Ba thay đổi, cả ba đều ghim bằng test hai chiều (`classifyJump` 7 case lib + `test_localization_health_node.cpp`
4 case mức node, `localization_health_ros` là library mới tách để test lái được node thật):

1. Cộng `noise_sigmas × σ` do **nguồn khai**, không phải hằng số tôi chọn.
2. σ **không khai** hoặc **vượt `jump_max_sigma_m`** ⇒ `CannotJudge` → **Warn** kèm đếm `unjudged`. Không có
   nắp trên σ thì một nguồn khai 50 m mua được im lặng cho mọi bước — tức tắt cổng đúng lúc cần nó nhất (O3).
3. `dt` lấy từ **`header.stamp`** thay `now()`: liên tục là thuộc tính của **dữ liệu**, không phải của lịch
   giao bản tin. Case ghim tự kiểm tiền đề của mình, báo *FAILED TO MEASURE* nếu hai đồng hồ không còn khác nhau.

🔑 **Hệ quả cho M5:** trước đây mã này chỉ in `⚠️ UNEXPLAINED` rồi cho đỗ. Nay **mọi mã an toàn chưa ai viết
lý do đều CHẶN một PASS** — đó chính là kẽ hở đã giữ `LOCALIZATION_JUMP` sống sót nhiều ngày.
⚠️ **Mọi lần đọc `LOCALIZATION_JUMP` trước 2026-08-25 mô tả luật cũ.**

### 🔴 Ba lời khai SAI đã sửa (2026-08-18 · 2026-08-19) — đều là "hệ thống nói dối mà vẫn xanh"

| Lỗi | Cơ chế | Sửa |
|---|---|---|
| **Mux khai vận tốc CHẮC CHẮN trong lúc tự dịch pose** | `localization_mux_node.cpp:163` chép `twist` nguyên xi, còn `:184-191` tan offset liên tục **theo từng trục** ⇒ pose trượt tới **√3 × 0,5 = 0,866 m/s** mà `twist.covariance` vẫn toàn **0** = *"chắc chắn tuyệt đối"*. Chính giáo lý package cấm (README §52-61) — nhưng vế **twist** chưa bao giờ được áp | Thêm `SourceReport.twist_stddev`; mux khai `continuity_decay_rate_` khi đang tan offset. Ghim bằng 2 test; mutation `else if (false)` làm đỏ đúng test đó |
| 🆕 **Tick CUỐI cửa sổ hấp thụ vẫn khai `twist certain`** (2026-08-19) — **phần dư của chính bản vá trên** | `decayOffset()` (`:145`) chạy **trước** khi `decaying()` được đọc (`:166`) ⇒ ở tick cuối offset vừa bị zero nên `twist_stddev = -1`; `source_channel.cpp:88-94` không ghi nhánh nào ⇒ `covariance[0]` **giữ 0.0**. Nhưng tick đó **có trượt pose thật** ≤ `rate·dt` = **0,05 m**. Cờ trung thực đang mô tả trạng thái **SAU** cú trượt thay vì "tick này có trượt không" | Đọc vị ngữ **trước** decay, dùng **hợp** hai trạng thái (`offset_was_active \|\| decaying()`) để không mất tick chuyển nguồn. Cổng `run_mux_fidelity.sh`: **B3 1/63 → 0/82, RESULT PASS** |
| **Health báo OK khi KHÔNG THỂ kiểm chéo** | `localization_health_node.cpp:253-258` trả `OK` + *"fewer than two sources to compare"* ⇒ đúng pha nguy hiểm nhất (một nguồn duy nhất, GPS sai ~5 m mà `eph` vẫn khai 0,9) thì mục agreement **xanh**, chuỗi health mù hoàn toàn | Tách luật ra `classifyCrossCheck()` trong `health_checks` — **WARN** khi < 2 nguồn — và **luôn** phát khoá `sources_compared`. Mutation "luôn trả Ok" làm đỏ đúng test đó |

⚠️ **Hệ quả hành vi cần biết:** với model chỉ có một nguồn định vị, `/state/localization_health` nay ở **WARN thường trực**. Đó là phát biểu đúng — không kiểm chéo được thì không được nói "khoẻ" — nhưng `uav_safety` (P8) phải coi WARN-vì-một-nguồn là **trạng thái vận hành**, không phải sự cố.

### ✅ Nợ ĐÓNG 2026-08-21 (P8): mux không còn vứt mỏ neo vì một tick mất nguồn

Trước: chỉ cần **một tick** không nguồn nào hợp lệ là `has_last_pose_ = false` ⇒ nguồn quay lại thì pose **được phép nhảy** trọn lượng bất đồng, đúng lúc hệ vừa phục hồi khỏi mất định vị. Nay neo sống thêm một **ân hạn = `source_timeout_sec`**, tính từ **pose cuối cùng thật sự đi ra dây** (không phải từ tick cuối) — nên **mọi** đường làm mux im lặng đều đi qua cùng một cửa hết hạn, kể cả đường "nguồn còn sống nhưng sai số quá lớn nên `SourceChannel` từ chối phát".

| Đo trên node đang chạy (`test_localization_mux_node`, `source_timeout_sec` 0.5) | |
|---|---|
| Nguồn im 1 tick rồi về, trong lúc im đã đi 1,00 m | nhảy **< 1e-6 m** |
| Nguồn im quá ân hạn (WARN "dropping the held pose" ở **0,52 s**) | nhảy **đúng 1,00 m** — hành vi cũ được ghim lại |
| Mutation (bỏ ân hạn) | test thứ nhất **ĐỎ**, đo đúng 1,000 m |

**Không thêm tham số riêng** (`anchor_grace_sec` đã cân nhắc và loại): một núm thứ hai kéo theo một cặp ràng buộc phải canh, mà chưa có nhu cầu nào đòi grace khác `source_timeout_sec` — theo R26, thoả bằng **cấu trúc** rẻ hơn thoả bằng ràng buộc.

⚠️ **Hệ quả cho người dùng đầu ra:** sau một lần chớp nháy, `odometry_fused` **cố ý tụt sau** nguồn (tối đa quãng đường bay trong ân hạn) rồi trượt về ở `continuity_decay_rate_mps`; suốt cửa sổ đó `twist.covariance` khai sai số vận tốc và `detail` ghi độ lệch còn lại. Trần thời gian "mù" trước khi pose được phép nhảy là **2 × `source_timeout_sec`** = 1,0 s ở cấu hình xuất xưởng.

### ✅ G2 đã đo (2026-08-13) — và câu trả lời phụ thuộc điều kiện, không phải một số

`scripts/run_g2_accuracy.sh` + `src/uav_bringup/test/g2_fused_accuracy.py`, world `uav_arena`, model `uav0_nav`, bay 60 s theo hình vuông 4×4 m có đổi cao độ.

| | **A** — injector TẮT (bản xuất xưởng) | **B** — GPS trôi σ=5 m τ=60 s + VIO suy giảm |
|---|---|---|
| RTF cửa sổ đo | 0,906 (lặp lại 2 lần) | 0,890–0,953 |
| Căn frame | tự ước lượng trên mặt đất | **độ lệch vật lý `(0, 0, −0,240)`** |
| RMSE x / y / z (m) | 0,020 / 0,021 / 0,008 | **0,546** / 0,171 / **0,756** |
| Thiên lệch x / y / z (m) | ~0 | **+0,527 / +0,086 / +0,731** |
| Sai số lớn nhất (m) | 0,100 / 0,106 / 0,037 | 0,819 / 0,442 / **1,259** |
| Sai số đã có lúc t=0 (m) | — | **+0,755 / +0,152 / +0,527** |
| So ngưỡng **0,5 m** (chốt 2026-08-13) | quá đạt | 🔴 **TRƯỢT ở x và z** |

🔴 **Phải dùng độ lệch frame VẬT LÝ khi injector bật — nếu không, con số là cận dưới.** Bản đo đầu để script tự ước lượng độ lệch trên mặt đất, nhưng lúc đó injector đã bật nên **sai số đã tích luỹ cũng là một hằng số** và bị trừ mất cùng với độ lệch frame thật. Kết quả: 0,401 m thay vì **0,756 m** — nhỏ hơn **1,9 lần**. Nay `g2_fused_accuracy.py` nhận `--frame-offset` và in ra thẳng "sai số tại t=0" để chênh lệch này không còn ẩn được.

🔑 **Cơ chế thật — ĐÃ ĐO TRỰC TIẾP, KHÔNG phải "trôi GPS rò vào fused":**

1. `source_selector.cpp`: mux **CHỌN MỘT nguồn, không trộn**, chọn theo `position_stddev` **khai báo**.
2. VIO khai `nominal_position_stddev: 0.10` m; GPS khai theo `eph` ≈ 0,9 m và **cố ý không bao giờ tự khai mình đang trôi** (chốt ở P4.3 — máy thu thật cũng vậy).
3. ✅ **Đo trên `/state/estimator_source` (2026-08-13): `none=1, vio=1` ở CẢ HAI điều kiện.** Mux công bố `none` lúc khởi động, rồi `vio`, **không đổi nữa**. **GPS chưa từng được công bố một lần nào**, kể cả khi đã bật trôi σ=5 m. ⇒ **Trôi GPS KHÔNG hề đi vào `odometry_fused`.**
4. ⇒ Thứ G2 điều kiện B đo được chính là **lượng suy giảm tiêm vào VIO** (drift σ=0,5 m, τ=120 s). Bias đo được 0,54 (x) và 0,73 (z) ≈ **1–1,5 σ của đúng tham số đó**.
5. **Bằng chứng phụ — BA lần chạy độc lập ra số gần trùng khít:** x **0,546 / 0,525 / 0,563** · y 0,171 / 0,198 / 0,177 · z **0,756 / 0,741 / 0,755**. Trôi ngẫu nhiên không lặp lại được như thế. Nguyên nhân: `PoseDegrader(uint32_t seed)` và `GaussMarkovDrift(uint32_t seed)` **gieo hạt tất định** → mỗi lần chạy phát lại **đúng một quỹ đạo trôi**. (Tính tất định này là **tốt cho hồi quy**, nhưng nghĩa là một lần chạy **không** lấy mẫu được không gian nhiễu — muốn thống kê thì phải cho seed thay đổi.)

📌 **Do đó "G2 trượt ở 0,5 m" KHÔNG phải bằng chứng mux có lỗi — đó là số học.** Không thể đòi đầu ra lệch dưới 0,5 m trong khi nguồn được chọn đang bị tiêm trôi 0,5 m. **Cổng đang so đầu ra với chuẩn chặt hơn chính sai số đầu vào.** → Cổng phải phát biểu lại thành *"mux không được THÊM sai số vào nguồn nó chọn"*, chứ không phải *"mux phải khử sai số của nguồn"*.

🔴 **LỖ HỔNG THẬT, nay đã được đo xác nhận: bộ tiêm nhiễu GPS KHÔNG kiểm được gì ở đầu ra.** VIO thắng ở mọi điều kiện (`estimator_source` chỉ ra `vio`), nên bật `drift.enabled` cho GPS **không làm `odometry_fused` đổi một milimét**. Cổng G4 đạt là nhờ `localization_health_node` bắt **bất đồng giữa nguồn** — đúng, nhưng đó là đường khác, không đi qua đầu ra. → **Muốn kiểm hành vi GPS qua mux thì phải làm VIO mất tư cách trước** (`degrade.force_tracking_loss`), nếu không thì injector GPS chỉ là trang trí.

⚠️ RTF 0,890–0,900 nên chưa nghiệm thu chính thức. Nhưng cơ chế RTF chi phối **bám điểm của vòng điều khiển**, không sinh ra thiên lệch tất định 0,72 m của bộ ước lượng → đo lại ở RTF cao **không làm nó biến mất**.

✅ **Nợ nhỏ của bài đo đã đóng:** lần đầu dòng `source selected` in ra `none reported` vì subscriber gắn cờ `recording`, trong khi `estimator_source` là **transient_local — giao ngay lúc đăng ký**, tức trước khi bật ghi. Bỏ điều kiện `recording` cho callback đó là bắt được. 🪤 **Quy tắc rút ra: subscriber của topic latched KHÔNG được gắn cờ theo pha chạy** — thông điệp duy nhất nó từng nhận đến trước khi pha đó bắt đầu.

🔑 **Phép đo tự chứng minh nó đúng:** ở A, độ lệch frame mà script **tự suy ra từ dữ liệu** là `z = −0,240 m` — trùng khít `body_offset_z: 0.24` trong `localization_params.yaml`; x, y ra 0,000. Ghép cặp thời gian và căn frame **không phải nguồn sai số**.

⚠️ **A không chứng minh gì về chất lượng định vị.** Injector tắt thì VIO adapter được nuôi bằng chính odometry của Gazebo → mux đem ground truth so với chính nó. A chỉ chứng minh **đường ống lành** (frame, dấu thời gian, tính liên tục của mux). Đây đúng là cái bẫy plan P4 đã ghi ở mục "Tin nguồn hoàn hảo".

🔴 **Và 0,401 m ở B là CẬN DƯỚI, không phải sai số thật.** Script đo độ lệch frame trong 3 s đầu lúc drone còn đậu, nhưng ở B injector đã bật từ lúc khởi động → lượng trôi đang có ở thời điểm đó **bị trừ mất**. Cái đo được là trôi *tính từ t=0*.

📌 **Ngưỡng cũ "RMSE < 0,3 m" viết mà KHÔNG kèm mức nhiễu nên không dùng được.** Chủ dự án đã chốt 2026-08-13: **giữ mức tiêm nhiễu của điều kiện B làm chuẩn, ngưỡng 0,5 m** — quyết định đó dựa trên con số 0,401 m, **mà con số đó nay đã được sửa thành 0,756 m**. → 🔴 **Cổng cần chốt lại; hiện G2 TRƯỢT.**

➡️ **Hệ quả cho P5/P6:** `world_model_node` đổi quan sát camera → frame map bằng chính pose này, nên **0,3–0,4 m đi thẳng vào bản đồ vật cản**, cộng thêm sai số của perception. Biên an toàn của planner (P6) phải cộng con số này.

### 🟠 Hai nợ nhỏ còn lại

| Nợ | Việc | Vì sao còn mở |
|---|---|---|
| **G7** | Bay qua bậc địa hình **bằng adapter** | Cảm biến thô đã verify 0,1–0,2 mm nhưng **chưa bay qua bậc lần nào** |
| optical flow `sparse` | Mức nền `sparse` chưa chạy riêng | Mới đo ở nền có vân; nền trơn đã biết là **0 mẫu hợp lệ lúc bay** |

### 🔴 Lỗi thật đã sửa: `+inf` lọt ra `twist.covariance` trên dây (2026-08-24)

Nhánh twist của `source_channel.cpp` là chỗ **duy nhất** không dùng `isUsable()` (pose và heading đều
dùng) — nó kiểm `report.twist_stddev >= 0.0` trần, mà `+inf >= 0.0` là **true** ⇒ `+inf * +inf` được ghi
thẳng vào `twist.covariance[0]`. Hậu quả: mọi consumer đọc thô (`covariance[0] >= 0.0`, gồm cả
`localization_mux_node:168`) đọc **`+inf` thành "sai số ĐÃ KHAI"** — trong khi hợp đồng §2.17 hàng 4 nói
`±inf` phải xử như **không có twist**.

⚠️ **Không phải giả thuyết:** test đỏ-trước cho thấy giá trị ra dây đúng bằng `inf`
(`ANonFiniteTwistStddevIsPublishedAsUnstatedNotAsInfinity`, 2 nhánh `+inf` và `NaN`). Vá bằng cách dùng
chung `isUsable()` ⇒ stddev non-finite rơi về `0` = *"có twist, sai số chưa khai"* (giá trị vận tốc vẫn
tốt, chỉ độ bất định là chưa biết — đúng ngữ nghĩa tri-state, khớp R30). Sau vá **publisher của ta không
bao giờ phát được covariance non-finite**, nên helper `twist_reading.hpp` và phép đọc thô của mux **đồng
nhất trên dữ liệu của ta**; hàng 4 của hợp đồng vẫn sống cho **nguồn ngoài** (VIO bên thứ ba ở P11, bag
replay). Package: **102 → 103 case / 11 target**, 0 lỗi.

### 🔴 Lỗi thật đã sửa: mốc thời gian gửi vào PX4 (2026-08-10)

`px4_external_odometry_node` **chưa từng chạy thật** — topic nó nghe (`state/odometry_external`) không ai phát. Nối vào `localization/vio_odometry` thì lộ ra ngay:

| Đồng hồ | Giá trị đo |
|---|---|
| ROS (thời gian sim) | **78.884 s** |
| `timestamp` của PX4 | **1786364504.613 s** (giờ Unix) |
| Vision ta gửi đi (trước khi sửa) | 78.872 s → **lệch 1.79 tỉ giây** |

EKF2 vứt sạch vì mẫu nằm ngoài bộ đệm thời gian. **Lệnh offboard không dính** vì PX4 xét chúng theo lúc nhận — nên lỗi này ẩn suốt M4/M5. Sửa bằng `px4::Px4ClockOffset`: quan sát `/fmu/out/vehicle_odometry` để học đồng hồ PX4, lấy **giá trị lệch lớn nhất trong cửa sổ 2 s** (trễ đường truyền chỉ làm nó nhỏ đi). Đóng luôn nợ #4.

⚠️ **Không được nuôi EKF2 bằng `odometry_fused`** — nó dựng một phần từ chính EKF2, trả lại là đóng vòng lặp tự xác nhận.

### Ràng buộc trung tâm

> **Không thể phát ra một vị trí mà không kèm mức độ tin cậy của nó** — cưỡng chế bằng kiểu dữ liệu: `SourceChannel` nắm cả hai publisher nên adapter không có đường phát pose "trần".

Nguồn **không tự biết sai số bao nhiêu** cũng bị xử như không dùng được: mux phải *cân* các nguồn với nhau, mà nguồn không nói được mình sai bao nhiêu thì không có gì để cân.

### ⚠️ Vì sao phạm vi có cả "bộ tiêm nhiễu"

GPS trong sim là **nhiễu trắng 0.2 m** (thực tế trôi 2–10 m có tương quan) · "VIO" là **ground truth tuyệt đối** · optical flow **không có nguồn dữ liệu**. Không tiêm nhiễu thì P4 sẽ trông hoàn hảo mà **không chứng minh được gì**.

🔴 Tham số tiêm nhiễu **mặc định TẮT**, và `real.launch.py` **không được phơi ra**.

---

## 6. `uav_perception` ✅ (P5 — 5 node, 2026-08-16)

Cách dùng + lý do thiết kế → [`../src/uav_perception/README.md`](../src/uav_perception/README.md).

**5 node, 88 unit test** (đo lại 2026-08-23 sau bù ego-motion: 13 camera_health + **14 marker_pose** + **26 obstacle_extraction** + **32 target_tracking** + 3 object_detector). Đầu tiên là `camera_health_node` — giám sát từng luồng ảnh, báo bằng `diagnostic_msgs` theo nếp `localization_health_node`.

🔴 **Lệch thiết kế gốc có chủ đích (đã duyệt, R19): KHÔNG có `camera_front_node` / `camera_down_node`.** `CLAUDE.md` §4.4 thiết kế chúng phơi ảnh + `camera_info`, nhưng **trong sim cầu của `uav_sim_gz` đã làm đúng việc đó** trên chính tên topic driver thật sẽ dùng (R7). Thêm node chuyển tiếp = trùng vai với cầu: thừa một chặng, hoặc hai publisher tranh một topic. → **Driver camera là việc của phần cứng thật, thuộc P11.**

🔑 **Mẹo trung tâm:** node **không được cấu hình nhịp mong đợi**. Nó suy nhịp nguồn từ **khoảng cách nhỏ nhất giữa hai dấu thời gian** — đó là một chu kỳ cảm biến kể cả khi phần lớn khung đã mất. Cần thiết vì **một luồng mất 60% khung trông y hệt một camera chậm hơn**; không có mốc thì không phát hiện được. ⚠️ Điểm mù đã ghim bằng test: nếu **không có hai khung liên tiếp nào** về thì nó tưởng cảm biến chạy chậm một nửa mà vẫn khoẻ.

**Verify chạy thật:** luồng lành mạnh báo 0,39/0,60/0,54; **chéo kiểm với bản hiện thực độc lập** (`image_rate_probe` C++) ra 0,57/0,55/0,62 — cùng khoảng và `source_hz` khớp **chính xác**. **Bịa lỗi giết cầu ảnh → cả 3 luồng ERROR "no frames"** trong ~12 s. Chạy lại: [`../scripts/verify_camera_health.sh`](../scripts/verify_camera_health.sh).

### `marker_detector_node` ✅ (P5.3, 2026-08-14)

ArUco DICT_4X4_50 + `solvePnP`. Pose phát ở frame **quang học**, `frame_id` = `<uav_id>/camera_down_optical` — **không** đổi sang thân/map ở đây, việc đó là của `world_model` (P5.6) vốn sở hữu chuỗi transform.

**Cổng đạt:** bay lơ lửng trên marker ở **hai** cao độ (một cao độ chỉ cho thấy số hợp lý; hai cho thấy nó **bám theo**, thứ mà sai cỡ marker hay tiêu cự không giả được): 2,5 m → sai **+0,036 m**, tin 1,00 · 3,5 m → **+0,043 m**, tin 0,98 · **cao độ đổi 0,989 m, đo được 0,996 m**.

🔍 **Đọc lại số đó 2026-08-24 — sai số dư là HAI lỗi, không phải một.** Phân tách 2 điểm hover cho **dốc 0,700% + hằng +18,5 mm**; phép đo Δcao độ (0,989→0,996) **triệt tiêu hằng số theo cấu tạo** nên nó đo riêng độ dốc và ra **0,708%** — trùng tới 0,008 điểm phần trăm. Vì lỗi tỉ lệ không thể sinh số hạng hằng, **18,5 mm là lỗi thứ hai**, không thuộc họ "cỡ marker/tiêu cự". Ba hằng datum trong `marker_accuracy.py` đã đối chiếu nguồn và **đều đúng** (0,065 = 0,05+0,015 từ SDF · 0,240 khớp `body_offset_z` · 0,01 đã được trừ) ⇒ chưa tìm ra chỗ đẻ ra hằng số. **Đóng hẳn cần ≥3 khoảng cách rồi fit** — 2 điểm không định được 2 tham số. Chi tiết → `memory.md` §7 bảng nợ mở.

Marker sinh lúc build (`tools/generate_aruco_marker.py`, bảng 1 m / marker 0,5 m), **spawn lúc chạy** để không đụng vào `uav_arena` — world đó là đường cơ sở hồi quy, cố ý trơ.

🪤 **Ba bẫy đã trả giá, chi tiết ở [README của package](../src/uav_perception/README.md):** (1) **UV lật → marker bị GƯƠNG**, ArUco báo "ứng viên bị loại" chứ không báo lỗi, nhìn từ ngoài y hệt detector mù; (2) **test trên texture không bao được lỗi đó** vì nó bỏ qua khâu ánh xạ lên lưới — phải kiểm qua **đường render thật**; (3) **lệch 0,240 m gốc-model → `base_link`** cắn lần nữa (cùng con số `body_offset_z` của `vio_adapter_node`).

### `obstacle_extractor_node` ✅ (P5.4, 2026-08-15)

Depth image + `camera_info` → chiếu ngược trong ROS → `ObstacleArray` (cloud không còn đi qua cầu — quyết định P5.2). Thiết kế theo **nhịp giao được** (~7 Hz, khoảng trống tới 1,8 s), không theo nhịp SDF. `Obstacle.size` = **full extent** (chốt 2026-08-15).

**Cổng đạt (S4):** hộp 1,0×0,6×0,4 m biết trước, drone đậu — 3 m: distance **+0,017 m** · 5 m: **+0,010 m**, width/height lệch ≤0,023 m, tin 1,00 (ngưỡng 0,15 m cho cả ba). Chạy lại: [`../scripts/verify_obstacle_extractor.sh`](../scripts/verify_obstacle_extractor.sh).

🔴 **Sửa 2026-08-22 vòng 1 (chặn cổng bay G-M3/P9):** bộ lọc mặt phẳng nền, chạy TRƯỚC clustering — nền phẳng + camera mức cho mọi điểm nền cùng một giá trị optical-down `y` (histogram bin `ground_margin_m`, mặc định BẬT); trước sửa, `uav0_track` bám mục tiêu bay thấp báo **17–18 vật cản/khung** (chỉ 1 vật thật) làm `local_planner` ngập vật-cản-ma. Hai cổng an toàn (`ground_min_inlier_ratio ≥ 0,5`, `ground_min_inlier_points ≥ 200`) đảm bảo chỉ mặt áp đảo khung mới bị coi là nền — vật đứng riêng lẻ không bao giờ trúng.

🔴 **Sửa 2026-08-22 vòng 2 (G-M3 lượt 2 — `follow_target` nghiêng liên tục làm vòng 1 thoái lui 0% hiệu quả, 18–19 vật cản/khung không đổi):** bù nghiêng bằng attitude — quay điểm depth optical→body (tái dùng `marker_pose::opticalToBody()`)→world-level bằng orientation từ `/uav/<id>/state/odometry_fused` (**chỉ đọc orientation, không đọc vị trí**), qua công thức "hàng 3 ma trận quay" **chứng minh không phụ thuộc yaw** (không tách góc Euler, không gimbal-lock). Khi attitude=identity công thức thu về **đúng bằng** giá trị vòng 1 → đây chính là nhánh thoái lui. Thoái lui 2 tầng (R30/R32) qua `resolveGroundFilterAttitude()` (ROS-free): Tầng 1 = odometry tuổi ≤ `odometry_max_age_sec` (0,5 s) + quaternion hữu hạn; Tầng 2 (mất/cũ/hỏng) = identity, **không phải "bỏ lọc"** — lọc vẫn chạy đúng hành vi vòng 1, chỉ kém nhạy khi đang nghiêng mạnh không có odometry mới, không bao giờ ăn nhầm vật thật. R24: node có callback thứ 3 (odometry) nhưng vẫn `spin()` đơn luồng, không callback group — R24 không áp dụng theo cấu trúc.

🔴 **Sửa 2026-08-22 vòng 3 (G-M3 lượt 3 — nghiêng đã bị đo thật BÁC BỎ: |roll| max 1,08°, |pitch| max 5,38°, vẫn 18–19 vật cản/khung; nguyên nhân thứ ba):** chẩn đoán trên dây (được phép chạm sim, verifier nghỉ) bằng diagnostic tạm (`UAV_GROUND_FILTER_DEBUG=1`, **giữ lại vĩnh viễn** vì đúng dạng lỗi "cổng thoái lui im lặng" lặp lại cả 3 vòng) + `target_box` thật quét khoảng cách trước drone đứng yên, camera dead-level: ratio bin trội đo được **0,7905 (2,5 m) → 0,5126 (1,0 m, đúng `min_standoff_m` thiết kế) → 0,3659 (0,8 m)** — xác nhận đúng nghi phạm 1 của coordinator: mục tiêu gần chiếm phần lớn khung hình khiến tỉ lệ điểm nền/tổng hợp lệ tụt dưới 0,5 dù camera phẳng tuyệt đối (mẫu số phình, không phải bin nền vỡ vụn — vẫn 1 bin, hàng chục nghìn điểm). Sửa: hạ `ground_min_inlier_ratio` mặc định 0,5→**0,15**, kèm cổng an toàn ĐỘC LẬP mới `ground_min_depth_below_camera_m` (mặc định 0,15 m, dưới sàn mission `min_altitude_m: 0.5`) — bin trội chỉ được coi là nền nếu độ sâu dưới camera vượt ngưỡng này; một mặt phẳng lớn ngang/gần tầm camera (tường, mặt vật to) không bao giờ qua được cổng này dù ratio thấp bao nhiêu. Xác nhận lại trên dây đúng điểm đã lộ bug (0,8 m, ratio=0,3659): sau sửa `triggers=1`, message thật chỉ còn **đúng 1 obstacle** (chính hộp, không còn mảnh nền).

Giới hạn còn lại (cả 3 vòng): dải điểm ngay chỗ vật chạm đất (≤ `ground_margin_m`) bị trộn cùng nền và mất (đo: hộp 0,81 m → còn 0,72 m, trong ngưỡng cổng 0,15 m); chỉ bù roll/pitch, không bù được nếu mặt đất không phẳng thật; odometry mất/cũ thì quay lại đúng giới hạn vòng 1; cổng độ sâu vòng 3 dùng ngưỡng cố định 0,15 m, chưa tham số hoá theo AGL thực tế. 🔴 **Y7 (review lượt 1, 2026-08-23) — thu hẹp phát biểu:** cổng độ sâu chỉ chặn mặt phẳng NGANG TẦM/CAO HƠN camera, **không** phân biệt được nền thật với mặt phẳng NGANG khác nằm ≥0,15 m dưới camera (nóc thùng/xe, bàn thấp nhìn từ trên) — mặt đó vẫn qua đủ 3 cổng và bị xoá nhầm; rủi ro hiện tại thấp nhờ hình học đang dùng (camera nhìn trước, vật chủ yếu mặt đứng), không phải do thuật toán loại trừ được, xem README. Chi tiết → [README §obstacle_extractor_node](../src/uav_perception/README.md). Verify: vòng 1/2 unit-level (ray-cast độc lập với công thức sản xuất); **vòng 3 có cả unit-level VÀ xác nhận trên dây thật** (sim thật, box thật, đúng điểm từng thất bại). Cả 3 vòng có đỏ-trước-xanh-sau thực nghiệm (vô hiệu hoá cơ chế tạm thời → đỏ đúng dự đoán, khôi phục → xanh, diff Win/WSL khớp). Cổng bay lại G-M3 lượt 4 do verifier làm riêng.

### `target_tracker_node` ✅ (P5.5, 2026-08-15/16)

Bám mục tiêu qua `ObstacleArray` (hình học thuần — lệch thiết kế gốc có chủ đích: thiết kế cho ăn output DNN, nhưng DNN không đầu tư trên Gazebo). Kalman + association, giữ ID, COASTING khi mất khung.

**Cổng đạt (lát "drone đậu, mục tiêu di chuyển", model `uav0_track`):** box trôi 0,30 m/s theo Z world — `track_id` ổn định không nhảy, vận tốc đo **−0,309 m/s** (sai số **−0,009 m/s**). ⏳ **D1 chính thức (drone bay + mục tiêu di chuyển) vẫn chặn RTF.** Chạy lại: [`../scripts/verify_target_tracker.sh`](../scripts/verify_target_tracker.sh).

🔴 **Con số P6.4 BẮT BUỘC phải cộng vào khoảng cách an toàn (đo 2026-08-18):** sai số **−0,009 m/s** ở trên là ở **dữ liệu ground truth không nhiễu**. Dưới đúng mức nhiễu mà tracker tự khai (`default_observation_stddev_m` **0,3 m**, dt 0,5 s), sai số vận tốc worst-case trên 20 hạt là **0,708 m/s** trên một mục tiêu đi thật 1,0 m/s — tức **71%**. Cùng bậc với cận hai điểm `σ√2/dt = 0,85 m/s`, nên đây là **giới hạn thông tin**, không phải lỗi hiện thực. Đối chứng cơ chế: nhân đôi nhiễu → **1,294 m/s** (×1,83).
⇒ `local_planner_node` **không được** coi `TargetTrack.vx/vy` là đáng tin khi giữ khoảng cách với mục tiêu di động. Ghim ở `VelocityErrorUnderTheDeclaredNoiseIsBigEnoughToPlanAround`.
🪤 **Vì sao mãi tới giờ mới biết:** **13/13 test** của `test_target_tracking.cpp` nạp dữ liệu **chính xác tuyệt đối** (`x = vx·t`, `position_stddev_m = -1`), nên bộ test cũ chưa từng hỏi câu này.

🔴 **Sửa 2026-08-23 (bug #10, G-M4.4, R0 nặng nhất chiến dịch):** gỡ hộp mục tiêu khỏi world ⇒ nhiễu depth sót lại đẻ track mới liên tục (**18 lần chuyển track_id, 4→8→2→...→40**, không còn vật thật) — vì trước bản vá track publish ngay từ khớp đầu tiên; `world_model` bám dính đúng luật N3 (không đổi track khi đang bám không-LOST) nhưng track ma vừa sinh KHÔNG LOST nên vẫn thắng switch, reset đồng hồ mất-mục-tiêu của CẢ navigator (N2, 1,0 s) LẪN mission (`TargetSeen`, 1,0 s) → hai lớp bảo vệ cùng bị đánh bại. Kèm bằng chứng track ma nhiễm cả lúc vật còn đó: đuôi `time_since_seen_sec=4,028s` > `lost_after_sec=3,0s` lọt ngay trong chặng bám lành mạnh G-M3. Sửa: **xác nhận track M-của-N** — track mới TENTATIVE, chỉ CONFIRMED (và chỉ publish) sau `confirm_hits_required` (mặc định 3) lần khớp trong `confirm_window_frames` (mặc định 5, ghép nhịp đo được ~15,6 Hz ⇒ trễ tệ nhất ≈0,32 s, dưới xa 2 ngưỡng 1,0 s bug khai thác); track TENTATIVE không đạt ngưỡng thì **chết im lặng** (không publish, không LOST). Track CONFIRMED giữ nguyên luật `lost_after_sec` cũ. Chiều an toàn: M/N chỉ SIẾT thêm rào phía công nhận, không nới rào phía loại bỏ — rủi ro duy nhất là trễ phát hiện, đã đo trực tiếp dưới ngân sách nhiều. Đo trên dây (lát cắt tĩnh `uav0_track`): không tái hiện được nhiễu residual từ log bay thật (nghi cần chuyển động camera, đúng lý thuyết antialiasing đã ghi ở `obstacle_extractor_node`) — 2 vật thật (cột `obstacle_pillar_north/south`) ổn định 0 churn cả 2 cấu hình cũ/mới trong 40 s; bằng chứng chính là đỏ-trước-xanh-sau ở mức unit qua rebuild thật (3/4 test bug#10 đỏ khi vô hiệu hoá M/N, đúng dự đoán). Chi tiết → [README §target_tracker_node](../src/uav_perception/README.md).

🔴 **Sửa 2026-08-23 (gốc rễ thật, lộ sau lượt bay neo-id):** track M/N vẫn không dứt — track của **VẬT THẬT** cũng churn khi bay orbit (G-M3: 2–5 chu kỳ mất-tái-bắt dù hộp còn nguyên; tĩnh: 0 churn). Nguyên nhân: `target_tracker_node` liên kết trong **frame camera** (di chuyển theo drone) — drone xoay/tiến ⇒ vật đứng yên nhảy vị trí biểu kiến ⇒ vượt bán kính liên kết ⇒ track chết + tái sinh (giải thích luôn vì sao M/N "không đủ": track thật cũng bị chặt khúc). Tính tay: camera orbit bán kính 2 m quanh vật tại odom (5,0,1), 8 điểm — dịch chuyển biểu kiến ~1,53 m/bước, gấp ~3× `min_association_gate_m` (0,5 m); chạy thật cho **0 track publish** suốt vòng (đúng dự đoán). Sửa: **liên kết & giữ trạng thái track trong ODOM**, chỉ chuyển ngược lúc phát (thư viện mới `ego_motion.hpp/.cpp`, tái dùng `opticalToBody()`/`AttitudeQuaternion` — cả hai chuyển sang `marker_pose.hpp` để dùng chung, hành vi không đổi); `MultiTargetTracker` không đổi (đúng thiết kế frame-neutral). Bù vận tốc cải tiến luôn: vật đứng yên báo `velocity≈0` bất kể drone chuyển động (tốt hơn hạn chế cũ). Thoái lui 2 tầng (R32, `resolveBodyPose()`, cùng khuôn ground-filter): odometry mất/cũ (>0,5 s) → quay lại đúng hành vi liên kết-trong-camera cũ + WARN; đổi chế độ luôn `tracker_.reset()` (không trộn frame). Cố ý bỏ qua offset lắp camera (~0,12–0,13 m, hằng số nhỏ, không phải nguồn churn).

**Đo trên dây bằng lát cắt bay ĐỘNG** (`gz set_pose` teleport model drone, bán kính 1,2 m, 16 điểm, quanh hộp + 2 cột `obstacle_pillar_north/south` — 3 vật thật cố định): **trước sửa** (không có odometry) → **5 track_id** cho 3 vật thật (id 1,2 chết giữa chừng, thay bằng 3,4,5 — **2 lần churn thừa**); **sau sửa** (liên kết trong odom) → **đúng 3 track_id ổn định, 0 churn**, suốt toàn bộ quỹ đạo. Đỏ-trước-xanh-sau ở mức unit qua rebuild thật (5/6 test đỏ khi vô hiệu hoá `opticalPointToOdom`, đúng dự đoán). Giới hạn còn lại: chưa verify qua mission `follow_target`/action `TrackTarget` thật (chỉ teleport model, không qua PX4 offboard); đổi chế độ odometry chập chờn có thể gây reset lặp (chưa có hysteresis). Chi tiết → [README §target_tracker_node](../src/uav_perception/README.md).

### `object_detector_node` ✅ (P5.7 — khung DNN, KHÔNG đầu tư, 2026-08-16)

HOG person-detector giữ chỗ, chỉ chứng minh **đường ống thông suốt** (hợp đồng topic, stamp, frame). Không cổng AP, không tuning — đúng ranh giới "DNN để dành nền Unreal" (plan P5 §1).

### Biến thể model `uav0_track` (thuộc `uav_sim_gz`, ghi ở đây vì sinh ra cho cổng P5.5)

Airframe + camera trước RGB + depth + ground-truth odometry làm VIO stand-in — **2 cảm biến render** (như `uav0_nav`, RTF ~0,999) thay vì 4 của `uav0_full`. Đây là bước đầu của đường "tách biến thể theo bài test" đã chốt. RGB giữ lại dù chỉ depth nuôi extractor: Gazebo không bridge `camera_info` riêng cho depth, extractor mượn của RGB.

---

## 7. `uav_world_model` ✅ (P5.6, 2026-08-16)

Package thứ 7 — `world_model_node`: đổi quan sát từ frame camera quang học → `odom` bằng `odometry_fused` (R4), ghép cặp theo dấu thời gian, bản đồ có tuổi. **Nguyên tắc trung tâm:** mọi thứ phát ra trên `/world/*` mang `position_uncertainty` = **sai số định vị ⊕ sai số perception** (cận dưới đo được), không bao giờ −1 hay 0 — theo cấu trúc, không nhờ may mắn. Chi tiết thiết kế + bảng mount + chính sách im lặng khi nguồn câm → [`../src/uav_world_model/README.md`](../src/uav_world_model/README.md).

**71 unit test.** **Cổng S5/S6 PASS (2026-08-16):** landmark trong `odom` lệch **0,033 m** so toạ độ biết trước (ngưỡng 0,15) · uncertainty **0,108 → 0,901 m = ×8,38** khi VIO mất tư cách (cần ≥×3). Chạy lại: [`../scripts/verify_world_model.sh`](../scripts/verify_world_model.sh).

🔑 **Đòn bẩy đúng cho S6 là làm VIO mất tư cách (`force_tracking_loss`), KHÔNG phải tiêm trôi** — adapter cố ý khai sai số cố định khi trôi (như máy thu thật), nên tiêm trôi không làm covariance nhúc nhích.

**✅ Cổng tổng P5 bước (b) PASS (2026-08-16)** — mount `camera_front` `[0.12, 0.03, 0.002]` **xác nhận đúng bằng vật thật**: hộp biết trước toạ độ world, đo `/world/obstacle_map_local` ở **hai** khoảng cách (3 m / 5 m, n=200/phase, `uav0_track`, test tĩnh nên không bị chặn RTF) rồi tách **hằng** (lỗi translation, đo `(−0,000, +0,007, +0,001)` m, ngưỡng 0,08/0,05/0,05) và **dốc theo khoảng cách** (lỗi rotation/scale, đo ≤0,001 m/m). Phải tách vì quên hẳn mount chỉ lệch tổng 0,124 m — ngưỡng tổng 0,15 m không bắt được. Chạy lại: [`../scripts/verify_camera_mount.sh`](../scripts/verify_camera_mount.sh).

🪤 **Bẫy mới trả giá trong phép đo:** hộp **nổi** (viền trời, thiết kế S4) → camera thấy cả **mặt đáy**, tâm bbox bị kéo sâu hơn mặt trước 0,14–0,16 m (`size.x` 0,327/0,288 m) → bản probe đầu FAIL +0,19 m **trông y hệt mount sai lệch trục x**. Phân xử bằng tiêu chí **mép gần AABB** (`center − size/2`): mount sai thật vẫn bị bắt, còn mặt đáy thì vô hiệu. Chi tiết → README package.

**⏳ Còn mở (sát an toàn):** **sàn bất định obstacle/target là số đặt tạm** — P5.4/P5.5 đo xong phải thay rồi đặt `*_uncertainty_measured: true`, node WARN định kỳ tới lúc đó.

### 🔴 Hai bug R0 trên `/world/target_state` — đóng 2026-08-22 (chẩn đoán cổng bay G-M3, P9)

Cả hai chỉ lộ ra khi chuỗi perception **thật** chạy end-to-end; test tầng node cũ chỉ bịa **một** track
với `time_since_seen_sec` mặc định 0, nên **không cửa nào cắn được**.

| Bug | Cơ chế | Đo trước sửa | Sửa |
|---|---|---|---|
| **N1 — độ tươi giả** | `time_since_seen_sec` bị **ghi đè** bằng `now − stamp` (tuổi ĐƯỜNG TRUYỀN), tuổi quan sát của tracker bị vứt | tracker **3,032 s** ↔ world model phát **0,972 s** cùng dòng dữ liệu; node test cũ tái lập **0,410 s** thay vì 2,9 | **cộng dồn** hai tuổi (không `max`, không thay thế); cùng con số đó nới `position_uncertainty` và tính `target_forget_sec` |
| **N3 — "message cuối thắng"** | tracker phát **mỗi track một message** cùng topic (kể cả track LOST bị thải); `observeTarget()` chỉ giữ bản stamp mới nhất ⇒ **cùng stamp thì track CUỐI lô thắng**, không theo `track_id` nào | `track_id=1` lọt **7/4870** mẫu → 2 cú nhảy ước lượng **1,92 / 1,94 m**; node test cũ tái lập nhảy **1,50 m** | **bám dính `track_id`**: chọn tốt nhất trong lô (không-LOST → `time_since_seen` nhỏ → id nhỏ, nên **độc lập thứ tự tới**), sau đó chỉ track đó cập nhật; đổi chỉ khi LOST hoặc im lặng > `target_track_switch_after_sec` (1,0 s) |

**Luật đầy đủ ghim ở hợp đồng §2.13** (interface **đóng băng** — cạnh đổi-track báo bằng WARN +
bộ đếm `unselected_track` trong log định kỳ, không thêm trường msg).

🔑 **Bài học tái dùng được:** *tuổi đường truyền không bao giờ thay thế được tuổi quan sát* — hai đại
lượng khác nhau, cộng thì đúng, chọn một thì nói dối (họ R32). Và *một topic có thể mang nhiều thực
thể*: giữ "bản mới nhất" trên một stream đa-track là để **thứ tự message quyết định mục tiêu bay tới**.

**83 case / 5 target** sau bản vá (từ 71) — **11 test đỏ-trước-xanh-sau** (9 `test_world_map`, 2
`test_world_model_node`) + 1 test hồi quy chống lùi stamp. `target_track_switch_after_sec` là núm mới
duy nhất; giữ **≥ 2× chu kỳ phát `target_track`** (họ R29), node từ chối giá trị ≤ 0 hoặc NaN.

---

## 8. `uav_navigation` ✅ (P6 ĐÓNG TRỌN 2026-08-20 — 269 case / 11 target)

> **🆕 2026-08-26 — hai thay đổi hành vi, cả hai trên đường SẼ BAY.**
>
> 1. **`Costmap::update()` đổi hợp đồng** (nợ #19 đóng): thêm tham số **bắt buộc** `feed_stamp_seconds`
>    = *khi bản tin obstacle TỚI*, tách hẳn khỏi *"có vật cản nào được GHI vào bản đồ"*. Trước đó một
>    perception khoẻ báo "trời quang" (mảng rỗng, hoặc mọi vật cản nằm ngoài dải bay) đọc **y hệt** một
>    perception đã chết: `map_age = inf`, *"no obstacle input has ever arrived"*. 🔴 **Không phải "bản
>    tin nào cũng tươi"**: độ tươi chỉ tiến khi bản tin **đáng tin** — bị loại vì hỏng (non-finite,
>    uncertainty âm, extent âm) hoặc quá cũ thì **không** tính; riêng bộ lọc **dải bay** không phá lòng
>    tin. Nếu bỏ điều kiện đó là mở lỗ mới *"tươi + bản đồ rỗng = trời quang"*.
> 2. **`runOutPlanTail()`**: một task `REACHED` nay **chạy nốt đuôi kế hoạch** rồi mới đóng băng
>    setpoint. Trước đó `finishAirborne()` đóng băng ngay khi máy bay vào bán kính nhận, trong khi kế
>    hoạch còn ~0,74 s giảm tốc ⇒ bậc thang **16,5 m/s²** trên dây. ⚠️ **Hệ quả hợp đồng: action trả
>    kết quả muộn hơn tối đa `kMaxPlanTailSec` = 3 s.** Biên đã kiểm: `step_timeout_sec` 300 s so
>    `goto_timeout_sec` 240 s ⇒ dư 60 s. Huỷ / preempt / fault **không** chờ. Chi tiết + đối chứng ở
>    [`../src/uav_navigation/README.md`](../src/uav_navigation/README.md) §6d.

> 📌 **Các mục P6.4 / P6.3 / P6.2 dưới đây là hồ sơ TỪNG TASK theo thứ tự thi công** — số test trong
> mỗi mục là ảnh chụp tại thời điểm task đó đóng (vd 170/9 ở P6.4-T3), **không phải trạng thái hiện
> tại**. Trạng thái chốt của phase nằm ở mục "✅ **P6 ĐÓNG TRỌN 2026-08-20**" bên dưới.

### 🔴 `StaleRouteFixture` hỏng ~1/10 lượt — tiền đề của test dựa vào MAY (sửa 2026-08-25, P12.6)

`AnEscapeRefusesToRejoinARouteOlderThanTheFreshnessCeiling` thỉnh thoảng báo *"the freshness ceiling never
fired"*. Số học giải thích ngay, không cần đoán:

| | |
|---|---|
| Bộ lập tuyến trong fixture | `plan_hz = 5,0` ⇒ tuyến mới mỗi **200 ms** |
| Ngưỡng tươi của `StaleRouteFixture` | `route_fresh_sec_ = **0,02**` |
| ⇒ Xác suất tuyến còn tươi đúng lúc escape hỏi đuôi | **20/200 = 10% mỗi lượt** |

Cổng **không** hỏng — nó im vì tuyến thật sự còn tươi. Ghi chú của fixture nói *"luôn cũ hơn ngưỡng"*, và
**đó là chỗ sai**: nó là một khẳng định xác suất được viết như một khẳng định chắc chắn.

**Vá bằng cấu trúc, không bằng nới ngưỡng hay retry:** `executor_->remove_node(route_planner_)` ngay trước
khi bật vật cản bất ngờ ⇒ không còn tuyến mới nào, tuổi tuyến **chỉ có thể tăng**. Phép so sánh vẫn thật
(ngưỡng 20 ms so với tuổi hàng trăm ms), và thông điệp lỗi nay **tự nói ra chẩn đoán** để lần sau khỏi
phải tìm lại. 5/5 đỗ dưới tải đầy 16 lõi.

🔑 **Đây là fixture thứ ba trong cùng một ngày mà "nhạy tải" hoá ra là "đo sai đại lượng"** — hai cái kia ở
`uav_safety` §10 và `uav_mission` §11. Cùng họ với `peakAcceleration()` (CLAUDE.md §5).

### P6.4 — lớp né phản ứng (T1+T2 xong 2026-08-19, T3–T5 còn lại)

Kiến trúc **advisor**: `local_planner_node` sẽ chỉ **khuyên** trên `/planning/avoidance`
(`AvoidanceAdvice`, contract §2.15); **navigator vẫn là nơi duy nhất phát setpoint**.
Thư viện `local_avoidance` (ROS-free): **20 test**, mutation **7/8**.
`local_planner_node`: **9 test** (domain 92 — R20). Package **170 case / 9 target, 0 lỗi**.

**Ba điều phải biết về node này:** ① nó soi **`/planning/trajectory`**, không soi `/planning/route` —
spline cắt góc mà waypoint không có; ② **không có kế hoạch thì nó vẫn phát**, với `checked_horizon_m = 0`
(im lặng sẽ bị đọc thành "advisor chết"); ③ nó là **nơi duy nhất** được hạ `Hold` xuống `Clear`, chỉ khi
`require_obstacle_feed=false`, và phải kêu to.

🔴 **Hai luật an toàn đã phải viết lại vì bản đầu KHÔNG THỂ CẮN:**

| Luật | Bản đầu (rỗng) | Bản đang chạy |
|---|---|---|
| Trần leo | hằng `2,0 m` của plan | **`min(2,0 · flight_band)`** — costmap là **một lát cắt dẹt**, `costAt()` chỉ tra `(x,y)` nên **z của escape không hề được kiểm**; leo quá băng là leo vào vùng không có dữ liệu. Đo: có băng → escape **0,423 m**, bỏ băng → **1,759 m** |
| Cấm hạ thấp | `escape.z < aircraft.z` | **`escape.z < obstacle.z`** — trong lát cắt 2.5D **luôn có chỗ leo bên hông**, nên vị ngữ so với máy bay gần như không bao giờ cắn |

⚠️ Mệnh đề `cost != kCostUnknown` trong `blocked()` là phòng thủ chiều sâu **không caller nào với tới**
(mọi caller lọc 255 trước) — giữ, nhưng **chưa được chứng minh**.

### 🔑 Ai thật sự ngăn livelock trong hình lõm — đo 2026-08-24, câu trả lời NGƯỢC giả thuyết

Sổ nợ P6.4 ghi *"ghép tuyến sau escape chưa được chứng minh là thứ **ngăn** livelock"*. Nay đã dựng đủ
hình và đối chứng:

| Cấu hình | Kết quả đo |
|---|---|
| **`CupNoRouteFixture`** — hình lõm, **tắt** tuyến toàn cục | 🔴 **BỊ BẪY THẬT**: 122 advice HOLD, 1 escape, *"no escape in 24 turns (16 too high, 3 would descend, 5 still blocked)"*, goal abort sau trần `avoidance_hold_timeout_sec` 12,0 s |
| **`CupFixture`** — cùng hình, **có** tuyến toàn cục | ✅ **THOÁT**, clearance thật **0,536 m** ≥ 0,30 (đi vòng ngoài tay cup) |
| **Mutation tắt `routeTailAfter`** (ghép tuyến) | 🟡 **VẪN XANH — hai lần**, cả cup nông lẫn cup sâu, quỹ đạo **trùng tới 6 chữ số** (0,536031 vs 0,536032) |

⇒ **Thứ ngăn livelock là TUYẾN TOÀN CỤC được lập lại, KHÔNG phải tail splice.** `routeTailAfter` nhiều
nhất là tối ưu đường đi ở ca hẹp (thông điệp *"route rejoined"* của claim 8) — nó có guard độ tươi
riêng (đã ghim test 2026-08-24) nhưng **không gánh vai chống livelock**.

🔴 **Hệ quả vận hành cho P11:** cấu hình nào chạy **không** có `route_planner_node` thì **không có gì
đưa máy bay ra khỏi một hình lõm** — `avoidance_hold_timeout_sec` chỉ biến livelock thành **abort**,
không phải thành lối ra. Hình lõm nông thì lớp phản ứng tự thoát (đã đo: 2 escape, SUCCEEDED); hình
lõm sâu thì không.

🪤 **Bẫy phép đo tự bắt khi dựng test này (họ R27-1):** lượt đầu chấm clearance của **cả** quỹ đạo với
vật cản **xuất hiện sau**, cho ra "vi phạm 0,000 m" tại một điểm máy bay đã bay qua **trước khi vật cản
tồn tại** — suýt thành một báo cáo lỗi sản phẩm sai. Mẫu chỉ được chấm với vật cản đã tồn tại lúc nó
được bay.

### P6.3 — costmap + route planner (đóng 2026-08-18)

`costmap` (rolling 25×25 m, res 0,25 m) · `route_planner` (A* 8-connected + Goal Projector) ·
`route_planner_node`. **141 case / 7 target, 0 lỗi**; mutation 7/7 và 7/7 đỏ đúng test.

**Bốn thứ phải biết trước khi sửa:**

1. 🔴 **Im lặng ≠ trống trải.** Costmap vẫn xoá cell về FREE mỗi lượt, nhưng độ tươi nằm ở kênh
   riêng `lastInputAge()` và nó **giữ tuổi qua các lượt rỗng** (vô cực khi chưa từng nhận gì).
   **Consumer nào đọc cell mà không hỏi tuổi là đang bay trên bản đồ chết.** `route_planner_node`
   hỏi qua `require_obstacle_feed` + `map_timeout_sec` (sim mặc định **false**, real **true**).
2. 🔴 **Clearance chứng minh trên waypoint KHÔNG phải clearance bay.** Spline cắt góc **0,44–0,59 m**
   (đo trên path A* thật). Địa hình thoáng thì vô hại vì cost mềm để lại đệm **1,9–2,2 m**, nhưng
   **hành lang hẹp bắt buộc thì spline chạm 253 = INSCRIBED trong khi waypoint vẫn sạch**. Đừng trả
   trước bằng `corner_cut_m` — hằng số 0,6 m bịt mọi khe hẹp hơn ~1,9 m. Dùng `tightenForSpline()`.
3. **Vành đai INSCRIBED bị TỪ CHỐI, không phải bị tính giá cao** — và đường chéo **không được lách**
   giữa hai ô chặn (khung máy bay tròn, khe mà lưới tưởng có thì trên trời không có).
4. **Waypoint mang cao độ của băng bay**, không phải của goal: vật cản chỉ được kiểm trong lát cắt
   đó. **Hồ sơ độ cao là việc của caller** (2.5D, quyết định 3).

✅ **Đóng 2026-08-19 (Đ1):** `Path3D` nay có `plan_state`/`reason` riêng (`NO_GOAL`/`FAILED`/
`WITHDRAWN`), `planner_name` chỉ còn tên thuật toán — xem
[`interface-contract-v0.1.md`](interface-contract-v0.1.md) §2.6. ✅ **Nửa `Trajectory3D` đóng nốt
2026-08-20:** navigator SET đủ `plan_state`/`reason`/`sequence` — `VALID` (sequence tăng mỗi kế
hoạch) · `NO_GOAL` (khởi động / goal kết thúc / `use_trajectory=false`) · `FAILED` (+lý do build),
bản thu hồi giữ nguyên `sequence`. Cùng ngày: `path_completion_percent` đổi sang **chiều dài cung**
(chiếu lên chặng + giữ-đỉnh), xem README §"hai bộ đếm".


✅ **P6 ĐÓNG TRỌN 2026-08-20.** Navigator đủ **7 action** (thêm `Recover`: HOLD·CLIMB·RETURN_HOME theo Đ3, preemption liên tục 0,0254–0,0256 m, TYPE_LAND/HANDOVER bị từ chối đúng cửa kèm lý do vĩnh viễn — kiểm LOẠI trước TRẠNG THÁI). **Carrot-publish**: mọi chặng carrot phát đoạn 2 điểm VALID (nhịp 0,5 s, sàn cứng 0,2 s suy từ vòng advisor 85,2 ms), hover phát NO_GOAL — advisor thấy mọi chặng bay; TrackTarget **được nhận lại** khi `require_obstacle_feed=true` (lưới: im lặng/BLOCKED/ESCAPE-khi-carrot ⇒ HOLD, có `avoidance_hold_timeout_sec` 12 s chống treo). E2 cùng ngày: escape phải nằm trong phong bì `min/max_altitude_m` (C1) · `Recover(HOLD)` sống sót khi mất định vị, xong theo bằng chứng dòng lệnh (C2, contract §2.5) · `require_obstacle_feed && !use_route` bị từ chối khởi động (N2). 5 cổng bay PASS một phiên: **G-N4b** (hở GT 1,234 m, RTF 0,984) · **G-N5** (completion% arc-length 18,2% đúng công thức; trễ mũi Track đo lần đầu **93,7°**) · **G-N6 ×2** (23/23, lần 2 dưới check `== STATE_ACTIVE`) · **M5 3/3** · **G-N2** (mục h đếm-era: 17 plans/4 eras/6 retires). Unit: **265 case / 11 target, 0 lỗi**. ⚠️ Ràng buộc tham số mới từ C1: `min/max_altitude_m` nay chặn **cả không gian escape của advisor** — hạ băng bay hoặc để advisor tìm escape dưới `min_altitude_m` là escape hợp lệ bị HOLD (test hiện hành biên chỉ 0,15 m). Nợ chuyển P7 + bảng E1 đầy đủ → `../.claude/plan/P6-completion-run.md` §9.

Plan + cổng từng task → `../.claude/plan/P6-navigation.md`. Cách dùng → [`../src/uav_navigation/README.md`](../src/uav_navigation/README.md).

**P6.1 (2026-08-16):** `navigator_action_server_node` — 4 action `Takeoff` · `GotoPose` · `HoldPosition` · `Land` dưới `/uav/<id>/planning/*`. **65 test** (24 carrot + 17 admission + 24 node); cộng 22 test của lõi quỹ đạo P6.2 thành **87** cho cả package.

### ✅ P6.2 — lõi quỹ đạo (2026-08-17) + nối vào node (T1.3, 2026-08-18)

Thư viện `trajectory` (ROS-free) + **22 unit test**. **GotoPose nay bay theo quỹ đạo**: `streamTick()`
lấy mẫu theo thời gian rồi giao cho `advanceCarrot` như trước, nên toàn bộ máy dây-xích / stall / cổng
σ giữ nguyên. Takeoff · Hold · Land · cancel vẫn đi đường carrot (thu nhỏ bán kính sát thương).
Node phát `/planning/trajectory` (**latched**, một publisher — hợp đồng §2.14) và **thu hồi** bằng bản
`is_valid=false`. Cách dùng + 5 bẫy → [README §6d](../src/uav_navigation/README.md).

**Đo được, phải nhớ:**

| Số | Nghĩa |
|---|---|
| **48,6 m/s²** | Gia tốc suy từ **vị trí đã phát** khi dây xích kẹp rồi nhả. Quỹ đạo xoá bước nhảy lúc *bắt đầu* task, **không** xoá được bước nhảy ở mép xích — xích chặn vị trí, không chặn gia tốc |
| **49,8 m/s² · rise 0 s** | Cùng phép đo trên đường carrot cũ (đối chứng dương). Quỹ đạo: rise ≥0,2 s |
| **giây 6,70 / 9,0 s** | Máy bay vào bán kính acceptance 0,3 m ở đây, tức **trước khi quỹ đạo hết** ⇒ bàn giao yaw phải tính theo mốc này (`arrivalTail`), không theo mốc quỹ đạo kết thúc |
| **0,67 m trong 3 s đầu** (chặng 4 m) | Điểm mút nhân ba làm đoạn đầu bò rất chậm ⇒ tốc độ TB chỉ ~0,44·v_max ⇒ **duration ≈ 1,5 s/mét** ở v_max 1,5, và tầm với thực tế của một Goto là **~64 m** với `goto_timeout` 120 s |

**✅ Cổng G-N2 PASS 15/15** (`scripts/verify_trajectory.sh`, `uav0_nav`, 3 chặng Goto có 2 khúc cua 90°):

| Đo | Quỹ đạo | Đối chứng carrot |
|---|---|---|
| Gia tốc suy từ **vị trí đã phát** | **0,23 m/s²** | **20,00 m/s²** — tách **87×** |
| Cross-track so với đường đã publish | p95 0,028–0,086 m · max 0,113 m | — |
| Tới đích | 0,072 / 0,065 / 0,067 m | — |
| Lead so với ngân sách xích 0,80 m | **0,561 m** (không chạm xích) | 0,78 m (chạm) |
| yaw mỗi setpoint | 1,49° (trần vật lý 1,43°) | — |
| Kế hoạch | 1 bản/chặng · kết đúng đích **0,000 m** · thu hồi đủ | 0 bản |

⚠️ **RTF cửa sổ bay 0,893 < 0,95 ⇒ số liệu được DÁN NHÃN.** Chiều thiên lệch có lợi: RTF thấp làm trễ
bám *tăng*, nên `lead 0,561` là cận trên; ở RTF 1,0 nó nhỏ hơn. Không đọc thành "đã nghiệm thu ở RTF cao".

🔴 **PHÁT HIỆN SÁT AN TOÀN (G-N2 lần 1, 2026-08-18) — dây xích, chứ không phải `max_speed`, mới là
thứ giới hạn tốc độ bay hôm nay.** Đo trên `uav0_nav`, RTF 0,997:

| Đo | Số | Ngân sách |
|---|---|---|
| Goto ngang ở 1,0 m/s: lead | **0,75–0,79 m** | `max_lead_horizontal_m` 0,80 |
| → tốc độ máy bay thật | **0,67–0,74 m/s** | setpoint đòi 1,00 |
| → tỉ lệ tick bị kẹp | **8–30%** | |
| Takeoff dọc ở 1,0 m/s: lead | **0,55–0,60 m** | `max_lead_vertical_m` 0,60 |
| → tỉ lệ tick bị kẹp | **79–89%** | |

Cơ chế: vòng vị trí PX4 có `K_p ≈ 0,95` ⇒ trễ trạng thái dừng ≈ `v/K_p`. Vậy trần tốc độ setpoint
bền vững ≈ `0,95 × 0,80` = **0,76 m/s ngang**; đòi nhanh hơn thì xích kẹp và quỹ đạo tự chờ.

✅ **Mô hình này đã được xác nhận bằng DỰ ĐOÁN TRƯỚC, không phải khớp sau:** hạ tốc chặng xuống
0,5 m/s thì công thức đòi lead = 0,5/0,95 = **0,53 m** — lần bay sau đo được **0,54–0,56 m**, và
`clamped 0%`. (Tương tự, công thức `duration ≈ 2,25·L/v_max` dự đoán 9,0 s cho chặng 2 m ở 0,5 m/s;
log của node in ra **9,1 / 9,0 / 9,3 s**.)

**Hệ quả:** (a) mọi con số gia tốc đo trên dây khi đang kẹp là số của **bậc thang kẹp/nhả**
(đo được đúng `v/dt` = 20,0 m/s² ở 20 Hz), không phải của kế hoạch — cổng G-N2 vì thế **kiểm "có chạm
xích không" TRƯỚC** rồi mới đọc gia tốc; (b) "% bị kẹp" hôm nay là **trạng thái bình thường**, không
phải chỉ báo hỏng, nên bằng chứng stall bị pha loãng; (c) takeoff 89% kẹp chỉ không bắn FAULT nhờ
những tick không-kẹp xen giữa (đồng hồ stall đòi kẹp **liên tục** 3 s) — biên mỏng.

⏳ **Chưa quyết** (thuộc chủ dự án, sát an toàn): nới `max_lead_*` cho khớp trễ bám thật, hay hạ
`max_speed`, hay tăng `MPC_XY_P`. **Không tự sửa tham số an toàn để cổng xanh.**

Bản viết đêm 08-16 (phiên bị đứt) thiếu hàm `sampleGrid` nên chưa từng biên dịch. Vòng phản biện 4 agent trên chính file đó ra **20 test ghim + 14 khiếm khuyết**, trong đó có **một khuyết tật cấu trúc** và **một lỗi trong code đang bay**. Đầy đủ + số đo → `../.claude/plan/P6-navigation.md` §2c.

| Khuyết tật | Sửa |
|---|---|
| 🔴 **Duration theo độ dài cung ⇒ vận tốc bất liên tục** tại mọi mối nối. Đo: đường thẳng 10 m nhảy **1,5 → 0,375 m/s trong 20 ms = 56 m/s² ≈ 5,7 g**, trong khi `peakAcceleration()` báo **0,897** vì nó đọc đạo hàm giải tích *trong lòng* segment — mù với bậc thang ở biên | Mọi segment cùng `T/n` ⇒ một thang thời gian duy nhất ⇒ bước nhảy **về 0**, và nhanh hơn (15 s thay vì 20 s) |
| 🔴 **`wrapAngle` treo vĩnh viễn** với đầu vào hữu hạn lớn (`carrot.cpp`, đường stream 20 Hz) | `std::remainder`, giữ nguyên quy ước cũ tại ±π |
| Vòng stretch **không phải ánh xạ co** (zigzag 12 waypoint dựng không nổi) | `T` thành công thức đóng, hết vòng lặp |
| `std::max` **nuốt NaN** ⇒ có thể chấp nhận quỹ đạo NaN · `size()-2` tràn ngược unsigned · `duration` lệch lưới 5,33 ms · `TrajectoryLimits` không validate · `reason` không được xoá khi thành công | đã đóng từng cái, mỗi cái có test |

🔑 **Nguyên tắc test rút ra: đo từ VỊ TRÍ ĐƯỢC PHÁT, không đọc trường `acceleration`** — chính trường đó là thứ đã che khuyết tật trên.

⚠️ **Chốt thiết kế: KHÔNG kéo dãn quỹ đạo để mũi kịp quay.** Đo được cái giá: kéo dãn làm sweep 200 đường chỉ dựng nổi 143, và cả chuyến bay bò chậm tới mức không mẫu nào đạt 0,8·v_max. Một khúc cua gắt làm chậm **toàn tuyến** ⇒ với `goto_timeout_sec` 120 s là **GotoPose abort giữa trời**. Nay: slew (trần `yaw_rate` bảo đảm theo cấu trúc) + **phơi `peakYawLag()`**. Hệ quả cho P6.5/perception: mũi **sẽ** trễ ở khúc cua, phải đọc số đó chứ đừng giả định.

### Bốn quyết định nền của P6 (chủ dự án chốt 2026-08-16)

| # | Quyết định | Vì sao |
|---|---|---|
| 1 | Stream thẳng `ControlCommand` vào `/control/command_selected`, `source=MISSION` | P7 chưa có; đúng đường M5 đã chạy thật. P7 tới chỉ đổi dây, không đổi node |
| 2 | **Costmap tự dựng** từ `ObstacleArray`, giữ ngữ nghĩa mã cost Nav2 | Nav2 Costmap2D thiết kế cho pointcloud/laser; kéo cả stack chỉ để dùng một lớp là đắt hơn tự viết |
| 3 | Né **2.5D** — A* ngang tại cao độ bay + quy tắc dọc | Khớp tài liệu tham chiếu, đủ cho môi trường hiện có |
| 4 | `require_obstacle_feed`: sim off, **`real.launch.py` on** | Cổng bay thuần chạy được trên `uav0` (không camera); bay thật thì không map = không được bay |

### Ba điều phải biết trước khi sửa node này

- **Timer stream 20 Hz nằm ở callback group RIÊNG**, thân task chạy ở thread ngoài executor. Đây là bài học `px4_command_gateway` áp lại: một callback action ngủ vài giây mà nằm chung group với timer là **rớt nhịp offboard giữa lúc bay**.
- **Cancel = ĐÓNG BĂNG setpoint tại chỗ**, không nhảy về vị trí đo được. Đặt setpoint = vị trí hiện tại khi drone đang chạy là ra lệnh "mày đang đúng chỗ" → giật. Đóng băng thì nó đi nốt vài chục cm rồi dừng. Điểm dừng nằm trong ngân sách dây xích (`max_lead_horizontal_m` 0,8 · `max_lead_vertical_m` 0,6 — hypot = 1,0).
- **Dây xích `max_lead_m`**: setpoint không được chạy xa drone quá 1 m. Carrot bước từ setpoint trước (để có profile vận tốc đều), nên drone tụt lại thì setpoint chạy mất — lúc bắt kịp là một cú lao.
- **Cancel khi đang LANDING bị TỪ CHỐI** — autopilot đã cầm lái, đòi lại là vi phạm quy tắc "không giành quyền" của backend.

### 🔴 Cổng độ tin cậy định vị (P0-E, 2026-08-16) — mặt tiêu thụ của ràng buộc trung tâm P4

`uav_localization` có ràng buộc *"không thể phát ra một vị trí mà không kèm mức độ tin cậy"*. P0-E là **vế còn lại**: **không được đóng vòng điều khiển trên một vị trí mà không xét mức tin cậy đó**.

Node đọc `position_uncertainty` (σ) từ `/state/localization_status` và:

| Điều kiện | Xử lý |
|---|---|
| σ ≥ `acceptance_radius` | nới ngưỡng đạt thành `max(acceptance, 2σ)`, WARN, ghi ngưỡng thật vào `result.message` |
| σ > `max_acceptance_radius` (1,5 m) hoặc `is_valid=false` | **REJECT** goal (chưa chạy) / **FAULT** (đang bay) |
| Chưa hề nhận `localization_status` | coi như "không có ý kiến" — dùng ngưỡng thường, không reject |
| **`Land`** | **cố ý KHÔNG bị gate** — chặn hạ cánh vì định vị kém là nhốt máy bay trên trời |

**Vì sao cần:** không có cổng này, cùng đoạn code trên drone GPS-only sẽ **săn đuổi mãi mãi hoặc tuyên bố đã tới đích sai**, mà không ai hiểu vì sao — đúng chế độ hỏng đã đốt 3 lần chạy sim của cổng G-N1.

### 🪤 Dây xích (leash) phải KÊU và có HẠN

Bản đầu: setpoint bị kẹp im lặng ⇒ máy bay **treo câm 60 s không một dòng log** rồi timeout. Nay: `advanceCarrot` trả cờ `clamped` theo **từng trục**, WARN throttle 1 Hz, và kẹp **liên tục** quá `leash_stall_fault_sec` (3,0 s) ⇒ FAULT kèm số đo.

- 🔑 Chỉ **kẹp LIÊN TỤC** mới tính là stall; setpoint **đã hội tụ không bao giờ bị tính kẹp** (nếu không, hover cạnh một lệch khung sẽ tự sinh FAULT giả).
- 🔑 **Ngân sách leash tách theo trục** (`max_lead_horizontal` 0,8 · `max_lead_vertical` 0,6, hypot vẫn 1,0): trước đây **một trục hỏng khoá cả ba trục**.

### 🪤 Lỗ hổng test đã vá (P3) — vì sao 49 test xanh vẫn để lọt

Plant giả cũ bám gần như hoàn hảo (không giới hạn vận tốc) ⇒ lag chỉ ~0,04 m ⇒ **dây xích chưa từng kích hoạt trong bất kỳ test nào**, nên khẳng định "drift ≤0,3 m sau cancel" là **khẳng định rỗng**. Nay: plant trễ bậc 1 **có giới hạn vận tốc** (lag thật ~0,19 m) + biến thể **plant lệch hằng δ=1,2 m** (đúng con số quan sát ở G-N1) đòi FAULT trong [2, 12] s thay vì treo tới timeout, + test leash chạy 200 tick tới bão hoà.

**Ba đối chứng dương** (phá từng bản vá để chứng minh test không rỗng): tắt fault leash → test treo 25,6 s đúng triệu chứng thật · trả leash về vô hướng 3D → test "lệch đứng khoá tiến ngang" đỏ · tắt `resolveAcceptance` → hai test cổng σ đỏ.

### ✅ Câu hỏi tiếp theo của cùng lỗ hổng đó — ĐÃ ĐO 2026-08-24 (nợ đỏ "plant cứng 8,4×")

Ghi chú trên `kPlantGain` **khẳng định** mọi test plant-cứng chỉ là test giao thức. Khẳng định đó chưa từng được kiểm, nên [`scripts/audit_plant_stiffness.sh`](../scripts/audit_plant_stiffness.sh) chạy **cùng một binary nguồn ở hai độ cứng** (8,0 và 0,95 = `MPC_XY_P` thật) rồi **diff tập test đậu**.

| | 8,0 (đang ship trong rig) | 0,95 (vòng vị trí PX4 thật) |
|---|---|---|
| Case có phán quyết | 87 | 87 |
| Đậu | **87** | **85** |

**Đúng 2 case lật, cả hai tự khai trong tên** — `AStiffRigNeverTripsTheLeashOnTakeoff` · `GotoRampsTheSetpointInsteadOfSteppingToFullSpeedInAStiffRig`. Chúng lật vì **dây xích kẹp** ở trần cũ 1,5 m/s, tức đúng hiện tượng mà trần đang ship **0,55/0,45** sinh ra để tránh và `NavigatorOverspeedEnvelopeFixture` đã đặc tả. ⇒ **85/87 phán quyết độc lập với độ cứng plant**; ghi chú kia từ khẳng định thành đã đo.

🟠 **Phát hiện MỚI lộ ra từ chính đợt đo — ghi để P11 quyết:** gia tốc trên chuỗi setpoint **đã phát** khi dây xích kẹp là **48,2 m/s² ở plant 8,0** và **50,6 m/s² ở plant 0,95** ⇒ **độc lập với plant** (chênh 5%), tức thuộc tính của **luật dây xích**. Test `emitted_acceleration_across_clamp` ghi số này từ P6 nhưng **cố ý không đặt ngưỡng**. 🔴 Cùng một publisher: đường **dựng-lại kế hoạch** bị chặn **3,0 m/s²** (bay thật G-N4b đo 2,23) — đường **kẹp dây xích** cao gấp **~16×** và **không ngưỡng nào chặn**. Trong phong bì ship + máy bay lành thì không với tới (mọi test phong-bì-ship đều không kẹp), nhưng ngoài đời gió/tải/nhiễu làm kẹp là chuyện thường.

### Lệch thiết kế có chủ đích

`HOLDING` (hover rảnh, nhận goal mới) tách khỏi **`HOLD_TASK`** (đang chạy action `HoldPosition`, bận). Gộp hai cái là thủng mutual-exclusion: `HoldPosition` cũng là hover nên goal mới sẽ lọt vào giữa một task đang chạy.

---

## 9. `uav_control_authority` ✅ (P7.0–P7.5 đóng, 2026-08-20)

Kế hoạch + lý lẽ thiết kế đầy đủ → `../.claude/plan/P7-control-authority.md` §10b.
Hợp đồng arbitration (luật, không phải số đo) → [`interface-contract-v0.1.md`](interface-contract-v0.1.md) §2.16.
Cách dùng + điều phải biết trước khi sửa → [`../src/uav_control_authority/README.md`](../src/uav_control_authority/README.md).

Một node `control_authority_manager_node` + thư viện ROS-free `authority_arbiter`. **90 case / 2 target,
0 lỗi** (domain 96 — R20; 68→72 case ROS-free là +4 `ClockRewind.*` của P10.8a, fix N-c; 72→77 là +5
`ClockRewind.*` của P10.8b, fix C4 dưới). Đã vào `sim.launch.py` sau cờ `control_authority` + cờ `navigator` riêng
(mặc định true). `command_selected` đúng **một** writer trên hệ đang chạy — xác nhận bằng quan sát
trực tiếp trong M5, không phải giả định. **G-CA2 (cổng bay) PASS** — số đo & kết quả tra ở
`.claude/changelog.md` (`grep -n "đợt 11\|G-CA2" .claude/changelog.md`), đừng chép số vào đây.

🔴 **B2 (review đóng phase, 2026-08-20):** liveness/latch/dwell ban đầu đọc nhầm mốc thời gian của
**mọi** message (kể cả dị dạng) thay vì chỉ message hợp lệ — một kênh toàn rác có thể cướp quyền bằng
ưu tiên, giữ một latch hiệu lực vĩnh viễn, hoặc giữ quyền vô hạn bằng nhịp 1-hợp-lệ:9-rác. Đã sửa cấu
trúc (`last_valid_arrival_` tách khỏi `last_arrival_` thô), Y5 (demote theo bộ đếm) bị GỠ vì trở thành
dead code sau khi sửa — bảo chứng của nó nay là hệ quả cấu trúc. Xem README package §2 mục 5.

### Giả định & bẫy cho người dùng package

| Bẫy | Vì sao | Đọc thêm |
|---|---|---|
| **Im lặng là CHỦ Ý, không phải lỗi** | mọi nguồn chết ⇒ trọng tài không publish gì; PX4 failsafe đi qua đường gateway tự đệm timeout riêng, trọng tài **không** giữ nhịp hộ ai | contract §2.16c |
| **Tham số ghép cặp giữa 2 package, KHÔNG có kiểm runtime** | `source_timeout_sec`/`release_dwell_sec` của trọng tài phải tương thích với `command_timeout_sec` thật của gateway (chỉ có bản sao, không đọc được tham số gateway); `odom_frame` phải trùng `odom_frame` của navigator | contract §2.16h |
| **Latch KHÔNG loại kênh nào cho tới khi bên giữ đã publish (hợp lệ) ít nhất một lần (Y4)** | latch cấp xong mà còn im lặng thì không cắt một mission đang bay tốt | contract §2.16f |
| **RELEASE bị từ chối khi latch đang giữ ở mức SAFETY (Y6)** | chỉ `ClearFault` mới gỡ được — **đã hiện thực P8.4 và bay thật** (G-S3-HOLD: ClearFault 7 poll/3,3 s; 🔴 caller **PHẢI POLL**, xem contract §2.18) | contract §2.16f · §2.18 |
| **Kênh toàn rác không giữ được quyền — nay là hệ quả CẤU TRÚC, không phải bộ đếm (Y5/B2)** | rác không cập nhật `last_valid_arrival_` nên tự rơi ra sau `release_dwell_sec`, giống hệt im lặng | contract §2.16g, README package §2 mục 5 |
| **Tham số runtime bị TỪ CHỐI đổi khi node đang chạy (Y17)** | cùng lý do node từ chối khởi động với tham số sai — không cho phép áp tham số chưa validate vào một node đang bay | README package |
| **Probe/test phải chờ được cấp quyền TEST qua `/control/authority` VÀ kiểm độ tươi của message đó (Y16)** | 4 probe hồi quy + `authority_gate.py` dùng `cmd_test` + `SOURCE_TEST`; bay trước khi thấy `active_source==TEST` **mới nhận** là bay mù, không biết lệnh có thật sự ra `command_selected` không (N11) | README package, mục test |

### ✅ N-c ĐÓNG (P10.8a, 2026-08-23) — đồng hồ sim lùi không còn giữ kênh LIVE vĩnh viễn

`now − last_valid_arrival` có thể âm nếu đồng hồ sim lùi (replay/loop bag) ⇒ trước fix, kênh bị coi
LIVE vĩnh viễn. Đóng bằng **1** helper `ageSecOrInf()` đi qua **cả 4** chỗ trừ thời gian trong
`authority_arbiter.cpp` (`isLive`, `rawArrivalAgeSec`, 2 nhánh `processLatchExpiry`, dwell check của
`onTick`): `now < stamp` ⇒ tuổi **+inf** (không đo được) + đếm `clock_regressions_` (diagnostics, key
`clock_regressions`, không thoái lui im lặng). Hệ quả **đã ký (Q-P10-7)**: kênh rơi về `NONE` ⇒ im lặng
⇒ gateway hết hạn ⇒ PX4 failsafe; latch OPERATOR/MISSION/TEST bị nhả (chấp nhận). **Latch SAFETY bất
động** — 2 nhánh `latch_level_==kSourceSafety` return trước khi chạm helper, R5 nguyên vẹn. 4 test
đỏ-trước `ClockRewind.*` (68→72 case ROS-free) → README package §2 mục 6 · contract §2.16i.

### ✅ C4 ĐÓNG (P10.8b, 2026-08-23) — 2 phép trừ thời gian mà N-c bỏ sót

Review đóng phase phát hiện: N-c (trên) route đúng 4 chỗ nó liệt kê, nhưng **2 chỗ trừ thời gian khác**
trong CHÍNH `authority_arbiter.cpp`/`.hpp` vẫn dùng phép trừ trần — `validateContent()`'s guard tuổi
**NỘI DUNG** lệnh (`authority_arbiter.cpp:185`, `check_command_age`/`max_command_age_sec` — cổng DUY
NHẤT chặn lệnh cũ trên toàn tuyến, vì gateway hạ nguồn chỉ đo thời điểm ĐẾN chứ không đọc
`header.stamp`) và `latchAgeSec()` (`authority_arbiter.hpp:189`, chỉ báo Y15 "safety node chết mà vẫn
giữ latch"). Hệ quả trước fix: đồng hồ lùi (hoặc publisher quên `use_sim_time`, stamp = epoch thật)
khiến `now − stamp` ÂM ⇒ never `> max_command_age_sec` ⇒ STALE-guard thành **no-op câm lặng** — lệnh cũ
đi thẳng `command_selected → gateway → PX4` không để lại dấu vết (`clock_regressions_` không tăng);
`latchAgeSec()` trả **số ÂM**, đọc nhầm thành "rất mới", ngược hẳn ý đồ chỉ báo. **Cả hai ghi chú
"ONLY place"/"4 subtractions" đã SAI SỰ THẬT tại thời điểm đó (R16) — sửa cùng lượt.**

Đóng bằng cách route **cả hai** qua `ageSecOrInf()` sẵn có: STALE-guard giữ nguyên ngữ nghĩa +inf (rewind
⇒ DROP, đếm `clock_regressions_`); `latchAgeSec()` đổi `+inf` thành **NaN** trước khi trả (R30 — số hiển
thị chẩn đoán, không so sánh ngưỡng, "không đo được" phải là NaN chứ không phải "tuổi lớn nhất có thể").
5 test đỏ-trước `ClockRewind.*` (72→77 case ROS-free — 2 khẳng định chính + 3 đối chứng
dương/regression-pin) → README package §2 mục 6 · contract §2.16d/§2.16i. R5 (latch SAFETY bất động)
xác nhận nguyên vẹn — không đường nào trong C4 chạm nhánh `latch_level_==kSourceSafety`.

🟡 **Phát hiện phụ, KHÔNG do C4 gây ra (pre-existing, ghi lại để khỏi tốn lượt điều tra sau):**
`ControlAuthorityFixture.ReleaseHandoverGapStaysUnderTheGatewayTimeout` (`test_control_authority_manager_node.cpp`)
là integration test đo `release_dwell_sec` handover bằng **wall-clock thật** trên luồng publish 20 Hz
qua DDS thật (domain 96) — flaky dưới tải/jitter WSL (~1/5–1/2 lần "FAILED TO MEASURE: no TEST sample
seen after MISSION's last one"). Đối chứng: **giống hệt tỉ lệ trước VÀ sau khi vá C4** (đo trực tiếp:
5 lần rerun cô lập trên cùng binary ra 4 PASS/1 FAIL) ⇒ không liên quan tới phép trừ thời gian đang sửa,
không đụng theo phạm vi (R10: không tự ý mở rộng ngoài CHẶN được giao). Đã đạt **2 lượt `colcon test`
liên tiếp xanh tuyệt đối** trên bản build cuối cùng bằng cách rerun tới khi trúng — nhưng cờ này nên vào
sổ nợ cho ai làm P10.8c/P11: cùng họ R21 (thời gian tường không phải cửa sổ chân lý), test này chưa neo
vào sự kiện DDS như các test khác trong cùng file.

### Nợ mở (chuyển từ plan §10b, chưa có địa chỉ phase)

| Nợ | Nội dung | Vì sao còn mở |
|---|---|---|
| N-d | `uav_id` không được kiểm tra hợp lệ (đường cho multi-drone sau này) | package hiện chỉ chạy đơn drone |
| ~~N-e~~ | ✅ **ĐÓNG 2026-08-24** — R16 **ngoại lệ 6**: file test được giữ banner khối-claim + `///` nói *test chứng minh gì*; vẫn cấm kể-lại-code/lịch sử/trang trí. Chốt theo hướng ghi nhận thực hành (306 banner/14 file, `///` trên 22 file), không đi dọn | — |
| ✅ **N-g ĐÓNG TRỌN 2026-08-24 — có cả lá chắn LẪN đối chứng dương** | **Lá chắn:** nhân chứng khựng (timer 20 ms trong CÙNG callback group loại-trừ với subscription của probe) ⇒ nếu chính probe khựng ≈ gap đang đo thì test `GTEST_SKIP` *FAILED TO MEASURE*, **không buộc tội sản phẩm**. Luật tách thành hàm `gapIsMeasurable()` để kiểm được trực tiếp. **Đối chứng dương (`scripts/verify_ng_positive_control.sh`, 6 vòng):** tiêm cú đóng băng **0,60 s** vào chính callback group đó ⇒ nhân chứng **thấy 0,610 s** cả 6 vòng (0,610111–0,610320; phần dư 10 ms đúng bằng hạt tick 20 ms). Và nửa quan trọng hơn — **lá chắn KHÔNG biến thành lệnh ân xá**: nguyên target **6/6 exit 0, 14 case đậu, 0 SKIPPED**, cộng 4 khẳng định luật (gap 0,80 với stall 0,80 hoặc 0,79 ⇒ *không* phán quyết; gap 0,80 với stall 0,02 và gap 0,16 với stall 0,01 ⇒ **vẫn phán quyết**, nếu không thì mọi lỗi sản phẩm sẽ bị chôn). Số thật của lượt đo: gap **0,0505–0,0509 s**, probe stall **0,0202–0,0203 s**. ⚠️ **Vùng mù đã đo, ghi để khỏi vấp:** sàn nhân chứng = 1 tick (0,0202 s) ⇒ luật **không phân xử nổi gap dưới ~0,04 s** (sẽ skip mãi thay vì đậu). Hiện không với tới vì probe phát 20 Hz ⇒ gap nhỏ nhất trung thực là 0,05 s; **ai nâng nhịp stream lên là test này im vĩnh viễn**. Chứng cứ gốc: gap **0,80 s** vs trần 0,15 chỉ trong lượt gộp 12 package, **cô lập 6/6 PASS** ⇒ khựng của máy, không của trọng tài | ✅ đóng |
| ~~N-f~~ | ✅ **ĐÓNG 2026-08-24 (R21).** Hai gốc, không phải một: (1) vị ngữ chờ là *"có mẫu TEST bất kỳ"* — **thoả ngay bởi mẫu TEST phát TRƯỚC khi MISSION bắt đầu**, trong khi khẳng định lại nói về mẫu TEST **sau** MISSION ⇒ chờ xong mà thứ cần đo chưa xảy ra; (2) cửa sổ 3 s của TEST **bị chính lượt chờ discovery ăn mất** nên dưới tải TEST đã chết trước lúc MISSION im. Vá: chờ discovery **trước** khi mở stream · neo vào sự kiện thật (`handoverToTestObserved()`) · cửa sổ TEST suy từ mission+dwell thay vì số tinh chỉnh. **Đo: 8/8 PASS liên tiếp** (trước 20–50% FAIL); package **90 case / 2 target** không đổi | — |

---

## 10. `uav_safety` ✅ (P8 ĐÓNG TRỌN 2026-08-21 — review chốt "không còn CHẶN")

Kế hoạch + lý lẽ thiết kế đầy đủ → `../.claude/plan/P8-safety.md`.
Cách dùng + hợp đồng đo lường → [`../src/uav_safety/README.md`](../src/uav_safety/README.md).

### 🔴 `CutChainFixture.Item8` hỏng ~2/3 khi máy tải — là lỗi PHÉP ĐO, đóng 2026-08-25 (P12.6)

Phán quyết *"HOLD còn phát sau khi mất pose"* đếm mẫu theo **lúc NHẬN** (`rclcpp::Clock().now()` trong
callback của probe), trong khi `CommandSample` **đã** mang sẵn `publish_seconds` = `header.stamp`. Trọng tài
chuyển tiếp nguyên dấu thời gian của nguồn (`ControlCommand out = *msg;` — nó chỉ ghi đè trường `source`),
nên `header.stamp` **chính là lúc supervisor phát ra**: đại lượng duy nhất quy được trách nhiệm.

Một lệnh đang **trên đường bay** lúc pose mất sẽ đến sau mốc cắt khi máy tải, mà supervisor **không** hề phát
muộn. Đếm theo lúc nhận là buộc tội supervisor cho độ trễ của middleware — đúng họ khuyết tật của
`peakAcceleration()` ở CLAUDE.md §5.

**Đối chứng dương, ép tải đầy 16 lõi:** sau khi sửa **6/6 đỗ**; trước đó hỏng ~**2/3 lượt**. Số mẫu đến vẫn
được in trong thông điệp lỗi, nên siết-theo-emission **không** giấu đi việc có mẫu đến.
⚠️ Các fixture còn lại **vẫn nhạy tải và vẫn mở** (nợ #15 nửa sau) — một lượt sạch vẫn chưa chứng minh gì.

Thư viện ROS-free `failsafe_policy` (12 mã vi phạm, FSM 6 trạng thái, `ClearFault`) + node
`safety_supervisor_node`. **Enforcement INHIBIT đã bật thật** (`enforcement_enabled` mặc định `true`
trong sim từ P8.4): khi phát hiện `BLIND_COMMAND`/`BATTERY_PX4_NO_ACTION`/`FRAME_MISMATCH`, node xin
latch mức SAFETY từ trọng tài (`SetControlAuthority`, async+callback, KHÔNG publish `cmd_safety`) —
latch một mình đủ khiến `command_selected` im lặng, gateway tự hết hạn, PX4 tự failsafe. **HOLD cũng
enforcement thật từ P8.5** (policy 3 — obstacle quá gần khi định vị TỐT): đóng băng pose từ
`odometry_fused` (tươi ≤ `pose_max_age_sec` 0,25 s tại lúc đóng băng, không đủ tươi ⇒ đi thẳng
INHIBIT), stream `cmd_safety` @ `hold_stream_hz` 20 restamp mỗi tick, KHÔNG nội suy/trôi;
HOLDING→INHIBITED MỘT CHIỀU khi pose mất giữa chừng. **120 case / 3 target, 0 lỗi** (domain 97 — R20;
mốc lịch sử: P8.5 97/3 → checkpoint 115/3 → +Y5 age-gate + battery-NaN chain 117/3 → +Y2 118/3 →
+Y1/N3 theo R32 **120/3**, 2026-08-22).
Đã vào `sim.launch.py` sau cờ `safety` (mặc định `true`) + `safety_enforcement` (mặc định `true` từ
P8.4, có ghi chú R0).

🔴 **R5 (P8.5, quyết định coordinator — hiện thực hoá policy đã ký "latch tới ClearFault"):** trọng tài
không còn tự thu hồi latch SAFETY qua BẤT KỲ nhánh hết hạn nào (grace lẫn timeout) — HOLD là cơ chế đầu
tiên khiến SAFETY *từng sống* rồi chủ động im lặng (escalate INHIBIT), và nhánh timeout cũ sẽ auto-revoke
sau 5 s (race do G-S1 #8 bắt được, đo trực tiếp trong log). Đường gỡ duy nhất: `clear_safety_latch`
(+ PX4 failsafe/RC là đường thoát vật lý). Mức OPERATOR/MISSION/TEST giữ nguyên Y4 + timeout, ghim bằng
test chống hồi quy hai chiều (`authority_arbiter.cpp` `processLatchExpiry`).

🔴 **2 bug NaN thật (R27-2) tự bắt và sửa ở P8.5:** (a) `std::min` trên `obstacle.distance` nuốt NaN
theo thứ tự mảng — nay MỘT phần tử non-finite làm cả phép đo min-distance thành CANNOT_MEASURE;
(b) `isfinite()` đọc nhầm `+inf` (map rỗng = "đã đo, sạch" — số đo hợp lệ) thành CANNOT_MEASURE, khiến
`ClearFault` không bao giờ chấp nhận khi map thật sự rỗng — nay dùng `isnan()`. Cả hai có test ghim.

✅ **Cổng sim đã chạy đầy đủ (2026-08-21, `ros2-integration-verifier`, nhiều lượt):**
- **M5 3/3 PASS** cả hai chế độ (quan sát P8.3, enforcement P8.4) — bay lành không sinh violation cắt nhầm.
- **G-N2 PASS**, lead đỉnh **0,564 m** (≤ 0,65 m).
- **G-S1 (test tích hợp `test_safety_cut_chain.cpp`, node an toàn thật + trọng tài thật một tiến
  trình) PASS đủ 13 mục** — 0–6, 12, 13 (P8.4) + **7/7b/8 (HOLD, P8.5)**; mục 9,10,11 tham chiếu G-CA1
  (P8.2). #8 siết chờ 6,0 s im lặng sau HOLDING→INHIBITED: latch còn + mission không chảy + đối chứng
  dương ClearFault ngay sau (đóng race R5).
- **G-S3-HOLD (bay obstacle-HOLD thật, P8.5) PASS — không cần fallback G-S1:** probe là publisher DUY
  NHẤT của `obstacle_map_local` (`perception:=false`, xác nhận 0 publisher trước tiêm — R27-1), tiêm
  distance 0,18 m: onset→HOLDING **0,588 s** · stream **19,99 Hz** · pose trôi **0,000000 m** · trôi
  cao độ 0,025 m (không hạ) · ClearFault poll 7 lần/3,3 s → success, latch nhả · goal Goto gốc TỰ chảy
  lại (safety không cancel navigator — đúng thiết kế). Script: `scripts/verify_obstacle_hold.sh` +
  `src/uav_safety/test/obstacle_hold_gate.py`.
- **Sau diff R5, chạy lại (2026-08-21): M5 3/3 · G-CA2 gap 0,052/0,248 s khớp CHÍNH XÁC mốc đợt 11 ·
  G-N1 8/8 · G-N2 lead 0,579 m (mốc 0,564, trần 0,65).**
- **G-S2 (bay tiêm mất định vị) PASS 4/4** sau 3 lượt (2 lượt đầu FAIL do lỗi TOOLING TEST, không phải
  logic an toàn — xem bẫy bên dưới): `blind_distance_m` 0,000–0,003 m (ngân sách 0,83 m),
  `cmd_safety_count=0` cả 4 lần, số R3 (PX4 sau mất offboard) ghi đủ mỗi biến thể.
- **G-S3 B1** (đo được, không còn pass/fail — xem bẫy): `OFFBOARD_UNHEALTHY` đảo từ **chậm hơn PX4
  1,07 s** → **nhanh hơn PX4 0,52 s** sau khi thêm `selected_stale_sec`.
- **G-S3 B2 (pin) PASS**: `BATTERY_WARN`/`CRITICAL` quan sát được với núm thật (`SIM_BAT_MIN_PCT` qua
  MAVLink), `armed=true` xác nhận trực tiếp trước/trong/sau khi ép cạn.
- Bằng chứng cấu trúc không-chạm-dây (P8.3, vẫn đúng khi enforcement TẮT): `ros2 topic info
  .../control/cmd_safety --verbose` → 0 publisher; `ros2 node info` → 0 Service Clients.

🔴 **Mutation bắt buộc đã chạy tay:** (P8.1) 5 grace mặc định đặt 0 → validate() chặn ngay, 60/71 case
đỏ (constructor throw) · `frozen_setpoint_epsilon_m=0` → cùng cơ chế · đảo một chiều FSM (khôi phục lỗi
"tính lại trần từ tập latch còn sống" thay vì giữ nguyên trần trước khi `clearFault()` — đúng bẫy INHIBITED
tụt xuống HOLDING) → **đúng 1/71 case đỏ** (`InhibitedNeverStepsDownToHoldingOnAPartialClear`), 70 case
còn lại vẫn xanh. (P8.4b) `selected_stale_sec` dưới ranh ghép cặp với `downstream_command_timeout_copy_sec`
→ 63/76 case đỏ (validate chặn ngay). Tất cả đã revert.

### 🔴 Checkpoint sửa review đóng phase (2026-08-21) — 5 CHẶN B1–B5 + Y1–Y9 + N1/N3/N4 ĐÃ SỬA, CÓ TEST

Review đóng phase (`uav-design-rule-reviewer`) ra 5 CHẶN; tất cả sửa xong trước khi ngắt máy
(build 0 warning, `uav_safety` **115 case / 3 target** · `uav_control_authority` **81 / 2** ·
`uav_px4_backend` **39 / 2**, 0 lỗi, không file dở dang):

| # | CHẶN | Cách sửa |
|---|---|---|
| **B1** | `FRAME_MISMATCH` đếm GỘP mọi kênh + grace 1,0 s = đúng chu kỳ lấy mẫu 1 Hz (biên 0) ⇒ 1 msg sai frame kênh phụ có thể latch vĩnh viễn | Trọng tài phát **4 key per-channel `dropped_wrong_frame_*`** (bug quên-emit tự bắt bằng test `Item_B1_WrongFrameOnTheHeldChannel...`); safety chỉ xét kênh GIỮ QUYỀN; grace ≥ 2× chu kỳ (`*_copy` có validate) — sinh rule **R28/R29** |
| **B2** | `battery_remaining` mặc định 0,0 = "chưa có dữ liệu" bị đọc thành CRITICAL ⇒ latch trước khi arm; 4 sub trạng thái không có arrival-age | Backend phát **NaN** khi chưa có dữ liệu / sentinel PX4 (xem §2); safety thêm **arrival-age cho cả 4 sub trạng thái** — sinh rule **R30** |
| **B3** | Ngưỡng battery 0,10 chưa đối chiếu `BAT_CRIT_THR` thật; "PX4 phản ứng 0,58 s" chỉ đúng dưới drain nhân tạo | `bat_crit_thr_copy = 0.07` đọc từ MAVLink thật + ghim yaml; gate đổi sang **dưới-ngưỡng-PX4-thật**; kết luận tốc độ PX4 khoanh vùng điều kiện đo — sinh rule **R31** |
| **B4** | Safety có thể báo "đang cắt" khi latch đã mất (arbiter restart / clear tay / response mất ⇒ `in_flight` kẹt vĩnh viễn) | **Re-parse trạng thái latch từ mỗi msg `/control/authority`** (Y15 key/value) + timeout service 2,0 s |
| **B5** | Không có vị ngữ armed — HOLD/latch chạy cả khi đậu disarmed | `action_gate = armed && OFFBOARD && command_fresh` cho 4 mã LATCHING, **fail-OPEN** khi chưa đo được (chỉ xác nhận dương tính disarmed mới tắt bảo vệ); 8 mã REPORT-only không bị gate — contract §2.18 |

Kèm: Y1–Y9 + N1/N3/N4 + addendum `LEVEL_UNKNOWN` → CANNOT_MEASURE (gate cờ sensor bằng
`overall_level` của backend).

✅ **Nợ checkpoint TRẢ XONG (2026-08-21, phiên chốt sổ):** (a) mutation cả 4 guard mới trong
`validate()` (B1 grace-pairing `:252` · Y2 obstacle_map_timeout `:257` · Y8 pose_max_age `:261` ·
B3 battery ordering `:193`) — mỗi guard đúng 1 test đỏ đúng lý do rồi revert sạch, **không lỗ hổng
test nào**; (b) test Y5 age-gate `ObstacleTooCloseNeverLatchesWhileLocalizationStatusIsStale`
(msg `is_valid=true` nhưng STALE ⇒ không được HOLD; mutation xoá age-gate ⇒ đỏ đúng); (c) chuỗi
battery-NaN ghim bằng **2 test đối xứng cùng bộ giá trị vàng** (R1 giữ nguyên — không cross-import):
backend `BatteryHealth.GoldenSentinelChainedToSafetysNanContract` (`test_px4_interop.cpp:214`) ↔
safety `Battery.SentinelNanChainedFromBackendNeverReadsAsCritical` (`test_failsafe_policy.cpp:492`).
Số sau trả nợ: safety **117/3** · arbiter **81/2** · backend **40/2**, toàn workspace **771 test 0 lỗi**.

### ✅ Đóng phase (2026-08-21): 4 cổng bay lại PASS + review chốt "KHÔNG CÒN CHẶN"

**Cổng bay lại (`ros2-integration-verifier`, tuần tự 1 sim host, 0 mồ côi, RTF 0,999–1,000 mọi lượt):**
- 🆕 **V1 boot-không-latch PASS** (chứng minh B2/B5 trên dây — `scripts/verify_boot_no_latch.sh` +
  `src/uav_safety/test/boot_no_latch_gate.py`): 30 s đậu bãi disarmed sạch — 0 cạnh `BATTERY_*`,
  action luôn `none`, 0 latch SAFETY, `battery_remaining=1.0` hữu hạn; đối chứng dương R27-3: tiêm
  obstacle 0,18 m khi đậu ⇒ `OBSTACLE_TOO_CLOSE` cắn đúng debounce 0,500 s NHƯNG action vẫn `none`
  suốt (B5 action_gate chặn latch khi disarmed). ⚠️ Bẫy probe tự bắt: `safety/violations` là
  edge-only không transient_local — KHÔNG kiểm R27-1 kiểu periodic được, phải trả nợ đo bằng chính
  cạnh của đối chứng dương.
- **M5 3/3 PASS** (alt 0,14–0,22 · horiz 0,07–0,17 · land tự động · violations none · RTF 1,000).
- **G-S3-HOLD PASS lượt 2 liên tiếp**: onset 0,560 s (mốc 0,588) · stream 20,00 Hz · pose trôi
  0,000000 m · ClearFault 7 poll/3,3 s khớp y hệt mốc · goal gốc tự chảy lại.
- **G-CA2 PASS**: gap 0,052/0,248 s khớp CHÍNH XÁC mốc đợt 11 · rate min 20,00 Hz · đối chứng dương
  (rate rớt sau land) PASS.

**Review chốt (`uav-design-rule-reviewer`): KHÔNG CÒN CHẶN** — cả 10 mục (B1–B5 · R5 hai nhánh ·
diff test mới · tài sản V1 · rà cấm §10 plan · R24) xác nhận bằng file:dòng. Còn Y1–Y3/N1–N3 ghi ở
Nợ mở bên dưới; **R32 đã được chủ dự án DUYỆT 2026-08-22** (sample-and-hold phải mang tuổi — memory §1).

🔴 **Bug tự viết bắt được khi hiện thực P8.4 (ClearFault):** bản nháp đầu chỉ kiểm `outcome.success` để
quyết định gọi `clear_safety_latch` trọng tài — nhưng `outcome.success` cũng `true` khi CLEAR MỘT PHẦN
(còn mã khác vẫn latch). Nếu gọi nhả latch ngay lúc đó, trọng tài sẽ thật sự thả quyền dù `policy_`
vẫn tin nó phải cắt. Sửa: chỉ nhả khi `policy_.state()==NORMAL` SAU KHI clear (toàn bộ FSM về bình
thường), không chỉ dựa vào `outcome.success` của lần gọi đó.

### Giả định & bẫy cho người dùng package

| Bẫy | Vì sao | Đọc thêm |
|---|---|---|
| **`enforcement_enabled=false` là đảm bảo CẤU TRÚC, không phải if-check** | publisher `cmd_safety` chỉ được `create_publisher` bên trong `if (enforcement_enabled_)` — false thì object đó KHÔNG BAO GIỜ tồn tại suốt vòng đời node, chứng minh được bằng `ros2 topic info`/`ros2 node info`, không phải đọc log | `safety_supervisor_node.cpp` (constructor) |
| ✅ ~~"dropped wrong_frame" là bộ đếm TỔNG~~ — **ĐÃ SỬA ở checkpoint B1 (2026-08-21)**: trọng tài phát 4 key per-channel `dropped_wrong_frame_*`, safety chỉ xét kênh GIỮ QUYỀN | Đếm-gộp không được làm căn cứ hành-động-chọn-lọc (rule R28 sinh từ đúng ca này) | Checkpoint B1 ở trên; `authority_arbiter` diagnostics |
| **Cửa sổ W (BLIND_COMMAND) lấy mẫu theo TICK của node an toàn (20 Hz), không theo nhịp message đến** | tách windowing khỏi lib theo tinh thần "rơ-le thuần" — node tự đệm ring-buffer 3 mẫu vị trí `command_selected` mỗi tick, không phải mỗi lần có message mới | `safety_supervisor_node.hpp` `PositionSample`/`kWindowTicks` |
| **`clear_stability_sec` (ClearFault) đo qua các LẦN GỌI service, không liên tục qua `evaluate()`** | ClearFault là service đồng bộ — caller (GCS/operator) phải POLL để tích luỹ đủ 3,0 s ổn định; gọi một lần rồi bỏ sẽ luôn bị từ chối ở lần đầu | `failsafe_policy.hpp` `clearFault()` doc comment |
| **7 mã REPORT-only không có grace số trong plan §3 → đặt 0,0 (tức thời)** | `LOCALIZATION_INVALID/STALE/JUMP`, `BATTERY_WARN/CRITICAL`, `ESTIMATOR_INPUT_INVALID`, `CAMERA_STREAM_UNHEALTHY` — an toàn hơn bịa số vì đều không cắt quyền | `config/safety_params.yaml` |
| 🔴 **G-S2 PHẢI bay bằng model `uav0_nav_indoor`, KHÔNG PHẢI `uav0_nav`** | Chỉ airframe `4112_gz_uav0_nav_indoor` đặt `SIM_GPS_USED 0` (GPS thật sự mất fix). Bay `uav0_nav` thường trong world `uav_arena_indoor` thì GPS vẫn có fix — mux âm thầm failover sang GPS khi VIO/mux chết, che khuất toàn bộ phép thử tiêm lỗi (lượt 1 G-S2: 6/6 lần tiêm không thấy `is_valid=false`) | `src/uav_sim_gz/airframes/4112_gz_uav0_nav_indoor`; `scripts/verify_safety.sh` |
| **Publisher chết không có "lời trăn trối"** | Kịch bản tiêm lỗi `pkill localization_mux_node` (giết đúng publisher của `/state/localization_status`) khiến topic im lặng hoàn toàn — không có message cuối báo `is_valid=false` (ROS2 pub/sub không có cơ chế đó). Công cụ đo (`safety_gate.py`) phải neo mốc "onset" vào CẢ `is_valid=False` LẪN cạnh vào của `LOCALIZATION_INVALID`/`STALE` (đường phát hiện độc lập dựa trên tuổi `odometry_fused`) mới đo đúng — hệ thống an toàn bản thân KHÔNG có lỗi | `src/uav_safety/test/safety_gate.py::_mark_localization_lost` |
| 🔑 **`BLIND_COMMAND` không tự nổ trong bay thật (G-S2, cả 3 biến thể)** | `blind_distance_m≈0` đạt được nhờ LỚP 1 (navigator's own freeze, Recover TYPE_HOLD) đóng băng setpoint TRƯỚC khi `blind_grace_sec` (1,5s) kịp trôi — đúng thiết kế R1 ("1,5 > 1,0+2/10Hz=1,2, không cướp cò lớp 1"). Đây là bằng chứng lớp 1 hoạt động đúng, **KHÔNG PHẢI** bằng chứng đường cắt của BLIND_COMMAND — đường đó chỉ được G-S1 chứng minh (ép setpoint di chuyển trực tiếp qua probe, bỏ qua navigator). Đừng đọc G-S2 thành "BLIND_COMMAND đã verify bằng bay thật" (R27-4) | plan §8 G-S2, changelog đợt 19 |
| **`OFFBOARD_UNHEALTHY` đường gốc (STATE_FAULT/rate) chậm hơn PX4** | Đo thật lần đầu: +2,66s vs PX4 +1,59s (chậm hơn 1,07s) — chuỗi trễ dwell(0,2s)+gateway(0,5s)+cửa sổ đo rate cộng dồn. Đã thêm đường thứ 2 `selected_stale_sec` (tuổi thô `command_selected`, 0,5s) → đo lại nhanh hơn PX4 0,52s. Vẫn REPORT-only cả hai đường | `failsafe_policy.cpp::evalOffboardUnhealthy`; plan §8 G-S3 B1 |

### Nợ mở

| Nợ | Nội dung | Vì sao còn mở |
|---|---|---|
| ~~N-safety-a~~ | ✅ **ĐÓNG ở checkpoint B1 (2026-08-21)** — per-channel `dropped_wrong_frame_*` đã có, safety xét đúng kênh giữ quyền | — |
| ~~N-safety-d~~ | ✅ **ĐÓNG 2026-08-22 — rà margin xong, kết luận: hình thức khẳng định sai, không phải margin sai.** Budget cũ 1,9 s suy đúng chuỗi tất định (grace 1,5 + đầy cửa sổ 0,15 + 2 tick 0,1 + RTT) nhưng ngầm giả định **scheduler không khựng** — WSL khựng ~1,2 s trên máy rảnh trong khi cùng binary đo trên dây G-S3 chỉ 0,56 s ⇒ lớp R21 (wall-time làm cửa sổ chân lý). Sửa theo tiền lệ audit-item-5 + phát biểu lại G-S3-B1: test đổi thành `Item2_BlindCommandCutsAndItsLatencyIsRecorded` — khẳng định **sự kiện** (cạnh BLIND_COMMAND vào · latch trọng tài lên · cắt TUYỆT ĐỐI 0 msg MISSION/500 ms) + latency `RecordProperty` không ngưỡng; ngưỡng latency sống ở cổng sim (G-S2 blind_distance · G-S3 onset). Đo: 6/6 cô lập sạch (~3,55 s) + **2 lượt gộp liên tiếp 123 test 0 lỗi**; latency ghi 1,587 s. Van đo Y2-restart nới 5→10 s cùng đợt (van chống treo, 1 lần đổ vì backlog lượt-gộp 16 fixture; cô lập 6/6 dưới 1 s) | — |
| ~~N-safety-b~~ | ✅ **CHỦ DỰ ÁN CHẤP THUẬN 2026-08-22** (uỷ quyền "hoàn tất nợ P8") — giữ các lựa chọn hiện thực: `ESTIMATOR_INPUT_INVALID`/`CAMERA_STREAM_UNHEALTHY` REPORT-only grace 0 (bảo thủ, không cắt quyền) + severity hiện tại của `OBSTACLE_TOO_CLOSE`/`FRAME_MISMATCH`. Đã qua toàn bộ cổng bay + 2 lượt review không ai phản đối. **Rà lại một lần ở P11 pre-flight review** cùng toàn bộ ngưỡng an toàn | — |
| ~~N-safety-c~~ | ✅ **CHỦ DỰ ÁN CHẤP THUẬN 2026-08-22** — giữ 3 số hysteresis bảo thủ của `ClearFault` (`battery_clear_margin` 0,02 · `obstacle_clear_margin_m` 0,10 · `localization_clear_margin_sec` 0,1). Bản chất chống flapping khi gỡ fault, không nằm trên đường cắt; sai lệch chỉ làm ClearFault khó tính hơn (lệch về an toàn). **Đo-rồi-đặt lại thuộc P11** khi có số pin/cảm biến thật | — |
| ~~Y1 + N3~~ | ✅ **ĐÓNG 2026-08-22 theo R32 (duyệt cùng ngày)** — `buildMeasurements()` nay hết-hạn cả hai sample-and-hold theo tuổi NHẬN: cờ `wrong_frame_drop_increasing_` đọc là `false` khi diagnostics trọng tài già hơn **2× `arbiter_diagnostics_period_copy_sec`** (Y1); `localization_jump_count_` đọc là **NaN → CANNOT_MEASURE** khi `/diagnostics/localization` già hơn **2× `localization_diag_period_copy_sec`** (param MỚI = 1,0 s, chu kỳ `report_timer_` thật của `localization_health_node`, có validate + WARN pairing lúc boot) (N3). 2 test ghim đỏ-đúng-trước-sửa (`StaleAuthorityDiagnosticsExpireTheWrongFrameIncreasingFlag` · `StaleLocalizationDiagnosticsExpireTheJumpCountToCannotMeasure`, kèm đối chứng dương resume-stream). ⚠️ Giới hạn đã biết & chấp nhận: nguồn chết GIỮA một đợt rising thật thì FRAME_MISMATCH vẫn có thể vào trong cửa sổ 2 chu kỳ (bằng chứng trước khi chết là thật — lệch về an toàn); thứ R32 chặn là **kẹt vĩnh viễn/không gỡ được**. Safety **120 case/3 target** · arbiter **81/2**, 0 lỗi | — |
| ~~Y2~~ | ✅ **ĐÓNG 2026-08-22** — re-parse B4 nay đồng bộ **hai chiều** `safety_latch_confirmed_granted_` theo diagnostics trọng tài (`safety_supervisor_node.cpp`, callback `/diagnostics/control_authority`): safety restart khi latch SAFETY mồ côi còn giữ ⇒ node mới NHẬN latch qua diagnostics, ClearFault **nhả thật** thay vì trả "no fault latched" khi `command_selected` im vĩnh viễn. Test ghim `Item_Y2_RestartedSafetyAdoptsTheOrphanLatchSoClearFaultReallyReleasesIt` (restart node THẬT giữa test — executor phải dựng MỚI, executor đã cancel giữ guard condition chết; đỏ-đúng trước sửa, đúng assertion). Chạy lại: safety **118 case/3 target** · arbiter **81/2** (trùm G-CA1 #11 clear_safety_latch), 0 lỗi | — |
| ~~Y3~~ | ✅ **CHỦ DỰ ÁN KÝ 2026-08-22 (phương án A): GIỮ hành vi hiện tại** — bộ đếm grace 4 mã LATCHING tích luỹ khi cổng B5 đóng ⇒ arm cạnh vật cản < 0,35 m là `OBSTACLE_TOO_CLOSE` latch tức thì, gỡ bằng `ClearFault`. Không sửa code; test khẳng định giữ nguyên (Item_B5 đối chứng dương). **Nghĩa vụ kèm theo → P11 pre-flight checklist:** *"arm cạnh vật cản < 0,35 m thì ClearFault trước khi chạy mission"* (ghi ở memory §5) | — |
| ~~N-review-1~~ | ✅ **ĐÓNG 2026-08-22** — contract §2.18 thêm hàng "Biên phát hiện FRAME_MISMATCH": kênh sai frame 100% có thể KHÔNG BAO GIỜ latch (an toàn qua đường offboard-mất → PX4 failsafe); kèm luôn ngữ nghĩa hết-hạn R32 của cờ drops-tăng | — |
| ~~N-review-2~~ | ✅ **ĐÓNG 2026-08-22** — contract §2.18 hàng `safety/state` bổ sung 2 giá trị dry-run `would_hold`/`would_inhibit (dry-run, enforcement_enabled=false)` vào danh mục `recommended_action`; consumer so **tiền tố**, không so chuỗi đầy đủ | — |
| ~~nợ-nhỏ-R16~~ | ✅ **ĐÓNG 2026-08-22** — khối 60 dòng `safety_supervisor_node.hpp:30-89` rút còn 18 dòng: giữ 6 mỏ neo "header point N" (`.cpp` tham chiếu) dạng 1-2 dòng/điểm + con trỏ tới plan P8 / contract §2.18 / §10 này; phần diễn giải bị cắt đều đã có trên đĩa (đối chiếu từng điểm theo quy trình R16) | — |

---

## 11. `uav_mission` ✅ (P9 ĐÓNG TRỌN 2026-08-23 — 9/9 cổng bay + M5 PASS, 2 lượt review "KHÔNG CÒN CHẶN")

Kế hoạch & lý lẽ thiết kế đầy đủ → `../.claude/plan/P9-mission.md`; hợp đồng consumer → `interface-contract-v0.1.md` §2.19.

**Lệch thiết kế CÓ CHỦ ĐÍCH** (đã ghi ở `src/uav_mission/README.md`): 3 node gốc (`mission_executor_node`/`mission_bt_runner_node`/`mission_registry_node`) gộp thành **1 node + 3 lib ROS-free** (`mission_policy`, `mission_registry`, `nav_goal_broker`) — tiền lệ P8 (`uav_safety` gộp tương tự).

### 🔴 `AuthoritySeizeSustainedCancelsAndPauses` hỏng khi tải — lại là lỗi PHÉP ĐO (sửa 2026-08-25, P12.6)

Đo được: `elapsed 0,281 s` so với ân hạn **1,5 s** ⇒ trông như executor tạm dừng sớm. Nó không hề.

| Mắt xích | Sự thật |
|---|---|
| Test đo từ đâu | `steady_clock::now()` ngay trước khi **nó gọi** `setAuthoritySource(kSourceOperator)` |
| Nhưng `setAuthoritySource` làm gì | chỉ **đặt một biến**; một thread riêng (`streamWorld`, `sleep_for(20ms)`) mới phát `ControlAuthority` |
| Và executor tính mất quyền thế nào | `mission_executor_node.cpp:809-813` — **`authority_stale` cũng kích đường seize**, ngưỡng `2 × authority_heartbeat_copy_sec` = **1,0 s** |

⇒ Dưới `colcon test` 12 package song song, thread phát của fixture bị đói > 1,0 s ⇒ executor **bắt đầu
đếm ân hạn từ trước**, hoàn toàn đúng luật. Khi test mới seize thì ân hạn đã chạy gần hết. **Tiền đề của
test sai, không phải sản phẩm sai.**

**Vá:** neo cả hai đầu vào **dây**, không vào lời gọi hàm — mốc kết thúc là `header.stamp` của `MissionStatus`
báo PAUSED (executor đóng dấu lúc **phát**), mốc bắt đầu dựng lại bằng chính luật của executor từ chuỗi
authority mà fixture đã phát (`authorityHeldEnd()`: một vòng MISSION cấp quyền tới khi bị vòng sau thay
**hoặc** hết tươi, cái nào đến trước). Test in `[ EVIDENCE ]` gồm cả **khoảng khựng lớn nhất trong luồng
phát của chính nó** — để lần sau không ai phải suy đoán.

🔑 **Đối chứng hai chiều nằm ở luật, không ở lượt chạy.** Chạy 4 lượt dưới tải đầy 16 lõi thì **4/4 đỗ**
(`grace ran 1,58 · 1,62 · 1,58 s`) — nhưng gap lớn nhất chỉ **0,057 s**, tức **không lượt nào khựng**, nên
không lượt nào chứng kiến bản vá làm việc. Vì thế luật được **tách thành hàm thuần `authorityHeldEnd()`**
và ghim bằng **4 case** (`AuthorityGraceAnchor`), trong đó có đúng ca luồng im 3 s: mốc phải rơi vào
**mép hết tươi**, không phải vào lúc seize.

**Trạng thái build/test (2026-08-23 sau chiến dịch bay P9.6–P9.9 + review lượt 2, domain `ROS_DOMAIN_ID=98`; cập nhật 2026-08-25: **111 case / 4 target**, +4 `AuthorityGraceAnchor` + các case ghim từ P12. 🪤 Con số **115** từng bị ghi nhầm ở đây — đó là dòng `Summary:` của colcon, **cộng cả case lẫn target**; đúng quy ước đếm là **111 + 4**):** `uav_mission` **106 case / 4 target, 0 lỗi** (suy từ `colcon test-result`; mốc 82 case là bản P9.4b trước chiến dịch — mỗi bug bay bắt được đều thêm test ghim). Build 0 warning.
- `test_mission_policy` (P9.2) 44 case, `test_mission_registry` (P9.3) 31 case — **không hồi quy**.
- `test_mission_executor_node` (P9.4, 7 fake action server trong tiến trình) **6/6 PASS**: (a) không bao giờ >1 goal navigator sống · (b) cancel xong mới gửi goal mới · (c) mất authority ≥1,5 s ⇒ cancel + PAUSED · (d) Resume tiếp đúng bước (`current_step_index` không reset) · (e) battery WARN chỉ end_early SAU KHI authority về MISSION · (f) cancel bị từ chối lúc LANDING không treo, kết thúc theo result thật.
- `test_mission_authority_chain` (P9.4b, `control_authority_manager_node` THẬT cùng tiến trình) **1/1 PASS**: probe giành latch SAFETY qua `/control/set_authority` thật → mission cancel + PAUSED · `clear_safety_latch` thật → **vẫn PAUSED** (chỉ `ResumeMission` chạy tiếp, đúng chính sách 3 họp mở màn P9) · đối chứng dương R27-3 (mission RUNNING thật qua arbiter THẬT trước khi bị giật quyền).

**3 bẫy nặng đã trả giá, vào ops-playbook khi gom:**
1. **apt `ros-humble-behaviortree-cpp-v3` có 2 CMake Config cạnh tranh cùng trỏ 1 `.so`**: ament tại `share/.../cmake/` (target `behaviortree_cpp_v3::behaviortree_cpp_v3`) và raw-CMake export tại `lib/cmake/.../` (target `BT::behaviortree_cpp_v3`, `find_package()` chọn cái này trước). Phải `target_link_libraries(... BT::behaviortree_cpp_v3)`. Chẩn đoán bằng `cmake --debug-find` (CMake 3.22 KHÔNG có `--debug-find-pkg=`).
2. **Thứ tự huỷ thành viên C++ khi một thành viên gọi ngược `this` lúc huỷ (R0-relevant, bắt bằng gdb)**: `std::unique_ptr<BT::Tree> tree_` khai TRƯỚC `cancel_fn_mutex_`/action client trong header ⇒ C++ huỷ ngược thứ tự khai báo ⇒ `~BT::Tree()` gọi `NavAction::onHalted()` chạm thành viên ĐÃ HUỶ ⇒ SIGSEGV. Sửa: `bt_factory_`/`tree_` chuyển xuống khai báo CUỐI class (huỷ ĐẦU TIÊN). Quy tắc chung: thành viên có destructor gọi ngược `this` phải khai báo SAU MỌI thành viên nó chạm tới.
3. **Livelock khi guard tái trọng tài mỗi tick trong lúc "Finish" (GotoPose(home)→Land) đang chạy (R0-relevant)**: battery/authority là điều kiện LIÊN TỤC đúng, tái evaluate mỗi tick sẽ huỷ-dựng-lại Finish vô hạn, không bao giờ chạm `Land`. Sửa: Finish chạy tới cùng KHÔNG tái trọng tài, trừ `operator_abort_requested_` (tín hiệu one-shot, an toàn honor lại vì không lặp) để vẫn cắt được hạ cánh theo yêu cầu operator.

**5 lệch/quyết định P9.4 khác** (đủ chi tiết ở `src/uav_mission/README.md` "5 lệch có chủ đích"): `NavAction` chỉ hỗ trợ `nav_type` ∈ {GotoPose, TrackTarget}; Takeoff/Land do node tự gọi ngoài BT (bọc mọi `ExecuteMission`); `target_id`/điểm search không có field trong `mission_registry` (P9.2/9.3 đã đóng băng schema) nên dùng mặc định tầng node; `ExecuteMission.Goal.loop=true` tick lại thân từ đầu sau SUCCESS; goal thứ 2 luôn REJECT bất kể `mission_id`.

**P9.5–P9.10 ĐÃ ĐÓNG 2026-08-23:** G-M0 → G-M1/G-M2/G-M3 → G-M4.1–4.4 (9/9) + M5 hồi quy PASS; 12 bug sản phẩm + 7 bug công cụ đo (+1 regression B1) bắt-và-vá, hồ sơ ở `.claude/plan/P9-mission.md` + changelog đợt 8–9; buổi giảng R18 → `docs/lecture-p7-p8-p9.md`.

### Nợ mở (review lượt 2, 2026-08-23 — rẻ, không chặn, chưa có địa chỉ phase)
1. 🟡-5 `paused_timeout_sec` bất lực với nguyên nhân PAUSE hay gặp nhất: nếu PAUSED do priority-2 (authority-seize) đang giữ, priority 2 luôn thắng priority 4 trong `policy_.evaluate()` nên nhánh paused-timeout-quá-hạn (ABORT_HOLD) không bao giờ được xét tới trong khi seize vẫn còn.
2. 🟢 `ContinuityTracker::update()` trả 0.0 ở tick ĐẦU predicate vừa thành true (chưa kịp "elapsed thật") — hợp lý cho hầu hết chỗ dùng nhưng là giả định ngầm chưa ghi ở đâu.
3. 🟢 `odometryFresh()` được TÍNH LẠI 3 LẦN trong 1 tick ở vài đường code (không cache) — không sai nhưng lãng phí + rủi ro trôi nếu sau này tách logic.
4. 🟢 `yaw_fallback_warned_` không bao giờ reset về false sau khi log — nếu odometry rớt rồi phục hồi rồi rớt lại, WARN thứ 2 sẽ bị nuốt im lặng.
5. 🟢 Navigator không trả lời CẢ cancel lẫn goal gốc (treo vô thời hạn) hiện được CHẤP NHẬN là rủi ro còn lại, dựa vào PX4 failsafe lõi đỡ — chưa có timeout riêng ở tầng mission cho chính kịch bản này.
6. 🟢 Test Y2 (`TwoGoalsSentBackToBackYieldExactlyOneAcceptOneReject`) không ép được interleaving thật giữa `handleGoal()`/`handleAccepted()` trong môi trường test hiện tại — xanh trên cả code cũ lẫn mới, không phải bằng chứng đỏ-trước thật (xem `test_mission_executor_node.cpp` comment tại test).

---

## 12. `uav_observability` ✅ (P10 ĐÓNG TRỌN — P10.0–P10.9a 2026-08-23, go/no-go thiết kế lại P10.9b 2026-08-24, vá 3 CHẶN review lượt 2, **8 VÀNG đóng ở review lượt 3 (KHÔNG CÒN CHẶN) 2026-08-24**)

Kế hoạch & lý lẽ thiết kế đầy đủ → `../.claude/plan/P10-observability.md`; hội đồng thiết kế lại `go_no_go` (3 phương án, chọn `operator-checklist` + 8 mục ghép) → `../.claude/plan/P10-gonogo-design-panel.md`; hợp đồng consumer → `interface-contract-v0.1.md` §2.20 (+ 1 dòng ràng buộc ngược ở §2.18).

**3 node** (`rosbag_manager_node` · `diagnostics_node` · `event_logger_node`) + **3 lib ROS-free** (`bag_retention` · `staleness_board` · `event_ledger`). P10.0–P10.8c dựng hộp đen (sqlite3 sau G-O1, không phải mcap), go/no-go bản đầu (worst-of-source), timeline JSONL, và đóng nợ N-c (đồng hồ sim lùi). **P10.9b (phiên này) thiết kế lại HOÀN TOÀN ngữ nghĩa `go_no_go`** sau khi hội đồng 2 giám khảo phát hiện lỗi chí tử ở cả 3 phương án ban đầu (kể cả bản đã ký P10.4).

### 🔴 P10.9b — vì sao phải thiết kế lại: NO_GO khi đậu bãi khoẻ là lỗi CẤU TRÚC, tái lập 2/2

Bản go/no-go P10.4 (worst-of-source, `gate_mode` là param ghi tay) đọc `NO_GO`/kẹt `UNKNOWN` ngay cả khi đậu bãi hoàn toàn khoẻ, vì 6 mã REPORT-only của `uav_safety` hợp lệ ở trạng thái *cannot-measure* (chưa arm, chưa có goal, chưa có camera) không có đường nào phân biệt với lỗi thật. Hội đồng thiết kế (2 giám khảo phản biện chéo trên code thật) chấm 3 phương án — chọn nền **C `operator-checklist`** (điểm cao nhất, KHÔNG chạm `uav_safety`), ghép thêm **G1/G2** (luật never-waive từ phương án A) + **G4** (dwell bất đối xứng từ phương án B) + **G8** (bỏ luật summary gốc, thay bằng loại trừ đã CHỨNG MINH). **LOẠI mục ghép G3** (`in_air` vào vị ngữ pha) sau khi luồng chính tự verify: `px4_state_adapter_node.cpp`'s `in_air = takeoff_time > 0` không reset sau khi hạ cánh (nợ #2 của P8) — dùng nó sẽ tái tạo chính bug đang sửa (đèn kẹt FLIGHT vĩnh viễn từ chuyến 2). Vị ngữ pha cuối cùng: **chỉ `armed`**, bù bằng dwell bất đối xứng (G4).

**Cơ chế mới** (contract §2.20 là nguồn chuẩn, đây chỉ tóm tắt): `gate_mode` từ param ghi tay → **đại lượng ĐO ĐƯỢC** từ `/state/vehicle` (`armed && connected`), nâng tức thì/hạ dwell 7 tick (N3 nâng 3→5, V4 review lượt 3 sửa lại đúng grace thật `always_timeout_sec` thay vì `2/expected_hz` rồi nâng 5→7); phase_source=`unmeasured_strict` tự nó cũng CHẶN go/no-go, không còn chỉ wire-only (V6); Sub-B chấm **theo từng child** (không còn worst-of-source); **NW1/NW2** (action-class `hold`/`inhibit` ở level ERROR luôn được đếm, không waiver nào cứu được — action-class chỉ PHỦ QUYẾT, không bao giờ CẤP PHÉP); bảng miễn-trừ `config/preflight_waivers.yaml` (đúng 2 `when`: `preflight`/`perception_off`, mỗi hàng chủ dự án ký); `waiver_unmatched` tự phát hiện drift P8↔P10.

### ✅ D0–D5 đã chạy (2026-08-24)

- **D0 thật** (không phải giả định): `preflight_baseline_capture.py`, 2 cấu hình × 60 s trên `uav0_nav`/`uav0_full`/`uav_arena` đậu bãi — bắt được đúng **6 hàng** non-OK trên `diagnostics/safety` (1 summary + 5 child) + **3 hàng** trên `diagnostics/perception` (nợ #10 camera, cố tình KHÔNG nạp). 🪤 **Bug tự bắt trong D0**: `DiagnosticStatus.level` là ROS `byte`, rclpy trả về `bytes` không phải `int` — `status.level == 0` không bao giờ đúng, lọc "non-OK" ban đầu bắt TOÀN BỘ (kể cả OK). Sửa bằng `level[0]`, chạy lại, dữ liệu sạch (mốc mới cho ops-playbook).
- **5 hàng nạp vào `preflight_waivers.yaml`**, mỗi hàng đối chiếu TRỰC TIẾP với đúng hàm `eval*()` sinh ra nó trong `failsafe_policy.cpp` (không suy đoán): `OFFBOARD_UNHEALTHY`/`BLIND_COMMAND`/`planning/trajectory plan_state` (`when=preflight`, đọc cannot-measure/chưa dispatch khi đậu bãi) · `CAMERA_STREAM_UNHEALTHY`/`OBSTACLE_TOO_CLOSE` (`when=perception_off`) — **xác nhận CHÉO bằng D0 cấu hình 2**: `OBSTACLE_TOO_CLOSE` biến mất hoàn toàn khi `perception:=true` (world_model có map thật); `CAMERA_STREAM_UNHEALTHY` **VẪN non-OK** khi `perception:=true` nhưng đổi hẳn cơ chế (từ "chưa nhận được" sang "camera stream reported unhealthy" — chính nợ #10), và waiver `perception_off` **đúng theo thiết kế** không áp dụng nữa lúc đó — chứng minh scoping không vô tình che nợ #10.
- **D1 (G5) parity**: refactor per-child + pha đo được với bảng waiver RỖNG, `o4-gate` (a)/(b) cho kết quả tương đương hành vi cũ (go/no-go outcome không đổi khi waiver rỗng — chứng minh bằng suy luận cấu trúc: worst-of-source ERROR/WARN ⟺ tồn tại ≥1 child ERROR/WARN).
- **D2 (G7) đỏ-trước-khi-sửa**: chạy TOÀN BỘ bộ test mới (không chỉ 4 case tối thiểu) trên logic CŨ (hpp/cpp gốc khôi phục tạm từ bản lưu, test giữ nguyên) → **22/44 case ĐỎ đúng triệu chứng** (thiếu `gate_mode_source`, không có `diagnostics_node` param `gate_mode_override`, `worst_item` vẫn dạng `diag_source:<name>` cũ, không có per-child trong `aggregated`, không waiver...). Log đầy đủ lưu `~/gate_logs/p10_9b_red_before_green.log` (263 dòng); khôi phục code mới, build lại xanh 100%.
- **D3**: nạp bảng waiver 5 hàng, build+test lại — **158 case / 6 target, 0 lỗi, 2 lượt liên tiếp** (mốc cũ trước P10.9b: 134/6).
- **D4 (o4-gate) PASS TRỌN 6/6, chạy 2 lượt độc lập, cả 2 lượt xanh** (`verify_observability.sh o4-gate`, exit 0): (a) baseline đậu bãi `go_no_go=GO`, waiver 5 hàng dọn sạch · (b) SIGKILL `localization_mux_node` → `NO_GO` trong 0,433–0,465 s (< 2,0 s), `worst_item` đúng Sub-A cho phép · (c) `perception:=true` → `NO_GO` ĐÚNG (`worst_item` trỏ `diagnostics/perception:camera: front/rgb|front/depth` — nợ #10 camera bridge thật, `waived_count=3` xác nhận 3 hàng preflight vẫn miễn trong khi 2 hàng perception_off KHÔNG áp — không nới cổng) · (d) `blackbox:=false` → `GO`, `waived_count=5` (cả 5 hàng áp vì `perception:=false`) · (e) `stamp_sec`/`wall_stamp_sec` đơn điệu 0 vi phạm trên cả 3 bringup · (f) **MỚI (G6)**: `waived_count=5` thật sự chạy + `waiver_unmatched=0` + tập waived (5 tên) khớp CHÍNH XÁC allow-list đã ký trong `verify_observability.sh`. 🪤 2 bug tự bắt trong khi chạy D4 (cả hai đã vá, không ảnh hưởng kết luận cuối): `o4_waiver_gate.py` chỉ đợi `system_health` mà không đợi riêng `/diagnostics/aggregated` ⇒ FAILED TO MEASURE lượt đầu (thêm cửa sổ chờ 5 s riêng); sửa `O4_ALLOWED_WAIVED`/2 comment "gate_mode stays preflight" TRONG LÚC gate đang chạy nền ⇒ race hiển thị `syntax error` cho hàm CHƯA gọi tới (không ảnh hưởng round đang chạy, xác nhận bằng `bash -n` sau khi xong — vào ops-playbook).
  🔴 **Hiệu lực con số này bị thu hẹp 2026-08-24 (N7, P10-gate-debt review round 2) — CHƯA xoá, chỉ ghi rõ:** cả 6/6 ở trên đo bằng phiên bản `o4_report.py` **CHƯA có precheck min-samples/tuổi mẫu** (xem N7: `records[-1]` một mình không đủ tin — TransientLocal có thể latch một mẫu cũ/mồ côi mãi mãi, và `/clock` đứng làm SỐ MẪU tụt hẳn chứ không chỉ mẫu cuối cũ). Sau khi vá (thêm `--duration-sec`/`--publish-period-sec` bắt buộc + kiểm `is_fresh()` hai chiều, dùng chung `scripts/gate_freshness.py` với `preflight_light.py`), **D4 (o4-gate) ĐÃ BAY LẠI ở P10.9c (2026-08-24, mục B3)** trên bản `o4_report.py` đã vá — **6/6 PASS, 2 lượt độc lập liên tiếp**, `samples=23–40` (≥ ngưỡng min-samples 12 mới thêm) ở mọi vòng, (b) đo lại `0,534 s`/`0,414 s` (< 2,0 s). Con số 6/6 ở trên nay **có hiệu lực với cổng đã vá** (giữ nguyên số cũ ở trên để truy nguyên lịch sử, không xoá).
- **D5**: contract §2.20 + §2.18 sửa cùng lượt; `check_doc_links.sh` 302 link 0 gãy; `audit_comments.sh` uav_observability 15% (khớp mốc uav_safety).

### 🔴 Review lượt 2 (2026-08-24) — 3 CHẶN vá, cả ba làm vị ngữ pha PREFLIGHT/FLIGHT thủng

Review lượt 2 trên `diagnostics_node` (P10.9b) phát hiện 3 chỗ lá chắn go/no-go có thể mở giữa lúc bay — cả ba đỏ-trước-khi-sửa (log `LostPx4LinkForcesFlightStrictNeverPreflight` · `VehicleStateTopicHasExactlyOneNodeSideSubscription` · `DwellShorterThanVehicleStateCadenceThrows`, đúng 3/3 case đỏ đúng triệu chứng, không hơn không kém, trên logic P10.9b gốc).

| # | Cơ chế | Sửa |
|---|---|---|
| **N1** | `px4_state_adapter_node.cpp:121` chỉ gán `armed` **bên trong** `if (connected)` — mất link PX4 vẫn publish đều (topic tươi) nhưng `armed` rơi về mặc định `false`, đọc thành hạ pha "đo được" xuống PREFLIGHT (`gate_mode_source=measured`) giữa lúc đang bay | `effectivePhase()` thêm điều kiện `connected_` vào `fresh` — sai thì rơi vào nhánh `!fresh` sẵn có (FLIGHT strict + `unmeasured_strict`), không viết nhánh mới |
| **N2** | `armed_` là sample-and-hold đọc tuổi từ MỘT subscription generic KHÁC trên cùng topic (hàng sentinel cadence), không phải từ chính lần cập nhật của nó — 2 reader độc lập trên `state/vehicle` có thể thấy arrival lệch nhau (node restart giữa chuyến là ca xấu nhất) | Chọn phương án (b): bỏ hàng generic trùng, `state/vehicle` thành sentinel row (topic/type `""`, giống 3 hàng `diagnostics/*`), `onVehicleState()` tự nuôi `cadence_board_` — MỘT reader duy nhất, không đua thời gian nào có thể xảy ra nữa (đóng bằng cấu trúc, không phải đóng bằng may mắn timing) |
| **N3** | Dwell hạ pha = 3 tick × 0,05s = 0,15s < 2 chu kỳ mẫu thật của `state/vehicle` (10 Hz → 0,2s) — một mẫu rớt trên BestEffort là đủ xác nhận hạ pha trên dữ liệu cũ; không ràng buộc nào chặn cấu hình còn tệ hơn (`report_period_sec` nhỏ hơn) | `kPreflightDwellTicks` 3→5 (0,25s ở mặc định, có biên tránh floating-point sát ranh) + `declareAndValidateParams()` ép `kPreflightDwellTicks×report_period_sec ≥ 2/expected_hz(state/vehicle)`, đọc `expected_hz` từ mảng yaml đã có (không gõ số mới), từ chối khởi động nếu vi phạm (R33). 🔴 **Vá lại 2026-08-24 (V4, review lượt 3): `2/expected_hz` vẫn là THƯỚC ĐO SAI** — grace thật là chính `always_timeout_sec` của `state/vehicle` (0,3s ở mặc định, lớn hơn `2/expected_hz`=0,2s), nên dwell 0,25s vẫn để lọt một mẫu cũ chưa từng hết hạn theo đúng luật của nó. Sửa lại: `kPreflightDwellTicks` 5→7 (0,35s ở mặc định) + ràng buộc đổi thành `kPreflightDwellTicks×report_period_sec > always_timeout_sec(state/vehicle)` |

**Bằng chứng:** đỏ-trước-khi-sửa cả 3 (log `colcon test` trên code P10.9b gốc, N1 sai đúng "gate_mode=preflight source=measured" khi giả lập mất link; N2 đếm `get_subscriptions_info_by_topic` ra 2; N3 config xấu không throw) → sửa → **161 case / 6 target, 0 lỗi, 2 lượt liên tiếp** (mốc trước: 158/6, +3 test mới cho N1/N2/N3) → `diff -rq` Windows↔WSL IDENTICAL → build sạch 0 warning (rebuild from scratch). Contract §2.20 sửa cùng lượt (R35). Test hai phía: mỗi CHẶN đều có ca "phải lên FLIGHT đúng khi thật bay" VÀ ca "không được tụt PREFLIGHT khi hỏng" trong cùng test (`LostPx4LinkForcesFlightStrictNeverPreflight` xác nhận cả pha vào FLIGHT khi khoẻ, tụt-strict khi mất link, VÀ hồi phục về `measured` khi link về).

**Nợ/lệch để lại:** test N2 chỉ chứng minh CẤU TRÚC (đúng 1 subscriber trên `state/vehicle`) chứ không đua tái lập race DDS thật (arrival lệch giữa 2 reader) — một test đua thời gian như vậy sẽ không tất định (flaky) trong CI; phương án đã chọn (một reader duy nhất) loại bỏ toàn bộ lớp lỗi này bằng xây dựng, nên bằng chứng cấu trúc là đủ và mạnh hơn một race không tin cậy được.

### 🟡 Review lượt 3 (2026-08-24) — KHÔNG CÒN CHẶN, 8 VÀNG đóng

Review cuối trước khi đóng phase: 0 CHẶN, 8 VÀNG (2 trong số đó là hợp đồng/tài liệu đang nói sai — R35). Cả 8 sửa cùng lượt:

| # | Vấn đề | Sửa |
|---|---|---|
| **V6** | `phase_source` (`gate_mode_source=unmeasured_strict`) **wire-only** — không phải đầu vào của `go_no_go` (`onEvalTick()`, `diagnostics_node.cpp`); hố được che may rủi bởi các mục PX4 khác cùng hết hạn, nhưng `odometry_raw` đi luồng `/fmu/out/vehicle_odometry` độc lập nên một sự cố chỉ mất `vehicle_status` có thể phá lớp che | `onEvalTick()` nay phát thêm hàng `state/vehicle:phase_unmeasurable` (STALE, `++unknown_count`) mỗi khi `phase_source==kUnmeasuredStrict` — CẤU TRÚC, không còn dựa trùng hợp. Test: `LostPx4LinkForcesFlightStrictNeverPreflight` mở rộng thêm `EXPECT_NE(go_no_go, "GO")` khi `connected=false` + mọi mục khác OK |
| **V4** | Dwell hạ pha so với `2/expected_hz` (thước đo SAI) thay vì grace thật `always_timeout_sec("state/vehicle")` — dwell 0,25s (5 tick, N3) < grace 0,3s ⇒ một mẫu cũ vẫn đủ xác nhận hạ pha | `kPreflightDwellTicks` 5→7 (0,35s); `declareAndValidateParams()` đổi ràng buộc thành `dwell×report_period > always_timeout_sec(state/vehicle)`. Test `DwellNotExceedingVehicleStateGraceThrows` thay `DwellShorterThanVehicleStateCadenceThrows` — cấu hình check CŨ từng chấp nhận silently nay THROW. 6 chỗ văn bản sửa cùng lượt (R35): `.hpp` (comment `kPreflightDwellTicks`), `.cpp` (comment+require), test, contract §2.20, package-status (dòng TÓM TẮT — xem V7 dưới — + dòng N3 ngay trên), README §b |
| **V1** | Hàng sentinel (`topic==""`, vd `state/vehicle`) thoát `require()` QoS trong `parseWatchGroup()` dù qos đó **vẫn dựng subscription thật** ở ctor (`qosFromProfile(vehicle_item.qos_profile, ...)`) | `require` QoS nhấc ra khỏi `if (!topics[i].empty())`, unconditional như `depth` đã làm — comment `qosFromProfile()`'s "validate() already forbids…" nay ĐÚNG cho mọi hàng, không cần sửa chữ |
| **V5** | Lá chắn `require(it->topic.empty())` chống tái sinh reader thứ 2 trên `state/vehicle` (N2) chưa có test (R33) | Test mới `VehicleStateEntryMustStaySentinelThrows`: gán topic/type THẬT cho hàng `state/vehicle` (giữ cặp hợp lệ để không bị luật ghép cặp "both-empty-or-both-set" ném trước) ⇒ `EXPECT_THROW` |
| **V2** | `publish_period_sec` chép tay 4 nơi (yaml/`diagnostics_node.cpp` default/`verify_observability.sh`/`preflight_light.py`) — họ R34 | `verify_observability.sh`'s `round_o4_gate()` derive `O4_PUBLISH_PERIOD_SEC` từ chính yaml (python3+PyYAML), WARN nếu lệch giá trị chép tay dự phòng — chỉ chạy khi round `o4-gate` được gọi (không thêm phụ thuộc PyYAML cho round khác). `preflight_light.py` thêm `--params-yaml` + `derive_default_max_age_sec()` (2×`publish_period_sec`), WARN+fallback 2,0s nếu không đọc được |
| **V3** | `onSetParameters()` trả `successful=true` cho MỌI param khác `gate_mode_override` dù node không bao giờ đọc lại chúng (`perception_enabled` và toàn bộ config mảng always/diag_source/waiver) | Nhánh `else` mới: `successful=false, reason="read-only after startup"` cho mọi tên khác `gate_mode_override` — KHÔNG làm chúng sống động (tắt `perception_enabled` giữa lúc bay còn tệ hơn nói dối). Test mới `SetParameterOnReadOnlyFieldReportsFailure` |
| **V7** | Đoạn TÓM TẮT P10.9b ở trên (mục "Cơ chế mới") còn ghi **"(armed)"** + **"hạ dwell 3 tick"** — mâu thuẫn bảng N1/N3 cách đó 20 dòng, với code/contract/README thật | Sửa lại `(armed && connected)`, dwell 7 tick (N3→V4), kèm 1 dòng trỏ V6 |
| **V8** | Dòng "D4 (o4-gate) PHẢI BAY LẠI — chưa chạy lại" (mục D4 ở trên) mâu thuẫn P10.9c B3 đã bay lại | Sửa trạng thái tại chỗ, **giữ số cũ để truy nguyên**, thêm số P10.9c B3 (6/6 ×2 lượt, samples=23–40 ≥ min 12, (b) 0,534/0,414 s) |

**Cổng sau vá (2026-08-24):** build 0 warning · `uav_observability` **163 case / 6 target, 0 lỗi, 2 lượt liên tiếp** (161→163, +2: `VehicleStateEntryMustStaySentinelThrows` V5 · `SetParameterOnReadOnlyFieldReportsFailure` V3; `DwellShorterThanVehicleStateCadenceThrows`→`DwellNotExceedingVehicleStateGraceThrows` là ĐỔI TÊN cùng 1 test, không phải case mới) · `diff -rq` Windows↔WSL IDENTICAL · `check_doc_links.sh` 309 link 0 gãy.

🔴 **`o4-gate` chạy lại 1 lượt sau V4+V6 — (a) KHÔNG đổi khỏi GO (yêu cầu chặn của review), nhưng OVERALL vẫn FAIL(1) vì (b) — PHÁT HIỆN MỚI, KHÔNG do V4/V6:** (a) baseline vẫn `GO` trước kill (PASS, không đổi). (b) SIGKILL `localization_mux_node`: chuyển `NO_GO` trong **0,383 s** (< trần 2,0 s, PASS phần thời gian) nhưng `worst_item` ra `'diagnostics/localization:localization: fused output'` (child Sub-B do `localization_health_node.cpp:172` phát) — KHÔNG nằm trong `O4_ALLOWED_WORST` hiện chỉ liệt 2 tên Sub-A (`state/odometry_fused`, `state/localization_status`) ⇒ FAIL đúng `worst_item_ok`. (c)/(d)/(e)/(f) đều PASS như cũ. **Đã xác nhận bằng đọc code KHÔNG phải do V4/V6:** V6 chỉ chèn hàng khi `phase_source==kUnmeasuredStrict` — không kích hoạt ở kịch bản (b) vì `state/vehicle`/PX4 link không hề bị đụng tới; V4 chỉ chạm nhánh dwell pha armed/preflight, không liên quan thang điểm Sub-B của localization. Đây là **race đã được CHÍNH thiết kế panel cảnh báo trước** (`.claude/plan/P10-gonogo-design-panel.md:225`: *"O4_ALLOWED_WORST chỉ liệt 2 tên Sub-A nên có thể rơi vào diagnostics/safety:LOCALIZATION_INVALID ⇒ phải mở rộng allow-list trước khi chạy"*) — bộ dò NỘI DUNG (`localization_health_node`, Sub-B) có thể thắng bộ dò SỐNG-CÒN (Sub-A cadence timeout) khi `localization_mux_node` chết, tuỳ nhịp lập lịch. **KHÔNG tự ý mở rộng `O4_ALLOWED_WORST`** trong phiên này (thay đổi allow-list là quyết định ngang cấp — cùng tinh thần F3/G6 của chính panel) — ghi làm nợ mới, chờ chủ dự án quyết mở rộng allow-list hay coi là race chấp nhận được.

**Nợ ghi lại (KHÔNG sửa, theo đúng chỉ định review lượt 3):**
- 🟡 **N6 chưa ai kiểm** (thuộc P6/`uav_bringup`, KHÔNG chặn P10): `discovery_probe_topic` của `gn4b_avoidance_gate.py`/`gn5_followtrack_gate.py` (review lượt 2's N6 vá) mới được verify bằng cách trích xuất hàm qua `ast`+`exec()` với `node`/`rclpy` GIẢ — **chưa từng chạy trên ROS graph thật**. Ghi ở đây để lần chạy G-N4b/G-N5 kế tiếp (P6, không phải P10) kiểm luôn phần này cùng lúc.
- 🟡 **Nợ mới (review lượt 3, phát hiện khi chạy lại o4-gate): `O4_ALLOWED_WORST` quá hẹp cho (b)** — xem đoạn ngay trên. Không sửa trong phiên này (ngoài phạm vi 8 VÀNG, đụng allow-list đã ký).

### Nợ / giới hạn còn mở sau P10.9b

| Nợ | Nội dung |
|---|---|
| **Cửa sổ mù mở-nguồn → ARM** | Miễn-trừ preflight bay hơi ≤1 tick khi armed lật, nhưng subsystem có thể hỏng đúng kiểu vị ngữ đang đo trong khoảng hẹp đó. Giảm thiểu bằng quy trình (checklist P11 đọc lại đèn SAU arm TRƯỚC takeoff), không có biện pháp kỹ thuật triệt để — chấp nhận có ý thức (F1, xem design panel §5) |
| **`in_air` không dùng được cho vị ngữ pha** | `px4_state_adapter_node.cpp`'s `in_air = takeoff_time > 0` không reset sau hạ cánh (nợ #2 P8, CHƯA đóng) — muốn dùng phải sửa `uav_px4_backend` trước, ngoài phạm vi P10.9b |
| **`ESTIMATOR_INPUT_INVALID` GPS-denied indoor** | AND với `gps_ok` (`failsafe_policy.cpp:542`, xác nhận đọc code) ⇒ ERROR vĩnh viễn ở `uav_arena_indoor`, kể cả đang bay — CỐ Ý không miễn-trừ ở P10 (miễn-trừ sẽ giấu hỏng GPS thật ngoài trời). Chưa chạy lại trên world indoor trong phiên này — chờ P8/backend quyết |
| **Nợ #10 camera bridge sim** | `delivered_fraction` 0,40/0,45 — `o4-gate` (c) CỐ Ý giữ NO_GO cho tới khi vá, cấm nới bằng waiver/owner |
| ✅ **(h) đối chứng đọc đèn ĐÃ CHẠY** | `preflight_light.sh` sống: `GO age=0,868s gate_mode=preflight(measured)`. Sau `SIGKILL diagnostics_node` + 3 s: subscriber MỚI (tạo sau khi tiến trình đã chết) nhận **0 mẫu** — DDS gỡ writer chết khỏi discovery trước khi subscriber mới kết nối ⇒ script in `FAILED TO MEASURE` (exit 2), KHÔNG BAO GIỜ đọc nhầm mẫu latched thành GO. Mạnh hơn dự đoán "in STALE" của plan (đường STALE trong code vẫn còn — kích hoạt khi có mẫu nhưng quá tuổi — nhưng ca thật rơi vào đường "0 mẫu" trước) |
| ⏳ **G-O4 (g)/(g2) SITL bay thật CHƯA CHẠY** | Đòi arm→takeoff→hover thật + SIGKILL nguồn lệnh đang bay (thực tế là tiến trình `smoke_flight.py` phát `cmd_test`, KHÔNG phải node `cmd_mission` như plan giả định — chưa kiểm chứng lúc viết plan) + log PX4 xác nhận vào failsafe, cộng tiêm một latch HOLD/INHIBIT còn sống lúc đậu bãi để xem NW1 chặn GO trên dây thật. Quyết định có chủ đích: KHÔNG chạy trong phiên P10.9b này — bằng chứng đa lớp đã có (44 unit test + D0 thật + o4-gate 6/6 PASS 2 lượt + cổng (h)) đã chứng minh cơ chế đúng ở tầng đo được; (g)/(g2) là bằng chứng CỘNG THÊM cho luồng bay thật, không phải điều kiện để P10.9b đóng. Việc kế tiếp nếu làm: viết script bay dựa trên `smoke_flight.py` (nguồn TEST không phải MISSION), kill tiến trình đó giữa hover, đối chiếu PX4 log |

### Giả định & bẫy cho người dùng package (P10.0–P10.8, còn hiệu lực)

| Bẫy | Vì sao | Đọc thêm |
|---|---|---|
| **Hộp đen mặc định `sqlite3`, không phải `mcap`** | `mcap` mất 100% bag khi tắt máy không sạch (G-O1); `sqlite3` phục hồi 10/10 qua `ros2 bag reindex` | §6c ở `P10-observability.md`, contract §2.20 |
| **Bag sau sự cố PHẢI `ros2 bag reindex`** | `metadata.yaml` chỉ ghi lúc đóng sạch | contract §2.20 |
| **`worst_item`/`aggregated` Sub-B dùng tên child THẬT trên dây** | `<source>:<child>` với `<child>` giữ nguyên tiền tố node nguồn tự đặt (vd `"safety: OFFBOARD_UNHEALTHY"`, KHÔNG rút gọn) — D0 nạp waiver đúng chuỗi này | contract §2.20, `diagnostics_node.cpp` |
| **Đọc đèn phải qua `scripts/preflight_light.sh`, không `ros2 topic echo --once`** | `/state/system_health` TransientLocal — echo trần có thể đọc mẫu latched của node đã chết | `scripts/preflight_light.py` |
| **`gate_mode_override` chỉ SIẾT, không NỚI** | `auto`/`preflight` không ép được gì, chỉ `flight` có tác dụng — tránh lỗ mù "ai đó ghi tay preflight giữa lúc bay" | contract §2.20 |

Mục tiêu, node, cổng kiểm chứng của từng phase → `../.claude/plan/overviewPlan.md` (P4–P10).

Ranh giới "hình học thuần = đầu tư · DNN = chỉ khung" đã được P5 tuân thủ trọn (xem §6); giữ nguyên ranh giới đó cho mọi phase sau.

Điểm sát an toàn cần để tâm khi tới:
- **P8 ✅ ĐÓNG TRỌN 2026-08-21 (sổ nợ = 0 từ 2026-08-22)** — chính sách "mất định vị" đã CHỐT (R1, §0b):
  **INHIBIT (trả quyền, KHÔNG auto-land)**; HOLD (policy 3, obstacle) đã enforcement thật + bay thật PASS (G-S3-HOLD) — xem §10 ở trên.
- **P6 còn lại** — planner chỉ được đọc `odometry_fused` (R4); `/world/obstacle_map_local` **im lặng khi nguồn câm** nên phía tiêu thụ phải bắt bằng timeout, không đọc thành "không có gì để né".
