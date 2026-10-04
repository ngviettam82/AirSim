# `uav_safety`

Giám sát an toàn cấp cao. **KHÔNG thay thế failsafe lõi của PX4** — đây là một
lớp giám sát **bổ sung**, canh những tín hiệu ở mức hệ thống mà bản thân PX4
không nhìn thấy (định vị, pin, vật cản, đường offboard, độ tươi của lệnh, tính
toàn vẹn frame), và phản ứng bằng cách **giành hoặc trả quyền điều khiển** qua
`uav_control_authority`.

Thiết kế + quyết định đầy đủ: `.claude/plan/P8-safety.md`.

> 🇻🇳 *Ghi chú 2026-08-24: file này trước đây viết bằng tiếng Anh — đã dịch sang
> tiếng Việt theo R8. Lý do không phải hình thức: R18 nói phần **sát an toàn phải
> giảng SÂU hơn**, mà tài liệu an toàn chủ dự án đọc không trôi thì đúng là chế độ
> hỏng R18 canh. Mọi định danh code, tên topic/tham số, con số và tham chiếu
> `file:dòng` **giữ nguyên**.*

## Trạng thái (2026-08-22)

**P8 ĐÓNG TRỌN 2026-08-21** (P8.1–P8.6, review chốt "không còn CHẶN", 4 cổng bay
lại PASS). **Mọi nợ mang sang đã đóng 2026-08-22** (Y1/Y2/N3 sửa kèm test; Y3 ký
giữ-nguyên-hành-vi; R32 duyệt; N-review-1/2 vào contract §2.18; N-safety-b/c chấp
nhận, rà lại ở P11) — **sổ nợ rỗng**, xem `docs/package-status.md` §10.

`enforcement_enabled` (tham số, đến từ cờ launch `safety_enforcement`, **mặc định
`true` trong sim**) là cửa mở cho enforcement thật: các violation thuộc lớp
INHIBIT (`BLIND_COMMAND`, `BATTERY_PX4_NO_ACTION`, `FRAME_MISMATCH`) **xin latch
SAFETY** của trọng tài (`/control/set_authority`, retry ≤ 1 Hz) **thay vì publish
bất cứ thứ gì** — riêng cái latch đã đủ làm `command_selected` thoái hoá thành im
lặng, bỏ đói ngưỡng cắt 0,5 s của chính gateway cho tới khi PX4 vào failsafe.

**HOLD là thật từ P8.5** (policy 3, `OBSTACLE_TOO_CLOSE`): pose được **đóng băng**
lấy từ `odometry_fused` (phải tươi ≤ `pose_max_age_sec` tại thời điểm đóng băng,
không đủ tươi thì đi **thẳng** sang INHIBIT), stream trên `cmd_safety` ở
`hold_stream_hz` và **restamp mỗi tick**, không nội suy; HOLDING→INHIBITED là
**một chiều** khi mất pose.

**R5 (trọng tài):** latch SAFETY một khi đã cấp thì **không bao giờ tự hết hạn**
(cả nhánh grace lẫn timeout) — `clear_safety_latch` là đường gỡ **duy nhất**.
`false` vẫn có nghĩa là **không tồn tại** publisher/client nào, đảm bảo bằng cấu
trúc (xem mục "Node" bên dưới).

Bằng chứng bay sim (G-S2 4/4 PASS sau 2 lần sửa công cụ; G-S3; M5 3/3) nằm ở báo
cáo bàn giao P8.4/P8.4b và [`docs/package-status.md`](../../docs/package-status.md)
§10 — **không chép lại ở đây** (R9). Hai phát hiện từ bay thật đáng biết **trước
khi sửa code này**:

1. **`BLIND_COMMAND` không hề nổ trong cả 3 biến thể tiêm lỗi của G-S2** — lớp 1
   của chính navigator (Recover TYPE_HOLD) luôn đóng băng setpoint **trước khi**
   `blind_grace_sec` kịp trôi, đúng như thiết kế R1 ("không được đua với lớp 1").
   Đây là bằng chứng **lớp 1 hoạt động**, **KHÔNG** phải bằng chứng đường cắt của
   BLIND_COMMAND hoạt động — việc đó là của G-S1 (nó ép setpoint tiếp tục di
   chuyển thẳng qua một probe, đi vòng qua navigator).
2. Đường phát hiện gốc của `OFFBOARD_UNHEALTHY` đo được **chậm hơn failsafe của
   chính PX4 1,07 s** khi bay thật (**ngược** với kỳ vọng "sớm hơn PX4" ghi trong
   plan gốc) — P8.4b thêm đường phát hiện thứ hai, nhanh hơn (`selected_stale_sec`,
   xem dưới); đo lại: **nhanh hơn 0,52 s**.

| Bước | Nội dung | Trạng thái |
|---|---|---|
| P8.1 | Thư viện `failsafe_policy` | xong |
| P8.2 | Sửa trọng tài `uav_control_authority` (ngoại lệ latch SAFETY, `clear_safety_latch`) | xong |
| P8.3 | `safety_supervisor_node`, thuần quan sát | xong |
| P8.4 | Enforcement INHIBIT + đường gỡ ClearFault | xong, đã verify trong sim |
| P8.4b | `selected_stale_sec` (đường thứ 2 của OFFBOARD_UNHEALTHY) + verify lại G-S2/G-S3 | xong |
| P8.5 | Nhánh HOLD + R5 (latch SAFETY không tự hết hạn) | xong, verify bằng bay sim (G-S3-HOLD: onset 0,588 s, stream 19,99 Hz, pose trôi 0,000000 m) |
| P8.6 | Tài liệu + chốt review | xong (2026-08-21; nợ dọn sạch 2026-08-22) |

## Taxonomy — đúng 3 hành động

| Hành động | Nghĩa |
|---|---|
| `REPORT` | Phát violation + event. **Không bao giờ** chạm tới quyền điều khiển. |
| `HOLD` | Giành quyền, stream một pose **đóng băng** (chỉ lấy từ `odometry_fused`). Latch tới khi `ClearFault`. |
| `INHIBIT` | Giành quyền, **không publish gì cả**. Latch tới khi `ClearFault`. |

Không có gì trong package này ra lệnh land / RTL / climb — xem plan §10.

## Hợp đồng thư viện (`include/uav_safety/failsafe_policy.hpp`)

- `FailsafePolicy::evaluate(Measurements, now_sec)` — chạy đủ **12 bộ phát hiện**
  của plan §3, đẩy mọi đồng hồ grace/clear-stability tiến lên, và báo lại trạng
  thái FSM sau tick này.
- `confirmHoldEngaged(pose_age_sec, now_sec)` / `confirmInhibitEngaged()` — **bắt
  tay hai bước tường minh** (FSM đỗ lại ở `ENGAGING_HOLD` / `ENGAGING_INHIBIT` cho
  tới khi caller xác nhận; caller nào muốn **không thêm một chút trễ nào** thì xác
  nhận ngay trong cùng tick).
- `clearFault(fault_code, Measurements, now_sec)` — ba đường từ chối của plan §3.1
  (không đo được / nguyên nhân còn sống / bước nhảy bàn giao).

**Thư viện không bao giờ tự đọc đồng hồ** (R21); mọi `now_sec` đều do caller đưa
vào. Không include kiểu ROS nào (tinh thần R1 mở rộng cho package này: lõi policy
**không link** `rclcpp`/`uav_interfaces`, nhờ vậy ma trận an toàn của nó được
**ghim bằng test, không phải bằng một chuyến bay** — cùng kỷ luật với
`authority_arbiter` của `uav_control_authority`).

## P8.1 CỐ Ý để lại gì cho caller (node ở P8.3)

- Độ dịch chuyển trong cửa sổ W mà `BLIND_COMMAND` dùng
  (`command_setpoint_displacement_m`) và khoảng cách bước-nhảy-bàn-giao mà
  `ClearFault` dùng (`cmd_mission_setpoint_distance_from_fused_m`) là các **phép so
  hình học** mà thư viện này không bao giờ tự đệm hay tự tính — **node** tính chúng
  từ mẫu thật của `/control/command_selected` và `/state/odometry_fused` rồi truyền
  vào dưới dạng số vô hướng.
- Một số ngưỡng mà package này **không nhìn thấy được** (cổng độ-cũ-của-pose của
  chính navigator, `source_timeout_sec` của `authority_arbiter`,
  `max_lead_horizontal_m` của navigator) đang là **bản sao giữ tay** trong
  `FailsafePolicyParams` — `validate()` cưỡng chế các luật ghép cặp, nhưng **không
  có gì giữ cho bản sao đồng bộ với giá trị thật ngoài một checklist thủ công**
  (xem phần chân của `config/safety_params.yaml`).

Chi tiết build/test, kết quả mutation, và ghi chú diễn giải của 3 mã mà plan §3
phải hoãn vì thiếu blueprint (`ESTIMATOR_INPUT_INVALID`, `CAMERA_STREAM_UNHEALTHY`,
mức nghiêm trọng của `OBSTACLE_TOO_CLOSE`/`FRAME_MISMATCH`) nằm ở báo cáo bàn giao
P8.1, không chép lại ở đây (R9).

## Node (`safety_supervisor_node`, P8.3)

Subscribe **12 topic thật** (`state/localization_status`, `state/odometry_fused`,
`diagnostics/localization` — **KHÔNG** phải `state/localization_health`, lý do ở
điểm 1 của báo cáo bàn giao P8.3 — `world/obstacle_map_local`, `state/health_px4`,
`state/vehicle`, `backend/offboard_status`, `control/command_selected`,
`control/cmd_mission`, `diagnostics/control_authority`, `state/camera_health`,
`planning/trajectory`), dựng `Measurements` từ **tuổi tính theo thời điểm ĐẾN**
(không bao giờ theo `header.stamp`), và tính hai giá trị hình học mà P8.1 để lại
cho caller: độ dịch chuyển cửa sổ W của `BLIND_COMMAND` (ring buffer 3 tick, node
tự lấy mẫu trên `command_selected`) và khoảng cách bước-nhảy-bàn-giao của
`ClearFault` (`cmd_mission` so với `odometry_fused`).

Publish `safety/state` (5 Hz + on-change, `TransientLocal`), `safety/violations`
(**chỉ khi có cạnh**, `DiagnosticArray`), `diagnostics/safety` (1 Hz, dump đầy đủ).
Phục vụ service `safety/clear_fault`. **R24:** một nhóm `MutuallyExclusive` duy
nhất cho mọi subscription, timer tick 20 Hz, và service.

**`enforcement_enabled` khoá 4 đối tượng bằng CẤU TRÚC**: `cmd_safety_publisher_`,
`set_authority_client_`, `clear_safety_latch_client_` **chỉ được dựng bên trong**
`if (enforcement_enabled_) { ... }` ở constructor — đặt nó `false` thì **không cái
nào tồn tại** trong suốt vòng đời node, và điều đó **chứng minh được** bằng
`ros2 topic info control/cmd_safety --verbose` (0 publisher) hoặc `ros2 node info`
(không liệt kê service client nào), **chứ không phải bằng cách tin vào một cờ**.

**Cơ chế P8.4**: INHIBIT **không bao giờ publish** trên `cmd_safety` — nó gọi
`SetControlAuthority(SOURCE_SAFETY)` một lần (retry ≤ 1 Hz tới khi được cấp, qua
`async_send_request` + callback, **không bao giờ** `spin_until_future_complete`).
Riêng việc được cấp đã nâng sàn của trọng tài lên trên MISSION (ngoại lệ Y4 của
P8.2 làm latch SAFETY có hiệu lực **mà không cần bằng chứng còn sống**), nên
`command_selected` **âm thầm thoái hoá thành không có gì** — đó chính là **đường
cắt**. Phản hồi của `ClearFault` về phía caller là **deferred**: `handleClearFault`
dùng overload service callback chỉ-có-header của rclcpp (không auto-response), chỉ
gọi `send_response()` khi **hoặc** thư viện từ chối ngay, **hoặc** — sau một lần
clear trọn vẹn — chính lời gọi `clear_safety_latch` của trọng tài xác nhận.

**Cơ chế P8.5 (HOLD)**: khi vào ENGAGING_HOLD, node **đóng băng** pose từ
`odometry_fused` (có kiểm nội dung: hữu hạn, quaternion dùng được; pose cũ hoặc
rác thì **leo thẳng** lên INHIBIT), xin đúng cái latch SAFETY đó, rồi một timer
riêng (**cùng `io_group_`**, nên không bao giờ chạy đồng thời với `onTick`) stream
pose đóng băng trên `cmd_safety` ở `hold_stream_hz`, **restamp mỗi tick** — trọng
tài **vứt** lệnh có stamp cũ, đúng điều mutation A đã chứng minh. Mất pose giữa
chừng thì dừng stream và leo **một chiều** lên INHIBITED; latch sống sót qua khoảng
im lặng đó (R5).

**P8.4b**: `OFFBOARD_UNHEALTHY` (**REPORT-only, không bao giờ cắt**) có thêm đường
phát hiện thứ hai — `command_selected_age_sec > selected_stale_sec` (0,5 s, chọn
để nằm **hẳn trên** khoảng im lặng bàn giao hợp lệ tệ nhất đã được validate của
trọng tài, ≤ 0,4 s) — đo **thẳng** trên tuổi-thời-điểm-đến thô của
`command_selected`, **không** bị ảnh hưởng bởi release-dwell của trọng tài,
`command_timeout_sec` của gateway, hay cửa sổ làm mượt nhịp của `offboard_status`
— cả ba thứ đó **chồng lên nhau** thành độ trễ 2,66 s của đường gốc khi bay thật.
`validate()` cưỡng chế `selected_stale_sec >= downstream_command_timeout_copy_sec`
(V7).

**Script cổng bay** (chạy từ Windows bằng `wsl.exe -- bash <file>`, **không bao giờ
gõ inline**): `scripts/verify_safety.sh` (G-S2 tiêm mất định vị, cần
`uav0_nav_indoor`), `scripts/verify_obstacle_hold.sh` (G-S3-HOLD),
`scripts/verify_boot_no_latch.sh` (V1 — 30 s boot ở trạng thái disarmed **không
được** sinh latch nào; đối chứng dương của nó tiêm một vật cản và **kỳ vọng REPORT
mà KHÔNG HOLD**, chứng minh cổng armed-gate B5 trên dây). Probe nằm ở `test/*.py`,
**dùng chung domain với sim** (ngoại lệ của R20), và **không bao giờ** import
`px4_msgs` (R25).

Báo cáo bàn giao đầy đủ P8.1/P8.3/P8.4/P8.4b (cây file, số test, bảng mutation,
bằng chứng verify sim, ghi chú diễn giải) **không chép lại ở đây** (R9) — xem các
bản bàn giao cho coordinator; trạng thái đóng phase và nợ mở (Y1–Y3, N-review,
N-safety-d) nằm ở `docs/package-status.md` §10.
