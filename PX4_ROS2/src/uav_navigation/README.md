# `uav_navigation` — từ mục tiêu tới chuyển động

Package trả lời câu hỏi *"muốn tới đó thì bay thế nào"*. Mission (P9) chỉ gọi action ở đây, không tự
bay. Backend PX4 vẫn là nơi duy nhất chạm `px4_msgs` (R1) — package này chỉ phát message nội bộ.

> Kế hoạch & cổng kiểm chứng → `.claude/plan/P6-navigation.md`
> · Hợp đồng action → [`docs/interface-contract-v0.1.md`](../../docs/interface-contract-v0.1.md) §2.8–§2.10

**Trạng thái:** ✅ **P6 ĐÓNG TRỌN 2026-08-20** (5 cổng bay PASS trong một phiên: G-N4b · G-N5 · G-N6×2 · M5 · G-N2). **270 case / 11 target**, 0 lỗi. P6.1–P6.6 đã hiện thực — `navigator_action_server_node` với **7 action**; GotoPose /
FollowPath / Recover bay quỹ đạo B-spline (§6c), `route_planner_node` (§6e) và `local_planner_node`
(§6f) chạy riêng.

> 🔴 **Lệch có chủ đích so với `CLAUDE.md` §4.6: KHÔNG có `recovery_planner_node` riêng.** `Recover` là
> action server thứ 7 **trong chính navigator**, dùng lib `recovery_planner` (ROS-free). Lý do giống
> hệt lý do không tách `trajectory_generator_node` ([contract §2.14](../../docs/interface-contract-v0.1.md)): `/planning/trajectory` là
> topic **latched một publisher duy nhất**, và recovery phải đi qua **cùng** bộ phát setpoint — hai
> node cùng phát là hai "kế hoạch hiện hành" mâu thuẫn cho reader vào muộn. Xem §6h.

---

## 1. `navigator_action_server_node`

| Action | Tên (R2) | Kiểu |
|---|---|---|
| Cất cánh | `/uav/<id>/planning/takeoff` | `uav_interfaces/action/Takeoff` |
| Bay tới toạ độ | `/uav/<id>/planning/goto_pose` | `GotoPose` |
| Giữ vị trí | `/uav/<id>/planning/hold_position` | `HoldPosition` |
| Bay theo path của caller | `/uav/<id>/planning/follow_path` | `FollowPath` (§6g) |
| Bám mục tiêu | `/uav/<id>/planning/track_target` | `TrackTarget` (§6g) |
| **Khôi phục** | `/uav/<id>/planning/recover` | `Recover` (§6h) — **action DUY NHẤT được preempt** |
| Hạ cánh | `/uav/<id>/planning/land` | `Land` |

**Đọc:** `/state/odometry_fused` (R4 — nguồn vị trí DUY NHẤT), `/state/localization_status`
(độ tin cậy của chính pose đó — xem §6b), `/state/vehicle`, `/backend/offboard_status`.
**Ghi:** `ControlCommand` MODE_POSITION lên `/control/cmd_mission`, `source = SOURCE_MISSION`,
`frame_id = "odom"`, 20 Hz · `Trajectory3D` lên `/planning/trajectory` (**latched**, xem §6c).
**Gọi service:** `/backend/arm`, `/backend/disarm`, `/backend/set_mode`.

> ✅ **P7.3 (đã xong):** navigator phát lên `/control/cmd_mission`, không phải
> `/control/command_selected` nữa. `control_authority_manager_node` (`uav_control_authority`) là
> **single writer duy nhất** của `/control/command_selected` — nó trọng tài 4 kênh nguồn
> (`cmd_safety`/`cmd_operator`/`cmd_mission`/`cmd_test`) rồi rơ-le một kênh thắng ra `command_selected`
> cho gateway đọc. Node navigator không đổi gì bên trong, chỉ đổi tên topic phát ra.
>
> 🔴 **`/planning/trajectory` KHÔNG nằm trên đường điều khiển.** Nó là bản kế hoạch để quan sát /
> ghi log / mission đọc. Thứ lái máy bay là `command_selected` (do trọng tài phát), vì vòng dây xích
> cần **pose đo được** nên `cmd_mission` phải ở cùng executor với bộ phát setpoint bên navigator.

## 2. Máy trạng thái & mutual exclusion

```
IDLE ──Takeoff──▶ TAKING_OFF ──▶ HOLDING ──GotoPose──▶ GOING_TO_POSE ──▶ HOLDING
                                    │  ├──HoldPosition──▶ HOLD_TASK ──▶ HOLDING
                                    └──Land──▶ LANDING ──▶ IDLE
```

**Chỉ `IDLE` và `HOLDING` nhận goal mới** — **trừ `Recover`** (§6h). Mọi goal khác tới trong lúc đang
chạy task bị **REJECT ngay ở goal callback** (skill `ros2-action-mission-architecture`, Algorithm 1)
— không xếp hàng, không preempt ngầm. Bảng đầy đủ:

| Trạng thái | Takeoff | GotoPose · Hold · FollowPath · TrackTarget | Land | **Recover** |
|---|---|---|---|---|
| `IDLE` (dưới đất) | ✅ | ❌ chưa bay | ❌ chưa bay | ❌ chưa bay |
| `TAKING_OFF` / `GOING_TO_POSE` / `HOLD_TASK` / `FOLLOWING_PATH` / `TRACKING_TARGET` | ❌ bận | ❌ bận | ❌ bận | ✅ **preempt** |
| `RECOVERING` | ❌ bận | ❌ bận | ❌ bận | ❌ đang khôi phục rồi |
| `LANDING` | ❌ bận | ❌ bận | ❌ bận | ❌ autopilot đang cầm lái |
| `HOLDING` (hover rảnh) | ❌ đang bay rồi | ✅ | ✅ | ✅ |

**Muốn cắt ngang một task → `cancel`, rồi gửi goal mới.** Cancel trả về gần như tức thì và để lại
máy bay ở `HOLDING`, nên đây là đường đi hợp lệ chứ không phải đường vòng.

**Cancel = chuyển HOLD, KHÔNG bao giờ buông tay lái.** Setpoint bị *đóng băng tại chỗ nó đang
đứng* (không nhảy về vị trí đo được): máy bay đi nốt vài chục cm còn lại rồi dừng. Chọn đóng băng
thay vì đặt setpoint = vị trí hiện tại vì cách sau tạo **bước nhảy setpoint** — nguồn giật, trái
yêu cầu quỹ đạo mượt (R0). Stream 20 Hz **vẫn chạy** sau cancel; ngừng stream giữa trời là rớt
offboard.

**Cancel khi đang `LANDING` bị TỪ CHỐI.** Lệnh LAND đã trao quyền cho autopilot; giành lại giữa
chừng vi phạm quy tắc "không giành quyền khi autopilot cầm lái"
([`docs/package-status.md`](../../docs/package-status.md) §2).

## 3. Enforcement ở cổng nhận goal (R0)

| Luật | Xử lý |
|---|---|
| `GotoPose.frame_id != "odom"` | REJECT — [hợp đồng v0.1](../../docs/interface-contract-v0.1.md) §2.8, không đoán, không tự convert |
| Toạ độ / bán kính / tốc độ không hữu hạn | REJECT |
| `acceptance_radius <= 0` | Dùng mặc định `acceptance_radius` (0,3 m) |
| `max_speed <= 0` | Dùng mặc định `max_speed`; lớn hơn trần cấu hình thì **kẹp xuống trần** |
| `z` ngoài `[min_altitude_m, max_altitude_m]` | REJECT — hàng rào thô, geofence thật là P8 |
| `target_altitude <= 0` hoặc `> max_takeoff_altitude_m` | REJECT |
| `Land.use_precision_landing = true` | REJECT — chưa hiện thực, **không im lặng bỏ qua** |

## 4. Chuỗi engagement (chép đúng đường đã bay thật ở M5)

1. Đọc vị trí hiện tại từ `odometry_fused`, neo setpoint tại đó.
2. Bật stream 20 Hz → backend có cái để phát, session manager bắt đầu priming.
3. Xoá trạng thái offboard đã nhớ (ACTIVE cũ sẽ **thoả mãn nhầm** phép chờ) → chờ
   `/backend/offboard_status` báo `STATE_ACTIVE`.
4. Gọi `/backend/arm`, chờ autopilot xác nhận.
5. Carrot leo tới cao độ goal.

🔑 **Vào offboard TRƯỚC, arm SAU** (`memory` §5, sửa 2026-08-10): đảo lại là khoá chết chuyến bay
trong nhà. Có unit test riêng ghim thứ tự này, và đã kiểm bằng **đối chứng dương**: bỏ bước chờ
offboard thì test đó FAIL đúng câu *"armed before offboard was confirmed"*.

**Cancel trong lúc engagement** (bước 1–3) được kiểm ở từng vòng chờ, không đợi tới lúc bay: huỷ
lúc đó thì **không arm**, ngừng stream, về `IDLE`. Sau khi đã arm thì cancel đi đường thường —
chuyển HOLD.

Hạ cánh đi ngược lại: `set_mode(LAND)` → chờ xác nhận → **đợi 1 s rồi mới ngừng stream** (ngừng
trước khi đổi mode là trượt failsafe) → chờ tự disarm; quá `disarm_timeout_sec` mới gọi
`/backend/disarm`.

## 5. Mô hình đồng thời — ĐỌC TRƯỚC KHI SỬA NODE NÀY

Cùng bài học với `px4_command_gateway_node`: **rớt nhịp stream là rớt offboard giữa trời.**

| Nơi chạy | Chứa | Vì sao tách |
|---|---|---|
| `stream_group_` (mutually exclusive) | timer 20 Hz + sub odometry/vehicle/offboard | Không callback nào ở đây ngủ, nên nhịp stream không bao giờ bị chặn |
| `action_group_` (reentrant) | 4 action server + 3 service client | Goal callback phải trả lời ngay cả khi task đang chạy |
| **Thread riêng** (không thuộc executor) | thân mỗi task | Task ngủ hàng chục giây chờ arm/mode/đến nơi — để trong executor là ăn mất thread |

Biến qua ranh giới:
- `motion_mutex_` giữ setpoint + vị trí **+ lưới quỹ đạo** (P6.2). **Không có gì bên trong khoá này
  chờ đợi** (không service, không sleep, không publish) — publish làm sau khi nhả khoá.
- 🔑 **`streamTick()` KHÔNG được gọi `setTarget()`/`setTrajectory()`**: nó đang giữ `motion_mutex_`
  và `std::mutex` không đệ quy ⇒ deadlock ngay trong timer 20 Hz = ngừng phát setpoint giữa trời.
  Nó ghi thẳng `target_`/`target_yaw_` dưới khoá đang giữ.
- `setTrajectory()` nhận `Trajectory&&` (move, không copy lưới) và **đẩy lưới cũ ra ngoài khoá rồi
  mới để nó chết** — đây là chỗ duy nhất có thể giải phóng megabyte gần đường stream.
- `/planning/trajectory` **chỉ publish từ thread task**. Publish RELIABLE có thể chờ tới
  `max_blocking_time`; đặt nó trong timer là đặt một chỗ chờ lên đường 20 Hz.
- `offboard_state_`, `armed_`, `flight_mode_`, `connected_`, `failsafe_active_` là `std::atomic` vì
  subscription ghi ở stream group còn thread task đọc.
- Một task tại một thời điểm, nên chỉ có một worker thread; `admission_mutex_` đóng khe hở
  "hai goal cùng lúc đều thấy HOLDING".

## 6. Carrot setpoint — tầng dây xích của MỌI đường bay

**P6.2 không thay carrot, nó chỉ đổi nguồn của `target_`:** `GotoPose` lấy target từ mẫu quỹ đạo,
còn Takeoff / HoldPosition / Land / cancel vẫn đặt target trực tiếp. Cả hai đều đi qua
`advanceCarrot()`, nên dây xích + bộ phát hiện stall + cổng σ (§6b) áp cho **tất cả**.

Mỗi tick setpoint bước về target: ngang ≤ `max_speed·dt`, dọc ≤ `max_vertical_speed·dt`, không bao
giờ vượt qua target. Yaw quay theo giới hạn `max_yaw_rate_rad_s`, đi đường ngắn qua ±π.

**Dây xích, NGÂN SÁCH TÁCH THEO TRỤC:** setpoint không được chạy xa máy bay quá
`max_lead_horizontal_m` (0,8) theo phương ngang và `max_lead_vertical_m` (0,6) theo phương đứng.
Máy bay tụt lại (gió, bão hoà lực đẩy) mà setpoint cứ chạy thì sai số vị trí phình ra, và lúc bắt
kịp là một cú lao. Xích chặn tiến nhưng vẫn cho setpoint **lùi về phía máy bay**.

🔴 **Vì sao phải tách trục:** một vô hướng 3D duy nhất thì **lệch đứng khoá luôn tiến ngang** — bay
đứng yên tại chỗ vì cao độ không lên được. `hypot(0,8; 0,6) = 1,0` nên trần 3D vẫn đúng bằng mức cũ.

⚠️ **Ngoại lệ khi bay quỹ đạo (P6.2):** ngân sách vẫn tách ở tầng carrot, **nhưng đồng hồ quỹ đạo là
một vô hướng** — kẹp ở bất kỳ trục nào cũng dừng cả kế hoạch, vì `sample(t)` là một điểm 3D, không
thể tiến x/y mà không tiến z. Đây là hệ quả không tránh được của tham số hoá theo thời gian; chặn
trên vẫn là `leash_stall_fault_sec` (3 s).

**Dây xích phải KÊU và có HẠN.** `advanceCarrot()` trả cờ `clamped_horizontal`/`clamped_vertical`.
Mỗi tick bị chặn được đếm; bị chặn **liên tục** quá `leash_stall_fault_sec` (3,0) → FAULT
`ABORTED_PLANNER_FAILED` kèm chi tiết *"setpoint clamped by leash for N s, lead h X v Y"*, rồi
freeze + HOLDING (không sinh chuyển động mới, không giành quyền). Trước bản vá này, máy bay không
theo được thì node **treo câm cho tới hết timeout, không một dòng log** — đúng cái đã xảy ra ở
G-N1 lần đầu. Nay: WARN 1 Hz kèm `lead`/`commanded_z`/`position_z`, INFO 5 s khi bình thường, và
**tỉ lệ % thời gian bị chặn đi vào `result->message`**.

Lưu ý: kẹp *một nhịp* khi đang đuổi theo là bình thường; chỉ **liên tục** mới là hỏng. Setpoint đã
hội tụ (hold) không bao giờ bị tính là kẹp, vì nó không đòi di chuyển.

🪤 **Hai dương tính giả đã trả giá ở G-N1 lần 2 — đọc trước khi chỉnh đồng hồ stall:**

1. **Quay động cơ trên mặt đất KHÔNG phải stall.** Sau khi arm, PX4 còn giữ máy bay tại chỗ
   (spool-up + land detector), `position_z` đứng yên đúng một chỗ vài giây → setpoint bị kẹp →
   *mọi* chuyến takeoff bị abort oan. Đồng hồ stall **giữ ở 0** trong `takeoff_stall_grace_sec`
   (5,0) đầu mỗi task. Vẫn đếm % kẹp cho phần bằng chứng, chỉ không tính là hỏng.
   *Vì sao dùng ân hạn theo thời gian chứ không dùng `VehicleState.in_air`:* cờ đó suy từ
   `takeoff_time > 0` và **sai (kẹt true) sau khi hạ cánh** (`docs/package-status.md` §2) — chuyến
   thứ hai trong cùng phiên sẽ mở cổng ngay và dương tính giả quay lại; còn nếu nó kẹt false thì
   stall thật **im lặng vĩnh viễn**. Ân hạn theo thời gian đơn điệu: nó chỉ có thể **trễ** cảnh
   báo một khoảng có chặn, không bao giờ tắt hẳn.
2. **Task mới KHÔNG được thừa kế đồng hồ của task cũ.** Lần đó goal mới chết sau **1 ms** vì đọc
   trúng 3,1 s của task trước, kéo theo (d) không có mẫu feedback nào và (e) đo nhầm. Nay
   `beginTask()` xoá đồng hồ + thống kê ngay tại chỗ task đặt target mới, **dưới cùng `motion_mutex_`
   với timer stream** — không còn dựa vào giả định "thế nào chả có một nhịp không kẹp xen giữa".
   ⚠️ Reset nằm ở `beginTask()`, **cố ý không nằm trong `setTarget()`**: P6.2 sẽ gọi `setTarget` mỗi
   nhịp sampling trajectory, đặt reset ở đó là vô hiệu hoá bộ phát hiện stall.

## 6b. Ngưỡng "đã tới" phải lớn hơn sàn nhiễu của chính pose (P0-E)

Node đóng vòng vị trí trên `odometry_fused`, nên nó **phải đọc mức tin cậy của chính tín hiệu đó**
(`/state/localization_status`, mặt tiêu thụ của ràng buộc trung tâm P4):

| σ (`position_uncertainty`) | Xử lý |
|---|---|
| σ < `acceptance_radius` | Giữ nguyên ngưỡng |
| σ ≥ `acceptance_radius` | WARN + nới ngưỡng thành `max(acceptance_radius, k·σ)`, `k` = `acceptance_sigma_factor` (2,0), trần `max_acceptance_radius` (1,5 m); ngưỡng đã dùng ghi vào `result->message` |
| σ > `max_acceptance_radius`, hoặc `is_valid == false` | **REJECT** goal (chưa chạy) · **FAULT `ABORTED_LOST_LOCALIZATION`** (đang chạy) |

🔴 **Land KHÔNG bị chặn bởi cổng này** — hạ cánh là đường xuống an toàn bất kể định vị nói gì;
chặn nó là nhốt máy bay trên trời.

**Vì sao:** cổng G-N1 lần đầu chạy trên model `uav0` (bridge chỉ có `/clock`, không có nguồn vision)
→ mux rơi về GPS với **σ_z = 0,5 m**, trong khi ngưỡng "đã tới" là 0,3 m giữ liên tục 1 s. Ngưỡng
**nằm dưới sàn nhiễu của đầu vào** ⇒ takeoff không bao giờ "đạt" → treo tới timeout. Trên drone
thật bay GPS-only thì đây là *săn đuổi vĩnh viễn hoặc tuyên bố đã tới sai* — hỏng âm thầm. Bản vá
biến nó thành một câu thông báo, và cấm bay tiếp khi định vị tệ hơn trần.

`advanceCarrot()` và `resolveGotoLimits()` là chỗ P6.2 đã cắm trajectory sampling vào: chỉ đổi nguồn
của `target_`, phần còn lại của node không phải sửa.

## 6c. Lõi quỹ đạo (P6.2) — `trajectory.hpp` / `trajectory.cpp`

B-spline bậc 3 **đều**, điểm mút nhân ba nên hai đầu **chính xác** và **đứng yên**. 22 unit test.
Thư viện thuần số học (ROS-free); cách nó được nối vào node ở §6d.

🔴 **Mọi segment mang CÙNG một duration `T/n`.** Bản viết dở ban đầu chia duration theo độ dài cung —
nghe hợp lý và **sai**: đó là phép đổi tham số tuyến tính *từng khúc*, nên `dP/dt = P'(s)/T_i` **nhảy
bậc tại mọi mối nối**. Đo trên đường thẳng 10 m: vận tốc nhảy **1,5 → 0,375 m/s trong một tick 20 ms
= 56 m/s² ≈ 5,7 g**, trong khi `peakAcceleration()` vẫn báo **0,897 m/s²** vì nó đọc đạo hàm giải tích
*trong lòng* segment — mù hoàn toàn với bậc thang ở biên. Chia đều làm bước nhảy về **đúng 0** và còn
nhanh hơn (15 s thay vì 20 s cho 10 m).

🔑 **Hệ quả cho test: đo từ VỊ TRÍ ĐƯỢC PHÁT, đừng đọc trường `acceleration`.** Chính trường đó là thứ
đã che khuyết tật trên, nên một khẳng định đặt lên nó có thể xanh trong khi chuyển động bị lệnh vượt trần.

**Trần thời lượng suy trực tiếp, không dò:** kéo dãn hệ số `k` chia tốc độ cho `k` và gia tốc cho `k²`,
nên `T` là công thức đóng — không còn vòng lặp 5 lượt vốn *không phải ánh xạ co* (zigzag 12 waypoint
từng cạn lượt rồi trả *"could not fit"*). `duration` được **làm tròn lên bội của `gridPeriod`** vì
`sample()` đi trên lưới cách đều từ t=0; làm tròn lên chỉ khiến bay chậm hơn, không bao giờ nhanh hơn.

⚠️ **Mũi quay TRỄ ở khúc cua, có chủ đích** — đọc `peakYawLag()`. Kéo dãn cả tuyến để mũi kịp quay thì
**một khúc cua gắt làm chậm toàn bộ chuyến bay**, và với `goto_timeout_sec` 120 s kết cục là GotoPose
bị abort giữa trời — tệ hơn hẳn mũi lệch, vốn vô hại với multirotor. Trần `yaw_rate` vẫn được bảo đảm
**theo cấu trúc** vì `stepYaw` kẹp từng nhịp.

⚠️ **Waypoint TRUNG GIAN chỉ được tiến gần, không được đi qua** (spline xấp xỉ, không nội suy). Góc bị
cắt đúng `|W0 − 2·W1 + W2| / 6` — góc vuông cạnh 5 m cắt **1,18 m**. **P6.3 phải cộng con số này vào
inflation**; bao lồi của *tập waypoint* KHÔNG phải hành lang quanh polyline.

## 6d. GotoPose bay quỹ đạo thế nào (P6.2 — T1.3)

```
executeGoto → applyAcceptance → startGotoMotion → beginTask → runProgressLoop
                                      │
                       Trajectory::build(carrot hiện tại → đích)
                          ├── dựng được → publish /planning/trajectory → setTrajectory()
                          └── không     → WARN + setTarget() (carrot) + phát bản is_valid=false
```

| Điểm | Vì sao đúng thế |
|---|---|
| Gốc quỹ đạo = **setpoint đang giữ** (`currentCarrot`), không phải pose đo được | Lấy pose đo được là tiêm nhiễu định vị vào gốc ⇒ setpoint giật ngay lúc nhận goal. Lấy carrot thì `sample(0) == carrot` ⇒ **không bước nhảy theo cấu trúc** |
| `initial_yaw` = **`commandedYaw()`** | Lõi tự slew từ mốc này; đưa yaw đo được vào là bơm một bước nhảy yaw ở tick đầu |
| Gốc + yaw đọc trong **một lần khoá** (`motionAnchor`) | Hai lần khoá cho timer chen vào giữa ⇒ gốc của tick T ghép với yaw của tick T+1 |
| Đồng hồ quỹ đạo **chỉ tiến khi tick trước không bị kẹp** | Quỹ đạo tham số theo *thời gian*: máy bay tụt lại mà mẫu vẫn chạy thì kế hoạch bỏ rơi máy bay. Cờ `clamped` chỉ có SAU `advanceCarrot` nên phải nhớ trạng thái tick trước |
| Đồng hồ là **đồng hồ ROS** (`/clock`) | Nợ #1 memory §7: `use_sim_time` mà timer chạy đồng hồ thực thì nhịp offboard tụt dưới 2 Hz theo giờ sim |
| `max_duration` bị kẹp xuống **0,8 × `goto_timeout_sec`** | Kế hoạch dài hơn thời gian cho phép thì **không được dựng**, thay vì bay 2 phút rồi abort giữa trời. Xa quá → rơi về carrot (carrot đi thẳng ở `max_speed` nên còn kịp) |
| Tham số hình dạng được **validate lúc khởi động** | `max_acceleration: -1` hay `sample_hz: 30` sẽ khiến **mọi** goal âm thầm rơi về carrot — mà carrot **không có trần gia tốc**. Nay node từ chối khởi động, kèm lý do |

🔴 **Tầm với thực tế của một GotoPose bay quỹ đạo: ~64 m** (`v_max` 1,5 · `goto_timeout` 120 s).
Điểm mút nhân ba làm tốc độ trung bình chỉ ~0,44·`v_max` ⇒ **duration ≈ 1,5 s cho mỗi mét**. Xa hơn
thì `build()` từ chối và chuyến bay rơi về carrot — có WARN và có lý do trong `result->message`.

### Yaw: bám hướng đi, rồi bàn giao đúng lúc

Lúc bay mũi theo hướng đi của quỹ đạo (lõi lo, có trần `max_yaw_rate`). Khi
`duration − t ≤ slew_cần + margin` thì target yaw **chốt** sang yaw goal yêu cầu, với
`margin = arrivalTail(plan, acceptance) + arrival_settle_sec` — `arrivalTail` là **thời lượng quỹ đạo
còn lại kể từ lúc mẫu đi vào bán kính acceptance**, quét thẳng trên lưới.

🪤 **Vì sao không tính theo "lúc quỹ đạo hết":** đo được — máy bay vào vùng acceptance 0,3 m ở **giây
6,70 của quỹ đạo 9,0 s**, nên mốc "quỹ đạo hết" tới **muộn hơn lúc goal đã thành công** ⇒ bàn giao
không bao giờ xảy ra (bản đầu đo ra yaw = 0 y nguyên).

🔑 **Bàn giao là latch DÍNH.** Khi dây xích kẹp, `remaining` đóng băng còn `slew_cần` vẫn giảm ⇒ điều
kiện không latch sẽ **lật qua lại ở nhịp 20 Hz** và bơm một dao động yaw thẳng vào PX4.

⚠️ **Arrival là phép thử VỊ TRÍ** (hợp đồng `GotoPose` chỉ trả `final_distance_error`). Chặng quá
ngắn cho cú quay — ví dụ 2 m (≈4,3 s) với Δyaw = π (cần 6,3 s) — thì goal vẫn `SUCCEEDED` còn mũi
chưa tới nơi; khi đó `result->message` **nói thẳng** *"nose N deg short of the requested yaw"*.
Im lặng mới là lỗi, không phải chuyện quay chưa xong.

### 🔴 Dây xích chặn VỊ TRÍ, không chặn GIA TỐC — và điều đó đo được

**48,6 m/s²** — gia tốc suy từ **vị trí đã phát** khi dây xích kẹp rồi nhả (đo trong
`TheTrajectoryClockStopsWhileTheLeashIsClamped`, in ra dòng `[ EVIDENCE ]`). Cơ chế: lúc kẹp carrot
đứng yên (v = 0), tick nhả nó đi trọn một bước kế hoạch ⇒ `v_plan → 0 → v_plan` trong hai tick.

⇒ **Mệnh đề "quỹ đạo xoá bước nhảy" chỉ đúng khi dây xích CHƯA kẹp.** Kẹp là trạng thái bất thường
(máy bay đã tụt >0,8 m), và thứ gửi xuống PX4 là *position setpoint* nên PX4 còn kẹp gia tốc bằng
`MPC_ACC_*` — nhưng đây là **nợ phải nhớ**, và là lý do cổng G-N2 kiểm "chuyến bay có chạm xích
không" **trước** khi đọc bất kỳ con số gia tốc nào.

### 🔴 Bậc thang thứ HAI: đỗ setpoint giữa lúc kế hoạch còn đuôi (tìm ra & vá 2026-08-26)

**16,5 m/s²** — cũng suy từ **vị trí đã phát**, nhưng cơ chế khác hẳn cú kẹp-nhả ở trên, và nó xảy ra
trên **đường bay bình thường**, không cần dây xích chạm.

| | |
|---|---|
| **Cơ chế** | `reached()` chấm trên **vị trí MÁY BAY** (`acceptance_radius` 0,3 m) rồi `arrival_settle_sec` 1,0 s. Máy bay vào quả cầu đó **trước khi kế hoạch giảm tốc xong** — đo được còn ~0,74 s đuôi. `finishAirborne()` gọi `freezeSetpoint()` ngay tại đó ⇒ setpoint đang chạy **0,331 m/s** bị đóng băng trong **một tick 20 ms** |
| **Số đo** | ba lần độc lập: **16,49 · 16,51 · 16,53 m/s²**, đều tại `t+5,17 s` với `v = 0,3314` — trùng khít tới bốn chữ số |
| **Đỉnh bình thường của chính kế hoạch** | **0,587 – 0,604 m/s²** (11 lượt, cả rảnh lẫn có tải) ⇒ phân bố **hai đỉnh**, không phải đuôi thống kê |
| **Vá** | `runOutPlanTail()` — chạy nốt đuôi kế hoạch **chỉ khi `REACHED`** rồi mới đóng băng. Huỷ / preempt / fault vẫn dừng **ngay**: lúc đó thứ cần là setpoint đứng yên bây giờ, không phải sau 0,7 s. Chặn **hai đồng hồ** (hạn theo giờ ROS + trần số tick theo giờ tường) nên `/clock` đứng cũng không treo action |
| **Đối chứng** | máy rảnh: có vá **3/3 PASS** · tắt đúng lời gọi đó **3/3 FAIL** (16,21 / 16,60 / 16,21) |

🔑 **Bài học phương pháp, khác họ nợ #15.** Test này đỏ ~1/6 nên suốt một ngày nó bị xếp nhầm vào
"suite nhạy tải". Nó **không** nhạy tải: khuyết tật **tất định**, xảy ra ở **mọi** lần tới đích; cái
chập chờn là **phép đo** — trace chỉ đôi khi ghi kịp mẫu đứng yên **sau** cú đỗ. Dấu phân biệt hai họ:

> **Nợ #15 (đo sai đại lượng): các lần trượt trị số TÁN LOẠN, vì nó đang đo cái máy.
> Họ này (đo đúng nhưng nhìn thiếu): các lần trượt TRÙNG TRỊ SỐ, vì nó đang đo một sự kiện có thật.**

Test đã sửa kèm: chờ **0,3 s** sau khi action trả kết quả để **luôn** ghi được cú đỗ — trước đó phép đo
dừng sớm đúng một mẫu, và đó chính là lý do khuyết tật sống sót qua mọi lượt chạy trước.

⚠️ **Hệ quả hợp đồng:** action bây giờ trả kết quả **muộn hơn tối đa `kMaxPlanTailSec` = 3 s** so với
trước. Biên đã kiểm: `step_timeout_sec` 300 s so `goto_timeout_sec` 240 s ⇒ dư **60 s**, không chạm.

### 🔑 Trần tốc độ được SUY TỪ ngân sách dây xích, không đặt độc lập (chốt 2026-08-18)

Vòng vị trí của PX4 là bộ P với `MPC_XY_P` **0,95** ⇒ máy bay luôn trễ sau setpoint đúng **`v / K_p`**.
Nếu `max_speed` đòi nhiều lead hơn ngân sách xích, dây xích sẽ kẹp **suốt chuyến bay lành** — và lúc đó
nó thôi làm bộ **báo lỗi**, chỉ còn là bộ **hạn tốc** câm.

```
max_speed  <=  K_p * max_lead / 1.3        # K_p: 0,95 ngang · 1,0 dọc
0,55       <=  0,95 * 0,80 / 1,3 = 0,584   ✓ headroom 1,38x
0,45       <=  1,00 * 0,60 / 1,3 = 0,462   ✓ headroom 1,33x
```

🔴 **Sửa `max_speed` mà không kiểm lại bất đẳng thức này là làm hỏng bộ phát hiện stall.** Nó được ghim
bằng hai test đi thành cặp — `TheShippedSpeedStaysInsideWhatTheAircraftCanTrack` (trần đã ship, plant
mô phỏng đúng `K_p` 0,95 ⇒ `result->message` **không** được chứa `"clamped"`) và
`TheOldSpeedCeilingRidesTheLeashTheWholeWay` (trần cũ 1,5 ⇒ **phải** chứa). Không có cái thứ hai thì
cái thứ nhất chỉ chứng minh rằng **không có gì chạy cả**.

**Cái giá, đo được:** chặng 3 m mất **13,4 s** thay vì 8,3 s (**+38%**). Đổi lại: bằng chứng stall sạch.
**Hệ quả tầm bay:** điểm mút nhân ba cho tốc độ TB ≈ **0,44 × trần** ⇒ với `goto_timeout_sec` 240 s và
`max_duration` bị node kẹp xuống `0,8 × timeout`, **một Goto với tới ≈ 46 m**. Xa hơn thì chia chặng.

### Hợp đồng của `/planning/trajectory`

| Điều | Nội dung |
|---|---|
| QoS | `KeepLast(1)` + `Reliable` + **`TransientLocal`** — bản kế hoạch **đang có hiệu lực** |
| Publisher | **ĐÚNG MỘT** (chính navigator). Latched + hai writer = node vào muộn nhận hai "kế hoạch hiện hành" mâu thuẫn, không thứ tự |
| Thu hồi | `is_valid=false` + `points` rỗng, phát khi task kết thúc/cancel/fault, khi fallback carrot, và **một bản lúc node khởi động** (để "chưa có kế hoạch" khác được với "navigator chết") |
| `start_time` | **danh nghĩa**. Đồng hồ kế hoạch **đóng băng khi dây xích kẹp** ⇒ cấm nội suy `time_from_start` theo giờ tường để suy ra "giờ này drone phải ở đâu" |
| `points[].yaw` | yaw **bám hướng đi**; đoạn bàn giao sang yaw đích ở cuối chuyến **chưa** được ghi lại vào lưới đã publish (nợ, xem §8) |

## 6e. Costmap + route planner (P6.3) — `costmap.hpp` · `route_planner.hpp` · `route_planner_node`

**Đường dữ liệu:** `/world/obstacle_map_local` (ObstacleArray) + `/state/odometry_fused` +
`/planning/route_goal` (PoseStamped) → `route_planner_node` → **`/uav/<id>/planning/route`** (Path3D).

### Bốn điều phải biết trước khi sửa

**① `lastInputAge()` là kênh độ tươi RIÊNG — cell không nói được điều đó.**
Costmap xoá mọi cell về `FREE` mỗi lượt update. Bản đồ **rỗng** và bản đồ **chết** trông y hệt nhau
nếu chỉ đọc cell. Tuổi được giữ qua các lượt rỗng, và là **vô cực** khi chưa từng nhận vật cản nào.
Node hỏi nó qua `require_obstacle_feed` + `map_timeout_sec` — **sim mặc định `false`, real `true`**
(cùng nếp bộ tiêm nhiễu P4: thứ nguy hiểm không được bật sẵn ở đời thật).

**② Clearance chứng minh trên waypoint KHÔNG phải clearance bay.**
Spline cắt góc **0,44–0,59 m** (đo trên path A\* thật, 2026-08-18). Địa hình thoáng thì vô hại vì
cost mềm để lại đệm **1,9–2,2 m** trong khi khung chỉ cần 0,35. **Hành lang hẹp bắt buộc thì khác:**
khe 1,4 m có khúc gấp ⇒ **waypoint sạch mà spline chạm 253 = INSCRIBED**.
→ Dùng `tightenForSpline()` (chia đôi đoạn tới khi sạch). 🔴 **Đừng trả trước bằng `corner_cut_m`** —
hằng số 0,6 m bịt mọi khe hẹp hơn ~1,9 m. Lý lẽ đầy đủ: `plan/P6-navigation.md` §2f.

**③ `INSCRIBED` bị TỪ CHỐI, không phải bị tính giá cao.** Và đường chéo **không được lách** giữa hai
ô chặn: khung máy bay tròn, khe mà lưới tưởng có thì trên trời không có.

**④ Waypoint mang cao độ của BĂNG BAY, không phải của goal.** Vật cản chỉ được kiểm trong lát cắt
`±flight_band_m` quanh cao độ hiện tại (2.5D, quyết định 3). **Hồ sơ độ cao là việc của caller.**

### Tham số hay chỉnh

| Tham số | Mặc định | Ghi chú |
|---|---|---|
| `costmap.width_m` · `resolution_m` | 25,0 · 0,25 | 100×100 cell |
| `costmap.drone_radius_m` | 0,35 | bán kính đường tròn ngoại tiếp khung |
| `costmap.cost_scaling` | 1,0 | **đây mới là thứ tạo đệm 2 m**, không phải `corner_cut_m` |
| `costmap.corner_cut_m` | **0,0** | cố ý — xem ② |
| `costmap.obstacle_timeout_sec` | 3,0 | map ~7 Hz + khoảng trống 1,8 s ⇒ 3 s là chết thật |
| `route.projection_timeout_sec` | 2,0 | đếm **mọi** lượt không ra tuyến, không riêng projection |
| `require_obstacle_feed` | sim `false` / real `true` | xem ① |

### Thất bại được PHÁT, không im lặng

Mọi lượt hỏng đều publish `Path3D` với `is_valid=false`. Người tiêu thụ thấy im lặng thì **không
phân biệt được "không có tuyến" với "node chết"**.

✅ **Đóng 2026-08-19 (Đ1):** `Path3D` có `plan_state` (`NO_GOAL`/`FAILED`/`WITHDRAWN`) + `reason`
riêng, `planner_name` trở lại đúng vai tên thuật toán. Thiếu `odometry_fused` → `NO_GOAL`; map cũ/thiếu
khi bắt buộc, A* bí, hoặc spline phá map → `FAILED`, lý do nằm trong `reason`, không còn mượn
`planner_name`. Chi tiết: [`docs/interface-contract-v0.1.md`](../../docs/interface-contract-v0.1.md) §2.6.

## 6f. Navigator tiêu thụ route + advice (P6.4 — B1/B2) — `navigator_action_server_node`

**Đường dữ liệu của một `GotoPose` kể từ 2026-08-19:**

```
GotoPose ─> navigator ──publish──> /planning/route_goal (PoseStamped)
                                        │
                    route_planner_node ─┘
                            └─publish──> /planning/route (Path3D)
                                        │
   navigator dựng B-spline TRÊN waypoint route ──> /planning/trajectory ──> stream 20 Hz
                                        │
   local_planner_node đọc trajectory ───┘
            └─publish──> /planning/avoidance (AvoidanceAdvice) ──> navigator quyết định
```

### Route: hỏi rồi chỉ nhận CÂU TRẢ LỜI CỦA MÌNH

| Điều | Nội dung |
|---|---|
| Chờ đúng cái gì | Chỉ nhận `Path3D` có **stamp muộn hơn lúc hỏi**. Route cũ là của goal khác |
| **Không ai publish** | Trả lời ngay `"no route planner is publishing"` — **không** chờ hết `route_timeout_sec`, nếu không mọi goal chậm thêm 2 s vì một node không tồn tại |
| Route hỏng + `require_obstacle_feed=true` | **ABORT goal**, `result.message` = `"no route and the map is required: <reason>"` |
| Route hỏng + `false` | Đường thẳng, **nhưng** `result.message` mang `"straight line, no route: <reason>"` — cấm im lặng |
| ⚠️ Lệch so với plan | Plan viết *"REJECT goal"*; hiện thực **abort khi đang chạy**, vì lúc goal-callback chưa hỏi route được (hỏi ở đó sẽ chặn executor tới 2 s). Abort giữ đúng feedback/cancel và vẫn nói rõ lý do |
| **④ của §6e trả ở đây** | Waypoint route mang cao độ **băng bay**; `applyAltitudeProfile()` đặt lại `z` đầu = origin, `z` cuối = goal, giữa **nội suy theo quãng đường ngang** ⇒ không chặng nào biến thành thang máy |

### Bảng chính sách `AvoidanceAdvice` — 5 dòng, mỗi dòng một test

| Advisor nói | Navigator làm | Test làm nó cắn |
|---|---|---|
| `CLEAR` **và** `checked_horizon_m > 0` | bay tiếp; nếu đang hold thì **nhả hold** | `AHoldThatClearsResumesTheSamePlan` (nhả hold) · `ACheckedClearAdviceLeavesTheFlightAlone` (không đổi gì) |
| `ESCAPE` **khi đang bay quỹ đạo** | dựng lại quỹ đạo **qua điểm escape** rồi tới goal | `AnEscapePointBendsTheFlightThroughIt` |
| 🔴 `ESCAPE` **khi đang bay carrot** | **HOLD + WARN**, KHÔNG bay escape — escape gắn với máy dựng lại quỹ đạo, chặng carrot không có máy đó (§6h) | `AnEscapeIsHeldNotFlownWhileTheChaseRidesTheCarrot` |
| `HOLD` | **tạm dừng** quỹ đạo tại chỗ (xem dưới) | `AHoldAdviceFreezesTheSetpointWithoutStoppingTheStream` |
| im lặng > `advice_timeout_sec` | `require_obstacle_feed=true` → **HOLD**; `false` → bay tiếp + `"flying unguarded: …"` | `SilenceHoldsTheFlightWhenTheMapIsRequired` · `SilenceIsFlownThroughLoudlyWhenTheMapIsNotRequired` |
| 🔴 `checked_horizon_m == 0` | **KHÔNG phải giấy phép bay.** Không làm mới đồng hồ permission ⇒ tự rơi vào đúng nhánh im lặng | `AZeroHorizonClearIsNotPermissionToFly` (bắt buộc → HOLD) · `AZeroHorizonClearIsFlownThroughLoudlyWhenTheMapIsNotRequired` |

🔴 **HOLD phải có đồng hồ riêng — `avoidance_hold_timeout_sec` 12,0 s** (thêm 2026-08-20, E2-N1). Một
hold **chờ** đường thoáng lại; hold không bao giờ thoáng thì **không phải đang chờ**, và ngồi ăn hết
`goto_timeout_sec` 240 s rồi mới `ABORTED_TIMEOUT` là **báo sai chuyện ở sai thời điểm** — đúng bài học
bản vá R-C2b (6988 ms → 972 ms). Quá hạn ⇒ `planner_fault_` ⇒ abort ngay với
`ABORTED_PLANNER_FAILED` + `"held by the obstacle advisor for X s with no way through"`.
**Suy số:** 12 s = **4 lần vòng đời bản đồ vật cản** (`avoid.map_timeout_sec` 3,0) — vật cản chưa nhúc
nhích qua bốn vòng đời bản đồ thì không còn là nhất thời; giá phải trả là **6,6 m** quãng đường chưa
bay ở 0,55 m/s. Áp cho **mọi** hold (quỹ đạo lẫn carrot), vì hold chết trên quỹ đạo cũng chết y hệt.
Test: `AHoldWithNoWayThroughEndsAsAHoldNotAsAGoalTimeout`.

**HOLD = TẠM DỪNG, không phải dựng lại.** `motion_paused_` đóng băng setpoint **và** đồng hồ kế hoạch;
nhả ra là chạy tiếp đúng chỗ cũ. Dựng lại quỹ đạo sẽ **khởi hành từ trạng thái nghỉ** ⇒ bậc vận tốc
trong dòng setpoint. Hai hệ quả phải nhớ:
- 🔑 **Kế hoạch đã publish KHÔNG bị thu hồi khi hold** — advisor cần chính nó để nói *"thoáng rồi"*.
  Thu hồi = **khoá chết**: không có plan ⇒ advisor phát horizon 0 ⇒ không bao giờ được nhả.
- Đồng hồ kế hoạch dừng ⇒ `feedback.estimated_time_remaining` **đứng yên**. Đó là kênh feedback duy
  nhất báo "đang bị giữ": `GotoPose.Feedback` **không có trường chữ** (hợp đồng v0.1 đã đóng băng).

### Escape KHÔNG được vứt tuyến toàn cục (R-N1, đóng 2026-08-19)

🔴 **Kịch bản hỏng nếu vứt:** goal nằm sau bức tường ⇒ A\* vòng qua ⇒ vật cản đầu tiên kích `ESCAPE`
⇒ dựng lại `escape → goal` **thẳng** ⇒ lại đâm vào tường ⇒ escape/hold luân phiên tới hết
`goto_timeout_sec` 240 s. Đúng **kẹt local-minima** mà route planner sinh ra để tránh.

**Cách làm:** sau escape, quỹ đạo mới là `setpoint hiện tại → escape → (phần tuyến còn lại) → goal`.
Phần đuôi lấy từ **tuyến đang chảy sẵn** trên `/planning/route`, chọn từ waypoint gần escape nhất trở đi.

🔴 **CẤM gọi `requestRoute()` chặn giữa chuyến.** Nó khoá task thread tới `route_timeout_sec` 2,0 s
trong khi máy bay **vẫn bay kế hoạch cũ về phía vật cản**: ở 0,55 m/s là **1,1 m**, so với hở đo được
**0,941 m** ⇒ đổi một lỗi lấy một lỗi tệ hơn. `route_planner_node` phát 5 Hz nên tuyến luôn có sẵn.

| Điều kiện để ghép đuôi | Không đạt thì |
|---|---|
| 🔴 `goal_stamp` của tuyến **khớp đúng** câu hỏi navigator vừa gửi | bay thẳng, ghi `"route on the wire answers a different goal"` |
| `route_fresh_sec` (1,0 s = 5 nhịp) | bay thẳng tới goal, `result.message` ghi `"no route to rejoin: route is X s old"` |
| `plan_state == VALID`, ≥ 2 waypoint, mọi waypoint hữu hạn | như trên, kèm `reason` của planner |
| Đuôi phải **tới gần goal hơn** điểm escape | như trên — chặn ghép nhầm tuyến của goal khác. ⚠️ Planner **kẹp goal xa về mép cửa sổ**, nên không thể đòi tuyến kết thúc ĐÚNG trên goal |
| Cao độ | `applyAltitudeProfile(via, escape, goal)` — giữ z của escape (advisor chọn có chủ ý), rải phần còn lại từ escape trở đi |

✅ **Đã đóng 2026-08-19:** `Path3D` thêm `goal_stamp` (thuần bổ sung) — `route_planner_node` chép
`header.stamp` của `route_goal` nó đang phục vụ. Navigator đối chiếu ở **cả hai** chỗ: lúc chờ tuyến
(`requestRoute`) và lúc ghép đuôi (`routeTailAfter`). **Thời gian tới không phải danh tính:** planner
phát 5 Hz nên một bản tuyến của goal TRƯỚC vẫn hạ cánh tới **200 ms sau** lúc goal mới được hỏi.
Chi tiết: [contract §2.6](../../docs/interface-contract-v0.1.md).

### 🔴 Cái giá của kiến trúc advisor — navigator TIN advisor về hình học

**Navigator không có costmap.** Nó **không thể** kiểm điểm escape có thoáng không — việc đó
`local_avoidance` làm trước khi phát (validate 2 đoạn, chặn `|Δz|` quá `max_escape_climb_m`, cấm chui
xuống dưới). Navigator chỉ kiểm được thứ nó **có** dữ kiện để kiểm:

| Kiểm được | Làm gì | Test |
|---|---|---|
| `escape_point` hữu hạn | NaN ⇒ **từ chối advice → HOLD** | `AnEscapePointThatIsNotFiniteIsRefusedAndHeld` |
| 🔴 **Nằm trong `[min_altitude_m, max_altitude_m]`** (thêm 2026-08-20, E2-C1) | ngoài phong bì ⇒ **từ chối → HOLD**. Phong bì cao độ chỉ được thực thi ở **cổng nhận goal**, mà escape **không đi qua cổng đó**: advisor hỏng phát `z = −3` (lệch 4,15 m < trần 8 m) thì lõi quỹ đạo dựng một đường **mượt xuống đất**, dây xích **không cắn** (drone bám kịp) và không còn ai nói không. Cùng lý lẽ với `buildRecoveryPlanner`: *một khôi phục không được bay chỗ mà một goal bị cấm* | `AnEscapeBelowTheAltitudeFloorIsRefusedAndHeld` |
| Lệch khỏi đoạn *(setpoint hiện tại → goal)* > `max_escape_deviation_m` | **từ chối → HOLD** | `AnEscapePointFarOffThePlanIsRefusedAndHeld` |
| Escape thấp hơn setpoint > 0,5 m | **KHÔNG từ chối** — chỉ WARN + ghi vào `result.message` | `AnEscapeThatGoesDownIsFlownButNamed` |

🔴 **Vì sao descent chỉ được ghi chứ không bị chặn:** chính sách *"cấm né bằng hạ thấp"* đã nằm ở
`avoid.allow_descent` của advisor. Chặn lần thứ hai ở navigator tạo **hai nơi cùng giữ một chính
sách** — bật `allow_descent=true` có chủ đích sẽ thành treo máy im lặng thay vì một hành vi có chủ.

⚠️ **Ràng buộc tham số phải giữ:**
`max_escape_deviation_m` ≥ `avoid.spiral_growth_m × 2√(avoid.max_spiral_steps)` = **4,90 m** với cấu
hình hiện tại. **Giá trị đang ship là 8,0** (yaml · `declare_parameter` · member initializer — ba nơi,
một số). Chọn 8,0 chứ không phải 5,0 vì guard này bắt advisor **HỎNG**, không phải để nghi ngờ một
advisor đang chạy đúng: 63% dư trên cận 4,90 m. Đặt thấp hơn cận đó ⇒ navigator từ chối chính những
escape advisor coi là hợp lệ (an toàn nhưng mission chết); đặt quá cao ⇒ mất tấm chắn duy nhất.

🔴 **Không tham số nào ở đây được phép tự vô hiệu hoá — node TỪ CHỐI KHỞI ĐỘNG (`throw`):**

| Từ chối khi | Vì sao |
|---|---|
| `route_timeout_sec` · `advice_timeout_sec` · `max_escape_deviation_m` · `escape_replan_interval_sec` · `escape_refresh_m` **không hữu hạn hoặc ≤ 0** | `advice_timeout_sec = 0` từng **tắt câm cả máy né**; `max_escape_deviation_m = NaN` làm `deviation > NaN` **luôn false** ⇒ tấm chắn không bao giờ cắn. Cùng nếp `Costmap::build`/`LocalAvoidance::build`: **refuse chứ không clamp** |
| `require_obstacle_feed=true` **và** `use_avoidance=false` | Đòi bản đồ rồi tắt máy đọc bản đồ |
| `require_obstacle_feed=true` **và** `use_route=false` (thêm 2026-08-20, E2-N2) | Rollback **im lặng nhất** trong ba cái: `requestRoute` trả rỗng **kèm reason rỗng** ⇒ Goto bay **đường thẳng** dưới đúng cấu hình "bản đồ bắt buộc" ⇒ kẹt local-minima mà route planner sinh ra để tránh. Một bản đồ được đòi nhưng chỉ soi đường thẳng là bản đồ không ai dùng |
| `require_obstacle_feed=true` **và** `use_trajectory=false` | ⚠️ **Lý do đã ĐỔI 2026-08-20** — không còn là vòng tự khoá (carrot nay publish đoạn của nó, §6h). Vẫn từ chối vì rollback này **vứt trần gia tốc của lõi quỹ đạo cho MỌI goal** (carrot đo được ~50 m/s² so với 0,23), mà cấu hình đòi bản đồ chính là cấu hình bay có người |

🔑 **Tắt máy né bằng `use_avoidance: false`, KHÔNG bằng `advice_timeout_sec: 0`.** Một cơ chế an toàn
bị tắt bằng một con số là cách nó bị tắt mà không ai nhận ra.

🔴 **Cùng vòng tự khoá đó với tới được mà KHÔNG cần cấu hình sai:** một tuyến dài hơn `max_duration`
làm `Trajectory::build` **thất bại giữa chuyến** rồi rơi về carrot. Nên khi `require_obstacle_feed=true`:

| Dựng quỹ đạo hỏng ở đâu | Navigator làm |
|---|---|
| Lúc nhận goal (`executeGoto`) | **ABORT ngay**, `ABORTED_PLANNER_FAILED`, `"no trajectory and the map is required"` |
| Lúc dựng lại qua escape (`flyAround`) | Đặt `planner_fault_` ⇒ `checkFaults()` nhặt ⇒ abort ngay, không chảy vào máy timeout |

**Chống dao động:** `escape_refresh_m` 0,5 (cùng một vật cản được khuyên lại không phải tin mới) +
`escape_replan_interval_sec` 1,0. Không có hai cái này, advisor 10 Hz sẽ dựng lại quỹ đạo 10 lần/giây
và mỗi lần đều khởi hành từ nghỉ.

### Nơi advice được tiêu thụ (R0)

Trong **vòng progress của task thread** ở `progress_rate_hz`, **không** trong timer stream 20 Hz.
Dựng lại quỹ đạo tốn hàng nghìn điểm lưới; đặt nó trên timer là **rớt nhịp offboard giữa trời**.
Có watch: `GotoPose` · `FollowPath` · **`TrackTarget`** (từ 2026-08-20, §6h) · **`Recover` khi nó có
chuyển động**. Không có watch: `Takeoff` · `HoldPosition` · `Land` · `Recover` loại HOLD — chúng bay
đứng hoặc đứng yên, **nhưng vẫn publish** thứ chúng đang bay (§6h). ⚠️ Nghĩa là
`require_obstacle_feed=true` **không** phủ ba action đầu — nợ ở §8.

## 6g. `FollowPath` + `TrackTarget` (P6.5 — B3)

### `FollowPath` — GotoPose với waypoint do caller đưa

Dùng **nguyên** lõi quỹ đạo + dây xích + cổng σ + tiêu thụ advice. **Không có đường phát setpoint thứ
hai.** `via = waypoints[0..n-2]`, `target = waypoints[n-1]`, `ViaAltitude::KEEP` (cao độ của caller là
có chủ ý, không rải lại).

🔑 **Hai bộ đếm, trả lời hai câu khác nhau** — đây là chỗ dễ nhầm nhất:

| Bộ đếm | Nghĩa | Tăng khi |
|---|---|---|
| `achieved[i]` → `result.waypoints_reached` | waypoint **thật sự bay tới trong** `waypoint_acceptance_radius` | chỉ khi vào trong bán kính |
| `ahead` → `feedback.path_completion_percent`, và là gốc để **ghép lại sau escape** | đã đi qua tới đâu | vào trong bán kính **hoặc** đã gần waypoint kế hơn waypoint hiện tại |

⚠️ **`path_completion_percent` tính theo CHIỀU DÀI CUNG, không theo chỉ số waypoint** (sửa
2026-08-20): phần trăm = (cung đã qua + hình chiếu lên chặng đang bay) / tổng chiều dài path, và
**giữ-đỉnh** (không tụt khi né vòng ra xa path). Path có chặng dài ngắn chênh nhau thì % nay phản ánh
quãng đường thật — trước đây 3 waypoint cách đều chỉ số cho mỗi bước +33% bất kể chặng 1 m hay 10 m.

**Vì sao cần cái thứ hai:** spline **xén góc** `|W0−2W1+W2|/6`. Nếu chỉ có `achieved`, một góc bị xén
làm `ahead` kẹt lại, và một lần escape sẽ **quay đầu máy bay về nhặt waypoint đã bỏ qua**.
⚠️ **Chọn `waypoint_acceptance_radius` > lượng xén góc** của chính path đó, nếu không
`waypoints_reached` sẽ báo thiếu dù máy bay đã bay đúng. Thiếu thì `result.message` nói rõ *"k of n
waypoints came inside X m"* — không im lặng.

**Thành công = tới waypoint CUỐI** (như GotoPose), không đòi đủ n waypoint.

### `TrackTarget` — phần sát an toàn nhất của P6.5

Đọc `/uav/<id>/world/target_state`. Bay bằng **carrot** (`setTarget` mỗi nhịp `progress_rate_hz`),
**không** dựng lại quỹ đạo — mục tiêu di chuyển liên tục, dựng lại mỗi nhịp là đúng bẫy thrash của
escape, và mỗi lần dựng đều khởi hành từ trạng thái nghỉ.

**Điểm đứng:** cách target `standoff` mét, **về phía máy bay đang đứng**, ở **cao độ của chính máy
bay**. 🔴 Không bám cao độ target — bám sẽ **dí máy bay xuống đất** khi target ở mặt đất.

⚠️ **Target nằm ĐÚNG dưới bụng drone** (khoảng cách ngang < 1 mm) thì không có phương "về phía máy bay"
nào cả: `standoffPoint()` rơi về **hướng +x tuỳ ý** để luôn trả ra một điểm hữu hạn. Máy bay sẽ dạt
`standoff` mét theo +x rồi mới bám bình thường. Từ 2026-08-20 chặng dạt đó **được publish** (§6h) nên
advisor soi được nó — trước đây nó là một đoạn bay tuỳ ý mà không ai kiểm. 🪤 Trong test, đây cũng là
cái bẫy: nguồn target giả mặc định phát `(0,0,0)`, trùng ngay dưới máy bay, nên chặng đầu tiên phát ra
**không liên quan gì tới target thật** nếu chưa kịp nhận bản tin đầu.

🔴 **Quyết định Đ4 — hai điều không thương lượng:**

| Điều | Hiện thực | Con số |
|---|---|---|
| Standoff cộng biên phản ứng | `max(min_standoff_m, requested) + target_velocity_error_mps × target_reaction_sec` | **+0,71 m** |
| 🔴 **Sàn cứng trong code** | `kMeasuredTargetVelocityError = 0.708` — tham số chỉ được **nới lên**; nhỏ hơn ⇒ **node từ chối khởi động**. Đây là **giới hạn đo được của chuỗi cảm biến**, không phải núm chỉnh: hạ nó xuống không làm drone an toàn hơn, chỉ làm hệ **nói dối về thứ nó biết** | sàn **0,708** |
| **CẤM lead bằng `vx/vy` khi thiếu `position_uncertainty`** | `target_lead_sec > 0` vẫn bị **từ chối** nếu `position_uncertainty` không hữu hạn hoặc `< 0`; ghi `"target lead refused"` vào `result.message` | mặc định `target_lead_sec = 0` |

**Căn cứ:** đo 2026-08-18 — sai số vận tốc mục tiêu **0,708 m/s** trên mục tiêu đi thật 1,0 m/s
(**71%**), cùng bậc với cận hai điểm `σ√2/dt = 0,85`. ⇒ **giới hạn thông tin, không sửa được bằng lọc
tốt hơn.** Ghim ở `test_target_tracking.cpp` của `uav_perception`.

✅ **TrackTarget nay CHẠY ĐƯỢC với `require_obstacle_feed=true`** (2026-08-20). Trước đó nó bị từ chối
ngay ở cổng nhận goal vì không publish gì cho advisor soi. Nay chặng carrot **tự publish đoạn 2 điểm**
(§6h) nên advisor soi được, và lưới an toàn là **chính bảng chính sách advice** ở §6f: im lặng quá
`advice_timeout_sec` ⇒ HOLD · `HOLD` ⇒ HOLD · `ESCAPE` ⇒ **HOLD chứ không bay escape**.

🔑 **Trong lúc HOLD, chặng vẫn được publish với đích DỰ ĐỊNH** — cùng nếp "kế hoạch đã publish không bị
thu hồi khi hold" ở §6f: thu hồi là khoá chết, vì advisor cần chính nó để nói *"thoáng rồi"*. Và
`pursue` **không** gọi `setTarget` khi đang hold (nếu gọi, `clearTrajectory()` bên trong sẽ tự tay gỡ
`motion_paused_` — hold biến mất sau đúng một nhịp).

**Mất target — HAI TRỤC, đừng nhầm (N2, sửa 2026-08-22):** `TargetState.header.stamp` là **thời điểm
publish** theo hợp đồng, nên `target_state_timeout_sec` **chỉ bắt được "world_model chết"** — nó
**không bao giờ** bắt được "không ai còn nhìn thấy mục tiêu". Trục thứ hai mới là trục đó:

| Trục | Trường đọc | Tham số | Bắt được gì |
|---|---|---|---|
| Sự sống của nguồn | `header.stamp` | `target_state_timeout_sec` **1,0** | world_model chết / im |
| **Tuổi lần NHÌN THẤY** | `time_since_seen_sec` + `status` | `target_sighting_timeout_sec` **1,0** | mục tiêu khuất, dù bản tin vẫn đều |

Coi là **không-thấy** khi `status == STATUS_LOST`, **hoặc** `time_since_seen_sec` > ngưỡng, **hoặc**
trường đó là NaN/âm (phép so sánh viết dạng phủ định vì `NaN > x` luôn false ⇒ NaN sẽ đọc thành
"vừa thấy"). Không-thấy ⇒ đóng băng setpoint tại chỗ; quá `target_lost_timeout` ⇒ abort
`ABORTED_LOST_TARGET` (không phải `ABORTED_TIMEOUT`).

🔴 **Vì sao đây là lỗi sát an toàn:** trước bản vá, `world_model` phát đều 20 Hz nên `age` luôn ≈ 0 ⇒
`ABORTED_LOST_TARGET` **bất khả đạt** qua đường này và `feedback.target_visible` **luôn true** — máy
bay bám một điểm ma và không ai biết. Đo trong lát cắt chẩn đoán G-M3: `status` báo LOST 7 mẫu,
COASTING 22 mẫu mà navigator vẫn ngắm vào.

⚠️ **`STATUS_COASTING` KHÔNG tự nó là mất target** — nó là "khuất một thoáng, vẫn đang dự đoán". Chỉ
**tuổi** mới quyết. Căn cứ chọn ngưỡng **1,0 s** (đo trên chuỗi G-M3 thật, 4870 mẫu một chuyến bám
lành mạnh): tuổi nhìn-thấy bằng **0 ở 95% số mẫu**, và **chưa bao giờ ở trên ngưỡng liên tục quá
0,37 s**. 1,0 s = 2,7× cái đuôi tệ nhất đo được, và vẫn **dưới** `lost_after_sec` 3,0 s của tracker.

**Ràng buộc ghép cặp (validate, họ R29) — node từ chối khởi động nếu sai:**
`2 × target_sighting_period_copy_sec  ≤  target_sighting_timeout_sec  <  tracker_lost_after_copy_sec`.
Vế trái: grace bằng đúng một chu kỳ lấy mẫu là biên 0 — một khung hình trễ thành "mất mục tiêu". Vế
phải: ngưỡng ≥ lúc tracker bỏ cuộc thì nhánh `status` luôn nổ trước, cổng tuổi **không bao giờ cắn**.
Hai `*_copy` là **bản sao để validate, không phải núm chỉnh** (chu kỳ phát hiện 15 Hz của depth
camera; `lost_after_sec` của `target_tracker_node`).

**Mũi drone:** quay về phía target bằng đúng bộ giới hạn `max_yaw_rate_rad_s`; ở khúc cua **mũi sẽ
trễ** — `peakYawLag()` đo 26–57° trong test. Quyết định 2026-08-17 giữ nguyên: **không** kéo dãn quỹ
đạo để mũi kịp quay.

## 6h. Chặng carrot tự publish, và `Recover` (P6.6 — B4b, 2026-08-20)

### Vì sao chặng carrot phải publish

Advisor **chỉ** soi `/planning/trajectory`. Mọi chặng bay bằng carrot (Takeoff · TrackTarget · Goto khi
quỹ đạo không dựng được · Recover rơi về carrot) trước đây **vô hình** với nó ⇒ `checked_horizon_m=0`
mãi ⇒ `require_obstacle_feed=true` là treo tới timeout. Đó là **cửa thứ ba của vòng tự khoá R-C2**.

✅ **Cách chữa (chủ dự án chốt 2026-08-19):** khi bay carrot, navigator **publish đúng thứ nó đang bay**
— một `Trajectory3D` **2 điểm**: `setpoint đang lệnh → đích carrot`. Advisor soi được ngay, **không đổi
một dòng nào ở advisor**. *Đã loại: thêm topic thứ hai cho "đoạn đang bay"* (thêm một interface + một
đường dữ liệu phải bảo trì, và §2.14 chốt `Trajectory3D` chỉ có **đúng một** publisher).

| Trường | Giá trị | Vì sao |
|---|---|---|
| `plan_state` | `PLAN_STATE_VALID`, `is_valid=true` | Đây là kế hoạch đang có hiệu lực thật, không phải bản nháp |
| `sequence` | **tăng mỗi lần phát** | Mỗi chặng carrot là một kế hoạch mới; consumer so được hai bản kế tiếp |
| `time_from_start` | `0` và `max(ngang/max_speed, dọc/max_vertical_speed)` | **Đúng công thức `advanceCarrot`** — carrot kẹp hai trục bằng hai trần riêng, nên thời lượng là cái nào hết sau |
| `velocity` | điểm đầu = vận tốc đang chạy, điểm cuối = **0** | Carrot chạy đều rồi **dừng hẳn** ở đích. Ghi vận tốc ở điểm cuối là nói dối rằng nó bay tiếp |
| `yaw` | yaw **đang lệnh** ở cả 2 điểm | Không hứa hẹn gì về lúc nào mũi quay xong — cùng khoảng trống đã ghi ở [contract §2.14](../../docs/interface-contract-v0.1.md) |
| Không có đích để bay | `PLAN_STATE_NO_GOAL` + `"holding the setpoint, no leg to check"` | Setpoint đứng yên (Hold, hoặc bám mục tiêu đã tới nơi) thì **không có gì phía trước để kiểm**. Phát đoạn dài 0 m là mời advisor báo "đã kiểm 0 m" — đúng cái bẫy `checked_horizon_m=0` |

⚠️ **Đoạn 2 điểm là ĐƯỜNG THẲNG, carrot thật thì không.** Hai trục có hai trần tốc độ riêng nên một
chặng vừa lên vừa ngang sẽ **lên xong trước rồi mới đi ngang**. Sai khác này **không ảnh hưởng thứ
advisor kiểm** — `Costmap::costAt()` tra ô theo **(x, y), bỏ qua z** — nhưng **đừng dùng đoạn này để dự
đoán z(t)**.

### Nhịp làm tươi — `carrot_plan_period_sec` **0,5 s**, sàn cứng **0,2 s**

🔴 **Không phát mỗi tick.** Không phải để tiết kiệm CPU, mà vì `watchAvoidance` chỉ nhận advice có
`stamp > plan_installed_`: **phát nhanh hơn advisor trả lời thì mọi câu trả lời đều nói về chặng vừa bị
thay** ⇒ navigator đọc thành **im lặng** ⇒ `require_obstacle_feed=true` là HOLD vĩnh viễn. Một chặng
đo được **85,2 ms** cho trọn vòng advisor (G-N4a) ⇒ sàn **0,2 s** (~2,3×), node **từ chối khởi động**
nếu đặt thấp hơn. Nhịp ship 0,5 s ≈ **6 câu trả lời mỗi lần phát**.

| Điều | Cách làm |
|---|---|
| Phát lại khi nào | Đã qua `carrot_plan_period_sec` **và** một trong hai đầu đoạn đã dịch ≥ `kMinWaypointSpacing` (0,05 m). Bản đầu tiên của mỗi task phát **ngay** |
| 🔴 Trần cứng | **Tối đa 1 message / period**, kể cả khi đổi qua lại giữa "có chặng" và "đứng yên" — nếu chỉ chặn phía SEGMENT thì một đoạn nhấp nháy quanh ngưỡng 0,05 m sẽ phát **mỗi tick** lên một topic reliable+latched |
| Đích đổi (TrackTarget dời đích liên tục) | Vào dây trong ≤ 0,5 s — ở 0,55 m/s là ≤ **0,275 m** chưa được kiểm, so với đệm inflation **0,591 m** |
| Đứng yên | **Không phát lại** — nội dung không đổi thì bản trên dây vẫn đúng |
| `plan_installed_` | **Cập nhật** mỗi lần phát (advice về chặng cũ không được tính cho chặng mới) |
| 🔴 `advice_permission_` | **KHÔNG** làm mới. Nếu làm mới, một mục tiêu di chuyển sẽ tự tay che một advisor đã chết bằng chính nhịp phát của mình |

⚠️ **Hệ quả đã biết, lệch về phía an toàn:** advisor chọn điểm gần nhất trên đoạn rồi lấy phần phía
trước, nên khi máy bay tới **quá nửa đoạn**, phần còn lại chỉ còn 1 điểm ⇒ advisor báo *"no active plan
to check"* ⇒ với map bắt buộc là một nhịp HOLD tới lần phát sau. Chỉ xảy ra ở cuối chặng ngắn
(< ~0,6 m), lúc đó máy bay gần như đã đứng, và HOLD ở đó là **dừng sớm hơn**, không phải đi xa hơn.

### `Recover` — action thứ 7, và là action DUY NHẤT preempt

**Quyết định Đ3 (chủ dự án chốt):** chỉ **3 loại**, và **goal CHỌN — node không bao giờ tự quyết**.

| Loại | Node làm | Kết thúc khi |
|---|---|---|
| `TYPE_HOLD` | Đóng băng setpoint tại chỗ | 🔴 **Bằng chứng dòng lệnh:** setpoint đứng yên **và** stream đã chảy đủ `arrival_settle_sec` — **không** đo `khoảng cách(pose, điểm giữ)` |
| `TYPE_CLIMB_TO_SAFE_ALTITUDE` | Bay thẳng lên `safe_altitude` (**tuyệt đối trong `odom`**) | Tới nơi |
| `TYPE_RETURN_HOME` | Leo lên `safe_altitude` rồi bay ngang về **trên đầu** điểm cất cánh | Tới nơi, **hover ở đó** |
| 🔴 `TYPE_LAND` · `TYPE_HANDOVER_TO_PILOT` · `TYPE_UNKNOWN` | **REJECT ngay ở cổng nhận, kèm lý do** | — |

🔴 **Vì sao không có LAND:** mất định vị thì máy bay **không biết mình ở đâu** — "hạ cánh tự động" là hạ
xuống một chỗ không ai biết. Quyết định *hạ cánh* hay *trả quyền pilot* thuộc tầng safety/authority
(P7–P8), không phải tầng chuyển động. Xem [contract §2.5](../../docs/interface-contract-v0.1.md) và skill
`uav-safety-failsafe`. ⚠️ Lý do từ chối chỉ ra **log** (goal bị REJECT thì ROS
action không mang message) — nên mỗi loại cấm có một test riêng ghim đúng sự từ chối đó.

🔑 **Kiểm LOẠI trước, kiểm TRẠNG THÁI sau** — có chủ đích: loại bị cấm thì cấm ở **mọi** trạng thái.
Nếu kiểm trạng thái trước, một `Recover(LAND)` gửi lúc còn dưới đất sẽ bị trả lời *"chưa bay"* — một lý
do **tạm thời** che mất một lý do **vĩnh viễn**, và bất kỳ cổng nào kiểm sự từ chối đó lúc drone chưa
cất cánh sẽ **xanh vì lý do sai**.

**Ba ràng buộc mang trọng lượng an toàn** (nằm ở lib `recovery_planner`, ROS-free, 28 test):
1. **Không bao giờ bịa `home`.** `(0,0,0)` trong `odom` chính là chỗ cất cánh — một toạ độ *trông rất
   hợp lệ*. Node chỉ biết home nếu **chính nó đã cất cánh**; chưa có thì `RETURN_HOME` bị từ chối đích danh.
2. **Không bao giờ hạ xuống.** `RETURN_HOME` kết thúc **hover trên đầu** home, cách ít nhất
   `recovery.min_home_clearance_m`.
3. **Từ chối chứ không kẹp, và không bao giờ đổi loại.** Kế hoạch không bay được như yêu cầu thì trả
   `ABORTED_INVALID_GOAL` kèm lý do; đổi sang HOLD là quyết định của người gọi.

**Đường bay:** đi qua **chính lõi quỹ đạo thường** (`startGotoMotion`) — cùng spline, cùng dây xích,
cùng publisher duy nhất. 🔴 **Không mở đường tắt thứ hai vào bộ phát setpoint.** Dựng không được thì
rơi về carrot **và carrot tự publish đoạn của nó** (ở trên), nên advisor vẫn soi được.

**Preemption (cổng D4):** `Recover` tới giữa một Goto/FollowPath/TrackTarget ⇒ task cũ dừng ngay
(`LoopOutcome::PREEMPTED` → `ABORTED_SAFETY`, message *"preempted by a recovery goal"*), recovery neo
vào `motionAnchor()` như mọi task khác nên **setpoint liên tục** và **stream không đứt**. Khe hở giữa
"nhận recovery" và "task cũ dừng" được đóng bằng `recovery_pending_`: trong khe đó **mọi goal khác bị
từ chối**, nếu không một Goto lọt vào đúng lúc `state_` vừa về `HOLDING`.

| Tham số | Ship | Ghi chú |
|---|---|---|
| `recovery.min_travel_m` | 0,3 | Nhỏ hơn là nằm trong nhiễu vị trí |
| `recovery.max_climb_m` | 15,0 | Trần một lần leo, đo từ máy bay |
| `recovery.max_safe_altitude_m` | **mặc định = `max_altitude_m`** | Cố ý **không ghi trong yaml**: hai hằng số cho một trần là cách recovery được phép bay chỗ mà goal bị cấm. Đặt cao hơn `max_altitude_m` ⇒ **node từ chối khởi động** |
| `recovery.min_home_clearance_m` | 1,0 | Về tới home mà chạm đất là hạ cánh không ai cho phép |
| `recovery.max_home_distance_m` | **25,0** | **Số đo, không phải sở thích:** 30 m kèm chặng leo đã ngốn **164 s** trong ngân sách **192 s** (`0,8 × goto_timeout_sec`) của lõi quỹ đạo ⇒ nới lên chỉ sinh ra kế hoạch mà lõi từ chối |

**Feedback:** `current_stage` = 1 leo · 2 bay ngang về · 3 giữ, suy **từ pose đo được** chứ không từ bộ
đếm bước (bộ đếm sẽ tiếp tục báo "đang leo" sau khi máy bay đã ngừng leo). `stage_description` kèm số
mét. **`result.executed_type`** = loại đã bay thật; kế hoạch bị từ chối thì nó là `TYPE_UNKNOWN`.

🔴 **`TYPE_HOLD` phải SỐNG SÓT qua chính cái nó được gọi để chữa** (E2-C2, 2026-08-20). Hover là loại
khôi phục **duy nhất không đọc pose**, nên với riêng nó navigator: (a) **không** chặn ở cổng độ tin cậy
định vị, (b) **tắt hai phép kiểm định vị** trong `checkFaults()`, (c) kết thúc theo **bằng chứng dòng
lệnh** như bảng trên. Nếu giữ nguyên ba thứ đó thì trong **đúng** ca lan can này, HOLD chết trong ≤1 s
với `ABORTED_LOST_LOCALIZATION` ⇒ P8 đọc *"khôi phục THẤT BẠI"* ở đúng nút leo thang, mà đường leo thang
còn lại chỉ còn LAND/HANDOVER — ngược giáo lý [contract §2.5](../../docs/interface-contract-v0.1.md).
⚠️ Nó **biết** pose đã bị disown và vẫn giữ: bằng chứng đó nằm trong `result.message`, không giấu.
`CLIMB`/`RETURN_HOME` giữ **nguyên mọi** phép kiểm — chúng có đọc pose.
Test: `AHoldRecoverySurvivesThePoseItCannotRead` (kèm đối chứng dương: cùng lúc đó một `GotoPose` bị từ
chối vì chính pose ấy).

🔴 **`Recover` KHÔNG hỏi route.** `requestRoute()` chặn task thread tới `route_timeout_sec` (2,0 s) —
ở 0,55 m/s là **1,1 m** bay mù, so với hở đo được **0,941 m**. Recovery là đường thoát, không phải
chuyến bay tuyến; advisor vẫn canh nó qua bảng §6f.

## 7. Chạy & kiểm chứng

```bash
ros2 run uav_navigation navigator_action_server_node \
  --ros-args --params-file install/uav_navigation/share/uav_navigation/config/navigation_params.yaml \
  -p use_sim_time:=true
```

- **Unit test:** `colcon test --packages-select uav_navigation` — ma trận admission, số học carrot,
  và một test chạy **action server thật** với backend giả (bay được, arm được, hạ được).
- **Cổng G-N1 (cần sim):** [`scripts/verify_navigator.sh`](../../scripts/verify_navigator.sh) — 8 phép kiểm: goto khi chưa cất cánh
  → REJECT · frame `map` → REJECT · takeoff 2,5 m (±0,3) · goto tới waypoint · feedback giảm dần ·
  goal chồng → REJECT · **cancel giữa chừng → hover, trôi < 0,3 m trong 5 s** · land tự disarm ≤ 40 s.
  Script này chưa từng được tác giả chạy — quyền chạy sim thuộc verifier.
  🔴 **Cổng chạy trên model `uav0_nav`, không phải `uav0`:** `uav0` chỉ bridge `/clock` nên không
  có nguồn vision, mux rơi về GPS (σ_z = 0,5 m) và mọi ngưỡng ≤0,3 m đều nằm dưới sàn nhiễu.
- **Cổng G-N2 (cần sim):** [`scripts/verify_trajectory.sh`](../../scripts/verify_trajectory.sh) — bay **3 chặng Goto liên tiếp** (hai
  khúc cua 90°, chặng cuối đổi cả cao độ) rồi đo **trên chính dòng setpoint đã phát**:

  | Kiểm | Ngưỡng | Vì sao ngưỡng đó |
  |---|---|---|
  | 0. mỗi chặng có đủ mẫu & đúng nhịp | ≥40 mẫu, 12–26 Hz | **Metric rỗng phải là FAIL.** Không có bước này thì một chặng không thu được mẫu nào vẫn cho "0,00 m/s² ≤ 8" và cổng báo PASS |
  | a/b. tốc độ setpoint / máy bay | ≤1,10 / ≤1,25 × goal speed | máy bay được phép vọt hơn setpoint một chút |
  | c. **không chạm dây xích** | lead ≤0,70 m (ngân sách 0,80) | kẹp xích làm dòng phát bước nhảy 48,6 m/s² **theo thiết kế** ⇒ mọi số gia tốc bên dưới chỉ có nghĩa khi chưa kẹp |
  | c2. là quỹ đạo, không phải carrot | ≤8 m/s² | carrot đo được ~50 m/s² ⇒ tách 6× |
  | c3. trong phong bì gia tốc | ≤3 m/s² | `max_acceleration` 1,0 + biên jitter |
  | d. yaw slew | ≤4,5°/setpoint | trần vật lý 0,5 rad/s ở 20 Hz = 1,43°/tick |
  | e. bám đường đã publish | cross-track p95 ≤0,3 m | G-N1 đo goto lệch 0,055 m |
  | f/g/h. kế hoạch: đúng 1 bản/chặng, thứ tự thời gian, kết đúng đích, thu hồi sau khi xong | endpoint ≤0,05 m | hợp đồng §6c |

  **Kết quả 2026-08-18 — PASS 15/15:** gia tốc phát ra **0,23 m/s²** (carrot đo cùng cách: **20,00**,
  tách 87×) · cross-track p95 0,028–0,086 m · tới đích 0,072/0,065/0,067 m · lead 0,561 m ·
  yaw 1,49°/setpoint · kế hoạch kết đúng đích 0,000 m. ⚠️ **RTF 0,893 < 0,95 nên số liệu được dán
  nhãn** (RTF thấp làm trễ bám tăng ⇒ lead là cận trên, không phải cận dưới).

  🔑 **Đối chứng dương nằm TRONG cổng:** chạy lại chặng đầu với `use_trajectory:=false` và đòi số
  gia tốc phải **vượt** 8 m/s². Đối chứng không nổ ⇒ **cả cổng FAIL** (không hạ xuống WARNING):
  một cổng chưa chứng minh phép đo của nó bắt được lỗi thì chưa chứng minh gì.

## 8. Giả định & nợ đang có hiệu lực

- 🟠 **Navigator tin advisor về hình học** — nó không có costmap nên chỉ chặn được *hữu hạn* và
  *lệch quá xa kế hoạch*. Advisor phát điểm escape **thoáng theo bản đồ của nó nhưng sai** thì
  navigator bay theo. Đây là cái giá đã chọn của kiến trúc advisor, xem §6f.
- ✅ **ĐÃ GỠ 2026-08-20:** *"advisor chỉ soi `/planning/trajectory` nên chặng carrot vô hình"* — chặng
  carrot nay tự publish đoạn 2 điểm (§6h), và `TrackTarget` **chạy được** với
  `require_obstacle_feed=true`, có tiêu thụ advice.
- 🟡 **`Takeoff`/`HoldPosition`/`Land` publish nhưng KHÔNG có watch advice** — cố ý (bay đứng/đứng
  yên). Nghĩa là `require_obstacle_feed=true` vẫn **không** phủ ba action đó. Mở watch cho `Takeoff`
  có giá thật: chặng leo cuối chỉ còn 1 điểm phía trước ⇒ một nhịp HOLD ⇒ có thể dừng dưới
  `acceptance_radius` và **làm hỏng chính chuyến cất cánh**. Phải giải cái đó trước (xem mục dưới).
- 🟠 **`ESCAPE` khi đang bay carrot chỉ HOLD, chưa bay được escape** (§6f) — escape gắn với máy dựng
  lại quỹ đạo. Với `TrackTarget` giữa vật cản: máy bay **dừng** thay vì đi vòng, mục tiêu chạy mất.
  Mở nó cần một chặng carrot đi qua điểm escape rồi mới tới đích — **việc sau**, chưa chốt.
- 🟡 **Cuối một chặng carrot ngắn (< ~0,6 m) advisor mất kế hoạch trong một nhịp** (§6h): nó lấy phần
  đoạn phía trước điểm gần nhất, quá nửa đoạn thì phần đó chỉ còn 1 điểm. Lệch về phía an toàn
  (dừng sớm), nhưng làm chuyển động cuối chặng **giật cục** khi map bắt buộc.
- 🟡 **`Recover` rơi về carrot thì bay THẲNG tới điểm giữ**, bỏ qua waypoint trung gian của lib (carrot
  chỉ nhận một đích). Vẫn không hạ thấp (hai trục có trần riêng, chặng leo xong sớm), nhưng đường bay
  khác kế hoạch lib vẽ. Chỉ xảy ra khi lõi quỹ đạo dựng hỏng.
- 🟡 **Kịch bản bức tường KHÔNG livelock kể cả khi bỏ ghép tuyến** (đo 2026-08-19, cổng claim 7/8).
  Hai cổng đó chứng minh **hội tụ** và **có ghép tuyến thật**, **không** chứng minh rằng ghép tuyến là
  thứ ngăn livelock — hình học đã chọn tự thoát được. Muốn chứng minh phải dựng hình lõm (chữ U).
- 🟠 **Vòng phản hồi escape chưa được đo:** navigator dựng lại quỹ đạo → advisor kiểm quỹ đạo MỚI →
  có thể khuyên escape khác. `escape_refresh_m` + `escape_replan_interval_sec` chặn tần suất chứ
  **không chứng minh hội tụ**. Phải đo ở cổng bay G-N4b (đếm số lần dựng lại trong một Goto).
- 🟠 **Dựng lại quỹ đạo giữa chuyến khởi hành từ trạng thái NGHỈ** (lõi P6.2 luôn bắt/kết ở rest) ⇒
  mỗi lần `ESCAPE` là một bậc vận tốc trong dòng setpoint. Vị trí vẫn liên tục và dây xích không kẹp
  (setpoint chậm lại, drone bắt kịp), nhưng **chưa có số đo** gia tốc phát ra qua điểm dựng lại.
- **Không có bản đồ vật cản.** Bay đường (cong) giữa hai điểm; né vật cản là P6.3/P6.4.
- 🟠 **Kẹp dây xích vẫn làm dòng setpoint bước nhảy (48,6 m/s² đo được).** Sửa đúng là cho carrot
  nhả kẹp theo dốc thay vì nhả trọn một bước — thuộc P6.3 khi kiến trúc carrot được xem lại.
- 🟠 **Lưới đã publish chưa mang đoạn bàn giao yaw** ⇒ ai dự đoán hướng mũi từ `points[].yaw` sẽ sai
  đúng bằng góc lệch goal-yaw trong những giây cuối. Phải đóng **trước khi P7/P9 subscribe**.
- ✅ **ĐÃ GỠ 2026-08-20:** navigator SET đủ `plan_state`/`reason`/`sequence` trên `Trajectory3D` —
  `VALID` cho cả quỹ đạo lẫn đoạn carrot, `FAILED` khi dựng hỏng (kèm `reason`), `NO_GOAL` khi hết
  goal / không có chặng nào để kiểm. `is_valid=false` không còn gánh ba nghĩa.
- 🟡 **Lưới publish full 50 Hz** (trần 20000 điểm ≈ 1,3 MB latched). Trong máy thì vô hại; qua link
  radio thật một sample reliable cỡ MB tranh băng thông với `/fmu/in/*` → decimate trước P11.
- 🟡 Tham số `max_acceleration`/`sample_hz`/`use_trajectory` đọc **một lần lúc khởi động**;
  `ros2 param set` sẽ báo thành công mà không đổi gì.
- `HoldPosition` neo vào **setpoint** đang giữ, không phải vị trí đo được — nếu không, mỗi lần gọi
  hold lại nhích theo nhiễu định vị.
- `min_altitude_m`/`max_altitude_m` đo trong frame `odom`, mà gốc `odom` là gốc EKF2 **chứ không
  phải mặt đất** ([`docs/package-status.md`](../../docs/package-status.md) §2). Với `uav_arena` hai thứ lệch ~0,1 m; ở nơi cất cánh
  khác cao độ thì phải chỉnh lại tham số.
- Mất `odometry_fused` quá `odometry_timeout_sec` → abort action + **giữ hover**, không hạ cánh
  (`plan P6` §3). Quyết định *khi nào* recover là P8; P6.6 chỉ **thực thi** `Recover`.
- ✅ **ĐÃ GỠ 2026-08-20 (E2-C2):** `Recover(TYPE_HOLD)` nay **sống sót** qua pose bị disown / stamp cũ —
  tắt hai phép kiểm định vị cho riêng nó và kết thúc theo bằng chứng dòng lệnh (§6h). Trước đó nó abort
  `ABORTED_LOST_LOCALIZATION` trong ≤1 s, tức **mã kết quả nói ngược** với trạng thái vật lý.
- 🟡 **Lan can đó chỉ che định vị, không che mất quyền điều khiển.** `Recover(TYPE_HOLD)` vẫn abort khi
  offboard rớt / autopilot cầm lái / failsafe — đúng (không giành quyền), nhưng nghĩa là hover-khôi-phục
  **không** phải phương án cuối cùng cho *mọi* lỗi, chỉ cho lỗi định vị.
