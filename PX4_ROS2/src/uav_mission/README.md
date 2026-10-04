# `uav_mission`

Cao nhất trong autonomy stack: orchestrate các action của `uav_navigation`
qua Behavior Tree, phản ứng theo world model + safety/authority. **Mission
không tự bay** — chỉ gọi action/service của package khác.

Trạng thái: ✅ **P9 ĐÓNG TRỌN 2026-08-23** — **9/9 cổng bay (G-M0→G-M4.4) + M5
PASS**, cả 3 mission bay end-to-end; 2 lượt review chốt "KHÔNG CÒN CHẶN".
**125 case / 5 target, 0 lỗi**, domain test **98** (Q-P9-3).
Thành phần: **3 lib ROS-free** (`mission_policy`, `mission_registry`,
`nav_goal_broker`) + `mission_executor_ros` (node BT thật: action server
`ExecuteMission`, 4 service Load/Pause/Resume/Abort, 7 action client, 6 BT
leaf node tuỳ biến).
Hồ sơ phase → `../../.claude/plan/P9-mission.md` ·
nợ mở → [`../../docs/package-status.md`](../../docs/package-status.md) §11.

## 4 điều CẤM (chống bẫy "mission ghi đè safety"; reviewer grep)

`uav_mission` **KHÔNG BAO GIỜ** được:
1. Publish `ControlCommand` hay bất kỳ topic `control/*`.
2. Gọi service `clear_fault` / `clear_safety_latch`.
3. Gọi arm/disarm/set_mode.
4. Gọi `SetControlAuthority`.

Mission chỉ nói chuyện với `uav_navigation` (qua 7 action) và tự publish
`/uav/<id>/mission/status|events`. Mọi thứ khác đọc-only (authority, safety
state, localization status, battery).

## Lệch thiết kế CÓ CHỦ ĐÍCH: 3 node → 1 node + 3 lib

Thiết kế gốc (`CLAUDE.md` §4.9) liệt kê 3 node (`mission_executor_node`,
`mission_bt_runner_node`, `mission_registry_node`). Theo tiền lệ P8
(`uav_safety` gộp 2 node → 1 node + lib `failsafe_policy`), P9 gộp thành
**1 node** (`mission_executor_node`, P9.4 — action server + BT engine)
**+ 3 thư viện ROS-free**:

- `mission_policy` — bảng quyết định reactive `MissionWorldView → GuardVerdict`.
- `mission_registry` — catalog 3 mission + parse/validate `mission_params`.
- `nav_goal_broker` — FSM "tối đa MỘT goal navigator, mãi mãi".

⚠️ **Chữ ký D-1 (2026-08-22) ghi "1 node + 2 lib"** — `nav_goal_broker` được
tách thành lib thứ ba lúc thi công P9.4, cùng lý do với hai lib kia: FSM một-goal
là luật an toàn, pin được bằng test thì không phải tin một chuyến bay.
`CMakeLists.txt` là nguồn chuẩn của số lib.

Lý do giống P8: tách logic an toàn/quyết định khỏi lớp ROS làm nó **pin được
bằng test**, không phải trust một chuyến bay. 3 node riêng biệt sẽ chỉ thêm
độ trễ giao tiếp giữa chúng mà không thêm gì về an toàn.

## "Guard trong CODE, thân mission trong XML"

`MissionGuard` (decorator C++ trong `mission_executor_node`, P9.4) gọi
`mission_policy::MissionPolicy::evaluate()` mỗi tick (10 Hz) và bọc quanh
SubTree nạp từ `config/missions/<id>.xml`. **`MissionGuard` không bao giờ là
một tag trong file XML** — một mission body chỉ có thể dùng node trong
whitelist (`mission_registry.cpp::allowedXmlTags()`), không thể tự gỡ hay
bỏ qua nhánh an toàn bằng cách sửa text. `MissionRegistry`'s constructor
validate whitelist của cả 3 XML shipped ngay lúc khởi động (fail-fast).

## `mission_policy` — bảng ưu tiên (P9 plan §2)

`evaluate(MissionWorldView) → GuardVerdict`, dừng ở điều kiện đầu tiên cắn:

1. Operator abort → `ABORT_HOLD` (không hạ cánh, đúng `AbortMission.srv`).
2. Control authority bị giành khỏi MISSION, giữ ≥ `authority_loss_grace_sec`
   (mặc định 1.5 s; R32: message hết hạn = coi như không giữ quyền) → `PAUSE`.
3. Battery: NaN quá `battery_unknown_timeout_sec` (30 s) hoặc dưới WARN
   (0.35) quá `battery_warn_dwell_sec` (3.0 s, R29) → `END_EARLY`. Kiểm SAU
   authority (một lệnh end_early gửi khi không giữ quyền là lệnh vào hư không).
4. Pause service, hoặc đang PAUSED (quá `paused_timeout_sec` 180 s không
   Resume → `ABORT_HOLD`/`ABORTED_TIMEOUT`) → `PAUSE`.
5. Lỗi thân (kết quả action navigator vừa xong): mất định vị → `ABORT_HOLD`
   ngay không retry; NO_AUTHORITY/REJECTED → `PAUSE` như mục 2; lỗi khác →
   retry tới `max_step_retries` (2) trừ khi lý do lặp lại hoặc mã là
   `ABORTED_SAFETY` (không bao giờ retry mù); hết ngân sách → Q-P9-1: chỉ
   `ABORT_LAND` khi CÒN giữ quyền MISSION + định vị hợp lệ + lý do không
   liên quan định vị, ngược lại `ABORT_HOLD`.

`evaluate()` là **hàm thuần** (không đọc đồng hồ, R21) — mọi dwell là
`*_elapsed_sec` do caller (broker P9.4) tính, khác phong cách
`ContinuityTracker` nội bộ của `failsafe_policy`. Lựa chọn có chủ đích: giữ
"trái tim an toàn" là một bảng quyết định thuần, test không cần vòng lặp
thời gian (R27-3).

## `mission_registry` — catalog + validate `mission_params`

`load(mission_id, params_str)` parse YAML một lần (YAML ⊇ JSON), validate
theo schema riêng từng mission (key lạ / số không hữu hạn / z ngoài
`[min_alt_copy_m 0.5, max_alt_copy_m 8.0]` / chặng > `goto_reach_copy_m` 40.0
/ `target_speed_mps` > 0.275 ⇒ **từ chối**), trả `MissionPlan` đã gõ kiểu +
`total_steps`.

`total_steps` **không phải** số leaf BT thô — định nghĩa rõ trong
`mission_registry.hpp`:
- `indoor_patrol`: = `loops` (mỗi vòng `Repeat` = một lần thăm waypoint).
- `follow_target`: cố định `1` (hành vi reactive liên tục, không có bước rời rạc).
- `inspect_point`: cố định `4` (approach, find_marker, goto_over_marker,
  dwell — "ghi pose" là tác dụng phụ của bước dwell hoàn tất, không phải
  leaf riêng; P9.4 cần xác nhận lại khi nối `MissionStatus.current_step_index` thật).

`InspectMarker.action` **không** được dùng (D-5, chưa có server trong
navigator) — `inspect_point` soạn hoàn toàn từ 7 action sẵn có.

## `nav_goal_broker` — FSM một-goal-mãi-mãi (P9.4)

`NavGoalBroker` (ROS-free, `include/uav_mission/nav_goal_broker.hpp`) là
FSM `IDLE/SENDING/ACTIVE/CANCELING/CANCEL_REFUSED` sau **một** mutex.
`request()` chỉ dispatch từ `IDLE` (không hàng đợi, khớp navigator
"REJECT ngay, không xếp hàng"); `cancel()` chỉ dispatch từ `ACTIVE`. Mọi
lệnh `async_send_goal`/`async_cancel_goal` thật do **caller** (
`mission_executor_node`) tiêm vào qua `std::function`, nên lớp FSM này
không include `rclcpp_action` — test được cả không cần ROS.

`mission_executor_node.hpp`'s `sendTypedGoal<ActionT>()` (template, 1 chỗ)
là nơi DUY NHẤT gọi `async_send_goal`/`async_cancel_goal` thật, dùng chung
cho cả 4 lời gọi (`sendGotoPose`/`sendTrackTarget`/`dispatchTakeoff`/
`dispatchLand`) — đảm bảo "tối đa một goal navigator" là bất biến **node
rộng**, không chỉ trong một cây BT.

## `mission_executor_node` (P9.4) — kiến trúc thật, không như plan mô tả nghĩa đen

- **`MissionContext`** (`include/uav_mission/mission_context.hpp`) là
  interface thuần ảo cắt vòng include: `bt_nodes.hpp` (6 leaf tuỳ biến) chỉ
  cần header này, không cần biết `MissionExecutorNode`.
- **Guard 2 tầng, không phải 1 chỗ như đọc nghĩa đen plan §1:**
  - **Top-level** (`onTick()`, trước khi tick cây BT): CHỈ ưu tiên 1–4
    (operator abort/authority seize/battery/pause) — `last_nav_result_code`
    luôn `UNKNOWN` nên ưu tiên 5 KHÔNG BAO GIỜ cắn ở đây.
  - **Trong leaf `NavAction`** (`onRunning()`, khi navigator vừa trả lỗi):
    gọi `evaluateStepFailure()` → **cùng** hàm `MissionPolicy::evaluate()`,
    nhưng ưu tiên 1/3/4 bị ép trơ (operator/pause = false, `*_elapsed_sec`
    ghim 0), **ưu tiên 2 chỉ ghim `authority_seize_elapsed_sec=0`** —
    `authority_active_source`/`age` vẫn THẬT, để Q-P9-1's
    `still_holds_authority` không nói dối.
  - Lý do tách: BT.CPP không có cơ chế "retry một leaf theo backoff do một
    node khác quyết" sẵn có; để retry/backoff/Q-P9-1 sống ĐÚNG chỗ dữ kiện
    của nó (kết quả navigator vừa trả) mà không nhân đôi logic.
- **"Finish" (GotoPose(home)→Land) là state machine C++ thuần**
  (`ExecPhase::kFinishGotoHome/kFinishLanding`), **KHÔNG phải BT subtree** —
  dùng chung cho CẢ BA: SUCCESS (hạ cánh bình thường), `END_EARLY` (pin),
  `ABORT_LAND` (hết ngân sách retry, Q-P9-1 cho phép). Đây là "CHUNG subtree
  Finish" plan §2 nhắc tới, không phải một tag XML.
- **R24: `tick_group_` (MutuallyExclusive, chỉ timer 10 Hz) + `io_group_`
  (Reentrant, mọi sub/service/action).** Khác diễn giải hẹp "chỉ chạm state
  qua broker" — thực tế `phase_`/`tree_`/mission bookkeeping/`goal_handle_`
  CŨNG bị cả hai nhóm chạm (io_group_ ghi lúc accept goal/gọi service,
  tick_group_ đọc-ghi mỗi tick) nên có **một** `state_mutex_` bổ sung
  (ngoài `broker_`'s mutex riêng): `onTick()` giữ nó SUỐT một tick (kể cả
  lúc tick cây BT lồng vào — mọi lệnh gọi `MissionContext` bên trong coi
  như ĐÃ có khoá), mỗi handler `io_group_` khoá ngắn cho phần việc của nó.
- **R32 thêm cho battery:** `freshBatteryOrNan()` — một bản đọc pin từ
  `/state/health_px4` chết phải hết hạn về NaN (2× chu kỳ 10 Hz), không giữ
  giá trị cuối mãi mãi.
- 🔴 **B1 (2026-08-23, review round 1): authority-seize KHÔNG bị suppress
  trong Finish/Recovering nữa.** Khối chống-livelock trong `onTick()`
  (kFinishGotoHome/kFinishLanding/kRecovering) trước đây tắt CẢ ưu tiên #2
  (authority-seize) lẫn #3 (battery) — SAI, vì lý lẽ livelock chỉ đúng cho
  điều kiện MỨC (battery ở dưới WARN mãi mãi, re-trigger lại restart Finish
  vô hạn), KHÔNG đúng cho điều kiện CẠNH (authority-seize là một sự
  chuyển-trạng-thái, không phải trạng thái đứng yên). Sửa: cô lập RIÊNG ưu
  tiên 2 (mọi field khác ép trơ, cùng kỹ thuật `evaluateStepFailure()` đã
  dùng) — battery/pause/khác vẫn suppress y hệt cũ.
  **Cách phục hồi khi Resume:** cancel goal đang bay (best-effort) →
  `phase_` vào `kPaused`, nhưng `paused_from_phase_` LƯU lại chính pha
  Finish/Recovering đang dở (`kFinishGotoHome`/`kRecovering` — **KHÔNG
  còn `kFinishLanding`, xem miễn-guard G-M1 dưới**) — cơ chế CÓ SẴN, dùng
  chung với pause từ `kBody`. `ResumeMission` gán lại
  `phase_ = paused_from_phase_` (đã tổng quát từ trước) + reset
  `phase_goal_dispatched_ = false` ⇒ Resume **redispatch lại đúng leg đó**
  (GotoPose(home)/Recover), không quay lại đầu thân mission.
  🔴 **`pending_termination_` CỐ Ý không đụng tới** trong nhánh authority-
  seize-trong-Finish — nó đang giữ lý do Finish GỐC (`beginFinish()`'s
  SUCCESS/END_EARLY/ABORT_LAND result_code+reason, chỉ tiêu thụ khi Land
  cuối cùng resolve); nếu tái dùng `beginCancelFor()` (ghi đè
  `pending_termination_` bằng verdict pause) như pause từ `kBody` thì sẽ
  MẤT lý do Finish gốc vĩnh viễn.
  🔴 **G-M1 review round 1 (2026-08-23) — `kFinishLanding` MIỄN guard
  authority-seize, `kFinishGotoHome`/`kRecovering` GIỮ NGUYÊN.** Bay hồi quy
  bắt: PAUSE giả ĐÚNG lúc vào `kFinishLanding`, `active_source=NONE`,
  `age_sec` răng cưa sạch 2 Hz (arbiter/navigator đều KHỎE — không phải đói
  executor như nghi ban đầu). Cơ chế thật (chốt bằng số trên dây,
  `gm1_diag1.log`): navigator trao quyền LANDING cho **PX4 LAND native**,
  không ai stream `cmd_mission` nữa nên arbiter báo NONE ĐÚNG THIẾT KẾ (an
  toàn khi hạ cánh, không phải bị giành). Vị ngữ `active_source != MISSION`
  của B1 không phân biệt "bị giành" với "không ai cần giữ" — cùng bài học
  B5-P8: **vị ngữ an toàn phải kèm tiền đề pha**, không chỉ đọc tín hiệu
  thô. `kFinishGotoHome`/`kRecovering` KHÔNG được miễn — NONE ở 2 pha đó
  vẫn là bất thường thật (navigator chết), chiều làm-ít-đi đúng nên giữ
  nguyên guard. Miễn giảm còn hợp lý ở khía cạnh hành động: cancel-khi-
  LANDING vốn đã bị navigator TỪ CHỐI theo hợp đồng (CANCEL_REFUSED) — nên
  PAUSED lúc `kFinishLanding` chỉ tạo ra trạng thái mồ côi không hành động
  được, trong khi để Land hoàn tất và báo đúng kết quả gốc mới THẬT SỰ làm
  được gì đó. `operator_abort_requested_` vẫn được honor ở CẢ 3 pha như cũ
  (không đổi). WARN chẩn đoán (đã chứng minh giá trị — chính nó chốt được
  cơ chế) HẠ CẤP còn 1 lần/đợt seize liên tục (`authority_seize_warned_
  this_episode_`, cùng nếp Y6), không còn spam mỗi tick.
- 🟡 **Y6 (2026-08-23): battery-WARN trong lúc PAUSED — GIỮ hành vi bay,
  THÊM observability.** Quyết định coordinator: PAUSED nghĩa là đã có lệnh
  dừng (operator/guard khác), node **không tự bay** vì lý do pin trong lúc
  đó (CRITICAL/PX4 failsafe lõi vẫn đỡ) — logic BAY không đổi. Cái THIẾU là
  quan sát: nay phát `MissionEvent` (tái dùng `EVENT_PAUSED` — `uav_interfaces`
  không có type "cảnh báo" riêng, đây là package khác, ngoài phạm vi sửa ở
  đây — description phân biệt rõ) đúng 1 lần mỗi đợt battery WARN/unknown
  liên tục trong khi paused (edge-latch, không spam 10 Hz), đọc field thô
  của `MissionWorldView` trực tiếp (không qua `verdict.action`) để không bị
  che bởi authority-seize ưu tiên cao hơn. Xem contract §2.19.

## 5 lệch có chủ đích so với plan (P9.4, ghi để P9.5+ biết)

1. **`NavAction` chỉ hỗ trợ `nav_type` ∈ {`GotoPose`, `TrackTarget`}** — hai
   giá trị duy nhất 3 XML shipped dùng. Takeoff/Land được node tự gọi
   ngoài BT (Takeoff trước thân, Land trong Finish); HoldPosition/
   FollowPath/Recover có action client kết nối (registry 7 client đủ) nhưng
   **không** đường XML nào gọi tới — `nav_type` khác bị từ chối ngay ở
   `onStart()` (`ABORTED_INVALID_GOAL`, không retry).
2. **`target_id` (follow_target) và các điểm search (follow_target +
   inspect_point) KHÔNG có field trong `mission_registry`** (schema đã đóng
   băng ở P9.2/P9.3). Mặc định ở tầng node: `target_id=-1` (bám mục tiêu
   đầu tiên thấy được), 2 điểm search = home/approach ± `search_offset_m`
   (mặc định 2,0 m) theo trục X, cùng cao độ approach/`takeoff_altitude_m`.
   Việc tinh chỉnh (nếu cần) thuộc P9.6–P9.9.
   🔴 **Neo danh tính (G-M4.4, 2026-08-23):** `target_id=-1` chỉ là bộ lọc
   "bám bất kỳ" cho `TargetSeen` — **goal `TrackTarget` thật sự gửi đi
   KHÔNG BAO GIỜ mang `-1`**. `TargetSeen` ghi `track_id` nó VỪA thấy ra
   blackboard (`{acquired_track_id}`), và `NavAction` đọc port `target_id`
   từ ĐÓ, không phải từ `{target_id}`. Một goal ĐANG CHẠY giữ nguyên id đã
   neo tới khi nó tự kết thúc (không đổi id giữa chừng dù blackboard đổi) —
   `onStart()` chỉ đọc port MỘT LẦN. Khi goal đó abort `LOST_TARGET` → rẽ
   search → `TargetSeen` tái-bắt thành công sẽ neo id MỚI cho goal kế tiếp
   — "bám vật vừa tìm được sau search" là đúng thiết kế, KHÔNG phải bug,
   kể cả khi id mới thuộc một vật thể khác (mission không tự phân biệt
   "cùng một mục tiêu logic" qua nhiều lần mất dấu — muốn chọn lọc chặt hơn
   phải truyền `target_id` cụ thể qua mission_params một khi schema mở lại).
   Lý do: trước fix này, `target_id=-1` gửi thẳng cho navigator vô hiệu hoá
   luôn bộ lọc `targetSample(wanted_id)` phía navigator
   (`navigator_action_server_node.cpp:2597`), đánh bại CẢ bộ lọc M/N phía
   tracker LẪN cơ chế pin-theo-id phía navigator — track ma lọt qua và
   reset đồng hồ `target_lost_timeout` vô thời hạn.
   🔴 **Ngân sách search theo episode (G-M4.4b, 2026-08-23):** một "loss
   episode" bắt đầu khi `TrackTarget` kết thúc KHÔNG-thành-công (ngân sách
   retry riêng của bước đó cạn, không phải mỗi lần LOST_TARGET thô); mỗi lần
   vào `search_when_lost` tăng bộ đếm (`MissionContext`, KHÔNG phải trong
   XML) — bộ đếm này SỐNG SÓT qua một cú nháy `TargetSeen` một khung (khác
   `resetStepRetryState()`, cái reset MỖI lần `onStart()`), chỉ reset khi có
   tiến triển bám THẬT (`TrackTarget` chạy ổn ≥ `track_progress_reset_sec`,
   mặc định 10 s). Cạn `search_attempts_max` (mặc định 2) → node điều
   `SearchBudgetAvailable` (leaf mới) chặn `search_when_lost` NGAY từ đầu →
   cây trả FAILURE với verdict `ABORTED_LOST_TARGET` còn nguyên → node tự
   dispatch `Recover(CLIMB)` rồi kết thúc mission `ABORTED_LOST_TARGET`
   (KHÔNG BAO GIỜ `ABORTED_TIMEOUT`, dù outer `Timeout` chưa hết hạn) — khớp
   plan P9 §3.6.
   🔴 **bug #12 (2026-08-23) — tái-bắt cuối `search_when_lost`:** `search1`
   + `search2` thành công CHỈ là điều hướng tới, KHÔNG phải tìm thấy —
   `search_when_lost` có thêm `target_visible_recheck` (khuôn
   `marker_visible_recheck` của `inspect_point.xml`) làm con CUỐI; bay hết
   2 điểm mà không thấy ⇒ recheck FAIL ⇒ cả `search_when_lost` FAIL ⇒ tiêu
   1 episode ngân sách. Bộ đếm ngân sách CHUYỂN từ "TrackTarget tự thất
   bại" (hook gốc G-M4.4b, không thấy được trường hợp mission KHÔNG BAO GIỜ
   dispatch TrackTarget — "arena hoàn toàn sạch") sang `SearchBudgetAvailable
   ::tick()` (`bt_nodes.cpp`) — chạy VÔ ĐIỀU KIỆN mỗi lần vào
   `search_when_lost`, phủ CẢ hai đường (đã từng bám rồi mất VÀ chưa từng
   thấy target lần nào). `tickBody()` (`mission_executor_node.cpp`): recheck
   fail mà ngân sách còn ⇒ **retry** (không kết thúc mission) — phân biệt
   với outer `<Timeout>` hết hạn thật bằng đồng hồ C++ độc lập
   (`mission_start_time_`, KHÔNG bao giờ reset khi cây BT tự khởi động lại
   sau một trạng thái terminal, khác `<Timeout>` node của chính cây). Mặt
   SUCCESS đối xứng: `search_when_lost` thành công (tái-bắt) cũng KHÔNG
   BAO GIỜ đọc là mission SUCCESS (follow_target vốn không có trạng thái
   "xong" hợp lệ, `TrackTarget` luôn `duration_seconds=0.0`) — retry giống
   `loop_mission_`. Ghi chú: trong CÂY THỰC TẾ, `ReactiveFallback` tick lại
   `track_when_seen` từ đầu MỖI tick ngoài, nên nhánh này luôn "thắng" một
   tái-bắt thật TRƯỚC KHI recheck có cơ hội tự thành công — nhánh SUCCESS
   của recheck là phòng thủ, không phải đường thật sự chạy trong cây này.
3. **Mission tự động Takeoff trước thân + Finish sau thân** — không mission
   XML nào tự gọi Takeoff/Land; node coi đó là bao bọc chuẩn của MỌI
   `ExecuteMission` (khớp thực tế vận hành: mission không cất/hạ cánh thì
   vô nghĩa).
4. **`ExecuteMission.Goal.loop=true`** → thân BT được tick lại từ đầu sau
   mỗi SUCCESS (dựa `Tree::tickRoot()` tự đưa root về IDLE sau SUCCESS/
   FAILURE — hành vi thư viện, đã xác nhận đọc `bt_factory.h`), Finish CHỈ
   chạy khi `loop=false` hoặc mission bị pause/abort/end_early.
5. **`mission_id` rỗng trên 4 service Load/Pause/Resume/Abort** = "mission
   đang chạy" (đúng comment `.srv`); diễn giải dòng contract §2.19 về
   `mission_id` trong GOAL `ExecuteMission` là: **luôn từ chối goal thứ 2**
   bất kể `mission_id`, không có khái niệm "đổi mission giữa chừng" — dòng
   "rỗng = đang chấp nhận" trong contract mô tả nếp chung của 4 service,
   không phải một nhánh riêng trong `handleGoal()`.

## Build & test (WSL)

```bash
rm -rf ~/PX4_ROS2/src/uav_mission
cp -r /mnt/c/code/PX4_ROS2/src/uav_mission ~/PX4_ROS2/src/
cd ~/PX4_ROS2 && colcon build --packages-select uav_mission uav_control_authority
colcon test --packages-select uav_mission && colcon test-result --verbose
```

`ROS_DOMAIN_ID` không cần khai cho `test_mission_policy`/`test_mission_registry`
(R20) — không `rclcpp::init`, không domain participant nào được tạo.
`test_mission_executor_node` và `test_mission_authority_chain` (P9.4/P9.4b)
**cần** `ROS_DOMAIN_ID=98` (Q-P9-3, đã khai trong `CMakeLists.txt`) — chúng
bịa traffic thật trên tên topic thật (`/uav/<id>/planning/*`,
`/uav/<id>/control/*`), tách domain để không đụng sim đang chạy.

⚠️ **`test_mission_authority_chain` cần build `uav_control_authority`
cùng lượt** (nó link `control_authority_manager_node` thật, khuôn
`test_safety_cut_chain.cpp`).
