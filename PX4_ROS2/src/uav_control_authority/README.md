# `uav_control_authority` — ai đang cầm lái

Package trả lời câu hỏi *"trong 4 nguồn có thể phát lệnh, nguồn nào đang thật sự lái máy bay"*. Chỉ có
**một** `control_authority_manager_node`, và nó là **rơ-le thuần** — không bao giờ tự sáng tác một
setpoint. Backend PX4 vẫn là nơi duy nhất chạm `px4_msgs` (R1); package này chỉ đọc/viết message nội bộ.

> Kế hoạch & lý lẽ thiết kế đầy đủ → `.claude/plan/P7-control-authority.md`
> · Hợp đồng message → [`docs/interface-contract-v0.1.md`](../../docs/interface-contract-v0.1.md) §2.2, §2.3

**Trạng thái (P7 ĐÓNG 2026-08-21; N-c ĐÓNG P10.8a 2026-08-23; C4 ĐÓNG P10.8b 2026-08-23):** package build
0 warning, 2 target test xanh — `test_authority_arbiter` (ROS-free, **77 case**) +
`test_control_authority_manager_node` (domain 96, **21 case**, + 7 ca luật thuần) = **98 case / 2 target** (68 → 72: +4
`ClockRewind.*` của P10.8a fix N-c; 72 → 77: +5 `ClockRewind.*` của P10.8b fix C4 — 2 phép trừ thời gian
mà N-c bỏ sót, xem §2 mục 6). **Đã nối vào
`sim.launch.py` VÀ đã LÀ single-writer thật của `command_selected`** (P7.2 nối node, P7.3 đổi dây —
cờ `control_authority` mặc định **`true`**). `navigator_action_server_node` phát vào
`/control/cmd_mission`, **không còn phát thẳng vào `command_selected`** — trọng tài là nơi duy nhất
ghi topic đó. Y12 (2026-08-20): đoạn này từng ghi ngược sự thật (cờ mặc định false, navigator vẫn
phát thẳng) — đó là mô tả ĐÚNG ở P7.1/P7.2, đã LẠC HẬU từ khi P7.3 đóng; đã sửa lại đúng hiện trạng.

---

## 1. Bốn nguồn vào, một nguồn ra

| Topic vào | Priority | Ai phát |
|---|---|---|
| `/uav/<id>/control/cmd_safety` | 4 SAFETY | `uav_safety` (P8) — chưa có publisher |
| `/uav/<id>/control/cmd_operator` | 3 OPERATOR | teleop/GCS (P11) — chưa có publisher |
| `/uav/<id>/control/cmd_mission` | 2 MISSION | `navigator_action_server_node` |
| `/uav/<id>/control/cmd_test` | 1 TEST | 4 probe bay hồi quy |

| Topic ra | Ghi chú |
|---|---|
| `/uav/<id>/control/command_selected` | **SINGLE WRITER vĩnh viễn.** QoS Reliable/KeepLast(10)/Volatile — không đổi, gateway đang đọc đúng hồ sơ này |
| `/uav/<id>/control/authority` | "Ai đang cầm lái" — phát khi đổi + nhịp tim 2 Hz. QoS latched (KeepLast(1)/TransientLocal) |
| `/uav/<id>/diagnostics/control_authority` | Mọi số đo phụ (drop count, publisher trùng, `clock_regressions` — N-c, §2 mục 6...) đi đường này, không lẫn vào `authority` |

**Luật chọn:** một topic = một mức ưu tiên. Nguồn LIVE (có tin đến trong `source_timeout_sec`) ưu
tiên cao nhất thắng. Không nguồn nào LIVE ⇒ `active_source = NONE`, **không publish gì** — im lặng là
cố ý (§2 dưới).

---

## 2. Sáu điều PHẢI biết trước khi sửa file này

1. **Node này KHÔNG BAO GIỜ tự phát setpoint.** Khi mọi nguồn im lặng, nó không hover hộ, không giữ
   nhịp — nó im theo. `px4_command_gateway_node` đã tự đệm 0,5 s (`command_timeout_sec`) và PX4
   failsafe khi hết đệm; đó là hành vi **đúng**. Một trọng tài tự sáng tác hover sẽ giữ offboard sống
   khi KHÔNG AI lái — chế độ hỏng tệ nhất có thể thiết kế ra. Nếu một sửa đổi khiến node này publish
   `command_selected` mà không có message nguồn tương ứng vừa tới → SAI, dừng lại.
2. **Toàn bộ số học sát an toàn (priority, hysteresis, latch) nằm trong `authority_arbiter`
   — thư viện ROS-free**, không phải trong node. `control_authority_manager_node.cpp` chỉ convert
   message và gọi 3 hàm: `onMessageArrived` (sự kiện), `onTick` (giám sát 1/`kMonitorPeriodSec` = 20 Hz),
   `requestLatch` (service). Sửa luật arbitration → sửa `authority_arbiter.cpp` + chạy
   `test_authority_arbiter` (không cần domain, chạy trong ~vài chục ms). Đừng nhét logic quyết định
   vào node.
3. **Downgrade (nhả quyền) CHỈ xảy ra qua `onTick()`, không qua `onMessageArrived()`.** Một message
   đến chỉ có thể đẩy quyền LÊN (chiếm ngay, không dwell); nhả quyền XUỐNG cần `release_dwell_sec`
   giây im lặng liên tục, và chỉ timer giám sát mới phát hiện được sự im lặng đó (im lặng không sinh
   callback). Test nào mô phỏng "một nguồn ngưng phát" mà không tự gọi `onTick()` song song sẽ pass
   giả — xem `test_authority_arbiter.cpp` các test `Hysteresis.*`/`Latch.*` để biết khuôn đúng.
4. **🔴 R24 — TUYỆT ĐỐI không cho service `SetControlAuthority` một callback group riêng.** `arbiter_`
   không tự khoá (không mutex/atomic); tính đúng đắn của toàn bộ node dựa vào việc **subs + timer +
   service cùng một `io_group_` MutuallyExclusive** nên không thể có hai callback chạm `arbiter_` cùng
   lúc. Đây chính là lỗi B1 một review bắt được 2026-08-20 (SAFETY có thể mất quyền ngay sau khi giành
   được, vì service tính trên snapshot cũ). Nếu cần thêm entry point mới chạm `arbiter_` → cũng phải
   vào `io_group_`, không có ngoại lệ.
5. **🔴 B2 — GARBAGE KHÔNG BAO GIỜ được tính là bằng chứng sống.** `authority_arbiter` giữ **hai** đồng
   hồ đến riêng biệt: `last_arrival_` (mọi message, kể cả rác — chỉ để chẩn đoán) và
   `last_valid_arrival_` (CHỈ message qua được `validateContent()`). `isLive()`, `latch_ever_alive_`,
   `release_dwell_sec`, và `processLatchExpiry()` **chỉ được phép đọc `last_valid_arrival_`**. Trước
   review 2026-08-20, cả bốn thứ đó đọc nhầm `last_arrival_` — hậu quả: kênh toàn rác cướp được quyền
   bằng ưu tiên, latch bị giữ hiệu lực vĩnh viễn bởi chính bên giữ phát rác, và một kênh có thể giữ
   quyền vô hạn bằng nhịp 1-hợp-lệ:9-rác. Sửa sai chỗ này lần nữa = mở lại đúng lỗ hổng đó.
6. **🔴 N-c (P10.8a) + C4 (P10.8b, chốt 2026-08-23) — đồng hồ LÙI không bao giờ được đọc thành "kênh
   còn sống mãi mãi" hoặc "lệnh vẫn còn tươi".** Mọi phép trừ `now − stamp` trong file này đi qua
   **một** helper `ageSecOrInf()`: `now < stamp` (rewind, vd replay/loop bag, hoặc publisher quên
   `use_sim_time` nên stamp bằng epoch thật) ⇒ tuổi **+inf** — cùng giá trị "chưa từng nhận" đã dùng
   sẵn — và đếm `clock_regressions_` (expose qua `clockRegressionsCount()`, vào
   `/diagnostics/control_authority` key `clock_regressions`, không thoái lui im lặng). N-c (P10.8a)
   route được **4** chỗ: `isLive`, `rawArrivalAgeSec`, `processLatchExpiry` x2, `onTick`'s dwell check.
   🔴 **C4 (P10.8b, review đóng phase phát hiện):** N-c bỏ sót **2** chỗ trừ thời gian khác trong CHÍNH
   file này — `validateContent()`'s guard tuổi **NỘI DUNG** lệnh (`check_command_age`/
   `max_command_age_sec`, cổng DUY NHẤT chặn lệnh cũ trên toàn tuyến, vì gateway hạ nguồn chỉ đo thời
   điểm ĐẾN chứ không đọc `header.stamp`) và `latchAgeSec()` (chỉ báo Y15 dùng để phát hiện "safety
   node chết mà vẫn giữ latch", §4). Cả hai vẫn dùng phép trừ trần trước C4 — rewind khiến STALE-guard
   thành no-op (lệnh cũ lọt qua như mới nhất, không tăng `clock_regressions_`) và `latchAgeSec()` trả
   về SỐ ÂM (đọc nhầm thành "rất mới", ngược hẳn ý đồ chỉ báo). Đã sửa: STALE-guard route qua
   `ageSecOrInf()` như 4 chỗ kia (rewind ⇒ +inf ⇒ luôn > `max_command_age_sec` ⇒ DROP); `latchAgeSec()`
   cũng route qua đó nhưng đổi `+inf` thành **NaN** trước khi trả về — nó là số hiển thị chẩn đoán,
   không so sánh ngưỡng, nên "không đo được" phải là NaN (R30), không phải giá trị lớn nhất có thể.
   Hệ quả **ĐÃ KÝ** (Q-P10-7, áp dụng tiếp cho C4): kênh rơi về không-LIVE (an toàn — im lặng, không
   phải "cướp quyền"); latch OPERATOR/MISSION/TEST bị nhả (`GRACE_EXPIRED`/`TIMED_OUT`) khi đồng hồ
   lùi. 🔴 **Latch SAFETY tuyệt đối không bị nhả** — hai nhánh `latch_level_ == kSourceSafety` trong
   `processLatchExpiry()` return TRƯỚC khi chạm helper, R5 nguyên vẹn (C4 không đổi gì ở đây). Lưu ý
   phụ: `onTick()`'s dwell-release trên `active_source_` là cơ chế CHUNG cho **mọi** kênh kể cả SAFETY
   (đã vậy từ trước N-c, không phải hành vi mới) — một rewind có thể khiến `active_source_` tạm về
   `NONE` dù latch/floor SAFETY còn nguyên, nghĩa là im lặng an toàn một tick, **không** phải mất
   quyền: floor vẫn chặn mọi kênh thấp hơn suốt lúc đó, và SAFETY tự chiếm lại ngay ở message kế tiếp
   (lên-ngay, không dwell, như mọi kênh khác).

---

## 3. Bộ lọc hợp lệ (chốt chặn cuối trước backend)

| Kiểm | Xử lý |
|---|---|
| Trường của `control_mode` đang dùng không hữu hạn (NaN/inf) | DROP + diagnostics ERROR |
| `header.frame_id != odom_frame` | DROP + diagnostics ERROR |
| `control_mode` ngoài {POSITION, VELOCITY, ACCELERATION} | DROP + diagnostics WARN |
| Tuổi `header.stamp` quá `max_command_age_sec` (khi `check_command_age=true`) | DROP + diagnostics WARN |
| `msg.source` ≠ mức của topic đang phát | **ĐÓNG DẤU LẠI** (`source` = mức của topic) + ERROR. KHÔNG drop |

Trọng tài **không bao giờ sửa nội dung hình học** của lệnh (không kẹp/mượt/nội suy) — chỉ cho qua /
chặn / đóng dấu provenance.

---

## 4. Latch (`SetControlAuthority`) — dựng SÀN, không dựng TRẦN

Publish lên kênh nguồn là giành quyền **ngầm** (đường chạy thật). Service `SetControlAuthority` đặt
một **sàn ưu tiên**: không nguồn nào thấp hơn mức latch được chọn, kể cả khi bên giữ latch im lặng —
**nhưng chỉ SAU KHI bên giữ đã từng publish ít nhất một lần** (`latchEffective()`, Y4 chốt 2026-08-20),
**TRỪ MỘT NGOẠI LỆ: mức SAFETY** (P8 R2, chốt 2026-08-20 — xem dưới). Một latch OPERATOR/MISSION/TEST
vừa cấp mà chưa ai phát gì thì **hoàn toàn không có hiệu lực** — không cắt bất kỳ nguồn nào đang bay.
Đây là sửa trực tiếp cho chế độ hỏng "operator giành quyền rồi cầm nhầm cần" từng khiến mission bị
khoá và command_selected im re dù mission đang bay tốt. SAFETY luôn thắng latch OPERATOR bằng publish
— latch không chặn được nguồn cao hơn nó.

🔴 **Ngoại lệ SAFETY (P8 R2 + R5, INHIBIT):** một latch SAFETY hiệu lực **NGAY LÚC CẤP**, không cần
publish gì, và **KHÔNG bao giờ tự hết hạn** — không GRACE-expiry, và (R5, chốt 2026-08-21 sau phát
hiện thật ở G-S1 #8) cũng **không TIMEOUT-expiry** dù đã từng sống. Nó persist vô hạn cho tới khi có ai
gọi `clear_safety_latch`, PX4 tự failsafe, hoặc RC (P11) giành lại. Lý do: policy-1 của `uav_safety`
(P8) cấm SAFETY phát setpoint trên một pose đã hỏng, nên nó **không có cách nào "chứng minh đang lái"**
như 3 mức kia — bắt nó chứng minh sẽ làm chế độ INHIBIT (im lặng có chủ đích) bất khả thi. R5 mở rộng
lý do đó sang cả sau khi đã sống: HOLD (P8.5) là cơ chế khiến SAFETY thực sự publish rồi **chủ động**
ngừng khi escalate sang INHIBIT — im lặng có chủ đích đó không được phép bị đọc nhầm thành "bên giữ đã
chết". `AuthorityArbiter::inhibitActive()` = latch SAFETY hiệu lực mà `latch_ever_alive_` còn `false`
(vẫn đúng — R5 không đổi định nghĩa INHIBIT, chỉ đổi việc latch có tự hết hạn hay không).
⚠️ **Giám sát bù:** vì SAFETY không tự hết hạn, `latch_age_sec` (Y15 diagnostics) tăng vô hạn trên
`latch_level=SAFETY` là tín hiệu phát hiện "safety node đã chết mà vẫn giữ latch" — không phải lỗi
thiếu cơ chế, đó chính là điều `clear_safety_latch`/PX4 failsafe/RC tồn tại để xử lý. 🔴 **C4 (P10.8b):**
nếu đồng hồ LÙI trong lúc latch đang giữ, `latch_age_sec` đọc **NaN** ("không đo được", R30) thay vì
số ÂM — người/công cụ giám sát phải coi **cả** "tăng vô hạn" LẪN "NaN" là đáng điều tra, không được đọc
NaN thành "rất mới/khoẻ mạnh" (§2 mục 6).

**Ba đường thu hồi cho OPERATOR/MISSION/TEST** (thiếu một đường là latch trở thành khoá chết máy bay;
SAFETY chỉ có đường (1), qua `clear_safety_latch`, không phải (2)/(3) ở trên):

1. Nhả tường minh (`requested_source = SOURCE_NONE`) — **CẤM khi latch đang giữ ở mức SAFETY** (Y6
   chốt, không đổi bởi P8): chỉ service **`clear_safety_latch`** (kiểu `ClearFault`, tái dùng, P8) mới
   gỡ được latch SAFETY — `AuthorityArbiter::clearSafetyLatch()`, đường DUY NHẤT. Mọi yêu cầu nhả qua
   `SetControlAuthority` (thành công hay bị từ chối) đều log ERROR + đếm trong diagnostics.
2. Latch cấp mà bên giữ **không publish** trong `latch_grace_sec` ⇒ tự huỷ (không ảnh hưởng gì vì
   Y4: latch chưa từng hiệu lực). **KHÔNG áp dụng cho SAFETY** (đó chính là INHIBIT có chủ đích).
3. Bên giữ đã sống rồi **im quá `latch_timeout_sec`** ⇒ tự huỷ — cho tới lúc đó, sàn latch vẫn
   tiếp tục loại các nguồn thấp hơn dù chính bên giữ cũng đang im (khác hẳn ưu tiên thường). **KHÔNG
   áp dụng cho SAFETY** (R5) — chỉ OPERATOR/MISSION/TEST.

`clear_safety_latch` (`/uav/<id>/control/clear_safety_latch`) nằm trong `io_group_` như mọi entry
point khác chạm `arbiter_` (R24, không ngoại lệ). Người gọi hợp lệ duy nhất là `uav_safety`, SAU KHI
`ClearFault` của chính nó đã chấp nhận — trọng tài **không tự phán** điều kiện gỡ, chỉ thi hành; mọi
lần gọi log ERROR kèm `fault_code` + kết quả.

**Kênh toàn rác (Y5, sửa lại theo B2 2026-08-20):** kênh **đang giữ quyền** mà đổ rác liên tục (NaN/
frame/mode/stale) không còn cần một bộ đếm riêng (`demote_after_bad_ticks` đã bị GỠ) — vì rác không
bao giờ cập nhật `last_valid_arrival_`, nó tự động rơi ra sau `release_dwell_sec` (0,20 s) giống hệt
mọi trường hợp im lặng khác. Bảo chứng Y5 ("kênh toàn rác không giữ được quyền") **giữ nguyên**, chỉ
đổi từ một cơ chế đếm riêng sang hệ quả cấu trúc của B2 — ít code hơn, ít chỗ để lệch pha hơn.

---

## 5. Tham số (`config/control_authority_params.yaml`)

Mọi tham số được validate lúc khởi động — sai thì node **từ chối chạy** (throw `std::invalid_argument`,
log `RCLCPP_FATAL` trước khi crash). Bốn quy tắc ghép cặp/ràng buộc chốt sau review 2026-08-20 (xem
comment trong file yaml):
- `source_timeout_sec <= 0.6 * downstream_command_timeout_sec`
- `release_dwell_sec >= source_timeout_sec` (Y3 — chặn NONE↔kênh vừa nhả rung ở biên chung)
- `release_dwell_sec + 2*kMonitorPeriodSec + 1/kAssumedSourceStreamHz <= 0.8 * downstream_command_timeout_sec` (Y1)
- `check_command_age` phải là boolean riêng — không được lấy `max_command_age_sec = 0` làm công tắc tắt

Timing mặc định đã **hạ từ 0,30/0,30 xuống 0,20/0,20 s** (`source_timeout_sec`/`release_dwell_sec`) để
vẫn lọt qua ràng buộc Y1 mới với `downstream_command_timeout_sec` giữ nguyên 0,50 s (giá trị thật của
gateway) — không nới gateway để khớp, siết trọng tài để khớp gateway.

---

## 6. Test

| File | Chạy ở đâu | Nội dung |
|---|---|---|
| `test/test_authority_arbiter.cpp` | Không cần domain (ROS-free), **77 case** | Bảng chân trị ưu tiên 5×5, hysteresis 2 chiều, 3 đường hết hạn latch (+ Y4: latch chưa hiệu lực không cắt gì), Y6 latch SAFETY chặn release, biên thời gian, bộ lọc nội dung (3 mode × NaN + lặp trên SAFETY), tham số sai ⇒ từ chối (13 case, gồm 2 ràng buộc Y1/Y3), **B2** — 3 test `GarbageChannel.*` ghim đúng 3 mặt review nêu (rác ưu tiên cao không cướp được quyền · rác từ holder latch không nuôi latch clock · nhịp 1-hợp-lệ:9-rác mất quyền trong ≤ dwell + 1 tick), **N-c (P10.8a)** — 4 test `ClockRewind.*` (kênh rơi NONE khi lùi, latch SAFETY bất động, bộ đếm không bị nuốt, hành vi thời gian tiến không đổi), **C4 (P10.8b)** — 5 test `ClockRewind.*` thêm (guard tuổi nội dung DROP-STALE khi lùi + đối chứng dương lệnh tươi vẫn qua, `latchAgeSec()` NaN khi lùi + đối chứng dương giá trị đúng khi tiến + không-latch luôn 0) |
| `test/test_control_authority_manager_node.cpp` | `ROS_DOMAIN_ID=96` (bịa `ControlCommand`, R20), **13 case** | 8 mục G-CA1 gốc + Y1 chiều xuống (dwell handover) + N-b (im lặng toàn phần ⇒ 0 msg, đối chứng dương) + P8.2/checkpoint B1 |

Đo được (P7.1b/c, WSL, chưa phải cổng bay G-CA2 — cả số đều đo trên message thật ra dây, không phải
biến nội bộ):

| Số đo | Giá trị | Ngân sách |
|---|---|---|
| Δt chuyển quyền SAFETY (chiều lên, ưu tiên chiếm ngay) | **0,14 ms** | ≤ 100 ms |
| Khoảng phát lớn nhất qua handover chiều lên | **50 ms** | ≤ 150 ms |
| Khoảng im qua handover chiều xuống (dwell, Y1) | **201 ms** | < 500 ms (cutoff gateway) |
| Phát hiện publisher thứ hai trên `command_selected` (Y2, đã hạ xuống 1 Hz) | **1,004 s** | ~1 diagnostics period, không còn `<1s` như trước Y2 |

Mutation `release_dwell_sec → 0` (P7.1b: sửa tham số trực tiếp; P7.1c re-run sau B2: sửa trực tiếp
điều kiện `silent_past_dwell` trong `onTick()`, không qua tham số vì Y3 đã chặn cấu hình đó ở
`validate()`) đã chạy tay, làm đỏ đúng 3 test `Hysteresis.*`, rồi revert — không commit bản mutation.
