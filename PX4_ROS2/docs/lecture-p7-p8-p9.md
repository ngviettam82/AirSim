# BUỔI GIẢNG R18 — P7 (trọng tài quyền điều khiển) · P8 (an toàn) · P9 (mission BT)

> **Ngày:** 2026-08-23 · **Người học:** chủ dự án · **Mục tiêu:** không phải "biết nó chạy", mà **tự làm lại được và tự chẩn đoán được**.
> **Nguồn:** mọi con số đều trích từ file trong repo — có ghi địa chỉ. Chỗ nào người soạn **chưa chắc**, ghi rõ là chưa chắc.
> **R0:** ba phase này là **toàn bộ đường cứu mạng** của hệ — ai được lái, khi nào cắt lái, ai ra lệnh. Sai ở đây không dừng ở "bay xấu", nó dừng ở "không ai lái mà máy bay vẫn treo". §6 giảng sâu hơn các mục khác **có chủ đích**.
>
> *Soạn bởi `project-mentor` (P9.10). Quy trình R18: đọc §0, tự trả lời, giữ câu trả lời lại; mỗi mục có ô **🎓 Tự kiểm** — nói cái anh đang hiểu trước, rồi mới đọc phần giải; cuối buổi làm §10.*

---

## 0. TRƯỚC KHI ĐỌC — anh tự trả lời 4 câu này đã

Đừng tra tài liệu. Trả lời bằng trực giác hiện tại. Bốn câu này là bốn chỗ mà **trực giác đúng-nghe của kỹ sư lại cho ra thiết kế giết người**, và cả bốn đều đã trả giá thật:

| # | Câu hỏi | Vì sao hỏi |
|---|---|---|
| **A** | Trọng tài quyền điều khiển thấy **mọi nguồn lệnh đều chết**. Nó nên (a) tự phát lệnh hover giữ máy bay đứng yên, hay (b) **im lặng hoàn toàn**? | §2a |
| **B** | Drone **mất định vị** giữa trời. Phản xạ đầu tiên của anh là gì — hạ cánh tự động? bay về nhà? Cái nào an toàn? | §3b |
| **C** | Safety đã chiếm quyền, mission bị treo. Sau khi `ClearFault`, **goal bay dở của navigator có tự chảy lại không**? Nếu có thì ai chịu trách nhiệm? | §4c, §6b |
| **D** | Mission bay 2 điểm tìm kiếm xong, cả hai điểm đều `SUCCESS`. Có được kết luận **"đã tìm thấy mục tiêu"** không? | §5c |

Nếu câu A anh trả lời "hover cho chắc" — đây đúng là buổi giảng anh cần. Hover tự sinh là **chế độ hỏng tệ nhất có thể thiết kế ra**, và tài liệu dự án gọi đúng bằng tên đó (`P7-control-authority.md` §1).

---

## 1. BỨC TRANH 3 TẦNG — MỘT MẠCH

### 1.1 Đường lệnh thật (tên topic thật, không rút gọn)

```
┌──── P9 · uav_mission ────────────────────────────────────────────┐
│ mission_executor_node   BT tick 10 Hz (đồng hồ ROS, KHÔNG event) │
│   ├ mission_policy   (ROS-free)  bảng 5 ưu tiên → GuardVerdict   │
│   ├ mission_registry (ROS-free)  3 XML + validate mission_params │
│   └ nav_goal_broker  (ROS-free)  FSM: TỐI ĐA 1 GOAL, MÃI MÃI     │
│  ⛔ KHÔNG publish control/* · KHÔNG clear_fault · KHÔNG arm      │
│  ⛔ KHÔNG SetControlAuthority                                    │
└───────────────┬──────────────────────────────────────────────────┘
                │ 7 action client (Takeoff/Land/Goto/Hold/
                │ FollowPath/TrackTarget/Recover)
                ▼
┌──── P6 · uav_navigation ─────────────────────────────────────────┐
│ navigator_action_server_node  ← publisher DUY NHẤT của cmd_mission│
└───────────────┬──────────────────────────────────────────────────┘
                │ /uav/uav0/control/cmd_mission        (prio 2)
                │
  /control/cmd_safety   (prio 4) ◄── P8 safety, CHỈ khi HOLDING
  /control/cmd_operator (prio 3) ◄── teleop/GCS — P11, chưa có publisher
  /control/cmd_test     (prio 1) ◄── 5 probe cổng bay
                │
                ▼
┌──── P7 · uav_control_authority ──────────────────────────────────┐
│ control_authority_manager_node   RƠ-LE THUẦN — không sáng tác gì │
│   authority_arbiter (ROS-free): priority · hysteresis · latch    │
│   bộ lọc: NaN · sai frame · mode lạ · tuổi   ⇒ DROP (cả SAFETY)  │
│   ra: /control/command_selected  🔴 SINGLE WRITER VĨNH VIỄN      │
│       /control/authority   Reliable/KeepLast(1)/TransientLocal   │
│                            phát khi đổi + nhịp tim 2 Hz          │
│       /diagnostics/control_authority  1 Hz + 6 key latch (Y15)   │
└───────────────┬──────────────────────────────────────────────────┘
                │
                ▼
        px4_command_gateway_node   timer 20 Hz, tự phát lại
        `latest_command_` chừng nào còn tươi 0,5 s; hết → NGỪNG
                │ /fmu/in/*
                ▼
              PX4  →  failsafe lõi + trả quyền pilot

┌──── P8 · uav_safety ─────────────────────────────────────────────┐
│ safety_supervisor_node  + failsafe_policy (ROS-free, 12 mã)      │
│  ĐỌC: localization_status · odometry_fused · localization_health │
│       obstacle_map_local · health_px4 · vehicle · offboard_status│
│       command_selected(!) · control/authority · diagnostics arb. │
│  LÀM: REPORT · HOLD (cmd_safety 20 Hz pose đóng băng) ·          │
│       INHIBIT (latch SAFETY, KHÔNG publish gì)                   │
│  ⛔ KHÔNG set_mode/arm/disarm/action · KHÔNG land/RTL/climb      │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 Ba tầng — mỗi tầng NGHI ai, TIN ai

Đây là phần đắt nhất của §1. Kiến trúc an toàn không phải là "nhiều lớp kiểm tra", mà là **một cây tin cậy có hướng**:

| Tầng | TIN cái gì | NGHI cái gì | Bằng chứng nó dùng để phán |
|---|---|---|---|
| **P7 trọng tài** | TIN **topic** (kênh nào = quyền nào). Không tin **nhãn** `msg.source` — sai nhãn thì đóng dấu lại, không drop | NGHI **nội dung** mọi nguồn, kể cả SAFETY: NaN / sai frame / mode lạ / quá tuổi ⇒ DROP | Chỉ **thời điểm đến của message ĐÃ QUA bộ lọc** (`last_valid_arrival_`) |
| **P8 safety** | TIN `odometry_fused` là **nguồn pose duy nhất** (R4). TIN PX4 failsafe lõi làm phần cứu cuối | NGHI **mọi tầng trên nó** — kể cả navigator: nó **đo trên dây** `command_selected` xem setpoint có đang **di chuyển trên pose hỏng** không | Đại lượng **đã phát ra dây**, không phải cờ nội bộ của ai |
| **P9 mission** | TIN `/control/authority` (ai cầm lái) và `VehicleHealth` (pin) | NGHI **chính mình**: guard nằm trong **code C++**, thân mission nằm trong XML — file text không gỡ được nhánh an toàn | `GuardVerdict` từ bảng quyết định thuần hàm, tick 10 Hz theo **đồng hồ**, không theo lưu lượng |

**Vì sao xếp đúng thứ tự này** — ba lý do, mỗi lý do là một phương án đã loại:

1. **Trọng tài ở DƯỚI safety, không ở trên.** Nếu safety đứng dưới cùng (ngay trước gateway), nó thành nguồn lệnh thứ 5 không ai trọng tài được. Đặt nó thành **một kênh có ưu tiên cao nhất** khiến chính luật ưu tiên đã ghi trong `ControlAuthority` (§2.2 contract) là thứ thi hành nó — không phải một `if` đặc biệt.
2. **Mission ở TRÊN CÙNG và không có tay.** `uav_mission` có **4 điều cấm** grep được (`src/uav_mission/README.md`): không publish `control/*`, không gọi `clear_fault`/`clear_safety_latch`, không arm/disarm/set_mode, không `SetControlAuthority`. Nghĩa là **tầng cao nhất là tầng ít quyền nhất** — nó chỉ nhờ navigator bay hộ. Đó là chủ ý: một BT sai logic không được phép trở thành một cú lao.
3. **Không tầng nào tự "chữa" cho tầng dưới.** Safety không cancel goal của navigator; mission không gỡ latch của safety. Mỗi tầng chỉ **làm ít đi** khi nghi ngờ. Chiều an toàn của cả hệ là *"làm ít đi"*, không phải *"cứu chữa"*.

### 1.3 Đường CẮT — đọc kỹ, nó không đi qua chỗ anh nghĩ

```
safety quyết INHIBIT
   │
   │  ⛔ KHÔNG publish cmd_safety      ⛔ KHÔNG gọi set_mode
   ▼
SetControlAuthority(SOURCE_SAFETY)  →  arbiter dựng SÀN ưu tiên
   │
   ▼
mọi kênh < SAFETY bị loại khỏi ứng viên   (cmd_mission có tươi cũng vô ích)
   │
   ▼
command_selected IM LẶNG  (không ai đủ tư cách, safety cũng không phát)
   │
   ▼
gateway đệm nốt command_timeout_sec 0,5 s  rồi NGỪNG /fmu/in/*
   │
   ▼
PX4 mất offboard → failsafe lõi → trả quyền pilot
```

🔑 **Đường cắt là một chuỗi IM LẶNG có kiểm soát, không phải một chuỗi LỆNH.** Đây là điều khó chấp nhận nhất về mặt trực giác trong cả ba phase, và §2a + §3b sẽ giải thích vì sao mọi phương án "gửi một lệnh gì đó" đều bị loại ở mức R0.

> **🎓 Tự kiểm §1:** Kể tên **ba** thứ mà `uav_mission` bị cấm làm, và với mỗi thứ nói xem nếu cho phép thì chế độ hỏng là gì.

---

## 2. P7 — BỐN BÀI TOÁN THEN CHỐT CỦA TRỌNG TÀI

Mỗi bài: **vấn đề → phương án đã LOẠI và vì sao → phương án chọn → con số chứng minh**. Phần "đã loại" mới là phần đắt.

---

### 2a. Rơ-le thuần — vì sao trọng tài KHÔNG BAO GIỜ tự phát setpoint (Q3, chữ ký R0)

#### Phát hiện lật ngược giả định ban đầu

Trước khi viết P7, giả định tự nhiên là: *"`offboard_session_manager_node` giữ nhịp ≥2 Hz cho PX4, nên chèn một node vào giữa sẽ làm đứt nhịp đó."* Đọc code thì sự thật khác (`P7-control-authority.md` §1):

| Sự thật đo được từ code | Dẫn chứng |
|---|---|
| `offboard_session_manager_node` **KHÔNG phát một setpoint nào**. Nó chỉ **đếm** `/fmu/in/offboard_control_mode` và xin đổi mode | `offboard_session_manager_node.cpp:62-67` — ghi chú nguyên văn: *"Counting the real stream beats trusting any status message"* |
| Nguồn proof-of-life THẬT là **timer 20 Hz của gateway**, tự phát lại từ `latest_command_` chừng nào lệnh còn tươi trong `command_timeout_sec` = **0,5 s** | `px4_command_gateway_node.cpp:123-125, :132-147` |
| Im lặng là **cố ý**, đã có ghi chú từ P2 | `px4_command_gateway_node.cpp:131` — *"Silence is deliberate: PX4 failsafe beats a stale target"* |

⇒ Gateway đã đệm **0,5 s ≈ 10 tick @20 Hz**. Một lần chuyển quyền tốn **≤1 tick** ⇒ **dư 10 lần**. Không cần ai giữ nhịp hộ.

#### Cây quyết định

| Phương án | Kết luận |
|---|---|
| ❌ **Trọng tài tự phát hover khi mọi nguồn im** | 🔴 Nó **giữ offboard sống khi KHÔNG AI lái**. Máy bay treo giữa trời, PX4 **không** failsafe vì vẫn thấy nhịp. Và ai tắt cái hover đó? Không ai — vì chính nó là nguồn duy nhất còn phát |
| ❌ **Trọng tài làm mượt / nội suy / kẹp lệnh** | Làm mượt một lệnh SAFETY = **giao lệnh cứu mạng trễ**. Và nó biến trọng tài thành **nguồn lệnh thứ 5 không có bậc ưu tiên** — nằm ngoài chính luật R6 mà nó thi hành |
| ❌ **Resample bằng timer** (nhận nguồn thắng, phát lại theo nhịp riêng) | Cộng tới 50 ms vào vòng vị trí. Ngân sách dây xích `max_lead` 0,80 m được tune **trên đường trực tiếp** — thêm trễ là ăn vào lá chắn đó |
| ✅ **Pass-through theo sự kiện** | Nhận msg nguồn thắng → validate → publish ngay. **0 ms trễ thêm ngoài 1 hop DDS** |

#### Con số trả lời câu hỏi "một hop DDS có đắt không"

Đây là cổng hồi quy **G-CA3** — chỗ dễ bị bỏ sót nhất, vì nó không kiểm P7, nó kiểm **thứ P7 vô tình làm hỏng**:

| Đo | Trước P7 (navigator phát thẳng) | Sau P7 (qua trọng tài) | Trần |
|---|---|---|---|
| G-N2 lead đỉnh | 0,563 m | **0,516 m** | 0,65 m |
| Δt chuyển quyền (G-CA1 #2) | — | **0,14 ms** | ≤ 100 ms |
| Khoảng phát lớn nhất chiều LÊN | — | **0,050 s** | ≤ 0,15 s |
| Khoảng im chiều NHẢ | — | **0,201 s** | < 0,50 s (đệm gateway) |

> Lead sau P7 **tốt hơn** mốc trước P7. Không phải vì hop DDS âm — mà vì nhiễu đo giữa hai lượt bay lớn hơn chi phí hop. **Kết luận đúng là "hop không đo được", không phải "hop giúp bay tốt hơn"** — đừng đọc bảng này theo chiều thứ hai.

> **🎓 Tự kiểm §2a:** Nếu ngày mai có người sửa trọng tài cho nó publish một lệnh hover khi mọi nguồn chết — hãy viết ra chuỗi sự kiện dẫn tới tai nạn, từng bước một.

---

### 2b. Một topic = một mức ưu tiên — vì sao không dùng trường `source` trong message

| Phương án | Vấn đề |
|---|---|
| ❌ Một topic chung, phân quyền theo `msg.source` | Ai cũng tự khai được `source = SAFETY`. Quyền trở thành thứ **tự nhận**, không phải thứ **được cấp** |
| ✅ **4 topic, mỗi topic một mức** | Arbitration là **thứ tự toàn phần** ⇒ không cần luật gỡ hoà. Quyền do **topic** quyết |
| — | `msg.source` sai ⇒ **đóng dấu lại + ERROR + đếm**, **KHÔNG drop**: kênh cứu mạng không được im chỉ vì một nhãn sai (contract §2.16d) |

**Hysteresis bất đối xứng** — điểm sát an toàn của mục này:

| Chiều | Luật | Vì sao |
|---|---|---|
| **LÊN** | chiếm quyền **ngay ở message đầu tiên**, không chờ, không dwell | Làm chậm SAFETY dù một tick là phản bội thứ tự ưu tiên. Không ngoại lệ |
| **XUỐNG** | chỉ khi nguồn đang giữ im lặng liên tục `release_dwell_sec` = **0,20 s** | Chống giật khi một stream rung nhẹ |

Và một chi tiết dễ viết sai: **liveness đo bằng THỜI ĐIỂM ĐẾN, không bằng `header.stamp`** — nguồn lệch đồng hồ không bị tuyên bố chết. `header.stamp` chỉ dùng cho phép kiểm tuổi **nội dung**, là một kiểm riêng.

#### 🪤 Bất đẳng thức Y1 — hai hằng số ở hai package mã hoá cùng một ngưỡng

`release_dwell + 2×chu_kỳ_giám_sát + 1/stream_hz ≤ 0,8 × downstream_command_timeout`

Với bộ số **đề xuất ban đầu** (dwell 0,30 · monitor 10 Hz): `0,30 + 0,20 + 0,05 = 0,55 > 0,40` ⇒ **VÔ NGHIỆM**. Phải đổi: monitor **10→20 Hz**, dwell **0,30→0,20** ⇒ `0,20 + 0,10 + 0,05 = 0,35 ≤ 0,40` ✓.

🔑 `downstream_command_timeout_sec` là **bản sao có tên** của `command_timeout_sec` bên gateway — trọng tài **không đọc được** tham số thật của package khác. Nên yaml phải ghi quy tắc ghép cặp và WARN khởi động in **cả hai số**. Đây là lần thứ **năm** dự án gặp khuôn lỗi *"hai hằng số độc lập mã hoá hai giả định mâu thuẫn"* (G6).

> **🎓 Tự kiểm §2b:** Vì sao chiều LÊN không có dwell còn chiều XUỐNG có? Nếu đặt `release_dwell_sec = 0` thì test nào phải đỏ?

---

### 2c. Latch — dựng SÀN, không dựng TRẦN, và ba đường thu hồi

Đã có "giành quyền ngầm bằng publish". Nếu service `SetControlAuthority` làm lại việc đó thì có **hai nguồn sự thật** về quyền. Nên nó được cho một vai khác hẳn:

| Cơ chế | Vai |
|---|---|
| **Publish lên kênh nguồn** | Giành quyền **ngầm**, hiệu lực khi stream còn tươi. Đường chạy thật 20 Hz |
| **`SetControlAuthority`** | **LATCH** — đặt **SÀN ưu tiên**: không nguồn nào thấp hơn mức latch được chọn, **kể cả khi bên giữ latch im lặng** |

🔴 **Ba đường thu hồi bắt buộc** (thiếu một là latch trở thành khoá chết máy bay):

| # | Cơ chế | Chế độ hỏng nó chặn |
|---|---|---|
| 1 | Nhả tường minh (`SOURCE_NONE`) | Đường bình thường |
| 2 | Latch cấp mà bên giữ **chưa từng publish** quá `latch_grace_sec` 2,0 s ⇒ tự huỷ | Operator latch rồi cầm nhầm cần → mission khoá, `command_selected` im → PX4 failsafe giữa trời. **Chế độ hỏng do chính cơ chế an toàn đẻ ra** |
| 3 | Đã từng sống rồi im quá `latch_timeout_sec` 5,0 s ⇒ huỷ | Nguồn giữ latch chết giữa chừng |
| — | ❌ **Không có đường thứ 4** | Nguồn thấp hơn tuyệt đối không huỷ được latch của nguồn cao hơn — đó chính là R6 |

#### 🔴 Rồi P8 phá đúng đường 2 và 3 — và đó là ĐÚNG

`SAFETY` được **miễn trừ cả hai** (R2 của P8, rồi R5 siết thêm). Lý lẽ:

- Đường 2 đòi bên giữ **chứng minh mình đang lái**. Nhưng policy-1 của P8 nói SAFETY **không được phép phát setpoint trên pose hỏng**. Bắt nó chứng minh là **tự mâu thuẫn** — INHIBIT (im lặng có chủ đích) trở thành bất khả thi.
- Đường 3 bị **G-S1 #8 bắt tại trận**: HOLD là cơ chế đầu tiên khiến SAFETY *từng sống* rồi **chủ động im** (HOLDING→INHIBITED, đúng thiết kế) ⇒ trọng tài auto-revoke sau **5,0 s** ⇒ `command_selected` có thể quay lại chọn MISSION **trong khi safety vẫn tưởng mình đang giữ quyền**. Race này chỉ lộ ra khi có HOLD thật.

⇒ **R5:** latch SAFETY một khi đã cấp **không tự hết hạn qua bất kỳ nhánh nào**. Đường gỡ duy nhất: `clear_safety_latch` (+ PX4 failsafe / RC là đường thoát vật lý). Bù lại bằng giám sát: `latch_age_sec` (Y15) tăng vô hạn trên `latch_level=SAFETY` là tín hiệu "safety node chết mà vẫn giữ latch".

> **🎓 Tự kiểm §2c:** Latch dựng sàn chứ không dựng trần — nghĩa là gì về mặt hành vi? SAFETY có thắng được một latch OPERATOR không, bằng cách nào?

---

### 2d. Hai CHẶN mà chỉ REVIEW bắt được — không test nào đỏ

Đây là mục để anh hiệu chỉnh kỳ vọng vào test.

| CHẶN | Cơ chế | Vì sao test không bắt được |
|---|---|---|
| **B1 — data race** (review giữa phase) | Service `SetControlAuthority` được cho **callback group riêng** + `MultiThreadedExecutor` ⇒ `requestLatch()` ghi `active_source_`/latch state **song song** với sub/tick, không mutex/atomic. Kịch bản: SAFETY vừa giành quyền, service ghi đè bằng kết quả tính trên **snapshot cũ** ⇒ **SAFETY mất quyền ngay sau khi giành được** | Race không đỏ **ổn định**. Nó là loại lỗi mà "chạy 100 lần xanh" không chứng minh gì. → sinh **R24** |
| **B2 — rác nuôi liveness** (review đóng phase) | `authority_arbiter.cpp:213-217` ghi `has_arrived_`/`last_arrival_`/`latch_ever_alive_` **TRƯỚC** `validateContent()` ⇒ msg dị dạng vẫn là "bằng chứng sống". Ba hệ quả: kênh cao toàn rác **cướp quyền** rồi im 0,55 s > trần gateway 0,50 · holder latch phát rác ⇒ latch **khoá chết vĩnh viễn** · nhịp 1-hợp-lệ:9-rác giữ quyền vô hạn | Không cổng nào bắt được: `setpoint_rate_hz` là **trung bình cửa sổ**, `max_gap` chỉ là số tham chiếu. → sinh **R26** |

**Cách sửa B2 là bài học kiến trúc, không phải bài học vá lỗi:** thay vì thêm bộ đếm `demote_after_bad_ticks` (bản P7.1b đã có), lời giải đúng là **tách hai đồng hồ**: `last_arrival_` (mọi msg — chỉ để chẩn đoán) và `last_valid_arrival_` (chỉ msg qua `validateContent()`). Mọi luật liveness/latch/dwell/expiry **chỉ đọc cái thứ hai**. Bộ đếm demote lập tức thành **dead code** ⇒ gỡ.

> 🔑 **Bảo chứng bằng CẤU TRÚC mạnh hơn bảo chứng bằng BỘ ĐẾM.** Bộ đếm phải giữ đồng bộ với các luật khác bằng tay; cấu trúc thì không thể lệch. Đây chính là điều R26 khuyên: *"cách thoả tốt nhất là gộp các đường về một đường đã được canh"*.

> **🎓 Tự kiểm §2d:** Nếu ngày mai anh thấy `isLive()` đọc `last_arrival_` thay vì `last_valid_arrival_` — mô tả **ba** kịch bản tấn công mà nó mở lại.

---

## 3. P8 — NĂM BÀI TOÁN THEN CHỐT CỦA LỚP AN TOÀN

### 3a. Taxonomy đúng BA hành động — và vì sao không có hành động thứ tư

Bảng policy do chủ dự án ký 2026-08-21 (memory §5). Taxonomy là **TỐI THIỂU có chủ đích**:

| Hành động | Nội dung | Kèm latch? |
|---|---|---|
| `REPORT` | violation + event, không chạm quyền | Không |
| `HOLD` | Chiếm quyền, phát `cmd_safety` **pose đóng băng** 20 Hz, restamp mỗi tick, **không nội suy/mượt/trôi** | Có, tới `ClearFault` |
| `INHIBIT` | Latch SAFETY, **không phát gì** — im lặng có chủ đích | Có, tới `ClearFault` |

**Ba hành động ĐÃ LOẠI, ghi thẳng vào §10 "Cấm" của plan:** `land` · `RTL` · `climb`.

Vì sao? Ba lý do xếp theo độ nặng:

1. **Không nhân đôi PX4.** PX4 đã có battery failsafe, có RTL. Safety làm lại là hai bộ não cùng lái.
2. **Land/RTL/climb đều cần pose tin được.** Mà phần lớn ca safety phải hành động **chính là ca pose không tin được**.
3. **Nó thuộc tầng khác.** Quyết *khi nào* bay về và *bay thế nào* là việc mission (P9) điều phối qua navigator. Safety chỉ **làm ít đi**.

> **🎓 Tự kiểm §3a:** `HOLD` phát lệnh, `INHIBIT` không phát gì. Vậy cái nào "mạnh" hơn? Trả lời kèm lý do vật lý, đừng trả lời theo tên.

---

### 3b. 🔴 Mất định vị: KHÔNG auto-land, KHÔNG bay mù — bài toán sát sinh mạng nhất

#### Vấn đề

Drone mất định vị. Ai làm gì?

#### Ba phản xạ tự nhiên — cả ba đều bị loại

| Phản xạ | Vì sao LOẠI |
|---|---|
| ❌ **Auto-land** | 🔴 **Khi đã mất định vị thì phương tiện không biết mình đang ở đâu.** "Hạ cánh tự động" có thể hạ trúng người hoặc chướng ngại vật. `interface-contract-v0.1.md` §2.5 gọi đây là *"điểm sát sinh mạng nhất của hợp đồng này"*. Đây cũng là chỗ **mâu thuẫn với `CLAUDE.md` §4.8 bản cũ** — và mâu thuẫn đó **được ghi ra**, không bị code lặng lẽ giải quyết một chiều |
| ❌ **Safety phát lệnh "vô hại" (vận tốc 0) để nuôi liveness** | 🔴 **Loại ở mức R0.** Mọi publish đều giữ gateway sống ⇒ **ngăn đúng cái PX4 failsafe mà policy-1 muốn xảy ra** |
| ❌ **Safety gọi `set_mode`** để chuyển PX4 sang HOLD/LAND | Loại: đó là **đường ghi thứ hai xuống FC**, phá single-writer vừa xây xong ở P7 |

#### Phương án chọn: IM LẶNG có kiểm soát

```
pose hỏng + có ai đó vẫn đang lái trên pose đó
   ↓
safety latch SAFETY (không publish gì)
   ↓
command_selected im  →  gateway hết đệm 0,5 s  →  PX4 failsafe lõi + trả quyền pilot
```

#### Nhưng "ai đó vẫn đang lái" phải ĐO, không được đoán

Đây là chỗ tinh tế nhất. Phát hiện F1 (`P8-safety.md` §1): **navigator KHÔNG im lặng khi mất định vị** — `finishAirborne()` đóng băng setpoint rồi **phát tiếp mãi** (`hand_over=false`). Nghĩa là "có stream" **không** đồng nghĩa "đang bay mù".

⇒ Tiêu chí phân biệt, **đo trên dây** (R1 của coordinator):

| Quan sát trên `command_selected` | Phán quyết |
|---|---|
| Setpoint **DI CHUYỂN** > `frozen_setpoint_epsilon_m` **0,05 m** trong cửa sổ W = **3 tick** | 🔴 **BAY MÙ** ⇒ INHIBIT + latch |
| Setpoint **đứng yên** | Giữ đóng băng hợp lệ (đúng `Recover(TYPE_HOLD)` contract §2.5) ⇒ **violation nhưng KHÔNG cắt** |

`0,05` là **bản sao có tên** của `kFrozenSetpointDrift` mà navigator dùng để tự phán hold (`navigator_action_server_node.cpp:2852`) — cùng ngưỡng, hai package, ghi rõ là bản sao.

#### Ngân sách bay mù — suy từ số, không chọn cho đẹp

```
blind_grace_sec = 1,5 s
   phải > odometry_timeout(1,0) + 2/progress_hz(10 Hz) = 1,2 s   ← không cướp cò LỚP 1
quãng bay mù trần = max_speed 0,55 × 1,5 = 0,83 m
```

**Đo thật ở G-S2 (4/4 PASS, 3 biến thể tiêm lỗi + 1 đối chứng):** `blind_distance_m` = **0,000–0,003 m** trên ngân sách 0,83 m. `cmd_safety_count = 0` cả 4 lần.

#### 🔑 NHƯNG — đọc kết quả này cho đúng (R27-4)

`BLIND_COMMAND` **KHÔNG nổ** trong bay thật ở cả 3 biến thể. `blind_distance ≈ 0` đạt được nhờ **LỚP 1** — chính navigator tự đóng băng setpoint **trước khi** `blind_grace_sec` kịp trôi. Đúng thiết kế.

> 🔴 **Đây LÀ bằng chứng lớp phòng thủ 1 hoạt động đúng. Nó KHÔNG PHẢI bằng chứng đường cắt của `BLIND_COMMAND` hoạt động.** Đường đó chỉ được G-S1 #2/#3/#4 chứng minh riêng — bằng cách **ép setpoint di chuyển trực tiếp qua probe**, bỏ qua navigator.

Nếu đọc G-S2 thành *"BLIND_COMMAND đã verify bằng bay thật"* thì anh đang tin một lá chắn **chưa từng cắn**. Đây là đúng khuôn lỗi §2c của buổi giảng P5+P6 (*"guard xanh mà rỗng"*), lần này bắt được **trước** khi nó thành niềm tin sai.

> **🎓 Tự kiểm §3b:** Vì sao `blind_grace_sec` phải **lớn hơn** timeout của navigator chứ không phải nhỏ hơn? (Trực giác "safety phải nhanh nhất" sai ở đây — giải thích tại sao.)

---

### 3c. HOLD — đóng băng pose, và cái giá của việc "phát lệnh"

`OBSTACLE_TOO_CLOSE` là mã duy nhất dẫn tới HOLD (policy 3): vật cản < `min_obstacle_distance_m` **0,35 m** (= `costmap.drone_radius_m` — *vào tới đó là planner đã hỏng*) liên tục > 0,5 s, map tươi, **và định vị TỐT**.

| Ràng buộc | Vì sao |
|---|---|
| Pose lấy từ `odometry_fused` **duy nhất** (R4), tươi **≤ `pose_max_age_sec` 0,25 s** tại lúc đóng băng | Không đủ tươi ⇒ **đi thẳng INHIBIT**, không HOLD trên pose mờ |
| `HOLDING → INHIBITED` **MỘT CHIỀU** khi pose mất giữa chừng | Không có đường quay lại. Một khi đã tuyên bố "không tin pose nữa" thì không được đổi ý |
| **Restamp mỗi tick** @ `hold_stream_hz` 20 | Nếu không, chính bộ lọc `check_command_age` của trọng tài sẽ DROP lệnh cứu mạng của mình |
| Đọc `/world/obstacle_map_local` (10 Hz), **⛔ KHÔNG** `/perception/obstacles_local` (7 Hz) | Trường distance của topic thứ hai **đóng băng tại thời điểm phát hiện** = một số đo nói dối. Topic thứ nhất tính lại mỗi lần phát |

**Đo thật, G-S3-HOLD bay thật PASS 2 lượt liên tiếp:**

| Đo | Lượt 1 | Lượt 2 |
|---|---|---|
| Onset tiêm → HOLDING | 0,588 s | **0,560 s** |
| Stream `cmd_safety` | 19,99 Hz | **20,00 Hz** |
| **Pose trôi** | **0,000000 m** | 0,000000 m |
| Trôi cao độ (không hạ) | 0,025 m | 0,025 m |
| `ClearFault` | poll 7 lần / 3,3 s → success | y hệt |

🪤 **Hai phát hiện của cổng này, đều là bẫy cho người dùng:**

1. **`clear_fault` PHẢI POLL.** `clear_stability_sec` 3,0 s đo **QUA CÁC LẦN GỌI** (`failsafe_policy.cpp:722-728`), không qua đồng hồ nội bộ. Gọi **một lần** sau khi ngồi chờ 5 s vẫn bị từ chối với `stable_for=0.000`. Điều này nằm trong contract §2.18 vì nó **phản trực giác 100%**.
2. **Goal Goto gốc TỰ CHẢY LẠI sau ClearFault** — vì safety **không cancel** navigator (đúng điều cấm §10). Navigator giữ goal suốt thời gian mất quyền. Đây là dữ kiện sinh ra chính sách 3 của P9 (§4c).

> **🎓 Tự kiểm §3c:** Vì sao HOLD đọc `/world/obstacle_map_local` chứ không đọc `/perception/obstacles_local`? Trả lời bằng bản chất của trường `distance`, đừng trả lời bằng tần số.

---

### 3d. Đo-trước-phán — bốn ca số đo nói dối trong chính P8

Đây là mục để anh nhìn thấy **khuôn**, vì nó lặp lại 4 lần trong một phase:

| # | Đại lượng định dùng | Vì sao nó NÓI DỐI | Đại lượng thay thế |
|---|---|---|---|
| 1 | `header.stamp` của `/planning/trajectory` để đo "kế hoạch cũ" | 🔴 **Đồng hồ kế hoạch bị ĐÓNG BĂNG khi kẹp dây xích** (contract §2.14) — một kế hoạch đang treo trông vẫn "tươi" | Độ tươi **`command_selected`** + `offboard_status.command_fresh` |
| 2 | `in_air` (nợ #2) | Suy từ `takeoff_time > 0`, **sai sau khi hạ cánh** | Vị ngữ `armed && OFFBOARD && command_fresh` |
| 3 | `battery_remaining` mặc định **0,0** | "Chưa có dữ liệu" bị đọc thành **CRITICAL** ⇒ latch **trước khi arm** | **NaN** + arrival-age cho cả 4 sub trạng thái → sinh **R30** |
| 4 | Bộ đếm `dropped_wrong_frame` **gộp mọi kênh** | 1 msg sai frame trên kênh **phụ** có thể latch vĩnh viễn kênh **đang giữ quyền** | 4 key **per-channel**, safety chỉ xét kênh giữ quyền → sinh **R28** |

Và hai bug NaN **thật**, tự bắt ở P8.5:

- `std::min` trên `obstacle.distance` **nuốt NaN theo thứ tự mảng** ⇒ nay MỘT phần tử non-finite làm cả phép đo min-distance thành `CANNOT_MEASURE`. (Đúng khuyết tật #4 của lõi quỹ đạo P6 — `std::max` nuốt NaN. Cùng họ, lần thứ hai.)
- `isfinite()` đọc nhầm `+inf` (**map rỗng = "đã đo, sạch" = số đo HỢP LỆ**) thành `CANNOT_MEASURE` ⇒ `ClearFault` **không bao giờ chấp nhận** khi map thật sự rỗng. Sửa: dùng `isnan()`.

> 🔑 **`+inf` và `NaN` không cùng nghĩa.** `+inf` cho khoảng cách vật cản nghĩa là *"đã đo, không có gì cả"*; `NaN` nghĩa là *"không đo được"*. Gộp hai cái là tự khoá chính mình.

> **🎓 Tự kiểm §3d:** Cho một đại lượng an toàn bất kỳ trong hệ (tự chọn), hãy hỏi: "nó có nói dối khi cái nó mô tả bị đóng băng / chưa khởi tạo / gộp nhiều nguồn không?"

---

### 3e. Enforcement bằng CẤU TRÚC, không bằng `if` — và cổng V1

| Cơ chế | Bản chất |
|---|---|
| `enforcement_enabled = false` | Publisher `cmd_safety` chỉ được `create_publisher` **bên trong** `if (enforcement_enabled_)`. False ⇒ object đó **KHÔNG BAO GIỜ tồn tại** suốt vòng đời node |
| Chứng minh | `ros2 topic info .../control/cmd_safety --verbose` → **0 publisher**; `ros2 node info` → **0 Service Clients**. **Không phải đọc log, không phải tin một cờ** |
| **B5 — vị ngữ armed** | 4 mã LATCHING chỉ được LATCH khi `armed && flight_mode==OFFBOARD && command_fresh`. 🔴 **Fail-OPEN**: `vehicle_state` cũ/không đo được **KHÔNG** được coi là "đã xác nhận đậu bãi" để tắt bảo vệ — chỉ **xác nhận DƯƠNG TÍNH** disarmed mới tắt được. 8 mã REPORT-only **không** bị gate |

**Cổng V1 "boot-không-latch"** sinh ra để chứng minh B2 + B5 **trên dây**: 30 s đậu bãi disarmed sạch — 0 cạnh `BATTERY_*`, action luôn `none`, 0 latch SAFETY, `battery_remaining = 1.0` hữu hạn.

🔴 **Và nó có đối chứng dương** (R27-3): tiêm obstacle 0,18 m **khi đang đậu** ⇒ `OBSTACLE_TOO_CLOSE` **cắn đúng debounce 0,500 s** NHƯNG `action` vẫn `none` suốt. Nếu thiếu đối chứng này, "30 s sạch" chỉ chứng minh **không có gì chạy cả**.

🪤 Bẫy probe tự bắt trong chính cổng này: `safety/violations` là **edge-only, không TransientLocal** ⇒ không kiểm được kiểu "poll định kỳ" (R27-1) — phải trả nợ đo bằng chính **cạnh của đối chứng dương**.

> **🎓 Tự kiểm §3e:** "Fail-OPEN" ở B5 nghĩa là gì, và nếu làm ngược (fail-CLOSED) thì kịch bản tai nạn là gì?

---

## 4. P9 — NĂM BÀI TOÁN THEN CHỐT CỦA MISSION

### 4a. Guard nằm trong CODE, thân mission nằm trong XML (D-4)

| Phương án | Kết luận |
|---|---|
| ❌ **Toàn bộ cây, kể cả nhánh an toàn, nằm trong XML** | 🔴 Một **file text** gỡ được nhánh an toàn. Ai sửa XML cũng đổi được hành vi cứu mạng, không qua build, không qua review, không qua test |
| ❌ **Toàn bộ trong C++, không XML** | Mất tính linh hoạt của BT; mỗi mission mới là một lần biên dịch. Và không tận dụng được BehaviorTree.CPP |
| ✅ **`MissionGuard` là decorator C++ bọc SubTree nạp từ XML** | `MissionGuard` **không bao giờ là một tag trong XML**. Thân mission chỉ được dùng node trong **whitelist** (`mission_registry.cpp::allowedXmlTags()`); constructor của `MissionRegistry` validate whitelist của cả 3 XML shipped **ngay lúc khởi động** (fail-fast) |

**Tick 10 Hz theo ĐỒNG HỒ ROS, không event-driven** — cùng họ bài học R32: *guard chạy trên đồng hồ thì topic chết guard vẫn cắn; guard chạy trên lưu lượng thì nguồn chết là guard chết theo.*

#### Thực tế phức tạp hơn đọc nghĩa đen plan: guard 2 TẦNG

| Tầng | Ở đâu | Xét ưu tiên nào |
|---|---|---|
| **Top-level** | `onTick()`, **trước** khi tick cây BT | CHỈ **1–4** (operator abort · authority seize · battery · pause). Ưu tiên 5 không bao giờ cắn ở đây vì `last_nav_result_code` luôn `UNKNOWN` |
| **Trong leaf `NavAction`** | `onRunning()`, khi navigator vừa trả lỗi | **Cùng** hàm `MissionPolicy::evaluate()`, nhưng ưu tiên 1/3/4 **bị ép trơ**; ưu tiên 2 chỉ ghim `authority_seize_elapsed_sec = 0` — `authority_active_source`/`age` vẫn **THẬT**, để `still_holds_authority` của Q-P9-1 không nói dối |

Lý do tách: BT.CPP không có sẵn cơ chế *"retry một leaf theo backoff do một node khác quyết"*. Đặt retry/backoff/Q-P9-1 **đúng chỗ dữ kiện của nó** (kết quả navigator vừa trả) rẻ hơn nhân đôi logic.

> **🎓 Tự kiểm §4a:** Vì sao `evaluate()` là **hàm thuần** (không đọc đồng hồ)? Cái đó mua được gì cho test?

---

### 4b. `NavGoalBroker` — tối đa MỘT goal navigator, mãi mãi

FSM `IDLE / SENDING / ACTIVE / CANCELING / CANCEL_REFUSED` sau **một** mutex.

| Luật | Chế độ hỏng nó chặn |
|---|---|
| `request()` **chỉ** dispatch từ `IDLE`, không hàng đợi | Khớp navigator "REJECT ngay, không xếp hàng" — hai bên cùng một mô hình, không ai giả định sai |
| `request()` **bị chặn khi CANCELING** | Chống đua: gửi goal mới trong lúc cancel chưa xong ⇒ navigator REJECT vì `busy` ⇒ mission tưởng navigator hỏng |
| 🪤 **Bẫy Land**: navigator **TỪ CHỐI cancel khi đang LANDING** | Nếu broker chờ vô hạn một cancel không bao giờ được chấp nhận ⇒ **treo mission**. Nên có trạng thái `CANCEL_REFUSED` riêng, kết thúc theo **result thật** |
| `sendTypedGoal<ActionT>()` là **nơi DUY NHẤT** gọi `async_send_goal`/`async_cancel_goal` thật | "Tối đa một goal" là bất biến **node-rộng**, không chỉ trong một cây BT |

Broker là **ROS-free**: mọi lời gọi thật do caller tiêm vào qua `std::function` ⇒ test được FSM mà không cần ROS.

> **🎓 Tự kiểm §4b:** Nếu navigator không trả lời **cả** cancel lẫn goal gốc — mission làm gì? (Gợi ý: xem §8, nợ mở #5. Câu trả lời trung thực hiện tại không đẹp.)

---

### 4c. 🔴 Safety chiếm quyền ⇒ mission phải CANCEL — vì sao PAUSED không tự bay lại

#### Dữ kiện chốt hạ, đo thật ở G-S3-HOLD

> Sau `ClearFault`, **goal Goto gốc TỰ CHẢY LẠI** — vì safety không cancel navigator (đúng thiết kế), navigator giữ goal suốt thời gian mất quyền.

#### Vì sao đó là VẤN ĐỀ chứ không phải tiện lợi

```
safety chiếm quyền vì OBSTACLE_TOO_CLOSE
   ↓
mission chỉ ghi nhận "đang PAUSED", KHÔNG cancel goal
   ↓
người vận hành ClearFault (đã dời vật cản / đã xử lý)
   ↓
🔴 navigator TIẾP TỤC BAY chính cái goal đã dẫn nó tới vật cản
   — không ai bấm nút, không ai xác nhận, không ai đánh giá lại
```

#### Chính sách chủ dự án chốt 2026-08-22 (họp mở màn P9, mục 3)

> **Safety chiếm quyền ⇒ mission CANCEL goal navigator, vào PAUSED; CHỈ `ResumeMission` mới chạy tiếp.**

| Chi tiết | Giá trị |
|---|---|
| Trigger | `/control/authority` (TransientLocal, nhịp tim 2 Hz là **hằng hợp đồng**) |
| Vị ngữ | `mission_in_flight && (active_source != MISSION \|\| age > 2×heartbeat)`, giữ ≥ **1,5 s** |
| R32 | Message hết hạn ⇒ coi là **KHÔNG giữ quyền** → cancel + PAUSED. Chiều an toàn của mission là **LÀM ÍT ĐI** |
| `/safety/state` | **Chỉ** dùng làm lý do ghi event — không phải trigger. Một nguồn sự thật cho "ai cầm lái" |
| PAUSED quá `paused_timeout_sec` **180 s** không ai Resume | `ABORTED_TIMEOUT`, **giữ HOLD** (không tự hạ cánh) |

#### 🔴 Đối chứng dương bắt buộc của cổng G-M4.3

Cổng chạy **hai** lượt:
1. Lượt chính: safety seize → mission cancel → PAUSED → ClearFault do **probe** gọi → **vẫn PAUSED** tới khi `Resume`.
2. **Lượt đối chứng dương:** BỎ cancel ⇒ **phải thấy goal tự chảy lại**. Không nổ ⇒ **cổng FAIL**.

> Lượt 2 mới là thứ chứng minh chính sách này **cần thiết**. Không có nó, lượt 1 chỉ chứng minh "mission có gọi cancel", chứ không chứng minh "nếu không gọi thì có tai nạn".

Kết quả: **G-M4.3 6/6 PASS + đối chứng dương** — và lượt đối chứng chạy lại bằng chính `verify_obstacle_hold.sh` (tái dùng cổng P8, không viết cổng mới).

> **🎓 Tự kiểm §4c:** Vì sao mission dùng `/control/authority` làm trigger chứ không dùng `/safety/state`? Cả hai đều biết safety đang cắt mà.

---

### 4d. Bảng ưu tiên 5 nhánh — và vì sao battery đứng DƯỚI authority

`mission_policy::evaluate(MissionWorldView) → GuardVerdict`, **dừng ở điều kiện đầu tiên cắn**:

| # | Nhánh | Verdict | Ngưỡng thật |
|---|---|---|---|
| 1 | Operator abort | `ABORT_HOLD` (**không hạ cánh**, đúng comment `.srv`) | — |
| 2 | **Authority bị giành khỏi MISSION** | `PAUSE` | giữ ≥ `authority_loss_grace_sec` **1,5 s** |
| 3 | **Battery** WARN / unknown | `END_EARLY` = `GotoPose(home)` → `Land` | WARN **0,35** giữ **3,0 s** (R29) · NaN quá **30 s** · min cất cánh **0,40** |
| 4 | Pause service / đang PAUSED quá hạn | `PAUSE` / `ABORT_HOLD` | `paused_timeout_sec` **180 s** |
| 5 | Lỗi thân (kết quả action navigator) | tuỳ mã | retry **2** lần, backoff 3 s |

🔑 **Vì sao battery (3) phải đứng DƯỚI authority (2):** *một lệnh `end_early` gửi khi không giữ quyền là một lệnh vào hư không.* Nếu đảo thứ tự, mission sẽ dispatch `GotoPose(home)` trong lúc safety đang latch — goal bị REJECT hoặc treo, và mission tưởng mình đang bay về nhà.

**Nhánh 5 chi tiết** (`src/uav_mission/README.md`):

| Mã navigator trả | Xử lý | Vì sao |
|---|---|---|
| `ABORTED_LOST_LOCALIZATION` | **ABORT_HOLD ngay, KHÔNG retry** | Policy P8 #1. Retry một goal trên pose hỏng là bay mù có chủ đích |
| `ABORTED_NO_AUTHORITY` / `REJECTED` | `PAUSE` như nhánh 2 | Cùng nguyên nhân gốc |
| `ABORTED_SAFETY` | **Không bao giờ retry mù** | — |
| Lỗi khác | Retry tới `max_step_retries` 2, **trừ khi lý do LẶP LẠI** | Retry một lỗi cho ra **cùng một message** lần thứ 3 là vòng lặp, không phải khôi phục |
| Hết ngân sách | Q-P9-1: **chỉ** `ABORT_LAND` khi CÒN giữ quyền MISSION **+** localization hợp lệ **+** lý do không liên quan định vị; ngược lại `ABORT_HOLD` | Ba điều kiện, chủ dự án ký |

🪤 Retry reset ở **RANH GIỚI BƯỚC**, không reset mỗi lần `onStart()` — bài học từ `beginTask`.

> **🎓 Tự kiểm §4d:** `ABORTED_LOW_BATTERY = 12` là mã riêng. Vì sao **cấm** mượn `ABORTED_SAFETY` cho lý do pin?

---

### 4e. Ba bẫy kỹ thuật nặng đã trả giá ở P9.4 — hai trong số đó R0-relevant

| # | Bẫy | Cơ chế | Sửa |
|---|---|---|---|
| 1 | **apt `ros-humble-behaviortree-cpp-v3` có HAI CMake Config cạnh tranh cùng trỏ một `.so`** | ament tại `share/.../cmake/` (target `behaviortree_cpp_v3::behaviortree_cpp_v3`) và raw-CMake export tại `lib/cmake/.../` (target `BT::behaviortree_cpp_v3`). `find_package()` **chọn cái thứ hai trước** | Link `BT::behaviortree_cpp_v3`. Chẩn đoán bằng `cmake --debug-find` (CMake 3.22 **không có** `--debug-find-pkg=`) |
| 2 | 🔴 **Thứ tự huỷ thành viên C++ khi một thành viên gọi ngược `this` lúc huỷ** | `std::unique_ptr<BT::Tree> tree_` khai **TRƯỚC** `cancel_fn_mutex_`/action client ⇒ C++ huỷ **ngược** thứ tự khai báo ⇒ `~BT::Tree()` gọi `NavAction::onHalted()` chạm thành viên **ĐÃ HUỶ** ⇒ **SIGSEGV** (bắt bằng gdb) | Dời `bt_factory_`/`tree_` xuống **khai báo CUỐI class** (huỷ ĐẦU TIÊN). **Quy tắc chung: thành viên có destructor gọi ngược `this` phải khai báo SAU MỌI thành viên nó chạm tới** |
| 3 | 🔴 **Livelock khi guard tái trọng tài mỗi tick trong lúc "Finish" đang chạy** | Battery/authority là điều kiện **LIÊN TỤC ĐÚNG**; tái evaluate mỗi tick ⇒ **huỷ-dựng-lại `Finish` vô hạn**, không bao giờ chạm `Land` | `Finish` chạy tới cùng **KHÔNG tái trọng tài**, trừ `operator_abort_requested_` (tín hiệu **one-shot**, an toàn honor lại vì không lặp) |

🔴 **Và bản vá số 3 chính là thứ đẻ ra CHẶN B1 của review** — xem §5b. Đây là ví dụ sạch nhất trong dự án về *"bản vá cho một chế độ hỏng mở ra một chế độ hỏng khác"*.

> **🎓 Tự kiểm §4e:** Bẫy 3 phân biệt điều kiện **MỨC** với điều kiện **CẠNH**. Cho ví dụ mỗi loại trong hệ này, và nói loại nào được phép suppress trong `Finish`.

---

## 5. BỐN CA CHẨN ĐOÁN KINH ĐIỂN CỦA CHIẾN DỊCH P9

**Bối cảnh số:** chiến dịch bay P9.6→P9.9 kéo **2 ngày**, đóng **9/9 cổng + M5**, giá phải trả là **12 bug sản phẩm + 7 bug công cụ đo** (+1 regression = 13), mỗi cái một test ghim.

> 🔑 **Tỉ lệ 12 sản phẩm : 7 công cụ đo là con số đáng nhớ.** Ở P6 tỉ lệ là 0 sản phẩm : 7 công cụ đo. Nghĩa là: khi hệ **mới** và đường bay **dài**, bug sản phẩm chiếm đa số; khi hệ **đã ổn định**, bug phép đo chiếm đa số. Đừng mang kỳ vọng của phase này sang phase kia.

Phần này giảng **phương pháp**. Kết quả chỉ đúng cho một ca; phương pháp dùng lại được.

---

### 5a. Bám mục tiêu: đứng yên thì ổn, bay thì VỠ — ca ego-motion

#### Triệu chứng quan sát được

| Điều kiện | Quan sát |
|---|---|
| Drone **đậu**, mục tiêu di chuyển (lát cắt tĩnh P5.5) | **0 churn**, track ổn định |
| Drone **bay orbit** quanh mục tiêu (G-M3) | Track của **VẬT THẬT** (hộp còn nguyên) mất-tái-bắt **2–5 chu kỳ mỗi vòng** |

#### Chuỗi loại trừ — và bước đầu tiên là một GIẢ THUYẾT SAI có ích

| # | Giả thuyết | Việc đã làm | 🔪 Kết quả |
|---|---|---|---|
| 1 | **Nhiễu depth đẻ track ma** | Log `gm4d_bringup.log`: gỡ hộp khỏi world, nhiễu sót vẫn khiến `track_id` chạy **4→8→2→11→…→40, 18 lần chuyển** dù không còn vật thật. Vá bằng **M/N (3-của-5)**: track mới ở TENTATIVE, chỉ CONFIRMED sau 3 khớp trong 5 khung | ✅ Bug **thật**, sửa **đúng**. NHƯNG… |
| 2 | *"M/N sẽ dứt churn"* | Bay lại: **vẫn churn** trên vật thật | ❌ **Giả thuyết bị bác** — và đây là dữ kiện quý nhất: M/N là bản vá đúng cho **một** cơ chế, không phải cho cơ chế **chính** |
| 3 | Đối chiếu **tĩnh vs bay** | Tĩnh 0 churn · bay 2–5 chu kỳ/vòng | 🎯 Nghi phạm chuyển sang **thứ chỉ tồn tại khi bay: chuyển động của chính camera** |
| 4 | **Tính tay trước khi sửa code** | Camera orbit bán kính 2 m quanh vật đứng yên tại odom (5, 0, 1), 8 điểm quanh vòng, **cùng orientation** — độ dịch vị trí **biểu kiến** giữa hai điểm liên tiếp trong frame camera = **~1,53 m** | 🎯 **DỮ KIỆN CHỐT HẠ:** gấp **~3 lần** `min_association_gate_m` mặc định **0,5 m** |

#### Cơ chế đầy đủ

```
target_tracker_node liên kết (associate) quan sát TRONG FRAME CAMERA
   ↓
frame camera di chuyển theo drone
   ↓
drone xoay/tiến ⇒ vị trí BIỂU KIẾN của vật ĐỨNG YÊN nhảy 1,53 m
   ↓
vượt gate 0,5 m  ⇒  track chết + tái sinh id mới
   ↓
🔴 và vì track thật bị "chặt khúc", nó KHÔNG BAO GIỜ tích đủ M=3 lần khớp
   ⇒ M/N làm mọi thứ TỆ HƠN: liên kết trong frame camera cho 0 track được publish
      suốt cả vòng orbit (mọi track chết TENTATIVE)
```

#### Bản vá — và điều đáng học nằm ở chỗ nó KHÔNG chạm vào đâu

| Chạm | Không chạm |
|---|---|
| Node subscribe `/state/odometry_fused` với **đầy đủ pose** (không chỉ orientation) | 🔴 **`MultiTargetTracker` KHÔNG đổi một dòng nào** — đúng thiết kế "frame-neutral core" đã ghi ở đầu file từ P5.5 |
| Lib mới `ego_motion.hpp/.cpp` (ROS-free): `opticalPointToOdom()` áp cho MỌI quan sát **trước** `update()` | **Contract giữ nguyên**: `odomPointToOptical()`/`odomVectorToOptical()` chuyển ngược lúc publish ⇒ `frame_id` và mọi trường msg **không đổi** ⇒ `world_model` **không phải sửa** |

**Lợi ích ngoài dự kiến:** vì trạng thái nội bộ giờ ở odom, `vx/vy/vz` của track là vận tốc **THẬT** — vật đứng yên báo `velocity ≈ 0` **bất kể drone di chuyển thế nào**. Hạn chế cũ *"vận tốc chỉ tuyệt đối khi drone đứng yên"* được xoá luôn.

**Thoái lui 2 tầng (R32):** odometry tuổi ≤ `odometry_max_age_sec` 0,5 s + pose hữu hạn ⇒ liên kết trong odom; mất/cũ/hỏng ⇒ **quay lại đúng hành vi cũ** (frame camera) + WARN. Và mỗi lần đổi chế độ ⇒ **`tracker_.reset()` trước khi xử lý** — không bao giờ để một track trộn giá trị hai frame.

#### 🔑 Ba bài học phương pháp

| Bài học | Áp dụng chung |
|---|---|
| **"Tĩnh ổn / động vỡ" là một dấu vân tay chẩn đoán** | Nó **loại sạch** mọi nghi phạm không phụ thuộc chuyển động (nhiễu cảm biến, ngưỡng, mã hoá) và chỉ vào đúng lớp "đại lượng đo trong một frame đang chuyển động" |
| 🔴 **TÍNH TAY TRƯỚC KHI SỬA CODE** | 1,53 m vs gate 0,5 m là **một phép nhân với một phép chiếu ngược**, làm được trong 10 phút, và nó biến "tôi nghĩ là do ego-motion" thành "**gấp 3 lần gate, không thể không vỡ**". Một bản vá không có số đứng sau là một phỏng đoán đắt tiền |
| ⚠️ **Bản vá ĐÚNG cho cơ chế PHỤ có thể che cơ chế CHÍNH** | M/N đúng và cần, nhưng khi áp một mình nó làm **0 track được publish** — tệ hơn trước. Sau khi vá mà triệu chứng **đổi hình dạng** chứ không biến mất ⇒ còn cơ chế thứ hai |

> **🎓 Tự kiểm §5a:** Vì sao bản vá ego-motion **không được phép** sửa `MultiTargetTracker`? Nói bằng ngôn ngữ "trách nhiệm của lớp nào".

---

### 5b. PAUSE giả ngay lúc chuẩn bị hạ cánh — ca `kFinishLanding`

Đây là ca **quý nhất** của cả chiến dịch về mặt phương pháp, vì nghi phạm ban đầu **sai** và thứ bác nó là một dữ kiện rất nhỏ.

#### Bối cảnh: một bản vá đẻ ra một regression

Review lượt 1 của P9.10 ra CHẶN **B1**: khối chống-livelock trong `onTick()` (§4e bẫy 3) đang tắt **CẢ** ưu tiên #2 (authority-seize) lẫn #3 (battery) trong `kFinishGotoHome`/`kFinishLanding`/`kRecovering`. Lý lẽ CHẶN đúng:

> Lý lẽ livelock chỉ đúng cho điều kiện **MỨC** (battery ở dưới WARN mãi mãi ⇒ re-trigger restart `Finish` vô hạn), **KHÔNG đúng cho điều kiện CẠNH** (authority-seize là một **chuyển trạng thái**, không phải trạng thái đứng yên).

Sửa: cô lập **RIÊNG** ưu tiên 2, các field khác ép trơ. Hợp lý. Rồi bay hồi quy G-M1 — và **hỏng**.

#### Triệu chứng quan sát được

```
mission indoor_patrol chạy sạch ... tới lúc vào pha kFinishLanding
   ↓
PAUSE — không ai bấm gì, không có safety violation nào
   ↓
/control/authority:  active_source = NONE
                     age_sec = răng cưa SẠCH, chu kỳ 2 Hz
```

#### Chuỗi loại trừ

| # | Giả thuyết | Dữ kiện | 🔪 Kết quả |
|---|---|---|---|
| 1 | **Đói executor** — node quá tải nên message authority tới trễ, `age` vượt `2×heartbeat` | Đọc **29 dòng WARN** chẩn đoán trên dây: `age_sec` có dạng **RĂNG CƯA SẠCH đúng 2 Hz** | ❌ **BÁC.** Đói executor cho ra `age` **nhảy loạn/lệch nhịp**, không cho ra răng cưa đều tăm tắp. Răng cưa 2 Hz = **arbiter đang phát nhịp tim ĐÚNG GIỜ, và mission đang NHẬN ĐÚNG GIỜ** ⇒ cả hai node **KHOẺ** |
| 2 | Nếu cả hai khoẻ, thì `NONE` là **giá trị THẬT** — vậy ai lấy quyền? | Không ai. **Không ai stream `cmd_mission` nữa** | 🎯 Nghi phạm: vì sao navigator ngừng stream giữa lúc hạ cánh? |
| 3 | Đọc hợp đồng của pha LANDING | Navigator **trao quyền hạ cánh cho PX4 LAND native** | 🎯 **DỮ KIỆN CHỐT HẠ** (`gm1_diag1.log`) |

#### Cơ chế đầy đủ

```
mission vào kFinishLanding → gọi action Land
   ↓
navigator trao quyền cho PX4 LAND native (đúng thiết kế)
   ↓
không ai stream cmd_mission nữa
   ↓
arbiter báo active_source = NONE  ← ĐÚNG THIẾT KẾ, đây là trạng thái AN TOÀN dự kiến
   ↓
🔴 vị ngữ B1 `active_source != MISSION` KHÔNG PHÂN BIỆT
   "bị giành"  với  "không ai cần giữ"
   ↓
mission tự PAUSE chính cú hạ cánh của mình
```

#### Phán quyết — và nó là một phán quyết CÓ ĐIỀU KIỆN, không phải rollback

| Pha | Guard authority-seize | Vì sao |
|---|---|---|
| `kFinishLanding` | 🔴 **MIỄN** | `NONE` ở đây là an toàn ĐÚNG DỰ KIẾN. **Và** cancel-khi-LANDING vốn đã bị navigator **TỪ CHỐI theo hợp đồng** (`CANCEL_REFUSED`) ⇒ PAUSED lúc này chỉ tạo ra **trạng thái mồ côi không hành động được**, trong khi để Land hoàn tất + báo đúng kết quả gốc mới **thật sự làm được gì đó** |
| `kFinishGotoHome` · `kRecovering` | **GIỮ NGUYÊN** | `NONE` ở hai pha đó vẫn là bất thường **thật** (navigator chết). Chiều "làm ít đi" đúng ⇒ giữ |
| Mọi pha | `operator_abort_requested_` **vẫn được honor** | Không đổi |

Và một chi tiết vận hành: WARN chẩn đoán (thứ **đã chứng minh giá trị** — chính nó chốt được cơ chế) **HẠ CẤP còn 1 lần/đợt seize liên tục**, không spam mỗi tick.

🔴 **`pending_termination_` CỐ Ý không đụng tới** trong nhánh này — nó đang giữ **lý do `Finish` GỐC** (SUCCESS / END_EARLY / ABORT_LAND). Nếu tái dùng `beginCancelFor()` như pause từ `kBody` thì ghi đè bằng verdict pause ⇒ **mất lý do gốc vĩnh viễn**.

#### 🔑 Bài học trung tâm — và nó là lần thứ HAI

> 🔴 **VỊ NGỮ AN TOÀN PHẢI KÈM TIỀN ĐỀ PHA, KHÔNG CHỈ ĐỌC TÍN HIỆU THÔ.**

Cùng bài học **B5 của P8**: `armed && OFFBOARD && command_fresh` — không phải chỉ `armed`. Hai ca, hai package, cùng một khuôn: **một tín hiệu đúng bị đọc trong một pha mà nó mang nghĩa khác.** Đây là căn cứ của đề xuất **R35** (§7).

| Bài học phương pháp | Áp dụng chung |
|---|---|
| **HÌNH DẠNG của tín hiệu bác được giả thuyết, không chỉ GIÁ TRỊ của nó** | `age_sec` răng cưa **sạch 2 Hz** loại giả thuyết "đói executor" nhanh hơn mọi phép đo CPU. Nhìn *dạng sóng*, đừng chỉ nhìn *số* |
| **Log chẩn đoán tạm là tài sản, không phải rác** | 29 dòng WARN thêm vào để soi chính là thứ chốt hạ. Nhưng sau đó **phải hạ cấp**, không xoá — nó còn giá trị lần sau |
| **Regression do chính bản vá đẻ ra là chuyện BÌNH THƯỜNG** — điều bất thường là không bay hồi quy | Bản vá B1 đúng về lý lẽ, sai về **phạm vi**. Chỉ bay mới lộ ra |

> **🎓 Tự kiểm §5b:** Nếu ngày mai `active_source = NONE` xuất hiện ở pha `kRecovering` — mission nên PAUSE hay không? Vì sao khác với `kFinishLanding`?

---

### 5c. Search trả SUCCESS giữa hư không — ca "điều hướng ≠ tái-bắt" (bug #12)

#### Triệu chứng

```
follow_target · arena hoàn toàn SẠCH (không có vật nào bám được)
   ↓
mất mục tiêu → vào search_when_lost
   ↓
search1 SUCCESS  ·  search2 SUCCESS
   ↓
🔴 cây BT đọc thành "search thành công"
```

#### Cơ chế — một lỗi NGỮ NGHĨA, không phải lỗi logic

`search1`/`search2` là hai `GotoPose`. Chúng trả `SUCCESS` khi **bay tới nơi**. Nhưng câu hỏi mà cây BT đang hỏi là *"đã tìm thấy chưa?"*.

> 🔑 **Bay hết 2 điểm search KHÔNG đồng nghĩa "tìm thấy".** `SUCCESS` của một action điều hướng trả lời câu hỏi *"tới nơi chưa"*, không trả lời câu hỏi *"thấy gì chưa"*. Ai đọc nó thành câu thứ hai là đang **ghép sai câu hỏi với câu trả lời**.

#### Ba tầng vá — và vì sao phải cả ba

| Tầng | Vá | Nó bịt lỗ nào |
|---|---|---|
| 1 | `search_when_lost` thêm **`target_visible_recheck`** (`TargetSeen`) làm **con CUỐI** — khuôn `marker_visible_recheck` của `inspect_point.xml` | Bay hết 2 điểm mà không thấy ⇒ recheck FAIL ⇒ cả `search_when_lost` FAIL ⇒ **tiêu 1 episode ngân sách** |
| 2 | Bộ đếm ngân sách **CHUYỂN** từ *"TrackTarget tự thất bại"* sang **`SearchBudgetAvailable::tick()`** — chạy **VÔ ĐIỀU KIỆN** mỗi lần vào `search_when_lost` | Hook gốc **không thấy được** trường hợp mission **KHÔNG BAO GIỜ dispatch TrackTarget** (arena hoàn toàn sạch từ đầu). Nay phủ **cả hai** đường: *đã từng bám rồi mất* **và** *chưa từng thấy lần nào* |
| 3 | `tickBody()`: recheck fail mà **ngân sách còn** ⇒ **retry**, KHÔNG kết thúc mission. Phân biệt với `<Timeout>` hết hạn thật bằng **đồng hồ C++ độc lập** (`mission_start_time_`) | `<Timeout>` node của chính cây BT **bị reset** khi cây tự khởi động lại sau một trạng thái terminal — dùng nó làm mốc thời gian mission là sai |

#### Mặt SUCCESS đối xứng — chỗ dễ quên nhất

`search_when_lost` **thành công** (tái-bắt được) cũng **KHÔNG BAO GIỜ** đọc là mission SUCCESS: `follow_target` vốn **không có trạng thái "xong" hợp lệ** (`TrackTarget` luôn `duration_seconds = 0.0`) ⇒ retry giống `loop_mission_`.

⚠️ **Ghi chú trung thực trong README:** trong **cây thực tế**, `ReactiveFallback` tick lại `track_when_seen` từ đầu **mỗi tick ngoài**, nên nhánh đó luôn "thắng" một tái-bắt thật **trước khi** recheck có cơ hội thành công. ⇒ **Nhánh SUCCESS của recheck là phòng thủ, không phải đường thật sự chạy trong cây này.** Ghi ra chứ không giả vờ nó đã được chứng minh.

#### Kết quả G-M4.4 (5/5 PASS, arena-sạch)

```
2 episode search: 31,3 s + 46,9 s
   ↓ ngân sách search_attempts_max = 2 cạn
Recover(CLIMB) trên dây: 8,5 m → 10 m
   ↓
ABORTED_LOST_TARGET (mã 9) — KHÔNG BAO GIỜ ABORTED_TIMEOUT
tổng 134,8 s  <  mission-timeout 180 s
```

🔑 Chú ý dòng cuối: mission kết thúc bằng **mã đúng nguyên nhân**, và nó kết thúc **trước** timeout. Nếu để `<Timeout>` bắn trước, ta sẽ có một `ABORTED_TIMEOUT` che mất `ABORTED_LOST_TARGET` — **một lý do TẠM THỜI che một lý do THẬT**, đúng khuôn lỗi đã gặp ở `Recover(TYPE_LAND)` của P6.

> **🎓 Tự kiểm §5c:** Nếu chỉ vá tầng 1 (thêm recheck) mà bỏ tầng 2 — kịch bản nào vẫn lọt?

---

### 5d. "-1 rebind sang cột trụ" — bug hay tính năng?

#### Triệu chứng

```
follow_target với target_id = -1 ("bám bất kỳ")
   ↓
hộp mục tiêu bị GỠ khỏi world
   ↓
🔴 drone bám CỘT TRỤ (obstacle_pillar) — 58 s ổn định, 0 cảnh báo
   ↓
chuỗi lost → search → Recover KHÔNG BAO GIỜ chạy
```

#### Chuỗi truy nguyên

| # | Câu hỏi | Dữ kiện |
|---|---|---|
| 1 | `-1` đi thẳng tới navigator? | 🔴 **Trước fix G-M4.4: CÓ.** Và `-1` **vô hiệu hoá** bộ lọc `targetSample(wanted_id)` phía navigator (`navigator_action_server_node.cpp:2597`) ⇒ đánh bại **CẢ** bộ lọc M/N phía tracker **LẪN** cơ chế pin-theo-id phía navigator ⇒ track ma lọt qua và **reset đồng hồ `target_lost_timeout` vô thời hạn** |
| 2 | Sau khi fix neo-id, còn rebind không? | **CÒN** — nhưng cơ chế đã khác hẳn |
| 3 | Cơ chế sau fix | `TargetSeen` ghi `track_id` nó **VỪA thấy** ra blackboard (`{acquired_track_id}`); `NavAction` đọc port `target_id` từ **ĐÓ**. Goal ĐANG CHẠY **giữ nguyên id đã neo** tới khi tự kết thúc (`onStart()` đọc port **một lần**). Goal abort `LOST_TARGET` → search → tái-bắt ⇒ **neo id MỚI** cho goal kế tiếp |

#### Phán quyết: **TÍNH NĂNG**, kèm giới hạn vận hành

| Lý lẽ | Nội dung |
|---|---|
| Đúng hợp đồng | `-1` nghĩa là *"bám bất kỳ"*. "Bám vật vừa tìm được sau search" là **đúng nghĩa** của `-1`, kể cả khi id mới thuộc một vật thể khác |
| Mission **không có** khái niệm "cùng một mục tiêu logic qua nhiều lần mất dấu" | Muốn chặt hơn thì phải truyền `target_id` **cụ thể** — mà schema `mission_registry` hiện **đóng băng**, chưa có field đó (lệch #2 của P9.4) |
| 🔴 **Giới hạn vận hành, ĐÃ VÀO CONTRACT §2.19** | *"Bay thật với nhiệm vụ đòi bám đúng MỘT vật thì phải truyền `target_id` cụ thể, và người thiết kế mission phải khảo sát vùng bay có vật bám-được nào khác không. Chuỗi lost/search/recover chỉ chạy trọn khi THẬT SỰ không còn gì bám được."* |

Và bằng chứng cho phán quyết đó: cổng G-M4.4 phải **gỡ cột trụ khỏi arena lúc runtime** ("arena-sạch") mới đo được chuỗi lost→search→Recover. Coordinator ghi thẳng: *cổng cũ đang test sai kịch bản*.

#### 🔑 Bài học phương pháp — khó nhất trong bốn ca

> **Một hành vi bất ngờ không tự động là bug. Câu hỏi đúng là: "hợp đồng nói gì?" — và nếu hợp đồng im lặng thì đó là NỢ HỢP ĐỒNG, không phải nợ code.**

Ba bước phán quyết dùng lại được:

1. **Hành vi này có mâu thuẫn một câu đã viết ra không?** Không → chưa phải bug.
2. **Nó có làm một cơ chế an toàn khác mất hiệu lực không?** — Ở đây **CÓ** ở bản trước fix (`-1` giết bộ lọc navigator) ⇒ phần đó **là bug thật, phải sửa**. Sau fix thì **không** ⇒ phần còn lại là tính năng.
3. **Nếu là tính năng: ai phải biết, và biết ở đâu?** ⇒ ghi **giới hạn vận hành** vào contract + nghĩa vụ cho pre-flight, không ghi vào comment code.

⚠️ Đây cũng là ca cho thấy **cổng có thể test sai kịch bản mà vẫn "chạy"**: G-M4.4 bản cũ chưa bao giờ đo được thứ nó định đo, vì arena luôn còn vật bám được.

> **🎓 Tự kiểm §5d:** Tách ca này thành hai phần: phần nào là **bug thật phải sửa**, phần nào là **tính năng kèm giới hạn**? Ranh giới nằm ở đâu?

---

## 6. PHẦN SÁT AN TOÀN — GIẢNG SÂU (R0)

Nếu chỉ nhớ được ba điều từ cả buổi thì nhớ ba mục này.

### 6a. Chuỗi cắt quyền END-TO-END — đọc từng mắt xích, kèm số

```
[1] safety_supervisor_node quyết INHIBIT
    ├ điều kiện: BLIND_COMMAND ∨ BATTERY_PX4_NO_ACTION ∨ FRAME_MISMATCH
    ├ cổng B5: chỉ LATCH khi armed && OFFBOARD && command_fresh (fail-OPEN)
    └ ⛔ KHÔNG publish cmd_safety · ⛔ KHÔNG set_mode/arm/disarm
                     │  SetControlAuthority(SOURCE_SAFETY), async + callback
                     ▼
[2] authority_arbiter dựng SÀN = SAFETY
    ├ hiệu lực NGAY, không cần latch_ever_alive (ngoại lệ P8 R2)
    ├ KHÔNG grace-expiry, KHÔNG timeout-expiry (R5)
    └ đường gỡ DUY NHẤT: clear_safety_latch  (+ PX4 failsafe / RC là đường vật lý)
                     │
                     ▼
[3] mọi kênh < SAFETY bị loại khỏi ứng viên
    cmd_mission dù tươi 20 Hz cũng KHÔNG được chọn
                     │
                     ▼
[4] /control/command_selected IM LẶNG TUYỆT ĐỐI
    ├ đo ở G-S1 #3: 0 msg MISSION trong 500 ms
    └ đối chứng dương: mission VẪN phát trên cmd_mission (nguồn còn sống thật)
                     │
                     ▼
[5] px4_command_gateway_node đệm nốt command_timeout_sec 0,5 s rồi NGỪNG
                     │
                     ▼
[6] PX4 mất offboard → failsafe lõi → trả quyền pilot
    số đo R3 đã GHI đủ ở G-S2 cho từng biến thể tiêm lỗi
                     │
                     ▼
[7] uav_mission thấy /control/authority ≠ MISSION ≥ 1,5 s
    → CANCEL goal navigator → PAUSED → chỉ ResumeMission chạy tiếp
```

**Ngân sách thời gian toàn chuỗi, cộng dồn worst-case:**

| Chặng | Thời gian | Nguồn |
|---|---|---|
| Phát hiện → latch | `blind_grace_sec` 1,5 s (BLIND) hoặc onset **0,560–0,588 s** (OBSTACLE, đo thật) | G-S3-HOLD |
| Latch → `command_selected` im | ≤ 1 tick giám sát (20 Hz) = **0,05 s** | thiết kế |
| Im → gateway ngừng | `command_timeout_sec` **0,50 s** | gateway |
| Gateway ngừng → PX4 failsafe | đo được **~1,5 s** ở G-S3 B1 (`failsafe_active` +1,515 s) | G-S3 B1 |
| Mission phát hiện mất quyền | ≥ **1,5 s** giữ liên tục | policy P9 #2 |

🔑 **Quãng bay mù trần = 0,55 m/s × 1,5 s = 0,83 m.** Đo thật: **0,000–0,003 m** (nhờ lớp 1). Nhưng đừng quên §3b: con số 0,003 đó **không chứng minh** đường cắt hoạt động.

#### Ba điểm mà một sửa đổi nhỏ sẽ phá cả chuỗi

| Nếu ai đó… | Chuỗi gãy ở đâu |
|---|---|
| cho trọng tài publish hover khi mọi nguồn im | **[5] không bao giờ tới** — gateway luôn thấy lệnh tươi |
| cho safety phát một lệnh "vô hại" (vận tốc 0) để nuôi liveness | **[4] không im** — cùng hậu quả |
| khôi phục grace/timeout-expiry cho latch SAFETY | **[2] tự nhả sau 5 s** — đúng race R5, `command_selected` quay lại chọn MISSION trong khi safety tưởng mình đang giữ |
| bỏ cancel ở [7] | Goal cũ **tự chảy lại** sau ClearFault — đối chứng dương G-M4.3 chứng minh |

---

### 6b. Đối chứng dương "goal tự chảy lại" — vì sao nó là mục sát an toàn, không phải mục test

Bình thường một cổng chứng minh *"hệ làm đúng việc X"*. Đối chứng dương chứng minh *"nếu KHÔNG làm X thì có hậu quả Y"* — và nó là thứ duy nhất biện minh cho việc X tồn tại.

| Lượt | Nội dung | Kết quả bắt buộc |
|---|---|---|
| Chính | Mission cancel → PAUSED → ClearFault (probe gọi) → **vẫn PAUSED** tới `Resume` | 6/6 PASS |
| **Đối chứng dương** | **BỎ cancel** ⇒ goal Goto gốc **phải tự chảy lại** | Không nổ ⇒ 🔴 **CỔNG FAIL** |

Cùng khuôn với ba đối chứng dương khác đã cứu dự án:
- **P6:** carrot **14,21 m/s²** phải bị G-N2 bắt — không nổ ⇒ cả cổng FAIL.
- **P8/V1:** tiêm obstacle 0,18 m khi đậu **phải** cắn debounce 0,500 s — nếu không, "30 s sạch" chỉ chứng minh **không có gì chạy**.
- **P7/G-CA1 #1:** tắt MISSION thì TEST **phải** ra dây — nếu không, "TEST = 0 tuyệt đối" chỉ chứng minh **cả hai nguồn đều chết**.

> 🔴 **MỘT KHẲNG ĐỊNH PHỦ ĐỊNH KHÔNG KÈM ĐỐI CHỨNG DƯƠNG LÀ MỘT KHẲNG ĐỊNH RỖNG** (R27-3). "Không thấy X" và "X không xảy ra" là hai mệnh đề khác nhau; chỉ đối chứng dương nối được chúng.

---

### 6c. Vì sao mission phải CANCEL — lý lẽ đầy đủ, ba tầng

1. **Tầng vật lý:** navigator giữ goal suốt thời gian mất quyền (đo thật). Sau `ClearFault`, setpoint bắt đầu chảy lại **ngay lập tức, không có bước xác nhận nào**.
2. **Tầng con người:** người vận hành `ClearFault` đang trả lời câu hỏi *"lỗi đã hết chưa?"*. Họ **không** đang trả lời câu hỏi *"có tiếp tục nhiệm vụ không?"*. Cho một hành động trả lời hộ hai câu hỏi là thiết kế sai.
3. **Tầng hệ thống:** nếu goal tự chảy lại, thì **cùng một goal đã dẫn drone tới vật cản** sẽ được bay lại y nguyên, với y nguyên tham số, trong một thế giới có thể đã đổi.

⇒ `PAUSED` **không tự bay**. Chỉ `ResumeMission` — một hành động có chủ thể, có dấu vết trong `mission/events`.

🪤 **Và `PAUSED` cũng không tự chết:** quá `paused_timeout_sec` 180 s ⇒ `ABORTED_TIMEOUT`, **giữ HOLD**, không tự hạ cánh. (⚠️ Xem §8 nợ #1: nhánh này hiện **bất lực** với chính nguyên nhân PAUSE hay gặp nhất — trung thực ghi ra.)

---

## 7. CÁC RULE SINH RA TỪ BA PHASE

Mỗi rule 2 câu: **ca thật đẻ ra nó** + **cách áp**. Toàn văn ở `.claude/memory.md` §1.

### Đã duyệt

| Rule | Ca thật đẻ ra nó | Cách áp |
|---|---|---|
| **R24** — state dùng chung giữa 2 callback group phải atomic/có khoá | P7 B1: service `SetControlAuthority` ở group riêng ⇒ **SAFETY mất quyền ngay sau khi giành được**. Test không bao giờ đỏ ổn định; chỉ review chặn được | Mỗi lần tách callback group trong `MultiThreadedExecutor`: **liệt kê state mà cả hai nhóm chạm** và chứng minh atomic/có khoá — hoặc gộp nhóm |
| **R25** — ranh giới R1 cho công cụ dòng lệnh | 15+ script cổng dùng `ros2 topic hz /fmu/*`; tốn một lượt review cãi nhau | `ros2 topic/echo/list` trên `/fmu/*` **trong shell script** là hợp lệ. Mọi **file mã nguồn** ngoài `uav_px4_backend`, kể cả probe `.py`, vẫn **cấm tuyệt đối** `import px4_msgs` |
| **R26** — mọi đường làm luồng lệnh im lặng phải có bất đẳng thức validate + test đo trên dây | P7: Y1 canh kỹ đường **dwell** nhưng đường **demotion** — cùng hậu quả, worst-case dài hơn — **không có ràng buộc nào** ⇒ CHẶN B2 | Liệt kê **mọi** đường có thể làm topic lệnh im (dwell, hết hạn, demote, latch…), mỗi đường một bất đẳng thức trong `validate()`. **Cách thoả tốt nhất là CẤU TRÚC: gộp các đường về một đường đã được canh** |
| **R27** — toàn vẹn cổng & phép đo, 5 điều | 5 ca: `peakAcceleration` 0,897 vs 55,7 · SHM-only cắt PX4 mà 9/9 PASS · `sleep 30` + "nodes up: 0" vẫn bay · escape thiếu kiểm cao độ | (1) kiểm phép đo **trước** khi kiểm đối tượng; (2) cấm `max()/min()` nuốt NaN; (3) **đối chứng dương hỏng ⇒ cổng hỏng**; (4) cổng phải đi qua **đúng mạch** nó bảo vệ; (5) mọi waypoint do node khác đặt vào đường bay phải qua **đúng bộ kiểm** của caller |
| **R28** — đếm-gộp không làm căn cứ hành-động-chọn-lọc | P8 CHẶN B1: `FRAME_MISMATCH` đếm **gộp** drop mọi kênh ⇒ 1 msg sai frame trên kênh **phụ** latch vĩnh viễn kênh **giữ quyền** | Muốn hành động chọn lọc ⇒ phải có tín hiệu **per-channel**. Trọng tài nay phát 4 key `dropped_wrong_frame_*` |
| **R29** — grace của tín hiệu lấy mẫu chậm ≥ 2× chu kỳ, chu kỳ là `*_copy` có validate | P8 CHẶN B1: grace 1,0 s **đúng bằng** chu kỳ lấy mẫu 1 Hz ⇒ **biên 0**, một mẫu trễ là kết luận sai | Ghi ràng buộc vào `validate()`, chu kỳ là **bản sao có tên**, không phải số bịa |
| **R30** — "0" không được là mặc định của đại lượng an toàn chưa có dữ liệu | P8 CHẶN B2: `battery_remaining` mặc định **0,0** đọc thành CRITICAL ⇒ **latch trước khi arm**. Sentinel PX4 `-1` cùng họ | Chưa có dữ liệu ⇒ **NaN**, và consumer phải kiểm **TUỔI** dữ liệu. 0 thường trùng đúng giá trị nguy hiểm nhất |
| **R31** — số đo dưới điều kiện ép nhân tạo chỉ có hiệu lực trong điều kiện đó | P8 CHẶN B3: *"PX4 phản ứng 0,58 s sau CRITICAL"* chỉ đúng dưới **drain nhân tạo**; bay thật chạm 10% pin theo cách khác | Mọi kết luận đo dưới drain/nhiễu/tải nhân tạo phải **khoanh vùng điều kiện** khi ghi. G-M4.2 dán nhãn R31 |
| **R32** — sample-and-hold phải mang tuổi | P8 Y1/N3: `wrong_frame_drop_increasing_` kẹt `true` khi diagnostics trọng tài chết ⇒ FRAME_MISMATCH **latch giả** / ClearFault không bao giờ gỡ được. `localization_jump_count_` giữ `jumps>0` vĩnh viễn khi health node chết | Mọi cờ/giá trị chốt-giữ đọc từ stream node khác phải kèm **dấu thời gian NHẬN** và tự hết hạn khi tuổi > **2× chu kỳ phát** (`*_copy` có validate). Hết hạn ⇒ đọc là **"không đo được"**, không phải giá trị cuối |

**R32 đã dùng lại 3 lần trong P9** — dấu hiệu một rule tốt: `freshBatteryOrNan()` (pin) · `latestLandmark()`/`latestTarget()` (B2 review, tuổi **cả tin nhắn** chứ không chỉ từng landmark) · thoái lui 2 tầng của ego-motion (`resolveBodyPose()`).

### Ba rule mới — chủ dự án DUYỆT 2026-08-23

| Rule | Ca thật đẻ ra nó | Cách áp đề xuất |
|---|---|---|
| **R33** — param-validate **không thực thi** = nói dối | P9 review: có tham số được validate lúc khởi động nhưng **không đường code nào thực sự dùng nó** để chặn ⇒ tài liệu và yaml hứa một lá chắn không tồn tại | Mỗi ràng buộc trong `validate()` phải có **ít nhất một test chứng minh nó CHẶN được một cấu hình xấu thật**; validate mà không ai đọc giá trị ⇒ gỡ ràng buộc hoặc nối vào chỗ dùng |
| **R34** — công cụ audit phải **DERIVE** danh sách, không hardcode | `audit_comments.sh` giữ danh sách file cứng ⇒ file mới **không bao giờ được audit**, và công cụ vẫn báo xanh | Mọi script audit/kiểm kê phải `find`-derive danh sách đối tượng. Đã chuyển `audit_comments.sh` sang find-derived |
| **R35** — thu hẹp vị ngữ phải sửa **cùng lượt** mọi phát biểu đối ngoại | **BA ca**: B5-P8 (`armed` → `armed && OFFBOARD && command_fresh`) · B1-P9 (`active_source != MISSION` → kèm tiền đề pha) · G-M1 (miễn `kFinishLanding`) | Khi một vị ngữ an toàn bị **thu hẹp**, phải sửa **cùng lượt**: contract + README + test + comment. Một phát biểu đối ngoại còn nói phiên bản rộng là một lời hứa sai |

**Ứng viên chưa đủ chín** (ghi để không quên): *thoái-lui-im-lặng-phải-quan-sát-được* · *guard mới phải có test HAI PHÍA (cắn và không cắn)* · *script cổng phải rà lại khi mặc định launch đổi* (loạt tiền-P7.4 còn **TREO** — `run_gn5.sh` đã dính lỗi double-navigator) · *chuyến bay xác nhận phải ghi rõ binary/mốc build nó phủ* (ca `gm1_taskE_confirm` 2026-08-23: G-M1 PASS cuối hoá ra bay trên binary cũ hơn bản build sau đợt dọn — phải bay lại).

---

## 8. NHỮNG THỨ CHƯA CHỨNG MINH / NỢ MỞ — phần trung thực

### 8a. `uav_control_authority` (package-status §9)

| # | Nợ | Vì sao còn mở |
|---|---|---|
| ~~N-c~~ | `now − last_valid_arrival` có thể **ÂM** nếu đồng hồ sim lùi ⇒ kênh bị coi **LIVE vĩnh viễn** | ✅ **ĐÃ ĐÓNG SAU BUỔI GIẢNG:** N-c (P10.8a) + C4 (P10.8b), 2026-08-23 — **6** phép trừ thời gian route qua helper `ageSecOrInf()` (đồng hồ lùi ⇒ tuổi `+inf`, đếm `clock_regressions_`); latch phi-SAFETY được nhả, latch SAFETY bất động (Q-P10-7). Xem package-status §9 · contract §2.16 |
| N-d | `uav_id` **không được kiểm tra hợp lệ** | Đường cho multi-drone sau này; hiện chỉ đơn drone |
| ~~N-e~~ | ✅ **ĐÓNG 2026-08-24 — R16 ngoại lệ 6** | File test được giữ **banner khối-claim** + **`///` nói test CHỨNG MINH gì**; vẫn cấm kể-lại-code/lịch sử/trang trí. Chốt theo hướng ghi nhận thực hành đang có (306 banner/14 file), không đi dọn |

### 8b. `uav_safety` — sổ nợ = 0, nhưng có BỐN giới hạn phải nhớ

| # | Giới hạn | Ghi chú |
|---|---|---|
| 1 | 🔴 **`BLIND_COMMAND` chưa từng nổ trong bay thật** | Đường cắt chỉ được G-S1 chứng minh (ép setpoint di chuyển qua probe). Đừng đọc G-S2 thành "đã verify bằng bay thật" (R27-4) |
| 2 | 🔴 **Kênh sai frame 100% có thể KHÔNG BAO GIỜ latch `FRAME_MISMATCH`** | Mọi msg bị DROP ⇒ topic hạ nguồn im ⇒ `command_fresh` sập ở 0,5 s **trước** `frame_mismatch_grace_sec` 2,0 s. Kết cục **vẫn an toàn** qua đường offboard-mất → PX4 failsafe, nhưng đừng đọc hợp đồng thành "sai frame thì chắc chắn latch" |
| 3 | ⚠️ **R32 vẫn để lọt một cửa sổ** | Nguồn chết **GIỮA** một đợt rising thật ⇒ FRAME_MISMATCH vẫn có thể vào trong cửa sổ 2 chu kỳ. Bằng chứng trước khi chết **là thật** ⇒ lệch về phía an toàn. Thứ R32 chặn là **kẹt vĩnh viễn**, không phải mọi dương tính giả |
| 4 | 🟡 **Y3 — nghĩa vụ vận hành, chưa phải code** | Arm cạnh vật cản < 0,35 m ⇒ `OBSTACLE_TOO_CLOSE` **latch tức thì**. Chủ dự án ký GIỮ hành vi. **Nghĩa vụ:** pre-flight checklist P11 phải ghi *"arm cạnh vật cản < 0,35 m thì ClearFault trước khi chạy mission"* |

Ngoài ra: `ESTIMATOR_INPUT_INVALID`/`CAMERA_STREAM_UNHEALTHY` grace **0,0** và 3 số hysteresis `ClearFault` là **số bảo thủ chưa đo-rồi-đặt** — chấp thuận 2026-08-22, **rà lại ở P11** khi có số pin/cảm biến thật.

### 8c. `uav_mission` (package-status §11, review lượt 2)

| # | Nợ | Mức |
|---|---|---|
| 1 | 🟡 **`paused_timeout_sec` BẤT LỰC với nguyên nhân PAUSE hay gặp nhất** — PAUSED do priority-2 (authority-seize) đang giữ thì priority 2 **luôn thắng** priority 4 trong `evaluate()` ⇒ nhánh paused-timeout-quá-hạn (`ABORT_HOLD`) **không bao giờ được xét tới** khi seize còn | 🟡 rẻ, không chặn |
| 2 | `ContinuityTracker::update()` trả **0.0** ở tick ĐẦU predicate vừa true — hợp lý cho hầu hết chỗ dùng nhưng là **giả định ngầm chưa ghi ở đâu** | 🟢 |
| 3 | `odometryFresh()` **tính lại 3 lần** trong 1 tick ở vài đường code (không cache) — không sai, nhưng rủi ro **trôi** nếu sau này tách logic | 🟢 |
| 4 | `yaw_fallback_warned_` **không bao giờ reset** — odometry rớt → phục hồi → rớt lại ⇒ **WARN thứ 2 bị nuốt im lặng** | 🟢 |
| 5 | 🔴 **Navigator không trả lời CẢ cancel LẪN goal gốc (treo vô thời hạn) hiện được CHẤP NHẬN là rủi ro còn lại** — dựa vào PX4 failsafe lõi đỡ. **Chưa có timeout riêng ở tầng mission cho chính kịch bản này** | 🟢 theo review, nhưng là chỗ đáng để mắt ở P11 |
| 6 | Test Y2 (`TwoGoalsSentBackToBackYieldExactlyOneAcceptOneReject`) **không ép được interleaving thật** — xanh trên **cả code cũ lẫn mới** ⇒ **không phải bằng chứng đỏ-trước thật** | 🟢 |

### 8d. Nợ liên phase mang từ trước — vẫn còn

| Nợ | Nội dung |
|---|---|
| 🟠 **EKF2 reset EV aiding 1 lần/giây** | `reset_{pos,vel}_to_vision` 71 lần/70 s. Sim vô hại (EV là ground truth); **VIO thật thì mỗi giây bơm một cú nhảy vào vận tốc bộ ước lượng**. 🔴 **Nguy cho ĐỜI THẬT, phải đóng trước P11** |
| 🟠 **Trễ mũi TrackTarget đỉnh 93,7°** | Đã đo (G-N5), **chưa có ngưỡng**. G-M3 **CẤM khẳng định hướng mũi** vì lý do này |
| 🟠 **Nợ yaw bàn giao giữa hai trajectory kế tiếp** | Chính vì nó mà contract §2.19 cấm `uav_mission` subscribe `/planning/trajectory` — **không cho một nợ chưa giải leo lên tầng cao hơn** |
| 🟠 **`InspectMarker.action` chưa có chủ** | D-5: không hiện thực ở P9; `inspect_point` soạn từ primitive. Interface tồn tại mà **không có server** — ghi sổ nợ |
| 🟡 **Loạt script cổng tiền-P7.4 chưa rà lại** | `run_gn5.sh` đã dính lỗi double-navigator sau khi mặc định launch đổi. Còn **TREO** |

---

## 9. CÁCH TỰ KIỂM CHỨNG — lệnh anh gõ được ngay

Tất cả trong WSL, workspace `~/PX4_ROS2`. Cây nguồn chuẩn ở Windows `C:\code\PX4_ROS2\src\`.

### 9.1 Phần tất định (không cần Gazebo) — nơi ĐẶT LÝ LẼ AN TOÀN

```bash
cd ~/PX4_ROS2
bash scripts/verify_workspace.sh          # 11 package
colcon test-result --all                  # báo "N case / M target" — SUY LẠI, đừng chép
```

**Đúng khi (mốc 2026-08-23):** toàn workspace **879+ case / 43 target, 0 lỗi lượt gộp**; trong đó `uav_control_authority` **81/2** · `uav_safety` **120/3** · `uav_mission` **106/4** · `uav_navigation` **269/11**.
🪤 Chéo kiểm: `Summary` = case + target (879 + 43 = 922 ✓). Số target thiếu = **thiếu nguyên một target** — đúng cái đã sai ngày 08-17.

### 9.2 Ba lõi ROS-free (chạy vài chục ms, không cần domain)

```bash
colcon test --packages-select uav_control_authority --ctest-args -R test_authority_arbiter
colcon test --packages-select uav_safety            --ctest-args -R test_failsafe_policy
colcon test --packages-select uav_mission           --ctest-args -R "test_mission_policy|test_mission_registry"
```
Đây là **trái tim an toàn** của cả ba phase. Nếu ba lệnh này xanh mà cổng bay đỏ ⇒ nghi phép đo trước, nghi logic sau.

### 9.3 Ba chuỗi tích hợp "node thật + node thật trong MỘT tiến trình"

```bash
colcon test --packages-select uav_safety  --ctest-args -R test_safety_cut_chain          # G-S1, domain 97
colcon test --packages-select uav_mission --ctest-args -R test_mission_authority_chain   # P9.4b, domain 98
colcon test --packages-select uav_mission --ctest-args -R test_mission_executor_node     # 6 case (a)-(f)
```
🔑 `test_mission_authority_chain` chạy `control_authority_manager_node` **THẬT** ⇒ phải build `uav_control_authority` **cùng lượt**.

### 9.4 Sáu cổng bay (mỗi lượt ~10–20 phút, **XẾP HÀNG** — không hai sim cùng lúc)

```bash
bash scripts/run_m5_regression.sh          # hồi quy nền, PHẢI PASS 3/3 ở mọi bước
bash scripts/verify_control_authority.sh   # G-CA1 + G-CA2
bash scripts/verify_boot_no_latch.sh       # V1 — 30 s disarmed + đối chứng dương
bash scripts/verify_obstacle_hold.sh       # G-S3-HOLD
bash scripts/verify_safety.sh              # G-S2 — 🔴 PHẢI uav0_nav_indoor
bash scripts/verify_mission.sh             # 9 vòng: m1 m2 m3 m4a m4b m4c m4c-control m4d m5
```
Chạy **một vòng** của mission gate: `bash scripts/verify_mission.sh m4c` (exit **0/1/2** — 2 = FAILED TO MEASURE, **không phải PASS**).

| Nhìn | Ngưỡng | Nếu lệch |
|---|---|---|
| RTF cửa sổ bay | ≥ 0,95 mới **không cần nhãn** | < 0,95 ⇒ số liệu **DÁN NHÃN**, không cấp chứng nhận |
| G-N2 lead qua trọng tài | ≤ **0,65 m** (mốc 0,516–0,579) | Sát 0,80 ⇒ hop đắt hơn dự tính ⇒ **BÁO CHỦ DỰ ÁN, không tự nới tham số an toàn** |
| G-CA2 gap | **0,052 / 0,248 s** | Lệch khỏi mốc ⇒ nghi thay đổi timing |
| G-S3-HOLD onset | **0,560–0,588 s**, pose trôi **0,000000 m** | Trôi ≠ 0 ⇒ ai đó đã thêm nội suy vào HOLD |
| 🔴 **G-S2 model** | **`uav0_nav_indoor`** | Bay `uav0_nav` thường trong `uav_arena_indoor` thì **GPS vẫn có fix** ⇒ mux âm thầm failover ⇒ **che khuất TOÀN BỘ phép thử** (lượt 1: 6/6 lần tiêm không thấy `is_valid=false`) |

### 9.5 Nhìn hệ thống ĐANG SỐNG (khi chẩn đoán)

```bash
ros2 topic hz   /uav/uav0/control/command_selected     # ~20 Hz — dưới 2 Hz là rớt offboard
ros2 topic echo /uav/uav0/control/authority --once     # LATCHED: có ngay cả khi vào muộn
ros2 topic echo /uav/uav0/safety/state --once          # LATCHED: recommended_action
ros2 topic echo /uav/uav0/mission/status --once        # LATCHED: last_result_code + goal_id
ros2 topic echo /uav/uav0/diagnostics/control_authority --once   # 6 key latch (Y15)
ros2 topic info /uav/uav0/control/cmd_safety --verbose # enforcement=false ⇒ 0 publisher
ros2 topic info /uav/uav0/control/command_selected -v  # PHẢI đúng 1 publisher
```

🪤 **Ba bẫy đọc kết quả:**
- `/safety/violations` là **edge-only + Volatile** ⇒ `echo` im **KHÔNG có nghĩa "sạch"**. Trạng thái tổng đọc ở `/safety/state`.
- `/mission/events` cũng **Volatile có chủ đích** — nó là **luồng cạnh lịch sử**, không phải trạng thái. Đọc `mission/status` cho trạng thái.
- `recommended_action` khi `safety_enforcement:=false` mang tiền tố dry-run (`would_hold (dry-run, ...)`) ⇒ **so TIỀN TỐ, không so chuỗi đầy đủ**.

---

## 10. CÂU HỎI TỰ KIỂM

| # | Câu hỏi | Đọc lại |
|---|---|---|
| 1 | Trọng tài thấy mọi nguồn chết. Nó làm gì, và **kể ra chuỗi 4 bước** dẫn tới kết cục an toàn | §2a, §6a |
| 2 | Vì sao `SAFETY` được miễn **cả hai** nhánh hết hạn latch, còn OPERATOR/MISSION/TEST thì không? | §2c |
| 3 | B2 của P7: một kênh phát 100% rác có thể làm được **ba** việc gì trước khi vá? | §2d |
| 4 | Mất định vị: ba phản xạ tự nhiên bị loại, mỗi cái vì lý do gì? Tiêu chí phân biệt "bay mù" với "giữ đóng băng" là gì, **đo ở đâu**? | §3b |
| 5 | G-S2 PASS 4/4 với `blind_distance = 0,003 m`. Được nói (a) "BLIND_COMMAND đã verify bằng bay thật" hay (b) gì? | §3b |
| 6 | Vì sao battery đứng **dưới** authority trong bảng ưu tiên mission? | §4d |
| 7 | Sau `ClearFault`, goal navigator tự chảy lại. Đó là bug của safety hay là hành vi đúng? Ai phải xử lý, và **bằng gì**? | §4c, §6b, §6c |
| 8 | Ca ego-motion: dữ kiện nào **loại sạch** nhóm nghi phạm lớn nhất trong một bước? Con số chốt hạ là bao nhiêu, so với cái gì? | §5a |
| 9 | Ca PAUSE giả: **29 dòng WARN** bác giả thuyết "đói executor" bằng cách nào — bằng giá trị hay bằng hình dạng? | §5b |
| 10 | "-1 rebind sang cột trụ": phần nào là **bug thật**, phần nào là **tính năng kèm giới hạn**? Ranh giới ở đâu? | §5d |
| 11 | Kể tên **ba** sửa đổi nhỏ, mỗi cái phá chuỗi cắt quyền ở một mắt xích khác nhau | §6a |
| 12 | R30 và R32 khác nhau chỗ nào? Cho một ví dụ trong P9 áp cả hai | §7 |

### Đáp án ngắn

**1.** Im lặng hoàn toàn. Chuỗi: `command_selected` im → gateway đệm nốt **0,5 s** rồi ngừng `/fmu/in/*` → PX4 mất offboard → failsafe lõi + trả quyền pilot. Hover tự sinh sẽ giữ offboard sống **khi không ai lái** — chế độ hỏng tệ nhất thiết kế ra được.

**2.** Vì SAFETY **không thể** chứng minh mình đang lái: policy-1 cấm nó phát setpoint trên pose hỏng ⇒ grace-expiry là yêu cầu tự mâu thuẫn. Còn timeout-expiry bị **G-S1 #8** bắt: HOLD→INHIBIT khiến SAFETY *từng sống rồi chủ động im* ⇒ auto-revoke ở +5 s trong khi safety vẫn tưởng mình giữ quyền (R5). Ba mức kia giữ nguyên Y4 vì hazard của chúng là *"latch rồi cầm nhầm cần khoá một mission đang bay tốt"*.

**3.** (a) **Cướp quyền** bằng ưu tiên rồi im 0,55 s > trần gateway 0,50; (b) giữ một **latch hiệu lực vĩnh viễn** (holder phát rác vẫn "sống") — latch SAFETY thì **không đường thoát**; (c) giữ quyền vô hạn bằng nhịp **1-hợp-lệ : 9-rác**. Sửa bằng cấu trúc: `last_valid_arrival_` tách khỏi `last_arrival_`.

**4.** Auto-land ❌ (không biết đang ở đâu — contract §2.5 "điểm sát sinh mạng nhất"); safety phát lệnh vô hại ❌ (mọi publish giữ gateway sống ⇒ **ngăn đúng cái failsafe mình muốn**); gọi `set_mode` ❌ (đường ghi thứ hai xuống FC, phá single-writer). Tiêu chí: setpoint **DI CHUYỂN > 0,05 m trong W=3 tick** ⇒ cắt; đứng yên ⇒ violation nhưng không cắt. **Đo trên `command_selected` đã phát ra dây**, không đọc cờ nội bộ của navigator.

**5.** Chỉ (b): *"lớp 1 (navigator tự đóng băng) hoạt động đúng, chặn trước khi `blind_grace` 1,5 s trôi hết"*. Đường cắt của `BLIND_COMMAND` chỉ được **G-S1 #2/#3/#4** chứng minh, bằng cách ép setpoint di chuyển qua probe, **bỏ qua navigator** (R27-4).

**6.** Vì **một lệnh `end_early` gửi khi không giữ quyền là một lệnh vào hư không**. Đảo thứ tự thì mission dispatch `GotoPose(home)` trong lúc safety đang latch ⇒ REJECT/treo, mà mission tưởng mình đang bay về.

**7.** **Hành vi đúng** của safety (nó cố ý không cancel — điều cấm §10 plan P8). Mission phải xử: **cancel goal khi mất quyền ≥ 1,5 s → PAUSED → chỉ `ResumeMission`**. Bằng chứng bắt buộc là **đối chứng dương G-M4.3**: một lượt BỎ cancel **phải** thấy goal tự chảy lại, không nổ ⇒ cổng FAIL.

**8.** Đối chiếu **tĩnh (0 churn) vs bay (2–5 chu kỳ/vòng)** — loại sạch mọi nghi phạm không phụ thuộc chuyển động. Con số chốt: **~1,53 m** dịch chuyển biểu kiến trong frame camera, so với `min_association_gate_m` **0,5 m** ⇒ **gấp ~3 lần**, tính tay trước khi sửa code.

**9.** Bằng **HÌNH DẠNG**: `age_sec` **răng cưa sạch đúng 2 Hz** = arbiter phát nhịp tim đúng giờ **và** mission nhận đúng giờ ⇒ cả hai node khoẻ ⇒ đói executor bị bác. Cơ chế thật: navigator trao quyền cho **PX4 LAND native** ⇒ không ai stream `cmd_mission` ⇒ `NONE` là **an toàn đúng dự kiến**; vị ngữ `!= MISSION` thiếu **tiền đề pha** (họ B5-P8).

**10.** **Bug thật:** `-1` gửi thẳng cho navigator **vô hiệu hoá** bộ lọc `targetSample(wanted_id)` ⇒ đánh bại cả M/N phía tracker lẫn pin-theo-id phía navigator ⇒ track ma reset `target_lost_timeout` vô thời hạn. **Tính năng:** sau khi neo id đúng, việc rebind sang vật khác sau search là **đúng nghĩa `-1`** ⇒ ghi **giới hạn vận hành** vào contract §2.19. Ranh giới: *hành vi có làm một cơ chế an toàn khác mất hiệu lực không?*

**11.** (a) Trọng tài publish hover khi mọi nguồn im ⇒ mắt xích [5] không tới; (b) safety phát lệnh vận tốc 0 để nuôi liveness ⇒ [4] không im; (c) khôi phục grace/timeout-expiry cho latch SAFETY ⇒ [2] tự nhả sau 5 s. (Thêm: bỏ cancel ở [7] ⇒ goal tự chảy lại.)

**12.** **R30** nói về **giá trị khởi tạo** (chưa có dữ liệu ⇒ NaN, không phải 0); **R32** nói về **tuổi của giá trị đã có** (sample-and-hold phải tự hết hạn sau 2× chu kỳ nguồn). Ví dụ áp cả hai trong P9: `freshBatteryOrNan()` — pin chưa có dữ liệu là **NaN** (R30) **và** một bản đọc pin từ node chết phải **hết hạn về NaN sau 2× chu kỳ 10 Hz** (R32).

---

## 11. BA CÂU MANG VỀ

> **1. ĐƯỜNG CỨU MẠNG LÀ MỘT CHUỖI IM LẶNG CÓ KIỂM SOÁT, KHÔNG PHẢI MỘT CHUỖI LỆNH.** Mọi phương án "gửi một lệnh gì đó cho chắc" — hover của trọng tài, lệnh vận tốc 0 của safety, `set_mode` xuống FC — đều bị loại ở mức R0 vì cùng một lý do: **chúng giữ offboard sống khi không ai còn đủ tư cách lái.**
>
> **2. VỊ NGỮ AN TOÀN PHẢI KÈM TIỀN ĐỀ PHA.** `armed` không đủ (B5-P8). `active_source != MISSION` không đủ (B1-P9). Một tín hiệu đúng, đọc trong một pha mà nó mang nghĩa khác, là một guard **cắn nhầm** — và guard cắn nhầm khó thấy hơn guard không bao giờ cắn, vì hệ thống trông có vẻ "cẩn thận".
>
> **3. MỘT KHẲNG ĐỊNH PHỦ ĐỊNH KHÔNG KÈM ĐỐI CHỨNG DƯƠNG LÀ MỘT KHẲNG ĐỊNH RỖNG.** "0 msg ra dây" · "30 s không latch" · "mission vẫn PAUSED" — cả ba chỉ có nghĩa khi có một lượt chứng minh **nếu bỏ cơ chế ra thì có hậu quả**. Cổng nào không tự chứng minh phép đo của nó bắt được lỗi thì chưa chứng minh gì.

---

### Phụ lục — file để tra sâu

| Cần | File |
|---|---|
| Lý lẽ thiết kế P7 + 2 lượt review (B1 · B2) + 7 câu hỏi Q1–Q7 | `.claude/plan/P7-control-authority.md` (§1 · §10 · §10b) |
| Lý lẽ thiết kế P8 + bảng policy + 4 quyết định coordinator R1–R5 | `.claude/plan/P8-safety.md` (§0b · §1 · §3 · §8) |
| Lý lẽ thiết kế P9 + 8 chữ ký duyệt + bảng thi công P9.0–P9.10 | `.claude/plan/P9-mission.md` |
| **5 điều PHẢI biết trước khi sửa trọng tài** | `src/uav_control_authority/README.md` §2 |
| Taxonomy 3 hành động + hợp đồng đo lường của safety | `src/uav_safety/README.md` |
| **Guard 2 tầng · broker · B1/B2/Y6 · 5 lệch có chủ đích** | `src/uav_mission/README.md` |
| Hợp đồng arbitration (a→h) · giám sát an toàn · điều phối mission | `docs/interface-contract-v0.1.md` §2.16 · §2.18 · §2.19 |
| Trạng thái + bẫy + nợ mở từng package | `docs/package-status.md` §9 · §10 · §11 |
| Ca ego-motion (1,53 m) + M/N 3-của-5 | `src/uav_perception/README.md` mục `target_tracker_node` |
| Toàn văn R24–R32 + bảng policy P8 + R5 + Y3 | `.claude/memory.md` §1 · §5 |
| Số chốt từng đợt (grep `"đợt 8"`, `"đợt 9"`, `"G-CA2"`, `"G-S3"`) | `.claude/changelog.md` |
| Bẫy công cụ/môi trường (BT.CPP 2 CMake Config, đếm test, …) | `docs/ops-playbook.md` §7 · §15 |
| Buổi giảng trước (P5 perception + P6 navigation) | `docs/lecture-p5-p6.md` |
