# BUỔI GIẢNG R18 — P5 (perception + world model) & P6 (navigation)

> **Ngày:** 2026-08-20 · **Người học:** chủ dự án · **Mục tiêu:** không phải "biết nó chạy", mà **tự làm lại được và tự chẩn đoán được**.
> **Nguồn:** mọi con số trong tài liệu này đều trích từ file trong repo — có ghi địa chỉ. Chỗ nào người soạn **chưa chắc**, ghi rõ là chưa chắc.
> **R0:** P6 là tầng sinh setpoint. Sai ở đây là sai trên đường bay thật. Phần §4 giảng sâu hơn phần khác **có chủ đích**.
>
> *Soạn bởi `project-mentor` (E3, Đ5 của `P6-completion-run.md`). Quy trình R18: đọc §0 trước, tự trả lời, rồi mới đọc tiếp; cuối buổi làm §7.*

---

## 0. TRƯỚC KHI ĐỌC — anh tự trả lời 3 câu này đã

Đừng tra tài liệu. Trả lời bằng trực giác hiện tại, rồi giữ câu trả lời đó lại để đối chiếu ở cuối. Ba câu này là ba chỗ mà **trực giác đúng-nghe của kỹ sư lại cho ra thiết kế sai**, và cả ba đều đã trả giá thật trong dự án:

| # | Câu hỏi | Vì sao hỏi |
|---|---|---|
| **A** | Máy bay bám không kịp setpoint. Anh có hai núm: **nới dây xích** (cho setpoint chạy xa máy bay hơn) hoặc **hạ trần tốc độ**. Chọn cái nào, và **cái bị loại mất gì**? | §2a |
| **B** | Một cổng kiểm chứng **bay thật trong sim** và một cổng **chạy trong RAM không có Gazebo**. Cái nào anh đặt lý lẽ an toàn vào? | §2b |
| **C** | Advisor né vật cản khuyên bay tới điểm `z = −3 m`. Điểm đó **lệch khỏi kế hoạch chỉ 4,15 m**, dưới trần 8 m mà anh đã đặt. Có ai chặn nó không? Ai? | §4b |

Nếu câu C anh trả lời "dây xích sẽ cắn" — thì đây chính là buổi giảng anh cần. Dây xích **không cắn**, vì máy bay bám kịp hoàn hảo trên đường mượt xuống đất.

---

## 1. BỨC TRANH P5 → P6 MỘT MẠCH

### 1.1 Đường dữ liệu thật (tên topic thật, không rút gọn)

```
[uav_sim_gz cầu Gazebo→ROS]  ← lớp mô phỏng, KHÔNG có bản sao trên drone thật
   │ /uav/uav0/perception/front/depth_image (32FC1, mét)
   │ /uav/uav0/perception/front/camera_info
   ▼
┌──────────────────── P5 · uav_perception ────────────────────┐
│ obstacle_extractor_node   depth → cụm liên thông → bbox 3D  │
│      out: /perception/obstacles_local   [frame QUANG HỌC]   │
│ target_tracker_node       KF hằng-vận-tốc + gate + NN assoc │
│      out: /perception/target_track      [frame QUANG HỌC]   │
│ marker_detector_node · camera_health_node · object_detector │
└─────────────────────────────┬───────────────────────────────┘
                              │  + /state/odometry_fused (R4: NGUỒN POSE DUY NHẤT)
                              ▼
┌──────────────── P5.6 · uav_world_model ─────────────────────┐
│ world_model_node   quang học → thân → odom, KÈM ĐỘ BẤT ĐỊNH │
│      out: /world/obstacle_map_local  [frame odom]           │
│           /world/target_state        [frame odom]           │
└─────────────────────────────┬───────────────────────────────┘
                              ▼
┌──────────────────── P6 · uav_navigation ────────────────────┐
│ route_planner_node   Costmap 25×25 m/0,25 m + A* + tighten  │
│      in : /world/obstacle_map_local, /planning/route_goal   │
│      out: /planning/route (Path3D)              ── 5 Hz     │
│                                                             │
│ navigator_action_server_node   ← 7 action, CẦM LÁI DUY NHẤT │
│      hỏi route → dựng B-spline TRÊN waypoint route          │
│      out: /planning/trajectory (Trajectory3D, LATCHED)      │
│      out: /control/command_selected (ControlCommand) 20 Hz  │
│                                                             │
│ local_planner_node   ← ADVISOR, chỉ KHUYÊN, không lái       │
│      in : /planning/trajectory + /world/obstacle_map_local  │
│      out: /planning/avoidance (AvoidanceAdvice) ── 10 Hz    │
│                        └────────► navigator QUYẾT ĐỊNH      │
└─────────────────────────────┬───────────────────────────────┘
                              ▼
       px4_command_gateway_node → /fmu/in/* → PX4 → Gazebo
```

### 1.2 Vì sao mỗi khâu tồn tại — mỗi khâu 1–2 câu

| Khâu | Vì sao có nó | Nếu bỏ thì hỏng thế nào |
|---|---|---|
| `obstacle_extractor` ở frame **quang học** | Nó **không** sở hữu pose, nên không được quyền đổi frame. Đổi frame ở đây là giấu một phép quay vào trong node perception | Sai một hằng số gắn camera thì mọi vật cản lệch đúng hằng số đó, và không ai biết địa chỉ để sửa |
| `world_model` là chỗ **duy nhất** đổi frame | Chuỗi transform thuộc về node sở hữu pose. Và đây là chỗ **cộng bất định**: `σ = √(σ_perception² + σ_định_vị² + σ_ghép_cặp² + σ_tuổi²)` | Bản đồ trông chính xác trong khi pose đã sai sẵn **0,3–0,4 m** (`P5-perception.md` §0.2, cổng G2) |
| `world_model` **ngừng phát** `obstacle_map_local` khi mất nguồn | Bản đồ vật cản **rỗng** đọc thành *"không có gì phải né"* | Phía sau tin một lời trấn an sai. Landmark thì ngược lại — vẫn phát mảng rỗng, vì bản đồ landmark rỗng không phải lời tuyên bố an toàn |
| `route_planner` (A* toàn cục) | Né phản ứng đơn độc **kẹt local-minima** — đó là nguyên văn cảnh báo cuối skill `motion-planning-avoidance` | Goal sau bức tường ⇒ escape/hold luân phiên tới hết 240 s |
| navigator dựng B-spline **trên waypoint route** | Waypoint là hình học; máy bay cần **quỹ đạo có thời gian**. Và spline phải do chính navigator dựng vì nó là publisher duy nhất | Hai nơi cùng quyết hình dạng đường bay |
| `local_planner` là **advisor**, không lái | Giữ nguyên máy dây-xích / stall / cổng σ đã qua G-N1 8/8 và G-N2 15/15. Đúng nếp `localization_health`: *đưa bằng chứng, không tự chữa* | Phải refactor đúng cái lõi vừa qua hai cổng — đổi rủi ro đã kiểm lấy rủi ro chưa kiểm |
| navigator là nơi **duy nhất** phát setpoint | Vòng dây xích cần **pose đo được**, nên nó phải ở cùng executor với bộ phát setpoint | `/planning/trajectory` latched + hai writer ⇒ node vào muộn nhận **hai** "kế hoạch hiện hành" mâu thuẫn |

### 1.3 Nhánh advice — đọc kỹ chiều mũi tên

```
/world/obstacle_map_local ─┐
/state/odometry_fused ─────┼──► local_planner_node ──► /planning/avoidance
/planning/trajectory ──────┘         (costmap riêng)          │
        ▲                                                     │
        └──────────── navigator publish ◄─────────────────────┘
                     (VÒNG KÍN: navigator nuôi advisor bằng chính kế hoạch của nó)
```

🔑 **Đây là chỗ dễ hiểu sai nhất của cả P6.** Advisor **không soi máy bay**, nó soi **kế hoạch**. Nghĩa là:
- Không có kế hoạch trên dây ⇒ advisor phát `checked_horizon_m = 0` ⇒ với `require_obstacle_feed=true` là **HOLD vĩnh viễn**.
- Đó là **vòng tự khoá R-C2**, và nó vào được từ **ba cửa** khác nhau (§2d).

**Bảng chính sách 5 dòng** (`src/uav_navigation/README.md` §6f) — mỗi dòng có ít nhất một test làm nó cắn:

| Advisor nói | Navigator làm |
|---|---|
| `CLEAR` **và** `checked_horizon_m > 0` | bay tiếp; đang hold thì nhả hold |
| `ESCAPE` khi đang bay **quỹ đạo** | dựng lại quỹ đạo qua điểm escape → ghép đuôi tuyến → goal |
| `ESCAPE` khi đang bay **carrot** | **HOLD + WARN**, KHÔNG bay escape |
| `HOLD` | **tạm dừng** setpoint *và* đồng hồ kế hoạch tại chỗ |
| im lặng > `advice_timeout_sec` (1,0 s) | map bắt buộc → **HOLD** · không bắt buộc → bay tiếp + `"flying unguarded: …"` |
| 🔴 `checked_horizon_m == 0` | **KHÔNG phải giấy phép bay** — không làm mới đồng hồ permission ⇒ tự rơi vào nhánh im lặng |

---

## 2. NĂM BÀI TOÁN THEN CHỐT

Mỗi bài: **vấn đề → phương án đã LOẠI và vì sao → phương án chọn → con số chứng minh**. Phần "đã loại" mới là phần đắt.

---

### 2a. Dây xích vs trễ bám (leash-vs-lag) — bài toán nền của cả P6

#### Vấn đề

`max_speed` = 1,5 m/s và `max_lead_horizontal_m` = 0,80 m được đặt **riêng rẽ, chưa bao giờ đối chiếu nhau**. Nhưng vòng vị trí của PX4 là **bộ P** với `MPC_XY_P` = **0,95** (airframe của ta không override — đã kiểm), nên trễ bám xác lập là:

```
lag = v / K_p
```

Ở 1,5 m/s: cần **1,58 m** lead trên ngân sách **0,80 m** ⇒ dây xích kẹp **suốt chuyến bay lành**.

#### 🔴 Cái giá thật KHÔNG phải tốc độ

Dây xích tồn tại để **báo lỗi**: *"máy bay không theo kịp"*. Đo được ở G-N2 lần 1 (`docs/package-status.md` §8):

| Đo trên `uav0_nav`, RTF 0,997 | Số | Ngân sách |
|---|---|---|
| Goto ngang ở 1,0 m/s: lead | **0,75–0,79 m** | 0,80 |
| → tốc độ máy bay thật | **0,67–0,74 m/s** | setpoint đòi 1,00 |
| → **tỉ lệ tick bị kẹp** | **8–30%** | |
| Takeoff dọc ở 1,0 m/s: **tỉ lệ tick kẹp** | **79–89%** | |

Khi kẹp là **trạng thái bình thường**, dây xích thôi làm **bộ báo lỗi** và chỉ còn là **bộ hạn tốc câm**. Takeoff 89% kẹp chỉ không bắn FAULT nhờ những tick không-kẹp xen giữa (đồng hồ stall đòi kẹp **liên tục** 3 s) — **biên mỏng**.

#### Cây quyết định

| Phương án | Số | Kết quả |
|---|---|---|
| ❌ **Nới `max_lead`** cho khớp trễ bám | phải lên **~2,05 m** | Làm yếu lá chắn *"setpoint chạy xa khỏi máy bay đang hỏng"* **2,5 lần**. Lá chắn này là thứ duy nhất chặn cú lao lúc bắt kịp |
| ❌ **Tăng `MPC_XY_P`** | — | Đó là **tuning của flight controller**, phải hiệu chỉnh lại theo từng airframe thật. Chưa có drone thật thì không lấy làm đòn bẩy chính. **Vẫn hợp lệ ở P11** |
| ✅ **Hạ trần tốc độ** | `max_speed` 1,5 → **0,55** · `max_vertical_speed` 1,0 → **0,45** | Dây xích trở lại làm bộ báo lỗi |

#### Bất đẳng thức được ghi thẳng vào yaml

`src/uav_navigation/config/navigation_params.yaml:8`:

```
max_speed  <=  K_p * max_lead / 1.3          # K_p: 0,95 ngang · 1,0 dọc
0,55       <=  0,95 * 0,80 / 1,3 = 0,584     ✓ headroom 1,38×
0,45       <=  1,00 * 0,60 / 1,3 = 0,462     ✓ headroom 1,33×
```

> 🔑 **Trần tốc độ bền vững ≈ K_p × max_lead.** Với cấu hình hiện tại đó là `0,95 × 0,80 = 0,76 m/s` — **không phải** `max_speed`. Hệ số 1,3 là biên cho jitter và gió.

#### Cái giá, đo được — và tại sao chủ dự án chấp nhận trả

| Test (cặp đi liền, `test_navigator_action_server_node.cpp`) | Khẳng định | Đo được |
|---|---|---|
| `TheShippedSpeedStaysInsideWhatTheAircraftCanTrack` | trần đã ship, plant `K_p=0,95` ⇒ message **không** chứa `"clamped"` | chặng 3 m: **13,4 s** |
| `TheOldSpeedCeilingRidesTheLeashTheWholeWay` | trần cũ 1,5 ⇒ message **phải** chứa `"clamped"` | chặng 3 m: **8,3 s** |

Đổi **+38% thời gian** lấy **một bộ báo lỗi còn hoạt động**.

🔴 **Không có test thứ hai thì test thứ nhất chỉ chứng minh rằng KHÔNG CÓ GÌ CHẠY CẢ.** Đây là khuôn mẫu "đối chứng dương" mà anh sẽ gặp lại 4 lần nữa trong buổi này.

#### Hệ quả dây chuyền phải nhớ

Điểm mút nhân ba của B-spline làm tốc độ TB chỉ ≈ **0,44 × trần** ⇒ `goto_timeout_sec` 120 s chỉ với tới **29 m** ⇒ phải nâng lên **240 s**. Node còn kẹp `max_duration = 0,8 × goto_timeout` = 192 s ⇒ **tầm với một Goto ≈ 46 m**, vẫn dư cho costmap 25×25 m.

> **Đây là lần thứ nhất của một khuôn lỗi lặp 4 lần trong dự án: hai hằng số độc lập mã hoá hai giả định mâu thuẫn.** Ba lần sau: `max_escape_deviation_m` vs tầm xoắn ốc · `max_home_distance` vs `max_duration` · ba tham số `*_timeout_sec` cùng mã hoá một ngưỡng tuổi (G6).

---

### 2b. Hai tầng cổng né — vì sao KHÔNG tin cổng bay

#### Vấn đề

Cổng G-N4 kiểm "né vật cản có hoạt động không". Đặt nó ở đâu?

#### Giả định đã bị BÁC bằng thực nghiệm

Giả định tự nhiên: *"RTF thấp là do máy bận, chạy lại trên máy rảnh sẽ đạt"*. Chạy `rerun_p4_gates.sh` trên WSL **vừa boot, 16 nhân, 21/23 GB trống, không việc nào khác** (`P6-navigation.md` §2g):

| Bài | RTF |
|---|---|
| Indoor M5 (3 chuyến) | mean **0,918** (min 0,203 · p50 0,999 · max 2,091) |
| G2 điều kiện A | **0,828** |
| G2 điều kiện B | **0,877** |

Quy tắc dự án: **chỉ tin số đo bay khi RTF ≥ 0,95**. Cả hai điều kiện G2 tự trả `NOT TRUSTED`.

⇒ **Máy rảnh không cứu được.** Chi phí là của **cả stack khi bay** (`uav0_nav` đậu 1,000 nhưng dưới 11 node lúc bay chỉ còn 0,906–0,953), không phải của tải ngoài.

⚠️ **Đọc `mean`, không đọc `p50`** — phân bố RTF ở đây lưỡng đỉnh (phần lớn 1,0, xen những cú khựng sâu) nên trung vị nhảy loạn giữa các lần chạy trên **cùng một cấu hình** (`P5-perception.md` §0.1).

#### Ba lần trượt liên tiếp

Cổng G2 (định vị) trượt **lần thứ ba liên tiếp** vì RTF, trong khi cùng ngày cổng **tất định** `run_mux_fidelity.sh` **đo được, ra kết luận, và còn bắt được một lỗi thật**. Hai bằng chứng cùng ngày chỉ về một phía.

#### Quyết định: chia hai tầng

| Tầng | Nội dung | Bị chặn RTF? | Vai |
|---|---|---|---|
| **G-N4a — TẤT ĐỊNH** | Harness bơm `ObstacleArray` + `odometry_fused` tổng hợp, **3 node thật cùng tiến trình**, plant `K_p=0,95`, không Gazebo/PX4 | ❌ Không | 🔴 **NƠI ĐẶT LÝ LẼ AN TOÀN** |
| **G-N4b — BAY** | `uav0_track`, hộp chắn đường Goto, khoảng cách ground-truth | ✅ Có | **Xác nhận + dán nhãn**, không cấp chứng nhận |

#### Kết quả G-N4a — ba con số trước đây chỉ là phỏng đoán, nay ĐO ĐƯỢC

`test_avoidance_chain.cpp`, **217 case / 11 target**:

| Đo | Số | Đối chiếu |
|---|---|---|
| **Trễ chặng advisor** | **85,2 ms** (map→advice 69,9 + advice→navigator 15,3) | ⇒ **0,047 m** ở 0,55 m/s, so với đệm clearance **0,591 m** |
| Gia tốc qua điểm dựng lại quỹ đạo | **0,227 m/s²** | không có bậc vận tốc |
| Hở nhỏ nhất **đo từ setpoint đã phát** | **0,941 m** | |

> Con số *"~50–100 ms"* dùng lúc chọn kiến trúc là **phỏng đoán**. Kiến trúc advisor được duyệt kèm điều kiện: **phải trả bằng số đo**. Đây là số đo đó.

#### Kết quả G-N4b (hôm nay, lượt 4) — dán nhãn

`scripts/run_gn4b.sh` · **PASS**, RTF **0,984** (đủ cao nên **không cần nhãn**):

| Đo | Số |
|---|---|
| Hở ground-truth tới hộp, nhỏ nhất | **1,234 m** / 2152 mẫu |
| Không hạ thấp | min z **2,387** ≥ sàn 2,223 |
| Gia tốc qua **2 lần** dựng lại quỹ đạo | **2,23 m/s²** ≤ trần 3,0 |
| Kết quả | `"route rejoined; 2 avoidance escapes flown"` |

🪤 **Bẫy phát hiện trong chính cổng này:** `message-count(1) ≠ sequence-count(2)` — chuỗi bằng chứng chỉ ghi escape **đầu tiên**; phải có **tally** `"N escapes flown"` mới đủ. Probe nay in cả hai.

#### 🔑 Bài học phương pháp

> **Cổng bay không phải là cổng an toàn — nó là cổng hồi quy.** Nó trả lời *"hôm nay có gì gãy so với hôm qua không"*. Câu *"hệ này an toàn"* phải được trả lời ở nơi phép đo không bị nhiễu bởi tài nguyên máy.
>
> **Hệ quả cho anh:** khi thêm một cổng mới, hỏi trước — *"cổng này có thể FAILED TO MEASURE vì máy chậm không?"* Nếu có, nó **không được** là nơi duy nhất giữ một lý lẽ an toàn.

---

### 2c. Vì sao 2.5D chặn trần leo — bài học "guard xanh mà rỗng"

#### Nền: costmap là một LÁT CẮT DẸT

Quyết định P6 số 3 (chủ dự án chốt 2026-08-16): né **2.5D** — A* trên mặt phẳng ngang tại cao độ bay + quy tắc dọc.

Cụ thể trong code: `Costmap::costAt()` tra ô theo **(x, y), BỎ QUA z**. Vật cản chỉ được nạp vào bản đồ nếu nằm trong lát cắt **`±flight_band_m`** (mặc định **1,0 m**) quanh cao độ hiện tại.

⇒ **Thành phần z của điểm escape KHÔNG HỀ ĐƯỢC KIỂM bởi bản đồ.**

#### Luật thứ nhất viết sai: trần leo

| | |
|---|---|
| Bản plan viết | `\|Δz\| ≤ 2,0 m` — một hằng số |
| **Vấn đề** | 2,0 m là **gấp đôi** thứ bản đồ bảo chứng được (băng ±1,0 m). Leo quá băng là leo vào **vùng không có dữ liệu** |
| Bản đang chạy | `climb_cap = min(max_escape_climb_m, map.flightBand())` |

**Đo trên cùng một lời gọi** (`test_local_avoidance.cpp:271-275`):

| | Escape leo |
|---|---|
| **Có** băng trong cap | **0,423 m** |
| **Bỏ** băng ra | **1,759 m** ← ra ngoài vùng dữ liệu |

Với `growth ≥ 2,5`: có băng → **Hold** (đúng: không tìm được đường an toàn trong dữ liệu có thật); bỏ băng → Escape leo 1,06–1,69 m (sai: bay vào chỗ mù).

#### Luật thứ hai viết sai: cấm né bằng hạ thấp

| | |
|---|---|
| Bản đầu | `candidate.z < position.z` (so với **máy bay**) |
| **Vấn đề đo được** | Trong một lát cắt 2.5D **luôn có chỗ leo bên hông vật cản**, nên xoắn ốc chỉ cần nở thêm một vòng là tìm được ứng viên cao hơn máy bay ⇒ vị ngữ đó **gần như không bao giờ cắn** |
| Bản đang chạy | `candidate.z < obstacle.z` — **cấm chui xuống dưới chính thứ đang tránh** |

#### 🪤 Bài học phương pháp đắt nhất của ngày 2026-08-19

**Ba lá chắn an toàn (`climb`, `descent`, `horizon`) đều XANH MÀ RỖNG ở bản test đầu** — ứng viên xoắn ốc **đầu tiên** đã thoả hết nên guard chưa từng cắn. Chỉ **mutation** mới lộ ra.

Và người viết đã **đoán sai hình học ba lần liên tiếp** khi cố dựng ca cho guard cắn:
- mẫu chặn nằm ở **vành đai nở**, không ở tâm vật cản;
- vành đai còn bị **lượng tử hoá theo ô 0,25 m**.

Chỉ khi **quét `growth` bằng probe và ĐỌC SỐ** mới tìm được điểm mà guard là thứ **duy nhất** chặn.

> **Một guard chưa từng cắn trong test nào là một guard chưa được kiểm chứng — và hình học của nó phải ĐO, không SUY.**

#### Thú nhận trung thực nằm trong tài liệu

Mệnh đề `cost != kCostUnknown` trong `blocked()` là **phòng thủ chiều sâu KHÔNG caller nào với tới** (cả `advise()` lẫn `segmentClear()` đều lọc 255 trước khi gọi). Mutation biến nó thành "255 là tường" **không làm đỏ test nào**.

→ **Giữ lại** (gỡ một chốt an toàn khỏi đường bay để "cho sạch" là đánh đổi sai với R0), nhưng ghi rõ **chưa được chứng minh**. Cùng nếp vòng quét NaN của lõi quỹ đạo.

---

### 2d. Carrot-publish — lỗ "advisor mù với hành động carrot"

#### Vấn đề lộ ra ở đâu

Khi làm B3 (`TrackTarget`, 2026-08-19), phát hiện này quan trọng hơn cả B3:

> 🔑 **Hành động cần né vật cản NHẤT lại là hành động advisor KHÔNG NHÌN THẤY.**

Cơ chế:

```
TrackTarget buộc bay carrot (mục tiêu di chuyển liên tục, dựng lại spline
mỗi nhịp là thrash + mỗi lần khởi hành từ NGHỈ = bậc vận tốc)
        ↓
không publish /planning/trajectory
        ↓
advisor phát checked_horizon_m = 0 mãi mãi
        ↓
require_obstacle_feed = true  ⇒  HOLD tới hết 240 s rồi ABORT, lặp cho MỌI goal
```

#### Vòng tự khoá R-C2 vào từ BA cửa — và cả ba đều với tới được

| Cửa | Điều kiện | Có cần cấu hình sai không? |
|---|---|---|
| 1 | `use_trajectory=false` (rollback) | Có — nhưng là rollback **có tài liệu** |
| 2 | `Trajectory::build` **thất bại giữa chuyến** (tuyến dài hơn `max_duration`) | 🔴 **KHÔNG** — chỉ cần bay xa |
| 3 | Mọi hành động bay carrot: `TrackTarget` · `Takeoff` · `Land` · `HoldPosition` · `Recover` rơi về carrot | 🔴 **KHÔNG** |

#### Ba xác nhận độc lập — hiếm, và đáng ghi

R-C2 được tìm ra bởi **ba nguồn không biết nhau**:

1. **Reviewer** tìm ra bằng **đọc code**.
2. **Agent dựng cổng G-N4a** đâm phải bằng **thực nghiệm** (`goto_timeout_sec=20` ⇒ build fail ⇒ carrot ⇒ advisor không soi) — lúc đó nó chỉ nới timeout, **chưa biết vừa gặp một lỗi kiến trúc**.
3. **Chính hai test của agent sửa Lane R** đang nằm trong bẫy: `NavigatorAdviceRequiredFixture` để `goto_timeout_sec=8` ⇒ `max_duration` 6,4 s < chặng 3 m (~7 s) ⇒ carrot. Hai test đó **xanh vì lý do một phần SAI**.

⇒ Đã thêm **dây bẫy** `expectTheTrajectoryWasFlown(result.message)` (cấm chuỗi `"carrot fallback"`) vào **5 test** hold/escape: **test phải tự chứng minh nó đi qua đúng đường nó nói.**

#### Cây quyết định cách chữa

| Phương án | Kết luận |
|---|---|
| ❌ **Thêm topic thứ hai** cho "đoạn đang bay" | Thêm một interface + một đường dữ liệu phải bảo trì. Và contract §2.14 đã chốt `Trajectory3D` có **đúng một** publisher — thêm topic là né luật chứ không giải quyết |
| ❌ Sửa advisor để nó soi thêm cái khác | Advisor đang đúng: nó soi **thứ sẽ đi ra dây**. Sửa nó là làm hai nơi cùng biết về hình dạng đường bay |
| ✅ **Bay carrot thì PUBLISH đúng thứ đang bay** (chủ dự án chốt 2026-08-19) | Một `Trajectory3D` **2 điểm**: `setpoint đang lệnh → đích carrot`. Advisor soi được ngay, **không đổi một dòng nào ở advisor** |

#### Hợp đồng của đoạn 2 điểm (`README` §6h)

| Trường | Giá trị | Vì sao |
|---|---|---|
| `plan_state` | `VALID` | Đây là kế hoạch đang có hiệu lực **thật** |
| `sequence` | tăng mỗi lần phát | Consumer so được hai bản kế tiếp |
| `time_from_start` | `0` và `max(ngang/max_speed, dọc/max_vertical_speed)` | **Đúng công thức `advanceCarrot`** — carrot kẹp hai trục bằng hai trần riêng |
| `velocity` cuối | **0** | Carrot chạy đều rồi **dừng hẳn**. Ghi khác 0 là nói dối rằng nó bay tiếp |
| Không có đích | `NO_GOAL` + `"holding the setpoint, no leg to check"` | Phát đoạn dài 0 m là mời advisor báo *"đã kiểm 0 m"* — đúng cái bẫy `checked_horizon_m=0` |

⚠️ **Đoạn 2 điểm là ĐƯỜNG THẲNG, carrot thật thì không** (hai trục hai trần ⇒ lên xong trước rồi mới đi ngang). Sai khác này **không ảnh hưởng thứ advisor kiểm** (`costAt()` bỏ qua z) — nhưng **đừng dùng đoạn này để dự đoán z(t)**.

#### 🔴 Nhịp làm tươi — suy từ số đo, không chọn cho đẹp

Đây là ví dụ mẫu của "suy tham số từ đại lượng đã đo":

```
Đo được (G-N4a):  trọn vòng advisor = 85,2 ms
                        ↓
SÀN CỨNG trong code: carrot_plan_period_sec >= 0,2 s   (≈ 2,3× vòng advisor)
                     node TỪ CHỐI KHỞI ĐỘNG nếu thấp hơn
                        ↓
Nhịp ship:           0,5 s  ≈ 6 câu trả lời advisor mỗi lần phát
```

**Vì sao có sàn:** `watchAvoidance` chỉ nhận advice có `stamp > plan_installed_`. **Phát nhanh hơn advisor trả lời thì MỌI câu trả lời đều nói về chặng vừa bị thay** ⇒ navigator đọc thành **im lặng** ⇒ `require_obstacle_feed=true` là **HOLD vĩnh viễn**.

> Đây là cùng một bệnh với R-C2, chỉ khác là nó **tự gây ra bởi bản vá cho R-C2**. Bài học: mỗi cơ chế mới đẩy tin lên dây đều phải hỏi *"nhịp này có làm phía nhận đọc thành im lặng không?"*

Hai chốt nữa trong bảng cùng mục:
- 🔴 **Trần cứng tối đa 1 message/period**, kể cả khi đổi qua lại giữa "có chặng" và "đứng yên" — nếu chỉ chặn phía SEGMENT thì một đoạn nhấp nháy quanh ngưỡng 0,05 m sẽ phát **mỗi tick** lên một topic **reliable + latched**.
- 🔴 **`advice_permission_` KHÔNG được làm mới** khi phát chặng mới. Nếu làm mới, một mục tiêu di chuyển sẽ **tự tay che một advisor đã chết** bằng chính nhịp phát của mình.

#### Kết quả

✅ `TrackTarget` **chạy được** với `require_obstacle_feed=true` (trước đó bị từ chối ngay ở cổng nhận goal ⇒ **cấu hình bay thật không bám được mục tiêu**).
🟠 **Còn mở:** `ESCAPE` khi bay carrot vẫn chỉ **HOLD**, chưa bay được escape — escape gắn với máy dựng lại quỹ đạo. Với `TrackTarget` giữa vật cản: máy bay **dừng** thay vì đi vòng, mục tiêu chạy mất. Mở nó cần một chặng carrot đi qua điểm escape rồi mới tới đích — **việc sau, chưa chốt**.

---

### 2e. `Recover` — ba loại, và vì sao KHÔNG có `LAND`

#### Quyết định Đ3 (chủ dự án chốt 2026-08-19)

| Loại | Node làm | Kết thúc khi |
|---|---|---|
| `TYPE_HOLD` | Đóng băng setpoint tại chỗ | 🔴 **Bằng chứng dòng lệnh** (xem dưới) |
| `TYPE_CLIMB_TO_SAFE_ALTITUDE` | Bay thẳng lên `safe_altitude` (**tuyệt đối trong `odom`**) | Tới nơi |
| `TYPE_RETURN_HOME` | Leo lên `safe_altitude` rồi bay ngang về **trên đầu** điểm cất cánh | Tới nơi, **hover ở đó** |
| 🔴 `TYPE_LAND` · `TYPE_HANDOVER_TO_PILOT` · `TYPE_UNKNOWN` | **REJECT ngay ở cổng nhận, kèm lý do** | — |

**Và goal CHỌN — node không bao giờ tự quyết.** P6 chỉ **thực thi**; *quyết khi nào recover và recover kiểu gì* là **P8**.

#### 🔴 Vì sao không có LAND — giáo lý gốc

`docs/interface-contract-v0.1.md` §2.5, nguyên văn: *"Đây là điểm sát sinh mạng nhất của hợp đồng này."*

> Phản xạ tự nhiên khi mất định vị là cho **hạ cánh tự động**. Nhưng **khi đã mất định vị thì phương tiện không biết mình đang ở đâu** — "hạ cánh tự động" có thể hạ trúng người hoặc chướng ngại vật.

Bằng chứng thực nghiệm trong tài liệu tham khảo của dự án nghiêng về **trả quyền cho người lái**. Điều này **mâu thuẫn với `CLAUDE.md` §4.8** (đang ghi "mất localization → land") — và mâu thuẫn đó **được ghi ra**, không bị code lặng lẽ giải quyết một chiều.

**Quyết định:** P6 **không hiện thực hoá** `LAND`. Quyết định *hạ cánh* hay *trả quyền pilot* thuộc tầng safety/authority (P7–P8), **không phải tầng chuyển động**.

⚠️ Lý do từ chối chỉ ra **log** (goal bị REJECT thì ROS action không mang message) — nên **mỗi loại cấm có một test riêng** ghim đúng sự từ chối đó.

#### 🔑 Kiểm LOẠI trước, kiểm TRẠNG THÁI sau — có chủ đích

Nếu kiểm trạng thái trước, một `Recover(LAND)` gửi lúc còn dưới đất sẽ bị trả lời *"chưa bay"* — **một lý do TẠM THỜI che mất một lý do VĨNH VIỄN**, và bất kỳ cổng nào kiểm sự từ chối đó lúc drone chưa cất cánh sẽ **xanh vì lý do sai**.

#### Ba ràng buộc mang trọng lượng an toàn (lib `recovery_planner`, ROS-free, 28 test)

1. **Không bao giờ bịa `home`.** `(0,0,0)` trong `odom` chính là chỗ cất cánh — **một toạ độ trông rất hợp lệ**. Node chỉ biết home nếu **chính nó đã cất cánh**; chưa có thì `RETURN_HOME` bị từ chối đích danh.
2. **Không bao giờ hạ xuống.** `RETURN_HOME` kết thúc **hover trên đầu** home, cách ít nhất `min_home_clearance_m` (1,0 m). *Một return kết thúc AT home là một cú hạ cánh.*
3. **Từ chối chứ không kẹp, và không bao giờ đổi loại.** Kế hoạch không bay được như yêu cầu ⇒ `ABORTED_INVALID_GOAL` kèm lý do; **đổi sang HOLD là quyết định của người gọi**.

#### Preemption — `Recover` là action DUY NHẤT được preempt

| Điều | Cách làm |
|---|---|
| Task cũ | `LoopOutcome::PREEMPTED` → `ABORTED_SAFETY`, message *"preempted by a recovery goal"* |
| Setpoint | Recovery neo vào `motionAnchor()` như mọi task ⇒ **liên tục**, stream **không đứt** |
| Khe hở "nhận recovery ↔ task cũ dừng" | Đóng bằng `recovery_pending_`: trong khe đó **mọi goal khác bị từ chối**, nếu không một Goto lọt vào đúng lúc `state_` vừa về `HOLDING` |
| Đường bay | Đi qua **chính lõi quỹ đạo thường** (`startGotoMotion`) — cùng spline, cùng dây xích, cùng publisher duy nhất. 🔴 **Không mở đường tắt thứ hai vào bộ phát setpoint** |

**Đo được ở cổng D4 (`run_gn6.sh`) hôm nay, PASS 23/23 lượt 4:**

| Đo | Số |
|---|---|
| Liên tục setpoint qua tiếp quản | **0,0255–0,0256 m** ≤ trần **0,0825 m** |
| Stream trong suốt 3 chuyến | min **19,93–20,00 Hz**, armed + offboard suốt |
| CLIMB | **4,135 / 4,241 m**, không tụt |
| RETURN_HOME | hội tụ **1,619 m** (trần 2,0) |
| TYPE_LAND | bị từ chối **đúng cửa** |

#### 🔴 Chốt hôm nay (E2-C2): `TYPE_HOLD` phải SỐNG SÓT qua chính cái nó được gọi để chữa

Đây là mục sát an toàn nhất của cả lượt review hôm nay. Đọc chậm:

**Quan sát:** `TYPE_HOLD` là loại khôi phục **DUY NHẤT không đọc pose**. Nó chỉ đóng băng setpoint.

**Lan can cũ:** navigator có cổng độ tin cậy định vị (§6b) + hai phép kiểm định vị trong `checkFaults()`. Chúng áp cho **mọi** task.

**Hậu quả nếu giữ nguyên:** trong **đúng ca lan can này** — mất định vị — `Recover(TYPE_HOLD)` chết trong **≤ 1 s** với `ABORTED_LOST_LOCALIZATION`.

```
Mất định vị
   ↓
P8 gọi Recover(HOLD)  ← nút leo thang
   ↓
HOLD abort sau ≤1 s với ABORTED_LOST_LOCALIZATION
   ↓
P8 đọc: "khôi phục THẤT BẠI"
   ↓
đường leo thang còn lại: chỉ còn LAND / HANDOVER
   ↓
🔴 NGƯỢC ĐÚNG GIÁO LÝ contract §2.5
```

**Bản vá — ba việc, chỉ cho riêng `TYPE_HOLD`:**
(a) **không** chặn ở cổng độ tin cậy định vị · (b) **tắt hai phép kiểm định vị** trong `checkFaults()` · (c) kết thúc theo **bằng chứng dòng lệnh** — *setpoint đứng yên* **và** *stream đã chảy đủ `arrival_settle_sec`* — **KHÔNG** đo `khoảng cách(pose, điểm giữ)`.

⚠️ **Nó BIẾT pose đã bị disown và vẫn giữ** — bằng chứng đó nằm trong `result.message`, **không giấu**.
`CLIMB`/`RETURN_HOME` giữ **nguyên mọi** phép kiểm — chúng **có** đọc pose.

**Test:** `AHoldRecoverySurvivesThePoseItCannotRead`, kèm **đối chứng dương**: cùng lúc đó một `GotoPose` **bị từ chối** vì chính pose ấy.

🟡 **Giới hạn còn lại, ghi ra chứ không giấu:** lan can đó chỉ che **định vị**, không che **mất quyền điều khiển**. `Recover(TYPE_HOLD)` vẫn abort khi offboard rớt / autopilot cầm lái / failsafe — **đúng** (không giành quyền), nhưng nghĩa là hover-khôi-phục **không** phải phương án cuối cùng cho *mọi* lỗi, chỉ cho lỗi định vị.

#### Vì sao lan can cũ KHÔNG BAO GIỜ CẮN — và đó là điều tệ hơn

Trước bản vá, cổng σ **có mặt** nhưng **không phát huy tác dụng gì có ích**: nó chỉ giết đúng một hành động vốn không cần pose. Guard vẫn "chạy", vẫn "xanh", nhưng cái nó chặn lại là **thứ duy nhất còn hoạt động được** ở đúng thời điểm mọi thứ khác đã hỏng.

> 🔑 **Một guard có thể sai theo hai chiều: không bao giờ cắn (§2c), hoặc cắn đúng thứ đáng lẽ phải được sống (§2e).** Chiều thứ hai khó thấy hơn nhiều vì hệ thống trông có vẻ "cẩn thận".

#### Số đo từ D3 hôm nay (`run_gn5.sh`, PASS 12/12 lượt 2)

| Đo | Số |
|---|---|
| FollowPath | **4/4** waypoint đúng thứ tự |
| completion% **arc-length** xác nhận trong bay | checkpoint **18,2%** đúng công thức (kiểu chỉ-số sẽ ra 25/33%) |
| TrackTarget standoff hội tụ | **4,088** vs 4,00 m (sai **0,094** ≤ 0,45) ⇒ **Đ4 đúng trong bay** |
| 2.5D giữ z khi target ở đất | lệch **0,046 m** |
| Mất target | `ABORTED_LOST_TARGET` đúng mã (không phải timeout) |
| 🟠 **NỢ 13 ĐO LẦN ĐẦU** | **trễ mũi TrackTarget đỉnh 93,7°** / 800 mẫu (orbit chậm 0,15 rad/s) — **chưa có ngưỡng**, số để dành P9 |

---

## 3. HAI BÀI HỌC CHẨN ĐOÁN ĐẮT NHẤT HÔM NAY

Phần này giảng **phương pháp**, không giảng kết quả. Kết quả chỉ đúng cho một ca; phương pháp dùng lại được.

---

### 3a. `uav0_track` không bay được qua navigator — nhưng M5 smoke bay tốt

#### Triệu chứng quan sát được

```
Navigator: engage offboard OK → arm OK → carrot leo …
           position_z ĐỨNG IM
PX4 log:   "Armed"  →  10 s sau  →  "auto preflight disarming"
```

Và điều làm nó khó: **M5 smoke test bay PASS 3/3 trên CÙNG model đó.**

#### Chuỗi loại trừ — mỗi bước loại được cái gì

| # | Hành động | Quan sát | 🔪 Loại trừ được |
|---|---|---|---|
| 1 | Nghi **dây xích** (bệnh quen của P6) | Log: setpoint bị kẹp, `position_z` đứng im | ❌ **Không loại được gì.** Dây xích kẹp vì máy bay không lên — nó là **triệu chứng**, không phải nguyên nhân. *Bẫy đầu tiên: bệnh quen thuộc nhất là bệnh dễ đổ oan nhất* |
| 2 | **Thẩm vấn PX4 SỐNG** (không đọc log ROS) | `arming_state` = 1 · offboard **active** · setpoint về **20 Hz** | ✅ Loại: nhịp stream · chuỗi engage offboard · lệnh arm · **toàn bộ phía ROS** |
| 3 | Đọc **log PX4** | `"Armed"` rồi `"auto preflight disarming"` | ✅ Loại: đây là **PX4 tự quyết**. Nghi phạm chuyển sang **bộ ước lượng / land detector** |
| 4 | **Đối chứng: M5 cùng model** | PASS 3/3 | ✅ Loại: model SDF · world · cầu Gazebo · vật lý. Sai khác nằm ở **cách ra lệnh** hoặc **cấu hình PX4** |
| 5 | Vì sao M5 thoát mà navigator không? | M5 ra **BƯỚC setpoint 2,5 m** · navigator **ramp** bị xích kẹp ở **0,58 m** | 🔑 Lệnh navigator quá "hiền" để phá land detector đang latch. *Vì sao latch* thì chưa biết |
| 6 | **`diff` airframe `4102` vs `4103`** | 4102 có **profile vision-primary**, 4103 **KHÔNG** | 🎯 **DỮ KIỆN CHỐT HẠ** |

#### Cơ chế đầy đủ

```
4103 thiếu  EKF2_GPS_CTRL 8  +  EKF2_HGT_REF 3
        ↓
EKF chạy GPS-primary … trong khi px4_external_odometry_node VẪN bơm vision song song
        ↓
hai khung quy chiếu cãi nhau trong bộ ước lượng
        ↓
"vertical velocity unstable" / "GPS drift too high"
        ↓
land detector GIỮ CHẶT
        ↓
navigator ramp (bị xích kẹp 0,58 m) không đủ mạnh để thoát
M5 (bước 2,5 m) thì thoát  ⇒ đúng cái đã che giấu lỗi suốt
```

#### Bản vá — và nó gọn đúng 2 dòng

`src/uav_sim_gz/airframes/4103_gz_uav0_track`: thêm `param set-default EKF2_GPS_CTRL 8` + `param set-default EKF2_HGT_REF 3` (airframe symlink nên **không cần rebuild**). Xác nhận bằng chính chuyến bay cổng D2 ngay sau đó.

Ý nghĩa (chép từ khối lý lẽ của 4102): `EKF2_GPS_CTRL 8` — chỉ bit yaw: xoá HPOS+VEL để `flags.gps` false, là **điều kiện DUY NHẤT khiến EKF2 reset khung ngang về vision thay vì ước lượng bias**; `EKF2_HGT_REF 3` — điều tương tự cho khung đứng.

#### 🔑 Bài học rút ra — và nó lớn hơn ca này

> **MỘT MODEL SINH RA CHO MỘT BÀI ĐẬU CHƯA CHẮC BAY ĐƯỢC.** `uav0_track` sinh ra cho cổng P5.5 (drone ĐẬU, target di chuyển) — chưa bao giờ cần bay, đến khi P6 dùng cho cổng bay thì lỗ mới lộ.
> **⇒ Quy tắc mới: mọi biến thể model mới phải qua MỘT CHUYẾN M5 trước khi dùng cho bất kỳ cổng bay nào.**

| Bài học phương pháp | Áp dụng chung |
|---|---|
| **Bệnh quen thuộc nhất là bệnh dễ đổ oan nhất** | Dây xích kẹp là *triệu chứng của mọi thứ không bay được*. Đừng dừng ở lớp mình quen |
| **Thẩm vấn hệ thống ĐANG SỐNG, đừng chỉ đọc log của mình** | `arming_state`, `offboard_status`, tần số setpoint — ba số loại sạch phía ROS trong 30 giây |
| **Đối chứng khác biệt nhỏ nhất** | M5-cùng-model PASS thu hẹp nghi phạm từ "cả stack" xuống "cách ra lệnh hoặc cấu hình PX4" bằng một lần chạy |

⚠️ **Một chỗ CHƯA CHẮC:** liệu có phải **chỉ** thiếu 2 param, hay còn tương tác khác giữa GPS-primary và external odometry. Bằng chứng đang có là *bản vá làm nó bay được + cổng D2 PASS* — đủ để đi tiếp, **không** phải chứng minh cơ chế. Muốn chứng minh phải đọc **ulog** (`estimator_status_flags`, `estimator_aid_src_*`) — nhớ: **`ECL_INFO`/`ECL_WARN` của EKF2 KHÔNG ra console** (`memory.md` §5).

---

### 3b. Chuỗi 7 ca test đỏ mà 0 ca phải sửa code sản phẩm

**Số đo:** B4b + carrot-publish, **6 vòng build-test**, **0 dòng code sản phẩm phải sửa sau review**. **7 ca đỏ đầu đều là lỗi phép đo phía test.**

#### Năm khuôn lỗi — nhận diện được thì lần sau tự bắt

| # | Khuôn lỗi | Ca thật | Vì sao test đỏ mà code đúng |
|---|---|---|---|
| **1** | **Cửa sổ đo không tồn tại** | Không có mẫu setpoint nào rơi vào cửa sổ đang đo | Khẳng định tính trên tập rỗng. Nay: metric rỗng phải là **FAILED TO MEASURE**, không được là PASS |
| **2** | **Đo TOÀN TRACE thay vì cửa sổ tiếp quản** | Đo liên tục setpoint qua preemption | Toàn trace phủ cả những pha **không nói gì** về tính liên tục. Nay đo đúng **±1,0 s quanh mốc tiếp quản** — cùng cửa sổ cổng G-N6 |
| **3** | **Test đua với fixture / với CPU** | Khẳng định **chuỗi lý do** của guard X bị **guard anh em cướp lời** (độ tươi route, đồng hồ tường) khi 11 target ctest song song đói CPU | Chữa bằng **GHIM THAM SỐ guard-không-thử ra khỏi tầm** (`route_fresh_sec=60` trong fixture), **KHÔNG nới assertion**. Đối chứng: chạy một mình PASS 2/2 |
| **4** | **Con số vô nghĩa mà lại tính ra được** | Lệch **0,42 m** ở test chặng carrot | `0,42 = 2×standoff − target.x = 2×2,71 − 5,00`: nguồn target giả mặc định phát **(0,0,0)** ngay dưới bụng drone ⇒ ca suy biến `standoffPoint()` ⇒ chặng đầu nhắm **+x tuỳ ý** |
| **5** | **Sai HỆ QUY CHIẾU** | Probe D4 lượt 3 đo HOLD lệch | Máy bay ghé về **SETPOINT ĐÓNG BĂNG**, không đứng ở **vị trí lúc preempt** — hai mốc cách nhau đúng **`v/K_p` ≈ 0,5 m** |

#### 🔑 Bài học trung tâm

> **PHÉP ĐO PHẢI TỰ CHỨNG MINH NÓ ĐO ĐÚNG THỨ NÓ NÓI.** Khi một test đỏ, câu hỏi **đầu tiên** không phải *"lỗi ở đâu trong code sản phẩm?"* mà là: **"Phép đo này có TƯ CÁCH kết luận không?"** — bốn câu con:
>
> | Câu | Bắt khuôn |
> |---|---|
> | Cửa sổ đo có mẫu nào không? | #1 |
> | Cửa sổ đo có đúng là cửa sổ mà khẳng định nói về không? | #2 |
> | Có guard/tiến trình nào khác có thể trả lời thay không? | #3 |
> | Số đó suy từ mốc nào, và **mốc đó có phải mốc mà vật lý dùng**? | #4, #5 |

#### ⚠️ Cảnh báo trung thực về chính bài học này

**7/7 ca đỏ đều "tại test" — bản thân con số đó đáng cảnh giác.** Nếu mọi lần đỏ đều kết luận vậy, ta đang một bước trước văn hoá *"sửa test cho tới khi xanh"*. Ba thứ giữ cho kết luận lương thiện — đòi cả ba mỗi lần:

1. **Mutation** — phá code sản phẩm, test phải đỏ **đúng test**.
2. **Đối chứng dương** trong chính bài đo (carrot **14,21 m/s²** nằm **bên trong** cổng G-N2 — không nổ ⇒ **cả cổng FAIL**).
3. **Dây bẫy chống đi nhầm đường** — `expectTheTrajectoryWasFlown` cấm `"carrot fallback"` trong 5 test hold/escape.

> **Một cổng chưa chứng minh phép đo của nó bắt được lỗi thì chưa chứng minh gì.**

---

## 4. PHẦN SÁT AN TOÀN — GIẢNG SÂU (R0)

Nếu chỉ nhớ được ba điều từ cả buổi thì nhớ ba mục này.

### 4a. Mọi số an toàn phải đo từ THỨ ĐÃ PHÁT RA DÂY

**Ca gốc:** lõi quỹ đạo P6.2 chia duration theo độ dài cung — nghe hợp lý, và sai: `dP/dt = P'(s)/T_i` **nhảy bậc tại MỌI mối nối**. Vận tốc nhảy 1,5 → 0,375 m/s trong một tick 20 ms ⇒ gia tốc thật **≈ 55,7 m/s² ≈ 5,7 g** — trong khi **`peakAcceleration()` báo 0,897** vì nó đọc đạo hàm **giải tích TRONG LÒNG** mỗi segment, còn bậc thang nằm **ở BIÊN**.

> 🔴 **MỘT CON SỐ AN TOÀN NÓI DỐI NGUY HƠN KHÔNG CÓ CON SỐ NÀO.** Không có số ⇒ biết là không biết. Có số sai ⇒ tưởng là biết, và **ship**.

Nguyên tắc lan ra 5 chỗ: vị trí đã phát (không phải trường accel) · clearance trên **spline** (không phải waypoint — spline cắt góc 0,442–0,591 m, chạm 253 ở khe 1,4 m) · **mép gần AABB** (không phải tâm bbox — FAIL giả +0,19 m ở cổng mount) · **tally** "N escapes flown" (không phải evidence chuỗi) · G-N2 kiểm **không-chạm-xích TRƯỚC** rồi mới đọc gia tốc (kẹp-nhả tạo 48,6 m/s² thuộc về cái kẹp).

**Số G-N2 hôm nay** (sau carrot-publish, RTF 0,957): cross-track p95 0,034–0,069 · lead đỉnh 0,563/0,80 · gia tốc 0,24 ≤ 3,0 · đối chứng carrot 14,21 vẫn bị bắt · mục h đếm-era: "17 plans in 4 eras, 6 retires".

🟠 **Nợ mở:** kẹp xích vẫn nhả trọn-một-bước (48,6 m/s² trên dòng setpoint) — hiện dựa vào `MPC_ACC_*` của PX4 kẹp hộ; sửa đúng là nhả theo dốc.

### 4b. Vì sao escape phải qua PHONG BÌ CAO ĐỘ của navigator (E2-C1)

**Lỗ:** phong bì `min/max_altitude_m` chỉ thực thi ở **cổng nhận goal** — escape không đi qua cửa đó. Advisor hỏng phát `z = −3` (lệch 4,15 < trần 8,0) ⇒ được nhận ⇒ B-spline **mượt** xuống đất ⇒ **dây xích không cắn** (máy bay bám kịp hoàn hảo) ⇒ không ai nói không.

> 🔑 **Một bộ phát hiện lỗi chỉ bắt được loại lỗi nó được thiết kế để bắt** — đừng đếm nó như lá chắn đa dụng.

**Vá:** navigator kiểm escape trong `[min_altitude_m, max_altitude_m]`, ngoài ⇒ từ chối → HOLD. Test `AnEscapeBelowTheAltitudeFloorIsRefusedAndHeld`. Lý lẽ cùng khuôn `buildRecoveryPlanner`: *một khôi phục không được bay chỗ một goal bị cấm — escape cũng vậy.*

**Descent chỉ GHI không CHẶN** — chính sách "cấm né bằng hạ thấp" đã nằm ở `avoid.allow_descent` của advisor; chặn lần hai ở navigator là **hai nơi giữ một chính sách** (nguyên tắc "một chính sách một nhà", cùng họ R23).

**`max_escape_deviation_m` = 8,0 chứ không phải 5,0:** ràng buộc `≥ spiral_growth × 2√steps = 4,90 m`; 5,0 chỉ biên 2% ⇒ từ chối chính advisor đang chạy đúng; 8,0 biên 63% — *guard này bắt một advisor HỎNG, không đi nghi ngờ một advisor đang chạy đúng*.

**Node TỪ CHỐI KHỞI ĐỘNG** khi tham số tự vô hiệu (`advice_timeout ≤ 0` từng tắt câm cả máy né; `max_escape_deviation = NaN` làm guard không bao giờ cắn; 3 combo `require_obstacle_feed` + rollback). **Công tắc phải là boolean có tên, không phải một con số.**

### 4c. Mã kết quả nói dối ở NÚT LEO THANG — thứ đắt nhất để lại cho P8

Kiến trúc an toàn là **cây leo thang**: P8 chọn khôi phục → thất bại thì leo bậc nặng hơn — bậc cuối là LAND/HANDOVER, "điểm sát sinh mạng nhất" của contract. **Mã kết quả sai ở nút leo thang không dừng ở đó — nó ĐẨY hệ lên một bậc lẽ ra không cần.**

| # | Nguyên tắc cho P7/P8 | Ca thật |
|---|---|---|
| 1 | Mã kết quả nói về **TRẠNG THÁI VẬT LÝ**, không về trạng thái nội bộ node | HOLD abort LOST_LOCALIZATION trong khi máy bay hover đúng chỗ, stream 20 Hz |
| 2 | Điều kiện kết thúc dùng đúng **loại bằng chứng hành động đó SỞ HỮU** | HOLD không đọc pose ⇒ xong theo bằng chứng dòng lệnh |
| 3 | **"Biết mà vẫn làm" phải được GHI**, không giấu | Bằng chứng disown nằm trong `result.message` |

P6 để lại cho P8 các câu trả lời trung thực: `executed_type` là loại **đã bay thật** · HOLD sống khi định vị chết, CLIMB/RETURN thì không — và nói ra · `avoidance_hold_timeout_sec` 12,0 s = 4× vòng đời bản đồ · R-C2b đưa "goal quá xa" từ 6988 ms xuống 972 ms abort-kèm-lý-do.

---

## 5. NHỮNG THỨ CHƯA CHỨNG MINH — phần trung thực

| # | Chưa chứng minh / còn hở | Ghi chú |
|---|---|---|
| 1 | 🟠 Navigator **TIN advisor về hình học** (chỉ chặn hữu hạn / phong bì cao độ / lệch xa) | Cái giá đã chọn của kiến trúc advisor |
| 2 | 🟠 `ESCAPE` khi bay carrot chỉ HOLD — TrackTarget giữa vật cản sẽ **dừng**, mục tiêu chạy mất | Việc sau, chưa chốt |
| 3 | 🟡 `Takeoff`/`HoldPosition`/`Land` publish nhưng **không có watch advice** | Mở watch cho Takeoff có giá thật (1 nhịp HOLD cuối chặng leo phá chính chuyến cất cánh) |
| 4 | 🟡 Cuối chặng carrot ngắn (<~0,6 m) advisor mất kế hoạch một nhịp | Lệch về phía an toàn nhưng giật cục khi map bắt buộc |
| 5 | 🟡 Ghép tuyến sau escape **chưa chứng minh ngăn livelock** — cần hình lõm chữ U | memory §7 mục 12 |
| 6 | 🔑 Bức tường không thử được reactive khi có route (0 escape) — phải dùng **vật cản xuất hiện SAU tuyến** | claim 8 |
| 7 | 🟠 Dựng lại giữa chuyến khởi hành từ NGHỈ = bậc vận tốc mỗi escape | Đã đo D2: 2,23 ≤ 3,0; cơ chế còn |
| 8 | 🟡 `escape_replan_interval_sec`/`escape_refresh_m` chưa test nào làm chúng là thứ DUY NHẤT chặn | throttle 1/58 |
| 9 | 🟠 Lưới publish **chưa mang đoạn bàn giao yaw** — dự đoán mũi từ `points[].yaw` sai ở giây cuối | **Đóng trước khi P7/P9 subscribe** |
| 10 | 🟡 Guard độ tươi route chưa có test riêng | nợ mới hôm nay |
| 11 | 🟡 Flake 1/7 `GotoInAForeignFrame...` khi suite chạy liên tiếp máy nóng | nợ mới hôm nay |
| 12 | 🟠 Trễ mũi TrackTarget 93,7° — chưa ngưỡng | số cho P9 |
| 13 | 🟡 Lưới publish full 50 Hz (~1,3 MB latched) — qua radio thật tranh băng thông `/fmu/in/*` | decimate trước P11 |
| 14 | 🟠 Sàn bất định world model là chỗ giữ chỗ — WARN định kỳ tới khi đo thật | P5.4/P5.5 follow-up |
| 15 | ⚠️ Mô hình bão hoà `σ_tuổi` hiệu chỉnh theo bộ tiêm dự án — VIO thật phải xem lại | P11 |

Và một nợ **nguy cho ĐỜI THẬT, không nguy cho sim** (memory §5): 🟠 **EKF2 reset EV aiding 1 lần/giây** (`reset_{pos,vel}_to_vision` 71 lần/70 s) — sim vô hại vì EV là ground truth; VIO thật thì mỗi giây bơm một cú nhảy vào vận tốc bộ ước lượng. **Đóng trước P11.**

---

## 6. CÁCH TỰ KIỂM CHỨNG — lệnh anh gõ được ngay

Tất cả trong WSL, workspace `~/PX4_ROS2`.

### 6.1 Phần tất định (không cần Gazebo)

```bash
cd ~/PX4_ROS2
colcon test --packages-select uav_navigation
colcon test-result --all          # báo dạng "N case / M target" — SUY LẠI, đừng chép
```
**Đúng khi:** `uav_navigation` **265 case / 11 target, 0 lỗi** (số đo 2026-08-20 sau E2). Số target < 11 = **thiếu nguyên một target** — đúng cái đã sai ngày 08-17.

### 6.2 Dây xích còn là bộ báo lỗi không (§2a)

```bash
colcon test --packages-select uav_navigation \
  --ctest-args -R "TheShippedSpeedStaysInsideWhatTheAircraftCanTrack|TheOldSpeedCeilingRidesTheLeashTheWholeWay"
```
**Đúng khi cả hai xanh.** Một mình cái đầu xanh không chứng minh gì.

### 6.3 Trần leo bị chặn bởi băng bay (§2c)

```bash
colcon test --packages-select uav_navigation \
  --ctest-args -R "TheClimbIsCappedByTheSliceTheMapCanActuallyVouchFor"
```

### 6.4 Escape ngoài phong bì + HOLD sống sót (§4b, §2e)

```bash
colcon test --packages-select uav_navigation \
  --ctest-args -R "AnEscapeBelowTheAltitudeFloorIsRefusedAndHeld|AHoldRecoverySurvivesThePoseItCannotRead"
```

### 6.5 Năm cổng bay (mỗi lượt ~10–15 phút, XẾP HÀNG — không hai sim cùng lúc)

```bash
scripts/run_m5_regression.sh    # hồi quy nền
scripts/verify_trajectory.sh    # G-N2
scripts/run_gn4b.sh             # G-N4b — né vật cản
scripts/run_gn5.sh              # G-N5 — FollowPath + TrackTarget
scripts/run_gn6.sh              # G-N6 — Recover + preemption
```

| Nhìn | Ngưỡng | Nếu lệch |
|---|---|---|
| RTF cửa sổ bay | ≥ 0,95 mới không cần nhãn | < 0,95 ⇒ số liệu DÁN NHÃN |
| Lead / ngân sách 0,80 | ≤ 0,70 (G-N2 mục c) | chạm xích ⇒ mọi số gia tốc vô nghĩa |
| Đối chứng carrot | **phải** vượt 8 m/s² | không nổ ⇒ cả cổng FAIL |
| Model | `uav0_track`/`uav0_nav` | `uav0` chỉ bridge `/clock` ⇒ σ_z GPS 0,5 m ⇒ ngưỡng ≤0,3 m dưới sàn nhiễu |

Tự kiểm model có nguồn vision: `grep -c vio_odometry_raw src/uav_sim_gz/config/bridge_<model>.yaml` phải = 1.

### 6.6 Nhìn hệ thống ĐANG SỐNG (khi chẩn đoán, §3a)

```bash
ros2 topic hz  /uav/uav0/control/command_selected    # ~20 Hz — dưới 2 Hz là rớt offboard
ros2 topic echo /uav/uav0/planning/avoidance --once
ros2 topic echo /uav/uav0/planning/trajectory --once # LATCHED: có ngay cả khi vào muộn
ros2 topic echo /uav/uav0/backend/offboard_status --once
```
🪤 **Bẫy công cụ trả giá hôm nay:** `ros2 action list` KHÔNG có `--no-daemon` (stderr nuốt ⇒ đọc thành "chưa quảng bá") · log rclcpp ra **STDERR** (script `2>/dev/null` đếm 0 vĩnh viễn) · `pgrep -f 'a\|b'` khớp 0 (ERE, `\|` literal — khác grep BRE).

---

## 7. CÂU HỎI TỰ KIỂM

| # | Câu hỏi | Đọc lại |
|---|---|---|
| 1 | Trần tốc độ bền vững là bao nhiêu, suy từ đâu? Vì sao không nới `max_lead`? | §2a |
| 2 | `run_gn4b.sh` PASS — được nói (a) "đã chứng minh an toàn" hay (b) "chạy đúng trong điều kiện đã đo, RTF 0,984"? | §2b |
| 3 | `costAt()` bỏ qua z ⇒ hai luật an toàn nào phải viết lại? | §2c |
| 4 | Sàn cứng 0,2 s của `carrot_plan_period_sec` suy từ đại lượng nào? Đặt thấp hơn thì sao? | §2d |
| 5 | Vì sao `Recover(TYPE_HOLD)` được tắt hai phép kiểm định vị còn CLIMB/RETURN thì không? | §2e, §4c |
| 6 | `peakAcceleration()` sai ở **cấu trúc** nào? Nêu một chỗ khác cùng khuôn | §4a |
| 7 | Escape `z=−3`, lệch 4,15 m: kể tên **mọi** cơ chế có cơ hội chặn trước bản vá, vì sao không cái nào cắn? | §4b |
| 8 | Test đỏ: bốn câu phải hỏi **trước** khi tìm bug trong code sản phẩm? | §3b |
| 9 | `uav0_track` đậu được mà không bay được: thứ tự loại trừ, bước nào loại nhiều nghi phạm nhất? | §3a |

### Đáp án ngắn

**1.** `≈ K_p × max_lead = 0,95 × 0,80 = 0,76 m/s`; ship 0,55 với `max_speed ≤ K_p·max_lead/1,3`. Không nới `max_lead` vì lên ~2,05 m là **yếu lá chắn 2,5 lần** — đổi bộ báo lỗi lấy tốc độ là đổi sai chiều.

**2.** Chỉ (b). Cổng bay bị chi phối bởi RTF (0,828–0,918 trên máy **thật sự rảnh** — "máy rảnh sẽ cứu" đã bị bác), nên nó dán nhãn; lý lẽ an toàn ở cổng tất định G-N4a (có mutation).

**3.** (i) trần leo = `min(max_escape_climb_m, map.flightBand())` — 0,423 có băng vs 1,759 bỏ băng; (ii) cấm hạ thấp so với **obstacle.z**, không phải position.z — so với máy bay thì gần như không bao giờ cắn.

**4.** Từ **85,2 ms vòng advisor đo ở G-N4a** (sàn ≈ 2,3×). Thấp hơn ⇒ mọi advice nói về chặng đã bị thay ⇒ navigator đọc thành im lặng ⇒ map bắt buộc là HOLD vĩnh viễn. Node từ chối khởi động.

**5.** HOLD là loại **duy nhất không đọc pose**. Giữ kiểm thì đúng ca mất định vị nó chết ≤1 s với LOST_LOCALIZATION ⇒ P8 đọc "khôi phục thất bại" ở đúng nút leo thang, đường còn lại chỉ LAND/HANDOVER — ngược contract §2.5. CLIMB/RETURN **có** đọc pose nên giữ nguyên kiểm. Bằng chứng "biết mà vẫn giữ" trong `result.message`.

**6.** Đọc đạo hàm giải tích **trong lòng** segment, bậc thang nằm **ở biên** ⇒ mù theo cấu trúc. Cùng khuôn: clearance trên waypoint thay vì spline · tâm bbox thay vì mép gần AABB · evidence chuỗi(1) thay vì tally(2).

**7.** (a) phong bì cao độ — không cắn vì chỉ ở cổng nhận goal; (b) guard lệch 8,0 — không cắn vì 4,15<8,0; (c) guard hữu hạn — −3 hữu hạn; (d) guard descent — chỉ WARN, cố ý; (e) dây xích — không cắn vì đường mượt, máy bay bám kịp. ⇒ Không ai nói không. Vá E2-C1.

**8.** (i) cửa sổ có mẫu không; (ii) cửa sổ có đúng là cửa sổ của khẳng định không; (iii) có guard anh em cướp lời không; (iv) mốc suy số có phải mốc vật lý dùng không. + nếu 7/7 lần đều "tại test" → đòi mutation + đối chứng dương ngay.

**9.** Dây xích (loại 0 — triệu chứng) → **thẩm vấn PX4 sống** (loại sạch phía ROS trong một lượt — nhiều nhất) → log PX4 → đối chứng M5 cùng model → bước 2,5 m vs ramp kẹp 0,58 → **diff airframe** (chốt hạ).

---

## 8. BA CÂU MANG VỀ

> **1. Số an toàn phải đo từ THỨ ĐÃ ĐI RA DÂY.** Trường nội bộ tiện tay là thứ đã che khuyết tật 5,7 g.
>
> **2. Một guard chưa từng CẮN trong test nào là một guard chưa được kiểm chứng — và hình học của nó phải ĐO, không SUY.**
>
> **3. Đặt lý lẽ an toàn ở nơi phép đo không phụ thuộc tài nguyên máy.** Cổng bay dán nhãn; cổng tất định cấp chứng nhận. Mọi cổng phải tự chứng minh phép đo của nó bắt được lỗi.

---

### Phụ lục — file để tra sâu

| Cần | File |
|---|---|
| Cách dùng navigator + mô hình đồng thời §5 + §6f–§6h + §8 nợ | `src/uav_navigation/README.md` |
| Lý lẽ thiết kế P6 (§2e xích · §2f cắt góc · §2g advisor) | `.claude/plan/P6-navigation.md` |
| Sáu quyết định đóng P6 + bảng lane + 7 điều CẤM | `.claude/plan/P6-completion-run.md` |
| Perception: vì sao không có driver camera | `src/uav_perception/README.md` |
| World model: ràng buộc trung tâm bản đồ | `src/uav_world_model/README.md` |
| Hợp đồng §2.5 Recover · §2.6 · §2.14 · §2.15 | `docs/interface-contract-v0.1.md` |
| Trạng thái + bẫy từng package §8 | `docs/package-status.md` |
| Tham số thật đang ship | `src/uav_navigation/config/navigation_params.yaml` |
| Bản vá airframe §3a | `src/uav_sim_gz/airframes/4103_gz_uav0_track` · `4102_gz_uav0_nav` |
| Test bằng chứng §2a/§2e/§3b | `src/uav_navigation/test/test_navigator_action_server_node.cpp` |
| Cổng tất định G-N4a | `src/uav_navigation/test/test_avoidance_chain.cpp` |
| Guard leo/hạ §2c | `src/uav_navigation/test/test_local_avoidance.cpp:271` |
