# `uav_observability`

Tầng **BẰNG CHỨNG** của framework — không phải tầng an toàn. Trong sim một
chuyến bay hỏng chạy lại được; trên drone thật thì không, nên hộp đen là thứ
duy nhất còn lại để chẩn đoán ngoài đời.

**Nguyên tắc chi phối O1:** *observability hỏng ⇒ mất bằng chứng, KHÔNG được
mất chuyến bay.* O1 không phải lời hứa suông — nó được **thi hành bằng cấu
trúc**, kiểm được bằng công cụ, không phải bằng lòng tin vào một cờ:
- **3 process riêng** (`rosbag_manager_node` · `diagnostics_node` ·
  `event_logger_node`), không composition — một node observability rớt không
  kéo hai node kia hay bất kỳ node bay nào theo.
- Mọi subscription **BestEffort mặc định** (`config/observability_params.yaml`
  gọi là `be`) — đổi lấy zero backpressure lên luồng bay; chỉ nhóm cứu-mạng
  và nhóm TransientLocal mới dùng Reliable.
- Mọi lỗi ghi = **WARN + degrade**, **không throw sau khi node đã lên** —
  `bag_retention.hpp`/`rosbag_manager_node.hpp` nói thẳng: một topic hỏng chỉ
  làm hỏng đúng topic đó (`topics_failed_++`), đĩa đầy hạ xuống
  `DISABLED_NO_SPACE`, không node nào chết vì một topic sai kiểu.
- `owner="blackbox"` (2 nguồn `diagnostics/observability*`) **không bao giờ**
  được tính vào `go_no_go` — sức khoẻ của chính hộp đen không được là điều
  kiện cất cánh (xem mục 1 dưới).

## Trạng thái (2026-08-24)

**P10 ĐÓNG TRỌN** (P10.0–P10.9a 2026-08-23, `go_no_go` thiết kế lại hoàn
toàn ở P10.9b 2026-08-24, vá 3 CHẶN ở review lượt 2, **8 VÀNG đóng ở review
lượt 3 (KHÔNG CÒN CHẶN)** — cùng ngày). 3 node + 3
lib ROS-free (`bag_retention` · `staleness_board` · `event_ledger`) —
**163 case / 6 target, 0 lỗi, 2 lượt liên tiếp**. Cả 3 node đã vào
`sim.launch.py` (cờ `blackbox`/`diagnostics`/`event_log`, mặc định **true**),
`rosbag_manager_node` xếp đầu danh sách node để thắng race discovery của
topic TransientLocal.

Lý lẽ thiết kế đầy đủ + kết quả từng cổng →
`.claude/plan/P10-observability.md`;
hội đồng thiết kế lại `go_no_go` →
`.claude/plan/P10-gonogo-design-panel.md`;
hợp đồng đối ngoại (nguồn chuẩn cho mọi field trên dây) →
[`docs/interface-contract-v0.1.md`](../../docs/interface-contract-v0.1.md) §2.20;
trạng thái/nợ đầy đủ → [`docs/package-status.md`](../../docs/package-status.md) §12.

## 7 điều PHẢI biết trước khi sửa

| # | Điều | Vì sao / chi tiết |
|---|---|---|
| **a** | `go_no_go` có **BA** giá trị, không phải hai | `GO` ⟺ mọi mục đang tính đều OK **và** không mục nào UNKNOWN/STALE · có mục UNKNOWN ⇒ **`UNKNOWN`** (KHÔNG phải `NO_GO`, cũng KHÔNG phải `GO`) · còn lại ⇒ `NO_GO`. Ai coi `!= GO` là `NO_GO` sẽ mất đúng phân biệt "chưa đo được" khỏi "đo được và xấu" (R30). Một nguồn Sub-B đang **WARN** cũng ép `NO_GO` (không phải chỉ ERROR). |
| **b** | Vị ngữ pha là **đại lượng ĐO ĐƯỢC**: `armed && connected`, mất `connected` ⇒ FLIGHT strict ngay | `gate_mode` không còn là param ghi tay — suy từ `/state/vehicle`. `px4_state_adapter_node.cpp` chỉ gán `armed` **bên trong** `if (connected)`; mất link PX4 vẫn publish đều (topic tươi) nhưng `armed` rơi về `false` mặc định — thiếu điều kiện `connected` từng đọc nhầm thành "hạ pha xuống PREFLIGHT giữa lúc đang bay". Nâng pha (`preflight→flight`) **TỨC THÌ**; hạ pha phải **dwell 7 tick liên tiếp** (`kPreflightDwellTicks`, mặc định 0,35 s ở `report_period_sec=0,05s`) — `declareAndValidateParams()` ép `dwell×report_period > always_timeout_sec(state/vehicle)` (V4 review lượt 3: so với `2/expected_hz` là thước đo SAI — grace thật là chính `always_timeout_sec` của `state/vehicle`, 0,3 s), từ chối khởi động nếu vi phạm (R33). Còn phase_source=`unmeasured_strict` (state/vehicle mất `connected` dù topic vẫn tươi) tự nó cũng CHẶN go/no-go (V6) — không còn chỉ dựa vào Sub-A của chính `state/vehicle` tình cờ đi kèm STALE. **KHÔNG dùng `in_air`** dù `VehicleState` có trường này: `in_air = takeoff_time > 0` **không reset sau hạ cánh** (nợ #2 của P8) ⇒ dùng nó sẽ tái tạo lại chính bug đang sửa (đèn kẹt FLIGHT vĩnh viễn từ chuyến bay thứ hai). |
| **c** | **NW1/NW2 — action-class MỘT CHIỀU: chỉ phủ quyết, không bao giờ cấp phép** | NW1: child mang KeyValue `action` tiền tố ∈ `{hold, inhibit, would_hold, would_inhibit}` **và** `level==ERROR` ⇒ **luôn được đếm**, mọi pha, mọi cấu hình, không hàng miễn-trừ/param nào tắt được. NW2: nguồn `action_aware=true` (hiện **chỉ** `diagnostics/safety`) mà một child ERROR thiếu/không phân giải được `action` cũng luôn được đếm (fail-closed khi P8 đổi từ vựng). `action=report` **không** tự miễn trừ gì — nó chỉ *không kích NW*, vẫn phải qua bảng miễn-trừ như bình thường. |
| **d** | Bảng `config/preflight_waivers.yaml` là **thứ nguy hiểm nhất** trong package | Thêm một hàng là nới một lá chắn go/no-go — ngang cấp sửa hợp đồng, không phải sửa config: mỗi hàng bắt buộc `waiver_reason` không rỗng và phải được **chủ dự án ký**. Đúng **HAI** giá trị `when` hợp lệ (`preflight`/`perception_off`) — thêm giá trị thứ ba là sửa hợp đồng. Không hàng nào được trỏ vào child never-waivable (mục c) hoặc child summary/rollup (`diag_source_summary_child`) — node **từ chối khởi động** nếu vi phạm. Có **allow-list ký cứng** trong cổng (`O4_ALLOWED_WAIVED` ở `scripts/verify_observability.sh`, vòng `o4-gate` (f)/G6) đối chiếu đúng `waived_count` + đúng tập tên đang được miễn — không ai âm thầm nới cổng chỉ bằng cách thêm một hàng yaml mà cổng không phát hiện. `waiver_unmatched` (KeyValue trên dây) tự phát hiện khi P8 đổi tên child và một hàng waiver mồ côi. |
| **e** | **Bag sau sự cố PHẢI `ros2 bag reindex`**; storage mặc định **sqlite3**, không phải mcap | `metadata.yaml` chỉ được ghi lúc `Writer::close()` sạch — tiến trình bị giết giữa chừng thì mất. G-O1 (đo thật, `kill -9` × 10/cấu hình): **mcap 0/10** đọc được kể cả sau reindex (plugin 0.15.16 từ chối mở khi thiếu footer, không có quét tuyến tính) · **sqlite3 10/10** đọc được **sau `ros2 bag reindex`** (0/10 trực tiếp), mất trung vị **0,032 s** dữ liệu cuối. Đây là lý do format mặc định lật từ mcap sang sqlite3 ở `config/observability_params.yaml`. |
| **f** | Dòng thời gian sự kiện là **FILE JSONL, không phải topic** (O2) | `event_logger_node` **không publish sự kiện nào lên bus** — chỉ ghi `<log_root>/<uav_id>_<UTC>_events.jsonl`. `/mission/events` vẫn là kênh sự kiện mission **duy nhất** trên dây; JSONL là **dẫn xuất**, không phải nguồn sự thật thứ hai (R9) — không consumer runtime nào được phép phụ thuộc vào nó. |
| **g** | Teardown recorder cần **SIGINT**, không SIGKILL | Giết cứng ⇒ bag không đóng sạch ⇒ phải reindex (mục e). `scripts/stop_sim.sh` từng **thiếu hẳn** pattern `uav_observability/` trong vòng pkill, khiến cả 3 node sống sót **mọi lần** teardown thành mồ côi vĩnh viễn (bắt được ở P10.6: 15+ phút sau "sim stopped, clean" vẫn thấy cả 3 tiến trình chạy). Đã vá bằng một khối `pkill -INT` riêng cho `uav_observability` chạy **trước** vòng SIGTERM chung. |

## Cách dùng

### Bật/tắt bằng cờ launch
`sim.launch.py` nhận 3 cờ boolean, mặc định **`true`** cả ba:
`blackbox` (→ `rosbag_manager_node`) · `diagnostics` (→ `diagnostics_node`) ·
`event_log` (→ `event_logger_node`). `diagnostics_node` còn nhận
`perception_enabled`/`blackbox_enabled` (nối thẳng từ cờ `perception`/
`blackbox` của chính bringup, C2) để biết mục nào đang hợp lệ "không có
publisher" thay vì kẹt UNKNOWN vĩnh viễn. `real.launch.py` (P11) sẽ để
`blackbox` mặc định `false` cho tới khi có review độc lập ngân sách đĩa/IO
onboard (D-4, cùng nếp `safety_enforcement` của `uav_safety`).

### Đọc đèn go/no-go
```bash
bash scripts/preflight_light.sh --uav-id uav0 [--max-age-sec 2.0] [--timeout 3.0]
```
🔴 **Không bao giờ** dùng `ros2 topic echo --once` trần —
`/state/system_health` là Reliable/KeepLast(1)/**TransientLocal**, mẫu latch
cuối của một `diagnostics_node` đã chết vẫn nằm mãi đó và `echo` không có
cách nào tự tố cáo điều đó. `preflight_light.sh`/`.py` tự kiểm tuổi mẫu
**hai chiều** qua `wall_stamp_sec` (không chỉ tuổi quá lớn — tuổi **âm**
cũng bị từ chối, in `CLOCK SKEW`, N8) bằng `scripts/gate_freshness.py`'s
`is_fresh()` — dùng chung logic với cổng `o4-gate` để hai nơi không lệch
luật. Exit code: `0`=GO, `1`=NO_GO, `2`=UNKNOWN/STALE/CLOCK SKEW/FAILED TO
MEASURE. **Checklist tiền bay bắt buộc đọc lại đèn SAU khi ARM, TRƯỚC khi
takeoff** (cửa sổ mù mở-nguồn→ARM, xem mục Nợ mở).

### Đọc bag / JSONL
- Bag ghi ở `bag_root` (mặc định `~/uav_bags`, `config/observability_params.yaml`).
  `ros2 bag info <dir>` bình thường; nếu chuyến trước bị crash/mồ côi →
  **`ros2 bag reindex <dir>` trước** (mục e ở trên).
- Timeline sự kiện ở `log_root` (mặc định `~/uav_events`), file
  `<uav_id>_<UTC>_events.jsonl` — một dòng JSON một sự kiện, khoá cố định
  `t_sim`/`t_wall`/`seq`/`src`/`field`/`from`/`to`/`level`/`detail`. Dòng
  `level ≥ ERROR` đã `fsync` xuống đĩa trước khi callback trả về; mức thấp
  hơn fsync theo chu kỳ `fsync_period_sec` (mặc định 2 s).

### Chạy cổng kiểm chứng
```bash
bash scripts/verify_observability.sh [round ...]
```
Các nhóm vòng chính (chi tiết từng vòng ở comment đầu file):
- `o1` (= `o1-control`/`o1-mcap-*`/`o1-sqlite3-none`/`o1-report`) — **G-O1**
  độ bền storage, thuần ROS domain **99**, không Gazebo. `o1-prod` là vòng
  riêng đóng nợ nhỏ (sqlite3+file/zstd), không nằm trong bundle `o1`.
- `o2` — **G-O2** cổng chi phí (ΔRTF, p99 khoảng-đến `command_selected`, CPU).
- `o3` — **G-O3** replay + nợ N-c (đồng hồ sim lùi); `o3-synth` là bằng
  chứng CHÍNH THỨC duy nhất cho cơ chế N-c (không đi qua `ros2 bag play`,
  vì `--loop`/không-`--loop` đo trúng nhiễu độ phân giải tick ~40,03 Hz của
  chính công cụ replay, xem Bẫy đã trả giá).
- `d0-baseline` — bắt số thật đậu bãi để nạp bảng waiver (không phụ thuộc
  `o4-gate`).
- `o4-gate` — **G-O4** go/no-go trên dây, bringup Gazebo+PX4+ROS thật domain
  0, mọi sub-part đứng yên đậu bãi (`gate_mode` tự đo ra `preflight`).

## Lệch thiết kế có chủ đích

| Lệch | Lý do |
|---|---|
| **Không thêm msg/srv nào** (Q-P10-5) | Mọi giao diện dùng `diagnostic_msgs` chuẩn — `KeyValue` đủ biểu đạt, không cần mở rộng `uav_interfaces` cho một tầng không phải an toàn. |
| **Không dùng `diagnostic_aggregator` apt** | Không canh được cadence của `/world/*` (không có bản DiagnosticStatus tự nhiên), không phân biệt UNKNOWN với OK (R30) — tự viết `staleness_board` để giữ đúng ngữ nghĩa 3 giá trị. |
| **Timeline file-only, không topic** (O2, Q-P10-6) | Publish sẽ trùng vai `/mission/events` và phá chính nguyên tắc "không tạo nguồn sự thật thứ hai". |
| **Không ghi ảnh mặc định** (D-2) | Cầu sim chỉ phát ảnh raw; muốn ghi phải qua `image_transport`/republish riêng — để P11 khi có driver camera thật. |
| **Không ghi `/clock`** (D-3) | `ros2 bag play --clock` tự tổng hợp lại, ghi thêm là dữ liệu thừa. |
| **Ghi `/fmu/*` bằng chuỗi kiểu** (Q-P10-1, hợp lệ họ R25) | Recorder chỉ ghi byte đã serialize, tên kiểu là **dữ liệu trong yaml** — không import/link/đọc trường nào. 4 điều kiện cứng: (1) `package.xml`/CMake không có `px4_msgs`; (2) whitelist tường minh trong yaml sau cờ `record_px4_topics`, không wildcard; (3) mã nguồn không chứa chuỗi `px4` ngoài yaml (ngoại lệ duy nhất: định danh cờ `record_px4_topics` + comment giải thích); (4) thiếu typesupport lúc chạy ⇒ degrade có báo cáo, không throw. |
| **3 process riêng, không composition** (D-1) | Cách ly lỗi (O1) — một node observability rớt không kéo node khác theo. |
| **`gate_mode` từ param ghi tay → đại lượng đo được** (P10.9b) | Bản đầu (`gate_mode` param, worst-of-source) đọc `NO_GO`/kẹt `UNKNOWN` ngay cả khi đậu bãi hoàn toàn khoẻ — lỗi CẤU TRÚC, tái lập 2/2. Thiết kế lại hoàn toàn theo hội đồng phản biện chéo, xem mục b ở trên. |

## Bẫy đã trả giá

| Bẫy | Triệu chứng → nguyên nhân → cách đúng |
|---|---|
| **mcap mất 100% bag khi tắt máy không sạch** | G-O1: `kill -9` giữa lúc ghi × 10 lượt → mcap **0/10** đọc được (kể cả sau reindex) vì plugin 0.15.16 từ chối mở bag thiếu footer, không quét tuyến tính. → Production đã lật sang `storage_id: sqlite3` (10/10 qua `ros2 bag reindex`). |
| **`stop_sim.sh` thiếu pattern → 3 node mồ côi mọi lần teardown** | P10.6: sau "sim stopped, clean", `ps aux` vẫn thấy cả 3 node observability sống 15+ phút. → Đã vá: khối `pkill -INT` riêng cho `uav_observability`, chạy **trước** vòng SIGTERM chung. |
| **`offered_qos_profiles` rỗng ⇒ replay mất topic latched** | P10.8c: `TopicMetadata` chỉ set `.name/.type/.serialization_format`, để trống durability ⇒ `ros2 bag play` phát mọi topic Volatile ⇒ subscriber TransientLocal từ chối mẫu. → Vá bằng `rosbag2_transport::Rosbag2QoS`'s `YAML::convert`. Vá RỒI VẪN CHƯA ĐỦ (C3, review): giá trị ghi là QoS **SUBSCRIBER** (`be`) chứ không phải QoS **PUBLISHER thật** — vá lần 2 bằng dời `create_topic()` sang lúc nhận **message đầu tiên thật sự**, đọc `get_publishers_info_by_topic()` tại đó. |
| **`/clock` của `ros2 bag play` là tick rời rạc ~40,03 Hz** | G-O3(c) CHỐT 2026-08-24: không đồng bộ pha với `cmd_mission` ~20 Hz ⇒ 29/30 mẫu đo âm ⇒ **mọi tiêu chí dung-sai-0 không đo được qua replay** (không phải lỗi hệ, là độ phân giải công cụ). → `o3-synth` (không đi qua `ros2 bag play`) là bằng chứng CHÍNH THỨC duy nhất cho cơ chế N-c; `--loop`/`clock_regressions` qua replay đã rút khỏi verdict. |
| **Cổng có thể PASS trên vòng thoái hoá** (N7) | `o4_report.py` bản cũ chỉ nhìn `records[-1]` một mình — không đủ tin: TransientLocal có thể latch một mẫu cũ/mồ côi mãi mãi, và `/clock` đứng làm **số mẫu** tụt hẳn chứ không chỉ mẫu cuối cũ. → Vá: precheck đếm mẫu tối thiểu + kiểm tuổi mẫu **hai chiều** qua `scripts/gate_freshness.py`'s `is_fresh()`, dùng chung với `preflight_light.py` (N8). |

## Nợ mở

Danh sách đầy đủ + số đo + trạng thái chốt/chưa chốt →
[`docs/package-status.md`](../../docs/package-status.md) §12 (không chép lại
ở đây — một thông tin một nhà). Headline đáng nhớ nhất khi bay thật: **cửa
sổ mù mở-nguồn→ARM** (miễn-trừ preflight bay hơi ≤1 tick khi `armed` lật —
giảm thiểu bằng quy trình, không có biện pháp kỹ thuật triệt để) và
**`ESTIMATOR_INPUT_INVALID` ở world GPS-denied indoor CHƯA CHỐT** (cố ý
KHÔNG miễn-trừ ở tầng observability, chờ P8/backend quyết).

## Build & test (WSL)

```bash
rm -rf ~/PX4_ROS2/src/uav_observability
cp -r /mnt/c/code/PX4_ROS2/src/uav_observability ~/PX4_ROS2/src/
cd ~/PX4_ROS2 && colcon build --packages-select uav_observability
colcon test --packages-select uav_observability && colcon test-result --verbose
```

`test_bag_retention`/`test_staleness_board`/`test_event_ledger` không cần
`ROS_DOMAIN_ID` (R20) — thuần, không `rclcpp::init`. 3 test node
(`test_rosbag_manager_node`/`test_diagnostics_node`/`test_event_logger_node`)
cần `ROS_DOMAIN_ID=99` (Q-P10-8, đã khai trong `CMakeLists.txt`) — chúng bịa
traffic thật trên tên topic thật, tách domain để không đụng sim đang chạy.
