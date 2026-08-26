# `uav_bringup`

Khởi động hệ thống tự hành và bài bay hồi quy.

---

## Hai lớp riêng biệt — đừng lẫn

Có hai thứ cần bật, và **chúng thuộc hai lớp khác nhau**:

| Lớp | Bật bằng gì | Gồm những gì | Có tương đương ở drone thật? |
|---|---|---|---|
| **Thế giới mô phỏng** | `scripts/start_sim.sh` | Gazebo, PX4 SITL, cầu uXRCE-DDS, **cầu Gazebo→ROS** | ❌ **Không** — drone thật thì phần này là **phần cứng và driver cảm biến** |
| **Phần mềm tự hành** | `ros2 launch uav_bringup sim.launch.py` | 5 node backend + 2 adapter định vị | ✅ **Có** — chạy y nguyên trên drone thật |

Tách vậy là có chủ đích: **thế giới mô phỏng là thứ thay thế cho hiện thực**, còn phần mềm tự hành là thứ **không đổi** giữa sim và thật.

Lợi ích thực tế khi phát triển: sim nặng và chậm khởi động (~45 s), còn node của ta build lại liên tục. Giữ sim chạy, restart node bao nhiêu lần cũng được.

## `sim.launch.py` — chạy phần mềm của ta, cấu hình cho mô phỏng

Khởi động **5 node backend** (`config/backend_params.yaml`) + **6 node định vị** (`config/localization_params.yaml`): `rangefinder_adapter_node`, `gps_adapter_node`, `vio_adapter_node`, `optical_flow_adapter_node`, `localization_mux_node`, `localization_health_node` + **3 node điều hướng** (`uav_navigation/config/navigation_params.yaml`): `route_planner_node`, `local_planner_node` (cờ `navigation`), `navigator_action_server_node` (cờ riêng `navigator`, P7.4) + **`control_authority_manager_node`** (`uav_control_authority/config/control_authority_params.yaml`, cờ `control_authority`).

### Cờ `navigator` — bật action gateway P6, MẶC ĐỊNH bật (P7.4, đóng nửa-sim của N13)

```bash
ros2 launch uav_bringup sim.launch.py navigator:=false   # tắt riêng navigator, giữ advisor + trọng tài
```

Tách khỏi cờ `navigation` (2 advisor) vì `navigator_action_server_node` là nguồn lệnh MISSION mà trọng tài phân xử (P7.3) — bật/tắt nó không cần kéo theo route/local planner. Nằm im khi không có goal (không stream `control/cmd_mission`) nên không tranh quyền với TEST trong M5. `real.launch.py` (P11) sẽ có cờ tương ứng.

### Cờ `navigation` — bật 2 node P6.3/P6.4, MẶC ĐỊNH bật

```bash
ros2 launch uav_bringup sim.launch.py navigation:=false   # tắt để dồn RTF cho một lượt M5
```

Mặc định `true` vì đây đã là node P6 chính thức, không phải thử nghiệm như `perception`. `require_obstacle_feed` bị ép **`false`** ngay trong `sim.launch.py` (quyết định 4 của P6) — `real.launch.py` khi ra đời **phải** ép `true` ở đúng vị trí tương ứng, không được dựa vào giá trị mặc định trong yaml.

🔴 **`localization_params.yaml` chứa tham số tiêm nhiễu GNSS, mặc định TẮT.** `real.launch.py` **không được** dùng lại file này — bay thật với cảm biến bị cố ý làm hỏng là kịch bản không được phép tồn tại. Khi bật, node kêu `WARN` và ghi `"simulated drift active"` vào `detail` của topic status.

**Không** khởi động Gazebo/PX4/agent — những thứ đó đã chạy sẵn từ `start_sim.sh`.

```bash
cd ~/PX4_ROS2 && UAV_MODEL=uav0_nav bash scripts/start_sim.sh   # T1: thế giới mô phỏng, tự source
```
```bash
source ~/PX4_ROS2/install/setup.bash && ros2 launch uav_bringup sim.launch.py   # T2: phần mềm tự hành
```

⚠️ **Thứ tự bắt buộc.** `sim.launch.py` bật `use_sim_time`, mà `/clock` do cầu Gazebo→ROS phát. Chạy ngược thứ tự thì mọi node ngồi chờ một đồng hồ không bao giờ tới. Không có thế giới mô phỏng thì dùng `use_sim_time:=false`.

⚠️ **Mỗi terminal mới phải `source install/setup.bash`.** Quên thì ROS2 chỉ thấy `/opt/ros/humble` và báo `Package 'uav_bringup' not found`. Chỉ `start_sim.sh` là tự source bên trong.

### Cờ `perception` — bật 5 node P5, TẮT theo mặc định

```bash
ros2 launch uav_bringup sim.launch.py perception:=true
```

`true` thêm **4 node `uav_perception`** (`marker_detector_node`, `obstacle_extractor_node`, `target_tracker_node`, `object_detector_node`) + **`world_model_node`** (`uav_world_model`), cùng namespace `/uav/<id>/...` và `use_sim_time` như phần còn lại. Không có config YAML dùng chung cho 4 node perception (khác backend/localization) — tham số override lấy theo đúng các script `verify_*.sh` đã kiểm chứng (`camera:=down` cho marker, `camera:=front` cho obstacle/object). `world_model_node` dùng `config/world_model_params.yaml` của chính `uav_world_model`.

Mặc định **`false`** — R0: không đổi node-list/hành vi của baseline M5 khi không ai bật cờ.

⚠️ **Bật cờ này làm TĂNG TẢI render.** Perception cần cảm biến trước (RGB+depth) nên chỉ có dữ liệu thật trên model có mang chúng (`uav0_track`, `uav0_full`) — bật trên `uav0`/`uav0_nav` thì node vẫn lên nhưng không có gì để xử lý. Số RTF/hiệu năng dưới từng workload xem cổng tổng P5 (`.claude/plan/P5-perception.md`, [`docs/package-status.md`](../../docs/package-status.md) §4).

## `real.launch.py` — cùng bộ node đó, cấu hình cho drone thật *(sẽ làm ở P11)*

Đây là chỗ **trả lời cho câu hỏi quan trọng nhất của cả dự án**: khi ra bay thật thì phải sửa những gì?

Câu trả lời mong muốn: **chỉ sửa launch file, không sửa một dòng code node nào** (quy tắc R7).

Khác biệt dự kiến giữa hai file:

| Hạng mục | `sim.launch.py` | `real.launch.py` |
|---|---|---|
| PX4 chạy ở đâu | SITL trên cùng máy | **Phần cứng flight controller** |
| Cầu nối uXRCE-DDS | UDP tới `localhost:8888` | **Serial/UART** tới FC |
| Cảm biến | Camera mô phỏng của Gazebo | **Driver camera thật** |
| Ngưỡng thời gian | Thoải mái | **Chặt hơn** — mạng và CPU thật kém ổn định hơn |
| Node backend | 5 node | **Đúng 5 node đó, không đổi** |
| Node định vị | 6 node, có tham số tiêm nhiễu | **Đúng 6 node đó** — nhưng **không phơi tham số tiêm nhiễu** |
| Node điều hướng | 2 node (`route_planner_node`, `local_planner_node`), `require_obstacle_feed:=false` | **Đúng 2 node đó**, nhưng `require_obstacle_feed:=true` (quyết định 4, P6) |

Nếu tới lúc làm `real.launch.py` mà phát hiện **phải sửa code node**, thì đó là **dấu hiệu thiết kế đã rò rỉ giả định về mô phỏng** vào trong node — phải sửa node, không phải sửa launch.

## `test/smoke_flight.py` — bài bay hồi quy

```bash
python3 install/uav_bringup/lib/uav_bringup/smoke_flight.py --flights 3
```

`chờ offboard → arm → takeoff → goto → land → disarm`. **Mã thoát 0 = đạt.**

🔑 **Offboard trước, arm sau.** PX4 khởi động vào LOITER, mà LOITER đòi **global position** — trong nhà không bao giờ có, nên arm ở đó là bị từ chối vĩnh viễn. OFFBOARD chỉ cần local position. Đảo thứ tự là thứ mở khoá được chuyến bay trong nhà.

⚠️ **Đi con thoi giữa hai điểm**, không bò dần về phía đông: 3 chuyến liên tiếp kiểu cũ là 9 m, xuyên tường phòng 12×12 m.

🔴 **Khi hạ cánh, THỨ TỰ quyết định:** đổi chế độ trước, ngừng phát setpoint sau. Làm ngược lại là bỏ đói offboard và **kích failsafe** ngay giữa lúc hạ.

⚠️ **Phải quên trạng thái offboard của chuyến trước.** Một `ACTIVE` cũ còn sót từ phiên trước sẽ thoả mãn điều kiện chờ, và arm sẽ xảy ra trong khi PX4 đã rơi về chế độ khởi động — chuyến bay bắt đầu ở một chế độ mà không ai kiểm.

🔑 **Node test phải dùng CHUNG đồng hồ với backend nó đang thử.** Nhịp setpoint phải ≥ 2 Hz *theo đồng hồ PX4 dùng*, nên nếu test chạy đồng hồ khác thì con số 20 Hz nó tự tin báo có thể là 1 Hz dưới mắt PX4.

### 🔴 Hai điều siết ngày 2026-08-25 — đọc trước khi đọc kết quả M5

**1. Arm CHỜ ĐƯỢC, và thời gian chờ luôn hiện ra.** `armWithinBudget()` gọi lại `/backend/arm` trong ngân sách **120 s** thay vì bỏ cuộc sau một lần. Lý do là một chuyến hỏng thật: PX4 từ chối arm vì `mag_test_ratio` **phi hữu hạn** ngay khoảnh khắc EKF2 tự reset quaternion **giữa chuyến** — trong khi yaw thật chỉ lệch **1,67°** so với ground truth Gazebo. Máy bay thật sự chưa sẵn sàng, nên bỏ cuộc sau một lần gọi là khuyết tật của **bài test**. Báo cáo in `arm=%.1f s over %d try(s)` **mỗi chuyến**: một chuyến cần một phút để arm là **một phát hiện**, không phải thứ được hấp thụ im lặng.
🔴 **Cấm nới `COM_ARM_EKF_YAW` để "cho qua"** — đó là tắt một kiểm tra tiền bay của PX4.

**2. Mã an toàn chưa ai viết lý do thì CHẶN một PASS.** M5 nay subscribe `/safety/state`; `safetyBlockers()` gộp hai điều kiện: có `recommended_action` **đang hành động** (`hold`/`inhibit`/dạng dry-run), **hoặc** có mã vi phạm **không nằm trong `SAFETY_EXPLAINED_CODES`**. Trước đó mã lạ chỉ in `⚠️ UNEXPLAINED` rồi vẫn cho đỗ — và đó chính là kẽ hở đã nuôi `LOCALIZATION_JUMP` sống nhiều ngày trong khi nó là một **cú nhảy pose thật 2,217 m / 100 ms**.
⚠️ Muốn thêm một mã vào danh sách miễn thì **phải viết lý do dài hơn 40 ký tự** — có test canh chính danh sách đó.

> 📌 **Hệ quả cho người đọc kết quả:** *"0 vi phạm"* của M5 giờ có nghĩa mạnh hơn trước, nhưng vẫn **không** có nghĩa *"planner và safety đã được kiểm"* — M5 chạy model `uav0` vốn **không có nguồn vision**, nên `odometry_fused` mà planner/safety đọc là GPS thô. Xem `docs/sim-boundary-statement.md` **B-61**.

## `test/g2_fused_accuracy.py` — độ chính xác của `odometry_fused`

Bay hình vuông có đổi cao độ để **kích thích cả ba trục**; các chặng tính theo **thời gian, không theo lúc tới nơi**, nên cửa sổ đo luôn đúng 60 s và so sánh được giữa các lần chạy.

🔑 **Vì sao phải nghe `estimator_source`:** mux dán nhãn đầu ra của chính nó là `SOURCE_FUSED`, nên topic trạng thái **không cho biết nó đã chọn nguồn nào**. Không có `estimator_source` thì sai số đo được không quy được cho nguồn nào — và đó chính là cách phát hiện ra rằng bộ tiêm nhiễu GPS không hề chạm tới đầu ra.

🔴 **Ngưỡng và mức tiêm nhiễu là MỘT cặp, không tách rời.** Cổng hiện tại: **RMSE < 0,5 m mỗi trục** *khi* GPS trôi σ=5 m τ=60 s và VIO bị suy giảm. Bản đầu chỉ ghi "< 0,3 m" mà không kèm điều kiện — và một con số không kèm điều kiện đo **không phải là cổng nghiệm thu**, đó chính là thứ đã phải phát biểu lại.

⚠️ **Căn frame:** hai chuỗi có gốc khác nhau (chỗ Gazebo spawn vs gốc EKF2). Độ lệch được **đo lúc drone còn đậu rồi giữ cố định**, nên mọi phân kỳ sau đó bị tính là sai số chứ không bị hấp thụ. Lệch yaw giữa hai frame sẽ lộ ra dưới dạng sai số **tăng dần theo khoảng cách tới gốc** — nên điều kiện A kiêm luôn vai trò kiểm tra căn frame.

Chi tiết số đo + hai điều kiện tiêm nhiễu → [`../../docs/package-status.md`](../../docs/package-status.md) §5.

Chạy được trên cả `uav_arena` (ngoài trời) và `uav_arena_indoor` (**không GPS, không la bàn**) — cả hai **PASS 3/3**.

| Ngưỡng | Giá trị |
|---|---|
| Sai số cao độ | ≤ 0.3 m |
| Sai số ngang | ≤ 0.5 m |
| Nhịp setpoint khi bay | ≥ 2 Hz |
| Failsafe | không được bật lần nào |
| Tắt động cơ sau lệnh hạ | trong 40 s |

## ⚠️ Ba điều phải biết

**1. Dọn node cũ trước khi chạy.** Node ROS2 sót lại từ lần trước sẽ **phục vụ binary cũ** và thắng lời gọi service — sửa code xong chạy vẫn ra lỗi cũ. `start_sim.sh` đã dọn sẵn; bài test in số instance gateway để phát hiện.

**2. `use_sim_time` đã BẬT (2026-08-10).** `/clock` do cầu Gazebo→ROS phát, và `start_sim.sh` tự bật cầu đó.

Bật tham số thôi thì **chưa đủ, mà còn nguy hiểm**: `now()` chuyển sang thời gian sim nhưng `create_wall_timer` vẫn chạy theo đồng hồ thực. Nếu sim chạy nhanh hơn thời gian thực, nhịp offboard 20 Hz *thực* có thể tụt xuống dưới 2 Hz *sim* → PX4 kích failsafe. Vì vậy cả ba timer đã chuyển sang `rclcpp::create_timer(..., get_clock(), ...)`.

**Cách kiểm lại (dùng được về sau):** tạm dừng Gazebo rồi xem `/uav/<id>/state/vehicle`. Timer theo đồng hồ sim thì **dừng hẳn**; timer theo đồng hồ thực thì vẫn chạy 10 Hz. Đo thật: 9.913 Hz → dừng → 9.903 Hz.

```bash
gz service -s /world/uav_arena/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 5000 --req 'pause: true'
```

**3. M5 tự sinh setpoint — và đó là PHÂN VAI ĐÃ CHỐT, không còn là nợ.** Bài test vừa *kiểm tra* vừa
*tự sinh setpoint*: nó phát `ControlCommand` vào **`/control/cmd_test`** — nguồn **TEST**, tức
`SOURCE_TEST = 1`, **mức ưu tiên THẤP NHẤT** trong bốn nguồn của trọng tài (`SOURCE_SAFETY = 4` cao
nhất). Nó chạy **song song** với `uav_navigation`, không đi qua navigator.

> ⚠️ **Sửa 2026-08-25:** đoạn này trước đây ghi `/control/cmd_mission`. **Sai, và sai theo hướng nguy
> hiểm** — nó khiến người đọc tưởng bài test tranh quyền với nguồn MISSION. Thực tế TEST là kênh thấp
> nhất, nên M5 **về mặt kiến trúc không thể** cướp quyền của mission, operator hay safety. (Đoạn "cờ
> `navigator`" ở trên vẫn luôn ghi đúng là TEST — README đã tự mâu thuẫn một thời gian.)

🔑 **Vì sao giữ nguyên (nợ #6 ĐÓNG 2026-08-25, chủ dự án ký):** P6 đã giải chuyện này bằng cách **THÊM**
chứ không **VIẾT LẠI**. Cổng **G-N1** (`scripts/verify_navigator.sh`) theo đúng đặc tả là *"tái lập M5
QUA action"* — Takeoff→Goto→Land, **cùng ngưỡng M5** (cao ≤0,3 · ngang ≤0,5 m), cùng model `uav0` — và
tiêu chí đạt của chính nó ghi *"**+ hồi quy M5 cũ vẫn PASS**"*. G-N1 **PASS 8/8 (2026-08-16)**.

| Bài | Tầng nó phán xử |
|---|---|
| `test/smoke_flight.py` (M5) | backend + trọng tài quyền — nguồn TEST, ưu tiên thấp nhất |
| `scripts/verify_navigator.sh` (G-N1) | tầng navigator: cùng bài bay, qua action |

🔴 **Cấm viết lại M5 để "chuyển setpoint sang navigator".** M5 là đường cơ sở hồi quy liên tục từ P3
(2026-08-07); đổi nó là **mất khả năng so sánh với mọi số đo lịch sử** mà không mua thêm vùng phủ nào —
vùng phủ đó G-N1 đã có. Rủi ro flaky còn lại (Python bơm 20 Hz, máy chậm tụt nhịp) vẫn có thật nhưng
chưa từng cắn: M5 PASS 3/3 ở mọi lượt hồi quy tới nay.
