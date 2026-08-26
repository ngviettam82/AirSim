# Ops Playbook — bài học vận hành đã trả giá (`scripts/`)

> R16 giới hạn ghi chú trong code còn **≤1 dòng/≤10 từ/tiếng Anh**. File này là nơi giữ phần lý do dài đã bị gỡ khỏi 27 script trong `scripts/` — không được mất, chỉ đổi chỗ ở. Mỗi mục: **triệu chứng → nguyên nhân → cách làm đúng**. Nguồn = script đã ghi nhận bài học đó (đọc script kèm playbook khi cần chi tiết lệnh).

---

## 1. Dừng & khởi động sim

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| Gõ lệnh `pkill -f '...' && ros2 launch ...` trực tiếp ở dòng lệnh thì **shell tự chết** (exit 15) | `pkill -f` khớp **toàn bộ dòng lệnh** của mọi tiến trình đang chạy — kể cả shell đang gõ lệnh đó, vì dòng lệnh của chính nó cũng chứa pattern | Luôn để các chuỗi `pkill -f` bên trong **một file script**, không gõ thẳng ở dòng lệnh tương tác. Khi gọi từ trong script, dòng lệnh của người gọi chỉ còn là tên file — không khớp pattern nữa. Đây là lý do `start_sim.sh`, `stop_sim.sh`, `view_world.sh`, `run_m5_regression.sh`… tồn tại dưới dạng file. |
| Node cũ vẫn trả lời service call dù đã build lại | Tiến trình sót từ lần chạy trước tiếp tục phục vụ **binary cũ** và thắng service call trước node mới | Trước khi start, dọn sạch theo thứ tự: `MicroXRCEAgent`, `px4_sitl_default/bin/px4`, `gz sim`, `parameter_bridge`, `image_bridge`, `make px4_sitl`, `sleep infinity`, rồi tới `uav_px4_backend/`, `ros2 launch uav_bringup`, `ros2 run uav_px4_backend` — có `sleep` giữa các đợt để tiến trình thoát hẳn (`start_sim.sh`). |
| `gz sim` không phản hồi SIGTERM, RTF của lần chạy sau bị nhiễm âm thầm | Server Gazebo bị "wedged" (kẹt) vẫn giữ GPU dù đã nhận lệnh dừng | Sau `pkill`, `pgrep` lại các tiến trình `gz sim`/`px4`/`MicroXRCEAgent` còn sống và `kill -9` chúng; nếu vẫn còn sau đó thì báo lỗi thay vì coi là đã dừng sạch (`stop_sim.sh`). |
| Mở world B nhưng GUI vẫn hiện world A | `gz sim <file>` **gắn vào server đang chạy sẵn** thay vì load file được truyền vào — world cũ hiển thị âm thầm dưới tên file mới | Luôn `pkill` mọi tiến trình `gz sim`/`gz-sim-server`/`gz-sim-gui` trước khi mở một world cụ thể (`view_bk_world.sh`, `view_world.sh`). |
| Model có sensor render (camera/lidar) không kịp spawn khi để PX4 tự gọi Gazebo | PX4 chỉ thử spawn **đúng 1 lần trong 1 giây** trừ khi `PX4_GZ_STANDALONE=1`; model nặng sensor không kịp lên trong 1s | Khởi động Gazebo độc lập (standalone) trước, để PX4 tự retry cho tới khi world sẵn sàng, thay vì để PX4 spawn 1 phát ăn cả (`start_sim.sh`). |
| Launch bị treo vài phút không rõ lý do | `make px4_sitl gz_<model>` build-và-chạy trong 1 lệnh — nếu cần rebuild, việc build biến thành compile ẩn nhiều phút bên trong bước launch, và nếu build lỗi thì để lại stack chạy dở dang | Build tách riêng trước (`make px4_sitl`) và **dừng ngay nếu lỗi**, chỉ chạy `gz_<model>` sau khi build sạch (`start_sim.sh`). |
| Log PX4 phình tới hàng triệu dòng | stdin là `/dev/null` khiến shell PX4 gặp EOF và vẽ lại liên tục | Cho PX4 shell một stdin **blocking** thay vì null: `sleep infinity | make px4_sitl gz_<model>` (`start_sim.sh`). |
| Script bay/đo bắt đầu quá sớm, PX4 chưa thật sự sống (còn đang compile) | `start_sim.sh` trả về ngay khi đã **launch xong**, không phải khi PX4 đã **chạy xong** | Chờ bằng chứng thật: poll `ros2 topic list --no-daemon` tới khi thấy `/fmu/out/vehicle_odometry`, có deadline (thường 420s), không dùng `sleep` cố định để đoán (dùng trong hầu hết script `run_*`/`verify_*`/`measure_*`). |
| Agent không tạo topic nào, không báo lỗi | Bản `MicroXRCEAgent` cài qua snap (v1.0.2) quá cũ so với PX4 v1.15, chạy được nhưng câm lặng | Luôn dùng bản tại `~/.local/bin/MicroXRCEAgent`, không dùng bản snap (`start_sim.sh`). |

---

## 2. Đo đạc & thống kê (RTF, tần số ảnh, độ chính xác)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| RTF (min/p50/max) đọc được dao động thất thường giữa các lần chạy cùng cấu hình (0.32 / 0.43 / 0.63 đo trên cùng 1 config) | Dưới tải render sensor, phân bố RTF là **bimodal**: đoạn dài ở 1.0 xen kẽ những bước chậm — median rơi vào mode nào đang chiếm đa số nên rất thất thường | Đọc RTF bằng **mean**, không phải p50/median — mean mới ổn định giữa các lần chạy và là số quyết định "có tin được phép đo bay hay không" (`sample_rtf.sh`, `measure_rtf_variants.sh`). |
| 🔴 `min` của `real_time_factor` đọc ra **0.10–0.15** ⇒ ai cũng mô tả thành *"những cú khựng sâu hiếm"*, rồi đi tìm nguyên nhân của một cú khựng không tồn tại | **`real_time_factor` là đại lượng của MỘT BƯỚC mô phỏng, không phải của cửa sổ.** Đo 2026-08-25 (`diagnose_rtf_stalls.sh`, `uav0_full`, 878 mẫu/89.8 s): cùng bản tin stats báo `real_time_factor` **min 0.151**, trong khi RTF suy từ chính `sim_time`/`real_time` của bản tin đó là **min 0.748**. Bản tin stats về đều như máy đếm nhịp — khoảng cách p50 **0.103 s**, **max 0.122 s** — nên **chưa bao giờ có gì bị chặn lâu**. Cái thật sự xảy ra là **chi phí TUẦN HOÀN**: 24.5% số quãng 100 ms chạy dưới 0.95×, **cách nhau p50 0.21 s**, khớp số học 4 cảm biến render 15/15/30/20 Hz **trùng tick mỗi 200 ms** | Đừng mô tả tail bằng `min` của trường đó. Muốn biết mô phỏng **có bị chặn không** thì đọc **khoảng cách giữa các bản tin stats**; muốn biết nó **chậm bao nhiêu** thì suy RTF từ `sim_time`/`real_time` trong cùng bản tin. `sample_rtf.sh` chỉ đọc trường báo sẵn nên **không phân biệt được hai chuyện đó** — dùng `diagnose_rtf_stalls.sh` + `rtf_stall_verdict.py` khi câu hỏi là *"vì sao chậm"*. |
| RTF tức thời đọc thấp dù world nhìn chung vẫn theo kịp thời gian thực | Một bước khựng luôn được bù lại bằng bước sau đó nhảy vọt trên 1.0 — mẫu tức thời không phản ánh trung bình thật | Đo thêm **RTF trung bình theo cửa sổ**: đọc `/clock` trước/sau một khoảng wall-clock cố định rồi lấy (Δsim-time)/(Δwall-time). Dùng `--field clock.sec --once` để tránh khớp nhầm `nanosec:`; **luôn kiểm tra giá trị đọc được không rỗng và có thay đổi** trước khi tin — echo rỗng khiến awk nhận 1 dòng trắng và lặng lẽ in ra 0.000 thay vì báo lỗi (`measure_rtf_variants.sh`). |
| `ros2 topic echo --once` (hoặc tương tự) trả về rỗng | Lệnh một-lần (`--once`) trên `ros2 topic echo` **không đáng tin** — có thể trả rỗng dù topic đang publish bình thường | Không dùng kết quả của một lần echo mà không kiểm tra rỗng; nếu cần số liệu chắc chắn, đọc nhiều mẫu hoặc dùng công cụ chuyên biệt (C++ probe) thay vì suy luận từ 1 lần đọc. |
| Không rõ RTF thấp đến từ camera/lidar hay từ cầu ROS bridge | Đo cả cụm cùng lúc không tách được biến số | Đổi **đúng một biến mỗi lần đo**: bật/tắt ảnh, có/không point cloud — 4 tổ hợp so sánh chéo thay vì đổi nhiều thứ một lượt (`measure_bridge_cost.sh`). |
| Không chắc số khung hình đo được có phản ánh đúng thực tế | Subscriber Python tự rớt khung hình, nên tần số đo được không phân biệt được "cầu bridge làm mất khung" với "phép đo làm mất khung" | Đo song song bằng **hai cách độc lập**: một consumer C++ và một Python trên cùng luồng, cùng lần chạy — hai kết quả khớp nhau thì tin bridge, lệch nhau thì lỗi nằm ở phép đo (`measure_image_rates.sh`). |
| `ros2 run ... --ros-args -p seconds:=30` báo lỗi khởi động | Tham số được node khai báo kiểu `double`, mà `ros2` parse `30` (không dấu chấm) thành số nguyên → lỗi cứng | Luôn truyền có dấu chấm thập phân cho tham số kiểu double: `30.0` (`measure_image_rates.sh`). |
| `ros2 topic list` / `ros2 node list` trả về rỗng dù stack đang chạy thật | Daemon ROS2 có thể trả danh sách rỗng/cũ dù tiến trình đang sống | Dùng `--no-daemon` cho mọi lần liệt kê dùng để **quyết định luồng chạy tiếp** của script (chờ PX4 lên, đếm node…) — dùng xuyên suốt các script `run_*`/`verify_*`/`measure_*`. |
| So sánh "điều kiện A vs B" (nhiễu tắt vs nhiễu bật) cho kết quả vô nghĩa | Không kiểm tra điều kiện B **thật sự** đã bật nhiễu — nếu file config sinh sai, "condition B" chạy giống hệt condition A mà không ai biết | Trước khi chạy điều kiện đã sửa config, đếm số cờ đã lật đúng như kỳ vọng (vd `grep -c 'enabled: true'` phải ra đúng 2), thất bại thì dừng ngay thay vì chạy tiếp một phép đo giả (`run_g2_accuracy.sh`). |
| So sánh độ chính xác định vị lệch bất thường | Để script **tự ước lượng** offset giữa 2 frame thay vì dùng số đo thật — cách này hấp thụ luôn phần trôi đã tích luỹ từ t=0 vào "offset", che mất sai số thật | Dùng offset đã **đo được** ở điều kiện chuẩn (vd điều kiện A đo ra z=-0.240, khớp `body_offset_z` trong config) làm tham số cố định cho điều kiện so sánh (`run_g2_accuracy.sh`). |
| Script chết ngay ở dòng `source /opt/ros/humble/setup.bash` | `set -u` (nounset) bật, mà `setup.bash` của ROS đọc biến (vd `AMENT_TRACE_SETUP_FILES`) **trước khi định nghĩa** nó | Không dùng `set -u` trong script có `source` ROS setup — chỉ dùng `set -o pipefail`. Áp dụng cho hầu hết `run_*`, `measure_*`, `verify_*`. |
| Ảnh chẩn đoán biến mất trước khi kịp xem (xảy ra 3 lần trong dự án) | Ảnh ghi vào `/tmp` — bị dọn/mất trước khi kịp mở | **Không bao giờ ghi ảnh chẩn đoán vào `/tmp`.** Ghi vào thư mục ổn định (vd `~/marker_debug/`), và **xoá ảnh của lần chạy trước** trước khi bắt đầu lần mới — ảnh cũ sót lại từng khiến script tự-chẩn-đoán kết luận nhầm "lỗi ở cảnh" cho một lần chạy mà nhận dạng thực ra đã đúng (`verify_marker_detector.sh`; cũng ghi ở `src/uav_perception/README.md`). |
| Một lần bay/detect fail, không biết lỗi ở node hay ở cảnh | Hỏi lại câu hỏi đó bằng cách bay lại thì không còn là đúng điều kiện cũ nữa | Chạy lại **đúng detector đó, offline**, trên **đúng khung hình** camera đã tạo ra trong lần chạy fail — trả lời được "cảnh không thấy được" hay "node không thấy" mà không cần bay lại (`verify_marker_detector.sh`). |
| Script one-off import `cv2` báo lỗi/khác hành vi so với node thật | Python nhặt nhầm bản `cv2` từ user-site, xung đột với bản trong workspace | Set `PYTHONNOUSERSITE=1` trước khi chạy đoạn Python one-off có `import cv2` (`verify_marker_detector.sh`). |
| Health monitor "trông có vẻ ổn" nhưng chưa ai biết nó có báo đúng khi có lỗi thật không | Monitor mới chỉ được test trên luồng khoẻ mạnh | Bài verify phải có **nửa fault-injection**: cố tình giết nguồn dữ liệu (vd kill image bridge) và xác nhận monitor phát hiện — health monitor chưa từng thấy lỗi là health monitor chưa được kiểm chứng (`verify_camera_health.sh`). |
| Không chắc một con số RTF đơn lẻ có đại diện được không | Một lần chạy từng dao động nội bộ 0.17–0.78 dù cấu hình không đổi | Sau mọi thay đổi renderer/môi trường, đo lại RTF và báo cả **min/p50/max/mean cùng lúc**, lặp lại nhiều lần (không tin 1 lần chạy) (`measure_rtf_variants.sh`, `verify_uav0_full.sh`). |
| So sánh 2 world cho sai số khác nhau, không rõ do world hay do lag khung hình | RTF thấp (do render sensor) tự nó có thể làm méo kết quả bay, gây nhầm lẫn với vấn đề của world | Cô lập biến bằng cách lặp lại so sánh với biến thể model **không có render sensor** (RTF giữ ~1.0) — loại được lag ra khỏi phương trình, sai số còn lại chắc chắn đến từ world (toạ độ, điều kiện GPS…) (`compare_flight_accuracy.sh`). |

---

## 3. Môi trường GPU / WSL

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| 🔴 **Hai tiến trình cùng máy, cùng `ROS_DOMAIN_ID`, thấy 0 topic của nhau** — mà cả hai đều sống | **Loopback của WSL hỏng — cả UDP lẫn TCP**, không riêng multicast. Kiểm bằng bài GỐC, đừng debug code của mình: `ros2 run demo_nodes_cpp talker` rồi `ros2 topic list --no-daemon` ở tiến trình khác — ra 0 là môi trường hỏng. Bằng chứng nó sâu hơn DDS (2026-08-18): `ros2 daemon stop` chết vì `TimeoutError: [Errno 110]` khi mở **XMLRPC qua TCP localhost** — *timeout* chứ không *refused* ⇒ gói đi vào hư không. Nghi gốc: `.wslconfig` có `networkingMode=mirrored` + `[experimental] hostAddressLoopback=true` | 🔴 **BỐN cách đã ĐO là KHÔNG cứu được — đừng thử lại:** `ROS_LOCALHOST_ONLY=1` · `sudo ip link set lo multicast on` (cờ lên thật mà vẫn `phát=8 nhận=0`, **và không sống qua restart WSL**) · **`wsl --shutdown`** (đã chạy đúng phép thử đó: boot sạch vẫn hỏng) · hồ sơ unicast 127.0.0.1 ([`config/fastdds_unicast_localhost.xml`](../config/fastdds_unicast_localhost.xml), 3 lượt harness sạch đều nhận=0). 🔴 **SHM-only cũng là ngõ cụt — xem dòng kế.** ✅ **CÁCH CHỮA (đo 2026-08-18):** bỏ **`networkingMode=mirrored`** (+ khối `[experimental] hostAddressLoopback`) trong `C:\Users\ASUS\.wslconfig` → **`wsl --shutdown`**. Mạng về NAT (`eth0` UP `172.22.x.x`, `loopback0` biến mất) ⇒ hết bệnh **không cần chỉnh gì ở tầng DDS**: talker/listener **9/9 · 8/8 · 8/8**, `topic list` **32 topic / 14 `/fmu/out`**, **M5 PASS 3/3**. 🔴 Cái giá: QGC mất đường cũ (mirrored vốn giữ vì nó). ⚠️ Từng chặn **mọi** cổng nhiều tiến trình; unit test cùng tiến trình **không** ảnh hưởng |
| 🔴 **Ép Fast DDS chạy SHM-only "chữa" được ROS↔ROS rồi GIẾT `/fmu/*`** | Agent link **Fast DDS 2.12.2** (`~/.local/lib/libfastrtps.so.2.12.2`), ROS Humble link **2.6.11**. RTPS-trên-UDP là chuẩn công khai nên hai bản liên thông; **layout segment SHM là nội bộ eProsima, không liên thông**. Đo 2026-08-18: SHM-only ⇒ `/clock` của bridge thấy được (ROS↔ROS sống) nhưng **toàn bộ `/fmu/out/*` biến mất** | **Không nối `fastdds_shm_only.xml` vào bất kỳ build nào** — một env hook như thế bắn ở mọi terminal và cắt đứt PX4↔ROS. File giữ ở `src/uav_bringup/config/` kèm cảnh báo đầu file, **chỉ dùng để cô lập lỗi discovery thuần ROS**. 🔑 Bài học: bài `talker/listener` **không đủ** làm cổng cho thay đổi tầng transport — phải chạy M5 vì nó là thứ duy nhất đi qua agent |
| 🪤 **`kill` một `ros2 run` KHÔNG giết node** ⇒ node mồ côi trả lời cho lượt đo sau = **dương tính giả** | `ros2 run` là bộ khởi động Python, node là tiến trình **con**. Ca thật 2026-08-18: cùng một cấu hình cho `nhận=0` rồi `nhận=17`; số 17 là talker mồ côi của lượt trước còn sống. Suýt kết luận ngược hoàn toàn | `setsid ros2 run ...` rồi `kill -TERM -$PGID` (PID âm = cả nhóm), **và đếm mồ côi trước/sau mỗi lượt** bằng `pgrep -f 'demo_nodes_cpp/(talker\|listener)'`. Đừng dùng `pkill -f` (bẫy tự sát). Muốn biết tiến trình ĐANG CHẠY thật sự mang biến gì: `tr '\0' '\n' < /proc/<pid>/environ` |
| 🪤 **`setsid nohup ... &` ⇒ `$!` KHÔNG phải PID của node** nên `kill "$!"` không giết được nó | Ca thật 2026-08-18: **12 `localization_mux_node` chồng chất**, mỗi lần chạy bench để lại một cái ⇒ bench đo một **dàn đồng ca** ở các pha khác nhau (2581 mẫu/24 s = ~108 Hz thay vì 10 Hz phát thật) | `pkill -f <đường/dẫn/binary>` **trước** khi khởi động, rồi **kiểm `pgrep -c` phải đúng 1**, không thì FATAL. Đếm trước khi đo là rẻ hơn nhiều so với một phép đo sai trông như đúng (mẫu ở `scripts/run_mux_fidelity.sh`) |
| 🪤 **Chạy `sim.launch.py` hai lần ⇒ bài bay TREO câm ở dòng đầu**, không lỗi, không timeout | **Hai bản stack chồng nhau** (dính 2026-08-20: `pgrep -c` ra **22** thay vì 11). Hai `px4_command_gateway_node` cùng phục vụ service `/uav/uav0/vehicle/arm` — **hai service server trùng tên là hành vi KHÔNG XÁC ĐỊNH trong ROS2** ⇒ lệnh arm không bao giờ trả về. Hai `offboard_session_manager` giẫm chân nhau, `offboard_status` báo `stream stopped`. Khác hẳn ca "12 mux" ở trên: ca đó cho **số đo sai trông như đúng**, ca này **treo hẳn** | **Trước mỗi lần bay, đếm:** `pgrep -c -f 'uav_px4_backend/\|uav_localization/'` phải ra **đúng 11**. Ra 22 ⇒ `pkill -f 'ros2 launch uav_bringup'` + `pkill -f 'install/uav_px4_backend/lib'` + `pkill -f 'install/uav_localization/lib'`, chờ, đếm lại phải ra 0, rồi mới khởi động một bản. 🔑 **`ros2 node list` KHÔNG dùng để kiểm được** — nó báo 4 rồi 9 trong khi thật là 11 (đếm thiếu dưới tải, kể cả `--no-daemon`) |
| 🪤 **`stop_sim.sh` không giết hết node ⇒ trọng tài mồ côi từ lượt trước sống lẫn với lượt sau, `/control/authority` dao động TEST↔OPERATOR giả, `command_selected` báo "2 publishers"** | `stop_sim.sh` liệt kê pattern theo TÊN GÓI (`uav_px4_backend/`, `uav_localization/`, `uav_navigation/`...) — **thiếu `uav_control_authority/`** vì package này thêm sau (P7.1) mà không ai cập nhật danh sách. `pkill -f 'ros2 launch uav_bringup'` không cascade SIGTERM tới con một cách đáng tin (đã thấy ở dòng "sim.launch.py hai lần" bên trên) nên node bị bỏ sót SỐNG SÓT qua nhiều lượt chạy, mỗi lượt tự phát heartbeat 2 Hz theo trạng thái nội bộ RIÊNG của nó → 2 (hoặc hơn) node cùng ghi `command_selected` = vi phạm chính bất biến P7 sinh ra để bảo vệ (dính 2026-08-20, đốt 2 lượt cổng G-CA2 trước khi bắt được) | **Đã vá**: thêm `'uav_control_authority/'` vào danh sách pattern của `stop_sim.sh`. Bài học chung: **mỗi khi thêm package mới có node chạy trong `sim.launch.py`, phải thêm pattern tương ứng vào `stop_sim.sh`** — không có gì tự động nhắc việc này. Kiểm tra nhanh trước khi tin một lượt cổng "sạch": `pgrep -af '[u]av_<package_mới>/'` phải ra rỗng sau `stop_sim.sh` |
| 🪤 **Biến môi trường đặt trong `wsl.exe -- bash -lc '...'` tới nơi thành RỖNG** — và mọi kết luận sau đó sai âm thầm | `$VAR` bị nuốt ở phía Windows trước khi tới bash. Chứng minh một dòng: `wsl.exe -- bash -lc 'export X=5; echo "X=$X"'` in ra **`X=`**. Ca thật 2026-08-18: `export ROS_DOMAIN_ID=95` không có tác dụng ⇒ probe chạy domain 0, thấy 0 publisher, tôi suýt kết luận nhầm là lỗi QoS/discovery | **Viết ra file script rồi chạy** (`bash scripts/x.sh`) — bên trong script biến hoạt động bình thường. Nếu buộc phải viết thẳng thì escape `\$`. Cùng họ với bài học "lồng quote trong `wsl.exe` hỏng liên tục". |
| Cần chạy lệnh root trong WSL từ phía Windows mà `sudo` đòi mật khẩu (Claude không gõ được) | `sudo` trong distro này **có** đòi mật khẩu | Gọi thẳng bằng user root: `wsl -u root -- <lệnh>` — vào `uid=0` **không cần mật khẩu**, không phải sửa sudoers. Nhớ `MSYS_NO_PATHCONV=1` nếu lệnh có đường dẫn Linux |
| `ros2 doctor --report` treo **>5 phút**, kéo sập cả script chẩn đoán | Nó dò mạng/middleware, mà loopback đang hỏng (dòng đầu bảng này) | **Đừng nhét `ros2 doctor` vào script chẩn đoán** có timeout. Cần thông tin RMW thì đọc thẳng: `ls /opt/ros/humble/lib/ \| grep librmw_` và `echo $RMW_IMPLEMENTATION` |
| RTF rơi xuống ~0.014 (chậm hơn ~70 lần) dù đã set adapter NVIDIA | `~/.bashrc` export `LIBGL_ALWAYS_SOFTWARE=1` toàn cục → mọi terminal tương tác render bằng CPU (llvmpipe), **đè lên mọi lựa chọn GPU khác** | Luôn `unset LIBGL_ALWAYS_SOFTWARE` **trước khi** set `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` — thiếu bước unset thì set adapter không có tác dụng gì (`view_world.sh`, `start_sim.sh`, `measure_rtf_variants.sh`). |
| Gazebo segfault khi bật sensor render, hoặc chạy nhầm GPU | Máy có 2 GPU (Intel iGPU + NVIDIA); không set adapter thì lớp D3D12 tự chọn Intel iGPU, và render sensor segfault trên iGPU | Luôn set `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` khi có sensor render (`start_sim.sh`, `view_world.sh`, `verify_bk_world.sh`, `view_bk_world.sh`). |
| Mở world thấy drone vô hình + hàng loạt lỗi "Unable to find file model://x500_base/…" | `GZ_SIM_RESOURCE_PATH` chỉ trỏ vào 1 trong 2 nơi cần thiết — model drone của dự án kế thừa mesh từ `x500_base` của PX4 | `GZ_SIM_RESOURCE_PATH` phải gồm **cả hai**: thư mục model của `uav_sim_gz` và `PX4-Autopilot/Tools/simulation/gz/models` (`view_world.sh`, `start_sim.sh`). |
| Phân vân dùng Docker Desktop hay Docker Engine trên WSL | Docker Desktop có điều khoản thương mại; Engine cài qua apt thì không, và chạy chung được với Ubuntu đang có ROS2+PX4+Gazebo | Cài **Docker Engine** qua apt theo docs.docker.com/engine/install/ubuntu, không cài Docker Desktop (`setup_docker_odm.sh`). |
| Sau khi `usermod -aG docker $USER`, `docker` vẫn đòi sudo | Đổi group thành viên không áp dụng ngay cho phiên WSL đang chạy | Cần khởi động lại WSL (`wsl --shutdown` gõ từ phía Windows), không chỉ mở shell mới (`setup_docker_odm.sh`). |
| Lo ngại `systemctl enable --now docker` không chạy trong WSL | WSL truyền thống không có systemd | Distro đang dùng ở đây đã có **systemd làm PID 1**, nên lệnh `systemctl` chạy bình thường, không cần workaround riêng (`setup_docker_odm.sh`). |
| Script dài chạy nền trong WSL chết ngay khi lệnh `wsl.exe` kết thúc — log 0 byte, không tiến trình nào sống, dù đã `setsid nohup ... & disown` bên trong | Phiên `wsl.exe` thoát là mọi tiến trình con trong đó bị giết; `nohup`/`disown`/`setsid` **lồng bên trong** không cứu được (dính 2026-08-16, cùng họ với bẫy `wsl.exe -- bash -lc '... &'` đã ghi trong memory 2026-08-13) | Giữ chính tiến trình `wsl.exe` sống suốt thời gian chạy: chạy script **foreground** qua `wsl.exe -- bash -c "script; echo EXITCODE:$?"` dưới `run_in_background` của công cụ Bash phía Windows, rồi poll log. KHÔNG nohup/disown lồng bên trong WSL. |
| `colcon build --packages-select` vài package thì `uav_px4_backend` **fail ngay ở configure**: `find_package(px4_msgs)` không thấy | Chỉ source `/opt/ros/humble/setup.bash` là chưa đủ — `px4_msgs` nằm trong `install/` của workspace. Càng dễ dính vì `verify_cleanup.sh` `cp -r` nguồn mỗi lần ⇒ mtime mới ⇒ cmake **re-configure mọi package**, chạy lại `find_package` (đã sinh ra một lần `build exit status: 2` với 0 test lỗi — triệu chứng khó hiểu) | Luôn `source install/setup.bash` (có guard `[ -f ]`) **trước** bước build. Đã đưa vào `verify_cleanup.sh`; đây cũng là điều kiện để nhánh `CLEAN=1` dùng được. |
| 🔴 Xoá "bản sao WSL" của `scripts/` thì **file gốc trên C: cũng mất** | `~/PX4_ROS2/scripts` **là SYMLINK** tới `/mnt/c/code/PX4_ROS2/scripts`, không phải bản sao. `rm -rf` trên đó xoá thẳng bản canonical (dính 2026-08-16, đã khôi phục) | Trước khi `rm` bất cứ thứ gì trong `~/PX4_ROS2/`, chạy `readlink -f` để biết nó trỏ đi đâu. Chỉ `src/` mới là bản sao thật (do `verify_cleanup.sh` đồng bộ); `scripts/` và các airframe là symlink hai chiều. |
| `wsl.exe -- bash /mnt/c/...` báo "No such file or directory" dù file tồn tại | **Git Bash (MSYS) tự dịch đường dẫn** `/mnt/c/...` thành `C:\mnt\c\...` trước khi trao cho wsl.exe | Đặt `MSYS_NO_PATHCONV=1` trước lệnh, hoặc gọi qua `wsl.exe -- bash -c '...'` với path nằm trong chuỗi quote (dính 2026-08-16). |
| Chạy xong script qua `wsl.exe`, quay lại đọc `/tmp/*` sau vài chục phút thì **file biến mất sạch**, `uptime` cho thấy WSL mới boot | Không còn phiên `wsl.exe` nào sống → **VM WSL tự tắt do idle**; lần gọi sau boot VM mới, `/tmp` (tmpfs) bị xoá trắng (dính **3 lần** 2026-08-16, mất trọn bằng chứng của hai lần chạy cổng) | Hai lớp: (1) đọc bằng chứng **trong cùng một phiên `wsl.exe`** với lệnh chạy; (2) 🔑 **script cổng phải TỰ IN log ra stdout ở bước teardown** thay vì chỉ trỏ tới `/tmp` — stdout đi vào capture phía Windows nên sống sót. Mẫu: `verify_navigator.sh` in 30 dòng cuối log node + các quyết định định vị trước khi thoát. 🆕 (3) **script phụ trợ dùng nhiều lần thì để ở `$HOME`, đừng để `/tmp`** — dính lại 2026-08-18: script build viết vào `/tmp` bốc hơi giữa hai lệnh, phải viết lại. `~/nav_build.sh` thì sống. |
| 🪤 **`wsl.exe -- bash -c '...'` làm RỖNG cả GÁN BIẾN LOCAL, không riêng `export`** (rộng hơn bẫy `export ROS_DOMAIN_ID` ở trên) | Đo 2026-08-21: `A=hello; echo "A=[$A]"` in `A=[]` — không liên quan gì tới env WSL, ngay cả `X=$HOME` cũng rỗng dù `echo "$HOME"` trực tiếp vẫn đúng | **Mọi script nhiều dòng có gán biến: viết ra FILE rồi chạy** `MSYS_NO_PATHCONV=1 wsl.exe -- bash /mnt/c/.../script.sh`. Không dùng `-c` với chuỗi có gán biến |
| 🪤 **`stop_sim.sh` gọi qua `wsl.exe -- bash -c "...stop_sim.sh..."` TỰ SÁT (exit 15)** | Chuỗi lệnh inline chứa tên/đường dẫn khớp chính pattern `pkill -f` bên trong script — cùng họ bẫy pkill-tự-sát §1, dễ dính lại khi thêm lớp `wsl.exe` (2026-08-21) | Luôn gọi qua file: `wsl.exe -- bash <path>/stop_sim.sh` — không inline |
| 🔑 **Chẩn đoán khoảng lặng publisher: ≥2 kênh ĐỘC LẬP cùng node (vd 5 Hz + 20 Hz) im lặng ĐÚNG CÙNG một khoảng** | Đó là khựng cấp hệ thống (Gazebo/WSL stutter), KHÔNG phải lỗi của publisher riêng lẻ — ca thật 2026-08-21: hold stream đo 14,35 Hz vì một khoảng lặng 2,5 s xuất hiện đồng thời trên cả `cmd_safety` lẫn `safety/state` | Bay lại một lượt để xác nhận trước khi kết luận lỗi sản phẩm (lượt 2 sạch 19,99 Hz). KHÔNG nới ngưỡng theo lượt bẩn |
| `colcon build` fail ở `ament_cmake_python_symlink_<pkg>` dù nguồn không đổi | Cache build-tree WSL hỏng: `build/<pkg>/ament_cmake_python/<pkg>/<pkg>` là **thư mục thật** thay vì symlink (di sản một lần build cũ, 2026-08-21) | `rm -rf` đúng thư mục đó rồi build lại — không phải lỗi code |
| 🪤 **Sửa một script `.sh` (`scripts/` symlink) TRONG LÚC một tiến trình bash khác đang chạy nó** ⇒ tiến trình đó báo `syntax error near unexpected token` cho một hàm nó CHƯA TỚI, dù file cuối cùng hoàn toàn hợp lệ | Bash đọc/parse script THEO DÒNG khi thực thi từ file (không nhất thiết nạp trọn vẹn lên bộ nhớ trước) — sửa file khi tiến trình đang giữa chừng có thể khiến nó đọc trúng một trạng thái file NỬA VỜI cho một hàm ở XA phía dưới, dù hàm ĐANG CHẠY lúc đó (đã đọc từ trước khi sửa) không hề bị ảnh hưởng và tiếp tục chạy đúng. Dính 2 lần P10.9b: sửa `verify_observability.sh` khi `d0-baseline` rồi `o4-gate` đang chạy nền — cả 2 lần round ĐANG CHẠY vẫn ra kết quả đúng, chỉ có hàm CHƯA GỌI TỚI (`round_o4_gate`) báo lỗi parse | `bash -n script.sh` xác nhận file CUỐI CÙNG hợp lệ (không phải bằng chứng cho **lượt đang chạy**). Quy tắc an toàn: **không sửa một script đang có tiến trình nền dùng nó** — đợi tiến trình đó thoát (`pgrep -f 'script.sh'`) rồi mới sửa; biến top-level (`VAR=value`) đã gán lúc script khởi động sẽ KHÔNG đổi theo file sửa sau, dù hàm gọi sau vẫn đọc được thân hàm mới |
| 🪤 **`diagnostic_msgs/DiagnosticStatus.level` là ROS `byte`, rclpy trả về `bytes` (không phải `int`)** ⇒ `status.level == 0` **không bao giờ đúng**, lọc "non-OK" bằng phép so sánh đó bắt TOÀN BỘ (kể cả OK) mà không báo lỗi gì | `byte` (khác `uint8`) là kiểu ROS2 riêng biệt ánh xạ sang Python `bytes` 1-byte trong `rclpy`, không tự ép kiểu về `int` như các trường số nguyên khác. Dính thật ở `preflight_baseline_capture.py` (P10.9b, D0): script chạy "thành công", in `NON_OK_RECORDS=1854` trông hợp lý, nhưng khi soát dữ liệu thấy CẢ mã đang OK (`b'\x00'`) lẫn ERROR (`b'\x02'`) đều lọt qua | Ép kiểu tường minh trước khi so sánh: `level_int = status.level[0] if isinstance(status.level, (bytes, bytearray)) else int(status.level)`. Không tin một script "chạy không lỗi, số ra hợp lý" — phải soát mẫu dữ liệu THẬT (vd `Counter` theo `(source,child,level)`) trước khi dùng số đó cho bất kỳ quyết định nào, đặc biệt khi trường đó nạp cấu hình an toàn (waiver) |

---

## 4. Bẫy công cụ ROS2 / Gazebo CLI

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| `pkill -f 'bin/gz'` đôi khi bỏ sót tiến trình | Match theo từ một người sẽ gõ, không phải theo path thật của binary | Match `pkill -f` theo **đường dẫn binary thật** (vd `bin/gz`, `gz-sim-server`, `gz-sim-gui`), không theo từ ngữ cảm tính (`view_world.sh`). |
| Spawn thêm entity lúc runtime (`gz service .../create`) làm gãy bridge | Ngộ nhận mọi entity spawn runtime đều cần khởi động lại bridge | Bridge chỉ **fail re-register** khi entity mang sensor bị spawn runtime; spawn một entity **không sensor** (vd bảng marker ArUco) là an toàn, không cần đụng tới bridge (`verify_marker_detector.sh`, `debug_marker_visible.sh`). |
| Node bay/bám điểm hoạt động tốt trên `uav0_nav` nhưng trên **`uav0`** thì cao độ sai ~0,5 m, không bao giờ "đạt đích", timeout dù RTF ~1,0 | `config/bridge_uav0.yaml` bridge **DUY NHẤT `/clock`** — `uav0` **không có ground-truth odometry**, nên `vio_adapter_node` câm, mux **rơi về GPS** (SITL: σ_ngang 0,2 m, **σ_đứng 0,5 m**). Mọi ngưỡng đạt <0,5 m nằm **dưới sàn nhiễu của chính đầu vào** (dính 2026-08-16, đốt 3 lần chạy sim + một workflow chẩn đoán) | Bài nào **chấm điểm bằng `odometry_fused`** phải chạy trên model có nguồn vision: `uav0_nav` / `uav0_track` / `uav0_full`. `uav0` chỉ hợp cho bài đóng vòng trên `odometry_raw` (M5 `smoke_flight.py`). Tự kiểm nhanh: `grep -c vio_odometry_raw config/bridge_<model>.yaml` phải ra 1. |
| Probe rclpy báo `action server never appeared` dù node đang chạy và **đã in "ready"**; `get_node_names()` chỉ thấy chính probe | `ActionClient.wait_for_server()` **chặn mà KHÔNG spin node** → graph cache của probe không bao giờ được làm mới, nên server có xuất hiện nó cũng không thấy. Cùng probe đó nhận topic bình thường (các wait khác có spin) nên triệu chứng trông y hệt "node không đăng ký được" (dính 2026-08-16, đốt một lần chạy cổng) | Chờ bằng vòng lặp **có spin**: `while ...: if client.server_is_ready(): break; rclpy.spin_once(node, 0.05)`. Đối chứng dương: chạy kèm một node đã qua cổng — nếu probe cũng không thấy nó thì lỗi ở probe, không phải ở node đang xét. |
| Ảnh chụp được toàn màu trắng trơn, nghi node detect hỏng | Texture của asset load lỗi cũng render ra trắng trơn — **giống hệt** triệu chứng "detector hỏng" nhìn từ bên ngoài | Trước khi debug node, chụp khung hình có/không có vật thể để loại trừ lỗi scene/asset (`debug_marker_visible.sh`). |

*(Bẫy `pkill -f` tự giết chính shell gọi nó → xem mục 1, đây là lý do cốt lõi khiến nhiều thao tác trong repo này phải nằm trong file `.sh` thay vì gõ trực tiếp.)*

---

## 5. Photogrammetry / ODM pipeline

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| Cần tạm dừng job ODM đang chạy nhiều giờ mà không muốn mất tiến độ | — | `docker pause`/`unpause` dùng cgroup freezer: mọi tiến trình dừng nguyên trạng, CPU được nhả ra, resume đúng chỗ đã dừng. **Không sống sót qua việc tắt WSL** — nhưng depth map ghi liên tục ra đĩa nên kill cứng chỉ mất đúng phần map đang xử lý dở (`odm_pause.sh`). |
| Chạy lại ODM sau khi crash thì chết ngay ở bước matching, thiếu file `.npz` | ODM coi bước "trích đặc trưng xong" là **thư mục output tồn tại**, không phải đã trích đủ — chạy dở dang để lại thư mục nửa vời, lần sau ODM tưởng đã xong và bỏ qua thẳng | Trước khi chạy, đếm số `.features.npz` so với số ảnh; nếu cache dở dang (`npz < ảnh`) thì ép `--rerun-from opensfm` — OpenSFM vẫn tự bỏ qua ảnh đã có, không làm lại từ đầu (`run_odm_sfm.sh`). |
| Container ODM bị OOM-kill giữa chừng | ODM cấp ngân sách RAM ~1GB/thread cho mỗi 2MP ảnh; ảnh khảo sát ở đây 16.8MP | Giới hạn `--max-concurrency 2` (`run_odm_sfm.sh`, `run_odm_dense.sh`, `run_odm_strip_halves.sh`). |
| Chiều cao công trình đo ra gần bằng 0 trên DTM | `--smrf-window` (bán kính bộ lọc tách mặt đất/vật thể) mặc định 18m của ODM nhỏ hơn các block ~40m ở khuôn viên khảo sát → mái nhà bị coi là mặt đất, chiều cao "phẳng" về 0 | Đặt `--smrf-window 30` (đủ phủ công trình rộng tới 60m); khuôn viên gần phẳng nên cửa sổ rộng ít làm mất chi tiết địa hình. **Luôn verify trên DTM** trước khi tin bất kỳ số liệu chiều cao nào (`run_odm_dense.sh`, `run_odm_strip_halves.sh`). |
| Không chắc nên đặt `--dem-resolution` bao nhiêu | Nhầm giữa "độ phân giải ảnh gốc (GSD)" và "độ phân giải bài toán cần" | Đặt theo nhu cầu hạ nguồn thực tế (ở đây DSM cần ~0.25m), không theo GSD gốc của ảnh (`run_odm_dense.sh`). |
| Việc chia đôi bộ ảnh để dựng 2 lần độc lập (kiểm tra chéo) tốn gấp đôi dung lượng đĩa | Copy file thay vì link | Dùng **hard link**, không copy — ảnh gốc và 2 nửa cùng nằm trên 1 filesystem, hard link không tốn thêm dung lượng (`run_odm_strip_halves.sh`). |
| Danh sách tên file (soạn/sửa trên Windows) đọc trên WSL bash thì không tìm thấy file nào | Mỗi dòng mang theo ký tự `\r` cuối dòng do khác biệt xuống dòng Windows/Unix | Luôn `tr -d '\r'` danh sách trước khi dùng làm input cho vòng lặp bash — từng gây lãng phí nguyên 1 lần chạy (`run_odm_strip_halves.sh`). |
| Một lần trích đặc trưng chết với "Unable to load image", đổ oan cho flight plan | Một file ảnh copy bị cắt cụt (silently truncated) mà không ai biết | Trước khi chạy ODM, verify **từng file** đã copy: khớp kích thước byte + đúng magic bytes JPEG (SOI `ffd8` đầu file, EOI `ffd9` cuối file) (`stage_survey_images.sh`). |
| Không biết job ODM đang chạy tới đâu, còn bao lâu | Đọc log tail không cho biết đang tiến triển hay đã treo | Đo **tốc độ tăng** số file `.npz` trong một cửa sổ thời gian cố định (vd 30s) để suy ra ảnh/giây và ETA, thay vì đoán từ log (`odm_status.sh`). |
| Lo lắng khi job ODM dừng giữa chừng, sợ mất hết | Không biết sản phẩm nào đã ghi bền vững ra đĩa | Kiểm tra danh sách checkpoint bền vững đã có sẵn trên đĩa (sparse reconstruction, dense cloud, mesh, DSM/DTM, orthophoto…) trước khi hoảng — nhiều bước sống sót độc lập với bước sau nó (`odm_dense_status.sh`). |

---

## 6. Đặc thù từng world

| World | Bẫy | Cách làm đúng |
|---|---|---|
| `uav_arena_bk` | Gốc toạ độ `(0,0,0)` nằm **bên trong** `tree_143` và sát `bk_019` — drone spawn ở gốc thì kẹt trong vật cản ngay từ đầu | Spawn ở pose đã kiểm chứng: `PX4_GZ_MODEL_POSE=-9.7,-94.9,0.25,0,0,0` (`run_g9_regression.sh`, `compare_flight_accuracy.sh`). |
| `uav_arena` | — | Giữ làm **baseline mặc định**: world trơ, không texture/scenery — để một lần bay fail có thể đổ lỗi cho code chứ không phải cho nội dung world (`start_sim.sh` và nhiều script `run_*`/`verify_*` dùng làm mặc định). |

---

## 7. Công cụ tự kiểm — chứng minh test/cổng KHÔNG rỗng

> Vì sao có nhóm này: dự án đã hai lần phát hiện một bộ test xanh mà **chưa từng chứng minh được gì** — plant giả bám gần như hoàn hảo nên dây xích chưa bao giờ kích hoạt (P3), và cổng "roof-collapse" báo PASS trong khi đo sai đối tượng. **Một test chưa từng ĐỎ là một test chưa được kiểm chứng.** Các script dưới đây là cách làm cho nó đỏ theo yêu cầu.

| Script | Trả lời câu hỏi nào | Dùng khi |
|---|---|---|
| `fault_inject_source_channel.sh` | Phá `uav_localization` (kênh nguồn định vị) **có chủ đích** rồi kiểm test có đỏ không | Trước khi tin bộ test của `uav_localization`; sau mỗi lần sửa `SourceChannel` |
| `fault_inject_world_model.sh` | Như trên cho `uav_world_model` | Sau khi sửa chuỗi transform / cộng bất định |
| `stress_source_channel.sh [runs] [load_procs]` | Test có sống dưới **tranh chấp CPU** đúng kiểu `colcon test` chạy song song không | Khi một test chỉ đỏ lúc chạy cả workspace mà chạy riêng thì xanh |
| `repro_world_model_flake.sh [runs] [load] [filter] [copies] [isolate]` | Flake đến từ **CPU** hay từ **DDS**? `isolate=1` cấp domain riêng cho mỗi bản sao ⇒ chỉ còn tranh chấp CPU | Săn flake; đây là công cụ đã dùng để chốt R21 |
| `audit_comments.sh` | Tỉ lệ ghi chú từng package + khối ghi chú dài (soát R16) | Trước khi đóng một package |
| `check_doc_links.sh` | Mọi link markdown tương đối có trỏ tới file thật không | Sau mỗi lần đổi tên/di chuyển tài liệu — bản đồ tài liệu ở `CLAUDE.md` là cách duy nhất tìm ra mọi thứ (R9), link chết = mất tri thức |
| 🆕 `check_px4_msgs_boundary.sh` | **R1/R25:** file nào ngoài `uav_px4_backend` đang cắt ranh giới `px4_msgs`? | Trước khi đóng bất kỳ package nào. Có **allowlist khai báo** (`px4_msgs_boundary_allowlist.txt`) — mỗi dòng là *một chỗ nữa phải sửa khi PX4 đổi API*, nên thêm dòng là quyết định thiết kế. Script **cũng bắt entry allowlist đã mục** (trỏ tới file không còn cắt ranh giới nữa). Đối chứng dương đã chạy: tiêm 1 file `import px4_msgs` ⇒ FAIL(1), gỡ ⇒ PASS |
| 🆕 `check_dds_profile_sim_only.sh` *(2026-08-25)* | **Profile DDS large-samples có còn là SIM-ONLY không?** Ai đang tham chiếu `fastdds_large_samples` / `FASTRTPS_DEFAULT_PROFILES_FILE` | **Trước khi đóng P11.2 (`real.launch.py`) và bất kỳ lúc nào thêm file phía real.** Vách 549 408 B là mặc định của **thư viện Fast DDS**, không của mô phỏng ⇒ máy thật *có lẽ* cũng cần — nhưng cổng R0 đo trên **loopback WSL** với `MicroXRCEAgent`, còn `useBuiltinTransports=false` thay transport của **mọi** participant đọc profile, kể cả đường tới flight controller. Mang số qua nền truyền khác là **R31**. Script **gọi đích danh `real.launch.py` / `start_real.sh`** để cắn đúng ngày file đó xuất hiện (đường lây khả dĩ nhất: copy từ `sim.launch.py`). Allowlist khai báo + bắt entry đã mục. **Đối chứng dương 4/4** (`selftest_check_dds_profile_sim_only.sh`, chạy trên cây nháp nên không bao giờ để lại `real.launch.py` sót): cây hiện tại PASS · real-side ⇒ FAIL · file không khai ⇒ VIOLATION · entry mục ⇒ STALE |
| 🆕 `check_test_domain_isolation.sh` *(viết lại 2026-08-24)* | **R20:** test nào tạo participant ROS mà **không** có `ROS_DOMAIN_ID` riêng? | Trước khi đóng package. Nay **derive cả package lẫn domain** (trước chép tay 5 package / 5 domain ⇒ mù với 96/97/98/99). Có `RESULT: PASS/FAIL` + exit code, trước đây chỉ in ra rồi thôi. Đối chứng dương 4/4 trên workspace giả: thiếu-domain ⇒ FAIL · có-domain ⇒ PASS · **chỉ nhắc `rclcpp::init` trong ghi chú ⇒ không bắt oan** · chưa-build ⇒ FAIL |

🔴 **`trim_bringup_test_comments.sh` — ONE-OFF ĐÃ DÙNG XONG, ĐỪNG CHẠY LẠI.** Nó ghi đè file trong `src/uav_bringup/test/` bằng bảng thay thế cứng của đợt dọn 2026-08-14. Chạy lại trên code đã đổi là **ghi đè mù**. Giữ lại chỉ để tra cách làm.

### 🪤 `colcon test-result --all` cộng lẫn HAI đơn vị — đừng dán thẳng dòng `Summary:` vào tài liệu

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| Tổng số test trong tài liệu (**318**) không bao giờ khớp tổng các con số per-package (72+63+55+34+71 = **295**), sửa chỗ này thì lệch chỗ kia | `colcon test-result --all` liệt kê **cả hai tầng**: `Testing/*/Test.xml` = số **test target** của ctest (23), rồi `test_results/*/*.gtest.xml` = số **test case** bên trong chính các target đó (295). Dòng `Summary:` cộng tất cả ⇒ **318 = 295 + 23**, mỗi target bị đếm hai lần | Báo cáo **"N case / M target"**, không dán dòng `Summary:`. Đếm case = khối `TEST*` được CMake đăng ký, chéo kiểm dòng `[==========] N tests` trong log. Hai cách phải ra cùng số. |

*(Không có quy ước này, riêng `uav_navigation` đã bị ghi **49 · 61 · 66** ở ba tài liệu khác nhau trong khi số thật là **63**.)*

🪤 **Bẫy anh em (dính 2026-08-20):** trong một file `.gtest.xml`, thuộc tính `tests="N"` xuất hiện **hai lần** — nút gốc `<testsuites>` và `<testsuite>` — nên `grep -o 'tests="[0-9]*"'` rồi cộng là **báo gấp đôi** (`verify_workspace.sh` từng báo 530 khi số thật 265). Sửa: `grep -m1` (mỗi file lấy match đầu). Chéo kiểm bắt được ngay: tổng case + tổng target phải bằng đúng dòng `Summary:` của `colcon test-result`.

### 🪤 Sync Windows→WSL sinh "Clock skew detected. Your build may be incomplete" (2026-08-24)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| `colcon build` báo *"N packages had stderr output"* nhưng **không có warning trình biên dịch nào**; mở `log/latest_build/<pkg>/stderr.log` thấy toàn `gmake[2]: Warning: File '…/progress.make' has modification time 1.9 s in the future` + `Clock skew detected. Your build may be incomplete.` | `cp -r` từ `/mnt/c` (NTFS) sang ext4 mang theo mtime **lệch về tương lai** vài giây so với đồng hồ WSL. **Không vô hại như trông thấy**: make so mtime nguồn với object, một nguồn "mới hơn chính nó" có thể bị **bỏ qua không biên dịch lại** — đúng nghĩa câu "build may be incomplete" | Sau mỗi lần `cp -r` sang cây build, **restamp**: `find "$WORKSPACE/src/$pkg" -exec touch {} +` (đã cài trong `verify_cleanup.sh`). Restamp lệch về phía **build lại thừa**, không phải phía bỏ sót. 🔑 Đây là họ hàng gần của ca 2026-08-24 *"cây build WSL đang đo BẢN CŨ"* — cùng một hậu quả (chạy cổng trên binary không phải cái mình nghĩ), khác đường vào |

### 🪤 Topic LATCH + chỉ-phát-khi-ĐỔI: subscriber VOLATILE không thấy gì SUỐT ĐỜI (2026-08-24)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| Một cổng báo *"topic X never arrived"* trong khi `ros2 topic echo X` **có dữ liệu**, và node phát hoàn toàn khoẻ | Topic phát **`TRANSIENT_LOCAL` + chỉ khi giá trị ĐỔI** (vd `state/estimator_source`: mux công bố **một lần** lúc chọn nguồn rồi im). Subscriber khai **VOLATILE** chỉ nhận mẫu MỚI ⇒ vào sau lần công bố đó là **không nhận được gì, mãi mãi** — không lỗi, không cảnh báo | Subscriber phải **khớp durability của publisher** (`TRANSIENT_LOCAL`) thì mới nhận được mẫu chốt-giữ. Danh sách đầy đủ topic latch: [`qos-policy-v1.md`](qos-policy-v1.md) §3 |
| `ros2 topic echo --once` trên topic latch in ra giá trị **cũ nhất** chứ không phải mới nhất | `KEEP_LAST(10)` + TL giữ tới 10 mẫu; `--once` lấy mẫu đầu hàng đợi | Đọc nhiều mẫu rồi lấy **cuối**, hoặc dùng script chuyên dụng (`preflight_light.sh` cho đèn go/no-go) |

🔑 **Ca thật:** G4 lượt 1 trả `FAILED TO MEASURE ×4` vì đúng lỗi này; `adjudicate_mux_when_parked.sh` phân xử được ngay — `vio_status.is_valid=true` + `localization_status detail=vio` ⇒ **sản phẩm lành, harness sai**. Họ hàng với bẫy `/state/system_health` đã ghi ở §12.

## 8. Sửa file bằng script — hai cái bẫy đã trả giá cùng một ngày (2026-08-19)

### 8z. 🔴 Vá file `.sh`/`.py` bằng Python **trên Windows** biến CẢ FILE thành CRLF (2026-08-25)

| Triệu chứng | Nguyên nhân | Cách đúng |
|---|---|---|
| Sau một bản vá một dòng, **cả script chết trong WSL** (`\r: command not found`, hoặc shebang không chạy). Dính **hai file cùng lúc**: `check_px4_msgs_boundary.sh` và `verify_workspace.sh` | Python trên Windows mặc định `newline=None` ⇒ **mọi** `\n` khi GHI thành `\r\n`, kể cả những dòng không hề đụng tới. Đọc cũng dịch ngược nên so chuỗi vẫn khớp — bản vá "thành công" | **Ép `newline` ở CẢ HAI đầu**: `io.open(p, 'r', encoding='utf-8', newline='')` và `io.open(p, 'w', encoding='utf-8', newline='\n')`. File vốn CRLF (vd `CMakeLists.txt` của repo này) thì **phát hiện rồi trả lại đúng dạng cũ**, đừng ép LF cho mọi thứ |
| 🔴 **Và hai phép đo ĐẦU TIÊN để chẩn đoán nó cũng sai** | `grep -c $"\r"` bị **locale dịch** nên không đếm CR thật; `cut -c1-45` **cắt mất đuôi** `"with CRLF line terminators"` của `file -b` | Kiểm CRLF bằng `file <f>` đọc **trọn dòng**, hoặc `grep -c $'\r' <f>` (nháy đơn ANSI-C, không phải `$"..."`). Bài học chung: **công cụ chẩn đoán cũng phải được đối chứng** — một phép đo sai về lỗi định dạng nhìn y hệt "không có lỗi" |

### 8a. 🔴 `open(p, 'w')` CẮT CỤT file TRƯỚC khi lỗi encoding nổ

| Triệu chứng | Nguyên nhân | Cách đúng |
|---|---|---|
| Chạy script sửa file, thấy `UnicodeEncodeError`, tưởng "không sao, nó fail rồi" — **file biến thành rỗng 0 byte** | `io.open(p, 'w')` **truncate ngay lúc mở**. Lỗi xảy ra trong `.write()`, tức **sau khi** nội dung cũ đã mất | Ghi ra **file tạm rồi `os.replace`**. Thao tác đổi tên là nguyên tử — hỏng giữa chừng thì file gốc còn nguyên |

```python
tmp = p + '.tmp'
with io.open(tmp, 'w', encoding='utf-8') as f:
    f.write(s)
os.replace(tmp, p)   # nguyên tử; hỏng trước đó thì file gốc còn nguyên
```

Ca thật: mất nguyên một tài liệu đề xuất ~200 dòng, phải viết lại từ đầu.

### 8b. 🔴 Escape `\uXXXX` cho emoji trong Python là **surrogate**, không dịch được ra UTF-8

Viết `"\ud83d\udd34"` trong mã Python cho ra **hai surrogate lẻ**, và `.write()` sẽ nổ
`UnicodeEncodeError: surrogates not allowed`. Dùng **ký tự thật** (`🔴`) hoặc escape **một điểm mã**
(`\U0001F534`). Đây chính là thứ kích hoạt 8a.

### 8c. `pkill -f` tự giết chính mình khi chạy qua `wsl.exe -- bash -lc "..."`

`pkill -f 'python3 listen.py'` khớp luôn **dòng lệnh của `bash -lc`** (vì chuỗi mẫu nằm trong đó)
⇒ shell tự sát, tool báo **exit code 15** không kèm log. Thêm `[.]` vào mẫu cũng vô ích vì mẫu vẫn
xuất hiện nguyên văn trong dòng lệnh.

**Cách đúng:** giết theo **cổng** (`fuser -k 4560/tcp`) hoặc theo **tên chính xác** (`pkill -x px4`),
hoặc lấy pid trước rồi `kill` ở lượt gọi sau.

**Biến thể dương tính giả (2026-08-20):** `pgrep -f <chuỗi>` chạy qua `wsl.exe -- bash -c '...'`
cũng tự khớp chính dòng lệnh của `bash -c` ⇒ báo *"tiến trình đang chạy"* khi thật ra không có
(ca thật: tưởng `ga0_try_arm.py` còn chạy). Với `pgrep` thì ngoặc vuông phá được tự-khớp:
`pgrep -f '[g]a0_try_arm'` — regex vẫn khớp tiến trình thật, còn chuỗi mẫu nguyên văn thì không —
hoặc chắc nhất là `pgrep -x <tên>`. *(Vụ `pkill` 08-19 ở trên đo được `[.]` không cứu — lớp trích
dẫn khác nhau; nghi ngờ thì đừng tin `-f`: dùng `-x`, cổng, hoặc pid.)*


## 9. Chạy build Windows (`.cmd`/`.bat`) từ phiên làm việc — 3 bẫy liên tiếp (2026-08-19)

Ca thật: gọi `build.cmd` của AirSim, **thất bại 3 lần với 3 nguyên nhân khác nhau**, mỗi lần đều
**exit code 0 hoặc 1 kèm thông báo đánh lạc hướng**. Ghi lại để không mất thêm lượt nào.

| # | Triệu chứng | Nguyên nhân thật | Cách đúng |
|---|---|---|---|
| 1 | `cmd.exe /c "..."` chạy xong **exit 0** nhưng **không làm gì**, chỉ in banner Windows | Git Bash (MSYS) **dịch `/c` thành đường dẫn `C:\`** trước khi truyền cho `cmd.exe` ⇒ `cmd` mở shell tương tác rồi thoát | Đừng gọi `cmd.exe` qua Bash. **Dùng công cụ PowerShell** và gọi `& "duong\dan\file.bat"`. (Hoặc `MSYS_NO_PATHCONV=1`, nhưng dùng PowerShell sạch hơn) |
| 2 | `'build.cmd' is not recognized` **dù `cd` đã đúng thư mục** | Biến môi trường **`NoDefaultCurrentDirectoryInExePath=1`** đang bật ⇒ `cmd` **không tìm lệnh trong thư mục hiện tại** | Trong file `.bat` bọc ngoài: `set "NoDefaultCurrentDirectoryInExePath="` **trước** khi gọi. Script build kiểu cũ hay gọi `.bat` anh em bằng **tên trần** (`check_cmake.bat`) nên bắt buộc phải có |
| 3 | `'vswhere.exe' is not recognized` từ trong `vcvars64.bat` | Thư mục `Installer` của VS không nằm trong PATH | `set "PATH=C:\Program Files (x86)\Microsoft Visual Studio\Installer;%PATH%"` |

🔑 **Bài học chung:** đừng gọi trực tiếp một dòng lệnh Windows dài từ phiên — **viết một file `.bat`
bọc ngoài** dựng đủ môi trường, in ra `%CD%` và `%VisualStudioVersion%` để mỗi lần hỏng còn biết
hỏng ở đâu. Cùng họ với §8: *thứ chạy được là file, không phải chuỗi đi qua ba lớp trích dẫn.*


### 🪤 Việc chạy DÀI trong WSL gọi từ Windows: `nohup &` và `setsid` đều KHÔNG cứu (2026-08-24, dính 2 lần)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| `wsl.exe -e bash -lc '... &'` báo "started, pid=NNN" nhưng **log chưa từng được tạo**, tiến trình biến mất | Tiến trình con nền **chết cùng phiên WSL** khi `wsl.exe` thoát | Đừng dùng `&` bên trong `wsl.exe` |
| Dùng `nohup setsid ... &` thì sống sót **vài phút** rồi cũng mất; `ps -eo etime` cho thấy **`/sbin/init` ELAPSED chỉ 21 giây** | 🔴 **WSL tắt cả DISTRO** khi không còn phiên nào giữ nó — `setsid` chỉ tách khỏi session, **không giữ distro sống**. Việc đang chạy bị mang theo | **Giữ một tiến trình `wsl.exe` ở FOREGROUND** và để harness theo dõi nó (`run_in_background` của công cụ Bash). Đó là cách M5 · G-N5 · `verify_workspace` chạy được tới cùng |
| Việc dài "xong" nhưng số đo vô nghĩa | Distro restart giữa lượt đo ⇒ đo trên môi trường khác lúc bắt đầu | Kiểm `ps -eo etime -p 1` — `/sbin/init` mới khởi động nghĩa là **mọi giả định về tiến trình đang chạy đều hết hiệu lực** |

🔑 **Đọc cùng với §7:** một lượt đo dài bị distro cắt ngang **không báo lỗi** — nó chỉ đơn giản không có kết quả, và rất dễ bị đọc thành "chưa xong".

## 10. 🔴 `Documents` trên máy này bị OneDrive chuyển hướng — có HAI thư mục cùng tên (2026-08-19)

| | |
|---|---|
| **Triệu chứng** | Ghi `~/Documents/AirSim/settings.json`, khởi động sim, sim vẫn dùng **cấu hình cũ hoàn toàn khác** (xe `Drone1`/`SimpleFlight`, gốc Zurich) — và **không có file nào trên đĩa** chứa nội dung đó khi tìm dưới `%USERPROFILE%\Documents` |
| **Nguyên nhân** | `[Environment]::GetFolderPath('MyDocuments')` = **`C:\Users\ASUS\OneDrive\Documents`**. Ứng dụng Windows dùng shell folder API nên đọc bản OneDrive; còn `~/Documents` trong Git Bash và `os.path.expanduser("~")` của Python trỏ tới **`C:\Users\ASUS\Documents`** — một thư mục **khác**, cũng tồn tại, cũng có `AirSim/` bên trong. Hai bản, chỉ một bản được đọc |
| **Cách kiểm** | `powershell -c "[Environment]::GetFolderPath('MyDocuments')"` — đừng bao giờ suy ra từ `$HOME` |
| **Cách đúng** | (a) Nếu công cụ có cờ chỉ đường dẫn cấu hình thì **dùng cờ đó** — AirSim có `-settings=<path>`, đứng đầu thứ tự tìm (`SimHUD.cpp:388`). (b) Nếu buộc phải ghi vào Documents thì lấy đường dẫn từ shell API |

🔑 **Lợi ích phụ của cách (a):** cấu hình của dự án nằm trong repo và **không đụng vào cấu hình cá nhân
của chủ máy**. Ca thật đã suýt hỏng: bản "backup" tôi tạo hoá ra là backup của một file **không liên
quan**, nên nếu tin vào nó thì đã khôi phục nhầm.

⚠️ **Cùng họ với bẫy này:** đừng cho rằng đường dẫn người dùng nhìn thấy và đường dẫn ứng dụng đọc là
một. Kiểm bằng cách **hỏi chính chương trình đang chạy** — ở đây `getSettingsString()` qua RPC 41451
trả lời dứt điểm trong 2 giây, sau khi đã đoán sai hai lần.


## 11. 🔴 Đừng "ping" cổng HIL của AirSim — phép thử ăn mất suất của PX4 (2026-08-19)

| | |
|---|---|
| **Triệu chứng** | Kiểm `4560` thấy **mở**, vài giây sau chạy cổng thì báo *"nothing is listening"*. `Get-NetTCPConnection` cho thấy một kết nối ở trạng thái **`CloseWait`** từ IP của WSL |
| **Nguyên nhân** | Phía HIL của AirSim là **`acceptTcp()`** (`MavLinkMultirotorApi.hpp:1327`) — nó **nhận ĐÚNG MỘT kết nối rồi thôi lắng nghe**. Phép thử sống-chết của ta chính là kết nối đó; probe đóng lại là AirSim còn một peer đã chết và **không mở lại cổng** |
| **Cách đúng** | Kiểm sức khoẻ AirSim bằng **cổng RPC `41451`** (server bình thường, nối bao nhiêu lần cũng được), **tuyệt đối không chạm `4560`**. Muốn chắc hơn nữa thì gọi `ping` qua msgpack-rpc |
| **Khắc phục khi lỡ** | Phải **khởi động lại Unreal** — không có cách nào bắt AirSim lắng nghe lại `4560` |

🔑 **Nguyên tắc rút ra:** *một phép thử sống-chết chỉ hợp lệ khi nó KHÔNG tiêu thụ tài nguyên nó đang
đo.* Cùng họ với bài học lõi quỹ đạo P6.2 — **cách đo làm hỏng thứ được đo** thì con số thu được là
con số của phép đo, không phải của hệ thống.


## 12. Bẫy gom ngày 2026-08-20 (chuỗi cổng bay P6)

| Bẫy | Triệu chứng | Cách đúng |
|---|---|---|
| **`pgrep -f 'a\|b'` khớp 0** | Vòng chờ "13 tiến trình" không bao giờ thoả dù đủ tiến trình | `pgrep` dùng **ERE**: pipe để **trần** `'a\|b'` → sai, `'a|b'` → đúng (đo: escaped=0, plain=2). Khác `grep` mặc định (BRE) nơi `\|` mới là phép hoặc |
| **Log rclcpp ra STDERR** | Script đếm `"I heard"` ra 0 vĩnh viễn dù node chạy | `RCLCPP_INFO` mặc định ra **stderr**. Phép đếm phải `2>&1`; `2>/dev/null` là tự bịt mắt |
| **`ros2 action list` KHÔNG có `--no-daemon`** | Kèm `2>/dev/null` ⇒ lỗi argparse thành im lặng ⇒ đọc thành "action chưa quảng bá", retry vô ích (mất 2 lượt cổng D4) | Chỉ `topic/node/service list` có cờ đó. `ros2 action list` chạy trần; muốn thấy lỗi thì `2>&1` |
| **`ros2 node list` một-phát ngay sau khi tiến trình lên** | Đếm process đủ nhưng node "chưa tồn tại" | Discovery đồ thị trễ vài giây sau bảng tiến trình — check node-name phải là **vòng retry có deadline**, không one-shot |
| **Model sinh cho bài ĐẬU chưa chắc BAY được** | `uav0_track` armed → không nhấc → PX4 "auto preflight disarming" sau 10 s; M5 smoke CÙNG model lại PASS (bước setpoint 2,5 m phá được land-detector đang latch, ramp bị xích kẹp 0,58 m thì không) | Gốc: airframe `4103` thiếu profile vision-primary của `4102` ⇒ EKF GPS-primary bị external-odometry bơm vision song song ⇒ "vertical velocity unstable" ⇒ land detector giữ chặt. **Quy tắc: mọi biến thể model mới phải qua MỘT chuyến M5 trước khi dùng cho cổng bay** |
| **Đen-boot EKF: arm bị từ chối "height estimate not stable"** | M5 trượt lượt 1 "arm rejected", lượt 2 PASS 3/3 nguyên trạng | GPS sim nhiễu trắng thỉnh thoảng giữ drift-checker quá timeout arm của smoke. Trước khi nghi code: đọc log PX4. Lưu ý PX4 **tự autoconfig khi đổi airframe** (`SYS_AUTOSTART cũ→mới`, param reset) — KHÔNG phải rò param giữa model |
| **Runner cổng thoát sớm không dọn** | 2 lần để sim mồ côi sau FATAL tiền-kiểm | FATAL path phải có teardown (nợ: thêm `trap` vào 3 runner); tạm thời chạy `stop_sim.sh` trước mỗi lượt |
| **Guard anh em CƯỚP LỜI guard đang thử khi ctest song song đói CPU** | Test khẳng định chuỗi lý do của guard danh tính đỏ chập chờn: guard ĐỘ TƯƠI (đồng hồ tường) nổ trước vì fixture bị bỏ đói (route 3,1 s tuổi > trần 1,0). Chạy target MỘT MÌNH thì PASS 2/2 | **GHIM THAM SỐ guard-không-thử ra khỏi tầm** (`route_fresh_sec=60` trong fixture) — KHÔNG nới assertion, KHÔNG chấp nhận "một trong hai lý do" (mất claim). Kèm: bộ phát của fixture phải nằm `sim_group_` như "cảm biến thật cứ phát bất kể máy tính bay bận" |
| ✅ **Xác nhận dùng được:** `gz service` EntityFactory nhận **`sdf:` inline string** | — | Spawn model giữa chuyến không cần file: reply `data: true`, kiểm bằng `gz model --list`. Đã dùng thật ở cổng G-N4b |

## 13. `use_sim_time` & timer — bật tham số là CHƯA ĐỦ, còn nguy hơn không bật (2026-08-10, chuyển từ memory §7 nợ #1)

| Bẫy | Triệu chứng | Cách đúng |
|---|---|---|
| **`create_wall_timer` vẫn theo đồng hồ THỰC sau khi bật `use_sim_time`** | `now()` đã là thời gian sim nhưng nhịp phát vẫn nhịp thực — sim chạy nhanh hơn thực (FTRT) thì offboard 20 Hz thực có thể tụt dưới **2 Hz sim** → PX4 kích failsafe | Mọi timer nhịp-phụ-thuộc-sim phải là `rclcpp::create_timer(this, get_clock(), ...)`; vòng chờ service dùng `get_clock()->sleep_for()` |
| **Ở RTF 1.0 không phép đo tần số nào phân biệt được hai loại timer** | Mọi thứ trông đúng cho tới khi RTF ≠ 1 | **Phép thử tạm dừng** (tái dùng được, cả cho AirSim Đ4): tạm dừng sim → timer đồng hồ sim **DỪNG HẲN**, timer đồng hồ thực vẫn chạy. Đo thật: `/state/vehicle` 9,913 Hz → pause → 9,903 Hz (không dừng = sai loại timer) |

## 14. Cài Micro XRCE-DDS Agent v2.4.2 (hồ sơ 3 lần fail — 2026-08-03, chuyển từ memory)

| Đường | Kết quả |
|---|---|
| snap (mọi kênh) | 🔴 NGÕ CỤT — v1.0.2 (2021), quá cũ cho PX4 v1.15. Triệu chứng: log `session established` nhưng `create_topic`/`create_publisher` = 0, session đóng-mở lặp 10 s, ROS2 thấy 0 topic `/fmu/*`. Nên `snap remove` để khỏi lẫn binary tốt |
| apt `ros-humble-micro-xrce-dds-agent` | 🔴 gói không tồn tại |
| Build SUPERBUILD=OFF dùng Fast-DDS của Humble | 🔴 fail — Humble có fastcdr 1.0.29, agent v2.4.2 cần 1.1.1 |
| **✅ Lời giải (đã chạy)** | Gốc 3 lần fail: `CMakeLists.txt:99` ghim `set(_fastdds_tag 2.12.x)` = tên **NHÁNH** mà eProsima đã xoá sạch nhánh 2.x; **TAG `v2.12.2` vẫn còn**. Làm: clone agent `v2.4.2` → `sed -i 's/set(_fastdds_tag 2\.12\.x)/set(_fastdds_tag v2.12.2)/' CMakeLists.txt` → cmake `-DCMAKE_INSTALL_PREFIX=$HOME/.local` (superbuild ON) → `make -j && make install`. Không cần sudo. Binary `~/.local/bin/MicroXRCEAgent`, cần `LD_LIBRARY_PATH=$HOME/.local/lib` |
| Bẫy phụ | `ros2 topic list` trả **rỗng hoàn toàn** (không cả `/rosout`) = daemon ROS2 treo, KHÔNG phải DDS hỏng → kiểm `ros2 topic list --no-daemon` (cờ này chỉ có ở `list`) |

## 15. Bẫy gom ngày 2026-08-22 (thi công P9 mission)

| Bẫy | Triệu chứng | Cách đúng |
|---|---|---|
| **Mutation check bằng `mv orig→file` không kích rebuild** | Sửa file nguồn để mutation nhưng test vẫn xanh — vì `mv` giữ **mtime cũ**, colcon coi như chưa đổi | `touch` file sau khi hoán, hoặc `colcon build --cmake-force-configure`; kiểm mutation THẬT đã vào binary trước khi tin kết quả |
| **apt `ros-humble-behaviortree-cpp-v3` có 2 CMake Config cạnh tranh** cùng trỏ 1 `.so` | `find_package(behaviortree_cpp_v3 REQUIRED)` "thành công" nhưng `target ... was not found` ở link | Config raw-CMake (`lib/cmake/`) được chọn trước bản ament (`share/.../cmake/`) ⇒ target đúng là **`BT::behaviortree_cpp_v3`**, không phải tên ament trực giác. Chẩn đoán `cmake --debug-find` (CMake 3.22 **chưa có** `--debug-find-pkg=`). Kèm: bản BT của ROS phải link thêm **`ament_index_cpp`** |
| **`gz-sim-velocity-control-system` + vật tiếp đất = lật** | Hộp cao (0,8 m) đặt khít mặt đất, lệnh vận tốc ngang → pitch leo dần rồi lật trong vài giây (contact solver sinh mô-men mỗi bước vì trọng tâm cao hơn điểm chạm) | `<gravity>false</gravity>` + spawn hở mặt đất ~0,05 m. Đối chứng đã đo: cùng model thả cách đất 5 m → RPY = 0 suốt |
| **Đo tốc độ model bằng `sleep N; gz model -p` trong bash** | Tốc độ đo lệch ~30–35% so với thật (overhead subprocess ăn vào Δt) | Đọc `/world/<w>/dynamic_pose/info` (Pose_V, **có sim-time header**) qua `gz.transport13`/`gz.msgs10` (có sẵn) — đo lại đúng 0,2000 m/s |
| **Spawn vật test tại (0,0)** | Va chạm giả với drone (spawn mặc định cùng chỗ), trông như lỗi vật lý | Luôn spawn xa gốc toạ độ khi test model riêng lẻ |
| **Số tiến trình bringup là 14 từ P7.4, không phải 13** | Vòng chờ "13 tiến trình" trong script cổng cũ sai âm thầm (`run_gn5.sh` "may mắn" đúng vì tự kill+restart navigator) | 5 backend + 6 localization + 3 navigation (navigator mặc định `true` từ P7.4). Script cổng mới đếm 14; đếm-cứng là điểm vỡ khi thêm cờ mặc định — cân nhắc đếm theo danh sách tên |

*(Hai bài học C++ cùng ngày — thứ tự huỷ thành viên khi destructor gọi ngược `this` (SIGSEGV) và livelock guard-tái-trọng-tài-khi-Finish — là bài học THIẾT KẾ, không phải công cụ: hồ sơ ở `docs/package-status.md` §11 + `src/uav_mission/README.md`.)*

## 16. Bẫy gom chiến dịch cổng bay P9 (2026-08-22 → 23)

| Bẫy | Triệu chứng | Cách đúng |
|---|---|---|
| **`pgrep -f <chuỗi>` tự khớp MỌI nơi chuỗi xuất hiện trong dòng lệnh** — kể cả trong `echo`/comment của chính lệnh bọc ngoài, và cả dòng lệnh của Monitor đang chạy vòng kiểm | Vòng chờ treo vĩnh viễn (monitor 1 giờ); `pkill` tự SIGTERM shell | Pattern ngoặc vuông `"[v]erify_mission"`, hoặc khớp tên file script; teardown luôn qua file script (biến thể mới của §8c) |
| **Lỗi khởi tạo blackboard BT chỉ hiện trong BRINGUP log** ("terminate called after throwing BT::LogicError") | Probe chỉ thấy action-timeout, trông như treo phía client | Im lặng phía client KHÔNG có nghĩa lỗi ở client — đọc log node thật trước khi nghi probe |
| **Hai kênh DDS độc lập (action result vs topic) không bảo đảm thứ tự tới CLIENT** dù server publish trước khi trả result | Đọc topic ngay sau `future.done()` vẫn thấy giá trị cũ | Settle-wait ngắn / `wait_terminal_status()` sau khi result về |
| **PX4 SITL LƯU BỀN `SIM_BAT_DRAIN`/`SIM_BAT_MIN_PCT`** trên đĩa (`eeprom/`) qua stop/start | Round tiêm-pin trước làm bẩn round sau | Reset về mặc định thật (đọc từ `battery_simulator_params.c`) sau MỖI lần PX4 boot |
| **Hai dòng log giống hệt chữ mang hai NGHĨA khác nhau** ("accepted goto_pose goal" = Search HOẶC Finish-GotoHome) | Cửa sổ đo cắt sai chỗ, số liệu nhiễm bẩn ≥52% | Không suy nghĩa từ text đơn thuần — đối chiếu THỨ TỰ + mốc chỉ-xảy-ra-một-lần (land) |
| **`src/` KHÔNG phải symlink như `scripts/`** | Sửa file test `.py` bên Windows, chạy WSL không thấy đổi | `cp` thủ công (hoặc chạy từ `/mnt/c` trực tiếp) |
| `rclpy.qos.QoSProfile(depth=N)` mặc định **RELIABLE** | — | Dùng để LOẠI TRỪ nghi mất gói khi debug probe |
| **"costmap dropped N obstacle(s)" của local_planner là filter RIÊNG (altitude band)** | Tưởng là chỉ số ground-filter của obstacle_extractor | Đo nguồn bằng `UAV_GROUND_FILTER_DEBUG=1` (ratio/candidate_drop/triggers mỗi khung) |
| **Ngưỡng thời gian của KỊCH BẢN cổng cũng phải suy từ cơ chế** (mission-timeout 90 s < 2 episode thật 99–114 s) | "Cải thiện nhưng vẫn FAIL" trông như bug sản phẩm | Trước khi kết tội sản phẩm: tự hỏi giả định phép đo của cổng còn đúng không (họ R27) |
| **Đọc đúng nghĩa AN TOÀN của hành vi trước khi viết tiêu chí** (Recover=CLIMB là HOLD trên cao, `armed=True` sau đó là ĐÚNG) | Tiêu chí "disarm sau Land" đỏ oan | Tiêu chí phải phát biểu theo HỢP ĐỒNG hành vi, không theo phản xạ "kết thúc = đã hạ cánh" |
| **add/remove_node khi MultiThreadedExecutor còn luồng trong rmw_wait ⇒ SIGSEGV**; respin executor đã cancel ⇒ guard-condition-invalid (lần 2) | Crash trong test hạ tầng | cancel + join TRƯỚC khi đụng node set; executor dùng lại phải DỰNG MỚI |
| **Thứ tự huỷ thành viên C++ khi destructor gọi ngược `this`** (`~BT::Tree()` → onHalted → chạm member đã huỷ) | SIGSEGV lúc teardown | Thành viên có destructor gọi ngược `this` khai báo SAU mọi thành viên nó chạm (huỷ ĐẦU TIÊN) |

## 17. Bẫy gom P10.6 (2026-08-23 — tích hợp observability vào bringup)

| Bẫy | Triệu chứng | Cách đúng |
|---|---|---|
| **`stop_sim.sh` liệt kê pattern pkill TAY theo từng package** — thêm package mới vào `sim.launch.py` mà quên thêm pattern tương ứng thì node đó **mồ côi vĩnh viễn** sau mọi lần teardown | Sau `stop_sim.sh` báo "sim stopped, clean", `ps aux` vẫn thấy `rosbag_manager_node`/`diagnostics_node`/`event_logger_node` sống 15+ phút; bag sqlite3 không có `metadata.yaml` vì `Writer` chưa từng được `close()` (không phải do kill -9, mà do KHÔNG hề nhận tín hiệu nào) | Mỗi package mới có node chạy nền phải thêm pattern riêng vào `stop_sim.sh` **cùng lượt** với việc thêm vào launch file — cùng bài học R34 (audit/teardown phải derive theo danh sách thật, không chép tay lẻ tẻ theo thời gian) |
| **Recorder cần graceful SIGINT để `close()` sạch, không phải SIGTERM chung** | `rclcpp` chỉ có handler mặc định cho SIGINT (gọi `rclcpp::shutdown()`, unwind về destructor); SIGTERM không có handler riêng ⇒ default OS action = chết ngay, không unwind, không `Writer::close()` | Với node có state cần flush (rosbag writer, file JSONL…), teardown phải gửi `SIGINT` trước (đợi vài giây) rồi mới tới vòng `pkill` SIGTERM chung làm lưới an toàn |
| **Tên node trong `ros2 node info` KHÔNG có namespace dù topic nó publish có** | Launch không truyền `namespace=` cho `Node(...)` — chỉ node tự prefix `/uav/<id>/...` vào TÊN TOPIC bên trong code, tên NODE vẫn trần (`/rosbag_manager_node`, không phải `/uav/uav0/rosbag_manager_node`) | Luôn `ros2 node list` để lấy tên thật trước khi `ros2 node info`, đừng suy đoán tên theo quy ước namespace của topic |

---

## 18. Bẫy gom P10.8c (2026-08-23 — vá `offered_qos_profiles` + chẩn đoán G-O3(c))

| Bẫy | Triệu chứng | Cách đúng |
|---|---|---|
| 🔴 **Comment XML chứa `--` là XML KHÔNG HỢP LỆ** — `package.xml` build fail | `catkin_pkg`/`expat` báo `not well-formed (invalid token)` đúng dòng chứa `<!-- ... -- ... -->`; thông báo lỗi không nói thẳng "double-hyphen trong comment" | Không bao giờ viết `--` (kể cả dùng như dấu gạch nối ý — "X -- Y") bên trong comment XML (`package.xml`, launch `.xml` nếu có); dùng dấu phẩy/ngoặc đơn thay thế |
| 🔑 **Giả thuyết "timer sim-time đóng băng sau cú lùi đồng hồ lớn" — ĐÃ ĐO VÀ LOẠI, đừng thử lại** | Đọc header `rcl/timer.h` (`rcl_timer_get_time_until_next_call = last_call_time+period−now`) gợi ý timer sẽ "kẹt" tới khi sim time leo lại ngang mốc cũ — trực giác hợp lý nhưng SAI khi đo thật | Thực nghiệm tối giản (`o3_timer_freeze_probe.py`, rclpy timer thuần + publisher `/clock` tự lái, không đụng code sản phẩm): cú lùi 248 s → timer **hồi phục sau ~0,15 s sim / ~0 s wall**, không hề đóng băng. Bài học: đọc header suy ra được API SURFACE, không suy ra được toàn bộ hành vi runtime (rcl có xử lý jump nội bộ không lộ trong `.h`) — **một giả thuyết dù có vẻ hợp logic từ tài liệu vẫn phải verify bằng thí nghiệm nhỏ trước khi tin (R14)**, nhất là trước khi nó dẫn tới kết luận "giới hạn hệ thống" ảnh hưởng mọi node sim-time |
| 🔑 **`ros2 bag play --loop` không tạo được "kênh GIỮ quyền qua một mốc cũ" — bằng chứng chính thức là `o3_clock_regression_synth.py`, KHÔNG phải `--loop`** | Cổng kỳ vọng `clock_regressions>0` sau `--loop` nhưng đo ra 0 dù cơ chế N-c (`ageSecOrInf`) đã kiểm chứng đúng bằng test tất định khác trên CHÍNH node đó | Sửa **tiêu chí cổng** (chuyển sang test tất định `o3_clock_regression_synth.py`, cố ý GIỮ kênh sống rồi mới im lặng qua cú lùi), không sửa arbiter. 🔴 **Kết luận nguyên nhân gốc bên dưới (cũ) ĐÃ BỊ THAY — đọc dòng kế tiếp trước khi trích dẫn bất cứ gì từ đây** |
| 🔴 **ĐÃ SỬA 2026-08-23 (review đóng P10, R35): kết luận cũ "MISSION tự nhả quyền bình thường trước khi bag kết thúc" là SAI** | Đọc lại `~/gate_logs/o3_play_loop.log` + `o3_ca_loop.log` (log gốc của chính phép đo, chưa từng đọc kỹ khi viết kết luận cũ) — không phải đo lại, chỉ đọc lại bằng chứng đã có | `o3_play_loop.log`: **ngay giây đầu tiên** của phiên `--loop` (`t≈113.27`, không phải lúc lùi đồng hồ), `rosbag2_player` báo `RELIABILITY_QOS_POLICY` incompatible trên cả 4 topic `cmd_mission`/`cmd_operator`/`cmd_safety`/`cmd_test` + `diagnostics/control_authority` — đúng bug C3 (recorder ghi `offered_qos_profiles=be` thay vì Reliable thật của publisher) ⇒ arbiter **0 cmd_mission SUỐT phiên**, không riêng lúc đồng hồ lùi. `o3_ca_loop.log`: arbiter tự log `2 publishers on .../command_selected, only this node may write it` lặp ~60 lần suốt phiên (mỗi ~2-3 s, `t=114`→`t=264`) — `command_selected`/`authority` cũng nằm trong bag, `ros2 bag play` tự dựng publisher THỨ HAI đè lên đúng topic exclusive-writer ⇒ đầu dò theo dõi `active_source` qua `/control/authority` trong phiên đó rất có thể đang thấy **bag phát lại chính nó**, không phải quyết định thời gian thực của arbiter sống. Kết luận đúng: nguyên nhân gốc là **arbiter chưa từng có cmd_mission LIVE để mất** (C3, toàn phiên) + tín hiệu quan sát nhiễm bởi publisher thứ hai từ bag — KHÔNG phải "mission nhả quyền bình thường". Sau khi vá C3, `cmd_mission`/`command_selected`/`authority` sẽ replay đúng QoS — **G-O3(c) cần chạy lại** (bay + capture bag + `--loop`) để xác nhận kết quả mới; ngoài phạm vi phiên sửa lỗi P10-review — coi kết luận cũ là **CHƯA CHỐT**, xem `docs/interface-contract-v0.1.md` S:2.20 |
| 🔑 **Trước khi kết luận "0 mẫu = replay lỗi", kiểm xem topic đó có publisher NÀO trong world/config này không** | Kiểm 8 topic `reliable_tl` sau khi vá QoS: 6/8 khớp 100% reference=live, nhưng `gnss_origin` và `/tf_static` đều 0/0 — trông giống lỗi vá dở | Grep code trước khi nghi replay: `/tf_static` **không có publisher nào trong toàn bộ codebase** (chỉ có `TransformBroadcaster` động → `/tf`) — 0 mẫu trên MỌI bag, MỌI world, mãi mãi cho tới khi ai đó thêm `StaticTransformBroadcaster`. `gnss_origin` chỉ phát khi PX4 EKF có tham chiếu GPS toàn cục — world indoor (GPS-denied theo thiết kế) không kích hoạt; bay M5 nhanh (~1 phút) trên world ngoài trời xác nhận **có đúng 1 mẫu**, chứng minh publisher đúng, chỉ là world đang test không tạo dữ liệu. Không tốn công vá code khi vấn đề thật là "chưa từng có ai publish", không phải "publish rồi mà replay mất" |
| 🔴 **P10.9a rerun (2026-08-24, chạy lại G-O3(c) sau khi vá double-writer bằng `--topics`): loại được giả thuyết C3/double-writer, nhưng lộ MỘT bất thường MỚI — `clock_regressions` tăng liên tục ngay cả KHI KHÔNG `--loop`** | `o3_run_authority_watch` đổi sang `ros2 bag play --topics cmd_mission cmd_safety cmd_operator cmd_test` (bỏ `command_selected`/`authority` khỏi replay — arbiter không còn thấy "2 publishers" nữa). Đối chứng dương MỚI (`ever_saw_mission`) xác nhận arbiter **có** nhận cmd_mission thật ở cả 2 lượt (`loop`: `first_mission_at_t` xác nhận; `loop_control`: cũng xác nhận) — giả thuyết C3 (arbiter mù hoàn toàn) đã bị LOẠI dứt điểm. Nhưng lượt **KHÔNG `--loop`** (đối chứng dương phải cho `clock_regressions==0`) đo ra **1505** (không phải 0), tăng đều ~20/s suốt phiên — cùng tốc độ với nhịp phát cmd_mission (~20 Hz), gần như MỖI message đều bị tính là một regression. Lượt `--loop` cũng tăng đều same-rate (4957 sau ~166s`), không có bước nhảy rõ rệt riêng tại điểm `--loop` wrap — nghĩa là hiện tượng này KHÔNG đặc thù cho việc lặp lại, mà xảy ra với BẤT KỲ phiên `ros2 bag play --clock` nào một khi cmd_mission bắt đầu chảy | **CHƯA điều tra xong root cause** (ngoài phạm vi phiên verify — cần kỹ sư `uav_control_authority` hoặc người hiểu rõ `rosbag2_cpp::Writer`/`ros2 bag play --clock`): nghi phạm hàng đầu là **domain thời gian lệch** giữa `/clock` mà `ros2 bag play --clock` tổng hợp lại (dựa trên timestamp `rosbag_manager_node.cpp:565` ghi bằng `now()` — sim-time lúc GHI) và `stamp_sec` nhúng trong từng `ControlCommand` (đọc ở `control_authority_manager_node.cpp:47` qua `rclcpp::Time(msg.header.stamp).seconds()` — sim-time lúc chính node gốc PHÁT, luôn sớm hơn lúc ghi một khoảng độ trễ transport). Về lý thuyết ghi luôn SAU phát ⇒ `now_sec` (dựa trên lúc ghi) phải LUÔN ≥ `stamp_sec` (lúc phát) ⇒ không giải thích được vì sao gần như MỌI message đều bị tính regression — giả thuyết này **chưa được đo xác nhận**, chỉ là nghi phạm hàng đầu dựa trên đọc code. Đường điều tra tiếp: dựng lại `o3_timer_freeze_probe.py`-style thí nghiệm tối giản đo TRỰC TIẾP `/clock` thật vs `stamp_sec` của message đầu tiên nhận được trong một phiên `ros2 bag play --clock` (không qua sim, không qua arbiter) để cô lập xem lệch nằm ở đâu. **Không suy rộng số 1505/4957 thành bất cứ điều gì về arbiter thật đang bay** — cả 2 lượt đều đi qua `ros2 bag play`, chưa có phép đo nào KHÔNG qua đường replay để so sánh |
| ✅ **ĐÃ CHỐT 2026-08-24 — nguyên nhân là ĐỘ PHÂN GIẢI `/clock` của chính công cụ replay (~40 Hz), KHÔNG phải lệch domain, KHÔNG ảnh hưởng bay thật** | Thí nghiệm cô lập: đo song song tick `/clock` + cặp `(now, header.stamp)` của 30 message `cmd_mission` đầu tiên trong một phiên `ros2 bag play --clock`; cộng một lượt **ground-truth đọc thẳng bag bằng `rosbag2_py`** (không qua `bag play`, không qua ROS graph) | **`/clock` do `ros2 bag play --clock` phát là tick RỜI RẠC 40,03 Hz (chu kỳ ~25 ms)**, độc lập pha với lịch phát message. `cmd_mission` chảy 20 Hz (50 ms) ⇒ khi subscriber `use_sim_time` đọc `now()`, mẫu `/clock` mới nhất **thường chưa tick qua** `header.stamp` của message đang xử lý ⇒ `now < stamp` GIẢ. Đo: **29/30 mẫu âm (96,7%)**, biên độ **−0,000 → −0,023 s** — khớp đúng chu kỳ tick 25 ms. 🔴 **Giả thuyết "lệch domain" ở dòng trên ĐÃ BỊ LOẠI**: ground-truth cho `recv_time` (recorder ghi bằng `now()`) và `header.stamp` (navigator gán bằng `now()`) **bằng nhau tới micro-giây — `diff = 0,000000` trên 15/15 mẫu**; `/clock` replay cũng chạy đúng khoảng bag (14,65→219,8 s). **Bay thật KHÔNG dính** (3 bằng chứng độc lập): M5/G-M1 PASS · ground-truth `recv−stamp=0` chứng minh khi bay cả hai đầu lấy từ **một luồng `/clock` sống liên tục** · cơ chế lỗi thuộc về **công cụ replay**, không tồn tại khi hệ chạy live. 🔴 **KHÔNG sửa `ageSecOrInf()` để tha cho nhiễu này** — dung sai 0 là ĐÚNG cho vận hành thật; nới ra là làm yếu chính cơ chế N-c bảo vệ bay thật. 🔑 **Bài học dùng lại được: mọi tiêu chí so sánh thời gian với dung sai 0 đều KHÔNG đo được qua `ros2 bag play --clock`** — độ phân giải công cụ (25 ms) lớn hơn tín hiệu cần đo; muốn đo phải dùng đường tất định không qua bag play (`o3_clock_regression_synth.py`) |

---

## 19. Hạ tầng cổng có thể báo PASS mà không đo được gì (P10-gate-debt review round 2, N4–N8, 2026-08-24)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| 🔴 **Cây build WSL lệch cây nguồn Windows ⇒ mọi lượt chạy cổng đo BẢN CŨ** (bắt được ở checklist đóng phiên R22 §3 bước `diff -rq`, 2026-08-24) | Nguồn chuẩn là Windows `C:\code\PX4_ROS2\src`, cây build là WSL `~/PX4_ROS2/src` — **hai cây riêng biệt** (chỉ `scripts/` là symlink nên không lệch được). Agent sửa file trên Windows rồi **quên sync** ⇒ WSL còn bản cũ. Ca thật: `gn4b_avoidance_gate.py`/`gn5_followtrack_gate.py` (Windows 04:06 có bản vá N6, WSL 02:47 chưa có — lệch 51 dòng) + `uav_observability/README.md` (Windows 163 case, WSL 161). 🔑 **Đây chính là lý do N6 "chưa ai kiểm"**: cổng G-N4b/G-N5 nếu chạy trên WSL sẽ chạy đúng bản NO-OP mà N6 vừa vá | (1) `diff -rq --exclude=__pycache__ /mnt/c/code/PX4_ROS2/src ~/PX4_ROS2/src` là **bước bắt buộc** mỗi lần đóng phiên (R22 §3) — và nên chạy **trước mỗi chiến dịch cổng**, không chỉ lúc đóng phiên. (2) So **mtime + kích thước** để biết bên nào mới, đừng đoán; Windows là canonical ⇒ push sang WSL, **không bao giờ ngược lại** trừ khi đã kiểm nội dung. (3) Cùng họ bài học `gm1_taskE_confirm`: *bằng chứng phải phủ đúng artefact hiện hành* — ở đây "artefact" là **cả cây nguồn**, không riêng binary |
| `assert_single_writer` (bash, `gate_common.sh`) thoát PASS ngay lần đọc `ros2 topic info` **ĐẦU TIÊN** dù đúng lúc đó một double-writer thật chưa kịp lộ | Mỗi lần gọi `ros2 topic info --no-daemon` là **một process mới**, tự chạy đua discovery riêng — "0 publisher" ở lần đọc đầu không phân biệt được với "một writer thật CHƯA được khám phá bởi query này". Đo bằng shim giả lập `ros2` (không cần ROS thật, chỉ script trả `Publisher count` theo kịch bản): chuỗi `[0,1,1,1]` với `expected=0` → PASS giả ngay ở mẫu đầu | (1) **Đối chứng dương**: trước khi tin bất kỳ số đọc nào trên `TOPIC`, xác nhận `<uav-prefix>/state/vehicle` (luôn có publisher khi stack thật lên) đã thấy ≥1 publisher — không thấy ⇒ `return 2`, KHÔNG PASS. (2) **Giữ nguyên qua thời gian**: đọc `TOPIC` **3 mẫu cách nhau ≥2 s**, cả 3 phải khớp `expected` — lệch bất kỳ mẫu nào ⇒ `return 1` ngay. `scripts/selftest_gate_common.sh` case 4/5 dựng lại 2 kịch bản này bằng `ros2 topic pub` thật (chưa chạy — chờ ROS graph rảnh, ghi rõ trong file) |
| Bản rclpy song sinh (`gn4b_avoidance_gate.py`/`gn5_followtrack_gate.py`) là **NO-OP đã đo**: gọi ngay sau `create_publisher()` với `expected=1`, thoát sau **0 vòng lặp** | `count_publishers()` đếm luôn publisher CỦA CHÍNH node gọi nó ⇒ `count==expected` đúng ngay từ đầu — writer từ xa (perception:=true để sót) chỉ lộ ra sau ~0,5 s, quá muộn để lọt vào phép đo | Cùng công thức: `discovery_probe_topic` (`state/vehicle`) phải thấy ≥1 publisher trước, rồi giữ `count_publishers(TOPIC)` ổn định suốt `hold_sec` (2,5 s, lấy mẫu mỗi 0,5 s qua `rclpy.spin_once`). Đo offline bằng cách trích xuất ĐÚNG hàm từ file thật (`ast` lấy khoảng dòng) rồi `exec()` với `rclpy`/`node` giả lập — kịch bản `[1,1,2,2,...]` (writer trễ) chứng minh code cũ im lặng, code mới bắt được |
| `o4_report.py cmd_go`/`cmd_ab` PASS trên vòng thoái hoá — `/clock` đứng (Gazebo pause/chết) nhưng node vẫn sống ⇒ mẫu `GO` cũ latch (TransientLocal) nằm mãi | Chỉ xét `records[-1]`: TransientLocal đảm bảo **luôn** có ít nhất 1 mẫu, kể cả khi `/clock` đứng làm CẢ eval-timer LẪN publish-timer của `diagnostics_node` đứng theo (2 timer cùng bám 1 đồng hồ) | Hai lớp: (1) **đếm mẫu** — `len(records) >= min_expected_samples(duration_sec, publish_period_sec)` (mặc định biên margin 0,5×) bắt được trường hợp tổng số mẫu tụt hẳn, không chỉ mẫu cuối cũ; (2) **tuổi mẫu cuối** — `is_fresh(now-wall_stamp_sec, 2×publish_period_sec)`. `cmd_ab`'s nhánh `[a]` neo tuổi tại **t_kill**, KHÔNG neo tại "now" (report chạy sau `O4_KILL_WAIT_SEC` cố ý chờ, neo vào "now" sẽ stale-out mọi baseline oan). Cả hai hàm dùng chung `scripts/gate_freshness.py` với `preflight_light.py` |
| `preflight_light.py` in `GO age=-12.345s` **exit 0** đúng lúc checklist bảo đọc đèn sau ARM trước takeoff | `age_sec <= max_age_sec` chỉ chặn MỘT phía — companion lệch NTP (đồng hồ chạy sau) cho `age_sec` ÂM, và `-12.0 <= 2.0` vẫn `True` | `is_fresh(age_sec, max_age_sec)` bắt buộc `0.0 <= age_sec <= max_age_sec` — âm hay NaN đều fail closed như nhau (so sánh NaN luôn `False` trong Python, cùng hướng fail-closed). `staleness_board.cpp` (C++) đã làm đúng luật này từ trước — N8 chỉ đồng bộ lại phía Python |
| 🪤 **Bẫy tự bắt khi vá N4**: thêm backtick vào một khối comment TIẾNG ANH bên trong `python3 -c "..."` (bash double-quoted) làm bash cố **thực thi backtick đó như command substitution** — dù nó nằm trong một dòng comment `#` của PYTHON, bash không biết gì về cú pháp Python, nó chỉ thấy một chuỗi double-quote có backtick | Sửa `round_o3_loop_report`'s block comment, gõ theo phản xạ Markdown (backtick = code span) quên mất ngữ cảnh là bash string | **KHÔNG BAO GIỜ dùng backtick trong bất kỳ đoạn text nào (kể cả comment) nằm bên trong `python3 -c "..."` của bash** — dùng dấu nháy đơn `'...'` hoặc bỏ hẳn ký hiệu code-span. Bắt được bằng cách chạy thử offline (source riêng function, feed JSON giả) TRƯỚC khi tin `bash -n` sạch là đủ — `bash -n` không phát hiện được lỗi này vì cú pháp vẫn hợp lệ |

## 20. Hai lượt đo giẫm lên nhau: "kiểm" không có nghĩa là "rẻ" (2026-08-24)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| `verify_workspace.sh` in **`WARNING: cross-check mismatch — colcon says 1105, per-package sums to 1116`** kèm `Skipping 'build/uav_navigation/Testing/TAG': could not find latest XML file` | 🔴 **`check_test_domain_isolation.sh` KHÔNG phải kiểm tĩnh** — giai đoạn 3/3 của nó chạy nguyên `colcon test` 9 package (~11 phút). Tôi gọi nó **2 lần** trong lúc `verify_workspace.sh` đang test ⇒ **3 lượt ctest cùng ghi vào `build/*/Testing/`**, cái sau xoá `Testing/TAG` của cái trước giữa chừng | **Không bao giờ chạy hai lượt test cùng lúc**, kể cả khi một trong hai mang tên `check_*`. Trước mỗi phép đo dùng `build/`: `ps -eo comm \| grep -qE '^ctest$'` phải **rỗng**. Script đã được vá để **tự từ chối** khi thấy ctest khác đang chạy, và có `SKIP_SUITE=1` cho hai giai đoạn tĩnh |
| Suy rộng | Tên `check_*` / `verify_*` **không nói gì về chi phí**. Cùng khuôn với R15 #2 (*build song song tráo `install/` giữa thí nghiệm*) nhưng ở tầng **kết quả test**, không phải tầng binary | Script nào tiêu một lượt test đầy đủ phải **nói ra trong header** và **tự gác**. Đọc header trước khi gọi một script lạ trong lúc đang đo |

🔑 **Điều đáng giá nhất: KHÔNG có gì hỏng ở sản phẩm — hỏng ở PHÉP ĐO.** Cổng chéo-kiểm `Summary == case + target` (thêm 2026-08-24 vì R34) là thứ **duy nhất** bắt được; không có nó thì con số 1105 đã được ghi vào tài liệu như sự thật. Cùng họ R27-1: *kiểm phép đo trước khi kiểm đối tượng*.

## 21. Hai bẫy cùng một lượt đo: tham số ROS bị suy kiểu từ chữ, và bộ lọc nuốt bằng chứng (2026-08-24)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| Node chết ngay lúc khởi động, **không in dòng nào**; script gọi nó báo *"never delivered a frame"* ⇒ đọc như **cảm biến/stack hỏng** | `--ros-args -p seconds:=60` — ROS2 **suy kiểu từ dạng chữ của giá trị**: `60` là *integer*, mà node `declare_parameter<double>("seconds", 30.0)` ⇒ `InvalidParameterTypeException`, abort. Mặc định cũ của script là `30.0` (có dấu chấm) nên **mọi lượt trước vẫn chạy** — bẫy chỉ lộ khi người dùng truyền số tròn | Với tham số `double`, **luôn truyền có dấu thập phân**. Script nhận số từ env phải **tự chuẩn hoá**: `case "$W" in *.*) ;; *) W="${W}.0" ;; esac`. Đã áp cho `measure_image_budget.sh` và `diagnose_probe_under_stack.sh` |
| Không thể biết vì sao thất bại — log chỉ có kết luận, không có bằng chứng | Script chạy `out=$(probe 2>&1)` rồi in bằng `grep -E 'MiB\|TOTAL\|only N messages'`. Thông báo ngoại lệ **không khớp mẫu nào** ⇒ bị vứt. **Bộ lọc trình bày đã trở thành bản ghi duy nhất** | 🔑 **Bộ lọc dùng để TRÌNH BÀY không được là bản ghi DUY NHẤT.** Luôn `printf '%s
' "$out" > /tmp/<nhãn>.log` **trước** khi grep, và ở nhánh thất bại phải in vài dòng đầu của output thô. Họ R27-1 ở tầng công cụ |

🔑 **Đối chứng là thứ cứu khỏi kết luận sai.** Lượt hỏng chỉ chạy điều kiện *có stack perception* ⇒ nghi ngay stack. Script chẩn đoán chạy **cả hai** điều kiện trong **một lần khởi động sim**: điều kiện *không stack* — vốn "luôn chạy được" — **trượt y hệt** ⇒ stack vô can, lỗi ở dòng lệnh. Không có đối chứng đó thì đã có một báo cáo *"perception làm chết luồng ảnh"* hoàn toàn sai.

## 22. Bẫy `pkill -f` tự sát đã RỘNG RA kể từ R34 — mọi đường dẫn `uav_<pkg>/` đều gây chết (2026-08-24)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| Lệnh chạy nền chết **exit 15**, log dừng ngay tại dòng gọi `stop_sim.sh` kèm `Hangup` | `stop_sim.sh` **derive** pattern từ package đã build (R34) rồi `pkill -f "<pkg>/"`. Dòng lệnh của tôi chứa `…/src/uav_bringup/config/fastdds_large_samples.xml` ⇒ khớp `uav_bringup/` ⇒ **tự giết chính shell gọi nó** | **Không để bất kỳ chuỗi `uav_<pkg>/` nào xuất hiện trên dòng lệnh** gọi thứ có dùng `stop_sim.sh`. Đưa mọi đường dẫn **vào trong file script** (`scripts/run_dds_transport_trial.sh` là mẫu) |

🔑 **Điểm mới, phải cập nhật nhận thức cũ:** cảnh báo cũ (memory §"Cách làm việc đã ổn định") chỉ liệt kê
**tên hạ tầng** — `MicroXRCEAgent`, `gz sim`, `make px4_sitl`. Từ khi R34 bắt `stop_sim.sh` **derive** danh
sách theo package, tập pattern **nở ra theo số package** (nay 12) và gồm cả những chuỗi **rất dễ xuất hiện
trong một đường dẫn bình thường** như `uav_bringup/`, `uav_sim_gz/`, `uav_perception/`. Một cải tiến đúng
đắn ở chỗ khác (R34 chống bỏ sót package) đã **mở rộng bề mặt của một bẫy cũ** — đây chính là lý do quy
tắc *"lệnh WSL phức tạp → viết ra file script rồi chạy"* không phải chuyện tiện tay mà là **lá chắn**.

🪤 **Bẫy kèm, tự bắt trong cùng lượt:** tôi định "dọn cho sạch bằng chứng" bằng
`find /dev/shm -name 'fastrtps_*' -delete`. **Không được làm thế** — các segment đó thuộc về những
participant **đang sống** (kể cả `ros2 daemon` và tiến trình của phiên khác); xoá dưới chân một tiến trình
đang chạy là phá hoại. Cách đúng: **đọc segment LỚN NHẤT** và so với giá trị profile yêu cầu; profile Fast
DDS nạp lỗi thì **im lặng**, nên phải kiểm bằng thứ nó cấp phát thật chứ không bằng biến môi trường.

---

## 23. 🔴 WSL2 TỰ TẮT RỒI KHỞI ĐỘNG LẠI — giết tiến trình `setsid nohup` và xoá sạch `/tmp` (2026-08-24)

**Triệu chứng (hai lần trong một phiên, ban đầu tưởng là hai sự cố khác nhau):**
1. Cây bằng chứng `/tmp/gate_r0/` (log `hz` của **cả hai** nhánh + 2 file `.txt`) **biến mất** giữa lúc
   cổng in verdict và lúc đọc lại. `systemd-tmpfiles-clean` chạy gần nhất **trước đó nhiều giờ**,
   `stop_sim.sh` không đụng gì dưới `/tmp`.
2. Một script chạy nền bằng `setsid nohup … &` **chết ngay lập tức**, để lại log **0 byte** — không một
   dòng nào, kể cả `echo` đầu tiên. Chạy đúng script đó ở foreground thì hoàn toàn bình thường.

**Nguyên nhân duy nhất, xác nhận bằng `who -b`:** `system boot 2026-08-24 20:08` — **VM của WSL2 tắt rồi
bật lại đúng thời điểm đó**. Boot mới ⇒ `/tmp` sạch, mọi tiến trình con (kể cả `setsid`+`nohup`) chết.
`nohup` chống SIGHUP; nó **không** chống việc cả máy ảo biến mất.

| Làm sai | Làm đúng |
|---|---|
| `setsid nohup bash script.sh &` rồi thả cho không còn phiên `wsl` nào bám | Chạy script **bám phiên** — để một tiến trình `wsl` sống suốt thời gian chạy (tool `run_in_background`, hoặc một vòng `until … done` chạy trong `wsl`) |
| Để bằng chứng cổng ở `/tmp` | Bằng chứng cổng ghi vào **`~/PX4_ROS2/gate_logs/<tên cổng>/`** |

🔑 **Vì sao các lượt chạy dài TRƯỚC ĐÓ sống sót:** tình cờ — lúc nào cũng có một lệnh `wsl` đang trực
(vòng `until grep … done`) giữ VM sống. Khi khoảng nghỉ giữa hai lệnh `wsl` đủ dài, VM tắt. Đây là loại
lỗi **không tái hiện theo ý muốn**, nên đừng chẩn đoán bằng cách thử lại — đọc `who -b`/`uptime` là xong.

🪤 **Bẫy kèm:** `pkill -f <mẫu>` mà **chính dòng lệnh gọi nó chứa mẫu trần** ⇒ tự bắn mình (exit 15).
Khác §22 (ở đó `stop_sim.sh` giết caller); ở đây caller giết **chính nó**. Luôn dùng bracket-pattern:
`pgrep -af '[g]ate_r0'`.

🪤 **Bẫy kèm nữa — gọi `wsl` từ Git Bash trên Windows:** đường dẫn `/mnt/...` bị dịch thành
`C:/Program Files/Git/mnt/...`. Dùng PowerShell, hoặc bọc trong `bash -lc '…'`. Và lệnh **nhiều dòng**
truyền qua `wsl … bash -lc` bị **gộp mất xuống dòng** ⇒ với việc gì quá một dòng, **viết ra file script**.

---

## 24b. `rsync -a` đẩy mtime LÙI — build "thành công" mà không hề dịch lại (2026-08-25)

| Triệu chứng | Nguyên nhân | Cách làm đúng |
|---|---|---|
| Sửa nguồn ở cây WSL, đồng bộ từ cây Windows sang, `colcon build` báo **OK**, suite vẫn **5 FAILED** đúng như trước khi sửa. Suýt báo cáo *"bản vá không có tác dụng"* | `rsync -a` **giữ nguyên mtime của nguồn**. Cây canonical Windows có mtime **cũ hơn** file vừa sửa tay trong WSL ⇒ sau khi đồng bộ, nguồn **cũ hơn** object đã dịch (đo thật: nguồn `12:27:06`, object `12:30:14`) ⇒ ninja kết luận không có gì phải làm và **im lặng bỏ qua**. Đây là họ khuyết tật *"công cụ trả lời đúng câu hỏi nó được hỏi"* | Sau mọi lần đồng bộ cây nguồn: **`find <cây> -type f -exec touch {} +`**, hoặc dùng `cp -r` (cp không giữ mtime khi thiếu `-p`). Sau khi `touch`, cùng bản vá đó cho **24/24 PASS**. 🔑 Nghi ngờ thì so trực tiếp: `stat -c '%y %n'` trên file nguồn và trên `.o` tương ứng trong `build/` — **nguồn phải MỚI HƠN** |

## 24. Bẫy gom P11 (2026-08-25)

| Triệu chứng | Nguyên nhân | Cách đúng |
|---|---|---|
| Cổng báo *"FATAL: a build/test run holds `install/`"* trong khi **không** có build nào của mình chạy | **Container Docker dùng chung PID namespace nhìn từ host** ⇒ `colcon build` chạy **bên trong** image build hiện ra với `pgrep -x colcon` trên host. Nó không thể chạm `install/` của workspace, nhưng mọi cổng dùng phép kiểm đó đều **chặn oan** | Lọc theo tiến trình **của host**: `readlink /proc/<pid>/root` phải bằng `/`. Tiến trình trong container trỏ vào filesystem của container. Đã áp ở `sync_build_package.sh`, `run_p11_sim_gates.sh`, `verify_pilot_override.sh` |
| `colcon build` một package chết ở `ament_package_xml.cmake` với *"returned error code 1"*, không nói vì sao | **XML cấm `--` bên trong comment.** Một ghi chú kiểu `... "PIL" -- the host had ...` làm `package.xml` **không parse được**, và thông báo lỗi của ament không hề nhắc tới XML | Đừng dùng dấu gạch đôi trong comment của `package.xml`/`.sdf`/mọi file XML. Triệu chứng "CMake chết ở bước đọc package.xml" ⇒ nghi XML trước, đừng nghi CMake |
| Xây image từ đầu chết vì thiếu một module Python mà máy host vẫn build ngon | **Phụ thuộc chưa khai báo, bị che bởi gói cài sẵn toàn hệ thống trên host.** Ca thật: `uav_sim_gz/scripts/generate_ground_texture.py` cần Pillow lúc **build**, `package.xml` không khai; host có sẵn `python3-pil` nên không ai thấy | Đây là **giá trị chính** của việc dựng image sạch, không phải tác dụng phụ. Sửa ở `package.xml` (khai `<build_depend>`), **đừng** `apt install` trong Dockerfile để che |
| Hai lượt build/chạy dài cùng `tee` một file log | Lượt sau **cắt cụt** file của lượt trước ⇒ đọc ra một bản trộn, và tôi đã kết luận nhầm *"cả ba build đều fail"* trong khi lượt đang chạy về sau báo OK | Log của mỗi lượt vào **file riêng**, hoặc đọc output do harness bắt riêng cho từng task |
| Bộ đếm/cờ trong ulog cho ra tần suất suy ra từ **số bản tin** | PX4 **phát lại** bản tin `estimator_event_flags` mỗi giây khi không có sự kiện mới (`EKF2.cpp:1129`), mọi bit latched còn nguyên | Đếm bằng **bộ đếm sự kiện** (`information_event_changes`), không đếm bản tin. Chi tiết + số đo → `package-status.md` mục nợ EKF2 EV-reset |

## 25. Harness tiêm tải CPU tự làm bẩn mọi phép đo sau nó (2026-08-25, phiên nợ #15)

**Triệu chứng:** cổng và test cho kết quả không tái lập được, "chập chờn" ở những fixture khác nhau
mỗi lượt — trong khi không ai đang chạy gì.

**Nguyên nhân:** script thí nghiệm khởi động các luồng quay tròn để ép tải, rồi dọn bằng
`kill $(jobs -p)`. Khi script bị thay, bị giết, hoặc chạy chồng, đám luồng đó **sống sót**. Đo được
lúc phát hiện: **18 spinner mồ côi, load average 23,46**, đúng trong lúc đang đọc kết quả cổng.

**Vì sao nguy:** nó không phát ra tín hiệu nào. Một cổng đỏ vì máy bận trông y hệt một cổng đỏ vì sản
phẩm hỏng — và cả buổi có thể trôi vào việc truy một lỗi không tồn tại. Cùng họ với ca G-O1 (build song
song **tráo `install/` giữa thí nghiệm**, phải bỏ 10 vòng đo).

**Cách đúng — ba thứ, thiếu cái nào cũng hỏng:**

| | |
|---|---|
| Spinner phải có **tên riêng** | `exec -a uavloadspinner bash -c 'while :; do :; done'` ⇒ dọn được kể cả khi mất PID |
| Dọn bằng **`trap`** | `trap stop_load EXIT INT TERM` — không dựa vào script chạy tới dòng cuối |
| **Tự từ chối đo** khi máy chưa sạch | `require_idle_machine`: đỏ nếu còn spinner mồ côi **hoặc** còn `colcon` đang bay (cùng họ luật `install/` sạch của R15) |

Bản dùng được: `~/loadlib.sh`. 🔑 **Chốt chặn này bắt được lỗi thật ngay lần chạy đầu tiên** — nó từ
chối một thí nghiệm vì còn `colcon` sót lại từ lượt trước.

🔑 **QUY TẮC CHUNG rút ra sau khi dính BA LẦN trong một phiên** (2026-08-26): *một tiến trình không
thể tìm tiến trình khác bằng chuỗi mà chính dòng lệnh của nó đang chứa.* Ba biến thể đã trả giá:
`pkill -f "[c]olcon"` gõ thẳng trong `bash -lc` (tự giết mình, exit 9) · một file tên `whocolcon.sh`
(chốt chặn thấy chính người gọi) · một **heredoc chứa chữ `colcon`** viết script ra đĩa (dòng lệnh của
shell ngoài mang nguyên nội dung heredoc). **Cách duy nhất chắc chắn: TÁCH LÀM HAI LƯỢT GỌI** — một
lượt ghi file, một lượt phóng, và dòng lệnh của lượt phóng không được chứa mẫu.

🪤 **Và nó dính lại đúng bẫy §22 hai lần:** mẫu `colcon` xuất hiện trong **dòng lệnh của chính người
gọi** (một lần là `pkill -f "[c]olcon"` gõ thẳng trong `bash -lc` ⇒ tự giết mình, exit 9; một lần là
tên file `whocolcon.sh` ⇒ chốt chặn thấy chính tôi và từ chối). **Bracket-pattern chỉ an toàn khi mẫu
KHÔNG nằm trong dòng lệnh của người gọi** — phải đưa qua **file script**, đúng như `run_p12_closing.sh`
đã ghi cho `stop_sim.sh`.

---

*Cập nhật lần đầu: 2026-08-14, khi dọn ghi chú `scripts/` theo R16 (19% dòng là ghi chú, khối dài tới 13 dòng → còn ≤1 dòng/script theo quy tắc, lý do dài chuyển hết vào đây). §7 thêm 2026-08-17; §8–§11 thêm 2026-08-19; §12 thêm 2026-08-20; §13–§14 thêm 2026-08-22 (đợt dọn định kỳ — chuyển 2 bài học công cụ từ memory §7 về đúng nhà); §15 thêm 2026-08-22 (thi công P9); §16 thêm 2026-08-23 (chiến dịch cổng bay P9); §17 thêm 2026-08-23 (tích hợp P10.6); §18 thêm 2026-08-23 (vá `offered_qos_profiles` + chẩn đoán G-O3(c), P10.8c); §18 dòng cuối thêm 2026-08-24 (P10.9a, rerun G-O3(c) sau vá double-writer — loại giả thuyết C3, lộ bất thường replay-clock mới); §18 **ĐÃ CHỐT 2026-08-24** (nguyên nhân: độ phân giải `/clock` ~40 Hz của `ros2 bag play`, không ảnh hưởng bay thật); §19 thêm 2026-08-24 (P10-gate-debt review round 2, N4–N8 — hạ tầng cổng có thể PASS mà không đo được gì, vá kèm bằng chứng offline/shim, KHÔNG chạy ROS thật trong phiên vá); §20 thêm 2026-08-24 (hai lượt test giẫm lên nhau — `check_*` không có nghĩa là rẻ); §21 thêm 2026-08-24 (tham số ROS suy kiểu từ chữ + bộ lọc nuốt bằng chứng); §22 thêm 2026-08-24 (bẫy `pkill -f` tự sát nở rộng theo R34 — mọi đường dẫn `uav_<pkg>/`); §23 thêm 2026-08-24 (WSL2 tự khởi động lại — giết `setsid nohup` và xoá `/tmp`; kèm bracket-pattern `pkill` và bẫy dịch đường dẫn Git Bash → `wsl`); §24 thêm 2026-08-25 (P11: `pgrep` thấy tiến trình trong container · `--` giết `package.xml` · phụ thuộc bị che bởi gói cài sẵn trên host · hai lượt `tee` chung một log · đếm bản tin thay vì đếm sự kiện trong ulog).*

## 26. Ba bẫy công cụ của phiên 2026-08-26 (đóng S7 · vá nợ #19 · S10)

| # | Triệu chứng | Nguyên nhân | Cách đúng |
|---|---|---|---|
| **26a** | `measure_coverage.sh: line 102: ake-args: command not found`, lượt đo chết ở bước 2/5 **sau khi đã test xong 9/10 package** (mất 16 phút máy) | **Sửa một shell script trong lúc `bash` đang chạy chính nó.** `bash` đọc file theo **offset byte**, không nạp hết vào bộ nhớ; chèn thêm dòng làm nó nhảy vào **giữa** một dòng khác — ở đây `--cmake-args` bị cắt cụt thành `ake-args` | **Không sửa script đang chạy.** Chờ nó xong, hoặc sửa một bản sao rồi thay **nguyên tử** bằng `mv` |
| **26b** | Tiến trình nền chết ngay khi lệnh trả về; `colcon build` báo `Aborted` + `SIGHUP` cho 2 package | `nohup ... &` **bên trong** `wsl.exe -e bash -lc` vẫn nhận SIGHUP khi phiên WSL của lệnh đó kết thúc | Chạy nền **bằng harness** (`run_in_background`), đừng tự nền bên trong WSL. Xem thêm §23 |
| **26c** | `grep -c $"\r"` báo **mọi** file đều có CRLF, kể cả file vừa kiểm là LF sạch | `$"..."` là **chuỗi dịch locale**, không phải ANSI-C. Mẫu thành `\r` nghĩa là ký tự `r`, khớp mọi dòng có chữ `r` | Dạng đúng là `$'...'`: `grep -c $'\r' file`. Đây là **tái phát** của cùng họ lỗi đã ghi ở §21 (đo bằng công cụ mà chưa kiểm chính công cụ) |

🔑 **Bẫy 26a và 26b có chung một gốc: tôi coi "chạy nền" là miễn phí.** Cả hai lần, cái chết xảy ra ở
chỗ **ranh giới vòng đời** — script bị sửa dưới chân tiến trình đang đọc nó, và tiến trình bị bỏ rơi khi
phiên đẻ ra nó đóng. Trước khi chạy một việc dài: hỏi *"ai giữ tiến trình này sống, và ai có thể sửa
thứ nó đang đọc?"*

---

## 27. 🔴 Tải CPU nhân tạo đẻ ra chế độ hỏng của RIÊNG nó (2026-08-26)

Để tái lập một test chập chờn, tôi bơm **14 vòng lặp bận** trên máy 16 lõi. Nó tái lập được thật —
nhưng nó **cũng** tạo ra một chế độ hỏng **không tồn tại** trong suite: `FAILED TO MEASURE:
rclcpp_action returned UNKNOWN, no result was delivered`, tức action 30 s không kịp trả kết quả.

⇒ Khi dùng tải nhân tạo để tái lập, **phải phân biệt hai loại đỏ**: cái trùng chữ ký với lượt thật, và
cái do chính tải đẻ ra. Nếu chữ ký khác ⇒ **chưa tái lập được gì**, và kết luận rút ra từ nó là rác.
Phép đo đúng của một fixture chạy trong suite vẫn là **chính lượt suite**.

*(Họ hàng với §25: ở đó harness tự tiêm tải; ở đây chính tôi tiêm.)*

---

## 28. 🔴 Ubuntu tự cập nhật gói sau khi WSL khởi động lại — và nó làm hỏng CẢ MỘT CHIẾN DỊCH ĐO (2026-08-26)

**Triệu chứng.** Năm lượt D3 liên tiếp cho window RTF **0,859–0,946** và **4/5 không arm nổi**, PX4 từ
chối bằng `Preflight Fail: height estimate not stable`. Cùng cổng, cùng ngày, đo rải rác lúc máy yên:
**0,942–0,973** và arm trong **0,1 s / 1 lần thử**.

**Nguyên nhân.** `uptime -s` cho thấy **WSL khởi động lại lúc 12:33:22**, ngay đầu chiến dịch (§23 đã
ghi WSL2 tự tắt/bật). Sau mỗi lần boot, Ubuntu chạy **`unattended-upgrades`** — đo được **10,2% CPU
liên tục hơn 3 phút**, kèm `packagekitd`. Load average lúc đó **2,72 / 4,63 / 3,31**; sau khi nó xong,
**0,01**.

🔑 **Vì sao nó nguy hơn một phép đo lệch bình thường:** nó không chỉ hạ RTF, nó làm **EKF2 không hội tụ
nổi** (`ekf2 missing data` · `Compass Sensor 0 missing` · `vertical velocity unstable` · `Yaw estimate
error`), nên triệu chứng hiện ra ở tầng **hoàn toàn khác** với nguyên nhân. Tôi đã đi truy đường lệnh
(`TEST command dropped: header.stamp older than max_command_age_sec`) và suýt kết luận sai về trọng tài
quyền điều khiển — trong khi thủ phạm là trình cập nhật gói của hệ điều hành.

**Cách đúng — kiểm TRƯỚC khi đo, đừng giải thích SAU khi đo:**

```bash
uptime -s                                   # boot gần đây ⇒ nghi unattended-upgrades
awk '{print $1}' /proc/loadavg              # load1 phải thấp (< ~0.6 khi rảnh)
ps -eo pcpu,comm --no-headers | awk '$1 > 5.0'
```

Bộ chạy chiến dịch nay **từ chối bắt đầu** khi máy chưa yên và ghi `SKIPPED` thay vì một con số —
không đo được thì nói không đo được, đừng in ra một số trông như đã đo (O3).

🪤 **Dấu hiệu nhận ra sớm nhất, và tôi đã bỏ lỡ nó:** lượt RTF **thấp nhất** (0,859) lại là lượt **bay
được**, trong khi lượt 0,946 thì không. **Một biến số không sắp thứ tự nổi kết quả nghĩa là nó không
phải nguyên nhân** — lúc đó phải quay ra ngoài tìm biến ẩn, chứ không phải đào sâu thêm vào sản phẩm.

### 28b. 🔴 Và biến ẩn thật ở NGOÀI WSL: một tiến trình Windows

Dọn `unattended-upgrades`, chờ `load1 = 0,13`, chạy lại — RTF vẫn **0,872** rồi **0,753**, so với
**0,942–0,973** cùng cổng buổi sáng. WSL không thấy gì bất thường vì thủ phạm không nằm trong WSL.

Đo từ phía Windows:

```powershell
Get-Process | Sort-Object CPU -Descending | Select-Object -First 8 Name, CPU, WS
# và đo mức tiêu thụ SỐNG, vì cột CPU là tích luỹ:
$a=(Get-Process <tên>).CPU; Start-Sleep 10; $b=(Get-Process <tên>).CPU; ($b-$a)*10  # % một lõi
```

Kết quả: một trò chơi (`ck3`) giữ **5351 s CPU tích luỹ, 6,69 GB RAM**, và đo sống là **285% một lõi**
— tức **~2,85 / 16 lõi bị lấy đi liên tục**, khoảng 18% cả máy.

🔑 **Bài học kiến trúc, không chỉ bài học vận hành:** `RTF ≥ 0,95` là **tiêu chí đo cái MÁY**, và máy
này dùng chung với Windows. Một vạch phụ thuộc trạng thái host thì **không phải một cổng ổn định** —
cùng một cây nguồn, cùng một cấu hình, cho `0,973` hay `0,753` tuỳ việc người dùng đang mở gì. Trước
khi tin bất kỳ số RTF nào: kiểm **cả hai phía**, WSL **và** Windows.

*(Cùng họ với §25 và §27 — cả ba đều là "phép đo đang đo thứ khác", chỉ khác nguồn tải: harness · chính
tôi · hệ điều hành host.)*

### 28c. Hỏi Windows thì tiện, nhưng nó tắt lúc nào không báo — hãy ĐO thay vì HỎI

Cách ở §28b gọi `powershell.exe` từ WSL. Nó chạy đúng một lần rồi thôi: lần sau trả về

```
/mnt/c/.../powershell.exe: 1: MZ...: not found
```

tức `binfmt_misc` **không còn mục `WSLInterop`** — WSL đem file `.exe` chạy như shell script.
`ls /proc/sys/fs/binfmt_misc/` chỉ còn `register` và `status`. **Một phép kiểm có thể âm thầm ngừng
hoạt động thì không phải phép kiểm** — và nếu nó ngừng đúng lúc host bận, ta lại được một chiến dịch
số rác nữa.

🔑 **Cách bền hơn: đừng hỏi ai đang lấy CPU, hãy đo xem CÒN LẠI bao nhiêu.** Chạy một khối việc cố định
trên mọi lõi và bấm giờ — nó bắt được tranh chấp từ **cả hai phía** ranh giới, không cần interop, không
cần biết tên tiến trình:

```bash
cpu_probe() {          # in ra số giây cho một khối việc cố định trên mọi lõi
  local n s e; n=$(nproc); s=$(date +%s.%N)
  for _ in $(seq 1 "$n"); do ( awk 'BEGIN{x=0; for(i=0;i<30000000;i++) x+=i}' ) & done
  wait; e=$(date +%s.%N)
  awk -v a="$s" -v b="$e" 'BEGIN{printf "%.3f", b-a}'
}
```

**Hiệu chuẩn đo được trên máy này (16 lõi), 2026-08-26:**

| Trạng thái | Thời gian probe |
|---|---|
| Yên (6 mẫu) | **1,637 – 1,840 s** |
| Bị chiếm 3 lõi (≈ đúng mức trò chơi lấy) | **2,121 – 2,276 s** |
| Yên trở lại | 1,637 – 1,678 s |

Trần **2,00 s** nằm giữa hai dải. Lấy **nhỏ nhất của 3 lần** — một mẫu chậm là nhiễu, ba mẫu chậm là
máy bận. ⚠️ Phải **hiệu chuẩn lại** khi đổi máy hoặc đổi số lõi; con số trên không mang đi được.


---

## 29. 🔴🔴 ĐỒNG HỒ WSL BỊ ĐẨY TỚI TỪNG ĐỢT — một nguyên nhân giải thích BA triệu chứng ở ba tầng khác nhau (2026-08-26)

**Đây là mục quan trọng nhất của cả nhóm §25–§29.** Ba thứ tưởng là ba bug riêng hoá ra cùng một gốc,
và gốc đó **không nằm trong dự án**.

### Triệu chứng, ở ba tầng

| Tầng | Triệu chứng | Tôi đã suýt kết luận sai thành |
|---|---|---|
| Đo lường | window RTF tụt **0,955 → 0,854** trên cùng cây nguồn, máy đã kiểm là rảnh | "sim chậm, cần waiver D3" |
| Điều khiển | `control_authority_manager_node: TEST command dropped: header.stamp older than max_command_age_sec` | "trọng tài quyền điều khiển có bug" |
| Ước lượng | PX4 `Preflight Fail: height estimate not stable` ⇒ **arm bị từ chối 124 s / 12 lần thử**, lặp lại y hệt | "EKF2 hoặc model có vấn đề" |

### Nguyên nhân, đo được

```
$ journalctl -b | grep -c "Clock change detected"
19                          # trong một phiên boot, khoảng mỗi 33 giây một lần

$ python3 -c "import time; m0=time.monotonic(); w0=time.time(); time.sleep(30); \
              print((time.time()-w0)-(time.monotonic()-m0))"
+0.522                      # đồng hồ TƯỜNG chạy nhanh hơn monotonic 1,74% trong 30 s
```

**Hai nguồn giờ đánh nhau.** `/etc/wsl.conf` không có mục `[time]` ⇒ **WSL vẫn đồng bộ giờ từ host**
(mặc định), trong khi `systemd-timesyncd` bên trong guest cũng đang chỉnh giờ — và nó poll
`ntp.ubuntu.com` ở `PollIntervalUSec=32s`, **đúng chu kỳ TỐI THIỂU**, dấu hiệu nó không hội tụ nổi và
cứ chỉnh lại mãi.

### Vì sao một cú nhảy giờ phá đúng ba tầng đó

| Cơ chế | Hệ quả |
|---|---|
| `window RTF = Δsim / Δreal`, mà `Δreal` lấy từ đồng hồ tường | một cú đẩy tới **1,5 s** rơi vào cửa sổ 20 s làm `Δreal` phồng **7,5%** ⇒ RTF đọc thấp đi ~0,07 — đúng bằng khoảng cách 0,955 → 0,854 |
| `max_command_age_sec = 0.5` so `now() - header.stamp` | một cú đẩy tới **≥ 0,5 s** làm **mọi** lệnh đang bay lập tức "quá cũ" ⇒ trọng tài loại sạch, offboard không bao giờ khoẻ |
| EKF2 gắn dấu thời gian cảm biến theo đồng hồ hệ thống | thời gian gián đoạn ⇒ hiệp phương sai độ cao không hội tụ ⇒ PX4 từ chối arm, **đúng luật** |

### 🔑 Nó TỪNG ĐỢT, và đó là phần nguy hiểm nhất

Ngay sau khi đo được +0,522 s / 30 s, mẫu 15 s tiếp theo cho **−0,000 s**. Vì vậy phép đo buổi sáng
(0,942–0,973) sạch còn buổi chiều (0,854) thì không — **cùng cây nguồn, cùng cấu hình, cùng cổng.**
Một môi trường hỏng-từng-đợt không làm mọi phép đo sai; nó làm **một số** phép đo sai, và đó chính là
thứ sinh ra "test chập chờn" mà người ta đi vá ở nhầm chỗ.

### Kiểm TRƯỚC mỗi chiến dịch đo

```bash
journalctl -b | grep -c "Clock change detected"     # phải gần 0
python3 -c "import time; m0=time.monotonic(); w0=time.time(); time.sleep(30); \
            print('skew %+.3f s' % ((time.time()-w0)-(time.monotonic()-m0)))"
```

**Cách chữa (một nguồn giờ, không phải hai):** tắt NTP trong guest và để WSL đồng bộ từ host —
`sudo systemctl disable --now systemd-timesyncd`. Đảo lại bằng `enable --now`.

🪤 **Bài học phương pháp:** ba triệu chứng ở ba tầng, mỗi cái đều có một lời giải thích hợp lý **trong**
dự án. Thứ cứu tôi khỏi vá nhầm cả ba là một câu hỏi rẻ tiền: *"có biến nào NGOÀI dự án đổi giữa lượt
đo sạch và lượt đo bẩn không?"* — hỏi nó **trước**, không phải sau khi đã đào ba tầng.

---
