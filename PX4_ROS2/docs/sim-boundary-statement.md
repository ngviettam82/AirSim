# Tuyên bố ranh giới sim ↔ real — P12.7 · 2026-08-25 · ✅ **CHỦ DỰ ÁN ĐÃ KÝ**

> # 🔴 ĐỌC DÒNG NÀY TRƯỚC, KHÔNG CUỘN QUA
>
> **Mô phỏng đã đóng mọi cổng NẰM TRONG TẦM CỦA NÓ. Nó KHÔNG chứng minh máy bay an toàn.**
>
> Ba tầng — **định vị thật · thị giác thật · khí động và điện** — **chưa được chứng minh một chút
> nào**, và một phần trong đó **không bao giờ chứng minh được bằng mô phỏng này**.
>
> Câu duy nhất mà toàn bộ công việc P0–P12 mua được là:
> ✅ *"Đã đóng phạm vi thi công phần mềm trong mô phỏng."*
> ❌ **KHÔNG** phải *"mô phỏng đã hoàn thiện"*. ❌ **KHÔNG** phải *"đã an toàn"*.

> ## ⚠️ Rủi ro lớn nhất của chính tài liệu này
>
> **Viết xong rồi coi như "đã xử lý ranh giới".** Danh sách B dưới đây **không giảm rủi ro một chút
> nào** — nó chỉ làm rủi ro **nhìn thấy được**. Một mục nằm trong B vẫn nguy hiểm đúng như trước khi
> nó được viết ra. Thứ duy nhất chuyển được rủi ro là **một phép đo trên phần cứng đích**.

---

## §0b. Mức xác minh của **chính tài liệu này**

Người ký cần biết tài liệu này đáng tin tới đâu, nên đây là trạng thái thật của nó.

| | |
|---|---|
| **Nguồn** | Một vòng khảo sát có hệ thống trên 6 tài liệu bắt buộc + 4 README package + plan P11, mỗi mục kèm `file:dòng` |
| **Đã TỰ KIỂM LẠI trong phiên viết** | **12/65 mục** của Danh sách B — chọn theo *"sai thì nguy nhất"*: toàn bộ nhóm **B-e** đã kiểm (B-32 · B-34 · B-35 · B-56 · B-43), cộng B-01 · B-10 · B-13 · B-25 · B-30 · B-31 · B-49. **Tất cả đều đứng vững đúng nguyên văn**. ⚠️ **Bổ sung 2026-08-25 (P12.6):** thêm **B-61 · B-62 · B-63**, cả ba là **số đo trong phiên này**, không phải suy luận — nguồn nêu ngay trong từng dòng (bag `uav0_20260825_073235Z`, log `07_32_24.ulg`) |
| **Chưa tự kiểm lại** | 48 mục còn lại — dựa trên tài liệu dự án, chưa đối chiếu lại mã nguồn trong phiên này |
| **Một mục không kiểm được từ đây** | **B-03** (`CMakeLists.txt` của plugin optical flow rỗng) — cây nguồn PX4 không nằm trong repo. Khẳng định này dựa **hoàn toàn** vào tài liệu dự án |
| **Hai mục đã LẠC HẬU và đã viết lại** | **B-45** (nợ #15 — nửa đã đóng 2026-08-25) · **B-47** (`uav_bringup` nay có ctest). Chúng lạc hậu **trong vòng một ngày** — đó là tốc độ mà tài liệu này sẽ mục nếu không ai rà lại |

🔑 **Hệ quả cho reviewer:** việc đáng làm nhất khi rà tài liệu này **không phải** đọc lại 65 mục đã có —
mà là hỏi **"còn thiếu mục nào"**. Một mục thiếu ở Danh sách B là một thứ người ta sẽ **tin nhầm rằng
đã được chứng minh**.

---

## §1. Danh sách A — sim ĐÃ chứng minh được gì

**Cột "điều kiện hiệu lực" là cột quan trọng nhất và hay bị bỏ nhất.** Không con số nào ở đây tự nó
chuyển sang đời thật; mỗi con số chỉ đúng **trong điều kiện nó được đo**.

**Cột "chuyển giao" chỉ có ba giá trị:**
**CÓ** (logic · giao thức · hợp đồng dữ liệu) · **PHẢI ĐO LẠI** (mọi thứ có hằng số thời gian, gain,
hoặc ngưỡng vật lý) · **KHÔNG** (mọi thứ đi qua cảm biến).

> 📌 Số đo **không chép vào đây** — trỏ về nguồn. Dự án đã trả giá vì cùng một con số nằm ba nhà rồi
> lệch nhau (ca thật: 49 · 61 · 66 cho cùng một số test).

| # | Đã chứng minh | Cổng / nguồn | Điều kiện hiệu lực | Chuyển giao |
|---|---|---|---|---|
| **A-01** | Bài bay hồi quy M5 arm→takeoff→goto→land→disarm, đường cơ sở liên tục từ P3 | `run_m5_regression.sh` · memory §4 | model `uav0` (**0 cảm biến render**), RTF mean 1,000, không tiêm nhiễu | **PHẢI ĐO LẠI** |
| **A-02** | Backend PX4 5/5 node: chuỗi offboard IDLE→STREAMING→ENGAGING→ACTIVE, arm, nhịp 20 Hz | package-status §2 | SITL, `uav0` | **CÓ** (giao thức) |
| **A-03** | Navigator đủ 7 action bay thật; G-N1 8/8 · G-N2 15/15 · G-N3/N4b/N5/N6 | memory §4 | `uav0_nav`/`uav0_track`, world `uav_arena` | **PHẢI ĐO LẠI** (mọi ngưỡng khoảng cách/thời gian) |
| **A-04** | Trọng tài là single-writer **DUY NHẤT** của `command_selected` — quan sát trực tiếp trên hệ đang chạy, không phải giả định | package-status §9 | SITL | **CÓ** |
| **A-05** | Safety enforcement **THẬT** (không dry-run) đã bay: G-S1 13 mục · G-S2 4/4 · G-S3 · V1; ClearFault gỡ latch đã bay | package-status §10 | `uav0_nav_indoor` cho G-S2 | **CÓ** (luật) · **PHẢI ĐO LẠI** (ngưỡng) |
| **A-06** | Mission BT end-to-end 9/9 cổng, cả 3 mission bay trọn | package-status §11 | SITL | **CÓ** |
| **A-07** | Tầng bằng chứng 4 cổng PASS; G-O1 **lật quyết định storage bằng thực nghiệm** (mcap mất 100% bag khi bị giết) | package-status §12 | SITL, domain 99 | **CÓ** (cơ chế) · **PHẢI ĐO LẠI** (chi phí CPU/đĩa) |
| **A-08** | Bay **trong nhà không GPS, không la bàn** PASS 3/3 — vị trí và cao độ đều từ nguồn vision | package-status §5 | 🔴 nguồn vision là **ground truth** (B-01) | 🔴 **KHÔNG** |
| **A-09** | G4 bất đồng nguồn định vị PASS 4/4, **kèm đối chứng âm** (drift OFF ⇒ mức OK) | `run_g4_gps_disagreement.sh` | mức tiêm nhiễu cụ thể của cổng | **PHẢI ĐO LẠI** |
| **A-10** | Nợ EKF2 EV-reset đóng **bằng ĐO, không bằng sửa code**; con số cũ "71 lần/70 s" là đếm **bản tin phát lại**, lệch 43× | `gate_e1_ev_reset.sh` | `uav0_nav` đậu | **CÓ** (kết luận) |
| **A-11** | Parity sim/real là bất biến kiểm tự động; `real.launch.py` **TRƠ** khi thiếu `reviewed:=true` | `check_sim_real_parity.py` | tĩnh | **CÓ** |
| **A-12** | Ranh giới `px4_msgs` (R1) và profile DDS sim-only là **bất biến kiểm tự động**, có allowlist khai báo | `check_px4_msgs_boundary.sh` | tĩnh | **CÓ** |
| **A-13** | Nguyên nhân gốc mất khung ảnh truy tới **cơ chế**: vách là segment SHM mặc định Fast DDS, không phải cảm biến | package-status §4 | 🔴 **loopback WSL** (B-20) | **PHẢI ĐO LẠI** |
| **A-14** | World model chuyển frame đúng; uncertainty nở ×8,38 khi VIO mất tư cách | package-status §7 | tĩnh + lát bay | **CÓ** (hình học) |
| **A-15** | Cổng **TĨNH** của perception đạt (marker, hộp vật cản biết trước) | package-status §6 | 🔴 **tĩnh** — cổng động D1/D2/D3 **chưa đạt** (B-26) | 🔴 **KHÔNG** |
| **A-16** | Quy mô kiểm thử: **12/12 package có ctest**, chéo kiểm tự động khớp | `verify_workspace.sh` | — | **CÓ** |
| **A-17** | Docker **một Dockerfile hai kiến trúc** dựng được; build sạch còn lộ một phụ thuộc chưa khai | plan P11 §0b | 🔴 arm64 mới xong rosdeps, **chưa chạy** (B-21) | **PHẢI ĐO LẠI** |
| **A-18** | Độ cứng plant giả trong test được **kiểm** thay vì khẳng định: 85/87 phán quyết không đổi giữa hai độ cứng | `audit_plant_stiffness.sh` | — | **CÓ** (phương pháp) |
| **A-19** | Đo được ai thật sự ngăn livelock trong hình lõm, và câu trả lời **NGƯỢC giả thuyết**: tuyến toàn cục ngăn, không phải ghép tuyến | package-status §8 | fixture | **CÓ** (kết luận) |
| **A-20** | Lỗi *"chỉ tồn tại ở đời thật"* (stack giành lại máy bay khỏi pilot) đã phát hiện và vá | package-status §2 | 🔴 **chưa có radio thật** (B-06) | **PHẢI ĐO LẠI** |

---

## §2. Danh sách B — sim KHÔNG (bao giờ) chứng minh được gì

**Sáu nhóm, để không mục nào bị đọc lướt.** Nhóm **B-e** có dấu 🔴 riêng: đó là **nợ còn sống trong
code SẼ BAY**, không phải chuyện "sim không mô phỏng".

### B-a · Cảm biến trong sim là **nói dối có chủ đích**

| # | Sự thật |
|---|---|
| **B-01** | 🔴 **"VIO" trong sim là GROUND TRUTH TUYỆT ĐỐI** — không nhiễu, không trôi, không mất bám, không trễ. Mọi kết luận về định vị dựa trên sim **không có hiệu lực** ngoài đời; thuật toán ăn nguồn này trông **giỏi hơn thực tế rất nhiều** |
| **B-02** | 🔴 **Rangefinder KHÔNG vào EKF2 trong sim** (cầu `gz_bridge` PX4 v1.15 không nhận distance sensor) nhưng trên drone thật nó thường nuôi EKF2 ⇒ **khác biệt KIẾN TRÚC**, hành vi ước lượng cao độ khác hẳn |
| **B-03** | **Optical flow chưa từng được test** — plugin PX4 v1.15.4 có thư mục nhưng `CMakeLists.txt` rỗng ⇒ `optical_flow_adapter_node` chưa bao giờ có nguồn dữ liệu |
| **B-04** | **Perception chưa từng thấy ảnh thật.** Driver camera thật là P11.3, đang chặn |
| **B-08** | 🔴 **GPS trong sim là nhiễu TRẮNG hardcode** (σ ngang 0,2 m), thực tế GPS **trôi 2–10 m CÓ TƯƠNG QUAN** ⇒ nhỏ hơn 10–50× **và sai bản chất**: nhiễu trắng thì lọc là hết, trôi thật thì không |
| **B-09** | SDF **không mô hình được TRÔI** cảm biến — lidar/camera chỉ có mean+stddev; chỉ IMU có dynamic bias |
| **B-17** | Nhiễu IMU/GPS/Mag dùng **mặc định của x500**, chưa hiệu chỉnh theo cảm biến thật nào |
| **B-18** | **Toàn bộ số cảm biến tự thêm là PLACEHOLDER**, chưa lấy từ datasheet nào (rangefinder 12 m, camera dưới fov 70°…) — đều ghi *"chưa xác minh"* |
| **B-19** | Độ phân giải camera trước 640×480@15 Hz chọn theo **phép đo RTF, không theo datasheet** — *"ràng buộc tính toán, không phải thông số thiết bị"* |
| **B-48** | Toạ độ gốc & từ trường world mặc định là **Zurich**. Đổi sang toạ độ Việt Nam mà quên `magnetic_field` thì EKF2 lệch heading ⇒ sim **không chứng minh gì** về la bàn tại bãi bay thật |

### B-b · Vật lý **KHÔNG mô hình**

| # | Sự thật |
|---|---|
| **B-05** | 🔴 **Mọi số vật lý là số MƯỢN của x500.** Tài liệu ghi nguyên văn: *"Không dùng bất kỳ số nào ở đây làm căn cứ cho bay thật."* Biên điều khiển, thời gian đáp ứng, thời gian bay đều **SAI** cho drone của dự án |
| **B-10** | **Gió KHÔNG bật** — không nhiễu loạn nào ⇒ bám điểm trong sim **luôn đẹp hơn thật**. Cấm lấy sai số bám điểm sim làm chỉ tiêu nghiệm thu |
| **B-11** | **Ground effect không mô hình** ⇒ hạ cánh thật sẽ "nổi" hơn. Cấm tune gain hạ cánh chỉ bằng sim |
| **B-12** | **Downwash / blade flapping không mô hình** ⇒ phải **tăng** khoảng cách an toàn tối thiểu khi bay thật |
| **B-13** | **Sụt áp pin theo tải không mô hình** ⇒ **không dùng thời gian bay của sim để lập kế hoạch** |
| **B-14** | Mật độ không khí khai `adiabatic` nhưng **không nối vào lực đẩy** ⇒ bay cao/trời nóng thì lực đẩy thật kém hơn |
| **B-15** | **Rung động khung không mô hình** ⇒ rung thật làm hỏng IMU và làm mờ ảnh. *"Không có cách bù trong sim."* |
| **B-16** | **Va chạm mềm / biến dạng càng đáp không mô hình** (thân cứng tuyệt đối) ⇒ ngưỡng phát hiện tiếp đất **lạc quan hơn thật** |
| **B-52** | Ba thí nghiệm vật lý đối chứng **E1/E3/E9 chưa chạy** — và chạy chúng trên x500 thì *"chỉ nghiệm thu lại x500, không nói gì về drone của dự án"* |

### B-c · Phần cứng **chưa tồn tại**

| # | Sự thật |
|---|---|
| **B-06** | 🔴 **RC override chưa từng có radio thật.** Rig SITL không tạo được đồng thời "đã arm" và "PX4 ở chế độ tay" |
| **B-07** | 🔴 **INHIBIT chưa một lần kích hoạt trên phần cứng.** Quyết định để `safety_enforcement=TRUE` phía real là **lý lẽ, không phải số đo** |
| **B-20** | Vách DDS large-samples và bản vá của nó **đo trên loopback WSL**; máy thật đi qua giao diện vật lý (R31) |
| **B-21** | 🔴 **Chưa một node nào chạy trên arm64 thật** |
| **B-22** | Benchmark chỉ trên workstation là **"bẫy chí mạng"**; HITL + Jetson + radio thật **chưa chạy** |
| **B-23** | Chi phí CPU/đĩa hộp đen trên companion computer **chưa đo** (máy chưa mua) |
| **B-24** | Cổng **G-O4 (g)/(g2)** — SITL bay thật của đèn go/no-go — **chưa chạy**, có chủ đích |
| **B-50** | 🔴 **Sim không chứng minh được NỬA VÔ TUYẾN.** Bước 6 hiện trường phải thử cướp quyền trên **sóng thật** |
| **B-51** | Mép đã biết: chốt pilot-override đóng khi **thấy** chế độ tay trong topic phát **~2 Hz** ⇒ cú cướp quyền **ngắn hơn một mẫu là VÔ HÌNH** |
| **B-53** | 🔴 Checklist tiền bay **chưa từng dùng**; **cả hai ô chữ ký chưa ký**; §1 còn **5 ô ⬜ chặn cứng** |
| **B-64** | 🔴 **CPU bão hoà làm mission tự huỷ một bước ĐÃ THỰC THI ĐÚNG — đo được 2026-08-25, và headroom của máy thật thì chưa ai đo.** Dưới 16 luồng quay tròn, `rclcpp_action` Humble trả `WrappedResult.code = UNKNOWN` kèm payload **mặc định hoàn toàn** (`message='' result_code=0 executed_type=0`) cho một goal `Recover` mà server **đã nhận và đang chạy** — trả lời chỉ **2 ms sau `accepted recover goal`**, trước cả khi node kịp thực thi. Rằng sản phẩm không sai được chứng minh ba lớp: mọi nhánh kết thúc của navigator đều điền `message` khác rỗng (nên payload rỗng không thể là của ta) · log không có nhánh abort nào · khẳng định *"máy bay đứng yên đúng chỗ"* trong cùng lượt **không đỏ**. `uav_mission` xử lý đúng hướng an toàn (`mission_executor_node.cpp` nhánh `default:` → `kResultAbortedInternalError`, *"navigator action client reported UNKNOWN (no result)"*) ⇒ hỏng về phía **huỷ**, không phải phía *"tưởng thành công"*. **Nhưng hệ quả vận hành là thật:** một companion computer thiếu headroom CPU có thể **huỷ một bước mission đã làm xong**. Tỉ lệ đo được: **0/10 lượt** khi máy rảnh · **0/20** dưới 8 luồng · **6/30** dưới 16 luồng ⇒ **ngưỡng nằm giữa 8 và 16 luồng trên workstation này, và con số đó KHÔNG chuyển sang máy bay** (R31). ⚠️ **Chưa từng thấy dưới tải thật của `colcon test`** — nên nó là **rủi ro phần cứng**, không phải bug phải vá; cửa đóng nó là **P11.5 đo headroom trên máy đích**, xem mục kiểm ở [`preflight-checklist.md`](preflight-checklist.md) |

### B-d · Thị giác & world

| # | Sự thật |
|---|---|
| **B-26** | 🔴 **Cổng ĐỘNG của perception D1/D2/D3 CHƯA ĐẠT** ⇒ **không có bài "drone đang bay VÀ mục tiêu di chuyển" nào được nghiệm thu** |
| **B-27** | `object_detector_node` chỉ là **khung** (HOG giữ chỗ): không AP, không benchmark, score không hiệu chuẩn ⇒ sim **chưa chứng minh gì** về nhận dạng vật thể |
| **B-28** | Độ thật thị giác của Gazebo **không đủ cho DNN** — đó là lý do world 3D Bách Khoa bị đóng & cất dù chạy được |
| **B-29** | Hình học world từ ảnh khảo sát sai **±1–1,5 m**, không phải ±0,2 m ⇒ biên an toàn phải đặt ở **inflation của planner**, không nướng vào world |
| **B-30** | 🔴 **Cảm biến render đọc `<visual>`, không đọc `<collision>`.** Và RTF 0,98 hiện tại là bằng chứng **chưa có mesh collision nào**, **không** phải bằng chứng còn dư sức *(P12.2 nay có `sdf_invariants.py` canh bất biến này tự động)* |
| **B-31** | `laser_retro` khai trong SDF 1.11 nhưng *"sẽ hiện thực ở bản sau"* ⇒ hạ cánh chính xác dựa vào **cường độ phản xạ** có lỗ hổng cứng |
| **B-36** | 🔴 **Tham số gom cụm vật cản ghép NGẦM vào độ phân giải** — `min_cluster_points` đếm **pixel** nên ý nghĩa vật lý co giãn theo **bình phương** độ phân giải. Đổi camera depth ⇒ hình học vật cản đổi, **không cảnh báo nào**. Bằng chứng: vật cản/khung nhảy 3→12 ở nửa độ phân giải |
| **B-37** | `TargetTrack.vx/vy` **không đáng tin** để giữ khoảng cách với mục tiêu di động: worst-case **71%** sai số vận tốc. **Giới hạn thông tin, không phải lỗi hiện thực** |
| **B-38** | `Obstacle.center` **thiên lệch về mặt gần +0,255 m** (camera đơn chỉ thấy một mặt). Consumer giữ khoảng cách theo center thật ra đang giữ tới **bề mặt**. **Không sửa được bằng tuning** |
| **B-39** | Bộ lọc mặt phẳng nền **không phân biệt được** nền thật với mặt phẳng ngang khác ≥0,15 m dưới camera (nóc thùng, nóc xe). Rủi ro thấp **nhờ hình học đang dùng**, không phải nhờ thuật toán |
| **B-40** | Sai số dư của marker **chưa truy nguyên xong** — hai lượt đo độc lập không đồng ý về cách phân tách |
| **B-49** | 🔴 **Chưa từng bay một chuyến đáng tin với đủ 4 cảm biến render.** `uav0_full` đo RTF **0,79–0,86** < ngưỡng tin cậy 0,95 |
| **B-57** | Sàn bất định obstacle/target là **số đặt tạm** ⇒ mọi `position_uncertainty` phát ra `/world/*` mang một **cận dưới chưa hiệu chuẩn** |

### B-e · 🔴 Nợ **còn sống trong code SẼ BAY**

> Nhóm này khác hẳn năm nhóm kia. Đây **không** phải "sim không mô phỏng được X" — đây là **khiếm
> khuyết đang nằm trong phần mềm sẽ chạy trên máy bay**. Nếu trình ngang hàng với 65 mục thì chúng
> chìm, nên chúng đứng riêng.

| # | Sự thật |
|---|---|
| **B-65** | 🔴 **`Costmap` suy ĐỘ TƯƠI từ sai đại lượng — perception khoẻ báo "trời quang" đọc GIỐNG HỆT perception đã chết.** `costmap.cpp:169-172` chỉ đẩy `last_input_stamp_` lên **sau khi vật cản đã qua bộ lọc dải bay**, và `local_planner_node.cpp:226-234` chỉ gán `header.stamp` **bên trong vòng lặp vật cản** ⇒ độ tươi đo *"có vật cản nào được **GHI**"*, không phải *"có bản tin nào **TỚI**"*. Đo 2026-08-26: mảng rỗng → `Hold`, `map_age=inf`, *"no obstacle input has ever arrived"*; **2 vật cản TƯƠI NGUYÊN nhưng ngoài dải bay → y hệt**; 1 vật cản trong dải → `Clear`, `map_age=0`. Hướng hỏng **về phía an toàn** (Hold, không bay mù), nhưng hệ quả vận hành thật: bay ra khoảng trống ⇒ advisor Hold dù perception hoàn hảo, và `require_obstacle_feed=true` là **mặc định phía real** (D1) ⇒ planner từ chối lập kế hoạch. **Chưa vá** — chờ chủ dự án quyết. Cùng họ `peakAcceleration()` (CLAUDE.md §5) |
| **B-32** | 🔴 `in_air` suy từ `takeoff_time > 0` ⇒ **sai sau khi hạ cánh, kẹt `true` vĩnh viễn từ chuyến bay thứ hai**. Đèn go/no-go đã phải **loại `in_air`** khỏi vị ngữ pha vì lý do đó |
| **B-33** | 🔴 Cờ sức khoẻ cảm biến là **suy diễn** từ `failsafe_flags` — cờ của **bộ ước lượng**, không phải cảm biến thô; `magnetometer_ok` mượn `attitude_invalid` làm đại diện |
| **B-34** | 🔴 **Bậc thang gia tốc 48,2 / 50,6 m/s² khi dây xích KẸP**, đo trên chuỗi setpoint **đã phát**, độc lập với plant ⇒ thuộc tính của **luật**. Đường dựng-lại-kế-hoạch bị chặn 3,0 m/s² còn đường kẹp cao **~16×** và **không ngưỡng nào chặn**. Chưa chứng minh PX4 nuốt được |
| **B-35** | 🔴 Cấu hình **không có `route_planner_node`** thì **không có gì đưa máy bay ra khỏi hình lõm** — `avoidance_hold_timeout_sec` chỉ biến livelock thành abort, **không phải thành lối ra** |
| **B-41** | Trần tốc độ thật do **dây xích** đặt (≈0,76 m/s ngang), suy từ `K_p≈0,95` của **hồ sơ PX4 x500** ⇒ tuning FC phải làm lại theo airframe thật |
| **B-43** | Bộ tiêm nhiễu GPS **không kiểm được gì ở đầu ra** — mux **luôn** chọn VIO ở cả hai điều kiện; GPS chưa từng được công bố lần nào |
| **B-44** | Với model **một nguồn định vị**, `/state/localization_health` ở **WARN thường trực** — *"không kiểm chéo được thì không được nói khoẻ"* |
| **B-56** | Nợ #12: `escape_replan_interval_sec` / `escape_refresh_m` chưa có test nào làm chúng là thứ **duy nhất** chặn; nhánh `planner_fault_` chưa test ⇒ **vùng chưa phủ nằm đúng trong logic thoát hiểm** |
| **B-58** | `ESTIMATOR_INPUT_INVALID` ở world indoor là **ERROR vĩnh viễn kể cả đang bay**, cố ý không miễn-trừ; **chưa chạy lại** sau khi thiết kế lại đèn |
| **B-59** | **Cửa sổ mù mở-nguồn→ARM** của đèn go/no-go: miễn-trừ preflight có thể bay hơi ≤1 tick. *"Không có biện pháp kỹ thuật triệt để — chấp nhận có ý thức"*, chỉ giảm thiểu bằng **quy trình** |
| **B-60** | Hành vi **đã ký** phải nhớ khi ra hiện trường: arm cạnh vật cản < 0,35 m ⇒ `OBSTACLE_TOO_CLOSE` **latch NGAY**, chỉ gỡ bằng ClearFault. Lựa chọn có chủ đích, nhưng là **cái giá vận hành** phải vào quy trình |
| **B-61** | 🔴 **Bài hồi quy M5 chạy cả tầng tự hành trên một pose mà chính playbook nói là không dùng để chấm được.** [`ops-playbook.md`](ops-playbook.md) §4 đã ghi: model `uav0` **không có nguồn vision**, nên mux rơi về GPS, và *"`uav0` chỉ hợp cho bài đóng vòng trên `odometry_raw`"*. M5 chấm **đúng** như vậy — nhưng nó cũng dựng **planner, safety và world model**, mà cả ba chỉ đọc `/state/odometry_fused` (R: một điểm hợp nhất duy nhất). Đo 2026-08-25, bag `uav0_20260825_073235Z`, khi treo 35–45 s: `odometry_fused` có \|dz\| **p50 37,1 cm · p95 132,2 cm · max 221,3 cm**; `odometry_raw` cùng lúc là **p50 0,1 cm · max 2,2 cm**. ⇒ **Mọi hành vi của planner/safety quan sát được trong M5 là hành vi ở mức nhiễu ±1,3 m, và chưa từng có ai nói ra điều đó.** Ranh giới đúng: M5 chứng minh **vòng điều khiển**, **không** chứng minh gì về planner/safety. Trên `uav0_nav`/`uav0_track`/`uav0_full` mux chọn VIO (xem **B-43**) nên con số này **không** chuyển sang các model đó |
| **B-62** | 🔴 **EKF2 tự reset quaternion GIỮA CHUYẾN BAY.** Đo trên `07_32_24.ulg`: `reset_count_quat` 0→2, lần hai tại **t=49,2 s** trong lúc đang armed; đúng khoảnh khắc đó `estimator_status.mag_test_ratio` **phi hữu hạn** (9 mẫu). Lần arm kế bị `COM_ARM_EKF_YAW` từ chối, **trong khi yaw chỉ lệch 1,67° so với sự thật nền Gazebo**. Từ trường ở bãi bay thật khác hẳn ⇒ **phải đo lại**, và quy trình arm phải chịu được một lần từ chối tạm thời |

### B-f · Phép đo của **chính dự án** có giới hạn

| # | Sự thật |
|---|---|
| **B-25** | 🔴 **`BLIND_COMMAND` chưa từng nổ trong bay thật.** G-S2 đạt là nhờ **lớp 1** (navigator tự đóng băng setpoint) cắn trước. Đọc G-S2 thành *"đường cắt BLIND_COMMAND đã verify"* là tin vào **một lá chắn chưa từng cắn** |
| **B-42** | 🔴 **NGƯỠNG KHÔNG KÈM ĐIỀU KIỆN ĐO KHÔNG PHẢI CỔNG.** Ngưỡng G2 0,5 m chỉ có nghĩa **kèm** mức nhiễu σ=5 m τ=60 s. ⇒ **Không con số cổng nào của sim tự nó chuyển sang đời thật** |
| **B-45** | ⚠️ **CẬP NHẬT 2026-08-25:** nợ #15 **một nửa đã đóng** — `mission_executor_node` từng phát mẫu terminal `goal_id` rỗng (**lỗi sản phẩm thật**, vá + đối chứng hai chiều, P12.1). **Nửa còn lại thu hẹp 2026-08-25**: `CutChainFixture.Item8` đã truy ra gốc — phán quyết đếm theo **lúc NHẬN** thay vì **lúc PHÁT** (`header.stamp`), nên nó quy tội supervisor cho độ trễ của middleware; sửa xong đo **6/6 đỗ dưới tải đầy 16 lõi**, trước đó hỏng ~2/3 lượt. **Fixture thứ hai cũng đã truy xong cùng ngày**: `MissionExecutorFixture.AuthoritySeizeSustainedCancelsAndPauses` đo `0,281 s` so với ân hạn 1,5 s — vì test đo từ **lời gọi hàm của nó** trong khi executor coi **authority hết tươi (1,0 s) cũng là mất quyền**, nên thread phát của fixture bị đói là ân hạn đã chạy trước; **executor đúng luật**. **Fixture thứ ba, họ KHÁC:** `uav_navigation.StaleRouteFixture` — `plan_hz = 5,0` phát tuyến mỗi **200 ms** trong khi ngưỡng tươi của fixture là **20 ms** ⇒ **10% số lượt** cổng im **không phải lỗi của cổng**; tiền đề của test dựa vào **may** thay vì được dựng. Vá bằng cấu trúc: ngừng bộ phát tuyến trước khi kích, tuổi tuyến chỉ tăng. 🔑 **Bài học chung: một test nhạy tải hầu như luôn là test đang đo SAI ĐẠI LƯỢNG, không phải sản phẩm chập chờn** — cùng họ `peakAcceleration()` ở CLAUDE.md §5. 📊 **Tỉ lệ đo được 2026-08-25: 16 lượt gộp, 8 sạch / 8 bẩn — 50%, rơi vào SÁU fixture khác nhau** ⇒ đây là **thuộc tính của cả suite**, không phải một bug. Còn ba chỗ chưa truy: `CutChain.Item1` · `NavigatorTrackingEnvelopeFixture.AHoldRecovery…` (khẳng định **nhịp phát trên cửa sổ giờ tường**, 36 vs 49) · `ControlAuthorityFixture.OnlyTheHigherPrioritySourceReachesCommandSelected`. **CHƯA có lượt 3-sạch-liên-tiếp nào sau ba bản vá** ⇒ **một lượt sạch vẫn không chứng minh gì, và con số "0 failures" của workspace vẫn CHƯA tái lập được**. 🔑 Đo cùng ngày: chạy **`--executor sequential`** thì suite cho **0 errors / 0 failures / 0 skipped** ⇒ nguyên nhân là **tranh chấp scheduler khi chạy song song**, không phải sản phẩm — nhưng điều đó **không** làm con số song song trở nên đúng, vì đó mới là cách suite thật sự được chạy |
| **B-46** | Nhiều số đo thời gian đo dưới scheduler WSL và **đã cắn nhiều lần**. R21/R31: **thời gian tường không bao giờ là cửa sổ chân lý** |
| **B-47** | ✅ **ĐÓNG 2026-08-25 (P12.2)** — `uav_bringup` nay có ctest (11 case tĩnh + đối chứng 8/8); toàn bộ **12/12 package có target** |
| **B-54** | ⚠️ Bảng *"tỉ lệ chuyển giao theo tầng"* (~90–95% logic · trung bình động lực · **thấp** cho định vị & thị giác) là **ước lượng định tính**, không phải số đo. Nó dùng để **chặn** cách đọc *"sim tốt ⇒ thực tế 70–80%"*, **bản thân nó không phải bằng chứng** |
| **B-55** | Kết luận cổng **G-O3(c) chưa chốt** — tài liệu ghi *"đừng trích dẫn kết luận cũ ở đây làm căn cứ cho bất kỳ quyết định nào"* |
| **B-63** | ⚠️ **Mọi lần đọc `LOCALIZATION_JUMP` TRƯỚC 2026-08-25 mô tả một luật sai.** Bộ kiểm liên tục áp biên động học `20·dt + 0,2` lên một dòng **do nhiễu chi phối**, không có số hạng nhiễu nào, dù bản tin **tự khai `position_uncertainty = 0,900 m`**. Cú "nhảy" duy nhất của chuyến đo là **2,217 m** — vượt vạch cũ đúng **17 mm**. Luật đã sửa (cộng 2σ do nguồn khai; σ không khai hoặc vô lý ⇒ **CannotJudge**, không phải OK) và cho **0/661** trên chính chuyến đó, còn dịch chuyển 10 m vẫn nổ |

---

## §3. Bảng **ĐỌC NHẦM ↔ SỰ THẬT**

> Đây là phần chống hiểu sai. Mỗi hàng là một câu người ta **sẽ** nói, và câu trả lời đúng.

| Sẽ có người nói… | Sự thật |
|---|---|
| *"Định vị đã kiểm chứng"* | Nguồn định vị trong sim là **ground truth tuyệt đối**. Con số kiểm chứng chỉ chứng minh **đường ống lành** (frame, dấu thời gian, liên tục), **không** chứng minh chất lượng định vị — `package-status.md` đã ghi thẳng câu này (**B-01**) |
| *"Tránh vật cản đã đạt trong sim"* | Cảm biến render đọc `<visual>`; đổi sang camera depth thật là **hình học vật cản đổi, không cảnh báo**; `center` thiên lệch **+0,255 m** (**B-30 · B-36 · B-38**) |
| *"Safety đã cắt được lệnh mù"* | **`BLIND_COMMAND` chưa từng nổ.** G-S2 đạt nhờ **lớp 1** cắn trước (**B-25**) |
| *"INHIBIT hoạt động"* | **Chưa một lần kích hoạt trên phần cứng.** Quyết định bật là **lý lẽ, không phải số đo** (**B-07**) |
| *"Pilot lấy lại quyền được"* | **Chưa từng có radio thật**; chốt đọc topic **2 Hz** nên cú cướp quyền **< 500 ms là vô hình** (**B-06 · B-51**) |
| *"Suite xanh, 0 lỗi"* | Nhạy tải ở **≥3 fixture**; n=3 lượt gộp cho **PASS·PASS·FAIL**. Một lượt sạch **không chứng minh gì** (**B-45**) |
| *"Thời gian bay / biên lực đẩy đã biết"* | **Toàn bộ là số x500**; pin **không mô hình sụt áp** (**B-05 · B-13**) |
| *"Hạ cánh đã tuned"* | **Không có ground effect**; thân **cứng tuyệt đối** nên ngưỡng tiếp đất **lạc quan** (**B-11 · B-16**) |
| *"Perception đã nghiệm thu"* | Chỉ cổng **TĨNH** đạt. Cổng **động D1/D2/D3 chưa đạt** ⇒ **không có bài "đang bay + mục tiêu di chuyển"** nào được nghiệm thu (**B-26 · A-15**) |
| *"Bay đủ cảm biến rồi"* | **Chưa từng.** `uav0_full` RTF **0,79–0,86** < 0,95 ⇒ mọi số ở cấu hình đó phải dán nhãn *"đo dưới RTF thấp"* (**B-49**) |

---

## §4. Quy tắc sử dụng — để tài liệu này không thành tài liệu chết

1. **Mọi phát biểu *"X đã được kiểm chứng"*** trong bất kỳ tài liệu/PR nào sau ngày ký **phải dẫn
   được về một hàng của Danh sách A, kèm cột điều kiện hiệu lực.** Dẫn không được ⇒ **nó thuộc
   Danh sách B**.

2. **Chuyển một mục từ B sang A chỉ bằng MỘT CÁCH:** một phép đo **trên phần cứng đích**, có script
   tái lập được, có **đối chứng dương** (R27-3). 🔴 **Cấm chuyển bằng lý lẽ.**

3. Tài liệu này là **đầu vào bắt buộc** của [`preflight-checklist.md`](preflight-checklist.md) §1 —
   mỗi ô ⬜ ở đó phải trỏ tới các mục B mà nó đóng.

---

## §5. Chữ ký

| Vai | Điều kiện ký | Ký |
|---|---|---|
| **Chủ dự án** | Đã đọc §0, §2 nhóm **B-e**, và §3 | ✅ **2026-08-25** |
| **Rà nội bộ** | Danh sách B có thiếu mục nào không — phương pháp ở dưới | ✅ **2026-08-25** |

🔴 **KHUYẾN NGHỊ CÒN MỞ — chưa có ai rà ĐỘC LẬP.** Lượt rà ở trên do **chính người viết tài liệu này**
thực hiện, nên theo định nghĩa nó không bắt được thứ mà người viết vốn đã không nhìn thấy. Một lượt
`uav-design-rule-reviewer` trên toàn Danh sách B vẫn còn đáng làm, và **chưa làm**.

*(Ghi chú về chính bảng này: bản thảo đầu ngày 2026-08-25 đặt "reviewer độc lập" thành một ô chữ ký thứ
hai. Đó là điều kiện do người viết tự đặt ra trong cùng phiên, không phải yêu cầu đã ký của dự án —
để nguyên thì cổng S14 sẽ bị chặn bởi một luật vừa tự nghĩ ra vài giờ trước. Nó được hạ xuống đúng cấp
của nó: **khuyến nghị**, ghi rõ là chưa làm.)*

**Phương pháp rà — chạy lại được:**

| Câu hỏi | Lệnh | Kết quả 2026-08-25 |
|---|---|---|
| Còn dấu nợ nào trong mã **sẽ bay** mà Danh sách B không biết? | `grep -rnE '(TODO\|FIXME\|HACK\|XXX)' src --include=*.cpp --include=*.hpp --include=*.py --include=*.yaml \| grep -v /test/` | **0 dấu** |
| Mọi nợ đang mở trong `memory.md` có mặt trong Danh sách B không? | `grep -oiE 'nợ #[0-9]+'` trên cả hai file rồi so tập hợp | Đang mở: **#12 → B-56**, **#15 → B-45**. #1 · #6 · #10 đã đóng. **Không thiếu mục nào** |

*Ký tài liệu này **không cho phép bay**. Nó chỉ ghi nhận rằng ranh giới đã được nhìn thấy.*
