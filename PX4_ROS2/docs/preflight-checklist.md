# Checklist tiền bay — lần bay thật đầu tiên

> **Đây là cổng cuối của P11.** Exit của P11 **không phải một cổng xanh** mà là **chữ ký của chủ dự án
> + reviewer** ở §6. Checklist này tồn tại để chữ ký đó dựa trên thứ đã kiểm, không dựa trên trí nhớ.
>
> Viết 2026-08-25. 🔴 **Chưa từng dùng cho một chuyến bay thật nào** — dự án chưa có phần cứng.

---

## 0. 🔴 Đọc trước: sim chứng minh được gì và KHÔNG chứng minh được gì

Toàn bộ bằng chứng của P0–P11 tới nay là **SITL**. Tỉ lệ chuyển giao sang đời thật **khác nhau cực lớn
theo tầng** — không có con số chung:

| Tầng | Chuyển giao | Nghĩa là |
|---|---|---|
| Logic điều khiển, máy trạng thái, giao thức | **~90–95%** | Cái sim bắt được thì đời thật cũng thế |
| Động lực học, tuning | trung bình | Phải hiệu chỉnh lại bằng số đo thật (P11.4) |
| **Định vị & thị giác** | **thấp** | 🔴 VIO/GPS/camera trong sim **không** đại diện cho đời thật |

> 📌 **Ba mục dưới đây là bản rút gọn.** Danh sách đầy đủ — 65 mục, 6 nhóm, kèm lý do và bằng chứng — ở [`sim-boundary-statement.md`](sim-boundary-statement.md) §2. Mỗi ô ⬜ ở §1 bên dưới phải trỏ được tới các mục B mà nó đóng.

**Ba giới hạn đã đo, ảnh hưởng thẳng tới chuyến bay đầu** (nợ #9):
- 🔴 **"VIO" trong sim là ground truth tuyệt đối** — không nhiễu, không trôi, không mất bám. Mọi kết
  luận về định vị dựa trên sim **không có hiệu lực** ngoài đời.
- 🔴 **Rangefinder KHÔNG vào EKF2 trong sim** (cầu `gz_bridge` v1.15 không nhận distance sensor) nhưng
  **CÓ trên drone thật** ⇒ khác biệt **kiến trúc**, phải kiểm riêng ở HITL.
- **Optical flow chưa từng được test** — plugin PX4 v1.15.4 có thư mục nhưng `CMakeLists.txt` rỗng.

---

## 1. ⛔ Cổng chặn cứng — chưa xong thì KHÔNG bay

Không mục nào dưới đây được bỏ qua bằng phán đoán. Mỗi mục là một cổng của P11 chưa chạy được vì
**thiếu phần cứng**, không phải vì chưa ai làm.

| # | Cổng | Vì sao chặn | Trạng thái |
|---|---|---|---|
| B1 | **P11.4 thrust benchmarking** → hiệu chỉnh SDF | Mọi số vật lý hiện tại là **số MƯỢN của x500**. `physics-parameters.md` §0: *"Không dùng bất kỳ số nào ở đây làm căn cứ cho bay thật."* Ước lượng biên điều khiển, thời gian đáp ứng, thời gian bay đều **sai** cho drone của ta | ⬜ chờ CAD + motor |
| B2 | **P11.3 driver camera thật** | Perception chưa từng thấy ảnh thật. `real.launch.py` để `perception:=false` đúng vì thế | ⬜ chờ camera |
| B3 | **P11.5 HITL + Jetson-in-the-loop + radio thật** | Benchmark chỉ trên workstation là **bẫy chí mạng** (overviewPlan §P11). Kiến trúc x86_64→arm64 khác về GPU/nhiệt/bộ nhớ | ⬜ chờ companion computer |
| B4 | **Profile DDS large-samples trên phần cứng thật** | Vách 549 408 B là mặc định **thư viện Fast DDS**, không của sim ⇒ camera thật nhiều khả năng cũng cần. Nhưng cổng R0 đo trên **loopback WSL**; máy thật đi qua giao diện vật lý (R31). `scripts/check_dds_profile_sim_only.sh` chặn nó lọt sang phía real | ⬜ chờ P11.3 |
| B5 | **Ba quyết định thừa kế trong `real.launch.py`** | Xem §3 | ⬜ chờ người ký |
| B6 | **Đo lại nhiễu `odometry_fused` trên chính máy bay, TRƯỚC khi tin bất kỳ hành vi planner/safety nào** | [`sim-boundary-statement.md`](sim-boundary-statement.md) **B-61**: trong sim, M5 chạy trên `uav0` — model **không có nguồn vision** — nên mux rơi về GPS và pose chính thức nhiễu \|dz\| **p50 37 cm, max 221 cm**. M5 chấm bằng `odometry_raw` nên đúng, **nhưng planner + safety + world model chỉ đọc `odometry_fused`** ⇒ mọi hành vi của chúng quan sát được trong sim là hành vi ở mức nhiễu ±1,3 m. Máy bay thật có nguồn vision khác hẳn | ⬜ chờ B2 + B3 |
| B7 | **Đo lại `mag_test_ratio` và số lần EKF2 reset quaternion tại BÃI BAY THẬT** | **B-62**: trong sim, EKF2 tự reset quaternion **giữa chuyến** (t=49,2 s) và `mag_test_ratio` **phi hữu hạn** đúng lúc đó ⇒ lần arm kế bị `COM_ARM_EKF_YAW` chặn, dù yaw chỉ lệch **1,67°**. Từ trường hiện trường khác hẳn Zurich mặc định (**B-48**). 🔴 **Cấm nới `COM_ARM_EKF_YAW` để cho qua** — quy trình phải chịu được một lần từ chối tạm thời, đúng như `armWithinBudget()` của M5 | ⬜ chờ bãi bay |
| B8 | **Đo HEADROOM CPU của companion computer khi stack chạy đủ tải, TRƯỚC khi tin bất kỳ kết quả action nào** | [`sim-boundary-statement.md`](sim-boundary-statement.md) **B-64**: dưới CPU bão hoà, `rclcpp_action` trả `UNKNOWN` + payload rỗng cho một goal **đã thực thi đúng**, và `uav_mission` dịch đúng luật thành **huỷ bước mission** (`kResultAbortedInternalError`). Tức máy thiếu headroom **tự huỷ việc nó vừa làm xong**. Đo được 0/10 lượt khi rảnh · 0/20 dưới 8 luồng · **6/30 dưới 16 luồng** trên workstation — ngưỡng đó **không chuyển sang máy bay** (R31). 🔴 **Cấm chữa bằng nới timeout hay thử lại goal** — cái phải sửa là tải, không phải phép chờ | ⬜ chờ B3 |

---

## 2. ✅ Đã kiểm được trong sim — chạy lại NGAY TRƯỚC chuyến bay

Chạy theo thứ tự. Mỗi dòng có lệnh; **không dòng nào được thay bằng "lần trước chạy rồi"**.

| # | Kiểm gì | Lệnh | Đạt khi |
|---|---|---|---|
| C1 | Toàn workspace build + test sạch | `bash scripts/verify_workspace.sh` | 12 pkg, **0 errors / 0 failures / 0 skipped**, chéo kiểm `Summary == case+target` |
| C2 | Ranh giới `px4_msgs` (R1) | `bash scripts/check_px4_msgs_boundary.sh` | `violations: 0  stale: 0` |
| C3 | Profile DDS vẫn là sim-only | `bash scripts/check_dds_profile_sim_only.sh` | `violations: 0  real-side: 0` |
| C4 | **Sim ↔ real cùng tập node (R7)** | `python3 scripts/check_sim_real_parity.py` | `parity PASSED` |
| C5 | `real.launch.py` **trơ** khi chưa duyệt | `bash scripts/verify_real_launch_inert.sh` | PASS — bare launch 0 node, `reviewed:=true` lên đủ node |
| C6 | **Stack đứng ngoài khi pilot cướp quyền** (P11.6) | `bash scripts/verify_pilot_override.sh` | 2 vòng: takeover đứng ngoài · control vẫn engage được |
| C7 | Nợ EKF2 EV-reset (G-E1) | `bash scripts/gate_e1_ev_reset.sh` | arm A ≤ 0,10 /s · arm B ≥ 0,50 /s |
| C8 | Bài bay hồi quy | `bash scripts/run_m5_regression.sh` | **PASS 3/3**, 0 vi phạm |
| C9 | Đèn go/no-go | 🔴 `bash scripts/preflight_light.sh` — **KHÔNG BAO GIỜ** `ros2 topic echo --once` | `GO`, `blocking=''`, và **tuổi mẫu** hợp lệ |

> 🔴 **C9 — vì sao cấm `topic echo`:** `/state/system_health` là **Reliable/KeepLast(1)/TransientLocal**,
> nên mẫu GO cuối cùng của một `diagnostics_node` **đã chết** vẫn nằm latched mãi mãi, và `echo` không có
> cách nào biết node đã chết. Chỉ `preflight_light.sh` mới tự kiểm **tuổi** mẫu (R32).

---

## 3. 🔴 Ba quyết định `real.launch.py` thừa kế — phải đọc trước khi truyền `reviewed:=true`

`real.launch.py` **không khởi động gì** nếu thiếu `reviewed:=true`. Đó không phải thủ tục: P6, P8 và P10
mỗi phase để lại một chỉ thị viết ra giấy gửi cho file này, và **không chỉ thị nào đóng được từ mô phỏng**.

| # | Quyết định | Trạng thái trong file | Cần gì để chốt |
|---|---|---|---|
| D1 | `require_obstacle_feed` | **TRUE** (sim chạy false) | ✅ Đã theo P6 Decision 4. Hệ quả: planner **từ chối lập kế hoạch khi không có nguồn vật cản** thay vì bay mù |
| D2 | `safety_enforcement` | **TRUE** | ⚠️ Lý lẽ, **không phải số đo**: INHIBIT được định nghĩa là *latch + im lặng có chủ đích → PX4 failsafe lõi + trả quyền pilot*, nên một **báo động giả** trao máy bay cho pilot, còn **tắt enforcement** nghĩa là mối nguy thật chỉ được *báo cáo* trong khi máy bay tiếp tục bay trên trạng thái không ai tin. Có pilot đứng cạnh ⇒ BẬT là chiều an toàn hơn. 🔴 **Chưa một INHIBIT nào từng kích hoạt trên phần cứng** |
| D3 | `blackbox` | **FALSE** | ⚠️ Theo đúng P10 D-4. **Đáng chất vấn trước khi bay:** một chuyến bay thật hỏng thì **không chạy lại được** — đó chính là lý lẽ dựng tầng bằng chứng; và O1 đã bảo đảm recorder **không thể** kéo máy bay xuống. Cái thật sự chưa biết là **chi phí CPU/đĩa trên companion computer chưa mua**. Đó là phép đo G-O2-on-target, không phải chuyện phán đoán từ đây |

---

## 4. Trình tự tại hiện trường

1. **Trước khi cấp nguồn:** chạy C1–C4 (tĩnh, không cần máy bay).
2. **Máy bay đã lắp, cánh quạt THÁO RA:** C5–C8 trên bàn.
3. **Lắp cánh quạt, máy bay ở nơi trống:** đọc đèn C9. Không GO ⇒ **dừng**, không "thử lại xem sao".
4. **Arm, chưa cất cánh:** đọc đèn C9 **LẦN NỮA** — bắt buộc theo hợp đồng §2.20. Trạng thái đổi giữa
   lúc kiểm trên bàn và lúc arm là chuyện bình thường, và đó chính là lúc phải bắt.
5. **Pilot cầm sẵn tay điều khiển, ngón đặt trên công tắc chế độ, suốt chuyến bay.**
6. Cất cánh thấp, hover, **chủ động thử cướp quyền bằng RC** ở độ cao an toàn — xác nhận stack đứng
   ngoài (đây là C6 nhưng trên sóng thật; sim không chứng minh được nửa vô tuyến).
7. Hạ cánh, disarm, đọc lại đèn.

---

## 5. Nếu có sự cố

| Tình huống | Làm gì |
|---|---|
| Đèn NO_GO bất kỳ lúc nào | Dừng. Đọc `worst_item`. **Không** sửa `preflight_waivers.yaml` để nó xanh |
| Stack im bất thường | Đó có thể là **INHIBIT có chủ đích** — PX4 failsafe lõi tiếp quản, pilot lấy quyền bằng RC |
| Sau chuyến bay | 🔴 Nếu tiến trình bị giết bất thường: **`ros2 bag reindex` TRƯỚC KHI đọc bag**, nếu không dữ liệu đọc ra sẽ thiếu |

---

## 6. Chữ ký

| Vai | Điều kiện ký | Ký |
|---|---|---|
| **Chủ dự án** | §1 không còn ô ⬜ · §2 chạy hết trong 24h trước · §3 đã đọc và chốt D2, D3 | ⬜ |
| **Reviewer** (`uav-design-rule-reviewer`) | Rà R0–R35 trên diff kể từ lần ký gần nhất | ⬜ |

*Không ký ⇒ không bay. Checklist này không tự cho phép bất cứ điều gì.*

---

### Phụ lục — nợ đã biết vẫn còn mở lúc viết

| Nợ | Ảnh hưởng chuyến bay đầu |
|---|---|
| **#2** `in_air` suy từ `takeoff_time > 0`, **sai sau khi hạ cánh** | P8 đã né bằng vị ngữ thay thế (`armed && OFFBOARD && command_fresh`); bất kỳ code mới nào dùng `in_air` phải kiểm lại |
| **#3** Cờ sức khoẻ cảm biến suy từ `failsafe_flags` (cờ của **bộ ước lượng**, không phải cảm biến thô) | `magnetometer_ok ← attitude_invalid` còn thô; đừng tin nó như một chẩn đoán cảm biến |
| **#12** `uav_navigation`: `escape_replan_interval_sec`/`escape_refresh_m` chưa có test làm chúng là thứ **duy nhất** chặn; nhánh `planner_fault_` trong `flyAround` chưa test | Vùng chưa phủ trong logic thoát hiểm |
| **D1/D2/D3 của P5** (cổng động perception) | Chưa đạt — chặn bởi RTF `uav0_full`, xem `plan/P5-perception.md` §3b |
| **G-O4(g)/(g2)** | Bằng chứng CỘNG THÊM cho luồng bay thật của đèn go/no-go |
