# Chạy AirSim + Unreal + PX4 + ROS2 — quy trình vận hành

> **Dùng khi:** muốn chạy nền mô phỏng AirSim tới đúng mức đã được chứng minh bởi **cổng G-A0**
> (2026-08-19): kết nối HIL · telemetry `/fmu/*` · arm được. **Chưa bay, chưa có camera/depth/`/clock`**
> — những thứ đó là **PA1/PA2**, xem `../.claude/plan/airsim-migration-proposal.md` §6.
>
> ⚠️ **R0:** đây là đường bay thật của tương lai. Không suy diễn từ "topic có publish" ra "hệ thống lành".

---

## 1. Ba cuộc hội thoại — hiểu cái này thì thứ tự khởi động tự nhiên đúng

| # | Ai với ai | Giao thức | Ai dựng |
|---|---|---|---|
| 1 | **AirSim → PX4** (cảm biến giả, actuator) | **MAVLink HIL**, TCP **4560** | AirSim lắng nghe · **PX4 gọi ra** |
| 2 | **PX4 → ROS2** (`/fmu/*`) | **uXRCE-DDS**, UDP **8888** | `MicroXRCEAgent` |
| 3 | **AirSim → ROS2** (ảnh, lidar, `/clock`) | **msgpack-RPC**, TCP **41451** → DDS | `airsim_node` — 🔴 **CHƯA dựng (PA1)** |

🔑 Cuộc 2 **y hệt** bản Gazebo ⇒ `uav_px4_backend` + `px4_msgs` không phải sửa gì. Đổi nền mô phỏng chỉ
đổi cuộc 1 và cuộc 3.

🔴 **Ba điều phản trực giác, mỗi điều đã tốn một phiên:**
1. **PX4 v1.15.4 là TCP *client*** — nó gọi sang Windows. ⇒ **Unreal phải lên TRƯỚC PX4.**
2. **Cổng 4560 nhận ĐÚNG MỘT kết nối** (`acceptTcp`) rồi thôi lắng nghe. ⇒ **cấm probe 4560**; kiểm sức
   khoẻ AirSim bằng RPC **41451**. Lỡ chạm ⇒ **phải khởi động lại Unreal**.
3. **`Documents` bị OneDrive chuyển hướng** (có hai thư mục cùng tên) ⇒ cấu hình truyền bằng cờ
   **`-settings=<file>`**, không ghi vào Documents.

---

## 2. Tài sản — 4 file, đã nằm trong repo

`config/airsim/`

| File | Vai trò |
|---|---|
| `gen_settings.py` | Sinh `airsim_settings_pa0.json` với **IP WSL hiện tại**. Chạy lại mỗi khi WSL reboot |
| `airsim_settings_pa0.json` | Cấu hình AirSim đọc: `PX4Multirotor`, TCP 4560, `LockStep:false`, gốc toạ độ Bách Khoa |
| `run_blocks.bat` | Phóng project Unreal `Blocks` ở chế độ game kèm `-settings=` |
| `ask_airsim.py` | Hỏi AirSim đang chạy xem nó **thật sự** đọc cấu hình nào (gỡ bẫy OneDrive) |

Phía WSL: [`../scripts/gate_ga0_airsim_hil.sh`](../scripts/gate_ga0_airsim_hil.sh) + [`../scripts/ga0_try_arm.py`](../scripts/ga0_try_arm.py).

**Ba tầng phía Windows** (đừng nhầm): engine `C:\Program Files\Epic Games\UE_5.5` · plugin
`C:\code\AirSim\Unreal\Plugins\AirSim` · **project** `C:\code\AirSim\Unreal\Environments\Blocks`
(đã build, plugin đã copy vào trong).

🔴 **Đừng đụng git trong `C:\code\AirSim`** — detached HEAD `cff11d4c` + **bản vá 3 dòng chưa commit**
(`GPULidarSimpleParams.hpp`, xem đề xuất §7b). Mất nó là mất khả năng build lại.

---

## 3. Flow chạy

### Bước 1 · Windows — sinh lại cấu hình

```powershell
cd C:\code\PX4_ROS2\config\airsim
python .\gen_settings.py
```

Đạt khi in ra 3 dòng IP, **không** có `FAIL`. Bỏ qua bước này sau khi WSL reboot ⇒ AirSim ôm IP cũ,
PX4 nối được nhưng đường điều khiển sai.

### Bước 2 · Windows — phóng Unreal

```powershell
.\run_blocks.bat
```

Cửa sổ **Blocks** 800×600 hiện ra, drone `uav0` đậu ở gốc. **Đợi ~15 s** (RPC sẵn sàng ~5 s, cổng HIL
~15 s). Cửa sổ PowerShell này bận cho tới khi đóng Blocks — để nguyên.

### Bước 3 · WSL — chạy cổng G-A0

```bash
cd ~/PX4_ROS2
bash scripts/gate_ga0_airsim_hil.sh
```

Script tự: kiểm RPC 41451 → dọn tiến trình cũ → bật agent `udp4:8888` → bật PX4 (`PX4_SYS_AUTOSTART=10016`,
`PX4_SIM_HOSTNAME` **suy từ `ip route`**) → chờ telemetry ≤90 s → đo nhịp → arm rồi disarm.

**Ngưỡng đạt** (số đo tham chiếu **2026-08-20**, sau khi sửa bẫy stdin — xem §6):

| Tiêu chí | Ngưỡng | Đo được lần PASS |
|---|---|---|
| a. AirSim nhận kết nối HIL | có | `Simulator connected on TCP port 4560` |
| b. nhịp `/fmu/out/vehicle_odometry` | ≥ 40 Hz | **109,2 Hz** |
| c. số topic `/fmu/*` | ≥ 40 | **44** |
| d. PX4 arm được | có | `arming_state 1→2→1` |
| e. log PX4 không phình | ≤ 50 MB | **0 MB** |
| — CPU của px4 (tham khảo) | — | **19,8 %** |

⚠️ **Số cũ 94,2 Hz (2026-08-19) đã bị thay** — nó đo khi PX4 đang đốt 136% CPU vì bẫy ở §6.

Chạy xong, agent + PX4 **vẫn sống ở nền** — dùng ROS2 ngay được. Log ở
`~/gate_logs/ga0_px4.log` (**không** để `/tmp`: WSL xoá sạch khi tự tắt vì idle).

### Bước 4 · WSL — dùng ROS2

```bash
source /opt/ros/humble/setup.bash
source ~/PX4_ROS2/install/setup.bash

ros2 topic list --no-daemon | grep -c '^/fmu/'        # ~44
ros2 topic hz /fmu/out/vehicle_odometry --window 200  # ~94 Hz
timeout 6 ros2 topic echo /fmu/out/vehicle_gps_position   # fix_type 3, Bach Khoa
timeout 6 ros2 topic echo /fmu/out/sensor_combined        # accel Z ~ -9.9 (FRD)
python3 ~/PX4_ROS2/scripts/ga0_try_arm.py                 # RESULT: ARMED
```

### Dừng

WSL `pkill -x px4; pkill -x MicroXRCEAgent` · Windows: đóng cửa sổ Blocks.
🔴 **Cấm gõ `pkill -f` trực tiếp** — nó khớp cả dòng lệnh của shell đang gõ và tự sát
(ops-playbook §1, đã dính 3 lần).

---

## 4. 🪤 Sự cố

| Triệu chứng | Nguyên nhân | Xử lý |
|---|---|---|
| `FAIL: AirSim RPC is not answering` | Unreal chưa lên xong | Đợi thêm ~10 s, chạy lại cổng |
| PX4 kẹt ở `Waiting for simulator...` | **Suất 4560 đã bị ăn** (PX4 cũ, hoặc lỡ probe) | **Đóng Blocks → `run_blocks.bat` lại** → mới chạy lại PX4 |
| `/fmu/*` = 0 topic dù PX4 sống | Agent sai bản, hoặc loopback WSL hỏng | Kiểm 30 giây bằng `talker`/`topic list --no-daemon` → ops-playbook §3 |
| `Preflight Fail: ekf2 missing data` | **Thoáng qua** lúc boot | Bỏ qua; đọc `vehicle_status` sau ~15 s (`pre_flight_checks_pass: true`). **Đọc trạng thái, đừng đọc lịch sử** |
| Nghi AirSim đọc nhầm cấu hình | Hai thư mục Documents | `python ask_airsim.py` — hỏi thẳng AirSim qua RPC |
| Mọi thứ chết sau khi WSL reboot | Cặp IP đã đổi | Bước 1 + bước 2 lại từ đầu |
| 🔴 **Bài bay TREO câm ngay ở `--- flight 1/1 ---`** | **Hai bản stack chồng nhau** — hai service server trùng tên `/uav/uav0/vehicle/arm` là hành vi không xác định trong ROS2 ⇒ lệnh arm không trả về | `pgrep -c -f 'uav_px4_backend/\|uav_localization/'` phải ra **đúng 11**. Ra 22 ⇒ giết sạch rồi khởi động **một** bản. Xem [`ops-playbook.md`](ops-playbook.md) §3 |

---

---

## 4b. ✅ Chạy STACK CỦA DỰ ÁN trên AirSim (2026-08-20) — và bay thật

Bước 3 mới chứng minh `/fmu/*`. Muốn chứng minh **framework của ta** thì chạy chính `sim.launch.py`,
chỉ đổi **một** tham số:

```bash
source /opt/ros/humble/setup.bash && source ~/PX4_ROS2/install/setup.bash
ros2 launch uav_bringup sim.launch.py use_sim_time:=false navigation:=false perception:=false
```

🔑 **`use_sim_time:=false` là ĐÚNG NGỮ NGHĨA, không phải lách.** Chính launch file mô tả tham số này là
*"The real drone sets this false"*, và AirSim chạy **thời gian thực** (`LockStep:false`) nên đồng hồ
tường mới là đồng hồ đúng. Đây cũng là cấu hình `real.launch.py` sẽ dùng ở P11.

Đo được: **10 node · 22 topic `/uav/uav0/*`** · `/state/vehicle` **9,999 Hz** ·
`/state/odometry_raw` **98,99 Hz** · `/state/health_px4` **9,999 Hz** · TF `odom → base_link` có phát ·
`odometry_raw` đọc ra vị trí thật (x 0,041 · y 0,036 · z −0,069 m).

### Bài bay M5 — arm → takeoff → goto → land

```bash
ros2 run uav_bringup smoke_flight.py --no-sim-time --flights 1
```

> 📌 **Dùng `ros2 run`, đừng gõ `python3 <đường/dẫn/dài>`.** Tài liệu cũ của dự án ghi
> `python3 install/uav_bringup/lib/uav_bringup/smoke_flight.py` là **tàn dư thời script này còn nằm
> trong `test/`**; từ khi `CMakeLists.txt` khai `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})`
> thì nó đã là executable hợp lệ của package. `ros2 run` không phụ thuộc đường dẫn workspace **và phơi
> mã thoát ra** (`[ros2run]: Process exited with failure 1`) — điều mà gọi `python3` trực tiếp giấu đi.

**Kết quả 3 chuyến đầu tiên trong lịch sử dự án trên nền AirSim:**

| Chuyến | alt_err (trần 0,30) | horiz_err (trần 0,50) | hạ cánh | Kết quả |
|---|---|---|---|---|
| 1 | 0,06 m | 0,35 m | 7 s tự tắt động cơ | 🔴 **FAIL** — `failsafe activated` |
| 2 | 0,24 m | 0,44 m | 7 s tự tắt động cơ | ✅ **PASS** — violations none |
| 3 | 0,11 m | 0,42 m | 6 s tự tắt động cơ | 🔴 **FAIL** — `failsafe activated` |

⇒ **Drone BAY ĐƯỢC bằng stack không sửa một dòng**, và **sai số nằm trong ngưỡng ở CẢ BA chuyến**.
Nhưng chỉ **1/3 PASS** — hỏng nằm ở failsafe, không nằm ở độ chính xác. **Flaky nguy hơn hỏng hẳn** vì
nó lọt cổng.

### 🔴 Nguyên nhân failsafe — đã bắt được tên, không phải đoán

Bắt `/fmu/out/failsafe_flags` suốt một chuyến (73 mẫu):

| Cờ | Số mẫu bật | Đọc thế nào |
|---|---|---|
| `manual_control_signal_lost` | 73/73 | Bình thường — SITL không có RC |
| `auto_mission_missing` | 73/73 | Bình thường — không nạp mission |
| 🔴 **`offboard_control_signal_lost`** | **28/73 = 38 %** | **Đây là thủ phạm** |

PX4 coi luồng offboard của ta **đứt 38% thời gian**. Log PX4 khớp: `Failsafe activated` ×4 ngay sau
`Takeoff detected`, kèm `flight_mode_manager: Matching flight task was not able to run, Nav state: 2`.

🟠 **Nghi phạm hàng đầu (CHƯA chứng minh):** chính cái mất đồng bộ thời gian ở §6b — mỗi ~13 giây
`lockstep_scheduler` đặt lại gốc thời gian, đủ để phép tính *"lần cuối nhận offboard cách đây bao lâu"*
vọt qua ngưỡng 0,5 s. Muốn chứng minh phải **đối chiếu dấu thời gian** của `time jump` với các mẫu
`offboard_control_signal_lost` — chưa làm.

⚠️ **Hệ quả: cổng G-A2 (M5 PASS 3/3 trên AirSim) CHƯA đạt** — mới 1 PASS / 1 FAIL. Đừng ghi là đã đạt.

---

## 5. 🔴 Giới hạn — nói thẳng

- **Chưa có `/clock`, chưa có camera/depth/lidar** — cuộc hội thoại 3 chưa dựng (PA1). Vì thế phải chạy
  `use_sim_time:=false` và `perception:=false`; mọi node thị giác/định vị-bằng-vision đều câm.
- **Luồng offboard đứt 38%** (§4b) — chưa dùng nền này để kết luận bất kỳ điều gì về timing.
- **Chưa đo RTF dưới tải render** của Unreal.
- **`airsim_node` chưa từng build** trong dự án này (thuộc quyết định Đ5 đang chờ chốt).
- Khi làm PA1 đã biết trước một chỗ lệch: `airsim_ros_wrapper.cpp:1656` trả **`sensor_msgs/Range`**,
  còn `rangefinder_adapter_node` của ta ăn **`LaserScan`** ⇒ phải viết adapter.
- 🔴 **Không dùng service `Takeoff`/`Land` và topic `VelCmd` của `airsim_node`** — chúng lái drone bằng
  API nội bộ AirSim, **đi vòng qua PX4** và mọi lá chắn an toàn của ta, lại **không tồn tại trên drone
  thật** (vi phạm R7). Giữ `enable_api_control=false`. AirSim chỉ được đóng vai **nguồn cảm biến**.

---

## 6. 🔴 Bẫy đã sửa 2026-08-20 — và cái nó để lộ ra

### 6a. Cổng tự làm bẩn phép đo của chính nó

Script cổng khởi động PX4 với `< /dev/null`. Shell `pxh` gặp EOF rồi **vẽ lại vô hạn** — đúng bẫy
[`ops-playbook.md`](ops-playbook.md) §1 mà chính dự án đã ghi từ trước. Đo được trước khi sửa:

| | Trước (`< /dev/null`) | Sau (`px4 -d`) |
|---|---|---|
| log PX4 | **12 GB** | **5,5 kB** |
| CPU của px4 | **136 %** (687 s CPU / 502 s tường) | **19,5 %** |
| khe hở lớn nhất giữa 2 mẫu odometry | **2,124 s** | **0,018 s** |
| nhịp odometry | 94,2 → 104–108 Hz (trôi) | **109,0 Hz** (ổn định 3 cửa sổ) |

**Cách chữa:** cờ **`-d`** của PX4 (*"daemon mode, don't start pxh shell"*) — sạch hơn
`sleep infinity |` vì nó **xoá hẳn** shell thay vì cho shell một stdin chặn. Đổi lại: không gõ được
lệnh vào `pxh>`; muốn tương tác thì chạy PX4 foreground **không** kèm `-d` (§3 bước 4 kiểu thủ công).

🔑 **Cổng nay tự đo cái tật đó** (tiêu chí **e**, trần 50 MB) — tái phát là cổng cắn, không im lặng
trả về số bẩn. Cùng học thuyết §11: *cách đo làm hỏng thứ được đo thì con số thu được là của phép đo*.

### 6b. 🟠 Thứ 12 GB rác đang che: PX4 mất đồng bộ thời gian mỗi ~13 giây

Log sạch rồi mới đọc được:

```
INFO  [lockstep_scheduler] setting initial absolute time to 1787192810316636 us
...
WARN  [timesync] time jump detected. Resetting time synchroniser.
WARN  [uxrce_dds_client] time sync no longer converged
INFO  [uxrce_dds_client] time sync converged
```

**Đo được: 11 lần `time jump` trong 146 s ⇒ trung bình 1 lần / 13 giây**, lặp đều cả trước lẫn sau khi
arm. Nghi phạm: PX4 SITL vẫn nạp **`lockstep_scheduler`** (lấy thời gian từ dấu thời gian `HIL_SENSOR`)
trong khi `settings.json` để **`LockStep: false`** ⇒ nguồn thời gian không được ai neo.

🔴 **Vì sao phải quan tâm (R0):** mỗi lần "no longer converged" là **dấu thời gian trên `/fmu/out/*` bị
đặt lại gốc**. Nợ #4 của dự án (`px4::Px4ClockOffset` học đồng hồ PX4) sẽ phải đuổi theo một mốc nhảy
mỗi 13 giây, và mọi `/fmu/in/*` mà PX4 xét theo dấu thời gian đều chịu ảnh hưởng.

⇒ **Đây là bằng chứng cho quyết định Đ3 (lockstep hay thời gian thực) đang chờ chốt.** Chưa xử lý —
ghi lại để quyết trên số liệu, không quyết bằng cảm giác.
