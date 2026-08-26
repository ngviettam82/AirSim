# `uav_sim_gz`

Tài sản mô phỏng Gazebo: world, model drone, mô-đun cảm biến, và cầu nối Gazebo → ROS2.

> Package này thuộc **lớp thế giới mô phỏng**, không phải lớp phần mềm tự hành. Trên drone thật, phần tương đương là **phần cứng và driver cảm biến** — xem [`../uav_bringup/README.md`](../uav_bringup/README.md) để hiểu ranh giới hai lớp.

---

## 1. Thang model — thiết kế để kế thừa, không copy

Bốn tầng, mỗi tầng làm đúng một việc. Thêm model mới = viết thêm một file lắp ráp ~20 dòng, **không** nhân bản model có sẵn.

```
uav0_frame          ← THÂN. Điểm thay CAD của team phần cứng.
   │                  Hiện kế thừa x500 của PX4.
   ├── sensor_lidar_down     ┐
   ├── sensor_camera_down    ├── mô-đun cảm biến, dùng lại được cho mọi thân
   ├── sensor_camera_front   │
   └── sensor_depth_front    ┘   (tất cả đều của dự án)
        │
        ├── uav0        = frame
        ├── uav0_nav    = frame + lidar dưới + camera dưới + odometry
        └── uav0_full   = uav0_nav + camera RGB/depth trước
```

| Model | Airframe | Dùng khi | Cảm biến render |
|---|---|---|---|
| `uav0` | `4100_gz_uav0` | Hồi quy động lực bay, lặp nhanh | 0 |
| `uav0_nav` | `4102_gz_uav0_nav` | **P4 định vị** | 2 |
| `uav0_full` | `4101_gz_uav0_full` | **P5 perception**, P6 tránh vật cản | 4 |

### Khi team phần cứng giao CAD

Sửa **đúng một file**: [`models/uav0_frame/model.sdf`](models/uav0_frame/model.sdf). Thay `<include>` x500 bằng thân thật (mesh + `<inertial>` + 4 rotor + `MulticopterMotorModel`). Ba biến thể và toàn bộ mô-đun cảm biến **không phải sửa gì** — vì chúng neo vào `base_link`, không neo vào gốc toạ độ model (xem §5, bẫy 4).

Cần sửa thêm khi thay thân: `airframes/4100_gz_uav0` (số `CA_ROTOR*` kế thừa từ x500 sẽ không còn đúng).

---

## 2. Chạy

```bash
UAV_MODEL=uav0_nav bash scripts/start_sim.sh
```

Script khởi động **cả cầu Gazebo→ROS**, vì cầu đó thuộc lớp thế giới mô phỏng và là nguồn duy nhất của `/clock` — thứ mọi node tự hành cần trước khi chạy. Muốn bật riêng:

```bash
ros2 launch uav_sim_gz gz_bridge.launch.py model:=uav0_nav
```

Thêm `gui` vào lệnh đầu nếu muốn thấy Gazebo. Mặc định `UAV_MODEL` là `uav0` (nhẹ nhất, giữ nguyên đường cơ sở đã chứng minh ở M5) — biến thể này vẫn có `bridge_uav0.yaml` chỉ để phát `/clock`.

### Cài đặt lần đầu / sau khi thêm airframe

```bash
ros2 run uav_sim_gz install_to_px4.sh
```

Script tạo symlink airframe + world vào cây PX4 và khai báo airframe vào `CMakeLists.txt` của ROMFS. Symlink trỏ về thư mục nguồn nên sửa file trong package là PX4 thấy ngay. **Chỉ khi thêm airframe mới** mới cần `make px4_sitl` lại — script tự báo.

Model **không** cần cài: Gazebo tìm qua `GZ_SIM_RESOURCE_PATH`, mà workspace tự set qua env hook.

---

## 3. Topic cầu nối ra ROS2

Tên topic phía ROS **đặt đúng như driver thật sẽ phát**, để node phía sau không phân biệt được sim hay thật (R7).

| ROS topic | Nguồn Gazebo | Ai dùng |
|---|---|---|
| `/clock` | `/world/uav_arena/clock` | Toàn hệ (`use_sim_time`) |
| `/uav/uav0/localization/range_scan_raw` | `/uav0/lidar_down` | `rangefinder_adapter_node` (P4) |
| `/uav/uav0/localization/vio_odometry_raw` | `/model/<model>_0/odometry` | `vio_adapter_node` (P4) |
| `/uav/uav0/perception/down/image_raw` | `/uav0/camera_down/image` | marker detect, optical flow |
| `/uav/uav0/perception/down/camera_info` | `/uav0/camera_down/camera_info` | — |
| `/uav/uav0/perception/front/image_raw` | `/uav0/camera_front/image` | object detect (P5) |
| `/uav/uav0/perception/front/camera_info` | `/uav0/camera_front/camera_info` | — |
| `/uav/uav0/perception/front/depth_image` | `/uav0/depth_front/image` | obstacle extract (P5) |
| ~~`/uav/uav0/perception/front/points`~~ | — | 🔴 **Cố ý KHÔNG bắc cầu.** Cloud chở lại đúng thông tin của ảnh depth mà **giành mất phần băng thông của nó** (depth tụt còn 18% số khung; bỏ cloud → 42%). P5 **chiếu ngược trong ROS** từ `depth_image` + `camera_info` — cũng là cách drone thật làm. Số đo: [`../../docs/package-status.md`](../../docs/package-status.md) §4 |

Ảnh đi qua `ros_gz_image`, phần còn lại qua `ros_gz_bridge`. Lý do ở §5 bẫy 5.

> ⚠️ Tên `*_raw` là **tạm**, chờ P4 chốt. P4 sở hữu các adapter này nên P4 có quyền đổi.

### Tần số đo được (uav0_nav, headless, RTF 1.0)

| Topic | Đo được | Cấu hình |
|---|---|---|
| `/clock` | 243–250 Hz | 250 Hz (world) |
| `range_scan_raw` | 19.9 Hz | 20 Hz |
| `down/camera_info` | 30.4 Hz | 30 Hz |
| `down/image_raw` | 13–18 Hz | 30 Hz — xem bẫy 5 |
| `vio_odometry_raw` | 50.0 Hz | — |

---

## 3b. Ba world — chọn theo việc cần làm

| World | Dùng khi | Có gì |
|---|---|---|
| `uav_arena` | **Hồi quy M5** | Nền trơn, không texture. Giữ nguyên có chủ đích: chuyến bay trượt thì đổ lỗi được cho **code**, không phải cho nội dung world |
| `uav_arena_outdoor` | P4 optical flow, P5, P6 | Nền có kết cấu · **cầu thang địa hình** 0.25/0.50/1.00 m tại y=5 · bệ đáp · 2 cột |
| `uav_arena_indoor` | P4 kịch bản không GPS | Phòng 12×12×4 m, đèn trần · **GPS mất fix** · thùng hàng, kệ |

World tự động theo model: model tên `*_indoor` → `uav_arena_indoor`, còn lại → `uav_arena`. Ghi đè bằng `UAV_WORLD`.

### `uav_arena_vn` — địa điểm thật ở Việt Nam, **sinh bằng công cụ**

```bash
python3 tools/fetch_osm_features.py --radius 400 --out /tmp/hcmut.json   # cần mạng, có cache
python3 tools/vn_world_gen.py --features /tmp/hcmut.json --radius 200

# chỗ khác, chỉ cần đổi toạ độ
python3 tools/fetch_osm_features.py --latitude 10.88 --longitude 106.80 --out /tmp/td.json
python3 tools/vn_world_gen.py --features /tmp/td.json --name uav_arena_thuduc
```

### Mở xem

```bash
cd ~/PX4_ROS2 && source install/setup.bash
unset LIBGL_ALWAYS_SOFTWARE                        # BẮT BUỘC, xem cảnh báo dưới
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
gz sim -r install/uav_sim_gz/share/uav_sim_gz/worlds/uav_arena_vn.sdf
```

Muốn có cả drone và PX4: `UAV_MODEL=uav0_nav UAV_WORLD=uav_arena_vn bash scripts/start_sim.sh gui`

🔴 **`~/.bashrc:119` đang đặt `LIBGL_ALWAYS_SOFTWARE=1`** → mọi terminal tương tác render bằng **phần mềm**, RTF tụt còn **0.014** (chậm 70×). `start_sim.sh` tự gỡ; chạy `gz sim` tay thì **phải tự `unset`**.

**Vì sao là bộ sinh chứ không phải một world khổng lồ:** Gazebo world là **cảnh vật lý tính theo mét** — mọi thứ trong đó đều tốn va chạm và render. Dựng cả một vùng lớn để bay vài chục mét là trả giá RTF cho thứ không ai nhìn tới. Bộ sinh phủ được cả nước; mỗi lần chạy chỉ nạp đúng khu đang bay.

Sinh ra ba thứ:

| Sản phẩm | Dùng làm gì |
|---|---|
| `worlds/<tên>.sdf` | World ở **toạ độ thật** |
| `models/<tên>_scene/` | Toàn cảnh: nhà, đường, mảng nền, hàng rào, cây |
| `docs/<tên>_ground_truth.json` | **Footprint + kích thước + vị trí từng vật** — thứ để chấm điểm perception |

**Các lớp và ai có va chạm:**

| Lớp | Nguồn | Collision |
|---|---|---|
| Toà nhà | OSM `building`, **đùn từ footprint thật** | ✅ |
| Hàng rào / tường | OSM `barrier` | ✅ |
| Cây | **tổng hợp**, rải dọc đường | ✅ (thân + tán) |
| Đường, lối đi | OSM `highway` | ❌ mặt phẳng, nền đã có |
| Mảng nền (cỏ, sân, nước, bãi xe) | OSM `landuse`/`leisure`/`natural` | ❌ như trên |

**Ba tính chất quan trọng hơn vẻ ngoài:**

1. **Lidar và vật lý phải nói cùng một chuyện.** Rangefinder của ta là `gpu_lidar` → nó bắn tia vào **`<visual>`**, còn va chạm ăn **`<collision>`**. Thả mesh ảnh-thật làm visual mà giữ collision phẳng thì **drone bay xuyên toà nhà trong khi lidar vẫn báo có vật cản** — không lỗi nào được in ra. Nên vật đặc thì có **cả hai**, vật vẽ trên nền thì **không có cái nào**.
2. **Cảnh không có nhãn thì không chấm điểm được.** Manifest ground-truth bù vào chỗ đó.
3. **Cây là đồ tổng hợp và manifest nói rõ.** OSM Việt Nam gần như không map cây riêng lẻ, mà một khuôn viên trơ nhựa đường thì không phải nơi ta định mô phỏng. Không được nhầm chúng với dữ liệu khảo sát.

🔑 **`<polyline>` của SDF chạy đầy đủ trong Harmonic — đã đo, không đoán.** Thả cầu lên nóc nhà chữ L: dừng đúng **z = 12.500000** (nóc 12 + bán kính 0.5). Thả cầu đối chứng vào **khuyết** chữ L: rơi xuống **z = −1232** → khuyết là **rỗng thật**, không bị lấp. Lidar từ z=30 đọc **17.04 m**, khớp. Nhờ vậy toà nhà dùng **footprint thật**, không phải hộp bao.

Phép chiếu dùng **đúng công thức azimuthal equidistant của PX4** (bán kính 6371 km), giống hệt `uav_localization/geodetic_projection.cpp` — nên một mét trong world bằng một mét của bộ ước lượng.

**Dữ liệu thật đã nạp** (Overpass, 400 m quanh datum): **74 toà nhà · 170 tuyến đường · 6 sân thể thao · công viên · mặt nước · hàng rào**. 29 nhà có tên — `Nhà A4/A5`, `B1/B3/B4/B6`, `C1/C4/C5/C6`, `Căn tin`, `Nhà thi đấu Phú Thọ`. Chính danh sách tên này **xác minh ngược lại toạ độ** là đúng khuôn viên.

**World hiện tại:** 18 toà nhà · 6 mảng nền · 66 tuyến · 193 cây tổng hợp — **385 link**.

**Đo thật khi bay:** PX4 GPS `lat=10.772901 lon=106.657701 alt=9.5 m fix=3 sats=10`; `/state/gnss_fix` của ta ra **cùng con số**; **RTF 0.982**; hồi quy **PASS 3/3**; Gazebo không báo lỗi nào.

**Nội dung gần như không tốn gì** — đo cùng điều kiện, cùng bộ node:

| World | RTF |
|---|---|
| `uav_arena` (nền trơn, ít nội dung nhất) | 0.887 |
| `uav_arena_outdoor` | 1.000 |
| **`uav_arena_vn` (385 link)** | **0.982** |

⚠️ World **ít nội dung nhất lại chậm nhất** → RTF đo một lần **dao động cỡ 0.1**, đừng kết luận từ một số đo. (Lần đầu tôi đo ra 0.869 và suýt đổ tội cho toà nhà.)

🔑 **Vì sao không dùng hộp bao nữa.** Khuôn viên nằm chéo so với hướng bắc. Đo thật: hộp **bao trục** phình **trung vị 1.91×, tệ nhất 4.54×** (`Nhà B1` thật 2095 m² thành hộp 95×100 m, nuốt cả sân trống lẫn gốc toạ độ). Hộp **bao có xoay** kéo về trung vị 1.00× nhưng nhà chữ L vẫn 2.68×. **Footprint thật bằng polyline** xoá hẳn vấn đề — bằng chứng: `Nhà B3` từng bị loại khỏi bãi cất cánh chỉ vì *hộp bao* của nó phủ gốc toạ độ; với footprint thật thì nó **quay lại world**.

🔑 **Đặt đúng toạ độ không phải chuyện trang trí.** PX4 tự tra từ trường theo lat/lon trong `SensorMagSim` — bay world Zurich rồi ra sân Việt Nam là **hai bài toán heading khác nhau**. Đổi toạ độ là đóng nợ này.
*(Giá trị từ trường suy ra từ mã nguồn + GPS đã đo, **chưa quan sát trực tiếp**: model không có magnetometer trong Gazebo và cầu DDS không phơi topic đó.)*

⚠️ **Giới hạn hiện tại:** chưa có cột điện, xe, người. Nền dùng texture **sinh bằng code**, chưa phải ảnh trực giao. Mái nhà phẳng, không có chi tiết mặt tiền. Đường là mặt phẳng ghép từng đoạn nên có khe nhỏ ở chỗ rẽ.
⚠️ **Datum nằm trong khu đã xây**, nên **2 toà nhà bị gỡ** để chừa bãi cất cánh 18 m (`--clear-radius`); có liệt kê trong manifest, không xoá lặng lẽ.
⚠️ `gz sdf -k` báo không tìm thấy `model://..._buildings` khi kiểm rời — đó là do trình kiểm không có đường tìm tài nguyên, **không phải lỗi world**; sim nạp bình thường.

### Bay trong nhà cần tắt ba thứ, và cả ba đều đo ra chứ không đoán (2026-08-10)

Airframe `4112_gz_uav0_nav_indoor` phải khai thêm ba tham số. Mỗi cái tương ứng một lần preflight từ chối arm:

| Tham số | Vì sao |
|---|---|
| `SYS_HAS_MAG 0` | 🔑 Đọc `SensorMagSim.cpp`: la bàn mô phỏng **chỉ phát khi đã có `vehicle_global_position` với `eph<1000`**. Không GPS ⇒ **không có la bàn nào cả**. Cũng là cấu hình thật của drone bay trong nhà: từ trường trong nhà vô dụng, yaw lấy từ vision. |
| `EKF2_BARO_CTRL 0` | Đo được `cs_baro_hgt=True` — **ghi chú cũ nói baro đã tắt là SAI**. Nó lệch gốc so với frame vision nên hai nguồn độ cao đánh nhau ⇒ "height estimate error". |
| `EKF2_GPS_CTRL 0` | Bộ kiểm tra vẫn chặn arm bằng "GPS fix too low" dù vision tốt. Máy thu vẫn chạy (`SIM_GPS_USED 0`), chỉ là bộ ước lượng thôi chờ nó. |

✅ **Bay được: PASS 3/3.** Nhưng ba tham số trên **chưa đủ** — thứ khoá chặt là một deadlock trong `offboard_session_manager_node`: nó chờ armed mới vào offboard, còn PX4 khởi động vào **LOITER** mà LOITER đòi **global position** (đo được `mode_req_global_position=56`). Phải **vào offboard trước, arm sau**. Xem `docs/package-status.md` mục `uav_px4_backend`.

⚠️ **Bài bay lặp trong phòng 12×12 m phải đi con thoi giữa hai điểm.** Bản cũ lấy vị trí hiện tại làm gốc rồi đi thêm 3 m mỗi chuyến → 3 chuyến là 9 m, xuyên tường.

**Ba mức kết cấu nền**, đo được chứ không phỏng đoán (gradient trung bình nhìn từ z=2 m):

| Mesh | Gradient | Dùng để |
|---|---|---|
| `ground_rich.obj` | **12.11** | trường hợp chạy tốt |
| `ground_sparse.obj` | **4.72** | trường hợp suy giảm |
| `ground_flat.obj` | **0.19** | đối chứng — flow phải ≈ 0 |

Đổi mức bằng cách sửa `<uri>` trong world. Texture **sinh lúc build** bằng `scripts/generate_ground_texture.py`, không commit ảnh nhị phân, không tải từ đâu.

> ⚠️ `<plane>` của SDF trải texture **một lần trên toàn mặt** (100 m / 1024 px = 10 cm/pixel, quá thô để bám). Vì vậy nền là **mesh có UV lặp** — 2 m mỗi tile, 1.95 mm/pixel.

## 4. World `uav_arena`

Kế thừa **nguyên khối** `Tools/simulation/gz/worlds/default.sdf` của PX4 v1.15.4: `physics type="ode"`, `max_step_size` 0.004 (250 Hz), `gravity 0 0 -9.8`, đủ bộ plugin, `spherical_coordinates` ENU. Không tự chế lại thứ PX4 đã tinh chỉnh.

Phần thêm của dự án, đặt **ngoài hành lang bay của bài hồi quy** (x 0→3, y≈0, z 0→2.5):

| Vật thể | Vị trí | Để làm gì |
|---|---|---|
| `landing_pad` | (3, 0), 1×1×0.01 m | Đích của bài bay; nền cho hạ cánh theo marker (P5) |
| `obstacle_pillar_north` | (8, 2), 0.4×0.4×2 m | Tránh vật cản (P6) |
| `obstacle_pillar_south` | (8, −2) | — |

---

## 5. Bẫy đã gặp và đã trả giá

Mỗi mục dưới đây là một lỗi thật đã tốn thời gian chẩn đoán. Ghi lại để không ai phải trả giá lần hai.

**1. PX4 chỉ thử spawn MỘT lần trong 1 giây.** Trong `GZBridge.cpp`, nếu không bật `PX4_GZ_STANDALONE=1` thì PX4 gọi service tạo model đúng một lần, timeout 1 s, thất bại là chết hẳn. Model có cảm biến render khởi động lâu hơn 1 s → luôn chết. `x500` trần không có camera nên kịp, vì thế bẫy này ẩn suốt M2–M5. → `start_sim.sh` **tự khởi động Gazebo trước**, đợi `/world/<w>/clock`, rồi mới bật PX4 ở chế độ standalone (nhánh này PX4 retry đến khi được).

**2. Cảm biến render làm Gazebo SEGFAULT nếu không ép GPU.** Đo thật trên máy Intel Arc iGPU + RTX dGPU:

| Cấu hình | Kết quả | RTF |
|---|---|---|
| env mặc định | 🔴 segfault trong `Ogre2RenderTarget::Copy` | — |
| `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` | ✅ chạy | **1.000** |
| `LIBGL_ALWAYS_SOFTWARE=1` | chạy | **0.014** (chậm 70×) |

`start_sim.sh` export biến NVIDIA và **`unset LIBGL_ALWAYS_SOFTWARE`** ở cả hai nhánh headless/gui. ⚠️ `~/.bashrc:119` đang bật `LIBGL_ALWAYS_SOFTWARE=1` — script phải chủ động gỡ, không được tin vào môi trường.

**3. Model không có collision sẽ rơi xuyên mặt đất.** Một model probe chỉ có `<inertial>` sẽ rơi tự do vô hạn. Sau vài giây nó ở hàng trăm mét dưới đất và **mọi cảm biến trả về `inf` / ảnh trắng** — triệu chứng giống hệt "render hỏng". Model dùng để đo phải `<static>true</static>` hoặc có collision.

**4. `<include><pose>` tính theo GỐC MODEL, không theo `base_link`.** Hai cái này lệch nhau đúng bằng chiều cao càng đáp — với x500 là **0.24 m**. Đặt `<pose>0 0 -0.05</pose>` khiến cảm biến nằm **dưới mặt đất 5 cm** trong khi vẫn phát topic đều đặn. `relative_to` trong `<joint>` **không** dời link con. → Luôn dùng `<pose relative_to="base_link">`. Đây cũng là thứ giữ cho mô-đun cảm biến còn đúng khi thay thân bằng CAD thật. *(Con số `.242` trong `x500_depth` của PX4 chính là toạ độ gốc-model, không phải offset so với thân.)*

**5. `parameter_bridge` là nút thắt với ảnh.** Gazebo phát đủ 30 Hz nhưng ROS chỉ nhận 5.8 Hz. Chuyển ảnh sang `ros_gz_image` → 13–18 Hz. Phần chênh còn lại một phần do chính `ros2 topic hz` (subscriber Python) làm rớt: `camera_info` cùng cảm biến vẫn về đủ 30.4 Hz. Consumer C++ ở P5 nhiều khả năng nhận cao hơn con số này.

**6. Tên topic `camera_info` do Gazebo tự suy ra bằng cách THAY đoạn cuối của topic ảnh**, không phải nối thêm. `<topic>uav0/camera_down</topic>` → camera_info ở `/uav0/camera_info`, nên hai camera cùng namespace sẽ **đè lên nhau**. → Đặt topic ảnh có nhánh riêng: `uav0/camera_down/image`.

**7. PX4 đặt tên thực thể là `<model>_<instance>`.** Model `uav0_nav` spawn ra thành `uav0_nav_0`, nên topic là `/model/uav0_nav_0/odometry`. File bridge phải khớp hậu tố này.

**8. `/tmp/px4.log` phình 4.5 triệu dòng trong 35 giây** khi stdin là `/dev/null` — shell PX4 gặp EOF rồi vẽ lại prompt liên tục. → `start_sim.sh` chạy PX4 với stdin chặn (`sleep infinity | make ...`). Lỗi này có từ trước, chỉ là chưa ai để ý.

**9. numpy của pip (2.x, ở `~/.local`) đè numpy của apt (1.x)** mà `apt`'s `python3-opencv` được build cùng, nên `import cv2` vỡ ở bất kỳ script nào chạm OpenCV (`generate_aruco_marker.py`, `inspect_marker_frame.py`...). → Chạy với `PYTHONNOUSERSITE=1` (đã set trong `CMakeLists.txt` cho bước build); script nào không cần OpenCV thì né hẳn (`grab_frame.py` dùng PIL thay `cv2.imwrite`).

**10. `gz-sim-velocity-control-system` + contact tiếp đất = lật vật.** Đo thật 2026-08-22 khi dựng `target_box` (P9.5): hộp 0.5×0.5×0.8 m đặt sát đất, ra lệnh `cmd_vel` linear.x=0.2 m/s → pitch leo dần **0.07 → 1.50 rad trong 7 s rồi lật hẳn**. Cơ chế: plugin ép lại vận tốc CoM mỗi bước, nhưng contact solver (ma sát + phản lực pháp tuyến tại đáy) vẫn sinh mô-men mỗi bước vì trọng tâm cao hơn điểm tiếp xúc — mô-men này cộng dồn vào ORIENTATION dù VẬN TỐC bị ép về đúng lệnh ngay bước sau. Đối chứng: cùng model thả **cách mặt đất 5 m** (không chạm gì) → 6 mẫu liên tiếp **RPY giữ đúng 0.000000**, tịnh tiến sạch tuyệt đối. → **Fix: `<gravity>false</gravity>` trên link + chừa hở mặt đất ~0.05 m** (không chạm collision nào) thay vì đặt khít z=0. Không dùng được cách giảm ma sát (`mu=0`) vì vẫn còn phản lực pháp tuyến gây nhiễu nhỏ khi số học dao động quanh z=0.

---

## 6. Giới hạn phải biết trước khi tin vào mô phỏng

| Hạng mục | Thực tế |
|---|---|
| **Rangefinder KHÔNG vào EKF2 của PX4** | `gz_bridge` của PX4 v1.15 chỉ nhận clock, pose, IMU, odometry, airspeed, baro, navsat. **Không có** distance sensor. Rangefinder của ta chỉ tồn tại phía ROS. Trên drone thật, rangefinder thường nối thẳng vào FC và nuôi EKF2 → **đây là khác biệt sim/real có thật**. |
| **"VIO" là ground truth** | `gz-sim-odometry-publisher-system` cho vị trí **chính xác tuyệt đối**, không nhiễu, không trôi, không mất bám. Thuật toán nào ăn nguồn này cũng sẽ trông giỏi hơn thực tế. Muốn đánh giá thật thì phải tự thêm nhiễu/độ trễ. |
| **Optical flow chưa có** | PX4 v1.15.4 có thư mục `gz_plugins/optical_flow` nhưng `CMakeLists.txt` **rỗng** — plugin chưa được tích hợp ở bản này. `optical_flow_adapter_node` (P4) chưa có nguồn dữ liệu. |
| **Số của cảm biến là placeholder** | Khối lượng, quán tính, nhiễu, tầm đo đều **chưa có nguồn thiết bị thật** — xem [`docs/model-sources.md`](docs/model-sources.md). |
| **Thân drone là x500, không phải drone của dự án** | Mọi kết luận về lực đẩy, quán tính, thời gian đáp ứng chỉ đúng với x500. |

---

## 7. Kiểm chứng đã chạy (R14)

| Hạng mục | Kết quả |
|---|---|
| Bài bay hồi quy M5, model `uav0` | **PASS 3/3** — sai số cao độ 0.01–0.19 m, ngang 0.03–0.06 m |
| Bài bay hồi quy M5, model `uav0_nav` (có cảm biến) | **PASS 3/3** — cao độ 0.10–0.13 m, ngang 0.14–0.16 m |
| RTF — ⚠️ **số cũ 2026-08-10, đã bị thay** | ~~1.000 cho cả ba biến thể~~ → xem hàng dưới |
| RTF đo lại 2026-08-13 (`scripts/measure_rtf_models.sh` · `scripts/sample_rtf.sh`, RTX xác nhận) | Lúc **đậu**, đọc **mean**: `uav0` **1.000** · `uav0_nav` **0.999** (2 lần) · `uav0_full` **0.79–0.86** sau khi thay OakD-Lite bằng mô-đun của dự án (trước: 0.50–0.67). Lúc **bay dưới stack đầy đủ** (`uav0_nav`, 11 node): **0.89–0.95**. 🟠 `uav0_full` vẫn dưới ngưỡng tin cậy 0.95 — cổng ĐỘNG của P5 phải tách biến thể hoặc chờ dual-boot |
| ⚠️ Thống kê nào dùng cho RTF | **mean, KHÔNG phải p50.** Phân bố lưỡng đỉnh (phần lớn 1.0, xen khựng sâu) nên trung vị nhảy loạn giữa các lần chạy trên cùng cấu hình — đã đo p50 ra 0.32 / 0.43 / 0.63 trong khi mean giữ 0.50–0.67 |
| Cảm biến trước của `uav0_full` còn ra dữ liệu thật (`tools/check_front_sensors.py`) | **PASS 3/3** — rgb 640×480 rgb8 (54 giá trị byte khác nhau), depth 640×480 32FC1 (173), point cloud 640×480 `x,y,z,rgb`, `camera_info` khớp fx=432.5 |
| Rangefinder khi đậu | **0.1769 m** (khoảng hở càng đáp thật) |
| Camera dưới khi đậu | `distinct=130` → **nhìn thấy mặt đất** |
| Kiểm hướng cảm biến (probe static ở z=3.0) | lidar `3.000 m`, depth `3.000 m`, camera hướng xuống thấy scene, hướng lên thấy trời |
