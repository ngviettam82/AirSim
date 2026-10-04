# Bài học — dựng world 3D Bách Khoa từ ảnh khảo sát (2026-08-12 → 08-13)

> **Trạng thái: ĐÓNG & CẤT.** Sản phẩm vẫn dùng được và vẫn nằm trong repo, nhưng
> hướng đi chính chuyển sang: **Gazebo cho ROS2/framework/logic**, còn thế giới 3D
> độ thật cao thì nghiên cứu nền Unreal (Cosys-AirSim) riêng.
>
> Plan gốc: `.claude/plan/bk-world-from-survey.md`

---

## 1. Đã làm được gì (đừng đánh giá thấp phần này)

| Sản phẩm | Vị trí |
|---|---|
| Khối SfM 997 ảnh, 1 thành phần liên thông, tái chiếu 1,47 px | `~/photogrammetry/bk_campus/` |
| DSM/DTM 0,25 m · **ortho 5 cm của chính ta** · point cloud | `odm_dem/`, `odm_orthophoto/` |
| **40 toà nhà** có chiều cao **đo được** + cột độ tin | `config/bk_buildings.csv` |
| **459 cây** có chiều cao & bán kính tán đo được | `config/bk_trees.csv` |
| World Gazebo **499 vật thể có nhãn**, bay được **M5 PASS 3/3** | `worlds/uav_arena_bk.sdf` |
| 9 công cụ đo/kiểm chứng tái dùng được | `tools/` |

**Cổng đã đạt:** thang đo (±0,5%) · mặt phẳng (RMS 0,061 m) · collision ⊇ visual
(959/959) · nhãn · bay M5 · RTF 1,0 ở world rỗng.

---

## 2. Bài học kỹ thuật — thứ đáng giữ nhất

### 2.1 🔑 Giới hạn nằm ở DỮ LIỆU, không ở công cụ

Bộ ảnh **100% nadir, không RTK, không GCP**. Ba hệ quả không phần mềm nào sửa được:

- **Không có mặt tiền** → LOD1 là trần, vĩnh viễn.
- **DTM chỉ đúng nơi camera THẤY được mặt đất.** Khu mái liền + hẻm hẹp thì nadir
  ở 90 m chưa từng thấy đất → không bộ lọc hình thái nào phục hồi được.
- **Chiều cao ±1–1,5 m** (đo bằng cổng G1‴, dựng lại độc lập từ dải bay chẵn).

> **Trước khi định dựng world từ ảnh, hãy hỏi: ảnh có NGHIÊNG không, có GCP không.**
> Hai câu đó quyết định trần của cả dự án, và hỏi mất 30 giây.

### 2.2 🔑 "Đo, không nhập" — kiến trúc đúng và vẫn đúng

Không nhập mesh photogrammetry vào Gazebo. Dùng ảnh làm **thiết bị đo**, rồi
**sinh world bằng code**. Nhờ vậy có được cả bốn thứ mà mesh không cho: nhãn
ground-truth, collision bao ngoài **do cấu trúc**, RTF cao, và số đo có độ tin.

### 2.3 🔑 Phép đo vi phân miễn nhiễm với biến dạng khối

Đổi `nDSM = DSM − DTM` (toàn cục) sang `DSM − DSM(vành 20 m)` (vi phân) khiến vòm
1 m/600 m chỉ còn đóng góp **0,0011 m**. Bài học tổng quát: **đo hiệu giữa hai điểm
GẦN nhau thì sai số hệ thống tự triệt tiêu.**

### 2.4 🔴 RTF thấp làm HỎNG độ chính xác bay — đo được 3×

| Cấu hình | RTF | Sai số ngang |
|---|---|---|
| `uav0` (không cảm biến render) | 0,986 | **0,02–0,14 m** |
| `uav0_nav` (có lidar/depth) | 0,17–0,78 | **0,32–0,37 m** |

Vật lý và cảm biến **không** bị bỏ bước (chúng theo sim time), **nhưng vòng điều
khiển thì có**: chặng DDS giữa ROS và PX4 chạy theo **đồng hồ thật**, nên RTF tụt
làm độ trễ **lớn hơn tính theo giờ mô phỏng**.

> **Quy tắc: chỉ tin số đo bay khi RTF ≥ ~0,95.**

### 2.5 Bẫy cài đặt trong `tools/` — giữ lại khi dọn ghi chú (R16)

Chi tiết dưới đây từng nằm trong comment nhiều dòng của code; gom về đây để code chỉ giữ 1 dòng trỏ tới mục này.

| # | Bẫy | Hậu quả nếu bỏ qua | Cách tránh / đã sửa |
|---|---|---|---|
| a | `bk_world_gen.py`: hệ thống `Sensors` trong world **chỉ lo cảm biến render** | Thiếu plugin `Imu`/`AirPressure`/`NavSat`/`Magnetometer` riêng → PX4 báo `"Accel Sensor 0 missing"` và **từ chối arm** dù cầu Gazebo đã nối tốt | Khai đủ plugin theo world đang bay được (`uav_arena.sdf` làm mẫu) |
| b | Đổ bóng (`cast_shadows`) tính giá theo **từng vật thể cast** | World BK có 499 vật thể → bật đổ bóng mặc định làm viewer ì | Tắt mặc định, bật lại bằng cờ `--shadows` khi cần thị giác thật cho P5 |
| c | `measure_buildings.py`: phân biệt mái/tán cây bằng "xanh VÀ gồ ghề" | Ở cửa sổ 0.25 m/1.5 m, tán cây dày cũng phẳng → **20% diện tích khu bị gọi là "mái"**, khuôn viên dính thành **1 khối 17 ha**, chỉ còn 7 đối tượng sống sót | Bỏ điều kiện "gồ ghề", chỉ dùng độ xanh (excess-green) làm tiêu chí — mái sơn xanh là điểm mù đã biết, không phải lý do nới ngưỡng |
| d | `measure_buildings.py`: gộp cấu trúc nhỏ trên mái (téc nước, lồng thang) vào toà nhà mẹ | Không gộp → mỗi téc nước thành 1 "toà nhà" 88 m² cao 23 m riêng; gộp vô tội vạ → **hàn dính hai nhà cạnh nhau** | Chỉ gộp đảo nhỏ hơn ngưỡng diện tích, và chỉ vào hàng xóm **lớn hơn ≥3 lần** |
| e | `measure_buildings.py`: chiều cao mái = `p25(mái) − p10(vành đất quanh)`; đỉnh mái = `p95` | `p50` bị kéo lệch khi téc nước/lồng thang chiếm nhiều diện tích mái nhỏ | Dùng p25 làm mép mái (chịu được nhiễu điểm cao), p95 làm đỉnh; đối chiếu với ước lượng độc lập thứ hai (min-filter 60 m) — hai bên lệch nhau mới hạ "độ tin" |
| f | `inspect_dem.py`: cửa sổ lọc DTM hẹp hơn toà nhà | Tâm mái bị lọc **hình thái** hút vào nền, chiều cao đo giữa mái **tụt về ~0** trong khi mép mái vẫn đúng — hình "bánh donut", **không thấy được bằng mắt trên hillshade** | Đo tỉ số `chiều cao lõi / chiều cao mép` theo khoảng cách tới biên (distance transform); tỉ số > 0.85 mới coi là "ok" |
| g | `analyse_block_height.py` / `check_block_doming.py`: đưa 2 chỉ báo **cộng tuyến** (sortie + strip) vào cùng một ma trận hồi quy | Ma trận **thiếu hạng**, hệ số hồi quy ra **vô nghĩa** mà không báo lỗi | Gỡ nhiễu theo **thứ tự lồng nghiêm ngặt**, không bao giờ fit hai khối cộng tuyến cùng lúc |
| h | `probe_sensor_cost.py`: cảm biến render trong Gazebo **cần có subscriber mới thật sự render** | Lần đo đầu tiên không giữ subscription nào → kết luận sai "camera 1080p tốn 0 chi phí" | Giữ một `gz topic -e` sống trên từng topic cảm biến trong lúc đo (`--subscribe`) |
| i | `check_front_sensors.py`: bản đầu chỉ lấy mẫu **byte đầu** của buffer ảnh để kiểm "có phải ảnh phẳng" | Byte đầu = góc trên-trái khung hình = bầu trời trống trong world mở → báo nhầm **camera đang hoạt động tốt là "flat/hỏng"** | Lấy mẫu rải đều (stride) trên toàn buffer, không chỉ đầu buffer |
| j | `split_strips.py`: ghi file danh sách ảnh **không ép LF** | Trên Windows, CRLF lọt vào, `\r` dính vào cuối tên file khi bash `read` đọc lại → không tìm thấy file | `open(..., newline="")` giữ LF thuần |

---

## 3. 🔴 Bài học PHƯƠNG PHÁP — đắt nhất, và lặp lại 5 lần

**Năm lần trong hai ngày, một cổng báo ĐẠT trong khi thứ nó đo đã hỏng:**

| # | Cổng "đạt" | Thực tế |
|---|---|---|
| 1 | Roof-collapse PASS 100% | Đo nhầm đối tượng: xếp hạng 12 khối lớn nhất, mà đó là **cây+nhà dính liền 13,6 ha** |
| 2 | DTM "trông hợp lý" | Nhiễm bẩn 84,9% pixel, "địa hình" cao 10,41 m — bất khả với TP.HCM |
| 3 | Sân tennis khớp chuẩn **0,13%** | Mask quét cả **mặt sân trong bóng râm**, không phải vạch sơn |
| 4 | G6 nhãn PASS 499/499 | Chỉ kiểm **file XML**; Gazebo từ chối sạch 459 nhãn vì > 255 |
| 5 | Nhãn đã sửa | Sửa file **nguồn**, quên `colcon build` → Gazebo vẫn nạp bản cũ |

**Ba quy tắc rút ra, áp cho mọi việc về sau:**

1. **VẼ RA XEM.** Một con số khớp hoàn hảo có thể đến từ mask sai. Bốn trong năm ca
   trên chỉ lộ khi vẽ hình.
2. **Cổng nào không chạy trong hệ thống THẬT thì chưa phải cổng.** Kiểm trên file
   là kiểm trên giấy.
3. **Kiểm đúng thứ mà hệ thống NẠP** — bản trong `install/`, không phải bản nguồn.

**Và hai lỗi suy luận:**

- **Suy luận hợp lý ≠ đúng.** "499 vật thể ⇒ tia lidar đắt ⇒ world là thủ phạm RTF"
  nghe rất thuyết phục và **sai**: đối chứng cho thấy world BK **nhanh hơn** world
  cũ gần trống. **Có đối chứng rồi mới được quy trách nhiệm.**
- **Chéo kiểm chỉ mạnh bằng thứ độc lập nhất giữa hai nhánh.** Hai bộ đo chiều cao
  trong **cùng một DSM** khớp nhau 0,16 m; dựng lại từ **ảnh khác** thì lệch
  **1,15 m bias + 0,92 m scatter**. Con số đầu mù với sai số mà cả hai cùng mắc.

---

## 4. Vì sao chuyển hướng — và vì sao đây KHÔNG phải thất bại

Việc dừng lại là do **so sánh giá trị**, không phải do bế tắc:

- Thứ Gazebo giỏi — **logic, timing, frame, failsafe, framework ROS2** — chuyển
  giao sang đời thật ở mức **~90–95%**. Đó là phần lớn số bug có thể giết drone.
- Thứ Gazebo yếu — **độ thật thị giác** — lại đúng là thứ cần cho perception, và
  không cải thiện được bằng cách dựng world giỏi hơn.

→ Hợp lý: **Gazebo lo framework**, nền khác lo **thế giới 3D độ thật cao**.

⚠️ **Nhưng đừng rút ra bài học sai.** Việc dựng world **đã thành công về mặt kỹ
thuật**: nó bay được, có nhãn, collision đúng, RTF 1,0. Cái không đạt là **độ thật
thị giác** — và nguyên nhân là **dữ liệu nadir + đường render WSL**, không phải
"Gazebo không dựng được world". Nhầm hai thứ này sẽ dẫn tới kỳ vọng sai ở nền tiếp
theo: **Unreal cho pixel đẹp hơn, nhưng KHÔNG tự động cho nhãn ground-truth** —
và nhãn mới là thứ để chấm điểm perception.

---

## 5. Cần gì nếu quay lại việc này

1. **Bay bổ sung ảnh nghiêng** (gimbal −45°, 4 hướng) + **8–12 GCP**. Đây là thứ
   duy nhất mở khoá được mặt tiền và độ tin tuyệt đối.
2. Dual-boot (bỏ WSL) để render không đi qua Mesa D3D12.
3. Toàn bộ công cụ trong `src/uav_sim_gz/tools/` chạy lại được ngay với dữ liệu mới.
