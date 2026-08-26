# Quyết định: world mô phỏng — độ thật, phủ sóng, và độ tin cậy

> Nghiên cứu 2026-08-12 · 13 agent, 6 nhánh có phản biện chéo, 702 lượt tra cứu.
> Câu hỏi: làm sao có world 3D **mô phỏng được mọi nơi**, **giống đời thực**, và **đáng tin**,
> mà vẫn giữ PX4 + ROS2 + nhãn ground-truth.

---

## Kết luận: KHÔNG đổi nền mô phỏng

> **Thứ hình học ảnh-thật mà ta định đổi nền để lấy, chính là thứ KHÔNG mang được nhãn ground-truth.**
> Đổi nền = mua được pixel đẹp, mất đúng cái ta cần.

Giữ **Gazebo Harmonic** (hỗ trợ tới 05/2029). Sửa **nguồn gốc số liệu**, **nhãn**, và **ràng buộc cảm biến↔va chạm** trước. Quyết định đổi nền chỉ được ra **từ một phép đo** (Giai đoạn 5), không phải từ cảm giác.

---

## 1. Bốn thuộc tính bị gộp làm một

Yêu cầu "world 3D hoàn chỉnh" thật ra là **bốn** thứ độc lập. **Không sản phẩm nào cho quá hai.**

| Thuộc tính | Nghĩa là gì | Cái gì giải được |
|---|---|---|
| **PHỦ SÓNG** | Mô phỏng bất kỳ đâu, bất kỳ lúc nào | OSM / Overture / Google Open Buildings; 3D Tiles streaming |
| **ĐỘ THẬT** | Nhìn và hành xử như đời thực | Tự bay chụp photogrammetry; 3D Gaussian splatting; render engine hiện đại |
| **ĐỘ TIN HÌNH HỌC** | Số đo là **đo được**, không phải bịa | Đo tay tại chỗ; photogrammetry có điểm khống chế; bản vẽ hoàn công |
| **NHÃN** | Biết vật nào là vật nào, để **chấm điểm** perception | Cảnh được **dựng có chủ đích**, mỗi vật một danh tính |

**Phủ sóng và độ thật nghịch nhau với công nghệ 2026.** Cái gì toàn cầu thì fidelity thấp, hình học yếu, hoặc bị cấm về pháp lý. Cái gì ảnh-thật thì **từng địa điểm một** và phải tự tới đó bay chụp.

🔑 **Nhãn là ràng buộc không ai tính tới.** Mesh photogrammetry, 3D Tiles và Gaussian splat đều là **hình học không phân biệt** — không tồn tại "toà nhà số 17" để gán nhãn. NVIDIA tự mô tả NuRec/3DGS của họ là *"chỉ là hình học thị giác, không có thuộc tính va chạm nào"*. Nên **đường ảnh-thật và đường chấm-điểm-được hiện là hai thế giới khác nhau**.

---

## 2. Ba nền được xét, và vì sao đều không đi được (bây giờ)

### 🔴 Google Photorealistic 3D Tiles — BỊ CẤM, loại vĩnh viễn

Chính sách Map Tiles API (cập nhật 2026-08-07, đã xác minh) ghi nguyên văn: không được dùng cho *"image analysis, machine interpretation, **object detection or identification**, geodata extraction, **offline uses**"*.

Đó là **mô tả chính xác** một simulator perception có chấm điểm. Cũng cấm cache, lưu trữ, và cấm mesh "trích xuất hoặc dẫn xuất bằng tay hay bằng máy".

**Đây là câu trả lời cho clip bạn gửi:** thứ làm game đó trông thật là dữ liệu **ta không được phép dùng vào việc của ta**.

Còn hỏng cả kỹ thuật: một người dùng **đúng stack ta định nhắm** (Isaac Sim 5.1 + Cesium + PX4 SITL) báo 2026-04-27 rằng ở **10–500 ft AGL** — đúng dải bay của ta — địa hình **đen hoàn toàn, không tile nào tải**.

### ❌ Isaac Sim — không chạy được với codebase của ta

- Nhân viên NVIDIA (2025-11-08): *"Isaac sim will not run on WSL, RTX is not supported on WSL"* — do WSL không có Vulkan, không phải bug sửa được.
- Đường vòng Windows+WSL2 đánh dấu Humble là **Deprecated** và ghi **"Custom ROS Interfaces are not supported"** → **giết thẳng `uav_interfaces`**, thứ mà mọi package của ta phụ thuộc theo thiết kế.
- Yêu cầu tối thiểu công bố: **RTX 4080 / 16 GB VRAM**.
- Pegasus ghim Isaac Sim 5.1 và tài liệu PX4 **v1.14.3** (ta ở v1.15.4).
- 3D Tiles streaming **không sinh collider** (cesium-omniverse #153, mở từ 2023).

### ❌ Unreal — đã mất đường PX4

- **Colosseum archive ngày 2026-07-11**, README từ chối Ubuntu 22.04.
- **Project AirSim** còn sống (commit tới 2026-08-11) nhưng tài liệu ghi hỗ trợ **PX4 v1.12.3**.
- Cosys-AirSim: PX4 **v1.11.x**, đã trôi sang Jazzy.
- Tài liệu PX4 xếp cả họ AirSim là *"community supported… may or may not work with current versions"*.

---

## 3. Vấn đề "không đáng tin" — tệ hơn tôi báo, và càng mở rộng càng tệ

Truy vấn Overpass trực tiếp (2026-08-12, chạy lại độc lập):

| Phạm vi | Số toà nhà | Có `height` | Có `building:levels` |
|---|---|---|---|
| Khuôn viên Bách Khoa | 1.271 | **1 (0,08%)** | 29 |
| Trung tâm TPHCM | 7.867 | 110 (1,4%) | 620 (7,9%) |

**Mở rộng world ra càng nhiều thì tỉ lệ bịa càng cao, không phải càng thấp.** Overture cũng không cứu: chiều cao của họ lấy từ OSM/Esri cộng lidar USGS 3DEP — **chương trình chỉ có ở Mỹ**.

### Không bộ dữ liệu mở nào chữa được

| Sản phẩm | Sai số **từng toà nhà** | Giấy phép | Kết luận |
|---|---|---|---|
| Microsoft heights | R=0,48 · **RMSE 20,25 m** | CDLA-2.0 | Vô dụng; gần như chắc chắn **không phủ Việt Nam** |
| 3D-GloBFP (2020) | R=0,67 · **RMSE 13,17 m** | **CC BY 4.0** | Dự phòng an toàn pháp lý; vẫn sai ~50% trên nhà 20 m |
| GlobalBuildingAtlas | Á RMSE 5,9 m — nhưng huấn luyện trên **2 thành phố châu Á vs 109 châu Âu**, có ghi nhận **ước thiếu nhà cao tầng châu Á** | **CC BY-NC** + Commons Clause | Loại nếu có đường thương mại; **ước thiếu là chiều nguy hiểm** |
| Google Open Buildings 2.5D | MAE 1,5 m — nhưng Google ghi rõ *"chỉ đánh giá ở Bắc Mỹ, châu Âu và Nhật"* | CC BY 4.0 | **Chưa kiểm chứng cho Việt Nam** |

> ⚠️ Chọn đại một sản phẩm rồi gọi là "chiều cao thật" là **thay phép đo bằng một trích dẫn** — cùng một lỗi với việc bịa số, chỉ nguỵ trang khéo hơn.

### Cách chữa tốt nhất lại rẻ nhất, và chưa ai nghĩ tới

1. **Xin bản vẽ hoàn công từ phòng cơ sở vật chất Bách Khoa.** 74 toà nhà, trong trường mình. **Chi phí: một email.** Nhiều khả năng là nguồn chuẩn xác nhất tồn tại.
2. **Ra đo tay 74 toà nhà.** Đếm tầng từ dưới đất, dùng máy đo laser hiệu chuẩn chiều cao tầng trên 3–5 nhà mẫu. **Sai số thực tế ±1,5–2,5 m** trên nhà 5 tầng — **tốt hơn mọi bộ dữ liệu toàn cầu 5–10 lần**, cho một ngày công. Rồi **đóng góp ngược lại OSM**.
3. **Photogrammetry có điểm khống chế** (ODM/WebODM) — 1–2 tuần, phụ thuộc **giấy phép bay** (chưa ai nghiên cứu, là rủi ro tiến độ).
4. **Bộ dữ liệu toàn cầu chỉ dùng làm ước lượng ban đầu**, luôn kèm sai số, **không bao giờ coi là sự thật**.

---

## 4. 🔴 Lỗi an toàn trong chính thiết kế hiện tại của ta

**`gpu_lidar` và depth camera bắn tia vào hình học `<visual>`** — đã được maintainer OSRF xác nhận, và Harmonic **không có** lidar CPU (bản `CpuLidarSensor` nhắm gz-sensors **10**, không bao giờ về Harmonic).

> **Nghĩa là: chiều cao tôi bịa CHÍNH LÀ dữ liệu khoảng cách mà stack perception bị chấm điểm dựa trên đó.**

Hai cơ chế xử lý, **một cái tôi đã bỏ sót hoàn toàn**:

- **`visibility_flags` / `visibility_mask`** (SDF 1.11): visual có `visibility_flags`, còn **cả camera lẫn lidar** đều có `visibility_mask`. Nên **có thể** gắn một visual thay thế khớp với khối va chạm, **chỉ hiện với lidar/depth**, và giấu mesh ảnh-thật khỏi cảm biến. Điều này bác bỏ khẳng định trước đó của tôi rằng lệch visual↔collision là không sửa được.
- **`mesh optimization="convex_decomposition"`** (V-HACD): **bắt buộc** cho mọi mesh nhập vào. Một ca PX4/ArduPilot có ghi nhận RTF sụp từ ~90% xuống **~5%** trên RTX 3060 chỉ vì collision mesh nhiều đa giác.

⚠️ **RTF 0.98 hiện nay là bằng chứng ta KHÔNG có mesh collision nào, không phải bằng chứng còn dư sức.**

🔴 **Quy tắc an toàn nhận ngay:** **không mesh visual nào được đi kèm khối va chạm nhỏ hơn nó.** Nếu không: drone *nhìn thấy* bức tường, *bay xuyên qua*, bài tránh vật cản **đạt trong sim** — và **giết người trên phần cứng thật**.

---

## 5. Gazebo còn dư địa chưa dùng

Vẻ xám xịt của world hiện tại là vấn đề **nội dung**, không phải trần của nền:

- **SDFormat 1.11** phơi đủ quy trình PBR metal/roughness: albedo, normal, metalness, roughness, AO, emissive, environment, light map.
- **gz-sim 8.14.0 — đúng phiên bản của ta** — có **global illumination bằng voxel cone tracing**, **mặc định tắt**, và **hoạt động với cảm biến render**. (Chỉ biến thể CI VCT mới cần Vulkan và chỉ chạy GUI — nên lo ngại Vulkan trên WSL **không áp dụng** cho tuỳ chọn ta sẽ dùng.)

Bằng chứng phản đối Gazebo yếu hơn vẻ ngoài: nghiên cứu RoboCup 2024 là **robot dịch vụ trong nhà, không phải UAV**, và chính bài đó ghi nhận cả hai simulator *"có phân bố hiệu năng tương quan cao với thực tế"*. Còn kết quả DOPE 0 phát hiện (ICRA 2023) là **Gazebo Classic**. **Chưa ai công bố so sánh với Harmonic có PBR + GI.**

⚠️ Nhưng phải ghi nhận mặt xấu: cùng bài ICRA 2023 báo fidelity thấp *"cản trở cả visual odometry lẫn object detection, **kể cả khi đã thêm texture**"* — liên quan trực tiếp tới `vio_adapter_node` của ta.

---

## 6. Lộ trình — rẻ trước, mỗi bước có phép đo chứng minh

| GĐ | Việc | Chi phí | Phép đo chứng minh |
|---|---|---|---|
| **0** | `nvidia-smi` lấy model + VRAM; đếm số nhà có height trong Overture | vài giờ, free | Hai con số trên giấy. VRAM chặn mọi phương án Isaac |
| **1** | Sinh khối `Label` plugin từ bộ sinh world | vài ngày, free | Camera segmentation phát `labels_map`; số instance có nhãn = số model có nhãn |
| **2** | `convex_decomposition` cho mọi mesh; kiểm thể tích visual vs collision; thử `visibility_mask` | vài ngày, free | Lidar so hình học biết trước ở 5 cự ly × 3 góc tới; vật rơi **đậu trên** địa hình; RTF ≥ 0.9 |
| **3** | **Xin bản vẽ + đo tay 74 toà nhà**; dựng lại; đóng góp OSM | **1 ngày công** | Bảng sai số từng toà nhà so với số đo thật. **Deliverable giá trị nhất toàn kế hoạch** |
| **4** | Tự bay photogrammetry có điểm khống chế | 1–2 tuần | 5 cự ly khống chế trong dung sai; RTF ≥ 0.9 khi nạp mesh |
| **5** | 🔑 **Thí nghiệm quyết định** | vài ngày, free | Chạy **cùng** detector + VIO trên: (a) world hôm nay · (b) world đó **có PBR + GI** · (c) **cảnh quay thật**. So **AP và độ trôi VIO** ba chiều |
| **6** | Thuê Isaac Sim trên cloud (L40S) — **chỉ khi GĐ5 chứng minh cần** | ~2 ngày, ~$50 | (1) `uav_interfaces` có build và truyền tin không; (2) **jitter nhịp offboard ≥2 Hz** trong 10 phút; (3) PX4 v1.15.4 có chạy với Pegasus không |

> **Giai đoạn 5 biến "Gazebo xấu quá" thành một con số.** Nếu (b) khép được phần lớn khoảng cách tới (c) thì Gazebo **không phải** nút thắt và câu hỏi đổi nền coi như đóng.

**Nếu tới GĐ6: thuê, đừng mua.** Cloud L40S cho Linux native + GPU ≥16 GB theo giờ, PX4 SITL và ROS2 nằm cùng máy nên lockstep không qua mạng. Gỡ được cả rào WSL2 lẫn rào phần cứng cho một đợt đánh giá 2 ngày.

---

## 7. Còn chưa biết (và cách chốt)

| Chưa biết | Trạng thái | Thí nghiệm chốt |
|---|---|---|
| ~~dGPU của laptop là gì, bao nhiêu VRAM~~ | ✅ **ĐÃ ĐO 2026-08-12: RTX 5060 Laptop, 8151 MiB VRAM** (CPU Ultra 9 285H 16 nhân, RAM 31,4 GB, đĩa C: còn 122 GB) | → **Isaac Sim loại vĩnh viễn**: yêu cầu công bố là RTX 4080 / 16 GB. Câu hỏi ĐÓNG, GĐ6 không còn khả thi tại chỗ |
| VCT GI có chạy dưới WSLg không | Không có bằng chứng nào cả hai chiều. Ca crash được trích là **lạc hậu** (Fortress, đóng 2023) | Bật `<global_illumination>` + cảm biến render, đo RTF và VRAM |
| Bridge `vision_msgs` có trong bản Humble+Harmonic **của máy này** không | **UNCERTAIN** — chỉ verify trên nhánh `ros2` | Soi registry của bridge đã cài |
| Gaussian splat có vô hình với lidar/depth không | **UNCERTAIN** — một báo cáo bên thứ ba nói splat không hiện trong depth. Nếu đúng thì đường splat **mất cả depth camera lẫn rangefinder** | Đợt thuê cloud |
| World Gazebo render ra là "Derivative Database" hay "Produced Work" theo ODbL | **Còn mở** — rào cản trước đây đã bị **bác bỏ** | Đọc OSMF Community Guidelines |
| Giấy phép bay UAV khảo sát khuôn viên đô thị ở Việt Nam | **Chưa ai nghiên cứu.** Chặn hoàn toàn GĐ4 | Hỏi chính thức; coi là rủi ro tiến độ |
| Phòng cơ sở vật chất có giữ bản vẽ hoàn công không | **Chưa ai hỏi** | Một email |
| Pegasus có chạy với PX4 v1.15.4 không | Không nguồn nào khẳng định hay phủ định | Bench test GĐ6 |

**Hai mốc lịch chưa ai nêu:** Gazebo Harmonic hỗ trợ tới **05/2029**, nhưng **Ubuntu 22.04 hết hạn 04/2027** — áp lực di chuyển nằm ở **tầng hệ điều hành**, không phải simulator, và nó dính với kế hoạch dual-boot. Và `laser_retro` có trong SDF 1.11 nhưng ghi *"sẽ hiện thực ở bản sau"* → kế hoạch hạ cánh chính xác dựa vào cường độ phản xạ có **lỗ hổng cứng**.

---

## 8. Những khẳng định đã bị BÁC BỎ (không được dùng lại)

Vòng phản biện đã bác các claim sau; chúng **không** xuất hiện như sự thật ở trên:

- Trang attribution của Overture bắt buộc ODbL share-alike cho world dẫn xuất — **sai**, trang đó không nói vậy.
- Cesium ion không có đường xuất offline hợp lệ — **sai**, ToS §2.3 cho phép Value-Added Clips xuất 3D Tiles/glTF. *(Nhưng Cesium OSM Buildings cũng là đùn từ OSM nên **mang đúng lỗi bịa chiều cao** ta đang muốn thoát.)*
- Không nguồn ảnh nào cho ra chiều cao nhà — **sai**, Google Open Buildings 2.5D suy chiều cao từ ảnh Sentinel-2.
- Gazebo không có mesh streaming — **sai**, gz-sim 8 có Levels (nạp/xả entity động).
- AW3D Enhanced là DSM dưới mét thương mại **duy nhất** trên Việt Nam — **sai**, Maxar Precision3D là nhà cung cấp thứ hai. Nếu trả tiền thì **lấy hai báo giá**.
- Điểm khống chế ODM "thường trong 5–10 cm" — **không có căn cứ** trong tài liệu ODM.
- Đường WSL2 của Isaac Sim chỉ "mong manh" — thực tế **không dùng được** với codebase dựng trên custom ROS interfaces.

---

## 9. 📸 Bước ngoặt: chủ dự án CÓ SẴN bộ ảnh khảo sát Bách Khoa (2026-08-12)

Sau khi nghiên cứu kết thúc, chủ dự án cho biết **đã có một bộ ảnh bay khảo sát toàn bộ khuôn viên Bách Khoa**. Điều này gỡ đúng ba thứ tệ nhất ở trên:

| Vấn đề trong báo cáo | Với bộ ảnh này |
|---|---|
| Chiều cao bịa — Bách Khoa chỉ **1/1271** nhà có số liệu | **Đo được thật** |
| Giấy phép bay khảo sát — rủi ro tiến độ, chưa ai nghiên cứu | **Đã bay rồi** |
| Giấy phép dữ liệu — Google 3D Tiles bị cấm | **Chủ dự án sở hữu dữ liệu** |

→ **Giai đoạn 4 của lộ trình có sẵn nguyên liệu.** Đây là việc mở màn của phiên sau, **trước** cả ba nợ P4 và P5.

### Kiến trúc để lấy được **cả bốn** thuộc tính (cho Bách Khoa)

> **Cắt mesh photogrammetry theo footprint từng toà nhà** → mỗi nhà thành một model riêng: mesh thật của chính nó + collision riêng + **nhãn riêng**.

Đây chính là thứ Google 3D Tiles không cho phép (**cấm cắt, cấm dẫn xuất**), còn dữ liệu tự bay thì cho.

| Cảm biến | Nhận được gì |
|---|---|
| Camera | Mặt tiền, mái, cây thật |
| Lidar / rangefinder | Bắn tia vào `<visual>` = **mesh thật**, kể cả tán cây |
| Depth camera | Cùng cơ chế |
| Va chạm | Convex hull của chính mesh đó — **bao ngoài mesh** → lệch về **phía an toàn** |
| Ground truth | ✅ có, vì mỗi nhà là một thực thể riêng |

**Phủ sóng vẫn giữ:** Bách Khoa dùng world đo thật; mọi nơi khác vẫn dùng bộ sinh OSM (`tools/vn_world_gen.py`). **Hai world, một pipeline.**

### ✅ Bốn câu hỏi về bộ dữ liệu — ĐÃ ĐO XONG 2026-08-12

Đo trực tiếp trên `C:\code\Python\MapDienHong_v2` (exiftool 997/997 ảnh + rasterio trên ortho). Chi tiết đầy đủ: `.claude/plan/bk-world-from-survey.md` §1.

| # | Câu hỏi | Đáp án đo được |
|---|---|---|
| 1 | Nghiêng hay nadir? | 🔴 **100% NADIR** — `GimbalPitchDegree` ∈ [−90.0°, −89.9°] trên cả 997 ảnh, **0 ảnh nghiêng** |
| 2 | Toạ độ? | 🔴 **Không RTK, không GCP** (không có tag `RtkFlag`/`RtkStd*`). XY tuyệt đối ±1–3 m; **datum cao độ sai ~+70 m — cấm dùng** |
| 3 | Bao nhiêu ảnh / cao / chồng phủ? | ✅ **997 ảnh · 90,0 m AGL (σ 0,12 m) · chồng phủ dọc ~85%** · dư thừa 31 ảnh/40 m · 4 sortie, 47 phút, 2026-05-08 |
| 4 | Đã dựng chưa? | ⚠️ **Chỉ có orthomosaic** (25345×30698, EPSG:4326, GSD **2,53 cm**, 27,6 ha). **Không có DSM/DTM/point cloud/mesh** |

**Kết luận:** rơi vào vế xấu của chính cảnh báo dưới đây — nadir-only, không GCP.
→ **Hệ quả kiến trúc:** bộ ảnh dùng làm **thiết bị ĐO** (chiều cao + mặt đất: xuất sắc, ~0,1 m tương đối), **không** dùng làm **tài sản world** (tường: không đáng tin). Kiến trúc "đo, không nhập" ở `bk-world-from-survey.md` §3.

> Nadir-only không GCP → bản đồ **đẹp nhưng vẫn không đáng tin về hình học**.
> Oblique + GCP → **tốt hơn mọi dữ liệu toàn cầu hiện hữu**. ← **một buổi bay bổ sung là đòn bẩy cao nhất còn lại** (§6 của plan).

### Rủi ro thật của mesh photogrammetry

Chủ dự án nêu **cây cối không ra point cloud** — đúng, nhưng đó là rủi ro **nhẹ nhất**: mesh cây lù xù nhưng lidar vẫn trả về, và convex hull **to hơn cây thật** → drone dừng sớm, lệch về phía an toàn.

Ba thứ nguy hơn:

- 🔴 **Mảnh rác bay lơ lửng** (bầu trời, mặt kính, mặt nước) → **vật cản ma**: lidar báo có vật ở chỗ trống, va chạm chặn drone giữa không trung. Không dọn thì bài tránh vật cản vô nghĩa.
- 🔴 **Mesh không có danh tính** → không chấm điểm perception được. Chính là lý do phải cắt theo footprint.
- 🔴 **Không có mặt đất phẳng** — đất và nhà dính một mặt → bãi cất cánh gồ ghề, không tách được nền cho optical flow.

### Bắt buộc trong pipeline

Giảm đa giác cho visual · **`mesh optimization="convex_decomposition"`** cho collision · **Levels** của gz-sim 8 để nạp/xả theo vùng.

⚠️ Có ca ghi nhận **RTF sụp 90% → 5%** chỉ vì collision mesh nhiều đa giác. **RTF 0.98 hiện tại là bằng chứng ta CHƯA có mesh collision nào, không phải bằng chứng còn dư sức.**

### Phép đo phải làm đầu tiên

**So mesh với thước dây thật** — để biết nó đáng tin tới đâu, chứ không chỉ đẹp tới đâu. Đẹp mà sai vẫn là "không đáng tin", đúng thứ đang muốn thoát.
