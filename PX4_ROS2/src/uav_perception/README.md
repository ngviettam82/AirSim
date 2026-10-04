# `uav_perception` — quan sát môi trường

> Trạng thái: **P5.1 + P5.3 + P5.4 + P5.5 + P5.7(khung) xong** — `camera_health_node`, `marker_detector_node`, `obstacle_extractor_node`, `target_tracker_node`, `object_detector_node` (khung, không tuning). world_model (P5.6) là package khác, `uav_world_model`.
>
> Kế hoạch + cổng kiểm chứng: `../../.claude/plan/P5-perception.md`

---

## 🔴 Package này KHÔNG chứa driver camera — và đó là chủ ý

Thiết kế gốc (`CLAUDE.md` §4.4) có `camera_front_node` / `camera_down_node` phơi ảnh + `camera_info`. Khi làm mới thấy: **trong mô phỏng, cầu của `uav_sim_gz` đã làm đúng việc đó** — nó phát trên chính tên topic mà driver thật sẽ dùng, nên node phía dưới không phân biệt được sim với real (R7).

Viết thêm node chuyển tiếp ở đây sẽ là **trùng vai với cầu**: hoặc thừa một chặng trên đường dữ liệu, hoặc hai publisher tranh nhau một topic. → **Driver camera thuộc phần cứng thật, làm ở P11.**

Cái P5 thật sự cần từ khối đó là **giám sát sức khoẻ camera**, và nhu cầu đó không phải suy đoán — xem mục dưới.

---

## `camera_health_node`

| Vào | Ra |
|---|---|
| `/uav/<id>/perception/front/image_raw` + `/front/camera_info` | `/uav/<id>/diagnostics/perception` (`DiagnosticArray`, chi tiết từng luồng) |
| `/uav/<id>/perception/front/depth_image` | `/uav/<id>/state/camera_health` (`DiagnosticStatus`, tóm tắt cho `uav_safety` ở P8) |
| `/uav/<id>/perception/down/image_raw` + `/down/camera_info` | |

Dùng `diagnostic_msgs` chứ không tạo kiểu riêng — theo đúng nếp `localization_health_node` của P4.

### 🔑 Vì sao nó tồn tại: ảnh mất khung là chuyện CÓ THẬT ở hệ này

Đo được 2026-08-13 (P5.2): ảnh mất **46–84% số khung**, khoảng trống dài nhất tới **1,8 s**, trong khi `camera_info` **cùng cảm biến, cùng nhịp** mất **0 khung**. Perception phải **biết** điều đó thay vì âm thầm bám dữ liệu cũ. Số đo đầy đủ: [`../../docs/package-status.md`](../../docs/package-status.md) §4.

### 🔑 Mẹo trung tâm: suy nhịp nguồn từ khoảng cách NHỎ NHẤT giữa hai dấu thời gian

Một luồng mất phần lớn khung **không thể bắt bằng cách đo tần số nó về**, vì phía dưới không biết đáng lẽ phải là bao nhiêu — **6 Hz trông y hệt một camera 6 Hz**.

Nên node không được cấu hình nhịp mong đợi. Nó lấy **khoảng cách nhỏ nhất** giữa hai dấu thời gian liên tiếp: đó là **một chu kỳ cảm biến** kể cả khi phần lớn khung đã mất. Từ đó suy ra tỉ lệ giao được, và số khung đã mất = tổng của `round(gap / chu_kỳ) − 1`.

Hệ quả có chủ đích: đổi `<update_rate>` trong SDF thì node **tự bắt kịp**, không phải sửa tham số ở hai nơi.

⚠️ **Giới hạn:** nếu **không có hai khung liên tiếp nào** cùng về thì khoảng cách nhỏ nhất là hai chu kỳ, và node sẽ tưởng cảm biến chạy chậm một nửa mà vẫn khoẻ. Đây là điểm mù thật, đã ghim bằng test `SeesEverySecondFrameMissing`.

🔑 **Chu kỳ cảm biến sống xuyên suốt kỳ báo cáo.** Mỗi lần `report()` chỉ xoá thống kê của cửa sổ đó (`received`, các khoảng cách); khoảng cách nhỏ nhất suy ra được (`source_hz`) được giữ lại — nó là thuộc tính của camera, không phải của cửa sổ báo cáo.

⚠️ **Dấu thời gian lùi bị bỏ qua, không tính là khung đến sớm.** Đồng hồ reset (vd chuyển giữa sim time / wall time) tạo dấu thời gian lùi; gộp nó vào sẽ làm hỏng chu kỳ cảm biến suy ra được cho phần còn lại của phiên chạy.

### Các phép kiểm

| Kiểm | Ngưỡng mặc định | Mức |
|---|---|---|
| Tỉ lệ giao được | < 0,8 → WARN · < 0,5 → ERROR | theo ngưỡng |
| Khoảng trống dài nhất | ≥ 0,3 s → WARN · ≥ 1,0 s → ERROR | theo ngưỡng |
| Ảnh phẳng (số giá trị byte khác nhau) | < 5 | **chỉ WARN, không bao giờ ERROR** |
| `camera_info` khớp ảnh | sai kích thước hoặc `fx` = 0 | **ERROR** |
| Không có khung nào | quá 2 s | **ERROR** |

🔑 **Vì sao 0,3 s / 1,0 s cho khoảng trống:** dưới 0,3 s, bộ theo dõi phía sau còn "lướt" qua lỗ hổng được; qua đó là đang ngoại suy; qua 1,0 s là đoán mò.

⚠️ **Ảnh phẳng chỉ được WARN.** Camera hướng lên trời quang **thật sự** cho ảnh phẳng, và không phép thử nào trên riêng bức ảnh phân biệt được nó với cảm biến chết. Nâng lên ERROR là cấm bay chỉ vì drone ngẩng lên.

🔑 **Lấy mẫu phải TRẢI ĐỀU toàn ảnh.** Lấy 4096 byte đầu là lấy góc trên bên trái — vùng trời — và điều đó đã **gọi nhầm một camera đang chạy tốt là hỏng** trong chính phiên xây node này.

🔑 **Kiểm `camera_info` khớp ảnh bắt đúng một bẫy có thật:** Gazebo suy tên topic `camera_info` bằng cách **thay** đoạn cuối của topic ảnh, nên hai camera chung một topic cha sẽ **dùng chung một `camera_info`** — và bộ nội tham số sai sẽ âm thầm làm hỏng mọi ước lượng pose phía sau.

---

## `marker_detector_node`

| Vào | Ra |
|---|---|
| `/uav/<id>/perception/down/image_raw` + `/down/camera_info` | `/uav/<id>/perception/markers` (`MarkerObservation`, một bản tin mỗi marker) |

ArUco **DICT_4X4_50**, pose bằng `solvePnP`, độ tin suy từ **sai số tái chiếu** của bốn góc.

🔑 **`camera_info` chưa tới đọc y hệt toàn số 0**, và `solvePnP` vẫn trả pose trông hợp lệ từ đó — trông giống mọi pose khác nhưng vô nghĩa. Node chặn bằng cách kiểm `fx,fy,cx,cy > 0` trước khi ước lượng bất cứ gì.

### 🔑 Pose ở frame QUANG HỌC, và `frame_id` nói đúng điều đó

`solvePnP` trả lời trong frame **optical** — X phải, Y xuống, Z dọc trục ống kính — còn phần còn lại của hệ nói theo kiểu thân: X trước, Y trái, Z lên. Hai frame khác nhau **một phép quay mà không trình biên dịch nào bắt được**, vì cả hai đều là ba số thực.

→ Node phát nguyên trong frame optical và đặt `header.frame_id` = `<uav_id>/camera_down_optical`. **Không tự đổi sang thân hay map ở đây** — việc đó thuộc `world_model` (P5.6), nơi sở hữu chuỗi transform; đổi ở đây là giấu phép quy đổi vào trong một node perception. Hàm `opticalToBody()` có sẵn trong `marker_pose` **đã có test ghim**, để P5.6 không phải nghĩ lại.

### ⚠️ Cố ý KHÔNG chặn theo tư thế

Tài liệu tham khảo chỉ nhận đo marker khi drone **thăng bằng**. Điều đó cần khi pose được suy ra với **giả định mặt đất phẳng**. `solvePnP` trả pose tương đối đủ 6 bậc tự do nên nghiêng vẫn đúng; chặn ở đây sẽ **âm thầm vứt bỏ quan sát hợp lệ**. Chất lượng báo qua `confidence`; ai đặt giả định mặt phẳng ở phía sau thì người đó chặn.

### ⚠️ Hạn chế đã biết

- **Nhập nhằng pose của marker phẳng:** nhìn gần vuông góc, một marker chấp nhận hai hướng cùng khớp ảnh. Sai số tái chiếu **không bắt được** cái này. Chỉ ảnh hưởng hướng, không ảnh hưởng vị trí.
- **Sai số dư ~1,2% theo khoảng cách** (+0,036 m ở 2,6 m, +0,043 m ở 3,6 m) — nhỏ, chưa truy nguyên; nghi lệch thang nhỏ ở cỡ marker hoặc tiêu cự.

---

## `obstacle_extractor_node` ✅ (P5.4, 2026-08-15)

| Vào | Ra |
|---|---|
| `/uav/<id>/perception/front/depth_image` (encoding `32FC1`, mét) | `/uav/<id>/perception/obstacles_local` (`ObstacleArray`) |
| `/uav/<id>/perception/front/camera_info` | |
| `/uav/<id>/state/odometry_fused` (chỉ đọc **orientation**, xem mục bộ lọc mặt phẳng nền) | |

Lọc depth hợp lệ → cụm liên thông theo bất-liên-tục độ sâu (flood fill trên lưới đã `pixel_stride`) → bbox 3D mỗi cụm. Không octomap (việc của world_model/P6 nếu cần), không giữ trạng thái giữa các khung — **mỗi khung xử lý độc lập**, đúng yêu cầu chịu được ảnh tới thưa/không đều (khoảng trống đo được tới 1,8 s, xem P5 §0.1b).

### 🔑 `camera_info` của depth mượn từ camera RGB trước — có nguồn kiểm chứng, không đoán

Gazebo suy `camera_info` cho `depth_front` bằng cách đổi đuôi topic ảnh, nhưng cầu (`bridge_uav0_full.yaml`) **không bridge topic đó** — chỉ bridge `camera_info` của `camera_front` (RGB). Node vì vậy đọc intrinsics từ `/front/camera_info` cho cả depth. Không phải đoán bừa: kiểm trong `uav0_full/model.sdf`, `sensor_camera_front` và `sensor_depth_front` **đồng vị trí tuyệt đối** (cùng `pose relative_to="base_link"` = `.12 .03 .002 0 0 0`) và **cùng `horizontal_fov` 1.274 rad, cùng 640×480** — nên K của RGB đúng cho cả depth trong sim hiện tại. ⚠️ Đây là đặc thù của model sim hiện tại, không phải quy tắc chung — khi đổi sang cảm biến depth thật (stereo/ToF riêng module), phải bridge `camera_info` riêng và bỏ giả định này.

### 🔑 Depth là Z (vuông góc mặt ảnh), không phải khoảng cách theo tia — xác nhận bằng đo thật trong sim, không tra tài liệu ngoài

`R_FLOAT32` của Gazebo không tự nói rõ quy ước. Node giả định pinhole chuẩn `x=(u-cx)z/fx, y=(v-cy)z/fy, z=depth` (cùng quy ước `depth_image_proc`/RealSense — chuẩn phổ biến của camera depth thật), rồi **verify bằng hình học biết trước trong sim** (R14) thay vì tra web (R12): hộp 1,0×0,6×0,4 m đặt ở 3 m và 5 m cho sai số kích thước **3–23 mm**, sai số khoảng cách **10–17 mm**. Nếu quy ước thật là "khoảng cách theo tia" thì sai số sẽ lệch hệ thống theo góc lệch trục (tăng dần ra rìa ảnh) — không nhỏ và ổn định như số đo được.

### 🔑 `distance` là khoảng cách tới CAMERA, không tới thân UAV

Node ở nguyên frame quang học (không đổi sang thân — cùng nguyên tắc `marker_detector_node`), nên `distance` = khoảng cách Euclid **gần nhất** từ tâm quang học tới cụm điểm. Lệch với thân khoảng 0,12–0,13 m (offset gắn camera, xem `uav0_full/model.sdf`) — `world_model` cộng bù khi đổi frame, giống cách nó xử lý `MarkerObservation`.

### 🪤 Bẫy đã trả giá khi dựng P5.4 — vật chạm mặt đất nối liền vào nền

Test đầu tiên đặt hộp lơ lửng **ngang tâm camera** → đáy hộp chìm dưới đất ~7,5 cm → depth nối liền không đứt quãng từ nền lên mặt hộp → flood fill gộp hộp + một dải nền thành **một cụm dài 3,3 m theo trục sâu** (`size.z=3.3`), méo hoàn toàn số đo (distance lệch +1,3 m, size lệch +1,3 m). Sửa test bằng cách **nâng hộp hẳn lên trên độ cao camera** (`--vertical-offset`, mặc định 0,6 m) để toàn bộ viền hộp chỉ giáp "trời" (depth vô hiệu), không giáp nền.

Nhưng phát hiện quan trọng hơn nằm ở world, không phải ở test: **`uav_arena` không "trơ" theo nghĩa hình học** — nó có mặt phẳng nền 100×100 m + `landing_pad` tại (3,0,0.005) + hai cột `obstacle_pillar_north/south` tại (8,±2,1) (xem `worlds/uav_arena.sdf`). "Trơ" ở đây chỉ nghĩa là không texture/scenery trang trí, KHÔNG phải không có hình học. Bất kỳ vật nào chạm/gần mặt đất sẽ nối liền depth với nền, và mặt nền nhìn xiên (grazing angle) là depth hợp lệ gần như ngay dưới đường chân trời.

→ Vì vậy thêm **`max_cluster_depth_span_m`** (ROS param, mặc định 2,0 m — ngưỡng thuật toán, không phải thông số thiết bị): chặn biên độ sâu **tích luỹ** của một cụm trong lúc flood fill, tách biệt với `cluster_depth_tolerance_m` (ngưỡng bước cục bộ giữa hai ô liền kề). Một mặt nền nhìn xiên có bước đổi rất nhỏ mỗi pixel nhưng cộng dồn có thể vô hạn — không chặn tổng thì một khung nhìn thấy nền sẽ tạo ra "vật cản" dài hàng mét, và tệ hơn: **có thể ăn hết `max_obstacles`, khiến vật cản thật ở xa hơn bị rớt khỏi mảng** — đây là lý do coi đây là lỗi cần sửa ngay (an toàn), không chỉ là hạn chế ghi nhận. Verify: unit test tổng hợp `BoundsChainingAcrossAGradualRamp` (dốc 2,45 m, bước 0,05 m/pixel → phải vỡ thành 2 cụm) + đo lại trong sim (cụm nền 0,415→3,737 m trước khi sửa vỡ thành 2 mảnh ≤2,0 m sau khi sửa, hộp vẫn đo đúng như trước).

### 🔑 Bộ lọc mặt phẳng nền (sửa 2026-08-22, chặn cổng bay G-M3 của P9)

**Triệu chứng thật trên dây:** `uav0_track` bay thấp bám mục tiêu, drone thấy **17–18 "vật cản" mỗi khung** trong khi chỉ có 1 vật thật — đúng dạng phân mảnh nền mô tả ở trên: bước đổi **cục bộ** giữa các dòng ảnh vượt `cluster_depth_tolerance_m` (không phải tích luỹ, nên `max_cluster_depth_span_m` không bắt được) → mỗi dòng nền vỡ thành cụm riêng. Hạ nguồn: `local_planner` ngập vật-cản-ma, né 24 hướng bí lối, mission `follow_target` không hội tụ.

**Bất biến hình học dùng để lọc — camera mức (level), nền phẳng:** tọa độ *optical-down* (`y` sau `backprojectPixel`) của MỌI điểm thuộc mặt đất bằng đúng **độ cao camera so với mặt đất**, bất kể điểm đó ở khoảng cách/góc nhìn nào (vì `y` đo độ cao vuông góc với mặt đất phẳng, trong khi các bề mặt của vật thật — không song song mặt đất — cho `y` trải rộng trên nhiều giá trị). Vì vậy node dựng histogram `y` (bin rộng `ground_margin_m`) trên TOÀN BỘ điểm depth hợp lệ **TRƯỚC khi cluster** (không phụ thuộc liên tục độ sâu, nên diệt được đúng dạng phân mảnh cục bộ ở trên); nếu bin đông nhất giải thích được ≥ `ground_min_inlier_ratio` số điểm (và vượt sàn tuyệt đối `ground_min_inlier_points`) thì mọi điểm trong bin đó (± `ground_margin_m`) bị loại khỏi cluster — không thì bỏ qua, giữ nguyên hành vi cũ (an toàn khi không chắc).

**Vì sao chiều an toàn đúng (R0) — vòng 1:** hai cổng bảo vệ (`ground_min_inlier_ratio`, `ground_min_inlier_points`) khiến bộ lọc chỉ kích hoạt khi một mặt phẳng THỰC SỰ áp đảo khung hình — mặt trước/mặt trên của một vật đứng riêng lẻ (như hộp target) không bao giờ chiếm đa số điểm hợp lệ theo kiểu đó, nên không bị nhận nhầm là nền. ⚠️ **`ground_min_inlier_ratio` đã bị HẠ ở vòng 3 (xem dưới) — bảo chứng an toàn giờ nằm ở cổng thứ ba (`ground_min_depth_below_camera_m`), không còn nằm ở ratio.** Giới hạn đã biết: dải điểm ngay chỗ vật **chạm đất** (cao ≤ `ground_margin_m` so với nền, mặc định 0,05 m) về mặt hình học không phân biệt được với nền thật — bị trộn cùng nền và mất, làm kích thước đo hụt một dải mỏng ở đáy (đo thực nghiệm: hộp 0,81 m cao mất ~3 hàng gần đáy, còn lại **0,72 m**, trong ngưỡng cổng P5.4 0,15 m). **Không bao giờ ăn mất phần thân vật** — chỉ đúng dải tiếp giáp nền.

⚠️ **Giả định camera gần như mức (level) — CHỈ đúng ở vòng 1 (xem bù nghiêng ở dưới).** Bất biến gốc trên chỉ đúng khi camera gần phẳng so với mặt đất; nếu drone nghiêng/pitch mạnh (đúng tư thế bám mục tiêu thật — camera xoay/nghiêng liên tục), các điểm nền không còn dồn về đúng một giá trị `y` nữa → bộ lọc **tự động kém nhạy hơn** (ratio không đạt, bỏ qua lọc) chứ **không** lệch sang hướng ăn nhầm vật thật — nhưng cũng có nghĩa là **0% hiệu quả quan sát được khi bay `follow_target` thật** (đo trên dây G-M3 lượt 2: 18–19 vật cản/chu kỳ, không đổi so với trước vòng 1). Sửa ở mục ngay dưới.

### 🔑 Bù nghiêng bằng attitude (vòng 2, 2026-08-22 — chặn cổng G-M3 lượt 2)

**Vì bất biến vòng 1 chỉ đúng khi mức:** camera `depth_front` gắn cứng vào thân (`0 0 0` tilt trong SDF), nên khi drone nghiêng để bám mục tiêu, camera nghiêng THEO đúng góc đó — bất biến "mọi điểm nền chung một `y` quang học" vỡ ngay cả khi không có vật cản nào cả.

**Cách phục hồi bất biến — quay điểm sang frame world-level trước khi tìm nền, KHÔNG dùng vị trí:** node subscribe `/uav/<id>/state/odometry_fused`, chỉ đọc `pose.pose.orientation`. Mỗi điểm depth quay theo chuỗi optical → body (permutation cố định, tái dùng `marker_pose::opticalToBody()` đã có test ghim) → world (dùng attitude quaternion). Giá trị lọc mới là **vertical-drop** = `-height_world`, với `height_world` tính qua **hàng thứ 3 của ma trận quay body→world** (`worldUpInBody()` trong `obstacle_extraction.cpp`) — công thức này **chứng minh được là không phụ thuộc yaw** (quay quanh trục Z thế giới không đổi thành phần Z của bất kỳ trục thân nào), nên tự động "bỏ yaw" đúng như yêu cầu mà **không cần tách góc Euler** (không có ca gimbal-lock phải lo). Khi attitude = identity, công thức thu về **ĐÚNG BẰNG** giá trị `y` quang học cũ (chứng minh trong code + test), nên đây cũng chính là nhánh thoái lui.

**Thoái lui 2 tầng (R30/R32), hàm `resolveGroundFilterAttitude()` — ROS-free, test độc lập không cần node:**
1. **Tầng 1 (compensate thật):** odometry đã có, tuổi ≤ `odometry_max_age_sec` (mặc định 0,5 s, tính từ `header.stamp`, tuổi âm/không hữu hạn bị từ chối — đồng hồ lùi không được đọc là "rất mới"), và quaternion hữu hạn + chuẩn hoá được.
2. **Tầng 2 (nhánh CŨ, không phải "bỏ lọc"):** bất kỳ điều kiện nào ở tầng 1 sai → attitude = **identity**, tức là chạy đúng công thức vòng 1 (lọc vẫn chạy, chỉ là không bù — an toàn, có thể kém nhạy khi đang nghiêng, nhưng KHÔNG BAO GIỜ ăn nhầm vật thật).

🔑 **R24 đã xét:** node giờ có thêm subscription thứ 3 (odometry) nhưng vẫn `rclcpp::spin()` mặc định (KHÔNG dùng callback group/`MultiThreadedExecutor`) → mọi callback chạy tuần tự cùng một luồng, R24 không áp dụng theo đúng cấu trúc (không phải nhờ khoá) — ghi rõ trong code, đừng thêm `MultiThreadedExecutor` mà không xét lại điểm này.

⚠️ **Còn hạn chế sau vòng 2 (đã sửa ở vòng 3, xem dưới):**
- Chỉ bù roll/pitch (đã bỏ yaw có chủ đích) — không bù được nếu mặt đất KHÔNG phẳng thật (giả định nền phẳng vẫn giữ nguyên từ vòng 1, xem README world `uav_arena`).
- Dải mỏng ngay chỗ vật chạm đất vẫn bị trộn cùng nền (đã đo ở vòng 1, trong ngưỡng cổng hiện có, xem trên) — không đổi bởi vòng 2/3.

### 🔑 Cổng "độ sâu dưới camera" + hạ ratio-gate (vòng 3, 2026-08-22 — chặn cổng G-M3 lượt 3, nguyên nhân thứ ba)

**Triệu chứng:** bản bù nghiêng vòng 2 vẫn **18–19 obstacle/chu kỳ KHÔNG ĐỔI** trên `follow_target`, trong khi log odometry 15 s trước lúc mission PAUSE (n=4846 mẫu) cho thấy tư thế **gần phẳng tuyệt đối** (|roll| max 1,08°, |pitch| max 5,38° — sâu trong vùng vòng 2 đã test PASS). Nghi ngờ nghiêng bị bác bỏ bằng đo thật.

**Chẩn đoán trên dây (được phép chạm sim, verifier nghỉ):** thêm diagnostic tạm (env `UAV_GROUND_FILTER_DEBUG=1`, in `n_valid`/`best_count`/`ratio`/`candidate_drop` mỗi khung — đã giữ lại vĩnh viễn, xem dưới), dựng `uav0_track` + `uav_arena`, spawn `target_box` thật (0,5×0,5×0,8 m) đứng yên trước drone đứng yên (camera dead-level), quét khoảng cách: **ratio bin trội đo được 0,7905 (2,5 m) → 0,7544 (2,0 m) → 0,6659 (1,5 m) → 0,5892 (1,2 m) → 0,5126 (1,0 m, đúng `min_standoff_m` thiết kế của `TrackTarget`) → 0,3659 (0,8 m)**. **Xác nhận đúng nghi phạm 1 của coordinator:** mục tiêu ở gần chiếm phần lớn khung hình (ở 0,8 m, hộp phủ ~43% diện tích ảnh theo tính góc nhìn) → tỉ lệ điểm nền/tổng điểm hợp lệ **tụt dưới ngưỡng 0,5 dù camera phẳng tuyệt đối** — không phải vì bin nền vỡ vụn (vẫn một bin, hàng chục nghìn điểm), mà vì **mẫu số** (tổng điểm hợp lệ) phình to do hộp+nền cộng lại, đúng cơ chế "ratio theo tỉ lệ khung không đại diện cảnh vật-gần" mà coordinator nêu.

**Cách sửa — KHÔNG chỉ hạ ratio, thêm cổng an toàn độc lập:** hạ `ground_min_inlier_ratio` mặc định **0,5 → 0,15** (dưới cả điểm tệ nhất đo được 0,3659, còn dư biên) — **nhưng CHỈ an toàn khi có cổng thứ ba `ground_min_depth_below_camera_m`** (mặc định 0,15 m): bin trội chỉ được coi là nền nếu **`candidate_drop` (độ sâu dưới camera, cùng đơn vị `verticalDrop`) ≥ ngưỡng này**. Nền luôn ở dưới drone một khoảng ≈ độ cao bay (sàn cứng của mission là `min_altitude_m: 0.5` trong `navigation_params.yaml`) — `0,15 m` nằm dưới sàn đó với biên dư. **Phát biểu chính xác (Y7, thu hẹp 2026-08-23):** cổng này chặn một mặt phẳng lớn NGANG TẦM HOẶC CAO HƠN camera (tường, mặt trước một vật to áp đảo khung) — **không bao giờ** đạt cổng này bất kể ratio thấp bao nhiêu. Nó **không** phân biệt được nền thật với một mặt phẳng NGANG khác cũng nằm dưới camera ≥0,15 m (nóc thùng hàng, nóc xe, mặt bàn thấp nhìn từ trên) — mặt đó vẫn qua đủ cả 3 cổng và bị xoá nhầm. Rủi ro hiện tại thấp vì hình học đang dùng: camera nhìn trước (không nhìn xuống), vật theo dõi chủ yếu là mặt đứng (hộp/tường) chứ không phải mặt ngang lớn — nhưng đây là giả định về CẢNH, không phải bảo chứng của thuật toán, xem "hạn chế đã biết" dưới.

**Xác nhận lại trên dây đúng điểm đã lộ bug:** hộp tại 0,8 m (điểm ratio=0,3659 từng thất bại) — sau sửa: `candidate_drop=0,2281` (qua ngưỡng 0,15), `triggers=1`, **một message thật chỉ còn ĐÚNG 1 obstacle** (chính là hộp, distance 0,55 m, size 0,50×0,48×~0 — khớp kích thước hộp thật), không còn mảnh nền nào.

**Diagnostic giữ lại chính thức:** `UAV_GROUND_FILTER_DEBUG=1` (env, không tốn chi phí khi tắt) in `n_valid/best_count/ratio/candidate_drop/triggers` mỗi khung qua `stderr` — giữ lại có chủ đích (không gỡ) vì đúng dạng lỗi "cổng thoái lui im lặng" đã lặp lại cả 3 vòng, cần công cụ chẩn đoán sẵn có cho lần sau.

Tham số hoá đầy đủ trong `config/perception_params.yaml` (mặc định BẬT: `enable_ground_filter: true`, `ground_margin_m: 0.05`, `ground_min_inlier_ratio: 0.15`, `ground_min_inlier_points: 200`, `ground_min_depth_below_camera_m: 0.15`, `odometry_max_age_sec: 0.5`); node validate cả 5 tham số lúc khởi động (`ground_min_depth_below_camera_m > 0` bắt buộc — bằng 0 sẽ cho một bin ĐÚNG BẰNG độ cao camera lọt qua, đó chính là tường/mặt vật, không phải nền), refuse-to-start nếu sai. File yaml **chưa nối vào `uav_bringup/launch/sim.launch.py`** (file đó tự ghi rõ chủ đích "không có yaml chung cho uav_perception") — mặc định C++ đã khớp yaml.

⚠️ **Còn hạn chế sau vòng 3:**
- **Cổng "độ sâu dưới camera" chỉ chặn mặt phẳng NGANG TẦM/CAO HƠN camera, KHÔNG phân biệt được nền thật với một mặt phẳng NGANG khác nằm ≥0,15 m dưới camera** (Y7, review lượt 1, 2026-08-23) — nóc thùng hàng, nóc xe, mặt bàn thấp nhìn từ trên vẫn qua đủ cả 3 cổng (ratio, sàn điểm, độ sâu) và bị xoá khỏi `obstacles_local` y như nền thật. Rủi ro hiện tại THẤP nhờ điều kiện hình học đang dùng (camera nhìn trước không nhìn xuống, vật theo dõi chủ yếu là mặt đứng) chứ không phải do thuật toán loại trừ được trường hợp này — nếu sau này thêm camera nhìn xuống hoặc vật mục tiêu có mặt trên ngang lớn, phải xét lại cổng này (thêm tín hiệu phân biệt, vd diện tích liên tục của mặt phẳng trong ảnh, hoặc biết trước AGL để so khớp đúng độ cao nền).
- Cổng độ sâu dùng ngưỡng CỐ ĐỊNH 0,15 m — nếu mission sau này hạ `min_altitude_m` xuống rất thấp (gần 0,15 m), biên an toàn co hẹp; chưa tham số hoá theo độ cao bay thực tế (world_model chưa cung cấp AGL cho node này).
- Vẫn giả định nền phẳng, chỉ bù roll/pitch (kế thừa vòng 2); chỉ verify với `follow_target` ở kịch bản drone đứng yên/hộp đứng yên (lát cắt rẻ per đề xuất coordinator), **chưa chạy trọn mission `follow_target` thật** (nhiệm vụ này không yêu cầu, verifier làm cổng bay lại G-M3 lượt 4 riêng).
- Dải mỏng ngay chỗ vật chạm đất vẫn bị trộn cùng nền (kế thừa vòng 1, không đổi).

### `position_uncertainty = -1` có chủ đích, không phải bỏ sót

Chưa có mô hình nhiễu cho depth camera trong sim hiện tại (`sensor_depth_front/model.sdf` không có khối `<noise>` — xem `docs/model-sources.md` §2.3, "TOÀN BỘ LÀ PLACEHOLDER, chưa chọn thiết bị"). Bịa một công thức nhiễu stereo điển hình (∝ z²/baseline) mà không có baseline/datasheet thật là đoán bừa (vi phạm R0). Theo đúng contract §2.12: `-1` hợp lệ ở tầng `/perception/*` khi detector chưa ước lượng được; `world_model` (P5.6) phải thay bằng sàn đo được khi đổi sang `/world/*` (cấm `-1` ở tầng đó).

### Field khác

- `shape = SHAPE_BOX` luôn — thuật toán chỉ dựng AABB, không phân loại hình dạng thật.
- `obstacle_id = -(thứ tự trong khung + 1)` — âm theo đúng comment `.msg` ("negative when tracking is not available"); phân biệt được để debug nhưng **không** mang ý nghĩa theo dõi liên khung (đó là P5.5 `target_tracker_node`).
- `size` là **full extent trong frame quang học** (không phải world-aligned) — theo đúng contract §2.7; vì node không đổi frame nên trục nào "rộng/cao/sâu" phụ thuộc hướng camera lúc quan sát.
- `sensing_range` = tham số `max_range_m` (mặc định 19,1 m = far clip của `sensor_depth_front/model.sdf`); `min_range_m` mặc định 0,2 m = near clip cùng file.

### ⚠️ Hạn chế đã biết

- **`size` theo trục sâu (Z quang học) không phải độ dày thật của vật.** Camera đơn chỉ thấy MỘT mặt, không thấy mặt sau (che khuất). Đo thật: 0,29 m ở 3 m cho vật thật dày 0,4 m — không ≈0 tuyệt đối như tính tay lý tưởng hoá, do rìa vật có pixel pha trộn nền/vật lúc render (antialiasing).
- **Hệ quả trực tiếp — `center` thiên lệch về MẶT GẦN, không phải tâm khối: đo +0,255 m** (chẩn đoán chuỗi TrackTarget G-M3, 2026-08-22). Cụm depth chỉ chứa mặt hướng về camera nên AABB của nó nằm trên mặt đó; consumer nào giữ khoảng cách theo `center` (standoff, inflation) thật ra đang giữ tới **bề mặt**, tức xa tâm vật thật thêm ~nửa bề dày. 🔴 **Bản chất sim-to-real, không sửa được bằng tuning**: mọi depth camera đơn (thật lẫn sim) đều thế — muốn tâm thật phải có mô hình vật hoặc quan sát từ nhiều hướng. Chỉ ghi nhận, **không** bù trừ mù trong node (đoán bề dày là đoán bừa, R0).
- **Không tracking liên khung.** Mỗi khung xử lý độc lập theo đúng yêu cầu chịu ảnh thưa. `obstacle_id` không ổn định giữa các khung — không dùng nó để "theo dõi" một vật qua nhiều lần publish. **Đây chính là việc `target_tracker_node` (P5.5, dưới) làm** — nó là consumer của `ObstacleArray`, không phải phần của node này.
- **`camera_info` mượn từ RGB** chỉ đúng vì hai cảm biến đồng vị trí + đồng FOV/độ phân giải trong SDF hiện tại — không phải quy tắc chung, phải xét lại khi đổi model cảm biến depth.
- **Dải nền mỏng gần chân trời** nay bị bộ lọc mặt phẳng nền loại trước khi cluster (sửa 2026-08-22, xem mục "Bộ lọc mặt phẳng nền" ở trên) — chỉ còn hiệu lực khi camera gần mức; nghiêng mạnh làm bộ lọc mất tác dụng (không mất an toàn) và dải mỏng có thể quay lại. `target_tracker_node` vẫn giữ lớp lọc kích thước riêng của nó làm lưới an toàn thứ hai.

---

## `target_tracker_node` ✅ (P5.5, 2026-08-15)

| Vào | Ra |
|---|---|
| `/uav/<id>/perception/obstacles_local` (`ObstacleArray`) | `/uav/<id>/perception/target_track` (`TargetTrack`, một bản tin mỗi track **đã CONFIRMED** đang theo dõi — kể cả bản tin LOST cuối cùng khi mất hẳn; track còn TENTATIVE **không bao giờ** xuất hiện ở đây, xem mục "Xác nhận M/N" dưới) |
| `/uav/<id>/state/odometry_fused` (đầy đủ pose, không chỉ orientation — xem mục "Bù ego-motion" dưới) | |

Constant-velocity Kalman **theo từng trục x/y/z độc lập** (ba bộ lọc 2 trạng thái [vị trí, vận tốc] với nhiễu quá trình *white-noise-acceleration*, không phải một ma trận 6×6 — tương đương toán học khi không mô hình tương quan chéo trục, và đơn giản hơn nhiều để viết đúng + test) cộng gating theo khoảng cách + nearest-neighbor association. Thư viện lõi (`target_tracking.hpp/.cpp`) không đụng ROS/OpenCV, đúng nếp `obstacle_extraction`/`marker_pose`.

### 🔑 Nguồn vào v0.1 là `ObstacleArray`, không phải `/perception/detections` — quyết định có chủ đích

Thiết kế gốc cho tracker ăn output của `object_detector_node` (DNN), nhưng dự án đã chốt "DNN chỉ dựng khung, không đầu tư trên Gazebo" (plan P5 §1) — trong khi nhánh hình học thuần (P5.4, PASS số đo mm-cm) đã cho quan sát 3D thật ngay trên nền hiện tại: một vật DI CHUYỂN xuất hiện trong `ObstacleArray` là mục tiêu bám được, đo được vận tốc so với ground truth.

Kiến trúc để sẵn chỗ cho nguồn thứ hai: lõi (`TrackObservation` — chỉ x/y/z + confidence + stddev, KHÔNG có field riêng của `Obstacle` như size/shape) trung tính nguồn; việc chuyển `ObstacleArray` → `TrackObservation` (kể cả lọc theo kích thước) nằm riêng trong `onObstacles()` của node, tách khỏi `MultiTargetTracker::update()`. Khi có `object_detector_node` thật (P11), thêm một callback `onDetections()` làm việc tương tự rồi gọi chung `update()` — không phải sửa lõi association/KF.

### 🔑 Chọn mục tiêu giữa nhiều obstacle — lọc kích thước tham số hoá, không hardcode cảnh

`ObstacleArray` có thể chứa dải nền mỏng (P5.4 đã khai: `size.y≈size.z≈0` gần chân trời). Trước khi đưa vào tracker, node lọc bằng `isPlausibleTargetExtent()`: từ chối cụm có trục nhỏ nhất < `min_target_extent_m` (mặc định 0,05 m, bắt dải mỏng) hoặc trục lớn nhất > `max_target_extent_m` (mặc định 5,0 m, bắt cụm nền dài bất thường). Cùng tinh thần `max_cluster_depth_span_m` của P5.4: ngưỡng thuật toán tham số hoá, không phải đặc thù một cảnh cụ thể.

### 🔑 Gate nở theo dt VÀ độ bất định — tự động từ hiệp phương sai KF, không phải công thức tuyến tính tay

Gate = `max(min_association_gate_m, association_gate_sigma × sqrt(mean(var_position_x,y,z)))`, tính từ hiệp phương sai **đã predict tới đúng stamp quan sát**. Vì nhiễu quá trình cộng dồn theo dt ngay trong bước predict, gate tự nở khi khoảng trống dài hơn — không cần một công thức "gate += k×dt" tách rời — và tự thu hẹp lại khi liên tục có quan sát khớp (KF hội tụ). Đây là lý do dùng KF thay vì bám điểm gần nhất trần trụi.

### 🔑 Xác nhận track M-của-N (sửa bug #10, G-M4.4, 2026-08-23) — chặn nhiễu 1-khung thành "mục tiêu"

**Triệu chứng thật (`~/gate_logs/gm4d_bringup.log`):** gỡ hộp mục tiêu khỏi world, nhiễu depth sót lại (1–18 obstacle thỉnh thoảng lọt qua bộ lọc nền) khiến tracker liên tục ĐẺ TRACK MỚI (`track_id` 4→8→2→11→...→40, **18 lần chuyển** trong khi không còn vật thật) — vì trước bản vá, **track mới publish ngay từ lần khớp ĐẦU TIÊN**. `world_model` bám dính đúng luật N3 (contract §2.13: chỉ đổi track khi track đang bám LOST hoặc im lặng > 1,0 s) nhưng mỗi track ma vừa sinh lại KHÔNG LOST → thắng switch → reset đồng hồ mất-mục-tiêu của cả navigator (N2, ceiling 1,0 s) lẫn mission (`TargetSeen` 1,0 s) → drone "bám" vật không tồn tại vô hạn. Bằng chứng thứ hai: đuôi `time_since_seen_sec=4,028s` (> `lost_after_sec=3,0s`) từng lọt ngay trong chặng bám LÀNH MẠNH của G-M3 — track ma nhiễm cả lúc vật thật còn đó.

**Chính sách sửa — track mới ở trạng thái TENTATIVE, chỉ CONFIRMED sau M lần khớp trong N khung (M=3, N=5 mặc định):** mỗi track mới sinh (`confirm_hits_required`/`confirm_window_frames`, tham số hoá + validate `M≥1` và `N≥M` lúc khởi động) đếm `hits`/`opportunities` mỗi lần `update()` chạy. Đạt `hits≥M` → CONFIRMED **vĩnh viễn** (chốt một chiều, không đánh giá lại); chưa đạt mà `opportunities≥N` → **chết im lặng**: xoá khỏi bộ nhớ trong, **không bao giờ xuất hiện trong bất kỳ snapshot nào** (không TRACKING, không LOST — đúng yêu cầu "TENTATIVE chết không ai thấy"). Track CONFIRMED giữ nguyên luật `lost_after_sec` cũ không đổi. M/N=3/5 ghép với nhịp đo được ~15,6 Hz (P5.2): độ trễ confirm tệ nhất 5/15,6≈0,32 s, dư biên lớn so với hai ngưỡng 1,0 s mà bug khai thác — xác nhận một vật thật không tự làm chậm tới mức vỡ chính các ngưỡng đó.

**Chiều an toàn (R0):** hạ ratio/ngưỡng không phải hướng sửa ở đây — M/N chỉ SIẾT chặt hơn (dựng rào phía "công nhận", không nới rào phía "loại bỏ"), nên không có nguy cơ mới về phía ăn nhầm vật thật; nguy cơ duy nhất là **trễ phát hiện vật thật** — đã đo trực tiếp (test `ConfirmationDelayStaysWellUnderConsumerTimeouts`): vật liên tục khớp confirm đúng **frame thứ M** (không trễ hơn), dưới 1,0 s nhiều.

**Đo trên dây (lát cắt tĩnh, drone đậu + `uav0_track`/`uav_arena`):** không tái hiện được nhiễu residual từ log bay thật ở kịch bản tĩnh này (2 vật thật ổn định — cột `obstacle_pillar_north/south` — 0 churn cả 2 cấu hình cũ/mới, `obstacles_local` ổn định đúng 2 vật/khung suốt 40 s). Kết luận trung thực: nhiễu residual trong log gốc nhiều khả năng sinh ra từ **chuyển động camera lúc bay** (đúng lý thuyết antialiasing/rendering-edge đã ghi ở `obstacle_extractor_node`), không tái hiện được bằng lát cắt tĩnh. Bằng chứng chính cho cơ chế là **đỏ-trước-xanh-sau ở mức unit qua rebuild thật** (xem "Kiểm chứng đã chạy").

### 🔑 Bù ego-motion — liên kết trong frame ODOM, không phải camera (sửa 2026-08-23, gốc rễ thật của churn)

**Triệu chứng lộ ra SAU bug #10:** bản M/N vẫn không dứt churn — track của **VẬT THẬT** (hộp còn nguyên) cũng mất-tái-bắt 2–5 chu kỳ mỗi vòng bay orbit (G-M3), trong khi lát cắt tĩnh (drone đậu) đo được **0 churn**. Kết luận: `target_tracker_node` liên kết (associate) quan sát **trong frame camera** — một frame di chuyển theo drone. Drone xoay/tiến ⇒ vị trí BIỂU KIẾN của một vật ĐỨNG YÊN nhảy trong frame camera ⇒ vượt bán kính liên kết ⇒ track chết + tái sinh id mới — kể cả khi không có nhiễu nào cả. Đây cũng giải thích vì sao M/N "không đủ": một track thật bị camera-motion chặt khúc thì không bao giờ tích đủ M lần khớp liên tiếp để CONFIRM.

**Tính tay xác nhận vượt bán kính (trước khi sửa code):** camera orbit bán kính 2 m quanh một vật đứng yên tại odom (5, 0, 1), 8 điểm quanh vòng tròn, **cùng orientation** (mức, không xoay) — độ dịch chuyển vị trí biểu kiến giữa hai điểm liên tiếp trong frame camera đo được **~1,53 m** (tính tay bằng phép chiếu ngược độc lập, xem test `StationaryTargetOrbitedByCameraKeepsOneIdWhenCompensated`), gấp **~3 lần** `min_association_gate_m` mặc định (0,5 m). Kết quả thật khi chạy: liên kết trong frame camera cho **0 track được publish** suốt cả vòng (mọi track sinh ra đều chết TENTATIVE trước khi đạt M=3, đúng dự đoán "track thật cũng bị chặt khúc").

**Cách sửa — liên kết & giữ trạng thái track trong frame ODOM, chỉ chuyển ngược lúc phát:** node subscribe `/uav/<id>/state/odometry_fused` với **đầy đủ pose** (không chỉ orientation như bộ lọc nền của `obstacle_extractor_node`). Thư viện mới `ego_motion.hpp/.cpp` (ROS-free, tái dùng `opticalToBody()`/`AttitudeQuaternion` đã có — cả hai **chuyển sang `marker_pose.hpp`** ngày 2026-08-23 để dùng chung, hành vi không đổi):
- `opticalPointToOdom()`: optical → body (permutation cố định) → odom (quay theo orientation, cộng translation odometry). Áp dụng cho MỌI quan sát **trước khi** gọi `MultiTargetTracker::update()`.
- `odomPointToOptical()` / `odomVectorToOptical()`: chiều ngược, áp dụng cho snapshot **lúc publish** — **giữ nguyên contract** (frame_id, các trường msg không đổi, thoả đúng yêu cầu "world_model không phải sửa").
- `MultiTargetTracker` (`target_tracking.hpp/.cpp`) **KHÔNG đổi một dòng nào** — đúng thiết kế "frame-neutral core" đã ghi ở đầu file; toàn bộ phép biến đổi nằm ở lớp ngoài (node), test được độc lập không cần tracker.

**Bù vận tốc là cải tiến, không chỉ giữ nguyên:** vì trạng thái nội bộ giờ ở odom, `vx/vy/vz` của track là vận tốc THẬT trong odom (không còn lẫn chuyển động của camera) — quy đổi ngược sang optical qua `odomVectorToOptical()` (chỉ quay, không cộng vận tốc camera — vận tốc odom của target vốn đã "sạch" ego-motion). Hệ quả: vật đứng yên giờ báo `velocity≈0` **bất kể drone di chuyển thế nào**, tốt hơn hạn chế cũ đã ghi ("vận tốc chỉ tuyệt đối khi drone đứng yên").

**Bỏ qua offset lắp camera (~0,12–0,13 m so với `base_link`) có chủ đích** — hằng số nhỏ, không lớn dần theo chuyển động, khác hẳn cú nhảy nhiều-mét gây churn; coi camera trùng gốc thân. Cần xét lại nếu sau này cần độ chính xác dưới mức offset lắp.

**Chế độ chuyển đổi = reset, giống hệt luật đổi frame_id đã có:** node theo dõi cờ `associating_in_odom_`; mỗi lần trạng thái sẵn sàng của odometry đổi (mới có / mất / hết hạn), node WARN + `tracker_.reset()` **trước khi** xử lý — không bao giờ để trạng thái nội bộ của một track trộn lẫn giá trị hai frame khác nhau (cùng nguyên tắc "không trộn frame" đã áp cho đổi `frame_id` camera).

**Thoái lui 2 tầng (R32)** qua `resolveBodyPose()` — cùng khuôn `resolveGroundFilterAttitude()`/`resolveGroundFilterAttitude`: Tầng 1 = odometry tuổi ≤ `odometry_max_age_sec` (0,5 s) + pose hữu hạn → liên kết trong odom. Tầng 2 (mất/cũ/hỏng) → quay lại **đúng hành vi cũ** (liên kết trong frame camera) + WARN throttle qua log mode-switch ở trên.

**R24 đã xét:** node có thêm subscription thứ 2 (odometry, đầy đủ pose) nhưng vẫn `rclcpp::spin()` đơn luồng — không callback group/`MultiThreadedExecutor` — R24 không áp dụng theo cấu trúc, giống hệt cách `obstacle_extractor_node` đã xử lý ở bug G-M3 vòng 2.

**Đo trên dây (được phép bay lát cắt động, máy trống):** orbit drone thật qua `gz set_pose` (bán kính 1,2 m quanh gốc, 16 điểm, ~0,4 s/điểm, drone gần mức) quanh hộp + 2 cột `obstacle_pillar_north/south` (3 vật thật cố định trong world), so sánh CÙNG chuyển động với/không có odometry:
- **Trước sửa** (không có odometry_fused, liên kết trong frame camera): **5 track_id khác nhau** xuất hiện cho 3 vật thật — id 1,2 chết giữa chừng, bị thay bằng id 3,4,5 mới, ổn định trở lại theo mẫu lặp "3,4,5" (**2 lần churn/tái sinh thừa**).
- **Sau sửa** (liên kết trong frame odom): **đúng 3 track_id ổn định** (1,2,3) xuyên suốt toàn bộ quỹ đạo, **0 lần churn**.

### `STATUS_TRACKING` / `STATUS_COASTING` / `STATUS_LOST` — ai đổi trạng thái khi nào

- Mỗi khi có `ObstacleArray` mới tới (kể cả rỗng — `obstacle_extractor_node` vẫn publish khi không thấy gì), track khớp được → `TRACKING`; track không khớp được khung này (kể cả do bị che khuất) → `COASTING`, vẫn predict tiếp, `time_since_seen_sec` tăng.
- Track vượt `lost_after_sec` (mặc định 3,0 s — vượt **1,8 s** khoảng trống lớn nhất đo được ở P5 §0.1b, cộng dư một chu kỳ lỡ nữa) → `LOST`, phát **một bản tin cuối** rồi xoá khỏi bộ nhớ trong; không bao giờ "sống lại" sau đó.
- Một timer riêng (`age_check_period_sec`, mặc định 0,5 s, theo `get_clock()` — **không phải wall timer**, đúng bài học nợ #1 của `offboard_session_manager_node`) quét LOST độc lập với việc có tin nhắn mới hay không — lưới an toàn cho trường hợp nguồn im hẳn (không còn `ObstacleArray` nào tới, kể cả rỗng), cùng kịch bản `camera_health_node` từng bắt (giết cầu ảnh → ERROR).

### `position_uncertainty` — số THẬT từ hiệp phương sai KF, không phải `-1`

`position_uncertainty = sqrt(mean(var_position_x, var_position_y, var_position_z))` — 1-sigma trung bình ba trục. Luôn > 0 (khởi tạo từ `position_stddev_m` của quan sát nếu nguồn có cung cấp, else `default_observation_stddev_m`), và **tăng đơn điệu khi COASTING** (predict-only chỉ cộng thêm nhiễu quá trình, không bao giờ giảm). `default_observation_stddev_m` (mặc định 0,3 m) là **giá trị tham số hoá đặt tạm, không phải số đo** — vì `obstacle_extractor_node` hiện luôn phát `position_uncertainty = -1` (chưa có mô hình nhiễu depth, xem mục "position_uncertainty = -1 có chủ đích" ở trên); khi nguồn đó có số thật, tracker dùng ngay không cần sửa code (đã kiểm `> 0.0` trước khi rơi về mặc định).

### ⚠️ Vận tốc ở FRAME CAMERA — được cải thiện 2026-08-23, không còn nhiễm ego-motion khi có odometry

`header.frame_id` **chép nguyên** từ `ObstacleArray` đầu vào (không hardcode, không tự suy `<uav_id>/camera_..._optical` như `marker_detector_node`/`obstacle_extractor_node`, vì tracker không phải nguồn gốc của frame đó). **Trước 2026-08-23:** nếu drone di chuyển/xoay, `velocity` đầu ra là vận tốc target tương đối với camera đang chuyển động, KHÔNG phải vận tốc target trong world — hạn chế đã ghi, chỉ đúng khi drone đứng yên. **Từ 2026-08-23 (khi odometry sẵn sàng, xem "Bù ego-motion" ở trên):** track được giữ trong odom nên `velocity` xuất bản = vận tốc THẬT của target trong odom, quay lại trục optical hiện tại — vật đứng yên báo `velocity≈0` bất kể drone chuyển động. Khi odometry mất/cũ (Tầng 2), quay lại đúng hạn chế cũ. Quy đổi hẳn sang `world_model`/`TargetState` (P5.6) vẫn là việc của world_model — tracker không tự trộn map/mission logic vào.

**Bảo vệ đổi frame giữa chừng:** nếu `frame_id` của `ObstacleArray` đổi giữa hai lần nhận (vd đổi camera nguồn), node `reset()` toàn bộ track đang có rồi log WARN — trộn vị trí giữa hai frame khác nhau sẽ làm hỏng mọi track một cách âm thầm.

### Field khác

- `pose.orientation` luôn identity (0,0,0,1) — obstacle chỉ có AABB, không có hướng để ước lượng.
- `confidence` = độ tin của quan sát khớp gần nhất **nhân hệ số tươi mới** giảm tuyến tính về 0 khi `time_since_seen_sec` tiến tới `lost_after_sec` — phản ánh cả chất lượng detector lẫn độ cũ của dự đoán.
- `track_id` tăng dần từ 1, không tái sử dụng sau khi một track đã LOST. **Có thể có khoảng trống trong dãy số** (vd 1,2,5 — thiếu 3,4) từ 2026-08-23: mỗi track TENTATIVE bị loại (không đạt M/N) vẫn tiêu một `id` nội bộ dù không bao giờ publish — bình thường, không phải bug.
- `max_tracks` (mặc định 20, cùng giá trị `max_obstacles` của P5.4) chặn **spawn track mới** khi đầy; track đang có vẫn tiếp tục bình thường (khác lớp lỗi đã sửa ở P5.4, nơi track cũ có thể bị "ăn" chỗ — ở đây không xảy ra vì chỉ chặn spawn, không chặn track sẵn có).

### ⚠️ Hạn chế đã biết

- **Bám nhiều mục tiêu đồng thời, nhưng chưa test với mật độ cao.** Association là nearest-neighbor + gate đơn giản (không phải JPDA/Hungarian) — đủ cho vài mục tiêu tách biệt rõ, có thể nhầm khi nhiều mục tiêu cùng lúc đi vào gate của nhau.
- **Trục độc lập (không mô hình tương quan chéo x/y/z).** Đúng với hầu hết chuyển động không có ràng buộc chéo trục biết trước; nếu sau này cần (vd mục tiêu có ràng buộc động lực học rõ), phải chuyển sang ma trận 6×6 đầy đủ.
- **`default_observation_stddev_m` là số đặt tạm, không phải đo được** — vì nguồn hiện tại (`obstacle_extractor_node`) chưa có mô hình nhiễu depth thật.
- **Cổng D1 chính thức (mục tiêu di chuyển VÀ drone đang bay) vẫn bị chặn RTF** (plan P5 §3). `scripts/verify_target_tracker.sh` chỉ đo lát "drone đứng yên, mục tiêu di chuyển" (đo được ngay theo plan P5 §0.1) — không thay thế D1 đầy đủ.
- **Xác nhận M/N (2026-08-23) thêm độ trễ phát hiện tối đa `confirm_window_frames` chu kỳ** (mặc định 5, ≈0,32 s ở ~15,6 Hz) trước khi một vật THẬT mới xuất hiện được publish lần đầu — đã đo dưới 1,0 s nhiều, nhưng là chi phí thật, không phải miễn phí.
- **Bỏ qua offset lắp camera so với `base_link`** (~0,12–0,13 m, xem mục "Bù ego-motion" ở trên) — hằng số nhỏ, đã cân nhắc và chấp nhận, không phải nguồn gây churn.
- **Chuyển chế độ (odometry mới có/mất) luôn `reset()` toàn bộ track** — đúng an toàn (không trộn frame) nhưng có nghĩa là mọi track phải xây lại từ đầu (qua M/N) mỗi lần đổi chế độ; nếu odometry chập chờn quanh ngưỡng `odometry_max_age_sec`, có thể gây reset lặp lại. Chưa có hysteresis cho việc này.
- **Chưa verify với mission `follow_target` thật đang bay qua BT/action đầy đủ** — đã verify bằng lát cắt động (orbit qua `gz set_pose`, teleport trực tiếp model, không qua PX4 offboard control thật) cho đúng cơ chế ego-motion; cổng bay mission-đầy-đủ do verifier làm riêng (G-M3/G-M4 lượt kế).

---

## `object_detector_node` ✅ (P5.7, khung — KHÔNG đầu tư, 2026-08-16)

| Vào | Ra |
|---|---|
| `/uav/<id>/perception/front/image_raw` (encoding bất kỳ cv_bridge đọc được, node tự ép `mono8`) | `/uav/<id>/perception/detections` (`vision_msgs/Detection2DArray`) |

### 🔴 Đây là KHUNG, không phải detector thật — đọc plan P5 §1 trước khi tuning bất cứ gì

Dự án đã chốt: nhận dạng bằng DNN phụ thuộc thẳng vào **độ thật ảnh**, mà Gazebo hiện tại không có (bài học world BK). Vì vậy node này chỉ có nhiệm vụ **giữ đường ống thông suốt và đúng hợp đồng topic** — cổng nghiệm thu là "đường ống chạy" (ảnh vào → `Detection2DArray` ra đúng cấu trúc/stamp/frame), **KHÔNG phải AP**. Không tuning, không huấn luyện, không benchmark chất lượng trên nền này. Detector DNN thật thuộc **P11** (drone thật) hoặc nền **Unreal/Cosys-AirSim** (đã chốt trong `CLAUDE.md` §5) — không phải Gazebo.

### Detector: `cv::HOGDescriptor` + `getDefaultPeopleDetector()` — không tải gì (R12)

Hệ số SVM người-đi-bộ **ship sẵn trong OpenCV** (mảng hardcode trong `objdetect`), không phải file model tải từ mạng — thoả R12 (cấm ra ngoài repo khi chưa xin phép). Ngưỡng/scale tham số hoá qua ROS param, KHÔNG hardcode:

| Param | Mặc định | Ý nghĩa |
|---|---|---|
| `hit_threshold` | 0.0 | ngưỡng quyết định của SVM (`cv::HOGDescriptor::detectMultiScale`) |
| `scale` | 1.05 | hệ số scale giữa các tầng pyramid |
| `win_stride_px` | 8 | bước trượt cửa sổ dò (px) |
| `group_threshold` | 2.0 | số hình chữ nhật chồng nhau tối thiểu để giữ lại (gom nhóm) |
| `min_process_period_sec` | 0.2 | **chặn thông lượng, không phải cổng chất lượng** — xem mục dưới |

### 🔑 `min_process_period_sec` — vì sao có, tính theo stamp ảnh chứ không phải wall clock

HOG chạy trên toàn khung là chi phí nặng hơn nhiều so với ArUco/depth clustering (P5.1/P5.2 đo được đường ảnh chỉ giao ~15–28 bản tin/giây cộng dồn — plan P5 §0.1b). Tham số này chặn tần suất xử lý để một detector nặng không nghẽn executor. So sánh dựa trên **hiệu hai `header.stamp`** liên tiếp (giống triết lý `target_tracker_node` dùng stamp quan sát, không dùng đồng hồ hệ thống — bài học nợ #1) nên hành vi nhất quán dù chạy sim time hay wall time.

### `vision_msgs/Detection2DArray` — đã kiểm tồn tại trong hệ trước khi dùng, không tự chế kiểu

`ros2 interface list` xác nhận `vision_msgs` **có sẵn** (`ros-humble-vision-msgs`, đã cài qua apt) trước khi đưa vào `package.xml`. Đây là kiểu ROS chuẩn, không phải `px4_msgs` — hợp lệ theo R1 (R1 chỉ cấm `px4_msgs`, không cấm mọi thứ ngoài `uav_interfaces`). `results[0].hypothesis.class_id = "person"`, `.score` là **giá trị quyết định SVM đã squash qua sigmoid** (`1/(1+e^-weight)`) — một phép co đơn điệu, **không phải xác suất đã hiệu chuẩn**. `results[].pose` để nguyên mặc định ROS (identity) vì detector 2D đơn mắt không có độ sâu — **không bịa pose 3D** (cùng tinh thần R0 với `position_uncertainty = -1` của `obstacle_extractor_node`).

### `header.stamp`/`frame_id` — cùng nếp `marker_detector_node`/`obstacle_extractor_node`

`array.header` và mọi `Detection2D.header` bên trong **chép nguyên `header.stamp` của ảnh nguồn** (không phải giờ publish); `frame_id` = `<uav_id>/camera_<camera>_optical`. Mỗi khung xử lý độc lập, không giữ trạng thái xuyên khung — chịu được ảnh tới thưa/không đều (plan P5 §0.1b).

### Kiểm thử: cấu trúc pipeline, KHÔNG kiểm chất lượng detector

Theo đúng ranh giới ở trên, unit test (`test_object_detector_node.cpp`) **không** khẳng định HOG có nhận ra "người giả" hay không — chỉ khẳng định: node không crash trên ảnh trống lẫn ảnh có hình chữ nhật giả người, `Detection2DArray` publish đúng cấu trúc, và `header.stamp`/`frame_id` đúng ảnh nguồn (không lẫn giờ hệ thống). Node được tách thành thư viện `object_detector_ros` + `main()` mỏng (nhại đúng cách `uav_world_model` làm với `world_model_node`) để một tiến trình gtest có thể tự publish ảnh tổng hợp và subscribe kết quả — không cần Gazebo.

### ⚠️ Hạn chế đã biết

- **Không phải kết quả chất lượng.** Không đo AP, không benchmark trên Gazebo — số đó không mang thông tin về đời thật (plan P5 §1, bài học world BK).
- **Không tracking/ID.** Đây là detector từng-khung; `target_tracker_node` (P5.5) hiện ăn `ObstacleArray` (hình học thuần), chưa nối với node này — README của `target_tracker_node` đã để sẵn chỗ (`onDetections()`) cho khi có DNN thật ở P11.
- **`score` không hiệu chuẩn.** Giá trị squash từ quyết định SVM, chỉ đơn điệu tăng theo độ tự tin của detector, không phải xác suất thống kê đúng nghĩa.
- **`min_process_period_sec` mặc định 0.2s là số đặt tạm** dựa trên ngân sách ảnh đo được ở P5.2, chưa đo riêng chi phí HOG trên máy đích.

---

## Kiểm chứng đã chạy (R14)

| Hạng mục | Kết quả |
|---|---|
| Unit test | **88 case / 5 target xanh** (đo lại 2026-08-23 sau bù ego-motion: 13 camera_health + **14 marker_pose** (+2: `bodyToOptical` round-trip) + **26 obstacle_extraction** (16 vòng 1 + 8 vòng 2 + 2 vòng 3) + **32 target_tracking** (15 nền + 4 bug #10 M/N + 13 ego-motion: round-trip point/vector, `resolveBodyPose` 2 tầng, orbit quanh vật đứng yên/di chuyển giữ 1 id, thoái lui khi odometry cũ) + 3 object_detector_node) — mọi vòng sửa `obstacle_extractor_node`/`target_tracker_node` đều có đỏ-trước-xanh-sau xác nhận bằng rebuild thật (vô hiệu hoá cơ chế tạm thời → đỏ đúng dự đoán, khôi phục → xanh, diff Win/WSL khớp tuyệt đối); vòng 3 obstacle + bug #10 + bù ego-motion đều có xác nhận trên dây thật (real sim) |
| **P5.7 object_detector_node: khung, build + unit test** | ✅ build sạch + **3/3 xanh** (ảnh trống, ảnh có hình chữ nhật giả-người, hai stamp khác nhau) — kiểm cấu trúc/stamp/frame, **không** kiểm chất lượng HOG (ngoài phạm vi, xem mục ở trên). Chưa chạy trong sim (nhiệm vụ này cấm chạm sim); chưa nối `ros_gz_image` thật, chỉ verify qua publisher tổng hợp trong tiến trình test |
| **P5.3 marker: bay lơ lửng trên marker ở hai cao độ** | ✅ **PASS** — 2,5 m: n=177, sai số **+0,036 m**, độ tin 1,00 · 3,5 m: n=193, **+0,043 m**, độ tin 0,98 · **bám cao độ: đổi 0,989 m, đo được 0,996 m** |
| Chạy thật, luồng lành mạnh | Báo đúng chiều: front/rgb 0,39 · front/depth 0,60 · down/rgb 0,54 |
| **Chéo kiểm với bản hiện thực độc lập** | `uav_sim_gz/image_rate_probe` (C++, viết riêng) đo cùng lúc ra 0,57 · 0,55 · 0,62 — **cùng khoảng, và `source_hz` khớp CHÍNH XÁC** (15,6 / 31,3 Hz) |
| **Bịa lỗi: giết cầu ảnh** | Cả 3 luồng → **ERROR "no frames"** trong ~12 s; tóm tắt → ERROR |
| **P5.4 obstacle: hộp 1,0×0,6×0,4 m biết trước, drone đậu, hai khoảng cách** | ✅ **PASS** — 3 m: distance +0,017 m, width −0,003 m, height −0,007 m, tin 1,00 · 5 m: distance +0,010 m, width −0,023 m, height −0,014 m, tin 1,00 (ngưỡng đạt 0,15 m cho cả ba) |
| **P5.5 target_tracker: 10 test hành vi** (ID ổn định qua vận tốc hằng/khoảng trống 1,8 s/nhịp bất thường/dấu thời gian lùi, hai track cắt nhau không tráo ID, LOST đúng ngưỡng qua cả `update()` lẫn `ageOut()`, uncertainty > 0 và tăng khi coasting, cap `max_tracks`) | ✅ build sạch + **14/14 xanh** |
| **P5.5 cổng sim: box trôi 0,30 m/s theo Z world, drone đậu** (`UAV_MODEL=uav0_track`, lát "drone đậu, mục tiêu di chuyển" theo plan P5 §0.1) | ✅ **PASS 2026-08-16** — `track_id` ổn định `[1]` không nhảy, vận tốc đo **−0,309 m/s** so kỳ vọng −0,300 (sai số **−0,009 m/s**), TRACKING 6/9 mẫu, exit 0. `gz set_pose` chạy đúng ngay lần đầu (tự-chẩn trong script không kích hoạt). ⏳ **D1 chính thức (mục tiêu di chuyển VÀ drone bay) vẫn chặn RTF.** **Bản vá P5.7:** harness (`target_track_accuracy.py`) tự sửa 2 lỗi trước lần chạy đầu — xem "Bẫy harness đã vá" ngay dưới |

⚠️ **Sai khác giữa hai bản đo camera health là do cửa sổ:** node báo mỗi 1 s nên chỉ có 5–16 khung/cửa sổ → ước lượng tỉ lệ khá thô. Nó đủ để **phát hiện suy giảm**, không dùng làm phép đo chính xác — cần số chính xác thì dùng `image_rate_probe`.

**Chạy lại:** [`../../scripts/verify_camera_health.sh`](../../scripts/verify_camera_health.sh) · [`../../scripts/verify_marker_detector.sh`](../../scripts/verify_marker_detector.sh) · [`../../scripts/verify_obstacle_extractor.sh`](../../scripts/verify_obstacle_extractor.sh) · [`../../scripts/verify_target_tracker.sh`](../../scripts/verify_target_tracker.sh)

### 🪤 Bẫy harness `target_track_accuracy.py` đã vá trước lần chạy sim đầu tiên (P5.7)

Phát hiện bằng đọc code (probe của `ros2-integration-verifier`), chưa từng chạy trong sim nên chưa từng biểu hiện thành số sai — vá trước khi tốn một lượt sim:

1. **`wait_until()`/`spin_for()` tính `deadline` trước khi có mẫu `/clock` đầu tiên.** `now_sec()` đọc `0` cho tới khi node nhận `/clock`; lần spin đầu tiên đồng hồ nhảy thẳng tới giờ sim hiện tại (~149 s), khiến `deadline = 0 + timeout` đã ở quá khứ → thoát ngay sau 1 vòng, 0 message. Vá bằng `_prime_clock()`: spin tới khi `now_sec() > 0` rồi mới tính deadline, gọi ở đầu cả hai hàm.
2. **Median vận tốc lẫn mẫu KHỞI TẠO của Kalman filter.** Mẫu TRACKING đầu tiên của một track luôn có `velocity=0.000` (KF khởi tạo từ prior 0), kéo median lệch về 0 một cách hệ thống. Vá bằng `converged_velocity_samples()`: loại mẫu đầu tiên của mỗi `track_id` trước khi tính median.
3. **Ngưỡng hợp nhất về `VELOCITY_TOLERANCE_M_S = 0.1`** (giá trị gốc của cổng P5.5; bản trước lệch thành 0.15 không có tài liệu giải thích).

⚠️ **Chưa vá:** vòng lặp `main()` gọi `subprocess.run(['gz', 'service', ...])` mỗi `--command-period` (mặc định 0,1 s → ~80 lần/8 s) để di chuyển hộp — mỗi lần là một tiến trình `gz service` mới, chi phí tiến trình này bị nghi góp phần kéo RTF xuống. Cách sạch hơn (một kết nối `gz-transport` bền vững thay vì spawn tiến trình mỗi tick) **chưa làm**: `python3-gz-transport13` có sẵn trên máy (`import gz.transport13` OK) nên khả thi, nhưng API Python chính xác (cách dựng request `gz.msgs.Pose`, gọi service) chưa được xác minh, và nhiệm vụ này không được chạm sim để tự kiểm — sửa mù một cơ chế đang tương tác với Gazebo mà không verify được là đúng thứ R14/R0 cấm. Để nguyên cơ chế `gz service` hiện tại (verifier đã xác nhận đúng schema, diff=0) và ghi lại đây làm việc tiếp theo.

### 🪤 Ba bẫy đã trả giá khi dựng P5.3 — đọc trước khi thêm asset có texture

**1. UV lật làm marker bị GƯƠNG, và ArUco báo là "ứng viên bị loại", không báo lỗi.** Nhìn từ ngoài y hệt một detector mù. Chẩn đoán: lấy đúng khung ảnh camera sinh ra, thử `flipud`/`fliplr` — lật một chiều là đọc ra ID ngay. **Dùng đúng thứ tự `vt` như bộ sinh mặt đất** (`0 0 / 1 0 / 1 1 / 0 1`); "sửa cho đúng thứ tự dòng ảnh" là **làm hỏng**.

**2. Test trên texture KHÔNG bao được lỗi này.** Chạy detector trên file PNG đã sinh thì đạt — chứng minh bộ sinh và bộ giải mã hiểu nhau — nhưng **bỏ qua đúng khâu ánh xạ texture lên lưới**, và lỗi nằm gọn trong khoảng trống không được kiểm đó. Kiểm chứng phải đi qua **đường render thật**.

**3. Lệch 0,240 m gốc-model → `base_link` cắn lần nữa.** Odometry của Gazebo báo **gốc model**, không phải `base_link`. Quên cộng thì mọi khoảng cách sai đúng một hằng số. Đây là cùng con số `vio_adapter_node` khai là `body_offset_z: 0.24`.

📌 **Và một quy tắc chung rút ra:** ảnh chẩn đoán **không được ghi vào `/tmp`** — ba lần trong dự án này file biến mất trước khi kịp xem. Ghi vào `~/marker_debug/`, và **xoá ảnh lần trước khi bắt đầu lần mới**: ảnh cũ sót lại đã khiến phần tự chẩn đoán kết luận "lỗi ở cảnh" cho một lần chạy mà nhận dạng đã chạy đúng.
