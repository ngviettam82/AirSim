# `uav_world_model`

Trả lời câu hỏi **"cái vừa nhìn thấy nằm ở đâu trong thế giới?"** — và trả lời **kèm mức độ tin cậy**.

> Đây là chỗ **duy nhất** trong hệ đổi quan sát từ frame camera sang frame `odom`. `marker_detector_node` cố ý **không** tự đổi (xem README của `uav_perception`), vì chuỗi transform thuộc về node sở hữu pose.

Kế hoạch & cổng kiểm chứng → `.claude/plan/P5-perception.md` §2 (P5.6), §3 (S5/S6).
Hợp đồng message `/world/*` → [`docs/interface-contract-v0.1.md`](../../docs/interface-contract-v0.1.md) §2.12–§2.13.

---

## Ràng buộc trung tâm

`uav_localization` có ràng buộc *"không thể phát ra một vị trí mà không kèm mức độ tin cậy"*. Package này là **mở rộng của ràng buộc đó sang lớp bản đồ**:

> **Sai số bản đồ = sai số perception ⊕ sai số định vị.** Và vế thứ hai đã **0,3–0,4 m** trước khi camera nhìn thấy gì (cổng G2).

Hệ quả cưỡng chế trong code, không phải lời dặn:

| Tình huống | Xử lý |
|---|---|
| Detector khai `position_uncertainty = -1` | thay bằng **sàn đo được** của detector |
| Detector để nguyên **0** (mặc định của msg) | y hệt `-1` — 0 nghĩa là "chính xác tuyệt đối", không phép đo thật nào như vậy |
| `odometry_fused` khai covariance **0 hoặc -1** | **quan sát bị loại**, không vào bản đồ, có đếm |
| `odometry_fused` mang vị trí NaN hoặc quaternion rỗng | **pose bị loại** — quaternion 0 chuẩn hoá thành đơn vị, tức *"bay bằng, mũi chỉ bắc"*, đúng cái bẫy `uav_localization` đã ghi |
| `odometry_fused` mang dấu thời gian ở **tương lai** quá `max_future_stamp_sec` | **pose bị loại** — một mẫu như vậy quét sạch bộ đệm rồi khoá nó vĩnh viễn; đó là chữ ký của lệch `use_sim_time` |
| Mount có tên trong `frames.names` nhưng **thiếu** `translation`/`rotation_rpy` | tên lạ → **ERROR, bỏ mount**; tên có sẵn số của sim → dùng tạm nhưng **WARN to** rằng đang chạy hình học MÔ PHỎNG |
| Không có pose đủ gần thời điểm quan sát | **quan sát bị loại**, có đếm |
| `frame_id` của quan sát không có trong bảng mount | **quan sát bị loại**, cảnh báo có tên frame |

→ Giá trị `position_uncertainty` trên `/world/*` **không bao giờ là -1 và không bao giờ là 0**. Điều đó đúng **theo cấu trúc**, không nhờ may mắn: mọi đường vào bản đồ đều đi qua một pose đã khai được sai số, cộng với một sàn perception dương.

## Mọi hằng số đến từ đâu

Sai một hằng số gắn camera thì **mọi landmark lệch đúng hằng số đó** — và lệch 0,240 m gốc-model → `base_link` đã cắn dự án **hai lần**. Nên mỗi số ở đây có địa chỉ:

| Hằng số | Giá trị | Nguồn |
|---|---|---|
| Camera dưới, vị trí so với `base_link` | `(0.06, 0, -0.065)` m | `uav_sim_gz/models/uav0_nav/model.sdf` (và `uav0_full`) gắn module ở `0.06 0 -0.05`; `sensor_camera_down/model.sdf` đặt cảm biến thêm `0 0 -0.015`. Đối chiếu độc lập: `uav_perception/test/marker_accuracy.py::CAMERA_BELOW_BASE_LINK = 0.065` |
| Camera dưới, hướng | rpy `(0, 1.5707963, 0)` | cùng dòng `<pose>` của cảm biến trong `sensor_camera_down/model.sdf` — *"Pitch +90 deg turns sensor +X axis into body -Z"* |
| Camera trước, vị trí so với `base_link` | `(0.12, 0.03, 0.002)` m | `uav0_full/model.sdf` gắn **cả** `sensor_camera_front` lẫn `sensor_depth_front` ở `<pose relative_to="base_link">.12 .03 .002`, `uav0_track/model.sdf` y hệt. Hai model cảm biến **không khai `<pose>`** cho link lẫn `<sensor>` → chuỗi cộng dừng tại đó, không có số hạng thứ hai như camera dưới. ✅ **Đối chứng vật thật PASS 2026-08-16** (cổng (b), residual hằng ≈0 — xem bảng Trạng thái) |
| Camera trước, hướng | rpy `(0, 0, 0)` | Không `<pose>` nào xoay cảm biến, nên trục quang đã nằm dọc body **+X**. Camera dưới phải pitch +90° mới nhìn xuống; camera trước thì không |
| Sàn bất định marker | `0.0185 + 0.007 · d` m | Hai điểm đo của P5.3, **theo cự ly xiên từ ống kính** — không phải cao độ hover. `marker_accuracy.py` cộng `0,240 − 0,065 − 0,01` nên hover 2,5/3,5 m là cự ly **2,665/3,665 m**. Sai số +0,036/+0,043 m tại hai cự ly đó ⇒ dốc `0,007`, chặn `0,0173`; **làm tròn LÊN 0,0185** cho biên an toàn |
| Trôi frame `odom` | σ `0.5` m, τ `120` s | `uav_bringup/config/localization_params.yaml`, `vio_adapter_node.degrade.drift` — chính mô hình trôi dự án đang tiêm |
| Cửa sổ quên landmark | `120` s | Một chu kỳ tương quan của trôi trên: quá đó, vị trí đã lưu không còn mang thông tin ngoài phần bất định đã bão hoà |
| Cửa sổ quên vật cản | `5` s | Khoảng trống ảnh **đo được tới 1,8 s** (P5.2); cửa sổ phải rộng hơn hẳn nếu không vật cản sẽ nhấp nháy |
| Sàn bất định vật cản / mục tiêu | `0.10 + 0.02 · d` m | 🟠 **Chỗ giữ chỗ** — P5.4/P5.5 chưa đo. Node **kêu WARN định kỳ 10 s** khi thật sự dùng tới nó, không phải một lần rồi thôi |

⚠️ **Hoành độ là cự ly xiên, không phải cao độ.** Ba tài liệu nội bộ đang ghi hai điểm này ở ba hoành độ khác nhau (2,5/3,5 · 2,6/3,6). Trong `uav_world_model` thì `d` **luôn là** `norm(pose marker trong frame quang học)` — cự ly từ ống kính tới marker. Ghim bằng test tại 2,665/3,665 m.

⚠️ **Số của P5.3 là thiên lệch hệ thống, không phải σ thống kê.** Dùng nó làm **sàn** là phát biểu đúng: *"sai số thật ít nhất chừng này"*. Khai nhỏ hơn con số đã đo được là nói dối. Khai nó như một σ đầy đủ thì lại thiếu phần nhiễu ngẫu nhiên — nên nó là **cận dưới**, và tài liệu này nói rõ như vậy.

## 🔴 Trường "so với máy bay" không được mang qua biên đổi frame

`Obstacle.distance` là *"closest range to the vehicle"* — một đại lượng **tương đối với máy bay tại thời điểm quan sát**. Bản đồ này sống tới `obstacle_forget_sec` và được phát lại 10 Hz, nên **chép nguyên con số đó là nói dối ngay khi drone dịch chuyển**: extractor báo 3,0 m, drone bay 2,5 m/s về phía nó, bốn giây sau bản đồ vẫn khai 3,0 m trong khi thực tế còn 0,5 m. Số nằm đúng đơn vị, đúng dải, không NaN — nên không ai bắt được.

→ `distance` **không được lưu** trong bản đồ. Nó được **tính lại mỗi lần phát** từ pose fused mới nhất:
`distance = max(0, |tâm − base_link| − ½·|kích_thước|)`. Trừ nửa đường chéo là **hướng bảo thủ** — không bao giờ khai xa hơn thực tế. Không có pose thì **không phát bản đồ**.

Cùng lý do, vận tốc mục tiêu cộng đủ **ba** số hạng khi nguồn khai vận tốc tương đối: `v_odom = R·v_rel + v_thân + ω × r`. Bỏ `ω × r` thì drone **hover xoay 0,5 rad/s** nhìn mục tiêu **đứng yên** cách 1,5 m sẽ báo mục tiêu chạy **0,75 m/s** — planner nào lead theo vận tốc sẽ đuổi bóng.

## Ghép cặp pose ↔ quan sát theo DẤU THỜI GIAN

Ảnh về **thưa và không đều** (P5.2 đo: khoảng trống 0,5–1,8 s). Ghép "cái mới nhất với cái mới nhất" là sai ngay khi drone đang bay: quan sát mang dấu thời gian cũ hơn pose mới nhất.

Nên node giữ một **bộ đệm pose** (mặc định 3 s) và với mỗi quan sát:

- Quan sát nằm **giữa hai pose** → nội suy tuyến tính vị trí, **slerp** hướng.
- Quan sát nằm **ngoài** bộ đệm → dùng pose gần nhất, nhưng chỉ khi lệch ≤ `max_pose_gap_sec` (0,25 s, tương ứng mux 10 Hz). Quá thì **loại**.
- Khoảng lệch thời gian còn lại được **trả giá bằng bất định**: `σ_ghép = |Δt| × tốc_độ`. Drone 2,5 m/s ghép với pose cũ 0,2 s là **0,5 m** — lớn hơn mọi thành phần khác, nên phải nói ra chứ không giấu.

## Bất định: bốn thành phần, cộng theo bình phương

```
σ = sqrt( σ_perception² + σ_định_vị² + σ_ghép_cặp² + σ_tuổi² )
```

| Thành phần | Lấy từ đâu |
|---|---|
| `σ_perception` | trường `position_uncertainty` của quan sát, nâng lên **sàn đo được** khi nguồn khai `-1`, `0`, hoặc nhỏ hơn sàn |
| `σ_định_vị` | **trục xấu nhất** của covariance vị trí trên `odometry_fused` (`sqrt(max(cov[0], cov[7], cov[14]))`) — chọn trục xấu nhất vì khai thiếu là hướng nguy hiểm |
| `σ_ghép_cặp` | `|Δt| × tốc_độ` như trên |
| `σ_tuổi` | trôi tương đối của frame `odom` giữa lúc thấy và lúc phát |

**`σ_tuổi` dùng đúng mô hình Gauss-Markov mà dự án đang tiêm:** với σ và τ cho trước, độ trôi *tương đối* sau Δt là `σ·sqrt(2(1 − e^(−Δt/τ)))`. Nó **bão hoà** ở `σ·√2` thay vì tăng vô hạn — đúng với một quá trình trôi có tương quan.

> ⚠️ **Giới hạn sim-to-real đã biết:** trên phần cứng thật, trôi của frame `odom` **không bị chặn** (đó là lý do REP-105 tách `map` khỏi `odom`). Mô hình bão hoà ở đây được hiệu chỉnh theo bộ tiêm của dự án. Khi có số đo trôi của VIO thật thì **phải xem lại chỗ này**.

## Vì sao KHÔNG trung bình các lần nhìn thấy lặp lại

Nhìn thấy marker 100 lần rồi lấy trung bình sẽ làm bất định co lại theo `1/√N` — **và đó là nói dối**. Sai số trội là **trôi định vị**, thứ có **tương quan mạnh theo thời gian**: 100 quan sát liên tiếp dùng chung gần như một pose sai giống nhau. Trung bình chúng khử được nhiễu pixel, **không** khử được 0,4 m kia.

→ Bản đồ **giữ lần nhìn thấy mới nhất** và công bố tuổi của nó. Cùng họ lý do với `localization_mux_node`: nó **chọn** một nguồn chứ không **trộn**.

*(Đã cân nhắc và loại: dùng marker tĩnh để hiệu chỉnh ngược trôi định vị — đó là SLAM, và định vị là việc của P4/`uav_localization` theo R4. Ghi lại ở đây để không ai thêm nhầm vào package này.)*

## Im lặng và lên tiếng: hai lựa chọn KHÔNG đối xứng

| Topic | Khi chưa có / mất nguồn | Vì sao |
|---|---|---|
| `world/semantic_landmarks` | **vẫn phát**, mảng rỗng | Bản đồ landmark rỗng không phải một lời tuyên bố an toàn. Phát đều cho phía sau thấy node còn sống |
| `world/obstacle_map_local` | **ngừng phát** | Một bản đồ vật cản rỗng đọc thành *"không có gì phải né"*. Nguồn chưa từng nói, đã câm quá `obstacle_source_timeout_sec`, **hoặc mọi lô đã hết hạn** → **im lặng**, để phía sau bắt bằng timeout thay vì tin một lời trấn an sai. Điều kiện "còn lô sống" khiến hai cửa sổ không cần phải đặt đúng thứ tự mới an toàn |
| `world/target_state` | **chỉ phát sau khi từng thấy** mục tiêu | Chưa từng thấy thì không có pose để bịa, cũng không có bất định để khai |

🔴 **`target_state` mang tuổi QUAN SÁT và bám dính một `track_id`** (sửa 2026-08-22, hai bug do chẩn đoán cổng bay G-M3 phát hiện). `time_since_seen_sec` = tuổi tracker khai **cộng** tuổi đường truyền, không phải chỉ tuổi message; và vì `target_tracker_node` phát **mỗi track một message trên cùng topic** (kể cả track LOST bị thải) nên node **chọn một track rồi bám**, không lấy message cuối. Luật đầy đủ + số đo trước-sau: hợp đồng [`docs/interface-contract-v0.1.md`](../../docs/interface-contract-v0.1.md) §2.13. Núm duy nhất: `target_track_switch_after_sec` (mặc định 1,0 s, giữ ≥ 2× chu kỳ phát `target_track`). Đổi track được phát ra bằng **WARN + bộ đếm `unselected_track`** trong log định kỳ — `TargetState` đóng băng, không thêm trường.

⚠️ **Đây là chỗ hợp đồng còn thiếu:** `ObstacleArray` **không có trường hợp lệ/trạng thái**, nên không có cách nào nói *"tôi còn sống nhưng dữ liệu đã cũ"* trên chính message đó. Im lặng là lựa chọn trung thực duy nhất còn lại. Nếu sau này thêm được trường trạng thái thì nên thêm.

`ObstacleArray` phát ra mang **dấu thời gian của mục cũ nhất** trong bản đồ, không phải giờ phát — bản đồ không được trông tươi hơn nội dung của nó. `sensing_range` lấy **nhỏ nhất** trong các nguồn góp mặt: chỉ dám bảo đảm tới tầm ngắn nhất.

## Kích thước vật cản khi đổi frame

`Obstacle.size` là **kích thước bao**, không phải một vector. Quay một khối hộp rồi giữ nguyên ba số là sai. Node tính **bao trục-song-song** của khối đã quay (`|R| · extent`) — kết quả **không bao giờ nhỏ hơn** khối gốc. Với việc né vật cản, khai thừa là hướng an toàn.

## 🔴 Vì sao bộ tiêm trôi của P4 là đòn bẩy SAI cho cổng S6

Cổng S6 phát biểu *"độ bất định phải tăng khi bật tiêm nhiễu P4"*. **Đi đường đó sẽ không bao giờ đạt**, và lý do nằm trong chính code của P4:

- `gps_adapter_node.cpp`: *"Injected drift deliberately does not inflate the reported accuracy."*
- `vio_adapter_node.cpp`: `report.position_stddev = nominal_position_stddev_;` — **hằng số**, bật suy giảm hay không cũng vậy.

Đó là **chủ ý và đúng**: máy thu thật vẫn khai mình chính xác trong lúc đang trôi. Nhưng nghĩa là bật trôi **không làm covariance của `odometry_fused` nhúc nhích**, nên `/world/*` cũng không.

→ **Đòn bẩy đúng là làm VIO mất tư cách** (`degrade.force_tracking_loss: true`). Mux khi đó buộc phải lấy GPS, mà GPS **khai** ≈0,9 m thay vì 0,10 m của VIO. Sai số khai báo tăng thật, và nó phải chảy tới `/world/*`. Đây chính là bài học đã ghi ở [`docs/package-status.md`](../../docs/package-status.md) §5: *"Muốn kiểm hành vi GPS qua mux thì phải làm VIO mất tư cách trước"*.

[`scripts/verify_world_model.sh`](../../scripts/verify_world_model.sh) làm đúng như vậy.

## Chạy

```bash
ros2 run uav_world_model world_model_node --ros-args \
  --params-file install/uav_world_model/share/uav_world_model/config/world_model_params.yaml \
  -p use_sim_time:=true
```

🔴 **`use_sim_time` phải bật trong sim.** Tuổi quan sát và dấu thời gian đều lấy từ đồng hồ node; để sai thì tuổi tính ra là hiệu giữa giờ tường và giờ mô phỏng.

| Vào | Kiểu |
|---|---|
| `/uav/<id>/state/odometry_fused` | `nav_msgs/Odometry` — **nguồn pose duy nhất** (R4) |
| `/uav/<id>/perception/markers` | `uav_interfaces/MarkerObservation` |
| `/uav/<id>/perception/obstacles_local` | `uav_interfaces/ObstacleArray` *(P5.4, vắng là bình thường)* |
| `/uav/<id>/perception/target_track` | `uav_interfaces/TargetTrack` *(P5.5, vắng là bình thường)* |

| Ra | Kiểu |
|---|---|
| `/uav/<id>/world/semantic_landmarks` | `uav_interfaces/SemanticLandmarkArray`, frame `odom` |
| `/uav/<id>/world/obstacle_map_local` | `uav_interfaces/ObstacleArray`, frame `odom` |
| `/uav/<id>/world/target_state` | `uav_interfaces/TargetState`, frame `odom` |
| `/uav/<id>/world/mission_reference` | `geometry_msgs/PoseStamped`, latched |

### Bảng gắn cảm biến (`frames.*`)

Node **không tra TF2** cho phần gắn cảm biến, vì hiện **chưa ai phát TF tĩnh `base_link → camera_*`** (kiểm trong `uav_bringup/launch/sim.launch.py`). Nó đọc bảng tham số, và **in ra từng mount lúc khởi động** để một con số sai nhìn là thấy chứ không phải suy đoán.

Ba lớp chặn, vì đoán một phép gắn là cách chắc chắn nhất để dời **toàn bộ** bản đồ đi một hằng số:

1. Frame không có trong bảng → quan sát **bị loại**, cảnh báo kèm tên frame.
2. Mount được liệt kê nhưng thiếu `translation`/`rotation_rpy` → **ERROR, bỏ mount**. Gõ sai tên khoá trong YAML thì ROS 2 im lặng bỏ qua, nên "thiếu" phải là lỗi to chứ không được thành `(0,0,0)`.
3. Riêng `camera_down` có số dựng sẵn trong code — nhưng đó là **hình học của MÔ PHỎNG**. Rơi vào nhánh này thì node **WARN**, nói thẳng rằng bay thật phải tự cấu hình (R7).

Hai mount đang khai trong [`config/world_model_params.yaml`](config/world_model_params.yaml):

| Mount | `frame_id` | Nguồn phát ở frame đó |
|---|---|---|
| `camera_down` | `uav0/camera_down_optical` | `marker_detector_node` |
| `camera_front` | `uav0/camera_front_optical` | `obstacle_extractor_node` — **và `target_tracker_node` đi cùng đường**: nó chép nguyên `header.frame_id` của `ObstacleArray` sang `TargetTrack` (`tracking_frame_ = message.header.frame_id` → `track.header.frame_id = header.frame_id`), nên một mount phủ cả vật cản lẫn mục tiêu, không cần khai thêm |

⚠️ **`camera_front` cố ý KHÔNG có số dựng sẵn trong code.** Khác `camera_down`, con số của nó chưa từng được một phép đo độc lập nào đối chứng — nên không được tồn tại đường "khai thiếu vẫn chạy": thiếu `translation`/`rotation_rpy` là **ERROR bỏ mount**, chứ không WARN rồi đoán.

✅ **Phép đo nghiệm thu (cổng tổng P5 bước (b)) đã chạy và PASS 2026-08-16** — hộp biết trước toạ độ world, đối chiếu `/world/obstacle_map_local` ở **hai** khoảng cách (3 m và 5 m) rồi tách sai số thành **thành phần hằng** (= lỗi translation mount) và **thành phần tỉ lệ theo khoảng cách** (= lỗi rotation/scale). Phải tách vì quên hẳn mount chỉ gây lệch tổng 0,124 m — ngưỡng tổng 0,15 m kiểu S5 không bắt được. Kết quả: hằng `(−0,000, +0,007, +0,001)` m (ngưỡng 0,08/0,05/0,05) · dốc `(+0,000, −0,0008, −0,0007)` m/m (ngưỡng 0,03/0,02/0,02). Chạy lại: [`scripts/verify_camera_mount.sh`](../../scripts/verify_camera_mount.sh).

🪤 **Bẫy đã trả giá trong chính phép đo này:** hộp đặt **nổi** để viền trời (thiết kế S4) thì camera nhìn từ dưới **thấy cả mặt đáy** — điểm mặt đáy trải sâu từ mép trước tới mép sau, kéo **tâm bbox** sâu hơn mặt trước 0,14–0,16 m (đo được, co theo khoảng cách; `size.x` 0,327/0,288 m xác nhận). Bản probe đầu so tâm bbox với mặt trước → FAIL +0,19 m trông y hệt mount sai. Phân xử: so **mép gần của AABB** (`center − size/2` theo trục nhìn) — mép gần luôn là mặt trước, và mount sai thật thì tiêu chí này vẫn bắt được. Họ hàng với bài học *"cổng PASS/FAIL chưa chứng minh gì nếu chưa kiểm nó đo đúng đối tượng"*.

### QoS

`odometry_fused` và `/perception/markers` dùng Reliable, khớp publisher đang chạy thật. `/perception/obstacles_local` và `/perception/target_track` dùng **BestEffort** — không phải vì chúng ít quan trọng, mà vì **P5.4/P5.5 chưa chốt QoS**: một subscriber BestEffort khớp được cả publisher Reliable lẫn BestEffort, còn chiều ngược lại thì **không khớp và không có triệu chứng gì** — không lỗi, không cảnh báo, chỉ là một topic vắng mặt.

## Trạng thái

| Việc | Xong? |
|---|---|
| Chuỗi transform quang học → thân → `odom` (cả hai mount) | ✅ 15 test, đối chiếu trực tiếp `uav_perception::opticalToBody()` |
| Cộng bất định + sàn perception + sentinel | ✅ 15 test |
| Ghép cặp theo dấu thời gian, loại pose cũ | ✅ 11 test |
| Bản đồ có tuổi, quên theo cửa sổ | ✅ 13 test |
| Node chạy thật trong một tiến trình test | ✅ 17 test |
| **Cổng S5/S6 trong sim** | ✅ **PASS 2026-08-16** ([`scripts/verify_world_model.sh`](../../scripts/verify_world_model.sh), `uav0_nav`, marker 7 tại (1,2 · 0,8) m) — **S5:** landmark trong `odom` lệch **0,033 m** (ngưỡng 0,15; 150 mẫu, frame offset fused-truth ≈ 0,001 m) · **S6:** uncertainty A (VIO khoẻ) **0,108 m** → B (VIO mất tư cách, mux rơi về GPS) **0,901 m** = **×8,38** (cần ≥×3). Sai số landmark ở B là 0,195 m — nới đúng thiết kế, pose phía sau tệ hơn |
| **Hình học mount camera trước đối chiếu vật thật** | ✅ **PASS 2026-08-16** ([`scripts/verify_camera_mount.sh`](../../scripts/verify_camera_mount.sh), `uav0_track`, hộp 0,4×1,0×0,6 m ở 3 m và 5 m, n=200/phase) — residual hằng `(−0,000, +0,007, +0,001)` m · dốc `≤0,001` m/m (trừ x = 0,000) · frame offset ≈ 0 · sai số x mỗi phase **0,000 m** |

**Tổng: 71 test** trong package = 15 + 15 + 11 + 13 + 17 (phép cộng viết ra vì dòng này từng ghi sai thành 67). Mọi test phủ định đều kèm **đối chứng dương** — "không thấy gì" phải phân biệt được với "đường ống chết".

Bốn test tầng node của mount camera trước **nạp chính `config/world_model_params.yaml`** (đường dẫn tiêm qua CMake), nên gõ sai một con số trong YAML là test đỏ, không phải là bản đồ lệch âm thầm.

### 🟠 Còn mở

- **Sàn bất định vật cản/mục tiêu là chỗ giữ chỗ** — P5.4/P5.5 phải đo rồi thay, rồi đặt `obstacle_uncertainty_measured` / `target_uncertainty_measured` thành `true`. Còn `false` thì node **kêu WARN định kỳ**, không phải một lần rồi thôi.
- **`world/mission_reference` neo lại nếu node khởi động lại giữa chuyến bay** — v0.1 neo ở pose hợp lệ đầu tiên, nên restart trên không sẽ dời "nhà" lên trời. Chốt ngữ nghĩa ở hợp đồng **trước** khi P9 có consumer.
- **`TargetTrack.velocity` đang được hiểu là vận tốc *tương đối*** với máy bay (tham số `target_velocity_is_relative`, mặc định `true`, node in ra lúc khởi động). P5.5 chốt quy ước xong thì phải xác nhận lại.
- **Nếu P5.4/P5.5 phát ở frame `odom`** thay vì frame cảm biến, quan sát sẽ bị loại vì không có mount. Lúc đó phải bàn lại, không tự thêm.
- **Cấu hình đang theo từng drone:** `frames.camera_down.frame_id` trong YAML ghi cứng `uav0/...`. Mặc định trong code bám theo `uav_id`, nhưng giá trị YAML thắng — multi-drone sẽ cần một file cấu hình cho mỗi drone.
- **`world/mission_reference` v0.1 = pose hợp lệ đầu tiên** (điểm neo/cất cánh). `uav_mission` (P9) chưa có nên chưa ai định nghĩa được ngữ nghĩa thật; tham số `publish_mission_reference` tắt được.
- **Chưa nối vào `uav_bringup`** — `sim.launch.py` chưa khởi động node này (ngoài phạm vi P5.6).
