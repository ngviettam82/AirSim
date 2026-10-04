# `uav_localization`

Trả lời câu hỏi **"drone đang ở đâu?"** bằng **một** câu trả lời mà cả hệ thống tin.

> R4: planner và mission **chỉ được đọc** `/uav/<id>/state/odometry_fused`. Không node nào ngoài package này được tự chọn giữa GPS, VIO hay optical flow.

Kế hoạch chi tiết & cổng kiểm chứng → `.claude/plan/P4-localization.md`

---

## Hợp đồng của mọi nguồn định vị

Đây là ràng buộc trung tâm của package, và nó được **cưỡng chế bằng kiểu dữ liệu** chứ không bằng lời dặn:

> **Không thể phát ra một vị trí mà không kèm mức độ tin cậy của nó.**

`SourceChannel` nắm cả hai publisher. Adapter không cầm publisher odometry riêng, nên không có đường nào để phát pose "trần".

| Tình huống | Odometry | Status |
|---|---|---|
| Sai số nhỏ | ✅ phát | `is_valid=true`, `QUALITY_GOOD` |
| Sai số trung bình | ✅ phát | `is_valid=true`, **`QUALITY_DEGRADED`** — vẫn bay được nhưng đã nói rõ |
| Sai số quá lớn | ❌ **không phát** | `is_valid=false`, `QUALITY_BAD` |
| **Không tự biết sai số bao nhiêu** | ❌ **không phát** | `is_valid=false`, `QUALITY_UNKNOWN` |
| Nguồn hỏng / mất bám | ❌ không phát | `is_valid=false` + lý do cụ thể |

**Vì sao "không biết sai số" bị xử như không dùng được:** mux phải **cân** các nguồn với nhau. Một nguồn không nói được mình sai bao nhiêu thì không có gì để cân — nhận nó vào là mux đang đoán.

**Vì sao nguồn hỏng phải LÊN TIẾNG chứ không im lặng:** với node phía sau, "im lặng" và "chết" trông y hệt nhau. `publishUnavailable()` biến một sự cố âm thầm thành một sự kiện quan sát được.

## Ngưỡng chất lượng — một chính sách chung

`QualityThresholds` dùng chung cho mọi nguồn, để không adapter nào tự nghĩ ra định nghĩa "đủ tốt" của riêng mình.

| Trường | Mặc định | Nghĩa |
|---|---|---|
| `degraded_above_m` | 0.5 | trên mức này là suy giảm |
| `bad_above_m` | 2.0 | trên mức này không dùng được |
| `stale_after_sec` | 0.5 | quá lâu không có số đo mới |

## Kiểu dữ liệu: tái sử dụng, không tạo mới

Đã kiểm theo R19 — `uav_interfaces` **đã có đủ**:

| Dùng | Cho việc gì |
|---|---|
| `nav_msgs/Odometry` | pose + twist + **covariance 6×6** |
| `uav_interfaces/LocalizationStatus` | nguồn nào, chất lượng, hợp lệ, sai số, thời gian từ lần cập nhật cuối |

Không thêm message mới nào. Sai số 1-sigma được ánh xạ vào đường chéo covariance (x, y, z, yaw) để phía sau dùng được ngay theo chuẩn ROS.

### Cái gì không đo được thì phải đánh dấu là không biết

Covariance mặc định của ROS là **toàn số 0**, mà 0 nghĩa là *"chắc chắn tuyệt đối"*. Nguồn nào không đo hướng hoặc không đo vận tốc mà cứ để mặc định thì đang **tuyên bố một điều nó chưa hề đo** — đúng lỗi đã trả giá một lần ở `px4_external_odometry_node`.

| Trường hợp | Đánh dấu |
|---|---|
| Không có hướng (GPS) | `pose.covariance[35] = -1` |
| Không có vận tốc thân (GPS) | `twist.covariance[0] = -1` |
| Có vận tốc nhưng **chưa khai sai số** | `twist.covariance` giữ **0** (tri-state, xem cảnh báo dưới) |
| Có vận tốc + khai `twist_stddev` (mux khi đang hấp thụ offset) | `twist.covariance[0/7/14] = stddev²` |

`-1` ở phần tử đầu là quy ước ROS cho "không biết".

> 🔴 **Tri-state twist — ĐÃ CHỐT GIỮ (2026-08-21), và nay có helper:** giá trị `0` ở đây nghĩa là
> *"có twist, sai số CHƯA KHAI"* — **không phải** *"đo được sai số bằng 0"*. Mux đọc
> `twist.covariance[0] >= 0` làm tín hiệu `twist_known` (`localization_mux_node.cpp:168`), và test
> `AKnownTwistLeavesTheCovarianceUnmarked` khóa hành vi này **có chủ đích**.
> ➡️ **Consumer PHẢI đọc qua [`include/uav_localization/twist_reading.hpp`](include/uav_localization/twist_reading.hpp)**
> (`readTwistTrust` / `statedTwistStddev` — header-only, ROS-free), **cấm** đọc thô rồi coi `0` là
> "vận tốc hoàn hảo"; nhớ `covariance` là **phương sai**, `1-sigma = sqrt`. Hợp đồng đầy đủ + lý do
> giữ ba trạng thái thay vì đổi msg → [`docs/interface-contract-v0.1.md`](../../docs/interface-contract-v0.1.md) **§2.17**. Nguy hiểm nhất là hướng: `geometry_msgs/Quaternion` mặc định có `w=1.0`, tức **quaternion đơn vị** — một nguồn quên gán hướng trông y hệt một nguồn đo được "đang bay bằng, mũi chỉ hướng bắc".

> ⚠️ Rangefinder **không phải nguồn vị trí** — nó chỉ cho độ cao trên mặt đất. Nó không đi qua `SourceChannel` và không xuất hiện trong `active_source`. Đó là lý do enum `LocalizationStatus` không cần thêm giá trị nào.

## Độ cao trên mặt đất — vì sao phải bù nghiêng

**Vì sao cần topic riêng thay vì đọc `z`:** `z` của PX4 đo từ **gốc bộ ước lượng**, không phải từ **mặt đất**. Bay qua gò đất hay qua nóc nhà thì `z` gần như không đổi trong khi khoảng cách thật tới đất đổi hẳn. Ai thật sự cần độ cao trên mặt đất (optical flow, hạ cánh, safety bay thấp) phải đọc `/localization/range` — đây là nguồn duy nhất đo mặt đất trực tiếp.

Tia lidar đo **khoảng cách xiên** dọc trục −Z của thân. Drone nghiêng thì trục đó không còn chỉ xuống đất, cùng một độ cao thật mà tia phải đi xa hơn → đọc thẳng thành độ cao sẽ **báo dư**, và dư nhiều nhất đúng lúc drone đang cơ động.

`height = khoảng_cách × cos(góc_nghiêng)`, cos lấy từ phần tử (2,2) của ma trận quay.

| | Đo được |
|---|---|
| Đậu dưới đất (thăng bằng) | tia thô 0.13994 m vs quy đổi 0.13991 m → lệch **0.03 mm** |
| Một chuyến bay | 2999 mẫu, **0 mẫu bị từ chối**, độ cao 0.129 → 2.639 m |
| Lúc nghiêng tới **23.1°** | độ cao dao động **7.8 cm**; không bù sẽ dư **23 cm** |

**Quá 40° thì từ chối chứ không bù.** Lúc đó tia chạm đất ở điểm cách xa vị trí drone nên nó không còn mô tả mặt đất bên dưới — bù cosine chỉ là tô son cho một số đo sai.

⚠️ **Bẫy:** `geometry_msgs::msg::Quaternion` khởi tạo mặc định có `w=1.0`, tức **quaternion đơn vị** = "hoàn toàn thăng bằng". Node nào quên gán tư thế sẽ được coi là bay bằng mà không có cảnh báo nào. Adapter chặn bằng cách **từ chối phát khi không có tư thế mới** (phát `range = +inf` — giá trị ngoài khoảng là cách chuẩn của `sensor_msgs/Range` để nói "đừng dùng số này").

⚠️ **Tư thế lấy từ `/state/odometry_raw`, KHÔNG lấy từ `odometry_fused`.** Fused là đầu ra của chuỗi định vị mà node này nằm trong đó; đọc ngược lại là **đóng một vòng phản hồi** — sai số sẽ tự nuôi chính nó mà không ai nhìn thấy.

## GPS — mốc quy chiếu và cái bẫy "cùng đơn vị, khác gốc"

Adapter đọc `sensor_msgs/NavSatFix` do backend phát, **không** đụng `px4_msgs` (R1) — nhờ vậy node này y hệt nhau giữa sim và drone thật. Mốc (`/state/gnss_origin`) publish **latched** (`transient_local`) nên node khởi động muộn vẫn nhận được.

GPS cho **lat/lon**, phần còn lại của hệ nói bằng **mét trong frame `odom`**. Đổi được thì phải có mốc, và **chọn mốc nào là câu hỏi sát an toàn chứ không phải chi tiết vặt**: nếu adapter tự lấy điểm fix đầu tiên làm mốc, nó nằm trong frame riêng của nó, và mux sẽ thấy GPS với VIO lệch nhau một hằng số **mãi mãi mà không nguồn nào sai cả**.

→ Mốc lấy từ **gốc EKF2** (`ref_lat/lon/alt`), do backend phát latched trên `/state/gnss_origin`. Đó cũng là gốc mà `odometry_raw` và TF `odom→base_link` đang neo vào. Gốc reset thì cả hai nhảy **cùng nhau**.

**Đo thật:** lệch so với ground truth Gazebo chỉ **(−0.025, +0.044) m** — nếu chọn sai mốc thì con số này là hàng mét.

Phép chiếu là **azimuthal equidistant**, chép đúng công thức `MapProjection` của PX4 (cùng bán kính Trái Đất 6371 km) để một mét ở đây bằng một mét của bộ ước lượng.

> **Đã cân nhắc và loại:** `robot_localization/navsat_transform_node` làm đúng việc này, nhưng nó tự quản mốc theo cách riêng — trong khi thứ ta cần là **bám đúng mốc của EKF2**. Đổi lấy một phụ thuộc nặng để rồi vẫn phải ép mốc là lỗ. Phép chiếu tự viết gọn trong ~40 dòng và ghim được bằng test.

## Bộ tiêm trôi — vì sao bắt buộc phải có

| | Trong sim | Ngoài đời |
|---|---|---|
| Sai số GPS | nhiễu **trắng** σ=0.2 m, hardcode trong PX4 | **trôi có tương quan** 2–10 m |

Khác biệt không nằm ở độ lớn mà ở **trí nhớ**: nhiễu trắng thì lọc trung bình là hết, nên mux **không bao giờ cần chuyển nguồn** và ta chẳng chứng minh được gì. Đo thật xác nhận đúng vậy — autocorrelation của sai số khi không tiêm là **−0.020**.

Trôi mô hình bằng **Gauss-Markov bậc 1** (`τ`, `σ` gắn ROS param).

🔴 **Mặc định TẮT**, và `real.launch.py` **không được phơi tham số này ra**. Khi bật, node kêu `WARN` và `detail` trên topic status ghi rõ `"simulated drift active"` — nhìn topic là biết cảm biến đang bị làm giả.

⚠️ **Trôi cố ý KHÔNG làm phồng `eph` báo về.** Máy thu thật vẫn báo mình chính xác trong lúc đang trôi. Nghĩa là phát hiện nó **phải đến từ việc bất đồng với nguồn khác**, không phải từ việc đọc con số tin cậy — đó là bài toán đúng cho P4.6/P4.7. Đo thật: sai số thật ~5 m trong khi status vẫn báo 0.9 m.

## VIO — nguồn hoàn hảo phải được làm cho hỏng

Trong sim, "VIO" là **ground truth tuyệt đối** và khai `covariance` **toàn số 0**. Nhận nguyên xi thì nó thắng mọi so sánh và mux trở thành đồ trang trí. Nên adapter làm ba việc:

1. **Khai một sai số tin được** (`nominal_position_stddev`, mặc định 0.10 m) thay cho số 0 vô nghĩa.
2. **Suy giảm có chủ đích** (`PoseDegrader`: nhiễu trắng + trôi tương quan) — mặc định TẮT.
3. **Mất bám thì lên tiếng**: watchdog phát hiện dữ liệu ngừng tới và báo không hợp lệ. Đo thật: **0.26 s** (ngưỡng 0.30 s). Lúc mất bám nó **xoá cả hàng đợi**, để pose cũ không rò ra sau.

> ⚠️ **Bù lệch thân, 0.24 m — không bù là hai nguồn cãi nhau vĩnh viễn.** Gazebo báo pose của `base_footprint` (gốc model), còn GPS và EKF2 nói về `base_link` cao hơn **0.24 m**. Không bù thì VIO và GPS luôn lệch đúng chừng đó theo phương đứng, và mux chuyển nguồn sẽ nhảy 0.24 m — đúng lúc hạ cánh. Lệch được xoay theo tư thế chứ không cộng thẳng vào z.
>
> Chỉ bù lever-arm cho **vị trí**. Thành phần vận tốc ω×r **cố ý bỏ qua**: ở tốc độ xoay của các bài bay này nó không đáng kể so với sai số đã khai (0.10 m). Nếu sau này bay cơ động mạnh hơn thì phải xem lại chỗ này.

**Trễ và mất bám nằm ở node, không nằm trong `PoseDegrader`.** Bộ suy giảm chỉ làm méo *con số* (nhiễu + trôi); còn trễ và ngừng dữ liệu là chuyện *dòng message*, chỉ mô phỏng đúng được ở chỗ có hàng đợi và timer. Node phát theo timer 100 Hz để một cơ chế phục vụ cả hai: xả hàng đợi theo độ trễ đặt trước, và watchdog phát hiện dữ liệu ngừng tới.

## Mux — chọn nguồn là một việc, giữ pose liền mạch là việc khác

Hai việc này hay bị gộp làm một, và gộp là hỏng.

**Chọn nguồn** (`SourceSelector`, tách riêng để test được): nguồn nào sai số nhỏ nhất thì thắng, nhưng đối thủ phải **tốt hơn 30 %** và **giữ được 1 giây** mới được đổi. Ngược lại, nguồn đang dùng mà **chết thì đổi ngay lập tức** — hysteresis chống rung, không phải cớ để bay tiếp trên thứ đã hỏng.

**Giữ liền mạch:** REP-105 bắt frame `odom` phải **liên tục**. Đổi nguồn mà để pose nhảy thì bộ điều khiển đọc thành "vừa bị đẩy đi mấy mét" và giật lại. Nên lúc chuyển, chênh lệch giữa hai nguồn được **hấp thụ vào một độ lệch**, rồi độ lệch đó tan dần (0.5 m/s).

**Mất sạch nguồn: neo được giữ trong một ân hạn, không vứt ngay.** Trước đây chỉ cần **một tick** không nguồn nào hợp lệ là mux vứt pose đang giữ — nguồn quay lại thì pose được phép **nhảy** trọn lượng bất đồng, đúng lúc hệ vừa phục hồi. Nay neo sống thêm `source_timeout_sec` **tính từ pose cuối cùng thật sự phát ra dây**; chớp nháy ngắn thì cơ chế hấp thụ làm việc của nó và pose liền mạch.

Quá ân hạn thì neo bị vứt như cũ, **có chủ đích**: hấp thụ sau một lần mất dài là **che giấu** việc ta không còn biết mình ở đâu. Tổng thời gian "mù" tối đa trước khi pose được phép nhảy là **2 × `source_timeout_sec`** (một lần cho nguồn hết hạn, một lần cho neo hết hạn) — vì vậy grace **không có núm riêng**: một tham số thứ hai chỉ thêm một cặp ràng buộc phải canh mà chưa có nhu cầu nào đòi.

| | Đo được (test node, `source_timeout_sec` 0.5) |
|---|---|
| Nguồn im 1 tick rồi về, trong lúc im đã đi 1,00 m | pose nhảy **< 1e-6 m** |
| Nguồn im quá ân hạn (neo bị vứt sau **0,52 s** không phát pose) | pose nhảy **đúng 1,00 m** — không che |

⚠️ Cái giá phải biết: trong lúc hấp thụ, `odometry_fused` **cố ý** tụt sau nguồn (tối đa quãng đường bay trong ân hạn) rồi trượt về với `continuity_decay_rate_mps`. Điều đó được khai báo — `twist.covariance` khai sai số vận tốc trong suốt cửa sổ trượt, `detail` ghi độ lệch còn lại.

**Một timer chung cho mọi nguồn, KHÔNG cập nhật trong callback của từng cảm biến.** Cập nhật theo callback thì đầu ra chạy theo nhịp của nguồn **nhanh nhất**, và mỗi lần chọn lại đang **so số đo ở những thời điểm khác nhau** — nguồn vừa về thì mới, nguồn kia thì cũ. Timer riêng giữ mọi nguồn được cân ở cùng một mốc thời gian, với nhịp ra ổn định cho phía sau.

**Đo thật:** hai nguồn lệch **1.22 m** (do bật trôi GPS) mà pose hợp nhất **nhảy đúng 0.0000 m**. Chiều ngược lại: lệch 0.52 m → nhảy 0.0000 m.

> 🐛 **Lỗi bắt được bằng đo, không bằng đọc code:** ban đầu tôi cho độ lệch tan **trước khi** áp nó, nên ngay nhịp chuyển đã hụt mất một bước tan. Đo ra 0.08 m thay vì 0. Đảo thứ tự là hết.

Độ lệch **không** được cộng vào covariance: nó là **sai số đã biết**, không phải sai số ngẫu nhiên. Cộng vào là khai báo nhầm một thiên lệch xác định thành nhiễu, và sẽ làm `odometry_fused` bị coi là không dùng được đúng lúc bộ điều khiển cần nó nhất. Nó được nêu ra ở `detail` và do health cảnh báo.

## Optical flow — không phải nguồn vị trí

Nó đo mặt đất trôi qua nhanh thế nào, **không** đo mặt đất ở đâu. Nên nó phát `TwistWithCovarianceStamped` trên `/localization/flow_velocity` và **không đi qua `SourceChannel`** — cùng lý do rangefinder không đi qua.

**Bù xoay là chỗ dễ sai nhất:** drone lắc tại chỗ quét mặt đất qua camera **y hệt** drone bay ngang. Không trừ phần xoay đi thì máy bay rung tại chỗ sẽ tự báo là đang trôi.

**Đo thật (bay 1.2 m, 1 m/s, nền `rich`):** thang **1.002**, sai số **8.0 %** (ngưỡng 15 %). Lúc treo: **0.020 m/s** — không bịa ra chuyển động. Nền `flat`: **6/437 mẫu hợp lệ, 0 mẫu trong lúc bay** → từ chối chứ không đoán.

**Chỉnh bộ bám (`search_window_px` 31, `pyramid_levels` 4):** LK chỉ bám nổi dịch chuyển cỡ **nửa cửa sổ × 2^levels** pixel. Ảnh về thưa nên mỗi khung mặt đất dịch rất xa — cửa sổ nhỏ hoặc ít tầng kim tự tháp là mất bám ngay khi bay nhanh. Hai dấu `roll_rate_sign` / `pitch_rate_sign` là **tham số đo bằng thực nghiệm, không suy ra được**: cách gắn camera quyết định chúng.

> 🔑 **Hai bài học đắt:**
> 1. **Kiểm tra bám ngược (forward-backward) là bắt buộc.** Không có nó, LK trả về vài điểm bám sai "rất tự tin", median bị kéo về gần 0 → flow báo **−0.011 m/s trong khi drone đang bay 1.2 m/s**. Với position hold thì đó là kiểu hỏng chết người: "đứng yên" trong lúc đang trôi.
> 2. **"Sai số 2×" ban đầu là do phép đo của tôi, không phải code.** Bài smoke bay ở 3 m — quá tầm rangefinder (~2.9 m) — nên flow chỉ hợp lệ lúc leo/hạ, đúng pha vừa nghiêng vừa đổi cao độ. Bay đúng chế độ (thấp, ngang, đều) thì thang ra 1.002.

## Health — đưa bằng chứng, không tự chữa

`localization_health_node` **không sửa gì cả**. Nó chỉ quan sát rồi publish bằng chứng (`/state/localization_health`, `/diagnostics/localization`); quyết định *"mất định vị thì làm gì"* là việc của `uav_safety`. Gộp hai việc lại là để một node vừa chấm điểm vừa hành động theo điểm nó tự chấm.

| Kiểm tra | Bắt được gì | Vì sao đặt như vậy |
|---|---|---|
| `classifyRate` | fused ngừng hoặc chậm | chấm theo **tỉ lệ** so với tần số kỳ vọng, không theo con số tuyệt đối |
| `classifyJump` | bước nhảy không máy bay nào bay nổi trong `dt`, **sau khi trừ nhiễu nguồn tự khai** | nhảy pose là lỗi **bộ ước lượng**, không phải chuyển động thật — nhưng nhiễu cũng không phải |
| `classifyDisagreement` | **nguồn trôi trong khi vẫn khai sai số nhỏ** | đọc con số tin cậy KHÔNG bắt được ca này — chỉ bất đồng với nguồn khác mới bắt được |

Vài lựa chọn cố ý:

- **`floor_m` = 0.2 m** chặn một nguồn quá tự tin (σ→0) làm tỉ số `gap/σ` bùng nổ thành báo động giả.
- **Nguồn im lặng chỉ là `Warn`, không phải `Error`.** Một nguồn tạm không có mặt là chuyện bình thường và hợp lệ (bay trong nhà thì GPS im là đúng). Chỉ khi **mux mất sạch nguồn** mới thành `Error`.
- **Sai số không khai báo thì health không chấm.** `SourceChannel` đã từ chối nguồn đó ngay từ đầu; báo lần thứ hai chỉ là nhiễu trong diagnostics.

### 🔴 Bộ kiểm liên tục — sửa 2026-08-25, và vì sao

Luật cũ là `step > max_speed·dt + tolerance` — một biên **thuần động học** áp lên một dòng **do nhiễu chi
phối**, trong khi chính bản tin đi kèm đã tự khai `position_uncertainty`. Trên `uav0` (mux rơi về GPS, xem
[`ops-playbook.md`](../../docs/ops-playbook.md) §4) nó nổ vì nhiễu bình thường:

| | |
|---|---|
| Cú "nhảy" duy nhất của chuyến đo (bag `uav0_20260825_073235Z`) | **2,217 m / 100 ms** |
| Vạch cũ `20·0,1 + 0,2` | **2,200 m** ⇒ vượt đúng **17 mm** |
| Nguồn tự khai | `position_uncertainty = 0,900 m` |
| Vạch mới `+ 2σ` | **4,000 m** ⇒ **0/661** cặp nổ, cú xấu nhất còn dư **1,783 m** |
| Dịch chuyển tức thời 10 m | vẫn nổ dưới **mọi** biến thể trên |

Hai điều bắt buộc đi kèm, cả hai đều là O3 (*không đo được ≠ OK*):

- σ **không khai** (`-1`, có thật: 12/674 mẫu) hoặc **vô lý** (`> jump_max_sigma_m`, mặc định 5 m) ⇒
  `CannotJudge`, báo **Warn** với đếm `unjudged` — **không** phải OK. Không có nắp trên σ thì một nguồn
  khai 50 m sẽ mua được sự im lặng cho mọi bước, tức tắt bộ kiểm đúng lúc cần nó nhất.
- `dt` lấy từ **`header.stamp`**, không phải `now()`. Liên tục là thuộc tính của **dữ liệu**; đo bằng giờ
  callback thì một lần giao dồn làm `dt→0` và biên co về `tolerance`. Ghim bằng
  `test_localization_health_node.cpp` — case đó tự kiểm luôn tiền đề của mình và báo *FAILED TO MEASURE*
  nếu máy chậm tới mức hai đồng hồ không còn khác nhau.

⚠️ Hệ quả phải nhớ: **mọi lần đọc `LOCALIZATION_JUMP` trước 2026-08-25 mô tả luật cũ.**

## Trạng thái

| Task | Việc | Xong? |
|---|---|---|
| P4.1 | Khung package + hợp đồng nguồn | ✅ 11 test |
| P4.2 | `rangefinder_adapter_node` | ✅ 11 test + bay thật |
| P4.3 | `gps_adapter_node` + mô hình trôi | ✅ 17 test + đo thật |
| P4.4 | `vio_adapter_node` + bộ suy giảm | ✅ mất bám báo trong 0.26 s |
| P4.5 | `optical_flow_adapter_node` | ✅ 11 test + bay thật, 8.0 % |
| P4.6 | `localization_mux_node` | ✅ 9 test + bước nhảy 0.0000 m |
| P4.7 | `localization_health_node` | ✅ 9 test |
| P4.8 | Nối external odometry (nợ #8) | ✅ **bay trong nhà PASS 3/3, không GPS** |

**Tổng (đo bằng `colcon test-result` 2026-08-24): 103 case / 11 target**, 0 lỗi (target thứ 11: `test_twist_reading` — helper tri-state P8.0b; +1 case 2026-08-24 ghim `+inf`/`NaN` không lọt ra `twist.covariance`, xem `docs/package-status.md` §5). Hồi quy: **3/3 ngoài trời và 3/3 trong nhà**.
