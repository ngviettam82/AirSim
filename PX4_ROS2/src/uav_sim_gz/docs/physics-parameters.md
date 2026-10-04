# Khung tham số vật lý — chỗ điền số của drone thật

> **Mục đích:** khi team phần cứng giao CAD và thông số motor, việc phải làm là **thay số**, không phải sửa cấu trúc. Tài liệu này liệt kê từng con số, nó nằm ở file nào, ai điền, và đo bằng cách nào.
>
> Cập nhật 2026-08-10 · Nguồn hiện tại: x500 của PX4 v1.15.4 (**số MƯỢN**)

---

## 0. Trạng thái: mọi số dưới đây đều là số mượn

Không con số nào trong bảng là của drone dự án. Chúng là của **x500** — một khung tham chiếu của PX4 — và chỉ có tác dụng khiến mô phỏng bay được. Mọi kết luận về lực đẩy, thời gian đáp ứng, biên điều khiển, thời gian bay **chỉ đúng với x500**.

> 🔴 **Không dùng bất kỳ số nào ở đây làm căn cứ cho bay thật.**

---

## 1. Bảng tham số — điền vào đây

### 1.1 Khối lượng & quán tính → `models/uav0_frame/model.sdf`

| Tham số | Thẻ XML | Giá trị mượn (x500) | Đơn vị | Đo bằng cách nào |
|---|---|---|---|---|
| Khối lượng toàn phần | `<inertial><mass>` | 2.0 | kg | **Cân** drone ở trạng thái sẵn sàng bay (đủ pin, đủ tải) |
| `ixx` | `<inertia><ixx>` | 0.02166666 | kg·m² | Xuất từ **CAD** sau khi gán vật liệu; hoặc thí nghiệm con lắc xoắn |
| `iyy` | `<inertia><iyy>` | 0.02166666 | kg·m² | như trên |
| `izz` | `<inertia><izz>` | 0.04 | kg·m² | như trên |
| Tích quán tính | `ixy`, `ixz`, `iyz` | 0 | kg·m² | CAD. Nếu drone đối xứng thì gần 0, **kiểm chứ đừng mặc định** |
| Vị trí trọng tâm | `<inertial><pose>` | 0 0 0 | m | CAD, hoặc thí nghiệm treo dây |

### 1.2 Động cơ & cánh quạt → `models/uav0_frame/model.sdf` (plugin `MulticopterMotorModel`)

| Tham số | Thẻ | Giá trị mượn | Đơn vị | Đo bằng cách nào |
|---|---|---|---|---|
| Hằng số lực đẩy kT | `motorConstant` | 8.54858e-06 | N·s²/rad² | **Bench test**: đo lực đẩy theo tốc độ quay, khớp `T = kT·ω²` |
| Hằng số mô-men kM | `momentConstant` | 0.016 | m | Bench test: đo mô-men phản lại theo lực đẩy |
| Tốc độ quay tối đa | `maxRotVelocity` | 1000.0 | rad/s | Từ **giới hạn dòng ESC** + KV motor + điện áp pin |
| Trễ tăng ga | `timeConstantUp` | 0.0125 | s | Bench test: đáp ứng bậc thang của tốc độ quay |
| Trễ giảm ga | `timeConstantDown` | 0.025 | s | như trên |
| Hệ số cản cánh | `rotorDragCoefficient` | 8.06428e-05 | — | Khó đo trực tiếp; giữ số tham chiếu và **ghi là chưa xác minh** |
| Mô-men lăn | `rollingMomentCoefficient` | 1e-06 | — | như trên |

### 1.3 Hình học & phân bổ điều khiển → `airframes/4100_gz_uav0`

| Tham số | Giá trị mượn | Đơn vị | Nguồn |
|---|---|---|---|
| `CA_ROTOR_COUNT` | 4 | — | đếm |
| `CA_ROTOR{0..3}_P{X,Y}` | ±0.13 / ±0.20–0.22 | m | **Đo wheelbase thật**, chia đôi |
| `CA_ROTOR{n}_KM` | ±0.05 | — | dấu theo chiều quay từng cánh |
| `MPC_THR_HOVER` | 0.60 | — | Bay thử: đọc ga trung bình khi hover ổn định |

### 1.4 Pin — **chưa mô hình**

x500 không mô hình sụt áp theo tải. Cần điền khi có pin thật: dung lượng (mAh), điện áp danh định, số cell, đường cong xả. Gazebo có `LinearBatteryPlugin`, chưa dùng.

---

## 2. Quy trình khi nhận được số thật

1. Điền vào bảng trên, **mỗi số ghi kèm nguồn** (CAD phiên bản nào, bench test ngày nào).
2. Sửa **duy nhất** `models/uav0_frame/model.sdf` — ba biến thể và mọi mô-đun cảm biến neo vào `base_link` nên không phải đụng.
3. Sửa `airframes/4100_gz_uav0` cho hình học rotor.
4. Chạy lại hồi quy M5 trên cả 3 world.
5. Chạy thí nghiệm đối chứng: **hover thrust** (ga hover đo được so với dự đoán từ khối lượng và kT, mục tiêu lệch < 5–10%).
6. Cập nhật `model-sources.md`: gỡ nhãn "số mượn".

---

## 3. 🔑 Phát hiện: SDF không mô hình được TRÔI cảm biến

Kiểm đặc tả SDF 1.8 (`/usr/share/sdformat12/1.8/lidar.sdf`, 2026-08-10):

| Loại cảm biến | `<noise>` hỗ trợ gì |
|---|---|
| **IMU** | `stddev` + **`dynamic_bias_stddev`** + **`dynamic_bias_correlation_time`** → mô hình được trôi |
| **Lidar** | **chỉ `mean` + `stddev`** → chỉ nhiễu trắng |
| Camera | như lidar |

**Hệ quả:** rangefinder và camera trong SDF **không thể có thiên lệch trôi theo thời gian**. Cộng với phát hiện tương tự ở GPS (nhiễu hardcode trong `sensor_gps_sim`, chỉ 0.2 m và là nhiễu trắng), kết luận thống nhất:

> **Mọi khiếm khuyết cảm biến mang tính TRÔI phải được mô hình ở tầng adapter của ta (P4), không phải trong SDF.**

Điều này thật ra **tốt hơn**: một chỗ duy nhất chịu trách nhiệm, bật/tắt được bằng ROS param lúc chạy, và nó test đúng thứ cần test — **logic hợp nhất của ta có chịu nổi cảm biến xấu không**. Cụ thể hoá ở P4:

| Nguồn | Cần tiêm gì | Vì sao |
|---|---|---|
| GPS | trôi tương quan 2–10 m + mất fix | PX4 chỉ cho nhiễu trắng 0.2 m — nhỏ hơn thực tế 10–50× và **sai bản chất** |
| Rangefinder | thiên lệch, mất tín hiệu trên bề mặt hấp thụ | SDF chỉ cho nhiễu trắng |
| "VIO" | nhiễu, trôi, độ trễ, **mất bám** | Nguồn hiện tại là **ground truth tuyệt đối** |

---

## 4. Nhắc lại điều dễ nhầm

**Cảnh đẹp không làm reality gap nhỏ lại.** Khoảng cách quan trọng nhất với UAV nằm ở khối lượng, quán tính, lực đẩy, khí động và đặc tính cảm biến — không nằm ở số lượng đa giác. Một world ảnh thật với thông số drone sai có gap **lớn hơn** một world trơn với thông số đúng.

Khung tham số này **không tự làm gap nhỏ đi**. Nó chỉ đảm bảo rằng khi có số thật thì việc phải làm là thay số, chứ không phải dựng lại.
