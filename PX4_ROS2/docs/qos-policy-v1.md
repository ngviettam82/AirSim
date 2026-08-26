# Chính sách QoS — v1 (bản chính thức)

> **Vì sao file này tồn tại.** QoS là quyết định #2 của cửa P0 (`.claude/plan/overviewPlan.md` §1) và
> đã nằm ở mục *"chờ chốt"* của `.claude/memory.md` §5 từ đó tới nay — **không
> phải vì chưa biết dùng gì**, mà vì chưa ai viết ra thành một nơi. Hệ thống đã chạy ổn định với đúng
> bộ luật dưới đây suốt P2→P10; file này **chép lại HIỆN TRẠNG đã được kiểm chứng bằng bay thật**, không
> đề xuất gì mới.
>
> ⚠️ **R0:** QoS sai không báo lỗi — nó làm topic **im lặng hoàn toàn**. Đó là lý do bảng này phải là
> một tài liệu, không phải kiến thức truyền miệng.
>
> Soạn 2026-08-24 từ code đang chạy (không phải từ trí nhớ). Trạng thái: **✅ chủ dự án KÝ 2026-08-24 — đang hiệu lực.**

---

## 1. Luật gốc — bốn nhóm, không có nhóm thứ năm

| Nhóm | Reliability | Durability | History | Ai dùng |
|---|---|---|---|---|
| **A. `/fmu/*` (PX4 uXRCE-DDS)** | `BEST_EFFORT` | `TRANSIENT_LOCAL` | `KEEP_LAST(5–10)` | **chỉ** `uav_px4_backend` |
| **B. Cảm biến / luồng dữ liệu tần số cao** | `BEST_EFFORT` (`SensorDataQoS`) | `VOLATILE` | `KEEP_LAST(5)` | ảnh, depth, lidar, odometry thô |
| **C. Lệnh & luồng điều khiển** | `RELIABLE` | `VOLATILE` | `KEEP_LAST(10)` | `control/cmd_*`, `command_selected`, trajectory stream |
| **D. TRẠNG THÁI chốt-giữ (latched)** | `RELIABLE` | **`TRANSIENT_LOCAL`** | `KEEP_LAST(1)` | xem §3 |

🔑 **Phép thử để chọn nhóm:** *"một subscriber đến MUỘN có cần biết giá trị hiện tại không?"*
Có → **D**. Không, và mất một bản tin là vô hại → **B**. Không, nhưng mất một bản tin là mất một lệnh
→ **C**.

## 2. 🔴 Nhóm A — sai một nét là KHÔNG NHẬN ĐƯỢC GÌ, mà không có lỗi

`/fmu/out/*` do uXRCE-DDS Agent publish với `BEST_EFFORT` + `TRANSIENT_LOCAL`. Subscriber khai
`RELIABLE` sẽ **không bao giờ khớp** publisher đó — và ROS2 **không báo lỗi**, topic chỉ đơn giản là
im. Đây là bẫy đắt nhất của cả nhóm A.

Hiện thực: cả 5 node của backend dựng cùng một `px4_qos` (`best_effort()` + `transient_local()`) —
`px4_state_adapter_node.cpp:56` · `px4_command_gateway_node.cpp:69` ·
`offboard_session_manager_node.cpp:45` · `px4_frame_bridge_node.cpp:33` ·
`px4_external_odometry_node.cpp:61`.

> **R1 giữ nguyên hiệu lực:** không package nào ngoài `uav_px4_backend` được đăng ký `/fmu/*` bằng code.
> Công cụ dòng lệnh (`ros2 topic hz/echo`) trong script cổng thì hợp lệ — R25.

## 3. Nhóm D — danh sách ĐẦY ĐỦ các topic chốt-giữ (derive từ code 2026-08-24)

Đây là nhóm dễ sai nhất khi thêm topic mới, nên liệt kê hết:

| Topic | Node phát | Vì sao phải latch |
|---|---|---|
| `/uav/<id>/control/authority` | `control_authority_manager_node` | ai đang cầm lái là trạng thái, không phải sự kiện |
| `/uav/<id>/mission/status` | `mission_executor_node` | mission đang ở bước nào |
| `/uav/<id>/safety/state` | `safety_supervisor_node` | subscriber đến muộn phải biết ngay đang có violation |
| `/uav/<id>/state/system_health` | `diagnostics_node` | đèn go/no-go |
| `/uav/<id>/state/estimator_source` | `localization_mux_node` | nguồn định vị đang thắng (`KEEP_LAST(10)`) |
| `/uav/<id>/planning/trajectory` | `navigator_action_server_node` | kế hoạch đang bay + bản thu hồi |
| GNSS datum | `px4_state_adapter_node` · `gps_adapter_node` | gốc toạ độ phát một lần |

🪤 **Bẫy đi kèm, đã trả giá thật (P10):** `TRANSIENT_LOCAL` nghĩa là **`ros2 topic echo --once` có thể
đọc được mẫu chốt-giữ của một node ĐÃ CHẾT** và tưởng hệ thống còn khoẻ. → Đọc đèn go/no-go **phải** qua
[`scripts/preflight_light.sh`](../scripts/preflight_light.sh) (nó kiểm **tuổi** mẫu), không dùng `echo`
trần. Chi tiết: [`interface-contract-v0.1.md`](interface-contract-v0.1.md) §2.20.

## 4. Quy tắc khi thêm topic mới

1. Chọn nhóm bằng phép thử §1; **không sáng tác profile thứ năm**.
2. Vào nhóm **D** thì thêm một hàng vào bảng §3 **cùng lượt** (R35 — phát biểu đối ngoại phải sửa cùng
   lượt với code).
3. Publisher và subscriber phải **cùng** reliability, nếu không topic im lặng — không có lỗi nào báo.
4. Đường ghi hộp đen (`uav_observability`) mặc định **`BEST_EFFORT`** bất kể nhóm gốc: nguyên tắc **O1**
   *(observability hỏng ⇒ mất bằng chứng, KHÔNG được mất chuyến bay)* cấm hộp đen tạo backpressure lên
   luồng bay. Ngoại lệ per-topic khai trong `config/observability_params.yaml`.

## 5. Cái này KHÔNG chốt gì về sim-vs-real

QoS **giống hệt nhau** ở sim và real (R7) — khác biệt chỉ nằm trong launch file. Không có núm QoS nào
được phép chỉ tồn tại ở một bên.

---

*Nguồn số liệu: đọc thẳng code ngày 2026-08-24 (66 chỗ `QoS(10)`, 17 `SensorDataQoS`, 14 chỗ
`transient_local`). Bằng chứng vận hành: M5 3/3 và toàn bộ cổng P2→P10 chạy trên đúng bộ luật này.*
