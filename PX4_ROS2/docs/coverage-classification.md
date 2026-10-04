# Phân loại dòng chưa phủ — cổng `G-SIM S3`

> **Cổng này KHÔNG hỏi phần trăm.** Nó hỏi đúng một câu: *"còn dòng nào chưa ai nhìn tới không?"*
>
> Lý do đã trả giá và ghi ở [`../CLAUDE.md`](../CLAUDE.md) §5: `peakAcceleration()` báo **0,897 m/s²**
> cho một gia tốc thật **55,7 m/s²**. **Một con số an toàn nói dối nguy hơn không có con số nào.** Một
> tỉ lệ phủ gộp chung dòng khởi tạo tầm thường với nhánh cắt của safety — 90% nghe rất yên tâm trong khi
> 8% thiếu có thể **toàn bộ** là đường cứu mạng. Nên dự án **không đặt ngưỡng phần trăm, vĩnh viễn**.
>
> **Khai báo:** [`../config/coverage_classification.tsv`](../config/coverage_classification.tsv)
> **Cổng tự kiểm:** `bash scripts/check_coverage_classified.sh`
> **Nguồn đo:** `gate_logs/coverage/uncovered_by_file.txt` (do `scripts/measure_coverage.sh` sinh)

---

## 1. Năm loại, và điều kiện của từng loại

| Loại | Nghĩa | Bằng chứng bắt buộc |
|---|---|---|
| **B-FLIGHT** | Không tới được bằng `ctest`, nhưng **đã được chứng minh bằng một cổng bay có tên** | Phải **nêu đích danh cổng**. Đây là kiến trúc của dự án chứ không phải lời bào chữa: logic nằm ở các thư viện ROS-free (đã unit-test), tầng node là phần **đấu dây**, và đấu dây được chứng minh bằng **bay nó** |
| **B-ENTRY** | Điểm vào tiến trình (`rclcpp::init/spin/shutdown`) | Chạy trong mọi lần launch của mọi cổng bay |
| **B-DIAG** | Bảng ánh xạ enum→chuỗi, nhánh `default:` phòng hờ, dòng chỉ để log | Một chuỗi sai **không làm máy bay dịch chuyển** |
| **A-TESTED** | Sát an toàn, và **đã có test viết trong đợt này** | Tên test |
| **D-BOUNDARY** | **Không thể phủ trong mô phỏng, chấm hết** | Phải trích dẫn một mục `B-nn` trong [`sim-boundary-statement.md`](sim-boundary-statement.md) **và** có chữ ký ở §4 dưới đây |

🔴 **`B-FLIGHT` là loại dễ bị lạm dụng nhất.** Nó chỉ hợp lệ cho **hàm dựng node, đấu pub/sub, và các
callback chính** — những thứ *bắt buộc* phải chạy thì chuyến bay mới xảy ra. Nó **KHÔNG** hợp lệ cho
**nhánh lỗi** nằm bên trong một hàm vốn đã được phủ: chuyến bay đi qua hàm đó không có nghĩa nó đi qua
nhánh xử lý sự cố. Nhánh lỗi sát an toàn phải là **A-TESTED**.

## 2. Cổng tự canh chính nó

`check_coverage_classified.sh` đỏ khi:

1. **Chưa đo** — không có `uncovered_by_file.txt`.
1b. 🆕 **Phép đo cũ hơn `src/`** (2026-08-26). Cổng `G-SIM` chỉ soi tuổi của `S3.ok`, nên một bản phân
   loại **viết mới đè lên một phép đo cũ** vẫn ra xanh. Suýt xảy ra: lượt đo **01:14** liệt kê 225 dòng
   chưa phân loại, còn **40 case** phủ một phần chúng được viết lúc **04:41–04:59**. Phân loại trên đó
   sẽ khai lý do cho những dòng **nay đã có test** — và chính các luật ấy rồi sẽ không khớp dòng nào,
   tức rơi thẳng vào luật 3 bên dưới.
1c. 🆕 **Không có dấu mốc nguồn.** `measure_coverage.sh` ghi `source_epoch.txt` **trước khi build**, và
   cổng so với dấu đó chứ không so mtime của file kết quả. Lý do: một lượt đo ~20 phút bắt đầu **trước**
   một lần sửa `src/` sẽ kết thúc **sau** nó, nên mtime của kết quả trẻ hơn bản sửa và luật 1b bị qua
   mặt. Thứ cần so là **nguồn mà nhị phân được dịch ra**, không phải lúc file được ghi.
2. **Còn dòng chưa phân loại** — in ra file nào, bao nhiêu dòng.
3. 🔑 **Có luật không khớp dòng nào** (*stale declaration*). Đây là nửa quan trọng bị bỏ quên ở hầu hết
   bản kiểm kê: một bản phân loại **âm thầm ngừng phủ** thứ nó tuyên bố khi code đổi, và không ai biết.
   Luật chết ⇒ cổng đỏ, buộc phải nhìn lại.

Mọi luật đều **bắt buộc có trường bằng chứng khác rỗng** — không viết được lý do thì không phân loại được.

## 3. Những gì đợt phân loại này TÌM RA

Việc phân loại không phải thủ tục hành chính. Nó lộ ra bốn chỗ **lõi sát an toàn chưa từng được ctest
chạm tới**, và cả bốn đã được viết test trong cùng đợt:

| Chỗ | Vì sao đáng lo | Test đã thêm |
|---|---|---|
| `frame_conversions.cpp` — `nedToEnuVector` · `enuToNedArray(Vector3)` · `fluToFrdArray` · `yawRateEnuToNed` | **Đường vận tốc và tốc độ xoay** của phép đổi hệ toạ độ R5. Các test cũ chỉ phủ đường **vị trí**. Sai dấu vận tốc nguy hiểm y hệt sai dấu vị trí | `test_frame_conversions` 13 → **22** |
| `nav_goal_broker.cpp` | **Chưa hề có unit test riêng.** Nhiệm vụ duy nhất của lớp này là giữ bất biến *"tối đa MỘT goal navigator, mãi mãi"*, và các nhánh tồn tại **chỉ để sống sót qua callback lạc/muộn** chưa từng chạy | `test_nav_goal_broker` **0 → 13** |
| `local_avoidance.cpp` | Các nhánh **từ chối** (costmap không hợp lệ, vị trí không hữu hạn) — chuỗi lý do mà cả stack ghi log và chấm cổng dựa vào | `test_local_avoidance` 23 → **25** |
| `failsafe_policy.cpp` | Hai đường phát hiện offboard **cùng lúc**; leo thang HOLD→INHIBIT **khi còn đang engage**; điều kiện clear của battery | `test_failsafe_policy` 94 → **97** |

🔴 **Và nó lộ một nghi vấn lỗi sản phẩm** (chưa vá, chờ chủ dự án quyết): `Costmap` suy **độ tươi** từ
*"có vật cản nào được GHI vào bản đồ không"* thay vì *"có bản tin nào TỚI không"*. Đo được: hai vật cản
**tươi nguyên** nhưng nằm ngoài dải bay đọc thành `map_age = inf`, *"no obstacle input has ever
arrived"* — **y hệt perception đã chết**. Chi tiết ở `.claude/session-notes.md` / `memory.md` §7.

## 4. Chữ ký cho các mục `D-BOUNDARY`

Một mục `D-BOUNDARY` nghĩa là: **mô phỏng này không bao giờ phủ được nó**, và dự án chấp nhận điều đó
một cách có ý thức. Nó **không** phải "không sát an toàn" — nó là một **lỗ hổng được ghi nhận**.

| Mục | Dòng | Ranh giới | Chữ ký |
|---|---|---|---|
| `uav_localization/src/optical_flow_adapter_node.cpp` | 157 | **B-03** — PX4 v1.15.4 ship một `CMakeLists.txt` **rỗng** cho plugin optical flow ⇒ **không tồn tại nguồn dữ liệu**. Node chưa từng chạy với dữ liệu thật, và không thể test ở đây. Chặn bởi P11 | ✅ **CHỦ DỰ ÁN KÝ 2026-08-26** |

🔴 **Cổng S3 chỉ ghi `S3.ok` khi mọi dòng đã được phân loại.** Ô chữ ký trên là điều kiện để loại
`D-BOUNDARY` được coi là một phân loại hợp lệ chứ không phải một chỗ bỏ trống được sơn lại.

> ✅ **Chữ ký 2026-08-26 — ghi rõ nó có nghĩa gì và KHÔNG có nghĩa gì.** Chủ dự án ký nhận `B-03` là một
> **lỗ được ghi nhận**: mô phỏng này không bao giờ phủ được 157 dòng đó, và dự án chấp nhận điều ấy một
> cách có ý thức. 🔴 Nó **không** phải lời tuyên bố "node này an toàn" hay "không sát an toàn" — nó chỉ
> nói *sim không phát biểu được gì về node này*. Điều kiện gỡ: P11 cấp một nguồn optical flow thật (phần
> cứng hoặc plugin thay thế); tới lúc đó `optical_flow_adapter_node` **không được dựa vào** trong bất kỳ
> lập luận an toàn nào.
