# Nguồn tham khảo — 8 tài liệu trong `dóc/pdf/`

> **Vì sao file này tồn tại.** Toàn bộ dự án trích dẫn các nguồn này **bằng tên file** — `tesi.pdf`,
> `2311.02296v7.pdf`, `tesi (1).pdf`. Với người đã tải chúng về thì đủ; với **bất kỳ ai khác** thì
> `tesi.pdf` (tiếng Ý: "luận văn") là một cái tên **không tra được**. Đây là bảng đối chiếu tên file →
> trích dẫn thật, đọc thẳng từ trang bìa của chính các file đó (2026-08-26).
>
> 🔑 **Ba trong tám nguồn đến từ cùng một nhóm ở Politecnico di Torino (Stefano Primatesta)**, và thiết
> kế điều hướng của dự án này thừa kế trực tiếp từ hai trong ba — A* trên costmap và escape xoắn ốc.
> Ai định sửa `uav_navigation` nên đọc chúng trước.

## Bảng đối chiếu

| Tên file trong `dóc/pdf/` | Trích dẫn | Dự án dùng cho |
|---|---|---|
| `2311.02296v7.pdf` | **Survey of Simulators for Aerial Robots: An Overview and In-Depth Systematic Comparisons** — Cora A. Dimmig, Giuseppe Silano, Kimberly McGuire, Chiara Gabellieri, Wolfgang Hönig, Joseph Moore, Marin Kobilarov. arXiv:2311.02296 | Chọn nền mô phỏng (skill `simulator-selection`); so sánh 44 simulator, 14 bản đối chiếu sâu |
| `2510.27327v1.pdf` | **A Modular and Scalable System Architecture for Heterogeneous UAV Swarms Using ROS 2 and PX4-Autopilot** — Robert Pommeranz, Kevin Tebbe, Ralf Heynicke, Gerd Scholl (Helmut-Schmidt-Universität Hamburg). arXiv:2510.27327, 31/10/2025. Bản tác giả, đã nhận đăng tại **12th IEEE ICMRE**, Oldenburg, 2–4/03/2026 | Kiến trúc module ROS2 + PX4, một node ROS2 cho mỗi thành phần |
| `2602.07264v2.pdf` | **`aerial-autonomy-stack` — a Faster-than-real-time, Autopilot-agnostic, ROS2 Framework to Simulate and Deploy Perception-based Drones** — Jacopo Panerati, Sina Sajjadi, Sina Soleymanpour, Varunkumar Mehta, Iraj Mantegh (National Research Council Canada). arXiv:2602.07264, 02/05/2026 | Action interface autopilot-agnostic; mô phỏng nhanh hơn thời gian thực; skill `sim-to-real-transfer` |
| `Gestelt__A_framework_for_acceleration_the_sim_to_real_transition_for_swarm_UAVs.pdf` | **Gestelt: A framework for accelerating the sim-to-real transition for swarm UAVs** — John Tan, Tianchen Sun, Feng Lin, Rodney Teo, Boo Cheong Khoo (National University of Singapore) | Môi trường virtual-physical; kỹ thuật thu hẹp khoảng cách sim→real |
| `IET Cyber-Syst and Robotics - 2023 - Bianchi - A novel distributed architecture for unmanned aircraft systems based on.pdf` | **A novel distributed architecture for unmanned aircraft systems based on Robot Operating System 2** — Lorenzo Bianchi, Daniele Carnevale, Fabio Del Frate, Roberto Masocco, Simone Mattogno, Fabrizio Romanelli, Alessandro Tenaglia (Univ. of Rome "Tor Vergata"). *IET Cyber-Systems and Robotics* 2023;5:e12083. **DOI 10.1049/csy2.12083**. ✅ **Open access, CC-BY** | Phân tầng theo mức tới hạn; máy trạng thái hữu hạn điều phối |
| `tesi.pdf` | **A\*-based Collision Avoidance for UAVs with ROS 2 and PX4 Integration** — Elena Berta. Luận văn thạc sĩ, Politecnico di Torino, Mechatronic Engineering, 10/2025. HD: Stefano Primatesta, Davide Bitetto, Gianluca Ristorto | 🔑 **Nguồn gốc của `route_planner` + `costmap`** — A* trên costmap kiểu Nav2, inflation, cost model |
| `tesi (1).pdf` | **Spiral-Based Reactive Obstacle Avoidance for UAVs with PX4 SITL Integration** — Alessandro Munafò. Luận văn thạc sĩ, Politecnico di Torino, Mechatronic Engineering, N.H. 2024/2025. HD: Stefano Primatesta; đồng HD: Riccardo Enrico | 🔑 **Nguồn gốc của escape xoắn ốc Archimedes** trong `local_avoidance`; Octomap |
| `tesi (2).pdf` | **The development of ROS-based offboard algorithms for autonomous UAVs intended for Mars exploration** — Riccardo Enrico. Luận văn thạc sĩ, Politecnico di Torino, Mechatronic Engineering, 04/2023. HD: Giorgio Guglieri, Stefano Primatesta | Pattern action offboard; hợp nhất Kalman |

## Giấy phép — kiểm chứ đừng đoán

| Nhóm | Tình trạng |
|---|---|
| Ba bản arXiv (`2311.*`, `2510.*`, `2602.*`) | Bản tiền ấn phẩm trên arXiv. Giấy phép **theo từng bài** — kiểm trang arXiv trước khi phát hành lại |
| Bianchi 2023 (IET) | ✅ **CC-BY** — chính trang 1 tự khai: *"open access article under the terms of the Creative Commons Attribution License, which permits use, distribution and reproduction in any medium, provided the original work is properly cited"* |
| Ba luận văn PoliTO | Luận văn công khai của Politecnico di Torino. Nếu phát hành lại thì trích dẫn tác giả + trường |

> ⚠️ **Ghi lại một lần đoán sai của chính tôi (2026-08-26):** tôi đã nêu file IET là rủi ro bản quyền vì
> nó là bài báo của nhà xuất bản Wiley. Đọc trang 1 thì nó là **CC-BY**. Bài học nhỏ nhưng đúng họ với
> R0: **đọc cái nhãn, đừng suy từ tên nhà xuất bản.**

## Kích thước

`dóc/pdf/` là **47 MB / 58 MB** của cả cây nguồn (81%). Nếu cần một bản bàn giao gọn, có thể bỏ thư mục
này — **file này chính là thứ làm cho việc bỏ đó không mất thông tin**, vì mọi trích dẫn nay tra được.
