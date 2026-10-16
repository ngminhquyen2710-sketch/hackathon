# VinBus / VinFast BlindSpot BEV & TTC — Simulation & Closed-Loop Perception System
**AI20k · Track 4 Capstone · Vin Perception Hackathon**

Hệ thống mô phỏng 3D khép kín (Closed-loop Simulation) chạy trực tiếp trên **1 máy tính cá nhân duy nhất** (60 FPS, chỉ cần `NumPy` + `OpenCV`), giải quyết bài toán **Cảnh báo Va chạm Điểm mù & Phanh Khẩn Cấp (AEB) trên xe buýt điện VinBus / xe VinFast** khi di chuyển qua mặt đường gồghề (ổ gà, gờ giảm tốc) và bị che khuất tầm nhìn tại đô thị Việt Nam.

---

## 1. Cách Chạy Ngay Lập Tức (Live Demo)

Mở terminal tại thư mục project và chạy:

```bash
# Chạy hệ thống Mô phỏng Tương tác Thời gian thực (60 FPS):
python main.py

# Hoặc chạy Benchmark tự động qua 4 kịch bản & xuất ảnh + file CSV Replay:
python main.py --benchmark
```

### Phím tắt tương tác trực tiếp dành cho Giám khảo (Live Demo 25/25 điểm):
| Phím | Chức năng | Ý nghĩa Kiến thức Track 4 |
| :--- | :--- | :--- |
| **`1`** | **Kịch bản 1:** Xe máy cắt ngang điểm mù & Người đi bộ | Detection (Day 3) + BEV (Day 8) + TTC & Phanh AEB (Day 7, 12) |
| **`2`** | **Kịch bản 2:** Đường xóc / Ổ gà liên tục | **Sensor Fusion Camera + IMU (Day 4, 8)** giúp giảm **~80% sai số RMSE** |
| **`3`** | **Kịch bản 3:** Người đi bộ bị xe tải che khuất 100% | **Kalman Filter Predict & Vòng Elip Bất định (Day 5, 7)** giữ track khi mất dấu |
| **`4`** | **Kịch bản 4 (CA LỖI):** Rào chắn thấp che khuất bàn chân | Chứng minh giới hạn của giả định mặt đất phẳng ($Z = 0$) trong Homography |
| **`P`** | Tạo cú xóc **Ổ gà tức thời** (`+5.2°` góc Pitch) | Giám khảo tự bấm để thử thách khả năng bù rung lắc của hệ thống |
| **`I`** | Bật / Tắt **IMU Pitch Compensation** | So sánh trực tiếp sai số có/không có hợp nhất cảm biến IMU |
| **`K`** | Bật / Tắt **Bộ lọc Kalman 2D** | So sánh quỹ đạo mượt + dự báo (Forecasting) so với đo thô |
| **`A`** | Bật / Tắt **Phanh tự động Closed-Loop AEB** | Kiểm chứng vòng lặp đóng từ Perception $\rightarrow$ Control |
| **`S`** | Lưu **Log CSV (`logs/vinbus_replay_log.csv`)** & Ảnh chụp | Phục vụ trụ cột **Dữ liệu & Replay (Day 12)** |

---

## 2. Bản Đồ Kiến Thức Track 4 Trong Code (25/25 điểm)

Project được chia thành 4 file độc lập, mỗi thành viên trong nhóm có thể phụ trách giải thích 1 file khi Hội đồng hỏi đáp:

1. [`simulator.py`](./simulator.py) — **Day 11 (Synthetic, Simulated & Trajectory Data) & Day 3 (2D Detection):**
   * Mô phỏng vật lý xe VinBus (tốc độ, phanh khẩn cấp, hệ thống treo lò xo giảm chấn dao động góc Pitch $\Delta\theta(t)$ khi qua ổ gà).
   * Mô hình **Pinhole Camera 3D** ($K[R|t]$) kết xuất ảnh RGB theo góc nghiêng thực tế và trích xuất Bounding Box 2D kèm nhiễu đo lường và xử lý che khuất (Occlusion).
2. [`geometry_bev.py`](./geometry_bev.py) — **Day 4 (Temporal Sync) & Day 8 (BEV & Camera-IMU Sensor Fusion):**
   * Chiếu ngược điểm đáy Bounding Box $(u, v)$ sang mặt phẳng **Bird's-Eye View (BEV)** $(X_{\text{BEV}}, Y_{\text{BEV}})$ tại $Z=0$.
   * **Hợp nhất Camera + IMU:** Bù góc chúi đầu xe $\theta = \theta_{\text{nominal}} + \Delta\theta_{\text{IMU}}$ theo thời gian thực:
     $$X_{\text{BEV}} = H_{\text{cam}} \frac{\cos\theta - y_n \sin\theta}{\sin\theta + y_n \cos\theta}, \quad Y_{\text{BEV}} = x_n (X_{\text{BEV}} \cos\theta + H_{\text{cam}} \sin\theta)$$
3. [`kalman_tracker.py`](./kalman_tracker.py) — **Day 5 (Kalman Filter & Uncertainty) & Day 7 (Tracking & Forecasting):**
   * Bộ lọc Kalman 4 trạng thái $\mathbf{x} = [X, Y, V_X, V_Y]^T$, ma trận hiệp phương sai $\mathbf{P} \in \mathbb{R}^{4\times 4}$ với nhiễu đo $\mathbf{R}$ phụ thuộc khoảng cách ($R \propto X^2$).
   * Trích xuất trị riêng (Eigenvalues) của $\mathbf{P}_{XY}$ để vẽ **Vòng Elip Bất định 95%** và tính **Time-To-Collision ($\text{TTC}$)**.
4. [`closed_loop_eval.py`](./closed_loop_eval.py) — **Day 12 (Replay & Closed-Loop Evaluation):**
   * Tính toán trực tiếp **RMSE (m)** giữa tọa độ ước lượng và Ground Truth, điều khiển phanh khẩn cấp **Closed-loop AEB**, và xuất log ra CSV.

---

## 3. Kịch Bản Thuyết Trình 10 Phút (Đúng Chuẩn Rubric)

* **Phút 0:00 – 1:00 (Bài toán Vin & Độ hữu ích - 15đ):**
  * Đội xe hơn **300 chiếc VinBus** và hàng chục nghìn xe **VinFast / Xanh SM** vận hành trong đô thị đông đúc tại Việt Nam thường xuyên gặp rủi ro xe máy cắt ngang điểm mù khi mặt đường gồ ghề.
  * Giải pháp tận dụng ngay **Camera góc rộng + cảm biến IMU có sẵn** trên xe để dựng bản đồ BEV và phanh tự động.
* **Phút 1:00 – 5:00 (Live Demo trực tiếp - 25đ):**
  * Chạy `python main.py`.
  * **Bấm `[1]`:** Cho xem xe máy cắt ngang, hệ thống dự báo mũi tên vàng, tính $\text{TTC} < 2.1\text{s}$ và kích hoạt **CLOSED-LOOP AEB BRAKING** dừng xe an toàn.
  * **Bấm `[2]` & `[P]`:** Cho xe đi qua ổ gà. Chỉ vào **Chấm Đỏ (Raw BEV)** nhảy loạn xạ sai lệch $2\text{--}3\text{m}$ khi đầu xe nhấp nhô, trong khi **Chấm Xanh (IMU + Kalman)** bám chặt **Vòng tròn Trắng (Ground Truth)**. Mời Giám khảo bấm phím `[I]` (tắt IMU) và `[P]` (tạo ổ gà) để kiểm chứng tại chỗ!
  * **Bấm `[3]`:** Cho người đi bộ đi khuất hoàn toàn sau xe tải (`occluder_van`). Chỉ vào nhãn `OCCLUDED -> KALMAN PREDICT` và vòng elip bất định tự động phình to khi mất camera, rồi thu nhỏ lại ngay khi người đi bộ bước ra.
* **Phút 5:00 – 7:00 (Số đo, Ca lỗi & Impact - 25đ):**
  * **Số đo kiểm chứng:** Nhìn vào bảng góc trên bên phải: Trên đường xóc (Kịch bản 2), `Raw BEV RMSE = 1.41 m`, còn `IMU + Kalman RMSE = 0.28 m` (**giảm ~80% sai số**).
  * **Ca lỗi chủ đích (Bấm `[4]`):** Một rào chắn thấp cao $0.78\text{m}$ che mất phần chân người đi bộ. Đáy Bounding Box lúc này nằm ở ngang hông ($Z = 0.78\text{m} \neq 0$). Phép chiếu Homography phẳng ($Z=0$) phóng tia nhìn xuyên qua hông xuống mặt đất phía xa, làm ước lượng khoảng cách bị vọt lên $18.1\text{m}$ thay vì $10.5\text{m}$ (**sai số $+7.8\text{m}$**). Sơ đồ hình học góc dưới bên trái giải thích trực quan nguyên nhân này.
  * **Impact:** Tiết kiệm **$\$1,500 - \$3,000$/xe** chi phí LiDAR (tương đương **$\$450,000 - \$900,000$** cho đội xe 300 chiếc VinBus), cảnh báo sớm $1.8\text{s}$ và tự động thu thập log các ca suýt va chạm (`logs/vinbus_replay_log.csv`) cho Data Engine của VinFast ADAS.
* **Phút 7:00 – 10:00 (Hỏi đáp Hội đồng - 10đ):**
  * Mỗi thành viên trả lời sâu về module mình phụ trách.
