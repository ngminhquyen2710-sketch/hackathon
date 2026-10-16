---
marp: true
theme: default
paginate: true
header: "Vin Perception Hackathon · Track 4 Capstone"
footer: "VinBus / VinFast BlindSpot BEV & Closed-Loop Perception System"
style: |
  section {
    background-color: #0b131e;
    color: #f1f5f9;
    font-family: 'Segoe UI', system-ui, sans-serif;
  }
  h1, h2 {
    color: #00d2ff;
  }
  h3 {
    color: #00e682;
  }
  code {
    background: #1e293b;
    color: #38bdf8;
  }
---

# VINBUS / VINFAST BLINDSPOT BEV & TTC SYSTEM
## Hệ Thống Mô Phỏng Cảnh Báo Điểm Mù, Hợp Nhất Camera + IMU & Phanh Tự Động AEB Khép Kín

**Track 4 Capstone · Vin Perception Hackathon**  
*Đội thi AI20k · Thiết Kế Tối Ưu Cho Đội Xe Điện Đô Thị Việt Nam (60 FPS CPU)*

---

## 1. Bối Cảnh Thực Tế & Bài Toán Kinh Doanh

### 🔴 4 Nỗi Đau Lớn Của Xe Buýt Điện VinBus:
- **Xe máy tạt đầu điểm mù:** Góc khuất bên phải khi xe buýt chuẩn bị cập trạm.
- **Đường gồ ghề (ổ gà, gờ giảm tốc):** Thân xe nhấp nhô $\pm 5^\circ$ khiến camera thông thường nhảy sai khoảng cách $2.0\text{m} - 3.5\text{m}$.
- **Phanh giật ảo (Phantom Braking):** Phanh gấp nhầm làm ngã hành khách đứng trên xe buýt.
- **Chi phí LiDAR quá cao:** \$1,500 – \$3,000/xe $\rightarrow$ ngốn \$450k – \$900k cho đội xe 300 chiếc VinBus!

### 🟢 Giải Pháp Đột Phá Của Nhóm:
- Tận dụng **Camera góc rộng sẵn có + Cảm biến IMU tiêu chuẩn**.
- **Hợp nhất Camera + IMU động (Day 8):** Bù góc chúi tức thời, giảm ngay **80% sai số**.
- **Kalman 2D thích ứng (Day 5 & 7):** $R(X) \propto X^2$, biểu diễn Vòng Elip Bất định 95%.
- **Closed-Loop AEB (Day 12):** Tự động phanh khẩn cấp dừng an toàn khi $TTC < 2.1\text{s}$.

---

## 2. Kiến Trúc Hệ Thống Khép Kín (Closed-Loop Pipeline)

1. **Cảm Biến & Mô Phỏng Vật Lý (Day 11):**
   - Động học VinBus ($v = 8\text{ m/s}$), dao động góc Pitch hệ thống treo $\Delta\theta(t)$.
   - Mô hình Pinhole $K[R|t]$ 640x420, trích xuất điểm đáy Bounding Box $(u_{bc}, v_{bc})$.
2. **Hình Học BEV & Hợp Nhất IMU (Day 4 & 8):**
   - Chiếu ngược mặt phẳng đất phẳng $Z = 0$.
   - Bù góc chúi đầu xe: $\theta(t) = \theta_0 + \Delta\theta_{\text{IMU}}(t)$.
3. **Bộ Lọc Kalman 2D & Dự Báo Quỹ Đạo (Day 5 & 7):**
   - Vector trạng thái $\mathbf{x} = [X, Y, V_X, V_Y]^T$.
   - Vòng Elip Bất định 95% ($\chi^2 = 5.991$) & Dự báo hướng đi $2.0\text{s}$.
4. **Điều Khiển Phanh AEB & Replay Log (Day 12):**
   - Phanh khẩn cấp $a = -6.5\text{ m/s}^2$ khi $TTC < 2.1\text{s}$.
   - Ghi dữ liệu vào `logs/vinbus_replay_log.csv`.

---

## 3. Cơ Sở Lý Thuyết: Chiếu Ngược BEV & Hợp Nhất IMU (Day 4 & 8)

### Đạo hàm phương trình chiếu ngược $Z=0$:
Tia quang học chuẩn hóa:
$$x_n = \frac{u - c_x}{f_x}, \quad y_n = \frac{v - c_y}{f_y}$$

Tọa độ khoảng cách dọc và ngang trên mặt đường:
$$X_{\text{BEV}} = H_{\text{cam}} \cdot \frac{\cos\theta - y_n \sin\theta}{\sin\theta + y_n \cos\theta}$$
$$Y_{\text{BEV}} = x_n \cdot \left(X_{\text{BEV}} \cos\theta + H_{\text{cam}} \sin\theta\right)$$

### Hợp nhất cảm biến con quay hồi chuyển IMU:
$$\theta(t) = \theta_{\text{nominal}} + \Delta\theta_{\text{IMU}}(t)$$
- Khi xe qua ổ gà ($\Delta\theta = +5.2^\circ$), IMU bù mẫu số tức thời.
- **Kết quả:** Sai số giảm từ **$1.41\text{ m} \rightarrow 0.28\text{ m}$** (giảm ~80% sai số).

---

## 4. Cơ Sở Lý Thuyết: Bộ Lọc Kalman 2D & Elip Bất Định (Day 5 & 7)

### Thuật toán Kalman Tuyến tính 4D:
- Vector trạng thái: $\mathbf{x} = [X, Y, V_X, V_Y]^T \in \mathbb{R}^4$.
- Ma trận nhiễu đo thích ứng khoảng cách:
  $$\mathbf{R}(X) = \mathbf{R}_{\text{base}} \cdot \max\left(1.0, \left(\frac{X}{10.0}\right)^2\right)$$
  *(Do vi phân Pinhole $dX \propto X^2 dv$, vật thể càng xa thì độ bất định càng lớn)*.

### Vòng Elip Bất định 95% (Uncertainty Ellipse):
- Phân rã trị riêng ma trận hiệp phương sai vị trí $\mathbf{P}_{XY}$:
  $$a = \sqrt{5.991 \cdot \lambda_1}, \quad b = \sqrt{5.991 \cdot \lambda_2}$$
- **Xử lý che khuất (Kịch bản 3):** Khi người đi bộ khuất sau xe tải, tracker chuyển sang chế độ **Kalman Predict (Dead Reckoning)**. Vòng elip tự phình to và tự co lại khi xuất hiện.

---

## 5. Live Demo: Kịch Bản 1 & Kịch Bản 2

### Kịch Bản 1: Xe Máy Cắt Ngang Điểm Mù (Phím `[1]`)
- Xe máy tạt đầu từ góc khuất bên phải cản trước.
- Mũi tên vàng dự báo quỹ đạo lao vào hành lang thân xe buýt.
- Tính $TTC = 2.0\text{s} < 2.1\text{s} \rightarrow$ Kích hoạt **CLOSED-LOOP AEB**, dừng xe an toàn cách $2.3\text{m}$.

### Kịch Bản 2: Đường Xóc Ổ Gà (Phím `[2]` & `[P]`)
- Bấm phím `[P]` tạo cú xóc $+5.2^\circ$.
- **Chấm Đỏ (Raw BEV):** Nhảy loạn xạ sai lệch $2\text{m} - 3.5\text{m}$ do không có IMU.
- **Chấm Xanh (IMU + Kalman):** Bám chặt vòng tròn trắng Ground Truth.
- Bấm `[I]` để tắt IMU $\rightarrow$ Thấy rõ sai số vọt lên $1.41\text{m}$. Bật lại `[I]` giảm về $0.28\text{m}$.

---

## 6. Live Demo: Kịch Bản 3 & Ca Lỗi Chủ Đích 4

### Kịch Bản 3: Che Khuất 100% Sau Xe Tải (Phím `[3]`)
- Người đi bộ bị xe tải giao hàng che khuất $1.5\text{s}$.
- Hệ thống kích hoạt `OCCLUDED -> KALMAN PREDICT`.
- Vòng elip bất định tự động phình to báo hiệu độ tin cậy giảm, và thu nhỏ lại ngay khi người bước ra.

### Kịch Bản 4: Ca Lỗi Chủ Đích Rào Chắn Thấp (Phím `[4]`)
- Rào chắn $0.78\text{m}$ che bàn chân người đi bộ $\implies$ Đáy Bbox ở ngang hông ($Z \neq 0$).
- Phép chiếu phẳng phóng tia xuyên hông:
  $$\Delta X_{\text{error}} = X_{\text{true}} \cdot \left(\frac{Z_{\text{barrier}}}{H_{\text{cam}} - Z_{\text{barrier}}}\right)$$
- **Thực tế:** $10.6\text{m} \implies$ **Ước lượng sai:** $18.4\text{m}$ (Sai lệch $+7.8\text{m}$)!
- Chứng minh giới hạn vật lý của camera đơn sắc và lý do cần kết hợp Radar/LiDAR.

---

## 7. Bằng Chứng Thực Nghiệm Định Lượng (Day 12)

| Chỉ số kiểm chứng | Kết quả đạt được | Ý nghĩa kỹ thuật |
| :--- | :--- | :--- |
| **Giảm sai số RMSE (Kịch bản 2)** | **-80.1%** | Từ $1.41\text{m}$ giảm còn $0.28\text{m}$ nhờ IMU Fusion |
| **Tốc độ xử lý** | **60 FPS** | Chạy 100% CPU trên laptop văn phòng (NumPy + OpenCV) |
| **Cảnh báo sớm TTC** | **1.8s - 2.1s** | Kích hoạt phanh dừng an toàn cách vật cản $>2\text{m}$ |
| **Độ trễ tính toán** | **< 2ms** | Đáp ứng tức thời, không trễ mạng nơ-ron |
| **Replay Data Engine** | **12 Cột CSV** | Ghi nhận 100% dữ liệu quỹ đạo phục vụ tái kiểm thử |

---

## 8. Tác Động Kinh Doanh & Lộ Trình Phát Triển

### 💰 Giá Trị Kinh Tế:
- Tiết kiệm **\$1,500 – \$3,000 / xe** chi phí cụm cảm biến LiDAR đắt tiền.
- Tiết kiệm **\$450,000 – \$900,000** cho toàn bộ đội xe 300 chiếc VinBus.

### 🛡️ An Toàn Đô Thị:
- Cảnh báo sớm phương tiện cắt ngang điểm mù tại giao lộ đông đúc.
- Triệt tiêu hoàn toàn hiện tượng phanh giật ảo (phantom braking) khi qua đường gồ ghề.

### 🚀 Khả Năng Mở Rộng:
- Module hóa rõ ràng, dễ dàng tích hợp qua mạng CAN bus hoặc ROS2.
- Kết hợp Radar sóng milimet 77GHz giá rẻ để khắc phục triệt để Ca lỗi 4 ($Z \neq 0$).

---

# CẢM ƠN HỘI ĐỒNG GIÁM KHẢO!

### Sẵn Sàng Live Demo & Trả Lời Câu Hỏi Phản Biện (Q&A)

* Mã nguồn mở đầy đủ: [https://github.com/ngminhquyen2710-sketch/hackathon](https://github.com/ngminhquyen2710-sketch/hackathon)
* File chạy mô phỏng trực tiếp: `python main.py`
* File chạy benchmark tự động: `python main.py --benchmark`
