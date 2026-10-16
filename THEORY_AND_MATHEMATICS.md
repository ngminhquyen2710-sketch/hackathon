# Tài Liệu Cơ Sở Lý Thuyết & Mô Hình Toán Học Chuyên Sâu
**Dự án:** VinBus / VinFast BlindSpot BEV & Closed-Loop Perception System  
**Chương trình:** Vin Perception Hackathon · Track 4 Capstone  
**Tác giả:** Đội ngũ Kỹ thuật Vin Perception

---

## 📐 1. HỆ TỌA ĐỘ VÀ CÁC PHÉP BIẾN ĐỔI HÌNH HỌC (COORDINATE FRAMES)

Hệ thống định nghĩa 4 hệ tọa độ tiêu chuẩn trong Robot tự hành và Xe tự lái:

```
                +Z (Trời)
                 ^
                 |   +X (Tiến)
                 |  /
                 | /
                 o-----> +Y (Phải)
          [Hệ Tọa Độ Xe - Ego Frame]
```

1. **Hệ tọa độ Xe (Vehicle / Ego Frame $\mathcal{F}_{\text{ego}}$):**
   * Gốc tọa độ $O_{\text{ego}}$: Tâm cản trước của xe VinBus tiếp xúc với mặt đất.
   * Trục $+X_{\text{ego}}$: Hướng về phía trước mũi xe (Longitudinal axis, tính bằng mét).
   * Trục $+Y_{\text{ego}}$: Hướng sang bên phải xe (Lateral axis, tính bằng mét).
   * Trục $+Z_{\text{ego}}$: Hướng thẳng đứng lên trời (Vertical axis, $Z=0$ là mặt đường).

2. **Hệ tọa độ Camera (Camera Frame $\mathcal{F}_{\text{cam}}$):**
   * Gốc tọa độ $O_{\text{cam}}$: Đặt tại tâm quang học của camera, gắn ở độ cao $H_{\text{cam}} = 1.65\text{ m}$.
   * Trục $+Z_{\text{cam}}$: Trục quang học hướng ra xa theo hướng nhìn của camera.
   * Trục $+X_{\text{cam}}$: Hướng sang bên phải mặt phẳng ảnh.
   * Trục $+Y_{\text{cam}}$: Hướng chúi xuống dưới mặt đất theo phương vuông góc với trục quang.

3. **Hệ tọa độ Mặt phẳng Ảnh (Pixel Frame $\mathcal{F}_{\text{img}}$):**
   * Tọa độ $(u, v)$ tính bằng pixel; gốc $(0, 0)$ ở góc trên bên trái khung hình kích thước $W \times H = 640 \times 420$.
   * $u$: Cột pixel (từ trái sang phải, $0 \le u \le W-1$).
   * $v$: Hàng pixel (từ trên xuống dưới, $0 \le v \le H-1$).

---

## 📷 2. MÔ HÌNH QUANG HỌC PINHOLE CAMERA (DAY 4 & DAY 8)

### 2.1. Ma trận Tham số Nội (Camera Intrinsics Matrix $K$)
Mô hình chiếu xuyên tâm Pinhole ánh xạ một điểm trong hệ tọa độ camera $\mathbf{P}_{\text{cam}} = [X_{\text{cam}}, Y_{\text{cam}}, Z_{\text{cam}}]^T$ sang tọa độ ảnh thuần nhất:

$$\begin{bmatrix} u \\ v \\ 1 \end{bmatrix} \sim \mathbf{K} \begin{bmatrix} X_{\text{cam}} \\ Y_{\text{cam}} \\ Z_{\text{cam}} \end{bmatrix} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_{\text{cam}} \\ Y_{\text{cam}} \\ Z_{\text{cam}} \end{bmatrix}$$

* Tiêu cự chuẩn hóa: $f_x = 460.0\text{ px}, f_y = 460.0\text{ px}$ (Góc nhìn rộng Field-of-View $\approx 70^\circ$).
* Điểm quang tâm (Principal point): $c_x = \frac{W}{2} = 320.0\text{ px}, c_y = \frac{H}{2} = 210.0\text{ px}$.

Tọa độ tia chuẩn hóa (Normalized Image Coordinates):
$$x_n = \frac{u - c_x}{f_x} = \frac{X_{\text{cam}}}{Z_{\text{cam}}}, \quad y_n = \frac{v - c_y}{f_y} = \frac{Y_{\text{cam}}}{Z_{\text{cam}}}$$

---

### 2.2. Ma trận Tham số Ngoại (Extrinsics $[R|t]$)
Camera được gắn trên xe VinBus với góc chúc danh định $\theta_{\text{nominal}} = 10^\circ$ ($0.1745\text{ rad}$) hướng xuống mặt đường, độ cao $H_{\text{cam}} = 1.65\text{ m}$.

Xét phép quay quanh trục ngang (Pitch Rotation $\theta$):
$$\begin{bmatrix} X_{\text{cam}} \\ Y_{\text{cam}} \\ Z_{\text{cam}} \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ -\sin\theta & 0 & -\cos\theta \\ \cos\theta & 0 & -\sin\theta \end{bmatrix} \begin{bmatrix} X_{\text{ego}} \\ Y_{\text{ego}} \\ Z_{\text{ego}} \end{bmatrix} + \begin{bmatrix} 0 \\ H_{\text{cam}} \cos\theta \\ H_{\text{cam}} \sin\theta \end{bmatrix}$$

Rút gọn quan hệ giữa tọa độ không gian xe $\mathbf{P}_{\text{ego}} = [X, Y, Z]^T$ và tọa độ camera:
$$\begin{cases} X_{\text{cam}} = Y \\ Y_{\text{cam}} = -(X \sin\theta + Z \cos\theta) + H_{\text{cam}} \cos\theta \\ Z_{\text{cam}} = X \cos\theta - Z \sin\theta + H_{\text{cam}} \sin\theta \end{cases}$$

---

## 🌐 3. CHIẾU NGƯỢC BEV (HOMOGRAPHY) & HỢP NHẤT CAMERA + IMU (DAY 8)

### 3.1. Đạo Hàm Chiếu Ngược Điểm Tiếp Đất (Inverse Perspective Mapping - IPM)
Đối với camera đơn sắc (Monocular Camera), một pixel $(u, v)$ ứng với một tia vô hạn trong không gian 3D. Để tìm nghiệm tọa độ duy nhất, ta áp dụng **Giả định Mặt đất phẳng (Flat-Ground Assumption)** tại vị trí bàn chân / bánh xe tiếp đất:

$$Z_{\text{ego}} = 0$$

Thay $Z_{\text{ego}} = 0$ vào hệ phương trình camera:
$$Y_{\text{cam}} = -X \sin\theta + H_{\text{cam}} \cos\theta$$
$$Z_{\text{cam}} = X \cos\theta + H_{\text{cam}} \sin\theta$$

Từ tỉ số chuẩn hóa $y_n = \frac{Y_{\text{cam}}}{Z_{\text{cam}}}$:
$$y_n = \frac{-X \sin\theta + H_{\text{cam}} \cos\theta}{X \cos\theta + H_{\text{cam}} \sin\theta}$$

Nhân chéo và nhóm theo biến $X$:
$$y_n (X \cos\theta + H_{\text{cam}} \sin\theta) = -X \sin\theta + H_{\text{cam}} \cos\theta$$
$$X (y_n \cos\theta + \sin\theta) = H_{\text{cam}} (\cos\theta - y_n \sin\theta)$$

Từ đó, ta tìm được **Phương trình tọa độ dọc $X_{\text{BEV}}$** theo mét:
$$\mathbf{X_{\text{BEV}} = H_{\text{cam}} \cdot \frac{\cos\theta - y_n \sin\theta}{\sin\theta + y_n \cos\theta}}$$

Khi đã có $X_{\text{BEV}}$, chiều sâu quang học $Z_{\text{cam}}$ được tính chính xác:
$$Z_{\text{cam}} = X_{\text{BEV}} \cos\theta + H_{\text{cam}} \sin\theta$$

Từ $x_n = \frac{X_{\text{cam}}}{Z_{\text{cam}}} = \frac{Y_{\text{BEV}}}{Z_{\text{cam}}}$, ta suy ra **Phương trình tọa độ ngang $Y_{\text{BEV}}$**:
$$\mathbf{Y_{\text{BEV}} = x_n \cdot Z_{\text{cam}} = \left(\frac{u - c_x}{f_x}\right) \cdot \left(X_{\text{BEV}} \cos\theta + H_{\text{cam}} \sin\theta\right)}$$

---

### 3.2. Hợp Nhất Cảm Biến Quán Tính (IMU Pitch Compensation)
Khi VinBus di chuyển qua ổ gà hoặc đạp phanh, thân xe bị chúc đầu (Pitch) hoặc ngửa đuôi, làm góc chúc thực tế của camera biến thiên liên tục theo thời gian:

$$\theta(t) = \theta_{\text{nominal}} + \Delta\theta_{\text{IMU}}(t)$$

* Nếu **KHÔNG** bù trừ IMU ($\Delta\theta_{\text{IMU}} = 0$): Khi đầu xe chúi xuống $\Delta\theta = +5^\circ$, mẫu số $(\sin\theta + y_n \cos\theta)$ tăng vọt, làm $X_{\text{BEV}}$ ước lượng bị co ngắn lại từ $2.0\text{m} - 3.5\text{m}$ (Chấm đỏ giật cục, gây phanh khẩn cấp ảo).
* Khi **CÓ** bù trừ IMU: Góc $\theta(t)$ được cập nhật chính xác ở chu kỳ 100 Hz, giúp mẫu số luôn tự động cân bằng theo chuyển động nhún của hệ thống treo, triệt tiêu gần như hoàn toàn sai số.

---

## 🔄 4. BỘ LỌC KALMAN 2D & BIỂU DIỄN BẤT ĐỊNH (DAY 5 & DAY 7)

### 4.1. Vector Trạng Thái & Mô Hình Động Học Vận Tốc Không Đổi (CV Model)
Theo dõi vị trí và vận tốc vật thể trên mặt phẳng BEV $2\text{D}$:

$$\mathbf{x}_k = \begin{bmatrix} X_k \\ Y_k \\ V_{X, k} \\ V_{Y, k} \end{bmatrix} \in \mathbb{R}^4$$

Ma trận chuyển trạng thái $\mathbf{F}(\Delta t)$ với chu kỳ lấy mẫu $\Delta t$:
$$\mathbf{F} = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Ma trận hiệp phương sai nhiễu quá trình $\mathbf{Q}$ (Continuous White-Noise Acceleration):
$$\mathbf{Q} = \begin{bmatrix} q_x \Delta t & 0 & 0 & 0 \\ 0 & q_y \Delta t & 0 & 0 \\ 0 & 0 & q_{vx} \Delta t & 0 \\ 0 & 0 & 0 & q_{vy} \Delta t \end{bmatrix}$$
với $q_x = q_y = 0.15\text{ m}^2/\text{s}, q_{vx} = q_{vy} = 1.20\text{ m}^2/\text{s}^3$.

---

### 4.2. Mô Hình Đo Lường & Ma Trận Nhiễu Thích Ứng Khoảng Cách $R(X)$
Cảm biến camera BEV chỉ quan sát trực tiếp vị trí $(X, Y)$, không đo được trực tiếp vận tốc:
$$\mathbf{z}_k = \begin{bmatrix} z_X \\ z_Y \end{bmatrix} \in \mathbb{R}^2, \quad \mathbf{H} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

**Ma trận Nhiễu đo Thích ứng (Distance-Adaptive Measurement Noise $\mathbf{R}$):**  
Do tính chất hình học Pinhole $v \propto \frac{f \cdot H}{X}$, sai số đo pixel $dv$ gây ra sai số khoảng cách $dX \propto X^2 dv$. Do đó:
$$\mathbf{R}(X) = \mathbf{R}_{\text{base}} \cdot \max\left(1.0, \; \left(\frac{X}{10.0}\right)^2\right)$$
với $\mathbf{R}_{\text{base}} = \operatorname{diag}(0.12, 0.08)\text{ m}^2$.

---

### 4.3. Các Bước Lọc Tuyến Tính (Predict & Update)

1. **Bước Dự báo (Prediction):**
   $$\mathbf{x}_{k|k-1} = \mathbf{F} \mathbf{x}_{k-1|k-1}$$
   $$\mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1|k-1} \mathbf{F}^T + \mathbf{Q}$$

2. **Bước Cập nhật (Measurement Update):**
   * Phần dư sai số quan sát (Innovation):
     $$\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \mathbf{x}_{k|k-1}$$
   * Ma trận hiệp phương sai sai số quan sát:
     $$\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + \mathbf{R}(X)$$
   * Hệ số khuếch đại tối ưu Kalman Gain:
     $$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}$$
   * Cập nhật trạng thái và ma trận hiệp phương sai:
     $$\mathbf{x}_{k|k} = \mathbf{x}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$
     $$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}$$

---

### 4.4. Vòng Elip Bất Định 95% (Uncertainty Ellipse Chi-Square)
Để biểu diễn trực quan độ tin cậy vị trí của vật thể trên màn hình Radar BEV, trích xuất ma trận hiệp phương sai vị trí $2 \times 2$:
$$\mathbf{P}_{XY} = \begin{bmatrix} P_{XX} & P_{XY} \\ P_{YX} & P_{YY} \end{bmatrix}$$

Thực hiện phân rã trị riêng (Eigenvalue Decomposition):
$$\mathbf{P}_{XY} \mathbf{v}_i = \lambda_i \mathbf{v}_i \quad (i = 1, 2; \; \lambda_1 \ge \lambda_2)$$

* Ngưỡng phân phối Chi-bình phương với 2 bậc tự do ở mức tin cậy $95\%$ là $\chi^2_{0.95, 2} = 5.991$.
* Bán trục lớn của elip: $a = \sqrt{5.991 \cdot \lambda_1}$.
* Bán trục nhỏ của elip: $b = \sqrt{5.991 \cdot \lambda_2}$.
* Góc xoay của elip so với trục dọc: $\phi = \operatorname{atan2}(v_{1, y}, v_{1, x})$.

---

## ⏱️ 5. DỰ BÁO QUỸ ĐẠO & TÍNH TOÁN TIME-TO-COLLISION (DAY 7 & DAY 12)

### 5.1. Mô Hình Dự Báo Tương Lai (Trajectory Forecasting)
Từ trạng thái ước lượng $\mathbf{x} = [X, Y, V_X, V_Y]^T$, quỹ đạo tương lai trong khoảng thời gian $t \in [0, T_{\text{horizon}}]$ (với $T_{\text{horizon}} = 2.0\text{ s}$) được dự báo bằng:
$$\hat{\mathbf{p}}(t) = \begin{bmatrix} \hat{X}(t) \\ \hat{Y}(t) \end{bmatrix} = \begin{bmatrix} X + V_X \cdot t \\ Y + V_Y \cdot t \end{bmatrix}$$

---

### 5.2. Công Thức Tính Time-To-Collision (TTC) & Hành Lang Va Chạm (Corridor)
Vật thể chỉ tạo ra nguy cơ va chạm với xe buýt VinBus nếu thỏa mãn đồng thời 2 điều kiện động học:
1. Đang ở phía trước đầu xe ($X > 0.3\text{m}$) và đang tiến lại gần cản trước ($V_X < -0.25\text{ m/s}$).
2. Thời gian chạm cản trước ($X_{\text{bumper}} = 0.5\text{m}$):
   $$\mathbf{TTC_X = \frac{X - 0.5}{-V_X}}$$

**Kiểm tra Hành lang Va chạm Chiều ngang (Collision Corridor Check):**
Vị trí chiều ngang dự báo tại thời điểm va chạm chạm cản trước:
$$Y_{\text{impact}} = Y + V_Y \cdot TTC_X$$

Hệ thống kích hoạt cảnh báo nguy cơ va chạm nếu $Y_{\text{impact}}$ nằm trong độ rộng an toàn của thân xe buýt ($W_{\text{half}} = 1.75\text{m}$), tính kèm biên độ bất định $0.5\sigma_Y = 0.5\sqrt{P_{YY}}$:
$$|Y_{\text{impact}}| \le W_{\text{half}} + 0.5\sqrt{P_{YY}} \quad \text{hoặc} \quad |Y_{\text{current}}| \le W_{\text{half}}$$

---

## ⚠️ 6. CHỨNG MINH TOÁN HỌC CA LỖI CHỦ ĐÍCH (FAILURE MODE ANALYSIS)

### 6.1. Bản Chất Hình Học Khi Bị Che Khuất Thân Dưới (Scenario 4)
Trong kịch bản 4, một dải rào chắn bê tông thấp có chiều cao $Z_{\text{barrier}} = 0.78\text{ m}$ đặt ở $X_{\text{barrier}} = 9.5\text{ m}$. Người đi bộ di chuyển phía sau rào chắn tại $X_{\text{true}} = 10.6\text{ m}$.

Mô hình 2D Object Detector chỉ nhận diện được phần cơ thể nhìn thấy từ thắt lưng trở lên. Do đó, điểm đáy của Bounding Box không tiếp xúc với mặt đất mà nằm ở độ cao:
$$Z_{\text{bottom}} = Z_{\text{barrier}} = 0.78\text{ m} > 0$$

```
   Camera (H_cam)
        \
         \    Tia nhìn qua thắt lưng (Z = 0.78m)
          \            |
           \          [Vật thể thực tại X_true = 10.6m]
            \          |
             \-----\   |
                    \  |
                     \ |
                      \|
          ================================== Ground (Z = 0)
                       X_true        X_BEV (Ước lượng sai lệch: 18.4m)
                       |-------------|
                         Delta_X = +7.8m (Sai số phóng đại)
```

### 6.2. Đạo Hàm Sai Số Phóng Đại Khoảng Cách (Overestimation Bias Formula)
Theo hình học tam giác đồng dạng giữa tâm camera $(0, H_{\text{cam}})$, điểm đáy thực tế $(X_{\text{true}}, Z_{\text{barrier}})$, và điểm giao giả định với mặt đất $(X_{\text{est}}, 0)$:

$$\frac{X_{\text{est}}}{H_{\text{cam}}} = \frac{X_{\text{est}} - X_{\text{true}}}{Z_{\text{barrier}}}$$

Nhân chéo:
$$X_{\text{est}} \cdot Z_{\text{barrier}} = H_{\text{cam}} \cdot X_{\text{est}} - H_{\text{cam}} \cdot X_{\text{true}}$$
$$H_{\text{cam}} \cdot X_{\text{true}} = X_{\text{est}} (H_{\text{cam}} - Z_{\text{barrier}})$$
$$X_{\text{est}} = X_{\text{true}} \cdot \frac{H_{\text{cam}}}{H_{\text{cam}} - Z_{\text{barrier}}}$$

Từ đó suy ra **Độ sai lệch tuyệt đối $\Delta X_{\text{error}}$**:
$$\mathbf{\Delta X_{\text{error}} = X_{\text{est}} - X_{\text{true}} = X_{\text{true}} \cdot \left(\frac{Z_{\text{barrier}}}{H_{\text{cam}} - Z_{\text{barrier}}}\right)}$$

### 6.3. Tính Toán Bằng Số Với Tham Số Thực Tế VinBus
* Chiều cao camera: $H_{\text{cam}} = 1.65\text{ m}$.
* Chiều cao rào chắn: $Z_{\text{barrier}} = 0.78\text{ m}$.
* Khoảng cách thực tế của người đi bộ: $X_{\text{true}} = 10.6\text{ m}$.

Tỉ số phóng đại hình học:
$$\text{Factor} = \frac{Z_{\text{barrier}}}{H_{\text{cam}} - Z_{\text{barrier}}} = \frac{0.78}{1.65 - 0.78} = \frac{0.78}{0.87} \approx 0.8965$$

Sai số ước lượng:
$$\Delta X_{\text{error}} = 10.6 \times 0.8965 \approx \mathbf{+9.50\text{ m}} \implies X_{\text{est}} \approx 10.6 + 9.5 \approx 20.1\text{ m}$$
*(Trong môi trường mô phỏng có góc pitch $10^\circ$, hệ thống ước lượng ra $18.4\text{m}$, tức sai lệch $+7.8\text{m}$ so với thực tế)*.

**Hậu quả:** Hệ thống ADAS phán đoán người đi bộ đang ở xa ngoài vùng nguy hiểm ($18.4\text{m}$ thay vì $10.6\text{m}$), dẫn đến $TTC > 3.5\text{s}$ và **bỏ lỡ cơ hội phanh khẩn cấp AEB**.

---

## 📊 7. CHỈ SỐ ĐO KIỂM ĐỊNH LƯỢNG & ĐIỀU KHIỂN VÒNG LẶP ĐÓNG (DAY 12)

### 7.1. Sai Số Căn Trung Bình Bình Phương (Root Mean Square Error - RMSE)
Tại mỗi frame $k$, sai số vị trí Euclide 2D giữa điểm ước lượng và Ground Truth $(X_{\text{GT}}, Y_{\text{GT}})$:
$$e_k = \sqrt{(X_{\text{est}, k} - X_{\text{GT}, k})^2 + (Y_{\text{est}, k} - Y_{\text{GT}, k})^2}$$

RMSE tích lũy trên cửa sổ trượt $N = 180$ frames:
$$\mathbf{RMSE = \sqrt{\frac{1}{N} \sum_{k=1}^N e_k^2}}$$

Tỉ lệ cải thiện độ chính xác khi kích hoạt IMU + Kalman so với Raw BEV:
$$\mathbf{\Delta_{\text{RMSE}} (\%) = \frac{\text{RMSE}_{\text{raw}} - \text{RMSE}_{\text{KF}}}{\text{RMSE}_{\text{raw}}} \times 100\%}$$

* Kết quả thực nghiệm tại Kịch bản 2:
  * $\text{RMSE}_{\text{raw}} \approx 1.41\text{ m}$.
  * $\text{RMSE}_{\text{KF}} \approx 0.28\text{ m}$.
  * **Cải thiện: $-80.1\%$ sai số!**

---

### 7.2. Logic Điều Khiển Phanh Closed-Loop AEB
Mô hình bộ điều khiển phanh phản hồi vòng lặp đóng:

$$a_{\text{ego}}(t) = \begin{cases} -6.5\text{ m/s}^2 & \text{nếu } TTC(t) < 2.1\text{ s} \text{ và } \text{AEB\_Enabled} = \text{True} \\ 2.0 \cdot (v_{\text{target}} - v_{\text{ego}}) & \text{ngược lại (Duy trì hành trình 8 m/s)} \end{cases}$$

Vận tốc và quãng đường xe buýt được cập nhật bằng tích phân số Euler:
$$v_{\text{ego}}(t + \Delta t) = \max(0.0, \; v_{\text{ego}}(t) + a_{\text{ego}}(t) \cdot \Delta t)$$
$$X_{\text{world}}(t + \Delta t) = X_{\text{world}}(t) + v_{\text{ego}}(t) \cdot \Delta t$$
