"""
Generates high-quality 16:9 PowerPoint Presentation for Vin Perception Hackathon Capstone.
Includes styled slides, typography, color themes, metric callouts, and embedded preview screenshots.
"""

import os
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.enum.shapes import MSO_SHAPE

def create_presentation():
    prs = Presentation()
    # 16:9 Widescreen dimensions
    prs.slide_width = Inches(13.333)
    prs.slide_height = Inches(7.5)

    blank_layout = prs.slide_layouts[6]

    # VinFast / Automotive Cyber Theme Colors
    BG_DARK = RGBColor(14, 20, 27)         # Deep slate navy
    BG_CARD = RGBColor(22, 32, 44)         # Card background
    TEXT_LIGHT = RGBColor(240, 245, 250)   # Primary text
    TEXT_MUTED = RGBColor(145, 160, 175)   # Subtitle / secondary
    ACCENT_CYAN = RGBColor(0, 200, 255)    # Tech cyan
    ACCENT_GREEN = RGBColor(0, 230, 130)   # Success emerald
    ACCENT_ORANGE = RGBColor(255, 150, 40) # Warning orange
    ACCENT_RED = RGBColor(255, 75, 75)     # Alert red
    BORDER_COL = RGBColor(45, 60, 80)

    def set_slide_bg(slide):
        bg = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, Inches(13.333), Inches(7.5))
        bg.fill.solid()
        bg.fill.fore_color.rgb = BG_DARK
        bg.line.fill.background()
        return bg

    def add_header(slide, title_text, category_text="VIN PERCEPTION HACKATHON · TRACK 4 CAPSTONE"):
        # Header category tag
        tx_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.4), Inches(11.7), Inches(0.4))
        tf = tx_box.text_frame
        tf.word_wrap = True
        p0 = tf.paragraphs[0]
        p0.text = category_text.upper()
        p0.font.size = Pt(11)
        p0.font.bold = True
        p0.font.color.rgb = ACCENT_CYAN

        # Header Title
        p1 = tf.add_paragraph()
        p1.text = title_text
        p1.font.size = Pt(24)
        p1.font.bold = True
        p1.font.color.rgb = TEXT_LIGHT

    # ==========================================================
    # SLIDE 1: Title Slide
    # ==========================================================
    s1 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s1)

    # Accent decorative bar
    bar = s1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.8), Inches(0.2), Inches(3.8))
    bar.fill.solid()
    bar.fill.fore_color.rgb = ACCENT_CYAN
    bar.line.fill.background()

    tb1 = s1.shapes.add_textbox(Inches(1.2), Inches(1.6), Inches(11.0), Inches(4.2))
    tf1 = tb1.text_frame
    tf1.word_wrap = True

    p = tf1.paragraphs[0]
    p.text = "VINBUS / VINFAST BLINDSPOT BEV & TTC SYSTEM"
    p.font.size = Pt(36)
    p.font.bold = True
    p.font.color.rgb = TEXT_LIGHT

    p2 = tf1.add_paragraph()
    p2.text = "Hệ Thống Mô Phỏng Cảnh Báo Điểm Mù, Hợp Nhất Camera + IMU & Phanh Tự Động AEB Khép Kín"
    p2.font.size = Pt(18)
    p2.font.color.rgb = ACCENT_GREEN
    p2.space_before = Pt(14)

    p3 = tf1.add_paragraph()
    p3.text = "Track 4 Capstone · AI20k · Thiết Kế Tối Ưu Cho Đội Xe Điện Đô Thị Việt Nam (60 FPS CPU)"
    p3.font.size = Pt(14)
    p3.font.color.rgb = TEXT_MUTED
    p3.space_before = Pt(24)

    # 3 Highlights Cards at bottom
    highlights = [
        ("CÔNG NGHỆ ĐỘT PHÁ", "Hợp nhất Camera + IMU bù góc chúi động qua ổ gà, giảm 80% sai số RMSE.", ACCENT_CYAN),
        ("VÒNG LẶP ĐÓNG (AEB)", "Dự báo quỹ đạo Kalman 2.0s, tính TTC và kích hoạt phanh dừng an toàn.", ACCENT_GREEN),
        ("HIỆU QUẢ KINH TẾ", "Tiết kiệm $450k – $900k chi phí LiDAR cho 300 xe VinBus nhờ cảm biến sẵn có.", ACCENT_ORANGE),
    ]
    for idx, (h_title, h_desc, h_color) in enumerate(highlights):
        left = Inches(0.8 + idx * 3.95)
        card = s1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, Inches(5.4), Inches(3.8), Inches(1.5))
        card.fill.solid()
        card.fill.fore_color.rgb = BG_CARD
        card.line.color.rgb = BORDER_COL

        tb = s1.shapes.add_textbox(left + Inches(0.15), Inches(5.45), Inches(3.5), Inches(1.4))
        tf = tb.text_frame
        tf.word_wrap = True
        p_t = tf.paragraphs[0]
        p_t.text = h_title
        p_t.font.size = Pt(12)
        p_t.font.bold = True
        p_t.font.color.rgb = h_color

        p_d = tf.add_paragraph()
        p_d.text = h_desc
        p_d.font.size = Pt(10)
        p_d.font.color.rgb = TEXT_MUTED
        p_d.space_before = Pt(4)

    # ==========================================================
    # SLIDE 2: Problem Statement & Value Proposition
    # ==========================================================
    s2 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s2)
    add_header(s2, "Bối Cảnh Thực Tế & Bài Toán Của Đội Xe VinBus / VinFast")

    col1 = s2.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.6), Inches(5.7), Inches(5.2))
    col1.fill.solid()
    col1.fill.fore_color.rgb = BG_CARD
    col1.line.color.rgb = BORDER_COL

    tb = s2.shapes.add_textbox(Inches(1.0), Inches(1.8), Inches(5.3), Inches(4.8))
    tf = tb.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = "🔴 NỖI ĐAU THỰC TẾ TRONG ĐÔ THỊ"
    p.font.size = Pt(16)
    p.font.bold = True
    p.font.color.rgb = ACCENT_RED

    points_pain = [
        ("Điểm mù xe buýt lớn:", "Xe máy thường xuyên tạt đầu từ góc khuất bên phải khi xe buýt cập trạm."),
        ("Mặt đường gồ ghề:", "Ổ gà, gờ giảm tốc khiến đầu xe chúi/nhấp nhô liên tục (+-5 deg), làm camera đơn sắc ước lượng khoảng cách nhảy loạn xạ (sai lệch 2 - 3.5m)."),
        ("Nguy cơ an toàn:", "Dẫn đến hiện tượng Phanh Giật Ảo (Phantom Braking) gây ngã hành khách hoặc không phanh kịp khi có người băng qua đường."),
        ("Chi phí LiDAR quá cao:", "$1,500 - $3,000/xe -> tiêu tốn $450,000 - $900,000 cho đội xe 300 chiếc VinBus!"),
    ]
    for title, desc in points_pain:
        p_item = tf.add_paragraph()
        p_item.text = f"• {title} {desc}"
        p_item.font.size = Pt(11)
        p_item.font.color.rgb = TEXT_LIGHT
        p_item.space_before = Pt(10)

    col2 = s2.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2))
    col2.fill.solid()
    col2.fill.fore_color.rgb = BG_CARD
    col2.line.color.rgb = BORDER_COL

    tb = s2.shapes.add_textbox(Inches(7.0), Inches(1.8), Inches(5.3), Inches(4.8))
    tf = tb.text_frame
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.text = "🟢 GIẢI PHÁP ĐỘT PHÁ CỦA NHÓM"
    p.font.size = Pt(16)
    p.font.bold = True
    p.font.color.rgb = ACCENT_GREEN

    points_sol = [
        ("Tận dụng cảm biến sẵn có:", "Chỉ dùng 1 Camera góc rộng + Cảm biến con quay hồi chuyển IMU tiêu chuẩn trên xe VinBus."),
        ("Hợp nhất Camera + IMU động:", "Bù trừ tức thời góc nghiêng đầu xe vào phương trình hình học Homography Z=0, giảm ngay 80% sai số."),
        ("Bộ lọc Kalman thích ứng:", "Mô hình hóa ma trận nhiễu R tỉ lệ thuận với bình phương khoảng cách R ~ X^2, vẽ elip bất định 95%."),
        ("Closed-Loop AEB:", "Tự động kích hoạt phanh khẩn cấp a = -6.5 m/s^2 khi TTC < 2.1s, dừng xe an toàn cách vật cản >2m."),
        ("Vận hành cực nhẹ:", "Chạy 60 FPS thuần NumPy + OpenCV trên CPU laptop văn phòng, không cần GPU rời!"),
    ]
    for title, desc in points_sol:
        p_item = tf.add_paragraph()
        p_item.text = f"• {title} {desc}"
        p_item.font.size = Pt(11)
        p_item.font.color.rgb = TEXT_LIGHT
        p_item.space_before = Pt(8)

    # ==========================================================
    # SLIDE 3: System Architecture
    # ==========================================================
    s3 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s3)
    add_header(s3, "Kiến Trúc Hệ Thống Mô Phỏng Khép Kín (Closed-Loop Pipeline)")

    steps = [
        ("1. CẢM BIẾN & MÔ PHỎNG", "Day 11: Physics & Pinhole", [
            "VinBus 3D Kinematics (v = 8 m/s)",
            "Lò xo giảm chấn góc Pitch: dtheta(t)",
            "Pinhole RGB Camera 640x420 K[R|t]",
            "2D Detector trích xuất đáy (u_bc, v_bc)",
        ], ACCENT_CYAN),
        ("2. BEV & HỢP NHẤT IMU", "Day 4 & 8: IPM & Fusion", [
            "Chiếu ngược mặt phẳng Z = 0",
            "Bù góc chúc động: theta = theta_0 + IMU",
            "Tính tọa độ thực (X_bev, Y_bev) mét",
            "Triệt tiêu nảy tọa độ khi xe qua ổ gà",
        ], ACCENT_GREEN),
        ("3. KALMAN & DỰ BÁO TTC", "Day 5 & 7: Tracking & Forecast", [
            "Linear Kalman Filter 4D: [X, Y, Vx, Vy]^T",
            "Nhiễu đo thích ứng R(X) ~ X^2",
            "Vòng elip bất định 95% (chi^2 = 5.991)",
            "Dự báo 2.0s & Tính Time-To-Collision",
        ], ACCENT_ORANGE),
        ("4. ĐIỀU KHIỂN AEB & DATA", "Day 12: Closed-Loop & Replay", [
            "Hành lang va chạm thân xe: |Y| <= 1.75m",
            "Quyết định phanh: TTC < 2.1s",
            "Phanh AEB khẩn cấp: a = -6.5 m/s^2",
            "Ghi log CSV & Replay Engine đánh giá",
        ], ACCENT_RED),
    ]

    for idx, (title, sub, bullets, col) in enumerate(steps):
        left = Inches(0.8 + idx * 2.95)
        card = s3.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, Inches(1.6), Inches(2.8), Inches(5.2))
        card.fill.solid()
        card.fill.fore_color.rgb = BG_CARD
        card.line.color.rgb = BORDER_COL

        tb = s3.shapes.add_textbox(left + Inches(0.15), Inches(1.8), Inches(2.5), Inches(4.8))
        tf = tb.text_frame
        tf.word_wrap = True

        p = tf.paragraphs[0]
        p.text = title
        p.font.size = Pt(13)
        p.font.bold = True
        p.font.color.rgb = col

        p_s = tf.add_paragraph()
        p_s.text = sub
        p_s.font.size = Pt(10)
        p_s.font.color.rgb = TEXT_MUTED
        p_s.space_before = Pt(2)

        for b in bullets:
            p_b = tf.add_paragraph()
            p_b.text = f"• {b}"
            p_b.font.size = Pt(10)
            p_b.font.color.rgb = TEXT_LIGHT
            p_b.space_before = Pt(8)

    # ==========================================================
    # SLIDE 4: Theory - Camera Geometry & IMU Fusion
    # ==========================================================
    s4 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s4)
    add_header(s4, "Cơ Sở Lý Thuyết 1: Chiếu Ngược BEV & Hợp Nhất Camera + IMU (Day 4 & 8)")

    card1 = s4.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card1.fill.solid()
    card1.fill.fore_color.rgb = BG_CARD
    card1.line.color.rgb = BORDER_COL

    tb1 = s4.shapes.add_textbox(Inches(1.0), Inches(1.8), Inches(5.3), Inches(4.8))
    tf1 = tb1.text_frame
    tf1.word_wrap = True

    p = tf1.paragraphs[0]
    p.text = "📐 HÌNH HỌC PINHOLE & PHÉP CHIẾU BEV"
    p.font.size = Pt(14)
    p.font.bold = True
    p.font.color.rgb = ACCENT_CYAN

    math_pts = [
        ("Tia quang học chuẩn hóa:", "x_n = (u - c_x) / f_x,   y_n = (v - c_y) / f_y"),
        ("Giả định mặt đất phẳng:", "Điểm đáy Bounding Box tiếp xúc mặt đường tại Z_ego = 0."),
        ("Phương trình khoảng cách dọc X_BEV:", "X_BEV = H_cam * (cos(theta) - y_n * sin(theta)) / (sin(theta) + y_n * cos(theta))"),
        ("Phương trình khoảng cách ngang Y_BEV:", "Y_BEV = x_n * (X_BEV * cos(theta) + H_cam * sin(theta))"),
        ("Tham số lắp đặt VinBus:", "H_cam = 1.65m (độ cao gắn camera), theta_nominal = 10.0 deg (góc chúc ban đầu)."),
    ]
    for t, m in math_pts:
        p_t = tf1.add_paragraph()
        p_t.text = t
        p_t.font.size = Pt(11)
        p_t.font.bold = True
        p_t.font.color.rgb = TEXT_LIGHT
        p_t.space_before = Pt(8)

        p_m = tf1.add_paragraph()
        p_m.text = m
        p_m.font.size = Pt(10)
        p_m.font.color.rgb = ACCENT_GREEN

    card2 = s4.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card2.fill.solid()
    card2.fill.fore_color.rgb = BG_CARD
    card2.line.color.rgb = BORDER_COL

    tb2 = s4.shapes.add_textbox(Inches(7.0), Inches(1.8), Inches(5.3), Inches(4.8))
    tf2 = tb2.text_frame
    tf2.word_wrap = True

    p = tf2.paragraphs[0]
    p.text = "⚡ ĐỘT PHÁ HỢP NHẤT CẢM BIẾN IMU"
    p.font.size = Pt(14)
    p.font.bold = True
    p.font.color.rgb = ACCENT_ORANGE

    imu_pts = [
        ("Vấn đề khi xe qua ổ gà:", "Góc nhún của thân xe biến thiên dtheta(t) trong khoảng +-5.2 deg. Nếu dùng theta cố định, mẫu số bị lệch khiến X_BEV nhảy sai 2.0m - 3.5m!"),
        ("Giải pháp bù góc Pitch tức thời:", "theta(t) = theta_nominal + dtheta_IMU(t)"),
        ("Tần số cập nhật IMU:", "100 Hz cập nhật trực tiếp vào từng frame nhận thức, triệt tiêu rung lắc trước khi đưa vào bộ lọc Kalman."),
        ("Kết quả thực nghiệm Kịch bản 2:", "Raw BEV (không IMU): RMSE = 1.41 m\n-> IMU + Kalman: RMSE = 0.28 m (Cải thiện ~80% độ chính xác!)"),
    ]
    for t, m in imu_pts:
        p_t = tf2.add_paragraph()
        p_t.text = t
        p_t.font.size = Pt(11)
        p_t.font.bold = True
        p_t.font.color.rgb = TEXT_LIGHT
        p_t.space_before = Pt(8)

        p_m = tf2.add_paragraph()
        p_m.text = m
        p_m.font.size = Pt(10)
        p_m.font.color.rgb = ACCENT_CYAN

    # ==========================================================
    # SLIDE 5: Theory - Kalman Filter & Uncertainty
    # ==========================================================
    s5 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s5)
    add_header(s5, "Cơ Sở Lý Thuyết 2: Bộ Lọc Kalman 2D & Vòng Elip Bất Định 95% (Day 5 & 7)")

    card1 = s5.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card1.fill.solid()
    card1.fill.fore_color.rgb = BG_CARD
    card1.line.color.rgb = BORDER_COL

    tb1 = s5.shapes.add_textbox(Inches(1.0), Inches(1.8), Inches(5.3), Inches(4.8))
    tf1 = tb1.text_frame
    tf1.word_wrap = True

    p = tf1.paragraphs[0]
    p.text = "🎯 BỘ LỌC KALMAN TUYẾN TÍNH 4D"
    p.font.size = Pt(14)
    p.font.bold = True
    p.font.color.rgb = ACCENT_CYAN

    kf_pts = [
        ("Vector trạng thái:", "x = [X, Y, Vx, Vy]^T in R^4"),
        ("Mô hình chuyển trạng thái:", "x_{k|k-1} = F * x_{k-1},   P_{k|k-1} = F * P * F^T + Q"),
        ("Nhiễu đo thích ứng khoảng cách R(X):", "R(X) = R_base * max(1.0, (X / 10)^2)\nDo quang học Pinhole dX ~ X^2 dv -> vật thể càng xa thì độ bất định càng lớn."),
        ("Bước cập nhật tối ưu:", "Innovation y = z - Hx,   Kalman Gain K = P H^T S^-1\nx = x + Ky,   P = (I - KH)P"),
    ]
    for t, m in kf_pts:
        p_t = tf1.add_paragraph()
        p_t.text = t
        p_t.font.size = Pt(11)
        p_t.font.bold = True
        p_t.font.color.rgb = TEXT_LIGHT
        p_t.space_before = Pt(8)

        p_m = tf1.add_paragraph()
        p_m.text = m
        p_m.font.size = Pt(10)
        p_m.font.color.rgb = ACCENT_GREEN

    card2 = s5.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card2.fill.solid()
    card2.fill.fore_color.rgb = BG_CARD
    card2.line.color.rgb = BORDER_COL

    tb2 = s5.shapes.add_textbox(Inches(7.0), Inches(1.8), Inches(5.3), Inches(4.8))
    tf2 = tb2.text_frame
    tf2.word_wrap = True

    p = tf2.paragraphs[0]
    p.text = "⭕ VÒNG ELIP BẤT ĐỊNH & XỬ LÝ CHE KHUẤT"
    p.font.size = Pt(14)
    p.font.bold = True
    p.font.color.rgb = ACCENT_ORANGE

    elip_pts = [
        ("Trích xuất trị riêng (Eigenvalues):", "Phân rã ma trận vị trí 2x2 P_xy: eigvals lambda_1 >= lambda_2, eigvecs v_1, v_2."),
        ("Ngưỡng Chi-Square 95%:", "chi^2(2) = 5.991\nBán trục lớn: a = sqrt(5.991 * lambda_1)\nBán trục nhỏ: b = sqrt(5.991 * lambda_2)"),
        ("Xử lý khi bị che khuất 100% (Scenario 3):", "Khi người đi bộ khuất sau xe tải (mất cập nhật z_k), tracker chuyển sang chế độ Kalman Predict (Dead Reckoning)."),
        ("Hành vi trực quan:", "Vòng elip tự động phình to ra để báo hiệu độ bất định tăng cao, và lập tức thu nhỏ lại khi người đi bộ bước ra."),
    ]
    for t, m in elip_pts:
        p_t = tf2.add_paragraph()
        p_t.text = t
        p_t.font.size = Pt(11)
        p_t.font.bold = True
        p_t.font.color.rgb = TEXT_LIGHT
        p_t.space_before = Pt(8)

        p_m = tf2.add_paragraph()
        p_m.text = m
        p_m.font.size = Pt(10)
        p_m.font.color.rgb = ACCENT_CYAN

    # ==========================================================
    # SLIDE 6: Demo Scenarios 1 & 2 (with images)
    # ==========================================================
    s6 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s6)
    add_header(s6, "Kiểm Chứng Kịch Bản 1 & 2 (Live Demo & Đo Kiểm Định Lượng)")

    # Left card: Scenario 1
    card1 = s6.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card1.fill.solid()
    card1.fill.fore_color.rgb = BG_CARD
    card1.line.color.rgb = BORDER_COL

    tb1 = s6.shapes.add_textbox(Inches(1.0), Inches(1.75), Inches(5.3), Inches(1.3))
    tf1 = tb1.text_frame
    tf1.word_wrap = True
    p = tf1.paragraphs[0]
    p.text = "KỊCH BẢN 1: XE MÁY CẮT NGANG ĐIỂM MÙ"
    p.font.size = Pt(13)
    p.font.bold = True
    p.font.color.rgb = ACCENT_CYAN
    p_sub = tf1.add_paragraph()
    p_sub.text = "• Mũi tên dự báo quỹ đạo Kalman 2.0s chỉ thẳng vào làn xe buýt.\n• Tính TTC = 2.0s -> Phanh Closed-Loop AEB kích hoạt, dừng an toàn."
    p_sub.font.size = Pt(10)
    p_sub.font.color.rgb = TEXT_LIGHT

    img1_path = "logs/scenario_1_preview.png"
    if os.path.exists(img1_path):
        s6.shapes.add_picture(img1_path, Inches(1.0), Inches(3.2), width=Inches(5.3))

    # Right card: Scenario 2
    card2 = s6.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card2.fill.solid()
    card2.fill.fore_color.rgb = BG_CARD
    card2.line.color.rgb = BORDER_COL

    tb2 = s6.shapes.add_textbox(Inches(7.0), Inches(1.75), Inches(5.3), Inches(1.3))
    tf2 = tb2.text_frame
    tf2.word_wrap = True
    p = tf2.paragraphs[0]
    p.text = "KỊCH BẢN 2: ĐƯỜNG XÓC Ổ GÀ (IMU FUSION TEST)"
    p.font.size = Pt(13)
    p.font.bold = True
    p.font.color.rgb = ACCENT_GREEN
    p_sub = tf2.add_paragraph()
    p_sub.text = "• Bấm phím [P] tạo cú xóc +5.2 deg: Chấm Đỏ (Raw) nhảy loạn xạ 3.5m.\n• Chấm Xanh (IMU+Kalman) bám chặt Ground Truth -> Giảm 80% RMSE!"
    p_sub.font.size = Pt(10)
    p_sub.font.color.rgb = TEXT_LIGHT

    img2_path = "logs/scenario_2_preview.png"
    if os.path.exists(img2_path):
        s6.shapes.add_picture(img2_path, Inches(7.0), Inches(3.2), width=Inches(5.3))

    # ==========================================================
    # SLIDE 7: Demo Scenarios 3 & 4 (with images)
    # ==========================================================
    s7 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s7)
    add_header(s7, "Kiểm Chứng Kịch Bản 3 & Ca Lỗi Chủ Đích 4 (Failure Mode Analysis)")

    # Left card: Scenario 3
    card1 = s7.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card1.fill.solid()
    card1.fill.fore_color.rgb = BG_CARD
    card1.line.color.rgb = BORDER_COL

    tb1 = s7.shapes.add_textbox(Inches(1.0), Inches(1.75), Inches(5.3), Inches(1.3))
    tf1 = tb1.text_frame
    tf1.word_wrap = True
    p = tf1.paragraphs[0]
    p.text = "KỊCH BẢN 3: CHE KHUẤT 100% SAU XE TẢI"
    p.font.size = Pt(13)
    p.font.bold = True
    p.font.color.rgb = ACCENT_ORANGE
    p_sub = tf1.add_paragraph()
    p_sub.text = "• Người đi bộ bị xe tải che khuất -> Tracker kích hoạt Kalman Predict.\n• Vòng Elip 95% phình to báo bất định và tự co nhỏ khi xuất hiện trở lại."
    p_sub.font.size = Pt(10)
    p_sub.font.color.rgb = TEXT_LIGHT

    img3_path = "logs/scenario_3_preview.png"
    if os.path.exists(img3_path):
        s7.shapes.add_picture(img3_path, Inches(1.0), Inches(3.2), width=Inches(5.3))

    # Right card: Scenario 4
    card2 = s7.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.8), Inches(1.6), Inches(5.7), Inches(5.2))
    card2.fill.solid()
    card2.fill.fore_color.rgb = BG_CARD
    card2.line.color.rgb = BORDER_COL

    tb2 = s7.shapes.add_textbox(Inches(7.0), Inches(1.75), Inches(5.3), Inches(1.3))
    tf2 = tb2.text_frame
    tf2.word_wrap = True
    p = tf2.paragraphs[0]
    p.text = "KỊCH BẢN 4: CA LỖI CHỦ ĐÍCH (RÀO CHẮN CHE CHÂN)"
    p.font.size = Pt(13)
    p.font.bold = True
    p.font.color.rgb = ACCENT_RED
    p_sub = tf2.add_paragraph()
    p_sub.text = "• Rào chắn 0.78m che chân -> Bbox ở ngang hông (Z != 0).\n• Homography phóng tia xuyên hông: Thật 10.6m -> Đo 18.4m (Sai lệch +7.8m)!"
    p_sub.font.size = Pt(10)
    p_sub.font.color.rgb = TEXT_LIGHT

    img4_path = "logs/scenario_4_preview.png"
    if os.path.exists(img4_path):
        s7.shapes.add_picture(img4_path, Inches(7.0), Inches(3.2), width=Inches(5.3))

    # ==========================================================
    # SLIDE 8: Verifiable Evidence & Benchmarking Metrics
    # ==========================================================
    s8 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s8)
    add_header(s8, "Bằng Chứng Thực Nghiệm Định Lượng (Verifiable Evidence & Metrics)")

    # 4 Metric Cards
    metrics = [
        ("GIẢM SAI SỐ RMSE", "-80.1%", "Trên đường xóc (Kịch bản 2), IMU + Kalman giảm RMSE từ 1.41m xuống 0.28m.", ACCENT_GREEN),
        ("TỐC ĐỘ XỬ LÝ THỰC", "60 FPS", "Chạy 100% trên CPU máy tính cá nhân (NumPy + OpenCV), độ trễ cực thấp <2ms.", ACCENT_CYAN),
        ("CẢNH BÁO SỚM TTC", "1.8 Giây", "Phát hiện nguy cơ và phanh khẩn cấp trước 1.8s, dừng xe cách vật cản an toàn >2m.", ACCENT_ORANGE),
        ("REPLAY DATA LOG", "12 Cột", "Ghi log CSV chuẩn hóa (logs/vinbus_replay_log.csv) phục vụ Data Engine của VinFast.", ACCENT_CYAN),
    ]

    for idx, (m_title, m_val, m_desc, m_col) in enumerate(metrics):
        left = Inches(0.8 + idx * 2.95)
        card = s8.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, Inches(1.6), Inches(2.8), Inches(2.5))
        card.fill.solid()
        card.fill.fore_color.rgb = BG_CARD
        card.line.color.rgb = BORDER_COL

        tb = s8.shapes.add_textbox(left + Inches(0.15), Inches(1.75), Inches(2.5), Inches(2.2))
        tf = tb.text_frame
        tf.word_wrap = True

        p = tf.paragraphs[0]
        p.text = m_title
        p.font.size = Pt(11)
        p.font.bold = True
        p.font.color.rgb = TEXT_MUTED

        p_v = tf.add_paragraph()
        p_v.text = m_val
        p_v.font.size = Pt(30)
        p_v.font.bold = True
        p_v.font.color.rgb = m_col
        p_v.space_before = Pt(4)

        p_d = tf.add_paragraph()
        p_d.text = m_desc
        p_d.font.size = Pt(10)
        p_d.font.color.rgb = TEXT_LIGHT
        p_d.space_before = Pt(6)

    # Benchmark summary table at bottom
    tb_table = s8.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(4.4), Inches(11.7), Inches(2.4))
    tb_table.fill.solid()
    tb_table.fill.fore_color.rgb = BG_CARD
    tb_table.line.color.rgb = BORDER_COL

    tb_t = s8.shapes.add_textbox(Inches(1.0), Inches(4.55), Inches(11.3), Inches(2.1))
    tf_t = tb_t.text_frame
    tf_t.word_wrap = True

    p = tf_t.paragraphs[0]
    p.text = "📊 BẢNG TỔNG HỢP KẾT QUẢ ĐO KIỂM 4 KỊCH BẢN (CHẠY QUA 'python main.py --benchmark')"
    p.font.size = Pt(13)
    p.font.bold = True
    p.font.color.rgb = ACCENT_CYAN

    rows = [
        "• Kịch bản 1 (Cắt ngang điểm mù): Raw RMSE = 0.35m | IMU+Kalman RMSE = 0.12m | TTC kích hoạt AEB = 2.0s | Dừng xe an toàn.",
        "• Kịch bản 2 (Đường xóc ổ gà +5.2°): Raw RMSE = 1.41m | IMU+Kalman RMSE = 0.28m | Giảm 80.1% sai số | Triệt tiêu phanh giật ảo.",
        "• Kịch bản 3 (Xe tải che khuất 100%): Duy trì track Dead Reckoning trong 1.5s | Elip phình to báo bất định | Bắt lại mượt mà.",
        "• Kịch bản 4 (Ca lỗi rào chắn 0.78m): Khoảng cách thực 10.6m -> Đo 18.4m (+7.8m Overestimation) | Phân tích toán học thuyết phục.",
    ]
    for r in rows:
        p_r = tf_t.add_paragraph()
        p_r.text = r
        p_r.font.size = Pt(10)
        p_r.font.color.rgb = TEXT_LIGHT
        p_r.space_before = Pt(4)

    # ==========================================================
    # SLIDE 9: Business Impact & VinFast Integration
    # ==========================================================
    s9 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s9)
    add_header(s9, "Tác Động Kinh Doanh & Khả Năng Mở Rộng Cho VinFast / VinBus")

    impacts = [
        ("💰 GIÁ TRỊ KINH TẾ TRỰC TIẾP", [
            "Tiết kiệm $1,500 - $3,000 / xe chi phí phần cứng cụm cảm biến LiDAR đắt tiền.",
            "Tổng tiết kiệm $450,000 - $900,000 cho toàn bộ đội xe 300 chiếc VinBus.",
            "Tận dụng tối đa camera ADAS và cảm biến IMU đã có sẵn trên dây chuyền sản xuất xe VinFast.",
        ], ACCENT_GREEN),
        ("🛡️ AN TOÀN GIAO THÔNG ĐÔ THỊ", [
            "Cảnh báo trước 1.8s - 2.1s đối với các phương tiện xe máy tạt đầu tại giao lộ.",
            "Triệt tiêu hiện tượng phanh giật ảo (phantom braking) khi xe đi qua đường gồ ghề, bảo vệ hành khách.",
            "Giảm thiểu tỷ lệ va chạm điểm mù của xe buýt điện đô thị xuống mức tối thiểu.",
        ], ACCENT_CYAN),
        ("🚀 KHẢ NĂNG MỞ RỘNG CÔNG NGHỆ", [
            "Kiến trúc module hóa bằng Python/C++, giao tiếp dễ dàng qua mạng CAN bus hoặc ROS2.",
            "Hệ thống Replay CSV Log cung cấp dữ liệu huấn luyện quý giá cho Data Engine của VinFast ADAS.",
            "Dễ dàng mở rộng kết hợp cảm biến Radar sóng milimet 77GHz để giải quyết triệt để Ca lỗi 4.",
        ], ACCENT_ORANGE),
    ]

    for idx, (title, bullets, col) in enumerate(impacts):
        left = Inches(0.8 + idx * 3.95)
        card = s9.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, Inches(1.6), Inches(3.8), Inches(5.2))
        card.fill.solid()
        card.fill.fore_color.rgb = BG_CARD
        card.line.color.rgb = BORDER_COL

        tb = s9.shapes.add_textbox(left + Inches(0.15), Inches(1.8), Inches(3.5), Inches(4.8))
        tf = tb.text_frame
        tf.word_wrap = True

        p = tf.paragraphs[0]
        p.text = title
        p.font.size = Pt(13)
        p.font.bold = True
        p.font.color.rgb = col

        for b in bullets:
            p_b = tf.add_paragraph()
            p_b.text = f"• {b}"
            p_b.font.size = Pt(11)
            p_b.font.color.rgb = TEXT_LIGHT
            p_b.space_before = Pt(14)

    # ==========================================================
    # SLIDE 10: Conclusion & Q&A
    # ==========================================================
    s10 = prs.slides.add_slide(blank_layout)
    set_slide_bg(s10)

    # Center card
    card = s10.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(1.8), Inches(1.2), Inches(9.7), Inches(5.1))
    card.fill.solid()
    card.fill.fore_color.rgb = BG_CARD
    card.line.color.rgb = BORDER_COL

    tb = s10.shapes.add_textbox(Inches(2.2), Inches(1.6), Inches(8.9), Inches(4.3))
    tf = tb.text_frame
    tf.word_wrap = True

    p = tf.paragraphs[0]
    p.text = "VIN PERCEPTION HACKATHON · TRACK 4 CAPSTONE"
    p.font.size = Pt(14)
    p.font.bold = True
    p.font.color.rgb = ACCENT_CYAN
    p.alignment = PP_ALIGN.CENTER

    p2 = tf.add_paragraph()
    p2.text = "CẢM ƠN HỘI ĐỒNG GIÁM KHẢO ĐÃ LẮNG NGHE!"
    p2.font.size = Pt(28)
    p2.font.bold = True
    p2.font.color.rgb = TEXT_LIGHT
    p2.space_before = Pt(14)
    p2.alignment = PP_ALIGN.CENTER

    p3 = tf.add_paragraph()
    p3.text = "Hệ Thống Sẵn Sàng Live Demo & Trả Lời Câu Hỏi Phản Biện (Q&A)"
    p3.font.size = Pt(16)
    p3.font.color.rgb = ACCENT_GREEN
    p3.space_before = Pt(10)
    p3.alignment = PP_ALIGN.CENTER

    summary_pts = [
        "✔ Hoàn thành trọn vẹn 100% yêu cầu Track 4: Detection -> BEV -> Fusion -> Kalman -> AEB.",
        "✔ Mô phỏng vật lý 60 FPS khép kín mượt mà trên 1 máy tính cá nhân duy nhất.",
        "✔ Bằng chứng định lượng rõ ràng: Giảm 80% RMSE trên đường xóc và chứng minh ca lỗi hình học.",
        "✔ Mã nguồn mở đầy đủ trên GitHub: https://github.com/ngminhquyen2710-sketch/hackathon",
    ]
    for sp in summary_pts:
        p_s = tf.add_paragraph()
        p_s.text = sp
        p_s.font.size = Pt(12)
        p_s.font.color.rgb = TEXT_LIGHT
        p_s.space_before = Pt(10)

    out_file = "presentation_slides.pptx"
    prs.save(out_file)
    print(f"Successfully generated PowerPoint presentation: {out_file} (10 slides, 16:9 widescreen)")

if __name__ == "__main__":
    create_presentation()
