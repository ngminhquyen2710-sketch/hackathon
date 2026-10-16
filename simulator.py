"""
Day 11: Synthetic, Simulated & Trajectory Data
3D Physical World & Pinhole Camera + IMU Suspension Simulator for VinBus / VinFast.
Runs 100% on a single laptop using NumPy & OpenCV (60 FPS, zero external hardware required).
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np
import cv2


@dataclass
class SimObject3D:
    obj_id: int
    label: str  # 'pedestrian', 'motorbike', 'occluder_box'
    x: float    # Longitudinal distance ahead of ego front bumper (meters, +X is forward)
    y: float    # Lateral position relative to ego center (meters, +Y is right, -Y is left)
    z: float    # Base height above ground (meters, 0.0 for ground objects)
    vx: float   # World velocity X (m/s)
    vy: float   # World velocity Y (m/s)
    width: float   # Lateral width (m)
    depth: float   # Longitudinal depth (m)
    height: float  # Vertical height (m)
    color_bgr: Tuple[int, int, int]
    is_occluder: bool = False


@dataclass
class Detection2D:
    obj_id: int
    label: str
    bbox: Tuple[int, int, int, int]  # (u1, v1, u2, v2) in image pixels
    bottom_center: Tuple[float, float]  # (u_bc, v_bc) pixel used for BEV projection
    confidence: float
    is_partially_occluded: bool = False
    visible_bottom_z: float = 0.0  # True height of visible bottom edge (0.0 normally, >0 in Failure Case!)


class VinBusSimulator:
    """
    Simulates a VinBus electric bus equipped with:
    - Front/Right Blind-Spot Wide-Angle RGB Camera (Pinhole model with Intrinsics K & Extrinsics R, t)
    - 6-Axis IMU (Measures dynamic pitch angle caused by potholes / speed bumps & braking dive)
    - Closed-Loop Longitudinal Dynamics (Speed, AEB Braking, Suspension Pitch Oscillator)
    """

    def __init__(self, img_w: int = 640, img_h: int = 420):
        self.img_w = img_w
        self.img_h = img_h

        # Camera Intrinsics (Wide-angle ADAS camera)
        self.fx = 460.0
        self.fy = 460.0
        self.cx = img_w / 2.0
        self.cy = img_h / 2.0
        self.K = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Camera Mounting Extrinsics on VinBus (Height = 1.65m, Nominal Pitch = 10 deg downward)
        self.cam_height_nominal = 1.65  # meters above ground
        self.cam_pitch_nominal = math.radians(10.0)  # +pitch = looking downward toward road

        # Ego Vehicle State
        self.ego_speed = 8.0          # m/s (~28.8 km/h urban bus speed)
        self.target_speed = 8.0       # m/s
        self.ego_accel = 0.0          # m/s^2
        self.ego_world_x = 0.0        # Total distance traveled (for road stripes animation)
        self.aeb_active = False       # Autonomous Emergency Braking status

        # Suspension Pitch Dynamics (Damped Harmonic Oscillator for Potholes / Braking Dive)
        self.pitch_offset = 0.0       # Dynamic pitch perturbation delta_theta (radians)
        self.pitch_vel = 0.0          # Angular velocity (rad/s)
        self.imu_pitch_measured = 0.0 # IMU reading (pitch_offset + sensor noise)

        # Scenario Management
        self.scenario_name = "1: Normal Blind-Spot Cut-in"
        self.failure_case_active = False
        self.pothole_timer = 0.0
        self.auto_pothole = True
        self.sim_time = 0.0
        self.objects: List[SimObject3D] = []

        self.load_scenario(1)

    def load_scenario(self, scenario_id: int):
        """
        Scenarios:
        1: Normal Traffic + Motorbike Blind-Spot Cut-In & Pedestrian Crossing
        2: Rough Road (Frequent Potholes / Speed Bumps shaking Camera Pitch)
        3: Full Occlusion Test (Pedestrian walks behind a parked delivery van -> tests Kalman Predict)
        4: FAILURE CASE (Lower-Body Occlusion / Elevated Box -> breaks Flat-Ground Homography assumption!)
        """
        self.objects.clear()
        self.ego_speed = 8.0
        self.aeb_active = False
        self.failure_case_active = (scenario_id == 4)

        if scenario_id == 1:
            self.scenario_name = "1: Motorbike Cut-in & Crossing"
            self.auto_pothole = False
            # Motorbike cutting in from right blind spot toward ego lane
            self.objects.append(SimObject3D(
                obj_id=101, label="motorbike",
                x=18.0, y=3.8, z=0.0,
                vx=-4.5, vy=-0.85,  # Relative closing speed
                width=0.8, depth=1.8, height=1.45,
                color_bgr=(40, 165, 255)  # Orange motorbike
            ))
            # Pedestrian walking from left to right further ahead
            self.objects.append(SimObject3D(
                obj_id=102, label="pedestrian",
                x=22.0, y=-3.2, z=0.0,
                vx=-3.0, vy=1.1,
                width=0.6, depth=0.6, height=1.72,
                color_bgr=(80, 230, 100)  # Green pedestrian
            ))

        elif scenario_id == 2:
            self.scenario_name = "2: Pothole Shock (IMU Fusion Test)"
            self.auto_pothole = True
            self.trigger_pothole(intensity_deg=5.2)
            # Pedestrian crossing steadily while bus hits potholes
            self.objects.append(SimObject3D(
                obj_id=201, label="pedestrian",
                x=15.0, y=2.2, z=0.0,
                vx=-2.2, vy=-0.45,
                width=0.65, depth=0.65, height=1.75,
                color_bgr=(80, 230, 100)
            ))
            self.objects.append(SimObject3D(
                obj_id=202, label="motorbike",
                x=21.0, y=-1.8, z=0.0,
                vx=-3.8, vy=0.25,
                width=0.8, depth=1.8, height=1.45,
                color_bgr=(40, 165, 255)
            ))

        elif scenario_id == 3:
            self.scenario_name = "3: Full Occlusion (Kalman Prediction)"
            self.auto_pothole = False
            # Obstacle barrier blocking view completely in the middle
            self.objects.append(SimObject3D(
                obj_id=900, label="occluder_van",
                x=11.5, y=1.3, z=0.0,
                vx=0.0, vy=0.0,
                width=1.6, depth=1.5, height=2.1,
                color_bgr=(110, 110, 125),
                is_occluder=True
            ))
            # Pedestrian walking from right to left behind the van
            self.objects.append(SimObject3D(
                obj_id=301, label="pedestrian",
                x=14.5, y=3.8, z=0.0,
                vx=-1.2, vy=-1.35,
                width=0.65, depth=0.65, height=1.75,
                color_bgr=(60, 240, 255)
            ))

        elif scenario_id == 4:
            self.scenario_name = "4: FAILURE CASE (Lower-Body Occlusion)"
            self.auto_pothole = False
            # Low concrete barrier (height 0.78m) at x=9.5m that hides the pedestrian's legs/feet!
            self.objects.append(SimObject3D(
                obj_id=901, label="low_barrier",
                x=9.5, y=0.2, z=0.0,
                vx=0.0, vy=0.0,
                width=4.2, depth=0.45, height=0.78,
                color_bgr=(90, 95, 145),
                is_occluder=True
            ))
            # Pedestrian walking laterally behind the low barrier (at x=10.6m > 9.5m)
            # Because legs (0..0.78m) are hidden behind the barrier, detector bbox bottom is at waist!
            self.objects.append(SimObject3D(
                obj_id=401, label="pedestrian",
                x=10.6, y=1.1, z=0.0,
                vx=-0.12, vy=-0.55,
                width=0.65, depth=0.65, height=1.75,
                color_bgr=(80, 230, 100)
            ))

    def trigger_pothole(self, intensity_deg: float = 4.8):
        """Injects an angular pitch impulse to simulate VinBus hitting a pothole / speed bump."""
        self.pitch_vel += math.radians(intensity_deg) * 6.5

    def step(self, dt: float, command_aeb: bool = False):
        """Advances physics simulation by dt seconds."""
        self.sim_time += dt
        self.aeb_active = command_aeb

        # 1. Ego Vehicle Longitudinal Dynamics (Closed-Loop AEB)
        if self.aeb_active:
            self.ego_accel = -6.5  # Hard emergency braking
        else:
            # Gently accelerate back toward target speed if path is clear
            self.ego_accel = 2.0 * (self.target_speed - self.ego_speed)

        prev_speed = self.ego_speed
        self.ego_speed = float(np.clip(self.ego_speed + self.ego_accel * dt, 0.0, 12.0))
        actual_accel = (self.ego_speed - prev_speed) / max(dt, 1e-4)
        self.ego_world_x += self.ego_speed * dt

        # 2. Periodic Pothole Trigger in Scenario 2
        if self.auto_pothole:
            self.pothole_timer += dt
            if self.pothole_timer > 2.4:
                self.pothole_timer = 0.0
                self.trigger_pothole(intensity_deg=4.8)

        # 3. Suspension Pitch Dynamics (Spring-Damper Oscillator + Braking Nose-Dive)
        wn = 14.0        # Natural frequency (rad/s)
        zeta = 0.22      # Underdamped suspension so camera oscillates visibly
        braking_torque = -0.025 * actual_accel  # Nose dives down (+pitch) when braking (accel < 0)
        pitch_acc = - (wn ** 2) * self.pitch_offset - 2.0 * zeta * wn * self.pitch_vel + braking_torque
        self.pitch_vel += pitch_acc * dt
        self.pitch_offset += self.pitch_vel * dt

        # Simulated IMU Sensor Output (True pitch offset + small Gaussian sensor noise)
        self.imu_pitch_measured = self.pitch_offset + np.random.normal(0.0, math.radians(0.12))

        # 4. Update Target Objects in Ego-Relative Frame
        # When ego slows down (AEB), relative closing speed decreases!
        speed_factor = self.ego_speed / 8.0
        for obj in self.objects:
            if not obj.is_occluder:
                # Adjust longitudinal motion by ego speed
                effective_vx = obj.vx * (0.35 + 0.65 * speed_factor)
                obj.x += effective_vx * dt
                obj.y += obj.vy * dt

                # Loop objects smoothly so demo never runs out of targets
                if self.failure_case_active:
                    if obj.x < 10.1 or obj.x > 11.5:
                        obj.vx = -obj.vx
                        obj.x = float(np.clip(obj.x, 10.15, 11.4))
                    if obj.y < -1.1 or obj.y > 1.4:
                        obj.vy = -obj.vy
                        obj.y = float(np.clip(obj.y, -1.05, 1.35))
                elif obj.x < 2.2 or obj.x > 28.0 or abs(obj.y) > 6.5:
                    obj.x = np.random.uniform(17.0, 23.0)
                    obj.y = np.random.choice([-1.0, 1.0]) * np.random.uniform(2.0, 4.0)
                    obj.vy = -0.45 * np.sign(obj.y) + np.random.uniform(-0.2, 0.2)

    def project_3d_to_pixel(self, X: float, Y: float, Z: float, total_pitch: float) -> Optional[Tuple[int, int]]:
        """
        Projects a 3D point in Ego Vehicle Frame:
          +X: forward (meters)
          +Y: right (meters)
          +Z: up from ground (meters)
        into 2D Camera Pixel coordinates (u, v) using Pinhole Camera Model with pitch angle `total_pitch`.
        """
        # Translate relative to camera optical center at (0, 0, cam_height)
        dx = X
        dy = Y
        dz = Z - self.cam_height_nominal

        # Camera pitch rotation (looking downward by total_pitch around Y_cam)
        cos_p = math.cos(total_pitch)
        sin_p = math.sin(total_pitch)

        # Standard camera axes: Z_cam = forward along optical axis, X_cam = right, Y_cam = down
        z_cam = dx * cos_p - dz * sin_p
        if z_cam <= 0.5:
            return None
        x_cam = dy
        y_cam = -dx * sin_p - dz * cos_p

        u = int(round(self.fx * (x_cam / z_cam) + self.cx))
        v = int(round(self.fy * (y_cam / z_cam) + self.cy))
        return (u, v)

    def render_camera_and_detect(self) -> Tuple[np.ndarray, List[Detection2D]]:
        """
        Renders the synthetic 3D Pinhole Camera RGB image and runs simulated 2D detection
        (including realistic bbox jitter, full occlusion dropout, and Failure Case partial lower-body occlusion).
        """
        img = np.zeros((self.img_h, self.img_w, 3), dtype=np.uint8)
        total_pitch = self.cam_pitch_nominal + self.pitch_offset

        # 1. Render Sky & Road Horizon shifted by dynamic camera pitch
        horizon_v = int(round(self.cy - self.fy * math.tan(total_pitch)))
        horizon_v_clamped = int(np.clip(horizon_v, 0, self.img_h - 1))

        # Sky gradient
        img[:horizon_v_clamped, :] = (52, 38, 28)
        # Road surface
        img[horizon_v_clamped:, :] = (46, 48, 52)

        # 2. Render 3D Ground Grid & Lane Markings in Perspective
        for x_grid in range(3, 30, 3):
            # Animate grid lines with ego motion
            x_moving = x_grid - (self.ego_world_x % 3.0)
            if x_moving < 2.0:
                x_moving += 3.0
            p1 = self.project_3d_to_pixel(x_moving, -6.0, 0.0, total_pitch)
            p2 = self.project_3d_to_pixel(x_moving, 6.0, 0.0, total_pitch)
            if p1 and p2:
                cv2.line(img, p1, p2, (62, 65, 70), 1, cv2.LINE_AA)

        # Lane lines (Y = -1.75m, +1.75m, +4.5m blind-spot lane)
        for y_lane, col, thick in [(-1.75, (190, 190, 190), 2), (1.75, (80, 200, 245), 2), (4.5, (120, 120, 140), 1)]:
            pts = []
            for x_step in np.linspace(2.0, 28.0, 25):
                p = self.project_3d_to_pixel(float(x_step), y_lane, 0.0, total_pitch)
                if p:
                    pts.append(p)
            if len(pts) >= 2:
                cv2.polylines(img, [np.array(pts, dtype=np.int32)], False, col, thick, cv2.LINE_AA)

        # 3. Sort 3D Objects Back-to-Front (Painter's Algorithm) & Check Occlusions
        sorted_objs = sorted(self.objects, key=lambda o: o.x, reverse=True)
        occluder_boxes_2d = []  # Stores (u1, v1, u2, v2, obj_3d) of occluding barriers/vans
        detections: List[Detection2D] = []

        # Precompute 2D projections
        rendered_items = []
        for obj in sorted_objs:
            corners_3d = [
                (obj.x - obj.depth / 2, obj.y - obj.width / 2, obj.z),
                (obj.x - obj.depth / 2, obj.y + obj.width / 2, obj.z),
                (obj.x + obj.depth / 2, obj.y + obj.width / 2, obj.z),
                (obj.x + obj.depth / 2, obj.y - obj.width / 2, obj.z),
                (obj.x - obj.depth / 2, obj.y - obj.width / 2, obj.z + obj.height),
                (obj.x - obj.depth / 2, obj.y + obj.width / 2, obj.z + obj.height),
                (obj.x + obj.depth / 2, obj.y + obj.width / 2, obj.z + obj.height),
                (obj.x + obj.depth / 2, obj.y - obj.width / 2, obj.z + obj.height),
            ]
            proj_pts = [self.project_3d_to_pixel(cx, cy, cz, total_pitch) for (cx, cy, cz) in corners_3d]
            valid_pts = [p for p in proj_pts if p is not None]
            if len(valid_pts) < 4:
                continue

            us = [p[0] for p in valid_pts]
            vs = [p[1] for p in valid_pts]
            u1, u2 = int(min(us)), int(max(us))
            v1, v2 = int(min(vs)), int(max(vs))

            # Skip if completely outside image
            if u2 < 0 or u1 >= self.img_w or v2 < 0 or v1 >= self.img_h:
                continue

            rendered_items.append((obj, u1, v1, u2, v2))
            if obj.is_occluder:
                occluder_boxes_2d.append((u1, v1, u2, v2, obj))

        # Draw objects back-to-front
        for (obj, u1, v1, u2, v2) in rendered_items:
            # Draw 3D-like shaded body in camera view
            cv2.rectangle(img, (u1, v1), (u2, v2), obj.color_bgr, -1)
            cv2.rectangle(img, (u1, v1), (u2, v2), (230, 230, 230), 1)

            if obj.is_occluder:
                cv2.putText(img, obj.label.upper(), (u1 + 6, (v1 + v2) // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)
                continue

            # Draw head & torso silhouette for pedestrians/motorbikes
            head_r = max(4, (u2 - u1) // 5)
            cv2.circle(img, ((u1 + u2) // 2, v1 + head_r + 2), head_r, (240, 240, 240), -1)

        # 4. Run 2D Detector Logic (Checking Full or Lower-Body Occlusion by foreground barriers)
        for (obj, u1, v1, u2, v2) in rendered_items:
            if obj.is_occluder:
                continue

            det_u1, det_v1, det_u2, det_v2 = u1, v1, u2, v2
            fully_occluded = False
            partially_occluded = False
            visible_bottom_z = 0.0

            for (ou1, ov1, ou2, ov2, occ_obj) in occluder_boxes_2d:
                # Only foreground objects (smaller X) can occlude background objects
                if occ_obj.x >= obj.x:
                    continue

                # Horizontal overlap check
                horiz_overlap = min(det_u2, ou2) - max(det_u1, ou1)
                if horiz_overlap > 0.45 * (det_u2 - det_u1):
                    # Case A: Tall occluder (e.g. Van in Scenario 3) blocks >= 70% height -> Full Occlusion!
                    if ov1 <= det_v1 + 0.3 * (det_v2 - det_v1) and ov2 >= det_v2 - 5:
                        fully_occluded = True
                        break
                    # Case B: Low barrier (Scenario 4 Failure Case) hides the feet/legs!
                    elif ov1 > det_v1 and ov2 >= det_v2 - 8:
                        partially_occluded = True
                        # Detector only sees the visible upper body above the barrier top edge (ov1)!
                        det_v2 = ov1
                        visible_bottom_z = occ_obj.height

            if fully_occluded:
                continue

            # Add realistic 2D detector pixel noise (~1.5 px std)
            noise_u = np.random.normal(0.0, 1.2)
            noise_v = np.random.normal(0.0, 1.5)
            u_bc = 0.5 * (det_u1 + det_u2) + noise_u
            v_bc = float(det_v2) + noise_v

            detections.append(Detection2D(
                obj_id=obj.obj_id,
                label=obj.label,
                bbox=(det_u1, det_v1, det_u2, int(round(v_bc))),
                bottom_center=(u_bc, v_bc),
                confidence=0.82 if partially_occluded else 0.94,
                is_partially_occluded=partially_occluded,
                visible_bottom_z=visible_bottom_z
            ))

        return img, detections
