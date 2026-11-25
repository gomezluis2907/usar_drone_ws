"""Dual-operator USAR Ground Control Station HUD (Pilot FPV + Thermal AI + Tactical Map)."""

from collections import deque
import json
import math
import os
from typing import Deque, Dict, List, Optional, Tuple

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from sar_drone_sim.usar_common import (
    COMMAND_POST_POS,
    DISASTER_SECTORS,
    HAZARDS,
    HELIPAD_POS,
    SAFE_PERIMETER_RADIUS_M,
)

PALETTES = [
    ('INFERNO', cv2.COLORMAP_INFERNO),
    ('WHITE-HOT', None),
    ('TURBO', cv2.COLORMAP_TURBO),
]


class GroundControlHUD(Node):
    """Renders 3 tactical panels: Pilot FPV, Observer Thermal AI, and USAR Map/Log."""

    PANEL_W = 520
    PANEL_H = 600
    WIN_UNIFIED = 'DRON USAR - Estacion Terrestre (Piloto FPV | Observador Termico | Mapa Tactico)'
    WIN_PILOT = 'DRON USAR - Piloto FPV (C07)'
    WIN_OBSERVER = 'DRON USAR - Observador Termico & Mapa Tactico (C08 / C16)'

    def __init__(self) -> None:
        super().__init__('gcs_hud')

        self.declare_parameter('show_window', True)
        self.declare_parameter('dual_windows', False)

        self.show_window = bool(self.get_parameter('show_window').value) and bool(
            os.environ.get('DISPLAY')
        )
        self.dual_windows = bool(self.get_parameter('dual_windows').value)

        self.bridge = CvBridge()
        self.latest_fpv: Optional[np.ndarray] = None
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_thermal_c: Optional[np.ndarray] = None

        self.status: Dict = {
            'mode': 'STANDBY',
            'rth_stage': 'NONE',
            'rth_reason': '',
            'battery_pct': 100.0,
            'battery_num': 1,
            'mission_elapsed_s': 0.0,
            'swapping_battery': False,
            'link_state': 'PRIMARY',
            'rssi_pct': 100.0,
            'distance_home_m': 0.0,
            'altitude_m': 0.14,
            'speed_mps': 0.0,
            'gimbal_pitch_rad': 0.48,
            'min_safe_alt_m': 2.2,
            'low_alt_warning': False,
            'gust_active': False,
            'orbit_target_id': None,
            'pos': [0.0, 0.0, 0.14],
            'rpy': [0.0, 0.0, 0.0],
        }
        self.detections: List[Dict] = []
        self.anomalies_count = 0
        self.findings: Dict = {
            'session_dir': '',
            'tracks': [],
            'anomalies_world': [],
            'sd_frames_saved': 0,
            'last_event': 'Esperando telemetria...',
            'counts': {'total': 0, 'suspect': 0, 'ready': 0, 'confirmed': 0, 'pending_cp': 0},
        }

        self.trail: Deque[Tuple[float, float]] = deque(maxlen=400)
        self.palette_idx = 0

        self.gcs_pub = self.create_publisher(Image, '/sar_drone/gcs/image', 10)

        self.create_subscription(Image, '/sar_drone/fpv/image_raw', self._fpv_cb, 10)
        self.create_subscription(Image, '/sar_drone/rgb/image_raw', self._rgb_cb, 10)
        self.create_subscription(Image, '/sar_drone/thermal/image_raw', self._thermal_cb, 10)
        self.create_subscription(Odometry, '/sar_drone/odom', self._odom_cb, 10)
        self.create_subscription(String, '/sar_drone/status', self._status_cb, 10)
        self.create_subscription(String, '/sar_drone/detections', self._det_cb, 10)
        self.create_subscription(String, '/sar_drone/findings', self._findings_cb, 10)
        self.create_subscription(String, '/sar_drone/mission_cmd', self._cmd_cb, 10)

        self.render_timer = self.create_timer(1.0 / 12.0, self._render_and_publish)
        self.get_logger().info(
            f'GroundControlHUD activo [show_window={self.show_window}, dual_windows={self.dual_windows}]'
        )

    def _fpv_cb(self, msg: Image) -> None:
        try:
            self.latest_fpv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _rgb_cb(self, msg: Image) -> None:
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _thermal_cb(self, msg: Image) -> None:
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if raw.dtype == np.uint16:
                self.latest_thermal_c = raw.astype(np.float32) * 0.01 - 273.15
            elif raw.dtype == np.uint8:
                self.latest_thermal_c = raw.astype(np.float32) * (100.0 / 255.0)
            else:
                self.latest_thermal_c = raw.astype(np.float32)
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= 0.25:
            self.trail.append((x, y))

    def _status_cb(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def _det_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.detections = list(data.get('detections', []))
            self.anomalies_count = int(data.get('anomalies_count', 0))
        except Exception:
            pass

    def _findings_cb(self, msg: String) -> None:
        try:
            self.findings = json.loads(msg.data)
        except Exception:
            pass

    def _cmd_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if data.get('cmd') == 'palette_next':
                self.palette_idx = (self.palette_idx + 1) % len(PALETTES)
        except Exception:
            pass

    def _draw_header(self, img: np.ndarray, title: str, accent: Tuple[int, int, int]) -> None:
        cv2.rectangle(img, (0, 0), (self.PANEL_W, 34), (24, 28, 34), -1)
        cv2.line(img, (0, 34), (self.PANEL_W, 34), accent, 2)
        cv2.putText(
            img, title, (12, 23),
            cv2.FONT_HERSHEY_SIMPLEX, 0.54, (240, 245, 250), 1, cv2.LINE_AA,
        )

    def _render_pilot_panel(self) -> np.ndarray:
        w, h = self.PANEL_W, self.PANEL_H
        src = self.latest_fpv if self.latest_fpv is not None else self.latest_rgb
        if src is not None:
            panel = cv2.resize(src, (w, h))
        else:
            panel = np.full((h, w, 3), (26, 30, 36), dtype=np.uint8)
            cv2.putText(
                panel, 'ESPERANDO VIDEO FPV...', (130, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (170, 180, 190), 2, cv2.LINE_AA,
            )

        # Subtle dark vignette at top/bottom for crisp HUD readability
        panel[0:70] = (panel[0:70].astype(np.float32) * 0.55).astype(np.uint8)
        panel[h - 75 : h] = (panel[h - 75 : h].astype(np.float32) * 0.55).astype(np.uint8)

        link_state = str(self.status.get('link_state', 'PRIMARY'))
        mode = str(self.status.get('mode', 'STANDBY'))
        if link_state == 'LOST':
            noise = np.random.randint(0, 65, (h, w, 3), dtype=np.uint8)
            panel = cv2.addWeighted(panel, 0.55, noise, 0.45, 0)

        self._draw_header(panel, 'PILOTO FPV (C07) | ENLACE EN VIVO', (40, 190, 255))

        # Compass tape at top
        rpy = self.status.get('rpy', [0.0, 0.0, 0.0])
        roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
        hdg = (math.degrees(-yaw) + 360.0) % 360.0
        cv2.rectangle(panel, (w // 2 - 42, 40), (w // 2 + 42, 64), (15, 20, 25), -1)
        cv2.rectangle(panel, (w // 2 - 42, 40), (w // 2 + 42, 64), (80, 220, 120), 1)
        cv2.putText(
            panel, f'HDG {hdg:03.0f}', (w // 2 - 34, 57),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 255, 160), 1, cv2.LINE_AA,
        )

        # Artificial horizon in center
        cx, cy = w // 2, h // 2
        pitch_px = int(pitch * 180.0)
        dx = int(math.cos(roll) * 85.0)
        dy = int(math.sin(roll) * 85.0)
        cv2.line(panel, (cx - dx, cy + pitch_px - dy), (cx + dx, cy + pitch_px + dy), (80, 255, 140), 2)
        cv2.line(panel, (cx - 20, cy), (cx - 6, cy), (255, 255, 255), 2)
        cv2.line(panel, (cx + 6, cy), (cx + 20, cy), (255, 255, 255), 2)
        cv2.circle(panel, (cx, cy), 3, (0, 220, 255), -1)

        # Speed & Altitude side readouts
        alt = float(self.status.get('altitude_m', 0.0))
        spd = float(self.status.get('speed_mps', 0.0))
        dist_h = float(self.status.get('distance_home_m', 0.0))
        gimbal_deg = math.degrees(float(self.status.get('gimbal_pitch_rad', 0.48)))

        cv2.rectangle(panel, (12, 260), (112, 330), (15, 20, 25), -1)
        cv2.rectangle(panel, (12, 260), (112, 330), (90, 200, 255), 1)
        cv2.putText(panel, 'VEL m/s', (18, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 210, 255), 1)
        cv2.putText(panel, f'{spd:4.1f}', (22, 316), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (255, 255, 255), 2)

        alt_col = (50, 70, 255) if self.status.get('low_alt_warning') else (90, 255, 140)
        cv2.rectangle(panel, (w - 114, 260), (w - 12, 330), (15, 20, 25), -1)
        cv2.rectangle(panel, (w - 114, 260), (w - 12, 330), alt_col, 1)
        cv2.putText(panel, 'ALT m', (w - 104, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.42, alt_col, 1)
        cv2.putText(panel, f'{alt:4.1f}', (w - 102, 316), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (255, 255, 255), 2)

        # Warning banners
        y_warn = 78
        if self.status.get('low_alt_warning'):
            cv2.rectangle(panel, (95, y_warn), (w - 95, y_warn + 26), (25, 30, 190), -1)
            cv2.putText(
                panel, 'ALERTA: ALTITUD BAJA SOBRE ESCOMBROS (C05)', (108, y_warn + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1, cv2.LINE_AA,
            )
            y_warn += 30
        if self.status.get('gust_active'):
            cv2.rectangle(panel, (130, y_warn), (w - 130, y_warn + 24), (20, 110, 210), -1)
            cv2.putText(
                panel, 'RAFAGA DE VIENTO DETECTADA', (148, y_warn + 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.43, (255, 255, 255), 1, cv2.LINE_AA,
            )
            y_warn += 28
        if link_state == 'LOST':
            cv2.rectangle(panel, (52, h // 2 + 52), (w - 52, h // 2 + 94), (20, 20, 190), -1)
            cv2.putText(
                panel, 'ENLACE PERDIDO - RTH ACTIVO | GRABANDO SD', (66, h // 2 + 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2, cv2.LINE_AA,
            )

        # Bottom telemetry bar
        bat_pct = float(self.status.get('battery_pct', 100.0))
        bat_num = int(self.status.get('battery_num', 1))
        bat_col = (60, 240, 90) if bat_pct > 30 else ((0, 180, 255) if bat_pct > 15 else (40, 40, 255))
        link_col = (60, 240, 90) if link_state == 'PRIMARY' else ((0, 210, 255) if link_state == 'BACKUP' else (40, 40, 255))

        cv2.putText(
            panel, f'MODO: {mode}', (14, h - 42),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 230, 110), 1, cv2.LINE_AA,
        )
        cv2.putText(
            panel, f'GIMBAL: {gimbal_deg:.0f} deg  |  HOME: {dist_h:.1f}m', (14, h - 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.46, (215, 225, 235), 1, cv2.LINE_AA,
        )
        cv2.putText(
            panel, f'BAT #{bat_num}: {bat_pct:5.1f}%', (w - 170, h - 42),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, bat_col, 1, cv2.LINE_AA,
        )
        rssi = float(self.status.get('rssi_pct', 100.0))
        cv2.putText(
            panel, f'LINK: {link_state} ({rssi:.0f}%)', (w - 195, h - 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.46, link_col, 1, cv2.LINE_AA,
        )
        return panel

    def _render_observer_panel(self) -> np.ndarray:
        w, h = self.PANEL_W, self.PANEL_H
        panel = np.full((h, w, 3), (18, 22, 28), dtype=np.uint8)
        pal_name, pal_cmap = PALETTES[self.palette_idx]

        # Thermal view area: 520 x 390 below 34px header, footer below
        th_y0, th_h = 42, 390
        if self.latest_thermal_c is not None:
            temp_c = self.latest_thermal_c
            norm = np.clip((temp_c - 15.0) / (45.0 - 15.0), 0.0, 1.0)
            gray = (norm * 255.0).astype(np.uint8)
            if pal_cmap is None:
                colored = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                colored = cv2.applyColorMap(gray, pal_cmap)
            th_view = cv2.resize(colored, (w, th_h), interpolation=cv2.INTER_NEAREST)

            sx = w / float(temp_c.shape[1])
            sy = th_h / float(temp_c.shape[0])
            status_colors = {
                'SUSPECT': (0, 220, 255),
                'READY_CONFIRM': (255, 220, 40),
                'CONFIRMED': (60, 255, 90),
            }
            for det in self.detections:
                x, y, bw, bh = det['bbox']
                p1 = (int(x * sx), int(y * sy))
                p2 = (int((x + bw) * sx), int((y + bh) * sy))
                st = det.get('track_status', 'SUSPECT')
                col = status_colors.get(st, (60, 255, 90))
                cv2.rectangle(th_view, p1, p2, col, 2)
                tid = det.get('track_id', f"#{det['id']}")
                d_ang = det.get('angle_span_deg', 0.0)
                lbl = f"{tid} {det['max_temp_c']:.1f}C {int(det['confidence']*100)}% dA:{d_ang:.0f}"
                ty = max(18, p1[1] - 6)
                cv2.rectangle(th_view, (p1[0], ty - 14), (min(w - 4, p1[0] + len(lbl) * 7 + 6), ty + 3), (15, 20, 20), -1)
                cv2.putText(th_view, lbl, (p1[0] + 3, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.39, col, 1, cv2.LINE_AA)

            # Center spot temperature
            ctemp = float(temp_c[temp_c.shape[0] // 2, temp_c.shape[1] // 2])
            cv2.drawMarker(th_view, (w // 2, th_h // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
            cv2.putText(
                th_view, f'{ctemp:.1f}C', (w // 2 + 10, th_h // 2 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA,
            )

            # PiP RGB reference from gimbal in bottom-left
            if self.latest_rgb is not None:
                pip_w, pip_h = 160, 120
                pip = cv2.resize(self.latest_rgb, (pip_w, pip_h))
                x0, y0 = 10, th_h - pip_h - 10
                th_view[y0 : y0 + pip_h, x0 : x0 + pip_w] = pip
                cv2.rectangle(th_view, (x0, y0), (x0 + pip_w, y0 + pip_h), (210, 220, 230), 1)
                cv2.putText(
                    th_view, 'RGB GIMBAL (REF)', (x0 + 5, y0 + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1, cv2.LINE_AA,
                )

            # Colorbar on right
            bar_h = 140
            grad = np.linspace(255, 0, bar_h, dtype=np.uint8)[:, None]
            if pal_cmap is None:
                bar = cv2.cvtColor(np.repeat(grad, 12, axis=1), cv2.COLOR_GRAY2BGR)
            else:
                bar = cv2.applyColorMap(np.repeat(grad, 12, axis=1), pal_cmap)
            bx, by = w - 24, 24
            th_view[by : by + bar_h, bx : bx + 12] = bar
            cv2.rectangle(th_view, (bx, by), (bx + 12, by + bar_h), (220, 220, 220), 1)
            cv2.putText(th_view, '45C', (bx - 30, by + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (240, 240, 240), 1)
            cv2.putText(th_view, '15C', (bx - 30, by + bar_h), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (240, 240, 240), 1)

            panel[th_y0 : th_y0 + th_h, 0:w] = th_view
        else:
            cv2.putText(
                panel, 'ESPERANDO CAMARA TERMICA L16...', (95, th_y0 + th_h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (160, 175, 190), 1, cv2.LINE_AA,
            )

        self._draw_header(panel, f'OBSERVADOR TERMICO IA (C08/C16) | {pal_name}', (255, 140, 40))

        # Observer status & candidate table below thermal image
        fy = th_y0 + th_h + 14
        tracks = list(self.findings.get('tracks', []))
        cnt = len(self.detections)
        banner_c = (40, 190, 60) if cnt > 0 else (180, 190, 200)
        cv2.putText(
            panel,
            f'EN VISTA: {cnt} persona(s)  |  Anomalias >42C: {self.anomalies_count}',
            (14, fy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, banner_c, 1, cv2.LINE_AA,
        )

        # List up to 5 tracked targets with angular confirmation progress
        row_y = fy + 36
        cv2.putText(
            panel, 'ID     ESTADO         TEMP    CONF   ANGULO(dA)   PC', (14, row_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.39, (150, 165, 180), 1, cv2.LINE_AA,
        )
        row_y += 8
        cv2.line(panel, (12, row_y), (w - 12, row_y), (55, 65, 75), 1)
        for tr in tracks[:5]:
            row_y += 22
            st = tr.get('status', 'SUSPECT')
            col = (60, 255, 90) if st == 'CONFIRMED' else ((255, 220, 40) if st == 'READY_CONFIRM' else (0, 220, 255))
            st_lbl = 'CONFIRMADA' if st == 'CONFIRMED' else ('2do ANG OK' if st == 'READY_CONFIRM' else 'SOSPECHA')
            cp_lbl = 'OK' if tr.get('sent_to_cp') else ('PEND' if st == 'CONFIRMED' else '-')
            line = (
                f"{tr['id']:<5} {st_lbl:<12} {tr['max_temp_c']:5.1f}C  "
                f"{int(tr['confidence']*100):3d}%    {tr['angle_span_deg']:5.1f} deg   {cp_lbl}"
            )
            cv2.putText(panel, line, (14, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1, cv2.LINE_AA)

        if not tracks:
            cv2.putText(
                panel, 'Sin candidatos geolocalizados todavia. Despegue (T) y explore.',
                (14, row_y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (140, 150, 160), 1, cv2.LINE_AA,
            )

        return panel

    def _world_to_map(self, wx: float, wy: float, map_w: int, map_h: int) -> Tuple[int, int]:
        # Map x in [-14, 34] (48m) horizontally, y in [-18, 18] (36m) vertically (up = +Y)
        nx = (wx - (-14.0)) / 48.0
        ny = 1.0 - (wy - (-18.0)) / 36.0
        px = int(max(4, min(map_w - 5, nx * map_w)))
        py = int(max(4, min(map_h - 5, ny * map_h)))
        return px, py

    def _render_map_panel(self) -> np.ndarray:
        w, h = self.PANEL_W, self.PANEL_H
        panel = np.full((h, w, 3), (18, 22, 28), dtype=np.uint8)
        self._draw_header(panel, 'MINIMAPA TACTICO USAR (C19/C20) & BITACORA', (90, 235, 130))

        map_y0, map_h = 38, 336
        map_img = np.full((map_h, w, 3), (26, 31, 38), dtype=np.uint8)

        # Grid every 10m
        for gx in range(-10, 35, 10):
            px, _ = self._world_to_map(float(gx), 0.0, w, map_h)
            cv2.line(map_img, (px, 0), (px, map_h), (38, 44, 52), 1)
        for gy in range(-10, 20, 10):
            _, py = self._world_to_map(0.0, float(gy), w, map_h)
            cv2.line(map_img, (0, py), (w, py), (38, 44, 52), 1)

        # Safe perimeter circle around Helipad (C20)
        hx, hy = self._world_to_map(HELIPAD_POS[0], HELIPAD_POS[1], w, map_h)
        rx, _ = self._world_to_map(HELIPAD_POS[0] + SAFE_PERIMETER_RADIUS_M, HELIPAD_POS[1], w, map_h)
        cv2.circle(map_img, (hx, hy), max(10, abs(rx - hx)), (45, 110, 55), 1, cv2.LINE_AA)

        # Collapse sectors (C19)
        for sec in DISASTER_SECTORS:
            x0, y0, x1, y1 = sec['bounds']
            p1 = self._world_to_map(x0, y1, w, map_h)
            p2 = self._world_to_map(x1, y0, w, map_h)
            cv2.rectangle(map_img, p1, p2, (52, 58, 68), -1)
            cv2.rectangle(map_img, p1, p2, sec['color'], 1)
            cv2.putText(
                map_img, sec['id'], (p1[0] + 4, p1[1] + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.34, (210, 215, 225), 1, cv2.LINE_AA,
            )
            cv2.putText(
                map_img, sec['pattern'], (p1[0] + 4, p1[1] + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30, (160, 175, 195), 1, cv2.LINE_AA,
            )

        # Helipad & Command Post
        cv2.rectangle(map_img, (hx - 9, hy - 9), (hx + 9, hy + 9), (40, 200, 240), 2)
        cv2.putText(map_img, 'H', (hx - 4, hy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (40, 220, 255), 1)
        cpx, cpy = self._world_to_map(COMMAND_POST_POS[0], COMMAND_POST_POS[1], w, map_h)
        cv2.rectangle(map_img, (cpx - 8, cpy - 8), (cpx + 8, cpy + 8), (40, 130, 240), -1)
        cv2.putText(map_img, 'PC', (cpx - 10, cpy + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (140, 200, 255), 1)

        # Hazards
        for hz in HAZARDS:
            px, py = self._world_to_map(hz['pos'][0], hz['pos'][1], w, map_h)
            col = (30, 100, 255) if hz['type'] == 'fire' else ((0, 210, 230) if hz['type'] == 'cable' else (150, 160, 170))
            cv2.drawMarker(map_img, (px, py), col, cv2.MARKER_TILTED_CROSS, 9, 2)
            lbl_x = px - 58 if hz['type'] == 'fire' else px + 6
            lbl_y = py + 14 if hz['type'] == 'fire' else py + 4
            cv2.putText(map_img, hz['id'], (lbl_x, lbl_y), cv2.FONT_HERSHEY_SIMPLEX, 0.30, col, 1)

        # Drone trail
        pts = [self._world_to_map(tx, ty, w, map_h) for tx, ty in self.trail]
        for i in range(1, len(pts)):
            cv2.line(map_img, pts[i - 1], pts[i], (180, 140, 40), 1, cv2.LINE_AA)

        # Tracked victims on map
        for tr in self.findings.get('tracks', []):
            vx, vy = self._world_to_map(float(tr['x']), float(tr['y']), w, map_h)
            st = tr.get('status', 'SUSPECT')
            col = (60, 255, 90) if st == 'CONFIRMED' else ((255, 220, 40) if st == 'READY_CONFIRM' else (0, 220, 255))
            cv2.circle(map_img, (vx, vy), 7, col, -1)
            cv2.circle(map_img, (vx, vy), 9, (255, 255, 255), 1)
            cv2.putText(map_img, tr['id'], (vx + 9, vy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1, cv2.LINE_AA)

        # Drone icon + FOV cone
        pos = self.status.get('pos', [0.0, 0.0, 0.0])
        yaw = float(self.status.get('rpy', [0.0, 0.0, 0.0])[2])
        dx, dy = self._world_to_map(float(pos[0]), float(pos[1]), w, map_h)
        fov_len = 6.5
        p_left = self._world_to_map(
            float(pos[0]) + fov_len * math.cos(yaw + 0.55),
            float(pos[1]) + fov_len * math.sin(yaw + 0.55),
            w,
            map_h,
        )
        p_right = self._world_to_map(
            float(pos[0]) + fov_len * math.cos(yaw - 0.55),
            float(pos[1]) + fov_len * math.sin(yaw - 0.55),
            w,
            map_h,
        )
        cv2.line(map_img, (dx, dy), p_left, (120, 230, 255), 1, cv2.LINE_AA)
        cv2.line(map_img, (dx, dy), p_right, (120, 230, 255), 1, cv2.LINE_AA)
        nose = self._world_to_map(
            float(pos[0]) + 1.4 * math.cos(yaw),
            float(pos[1]) + 1.4 * math.sin(yaw),
            w,
            map_h,
        )
        cv2.circle(map_img, (dx, dy), 5, (255, 255, 255), -1)
        cv2.line(map_img, (dx, dy), nose, (40, 255, 90), 2)

        panel[map_y0 : map_y0 + map_h, 0:w] = map_img

        # Bottom mission log & controls cheatsheet
        by = map_y0 + map_h + 18
        counts = self.findings.get('counts', {})
        cv2.putText(
            panel,
            f"CONFIRMADAS: {counts.get('confirmed', 0)}  |  LISTAS: {counts.get('ready', 0)}  "
            f"|  SOSPECHAS: {counts.get('suspect', 0)}  |  SD: {self.findings.get('sd_frames_saved', 0)} frames",
            (12, by), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (140, 255, 160), 1, cv2.LINE_AA,
        )
        by += 22
        last_ev = str(self.findings.get('last_event', ''))[:58]
        cv2.putText(
            panel, f'EVENTO: {last_ev}', (12, by),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 230, 120), 1, cv2.LINE_AA,
        )
        by += 22
        confirmed_list = [t for t in self.findings.get('tracks', []) if t.get('status') == 'CONFIRMED']
        if confirmed_list:
            lt = confirmed_list[-1]
            geo_txt = (
                f"ULTIMO REGISTRO: {lt['id']} ({lt['lat']:.5f}, {lt['lon']:.5f}) "
                f"PC:{'OK' if lt.get('sent_to_cp') else 'COLA'}"
            )
        else:
            geo_txt = f"COLA PC: {counts.get('pending_cp', 0)} | Carpeta: mission_logs/"
        cv2.putText(panel, geo_txt[:60], (12, by), cv2.FONT_HERSHEY_SIMPLEX, 0.39, (200, 215, 230), 1, cv2.LINE_AA)

        by += 22
        cv2.line(panel, (12, by - 6), (w - 12, by - 6), (55, 65, 75), 1)
        cv2.putText(
            panel, 'CONTROLES: [T] Despegar  [WASD/RF/QE] Volar  [Espacio] Hover',
            (12, by + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (175, 188, 200), 1, cv2.LINE_AA,
        )
        cv2.putText(
            panel, '  [U/J] Gimbal  [O] Orbita 2do-Ang  [C] Confirmar  [P] Paleta',
            (12, by + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (175, 188, 200), 1, cv2.LINE_AA,
        )
        cv2.putText(
            panel, '  [H] RTH Auto  [L] Falla Enlace  [B] Relevo Bat  [G] Rafaga',
            (12, by + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (175, 188, 200), 1, cv2.LINE_AA,
        )
        return panel

    def _render_and_publish(self) -> None:
        pilot = self._render_pilot_panel()
        observer = self._render_observer_panel()
        tactical_map = self._render_map_panel()
        unified = np.hstack([pilot, observer, tactical_map])

        out_msg = self.bridge.cv2_to_imgmsg(unified, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.gcs_pub.publish(out_msg)

        if self.show_window:
            try:
                if self.dual_windows:
                    cv2.imshow(self.WIN_PILOT, pilot)
                    cv2.imshow(self.WIN_OBSERVER, np.hstack([observer, tactical_map]))
                else:
                    cv2.imshow(self.WIN_UNIFIED, unified)
                cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().warning(
                    f'Error mostrando ventana GCS ({exc}); desactivando show_window.'
                )
                self.show_window = False

    def destroy_node(self) -> bool:
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundControlHUD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
