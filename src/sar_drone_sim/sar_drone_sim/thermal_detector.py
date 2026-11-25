"""ROS 2 thermal computer vision, 3D geolocation, double-confirmation & offline logger."""

from datetime import datetime
import json
import math
import os
import time
from typing import Dict, List, Optional, Tuple

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from sar_drone_sim.usar_common import (
    angular_span_deg,
    enu_to_wgs84,
    quat_to_rot_matrix,
)


class ThermalHumanDetector(Node):
    """Detects human thermal signatures, tracks 3D candidates, and logs USAR findings."""

    WINDOW_NAME = 'SAR Drone - Camara Termica AI | Camara RGB'

    def __init__(self) -> None:
        super().__init__('thermal_detector')

        self.declare_parameter('linear_resolution', 0.01)
        self.declare_parameter('min_human_temp_c', 32.5)
        self.declare_parameter('max_human_temp_c', 41.5)
        self.declare_parameter('min_area_px', 60)
        self.declare_parameter('max_area_px', 45000)
        self.declare_parameter('show_window', False)
        self.declare_parameter(
            'mission_log_dir', os.path.expanduser('~/usar_drone_ws/mission_logs')
        )
        self.declare_parameter('confirm_angle_deg', 35.0)
        self.declare_parameter('assoc_radius_m', 2.2)
        self.declare_parameter('ground_plane_z', 0.25)

        self.linear_resolution = float(self.get_parameter('linear_resolution').value)
        self.min_human_temp_c = float(self.get_parameter('min_human_temp_c').value)
        self.max_human_temp_c = float(self.get_parameter('max_human_temp_c').value)
        self.min_area_px = int(self.get_parameter('min_area_px').value)
        self.max_area_px = int(self.get_parameter('max_area_px').value)
        self.show_window = bool(self.get_parameter('show_window').value) and bool(
            os.environ.get('DISPLAY')
        )
        self.mission_log_dir = str(self.get_parameter('mission_log_dir').value)
        self.confirm_angle_deg = float(self.get_parameter('confirm_angle_deg').value)
        self.assoc_radius_m = float(self.get_parameter('assoc_radius_m').value)
        self.ground_plane_z = float(self.get_parameter('ground_plane_z').value)

        # Offline mission log structure (C10, C23)
        session_name = f"mision_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.session_dir = os.path.join(self.mission_log_dir, session_name)
        self.sd_dir = os.path.join(self.session_dir, 'sd_onboard')
        self.findings_dir = os.path.join(self.session_dir, 'findings')
        os.makedirs(self.sd_dir, exist_ok=True)
        os.makedirs(self.findings_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_hud: Optional[np.ndarray] = None
        self._kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Camera intrinsics (defaults for 320x240, hfov=1.15 rad)
        self.cam_fx = 320.0 / (2.0 * math.tan(1.15 / 2.0))
        self.cam_fy = self.cam_fx
        self.cam_cx = 160.0
        self.cam_cy = 120.0

        # Drone pose and flight status
        self.drone_pos = np.array([0.0, 0.0, 0.14], dtype=np.float64)
        self.drone_rot = np.eye(3, dtype=np.float64)
        self.gimbal_pitch_rad = 0.48
        self.flight_mode = 'STANDBY'
        self.link_state = 'PRIMARY'
        self.orbit_target_id: Optional[str] = None

        # Persistent victim candidate tracks
        self.tracks: Dict[str, Dict] = {}
        self.next_track_num = 1
        self.anomalies_world: List[Dict] = []
        self.last_event = 'Escaneo termico iniciado.'

        # SD & report state
        self.sd_frames_saved = 0
        self._sd_ring: List[str] = []
        self._last_sd_save = 0.0

        # Publishers
        self.annotated_pub = self.create_publisher(Image, '/sar_drone/thermal/annotated', 10)
        self.detections_pub = self.create_publisher(String, '/sar_drone/detections', 10)
        self.alert_pub = self.create_publisher(Bool, '/sar_drone/victim_detected', 10)
        self.findings_pub = self.create_publisher(String, '/sar_drone/findings', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/sar_drone/victim_markers', 10)

        # Subscriptions
        self.thermal_sub = self.create_subscription(
            Image, '/sar_drone/thermal/image_raw', self._thermal_callback, 10
        )
        self.rgb_sub = self.create_subscription(
            Image, '/sar_drone/rgb/image_raw', self._rgb_callback, 10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/sar_drone/thermal/camera_info', self._cam_info_cb, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/sar_drone/odom', self._odom_cb, 10
        )
        self.status_sub = self.create_subscription(
            String, '/sar_drone/status', self._status_cb, 10
        )
        self.mission_cmd_sub = self.create_subscription(
            String, '/sar_drone/mission_cmd', self._mission_cmd_cb, 10
        )

        self.findings_timer = self.create_timer(0.5, self._publish_findings_and_markers)

        self.get_logger().info(
            f'ThermalHumanDetector activo [rango={self.min_human_temp_c:.1f}-'
            f'{self.max_human_temp_c:.1f}C, logs={self.session_dir}]'
        )

    def _cam_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9 and msg.k[0] > 1.0 and msg.k[4] > 1.0:
            self.cam_fx = float(msg.k[0])
            self.cam_cx = float(msg.k[2])
            self.cam_fy = float(msg.k[4])
            self.cam_cy = float(msg.k[5])

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.drone_pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.drone_rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)

    def _status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.gimbal_pitch_rad = float(data.get('gimbal_pitch_rad', self.gimbal_pitch_rad))
            self.flight_mode = str(data.get('mode', self.flight_mode))
            self.link_state = str(data.get('link_state', self.link_state))
            self.orbit_target_id = data.get('orbit_target_id')
            self._sync_pending_findings()
        except Exception:
            pass

    def _mission_cmd_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get('cmd') == 'confirm_finding':
            self._handle_confirm_command()

    def _rgb_callback(self, msg: Image) -> None:
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Error convirtiendo imagen RGB: {exc}')

    def _image_to_celsius(self, raw_img: np.ndarray) -> np.ndarray:
        if raw_img.dtype == np.uint16:
            return raw_img.astype(np.float32) * self.linear_resolution - 273.15
        if raw_img.dtype == np.uint8:
            return raw_img.astype(np.float32) * (100.0 / 255.0)
        return raw_img.astype(np.float32)

    def _project_pixel_to_ground(self, u: float, v: float) -> Optional[Tuple[float, float, float]]:
        """Project thermal pixel (u, v) onto ground_plane_z using drone odom + gimbal pitch."""
        if self.drone_pos[2] < 0.65:
            return None
        ray_cam = np.array(
            [1.0, -(u - self.cam_cx) / self.cam_fx, -(v - self.cam_cy) / self.cam_fy],
            dtype=np.float64,
        )
        ray_cam /= np.linalg.norm(ray_cam)

        cp = math.cos(self.gimbal_pitch_rad)
        sp = math.sin(self.gimbal_pitch_rad)
        r_gimbal = np.array(
            [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
            dtype=np.float64,
        )
        p_cam_body = np.array([0.14, 0.0, -0.025], dtype=np.float64) + r_gimbal @ np.array(
            [0.025, 0.0, -0.005], dtype=np.float64
        )
        cam_origin = self.drone_pos + self.drone_rot @ p_cam_body
        ray_world = self.drone_rot @ (r_gimbal @ ray_cam)

        if ray_world[2] >= -0.05:
            return None
        t = (self.ground_plane_z - cam_origin[2]) / ray_world[2]
        if t < 0.6 or t > 45.0:
            return None
        hit = cam_origin + t * ray_world
        return float(hit[0]), float(hit[1]), float(hit[2])

    def _find_human_detections(
        self, temp_c: np.ndarray
    ) -> Tuple[List[Dict], List[Tuple[int, int, int, int]]]:
        human_mask = np.where(
            (temp_c >= self.min_human_temp_c) & (temp_c <= self.max_human_temp_c),
            255,
            0,
        ).astype(np.uint8)
        human_mask = cv2.morphologyEx(human_mask, cv2.MORPH_OPEN, self._kernel_open)
        human_mask = cv2.morphologyEx(human_mask, cv2.MORPH_CLOSE, self._kernel_close)

        contours, _ = cv2.findContours(
            human_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections: List[Dict] = []
        det_id = 0
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < self.min_area_px or area > self.max_area_px:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / max(h, 1)
            if aspect < 0.15 or aspect > 6.5:
                continue

            roi_mask = np.zeros((h, w), dtype=np.uint8)
            shifted = cnt - np.array([[[x, y]]])
            cv2.drawContours(roi_mask, [shifted], -1, 255, thickness=cv2.FILLED)
            roi_temps = temp_c[y : y + h, x : x + w][roi_mask > 0]
            if roi_temps.size == 0:
                continue

            max_t = float(np.max(roi_temps))
            mean_t = float(np.mean(roi_temps))

            temp_score = max(0.0, 1.0 - abs(max_t - 36.5) / 5.0)
            size_score = min(1.0, area / 400.0)
            confidence = round(min(0.99, 0.55 * temp_score + 0.45 * size_score), 2)

            u_c = x + w * 0.5
            v_c = y + h * 0.5
            world_hit = self._project_pixel_to_ground(u_c, v_c)

            det_id += 1
            det_dict: Dict = {
                'id': det_id,
                'bbox': [int(x), int(y), int(w), int(h)],
                'area_px': round(area, 1),
                'max_temp_c': round(max_t, 2),
                'mean_temp_c': round(mean_t, 2),
                'confidence': confidence,
            }
            if world_hit is not None:
                det_dict['world_xyz'] = [round(world_hit[0], 2), round(world_hit[1], 2), round(world_hit[2], 2)]
            detections.append(det_dict)

        # Non-human high-temperature anomalies (e.g. fires/hot electrical equipment)
        hot_mask = np.where(temp_c > self.max_human_temp_c, 255, 0).astype(np.uint8)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, self._kernel_open)
        hot_contours, _ = cv2.findContours(
            hot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        anomalies: List[Tuple[int, int, int, int]] = []
        anomalies_w: List[Dict] = []
        for cnt in hot_contours:
            if cv2.contourArea(cnt) >= 40:
                ax, ay, aw, ah = cv2.boundingRect(cnt)
                anomalies.append((ax, ay, aw, ah))
                hit = self._project_pixel_to_ground(ax + aw * 0.5, ay + ah * 0.5)
                if hit is not None:
                    anomalies_w.append({'x': round(hit[0], 2), 'y': round(hit[1], 2)})
        if anomalies_w:
            self.anomalies_world = anomalies_w[:6]

        return detections, anomalies

    def _update_tracks(self, detections: List[Dict]) -> None:
        now_s = time.monotonic()
        for det in detections:
            wxyz = det.get('world_xyz')
            if not wxyz:
                continue
            wx, wy, wz = float(wxyz[0]), float(wxyz[1]), float(wxyz[2])

            best_id: Optional[str] = None
            best_dist = self.assoc_radius_m
            for tid, tr in self.tracks.items():
                d = math.hypot(wx - tr['x'], wy - tr['y'])
                if d <= best_dist:
                    best_dist = d
                    best_id = tid

            if best_id is None:
                tid = f'V-{self.next_track_num:02d}'
                self.next_track_num += 1
                lat, lon, alt = enu_to_wgs84(wx, wy, wz)
                az = math.degrees(math.atan2(self.drone_pos[1] - wy, self.drone_pos[0] - wx)) % 360.0
                self.tracks[tid] = {
                    'id': tid,
                    'x': wx,
                    'y': wy,
                    'z': wz,
                    'lat': round(lat, 6),
                    'lon': round(lon, 6),
                    'alt_wgs84': round(alt, 1),
                    'max_temp_c': det['max_temp_c'],
                    'confidence': det['confidence'],
                    'observations': 1,
                    'angles_deg': [round(az, 1)],
                    'angle_span_deg': 0.0,
                    'status': 'SUSPECT',
                    'confirmed_at': None,
                    'sent_to_cp': False,
                    'finding_dir': None,
                    'last_seen_s': now_s,
                }
                self.last_event = f'{tid} detectada en ({wx:.1f}, {wy:.1f}) [SOSPECHA]'
                best_id = tid
            else:
                tr = self.tracks[best_id]
                alpha = 0.08 if tr['status'] == 'CONFIRMED' else 0.25
                tr['x'] = (1.0 - alpha) * tr['x'] + alpha * wx
                tr['y'] = (1.0 - alpha) * tr['y'] + alpha * wy
                lat, lon, alt = enu_to_wgs84(tr['x'], tr['y'], tr['z'])
                tr['lat'] = round(lat, 6)
                tr['lon'] = round(lon, 6)
                tr['max_temp_c'] = max(tr['max_temp_c'], det['max_temp_c'])
                tr['confidence'] = max(tr['confidence'], det['confidence'])
                tr['observations'] += 1
                tr['last_seen_s'] = now_s

                az = math.degrees(
                    math.atan2(self.drone_pos[1] - tr['y'], self.drone_pos[0] - tr['x'])
                ) % 360.0
                last_az = tr['angles_deg'][-1]
                diff = abs((az - last_az + 180.0) % 360.0 - 180.0)
                if diff >= 3.0:
                    tr['angles_deg'].append(round(az, 1))
                span = angular_span_deg(tr['angles_deg'])
                tr['angle_span_deg'] = round(span, 1)

                if tr['status'] == 'SUSPECT' and span >= self.confirm_angle_deg and tr['observations'] >= 3:
                    tr['status'] = 'READY_CONFIRM'
                    self.last_event = f'{best_id} listo para confirmar (dA={span:.0f} deg)'
                elif (
                    tr['status'] != 'CONFIRMED'
                    and self.flight_mode == 'ORBIT_CONFIRM'
                    and (self.orbit_target_id in (None, best_id))
                    and span >= 70.0
                    and tr['observations'] >= 6
                ):
                    self._confirm_track(best_id, auto_orbit=True)

            det['track_id'] = best_id
            det['track_status'] = self.tracks[best_id]['status']
            det['angle_span_deg'] = self.tracks[best_id]['angle_span_deg']

    def _confirm_track(self, tid: str, auto_orbit: bool = False) -> None:
        tr = self.tracks[tid]
        if tr['status'] == 'CONFIRMED':
            return
        prev_status = tr['status']
        tr['status'] = 'CONFIRMED'
        tr['confirmed_at'] = datetime.now().isoformat(timespec='seconds')
        tr['sent_to_cp'] = self.link_state != 'LOST'

        # Persist finding artifacts locally (C09, C23)
        fdir = os.path.join(self.findings_dir, tid)
        os.makedirs(fdir, exist_ok=True)
        tr['finding_dir'] = fdir

        if self.latest_hud is not None:
            cv2.imwrite(os.path.join(fdir, 'thermal_annotated.png'), self.latest_hud)
        if self.latest_rgb is not None:
            cv2.imwrite(os.path.join(fdir, 'rgb_snapshot.png'), self.latest_rgb)

        self._write_finding_json(tr)
        self._write_mission_report()

        mode_txt = 'auto-orbita' if auto_orbit else (
            '2do angulo' if prev_status == 'READY_CONFIRM' else 'manual-1er-angulo'
        )
        cp_txt = 'ENVIADO a PC' if tr['sent_to_cp'] else 'PENDIENTE (sin enlace)'
        self.last_event = f'{tid} CONFIRMADA ({mode_txt}) - {cp_txt}'
        self.get_logger().info(f'Hallazgo {tid} confirmado [{mode_txt}] guardado en {fdir}')

    def _handle_confirm_command(self) -> None:
        unconfirmed = [t for t in self.tracks.values() if t['status'] != 'CONFIRMED']
        if not unconfirmed:
            self.last_event = 'Sin victimas pendientes para confirmar.'
            return
        ready = [t for t in unconfirmed if t['status'] == 'READY_CONFIRM']
        pool = ready if ready else unconfirmed
        target = min(
            pool,
            key=lambda t: math.hypot(t['x'] - self.drone_pos[0], t['y'] - self.drone_pos[1]),
        )
        self._confirm_track(target['id'], auto_orbit=False)

    def _write_finding_json(self, tr: Dict) -> None:
        fdir = tr.get('finding_dir')
        if not fdir:
            return
        record = {
            'id': tr['id'],
            'status': tr['status'],
            'confirmed_at': tr['confirmed_at'],
            'local_xyz_m': [round(tr['x'], 2), round(tr['y'], 2), round(tr['z'], 2)],
            'wgs84': {'lat': tr['lat'], 'lon': tr['lon'], 'alt_m': tr['alt_wgs84']},
            'drone_altitude_m': round(float(self.drone_pos[2]), 2),
            'max_temp_c': tr['max_temp_c'],
            'confidence': tr['confidence'],
            'angle_span_deg': tr['angle_span_deg'],
            'observations': tr['observations'],
            'sent_to_cp': tr['sent_to_cp'],
        }
        with open(os.path.join(fdir, 'finding.json'), 'w', encoding='utf-8') as f:
            json.dump(record, f, indent=2)

    def _write_mission_report(self) -> None:
        confirmed = [t for t in self.tracks.values() if t['status'] == 'CONFIRMED']
        report = {
            'session_dir': self.session_dir,
            'updated_at': datetime.now().isoformat(timespec='seconds'),
            'confirmed_count': len(confirmed),
            'sd_frames_saved': self.sd_frames_saved,
            'findings': [
                {
                    'id': t['id'],
                    'local_xy': [round(t['x'], 2), round(t['y'], 2)],
                    'lat': t['lat'],
                    'lon': t['lon'],
                    'max_temp_c': t['max_temp_c'],
                    'angle_span_deg': t['angle_span_deg'],
                    'confirmed_at': t['confirmed_at'],
                    'sent_to_cp': t['sent_to_cp'],
                }
                for t in confirmed
            ],
        }
        with open(os.path.join(self.session_dir, 'mission_report.json'), 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2)

    def _sync_pending_findings(self) -> None:
        if self.link_state == 'LOST':
            return
        changed = False
        for tr in self.tracks.values():
            if tr['status'] == 'CONFIRMED' and not tr['sent_to_cp']:
                tr['sent_to_cp'] = True
                self._write_finding_json(tr)
                changed = True
                self.last_event = f"{tr['id']} sincronizado con Puesto de Comando"
        if changed:
            self._write_mission_report()

    def _maybe_save_sd(self, hud: np.ndarray, detections: List[Dict]) -> None:
        now = time.monotonic()
        interval = 1.5 if self.link_state == 'LOST' else 3.0
        if now - self._last_sd_save < interval:
            return
        self._last_sd_save = now
        self.sd_frames_saved += 1
        stem = f'sd_{self.sd_frames_saved:04d}'
        img_path = os.path.join(self.sd_dir, f'{stem}.png')
        meta_path = os.path.join(self.sd_dir, f'{stem}.json')
        cv2.imwrite(img_path, hud)
        meta = {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'drone_xyz': [round(float(v), 2) for v in self.drone_pos],
            'link_state': self.link_state,
            'detections_in_frame': len(detections),
        }
        with open(meta_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f)
        self._sd_ring.append(stem)
        if len(self._sd_ring) > 60:
            old = self._sd_ring.pop(0)
            for ext in ('.png', '.json'):
                p = os.path.join(self.sd_dir, old + ext)
                if os.path.exists(p):
                    os.remove(p)

    def _render_thermal_hud(
        self,
        temp_c: np.ndarray,
        detections: List[Dict],
        anomalies: List[Tuple[int, int, int, int]],
    ) -> np.ndarray:
        norm = np.clip((temp_c - 15.0) / (45.0 - 15.0), 0.0, 1.0)
        gray = (norm * 255.0).astype(np.uint8)
        heatmap = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)

        scale = 2
        hud = cv2.resize(
            heatmap,
            (temp_c.shape[1] * scale, temp_c.shape[0] * scale),
            interpolation=cv2.INTER_NEAREST,
        )
        h_hud, w_hud = hud.shape[:2]

        for ax, ay, aw, ah in anomalies:
            p1 = (ax * scale, ay * scale)
            p2 = ((ax + aw) * scale, (ay + ah) * scale)
            cv2.rectangle(hud, p1, p2, (0, 220, 255), 2)
            cv2.putText(
                hud, 'ANOMALIA >42C', (p1[0], max(18, p1[1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1, cv2.LINE_AA,
            )

        status_colors = {
            'SUSPECT': (0, 220, 255),
            'READY_CONFIRM': (255, 210, 40),
            'CONFIRMED': (60, 255, 90),
        }
        for det in detections:
            x, y, w, h = det['bbox']
            p1 = (x * scale, y * scale)
            p2 = ((x + w) * scale, (y + h) * scale)
            st = det.get('track_status', 'SUSPECT')
            col = status_colors.get(st, (40, 255, 80))
            tid = det.get('track_id', f"#{det['id']}")
            d_ang = det.get('angle_span_deg', 0.0)
            cv2.rectangle(hud, p1, p2, col, 2)
            label = f"{tid} | {det['max_temp_c']:.1f}C ({int(det['confidence']*100)}%) dA:{d_ang:.0f}"
            text_y = max(22, p1[1] - 8)
            cv2.rectangle(
                hud,
                (p1[0], text_y - 16),
                (min(w_hud - 4, p1[0] + len(label) * 7 + 8), text_y + 4),
                (15, 25, 20),
                -1,
            )
            cv2.putText(
                hud, label, (p1[0] + 4, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA,
            )

        cx, cy = w_hud // 2, h_hud // 2
        center_temp = float(temp_c[temp_c.shape[0] // 2, temp_c.shape[1] // 2])
        cv2.drawMarker(hud, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)
        cv2.putText(
            hud, f'{center_temp:.1f} C', (cx + 10, cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )

        count = len(detections)
        banner_color = (20, 120, 20) if count > 0 else (40, 40, 45)
        banner_text = (
            f'ALERTA: {count} PERSONA(S) DETECTADA(S)'
            if count > 0
            else 'ESCANEO TERMICO ACTIVO - SIN PERSONAS EN VISTA'
        )
        cv2.rectangle(hud, (0, 0), (w_hud, 30), banner_color, -1)
        cv2.putText(
            hud, banner_text, (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA,
        )

        bar_h = 180
        bar_y0 = 50
        bar_x0 = w_hud - 28
        grad = np.linspace(255, 0, bar_h, dtype=np.uint8)[:, None]
        bar = cv2.applyColorMap(np.repeat(grad, 14, axis=1), cv2.COLORMAP_INFERNO)
        hud[bar_y0 : bar_y0 + bar_h, bar_x0 : bar_x0 + 14] = bar
        cv2.rectangle(hud, (bar_x0, bar_y0), (bar_x0 + 14, bar_y0 + bar_h), (220, 220, 220), 1)
        cv2.putText(hud, '45C', (bar_x0 - 34, bar_y0 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (240, 240, 240), 1)
        cv2.putText(hud, '15C', (bar_x0 - 34, bar_y0 + bar_h), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (240, 240, 240), 1)
        return hud

    def _thermal_callback(self, msg: Image) -> None:
        try:
            raw_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warning(f'Error convirtiendo imagen termica: {exc}')
            return

        temp_c = self._image_to_celsius(raw_img)
        detections, anomalies = self._find_human_detections(temp_c)
        self._update_tracks(detections)
        hud = self._render_thermal_hud(temp_c, detections, anomalies)
        self.latest_hud = hud

        self._maybe_save_sd(hud, detections)

        out_msg = self.bridge.cv2_to_imgmsg(hud, encoding='bgr8')
        out_msg.header = msg.header
        self.annotated_pub.publish(out_msg)

        payload = {
            'count': len(detections),
            'detections': detections,
            'anomalies_count': len(anomalies),
        }
        self.detections_pub.publish(String(data=json.dumps(payload)))

        alert_msg = Bool()
        alert_msg.data = len(detections) > 0
        self.alert_pub.publish(alert_msg)

        if self.show_window:
            try:
                if self.latest_rgb is not None:
                    rgb_view = cv2.resize(self.latest_rgb, (hud.shape[1], hud.shape[0]))
                    combined = np.hstack([hud, rgb_view])
                else:
                    combined = hud
                cv2.imshow(self.WINDOW_NAME, combined)
                cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().warning(
                    f'No se pudo abrir ventana OpenCV ({exc}); desactivando show_window.'
                )
                self.show_window = False

    def _publish_findings_and_markers(self) -> None:
        tracks_list = sorted(self.tracks.values(), key=lambda t: t['id'])
        counts = {
            'total': len(tracks_list),
            'suspect': sum(1 for t in tracks_list if t['status'] == 'SUSPECT'),
            'ready': sum(1 for t in tracks_list if t['status'] == 'READY_CONFIRM'),
            'confirmed': sum(1 for t in tracks_list if t['status'] == 'CONFIRMED'),
            'pending_cp': sum(
                1 for t in tracks_list if t['status'] == 'CONFIRMED' and not t['sent_to_cp']
            ),
        }
        payload = {
            'session_dir': self.session_dir,
            'tracks': [
                {
                    'id': t['id'],
                    'x': round(t['x'], 2),
                    'y': round(t['y'], 2),
                    'z': round(t['z'], 2),
                    'lat': t['lat'],
                    'lon': t['lon'],
                    'max_temp_c': t['max_temp_c'],
                    'confidence': t['confidence'],
                    'observations': t['observations'],
                    'angle_span_deg': t['angle_span_deg'],
                    'status': t['status'],
                    'confirmed_at': t['confirmed_at'],
                    'sent_to_cp': t['sent_to_cp'],
                }
                for t in tracks_list
            ],
            'anomalies_world': self.anomalies_world,
            'sd_frames_saved': self.sd_frames_saved,
            'last_event': self.last_event,
            'counts': counts,
        }
        self.findings_pub.publish(String(data=json.dumps(payload)))

        # Publish RViz MarkerArray
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, t in enumerate(tracks_list):
            if t['status'] == 'CONFIRMED':
                rgba = (0.15, 0.95, 0.25, 0.95)
            elif t['status'] == 'READY_CONFIRM':
                rgba = (0.15, 0.85, 1.0, 0.95)
            else:
                rgba = (1.0, 0.85, 0.1, 0.9)

            sphere = Marker()
            sphere.header.frame_id = 'odom'
            sphere.header.stamp = stamp
            sphere.ns = 'usar_victims'
            sphere.id = idx * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(t['x'])
            sphere.pose.position.y = float(t['y'])
            sphere.pose.position.z = float(t['z']) + 0.4
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.65
            sphere.scale.y = 0.65
            sphere.scale.z = 0.65
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = rgba
            ma.markers.append(sphere)

            txt = Marker()
            txt.header.frame_id = 'odom'
            txt.header.stamp = stamp
            txt.ns = 'usar_victims_label'
            txt.id = idx * 2 + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(t['x'])
            txt.pose.position.y = float(t['y'])
            txt.pose.position.z = float(t['z']) + 1.2
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.45
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = rgba
            txt.text = f"{t['id']} [{t['status']}] {t['max_temp_c']:.1f}C"
            ma.markers.append(txt)

        self.markers_pub.publish(ma)

    def destroy_node(self) -> bool:
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThermalHumanDetector()
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
