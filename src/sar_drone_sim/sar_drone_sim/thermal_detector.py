"""ROS 2 computer vision node for thermal human survivor detection."""

import json
import os
from typing import Dict, List, Optional, Tuple

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class ThermalHumanDetector(Node):
    """Detects human thermal signatures from L16 Gazebo thermal frames."""

    WINDOW_NAME = 'SAR Drone - Camara Termica AI | Camara RGB'

    def __init__(self) -> None:
        super().__init__('thermal_detector')

        self.declare_parameter('linear_resolution', 0.01)
        self.declare_parameter('min_human_temp_c', 32.5)
        self.declare_parameter('max_human_temp_c', 41.5)
        self.declare_parameter('min_area_px', 60)
        self.declare_parameter('max_area_px', 45000)
        self.declare_parameter('show_window', True)

        self.linear_resolution = float(self.get_parameter('linear_resolution').value)
        self.min_human_temp_c = float(self.get_parameter('min_human_temp_c').value)
        self.max_human_temp_c = float(self.get_parameter('max_human_temp_c').value)
        self.min_area_px = int(self.get_parameter('min_area_px').value)
        self.max_area_px = int(self.get_parameter('max_area_px').value)
        self.show_window = bool(self.get_parameter('show_window').value) and bool(
            os.environ.get('DISPLAY')
        )

        self.bridge = CvBridge()
        self.latest_rgb: Optional[np.ndarray] = None
        self._kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        self.annotated_pub = self.create_publisher(Image, '/sar_drone/thermal/annotated', 10)
        self.detections_pub = self.create_publisher(String, '/sar_drone/detections', 10)
        self.alert_pub = self.create_publisher(Bool, '/sar_drone/victim_detected', 10)

        self.thermal_sub = self.create_subscription(
            Image, '/sar_drone/thermal/image_raw', self._thermal_callback, 10
        )
        self.rgb_sub = self.create_subscription(
            Image, '/sar_drone/rgb/image_raw', self._rgb_callback, 10
        )

        self.get_logger().info(
            f'ThermalHumanDetector activo [rango humano: '
            f'{self.min_human_temp_c:.1f}-{self.max_human_temp_c:.1f} C, '
            f'show_window={self.show_window}]'
        )

    def _rgb_callback(self, msg: Image) -> None:
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'Error convirtiendo imagen RGB: {exc}')

    def _image_to_celsius(self, raw_img: np.ndarray) -> np.ndarray:
        if raw_img.dtype == np.uint16:
            return raw_img.astype(np.float32) * self.linear_resolution - 273.15
        if raw_img.dtype == np.uint8:
            # Fallback if a mono8 thermal stream is supplied (0-100 C span)
            return raw_img.astype(np.float32) * (100.0 / 255.0)
        return raw_img.astype(np.float32)

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

            # Confidence combines proximity to 36.5 C core temp and blob compactness
            temp_score = max(0.0, 1.0 - abs(max_t - 36.5) / 5.0)
            size_score = min(1.0, area / 400.0)
            confidence = round(min(0.99, 0.55 * temp_score + 0.45 * size_score), 2)

            det_id += 1
            detections.append(
                {
                    'id': det_id,
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'area_px': round(area, 1),
                    'max_temp_c': round(max_t, 2),
                    'mean_temp_c': round(mean_t, 2),
                    'confidence': confidence,
                }
            )

        # Non-human high-temperature anomalies (e.g. fires/hot electrical equipment)
        hot_mask = np.where(temp_c > self.max_human_temp_c, 255, 0).astype(np.uint8)
        hot_mask = cv2.morphologyEx(hot_mask, cv2.MORPH_OPEN, self._kernel_open)
        hot_contours, _ = cv2.findContours(
            hot_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        anomalies: List[Tuple[int, int, int, int]] = []
        for cnt in hot_contours:
            if cv2.contourArea(cnt) >= 40:
                anomalies.append(cv2.boundingRect(cnt))

        return detections, anomalies

    def _render_thermal_hud(
        self,
        temp_c: np.ndarray,
        detections: List[Dict],
        anomalies: List[Tuple[int, int, int, int]],
    ) -> np.ndarray:
        # Fixed 15 C .. 45 C false-color scale so cool rubble vs 37 C body is consistent
        norm = np.clip((temp_c - 15.0) / (45.0 - 15.0), 0.0, 1.0)
        gray = (norm * 255.0).astype(np.uint8)
        heatmap = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)

        # Upscale 320x240 -> 640x480 for crisp operator HUD text
        scale = 2
        hud = cv2.resize(
            heatmap, (temp_c.shape[1] * scale, temp_c.shape[0] * scale),
            interpolation=cv2.INTER_NEAREST,
        )
        h_hud, w_hud = hud.shape[:2]

        # Draw high-heat non-human anomalies in cyan/yellow
        for ax, ay, aw, ah in anomalies:
            p1 = (ax * scale, ay * scale)
            p2 = ((ax + aw) * scale, (ay + ah) * scale)
            cv2.rectangle(hud, p1, p2, (0, 220, 255), 2)
            cv2.putText(
                hud,
                'ANOMALIA >42C',
                (p1[0], max(18, p1[1] - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 220, 255),
                1,
                cv2.LINE_AA,
            )

        # Draw human detections in bright green
        for det in detections:
            x, y, w, h = det['bbox']
            p1 = (x * scale, y * scale)
            p2 = ((x + w) * scale, (y + h) * scale)
            cv2.rectangle(hud, p1, p2, (40, 255, 80), 2)
            label = f"PERSONA #{det['id']} | {det['max_temp_c']:.1f}C ({int(det['confidence']*100)}%)"
            text_y = max(22, p1[1] - 8)
            cv2.rectangle(
                hud,
                (p1[0], text_y - 16),
                (min(w_hud - 4, p1[0] + len(label) * 7 + 8), text_y + 4),
                (15, 45, 15),
                -1,
            )
            cv2.putText(
                hud,
                label,
                (p1[0] + 4, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (120, 255, 140),
                1,
                cv2.LINE_AA,
            )

        # Center crosshair + spot temperature readout
        cx, cy = w_hud // 2, h_hud // 2
        center_temp = float(temp_c[temp_c.shape[0] // 2, temp_c.shape[1] // 2])
        cv2.drawMarker(hud, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)
        cv2.putText(
            hud,
            f'{center_temp:.1f} C',
            (cx + 10, cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # Status banner
        count = len(detections)
        banner_color = (20, 120, 20) if count > 0 else (40, 40, 45)
        banner_text = (
            f'ALERTA: {count} PERSONA(S) DETECTADA(S)'
            if count > 0
            else 'ESCANEO TERMICO ACTIVO - SIN PERSONAS EN VISTA'
        )
        cv2.rectangle(hud, (0, 0), (w_hud, 30), banner_color, -1)
        cv2.putText(
            hud,
            banner_text,
            (10, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # Vertical temperature colorbar on the right
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
        hud = self._render_thermal_hud(temp_c, detections, anomalies)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(hud, encoding='bgr8')
        out_msg.header = msg.header
        self.annotated_pub.publish(out_msg)

        # Publish structured JSON detections & boolean alert
        payload = {
            'count': len(detections),
            'detections': detections,
            'anomalies_count': len(anomalies),
        }
        det_msg = String()
        det_msg.data = json.dumps(payload)
        self.detections_pub.publish(det_msg)

        alert_msg = Bool()
        alert_msg.data = len(detections) > 0
        self.alert_pub.publish(alert_msg)

        if self.show_window:
            try:
                if self.latest_rgb is not None:
                    rgb_view = cv2.resize(self.latest_rgb, (hud.shape[1], hud.shape[0]))
                    cv2.rectangle(rgb_view, (0, 0), (rgb_view.shape[1], 30), (35, 35, 40), -1)
                    cv2.putText(
                        rgb_view,
                        'CAMARA RGB (REFERENCIA VISUAL)',
                        (10, 21),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (240, 240, 240),
                        2,
                        cv2.LINE_AA,
                    )
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
