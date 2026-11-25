"""Shared geometry, geodetic, and tactical map utilities for USAR drone nodes."""

import math
from typing import Dict, List, Tuple

import numpy as np

# Geodetic origin matching earthquake_disaster.sdf (Ciudad Universitaria, UNAM)
ORIGIN_LAT = 19.3320
ORIGIN_LON = -99.1870
ORIGIN_ALT = 2240.0

HELIPAD_POS = (0.0, 0.0)
COMMAND_POST_POS = (-4.8, -4.5)
SAFE_PERIMETER_RADIUS_M = 8.5

# Collapse sectors matching earthquake_disaster.sdf (C19 collapse pattern metadata)
DISASTER_SECTORS: List[Dict] = [
    {
        'id': 'EDIFICIO A',
        'pattern': 'Losa Inclinada',
        'bounds': (10.0, 5.0, 20.0, 15.0),
        'color': (110, 120, 145),
    },
    {
        'id': 'EDIFICIO B',
        'pattern': 'Vacio Superv.',
        'bounds': (8.0, -15.0, 18.0, -5.0),
        'color': (105, 110, 140),
    },
    {
        'id': 'EDIFICIO C',
        'pattern': 'Pancake',
        'bounds': (21.0, -6.0, 31.5, 5.5),
        'color': (115, 115, 130),
    },
    {
        'id': 'RUINAS SUR',
        'pattern': 'Muro Colapsado',
        'bounds': (-10.0, 5.8, -3.5, 10.5),
        'color': (100, 115, 125),
    },
    {
        'id': 'RUINAS SW',
        'pattern': 'Esquina Derruida',
        'bounds': (-10.5, -10.5, -4.5, -7.8),
        'color': (100, 105, 120),
    },
]

HAZARDS: List[Dict] = [
    {'id': 'FOCO 75C', 'pos': (17.5, 2.5), 'type': 'fire'},
    {'id': 'AUTO 28C', 'pos': (13.5, -1.5), 'type': 'car'},
    {'id': 'CABLES', 'pos': (8.0, 7.6), 'type': 'cable'},
]


def enu_to_wgs84(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert local ENU-like world coordinates (m) to simulated WGS84 (lat, lon, alt)."""
    meters_per_deg_lat = 110692.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(ORIGIN_LAT))
    lat = ORIGIN_LAT + (y / meters_per_deg_lat)
    lon = ORIGIN_LON + (x / meters_per_deg_lon)
    alt = ORIGIN_ALT + z
    return lat, lon, alt


def quat_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Return (roll, pitch, yaw) in radians from quaternion (x, y, z, w)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Return 3x3 rotation matrix from unit quaternion (x, y, z, w)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def normalize_angle_rad(angle: float) -> float:
    """Wrap angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_span_deg(angles_deg: List[float]) -> float:
    """Compute minimal covered circular arc (in degrees) across observation angles."""
    if len(angles_deg) < 2:
        return 0.0
    wrapped = sorted(a % 360.0 for a in angles_deg)
    max_gap = 0.0
    for i in range(len(wrapped) - 1):
        gap = wrapped[i + 1] - wrapped[i]
        if gap > max_gap:
            max_gap = gap
    wrap_gap = (wrapped[0] + 360.0) - wrapped[-1]
    if wrap_gap > max_gap:
        max_gap = wrap_gap
    return max(0.0, 360.0 - max_gap)
