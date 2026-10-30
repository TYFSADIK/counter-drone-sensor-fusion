"""Synthetic multi-sensor feed: radar (position), EO (position, short range), two acoustic/RF bearing sensors."""
import numpy as np

from .models import Measurement

SENSORS = {
    "radar-1":   {"kind": "position", "origin": (0, 0, 0), "sigma": 8.0, "range": 2500, "pd": 0.9, "clutter": 0.3},
    "eo-1":      {"kind": "position", "origin": (300, 0, 0), "sigma": 3.0, "range": 700, "pd": 0.8, "clutter": 0.0},
    "acoustic-1": {"kind": "bearing", "origin": (-200, 200, 0), "sigma": np.deg2rad(3), "range": 600, "pd": 0.7, "clutter": 0.0},
    "rf-1":      {"kind": "bearing", "origin": (200, -300, 0), "sigma": np.deg2rad(5), "range": 3000, "pd": 0.6, "clutter": 0.0},
}


def truth(t):
    """Two drones inbound toward the asset at origin, one loitering bird-like clutter source is in radar clutter."""
    return {
        "D1": np.array([2000 - 15 * t, 300 - 2 * t, 80.0]),
        "D2": np.array([-1500 + 10 * t, -1200 + 8 * t, 60.0]),
    }


def scan(t, sensor_id, rng):
    s = SENSORS[sensor_id]
    o = np.array(s["origin"], float)
    out = []
    for tid, p in truth(t).items():
        if np.linalg.norm(p[:2] - o[:2]) > s["range"] or rng.random() > s["pd"]:
            continue
        if s["kind"] == "position":
            z = p + rng.normal(0, s["sigma"], 3)
            R = np.eye(3) * s["sigma"] ** 2
        else:
            z = np.array([np.arctan2(p[1] - o[1], p[0] - o[0]) + rng.normal(0, s["sigma"])])
            R = np.array([[s["sigma"] ** 2]])
        cls = ("UAV/multirotor", 0.6) if sensor_id in ("eo-1", "acoustic-1", "rf-1") else ("unknown", 0.0)
        out.append(Measurement(t, sensor_id, s["kind"], z, R, o, *cls))
    for _ in range(rng.poisson(s["clutter"])):
        if s["kind"] == "position":
            r, a = s["range"] * np.sqrt(rng.random()), rng.uniform(0, 2 * np.pi)
            z = o + np.array([r * np.cos(a), r * np.sin(a), rng.uniform(10, 150)])
            out.append(Measurement(t, sensor_id, "position", z, np.eye(3) * s["sigma"] ** 2, o))
    return out
