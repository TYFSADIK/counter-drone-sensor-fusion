"""Multi-sensor multi-target tracker.

- EKF per track (position + bearing-only updates)
- Mahalanobis gating (chi-square) + global nearest neighbour (Hungarian) per sensor scan
- M-of-N confirmation, miss-count deletion
- Bearing-only reports never start tracks (they can't be localised alone); they only update
"""
from dataclasses import dataclass, field
from itertools import count

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.stats import chi2

from .models import Measurement, cv_model, h_and_H, innovation

_ids = count(1)


@dataclass
class Track:
    x: np.ndarray
    P: np.ndarray
    t: float
    id: int = field(default_factory=lambda: next(_ids))
    status: str = "tentative"          # tentative | confirmed | deleted
    hits: int = 1
    misses: int = 0
    history: list = field(default_factory=list)     # recent hit(1)/miss(0)
    sensors_seen: set = field(default_factory=set)
    nis: list = field(default_factory=list)         # normalised innovation squared
    class_votes: dict = field(default_factory=dict)

    def predict(self, t, q):
        dt = t - self.t
        if dt > 0:
            F, Q = cv_model(dt, q)
            self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + Q, t

    def gate_distance(self, m: Measurement):
        zp, H = h_and_H(self.x, m)
        y = innovation(m.z, zp, m.kind)
        S = H @ self.P @ H.T + m.R
        return float(y @ np.linalg.solve(S, y)), y, H, S

    def update(self, m: Measurement):
        d2, y, H, S = self.gate_distance(m)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ m.R @ K.T      # Joseph form: stays symmetric PSD
        self.hits += 1
        self.misses = 0
        self.history = (self.history + [1])[-10:]
        self.sensors_seen.add(m.sensor_id)
        self.nis = (self.nis + [d2 / len(y)])[-20:]
        if m.class_conf > 0:
            self.class_votes[m.classification] = self.class_votes.get(m.classification, 0) + m.class_conf


class Tracker:
    def __init__(self, q=3.0, gate_prob=0.997, confirm=(3, 5), max_misses=5, init_vel_sigma=30.0):
        self.q, self.confirm, self.max_misses = q, confirm, max_misses
        self.gate_prob = gate_prob
        self.init_vel_sigma = init_vel_sigma
        self.tracks: list[Track] = []

    def _gate(self, dim):
        return chi2.ppf(self.gate_prob, dim)

    def process_scan(self, t: float, meas: list[Measurement]):
        """All measurements from ONE sensor at ONE time. Returns ids of tracks updated or created."""
        live = [tr for tr in self.tracks if tr.status != "deleted"]
        for tr in live:
            tr.predict(t, self.q)
        n, m = len(live), len(meas)
        assigned_m, updated = set(), set()
        if n and m:
            cost = np.full((n, m), 1e6)
            for i, tr in enumerate(live):
                for j, z in enumerate(meas):
                    d2, *_ = tr.gate_distance(z)
                    if d2 < self._gate(len(z.z)):
                        cost[i, j] = d2
            for i, j in zip(*linear_sum_assignment(cost)):
                if cost[i, j] < 1e6:
                    live[i].update(meas[j])
                    assigned_m.add(j)
                    updated.add(live[i].id)
        for j, z in enumerate(meas):
            if j not in assigned_m and z.kind == "position":
                P = np.diag([*np.diag(z.R), *[self.init_vel_sigma ** 2] * 3])
                tr = Track(np.r_[z.z, 0, 0, 0], P, t)
                tr.sensors_seen.add(z.sensor_id)
                tr.history = [1]
                self.tracks.append(tr)
                updated.add(tr.id)
        return updated

    def end_cycle(self, t: float, updated_ids: set):
        """Call once per fusion cycle with ids of tracks updated by any sensor this cycle."""
        M, N = self.confirm
        for tr in self.tracks:
            if tr.status == "deleted":
                continue
            if tr.id not in updated_ids:
                tr.misses += 1
                tr.history = (tr.history + [0])[-10:]
            if tr.status == "tentative" and sum(tr.history[-N:]) >= M:
                tr.status = "confirmed"
            limit = self.max_misses if tr.status == "confirmed" else 2
            if tr.misses > limit:
                tr.status = "deleted"

    def confirmed(self):
        return [t for t in self.tracks if t.status == "confirmed"]
