"""Confidence and triage. The system ranks; a human decides.

confidence in [0,1] combines:
  - persistence: hit ratio over the recent window
  - diversity:   number of independent sensor types that saw it (passive + active agreement)
  - consistency: mean NIS close to 1 means the track behaves like the model expects
  - covariance:  positional uncertainty (smaller is better)
"""
import math

import numpy as np


def confidence(tr, n_sensor_types_total=4):
    persistence = sum(tr.history) / max(len(tr.history), 1)
    diversity = min(len(tr.sensors_seen) / min(3, n_sensor_types_total), 1.0)
    nis = np.mean(tr.nis) if tr.nis else 3.0
    consistency = math.exp(-abs(nis - 1.0) / 2.0)
    pos_sigma = math.sqrt(np.trace(tr.P[0:3, 0:3]) / 3)
    certainty = 1.0 / (1.0 + pos_sigma / 25.0)
    return round(0.35 * persistence + 0.25 * diversity + 0.2 * consistency + 0.2 * certainty, 3)


def threat_context(tr, asset=(0.0, 0.0, 0.0)):
    """Range to the defended asset, closing speed (+ = approaching), time to arrival."""
    p, v = tr.x[0:3] - np.asarray(asset), tr.x[3:6]
    rng = float(np.linalg.norm(p))
    closing = float(-(p @ v) / rng) if rng > 1 else 0.0
    tta = rng / closing if closing > 0.5 else None
    return {"range_m": round(rng), "closing_mps": round(closing, 1), "tta_s": round(tta) if tta else None}


def triage_queue(tracks, asset=(0.0, 0.0, 0.0)):
    """Sorted list for the operator: soonest arrival first, then highest confidence."""
    rows = []
    for tr in tracks:
        ctx = threat_context(tr, asset)
        cls = max(tr.class_votes, key=tr.class_votes.get) if tr.class_votes else "unknown"
        rows.append({"track": tr.id, "confidence": confidence(tr), "class": cls,
                     "sensors": sorted(tr.sensors_seen), **ctx, "operator_decision": None})
    return sorted(rows, key=lambda r: (r["tta_s"] if r["tta_s"] is not None else 1e9, -r["confidence"]))
