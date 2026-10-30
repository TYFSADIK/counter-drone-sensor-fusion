"""Run: python demo.py  -> 60 s simulated engagement, prints the operator triage queue."""
import numpy as np

from fusion.confidence import triage_queue
from fusion.synthetic import SENSORS, scan, truth
from fusion.tracker import Tracker

rng = np.random.default_rng(7)
trk = Tracker()
for t in np.arange(0, 60, 1.0):
    updated = set()
    for sid in SENSORS:
        updated |= trk.process_scan(t, scan(t, sid, rng))
    trk.end_cycle(t, updated)

print(f"confirmed tracks: {len(trk.confirmed())}  (truth: 2 drones)")
for tid, p in truth(59).items():
    best = min(trk.confirmed(), key=lambda tr: np.linalg.norm(tr.x[:3] - p))
    print(f"  {tid}: nearest track #{best.id} error {np.linalg.norm(best.x[:3]-p):.1f} m")
print("triage queue (human decides):")
for row in triage_queue(trk.confirmed()):
    print("  ", row)
