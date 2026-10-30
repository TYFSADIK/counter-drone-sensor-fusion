# Counter-Drone Sensor Fusion (simulation only)

Private research repo. A defensive counter-drone project that runs entirely in simulation. Synthetic radar, RF, acoustic, and camera tracks are fused with Kalman filters, scored for confidence, and triaged by a human. **Detection and decision support only. No effectors.**

Full write-up: [`docs/project-page.md`](docs/project-page.md)

## What's here

| File | Purpose |
|---|---|
| `fusion/models.py` | Constant-velocity 3D model, position and **bearing-only EKF** measurement models, angle-wrapped innovation |
| `fusion/tracker.py` | Multi-target tracker: χ² Mahalanobis gating, Hungarian GNN assignment, Joseph-form update, M-of-N confirmation, deletion. Bearing-only sensors refine tracks but never start them |
| `fusion/confidence.py` | Confidence from persistence, sensor diversity, NIS consistency, and covariance, plus a triage queue sorted by time-to-arrival |
| `fusion/synthetic.py` | Radar, EO, acoustic-bearing, and RF-bearing sensors with Pd, noise, and clutter, and two inbound drones |

## Quickstart

```bash
pip install -r requirements.txt
python demo.py && python -m unittest -v
```

## Next steps
- [ ] **IMM** (constant-velocity + coordinated-turn + hover) for manoeuvring multirotors
- [ ] Feed confirmed tracks into `counter-drone-scoring-harness` (replace its naive `cluster()`) and report GOSPA
- [ ] Real inputs: bearings from `acoustic-drone-detector`, plots from `sky-passive-radar-node`, all via the `multi-domain-common-operating-picture` message bus
- [ ] Out-of-sequence measurement handling (sensors with different latencies)
- [ ] Log every operator decision to `decision-black-box`
