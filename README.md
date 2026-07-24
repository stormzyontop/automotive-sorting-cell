# automotive-sorting-cell

A small simulation of an automated quality-inspection and sorting cell for
automotive parts, as found on a production line: parts are measured,
checked against quality thresholds, and a robot sorts each part into a
pass / rework / fail bin. Every result is logged to CSV for traceability.

## Project structure

```
automotive-sorting-cell/
├── README.md
├── LICENSE
├── requirements.txt
├── config.json
├── src/
│   ├── main.py               # entry point, orchestrates the cell
│   ├── quality_check.py      # measurement -> pass/fail logic
│   ├── robot_control.py      # simulated robot interface
│   ├── production_logger.py  # CSV production logging
│   └── robodk_setup.py       # builds the station in RoboDK (frames, pallets, workpiece)
├── docs/
│   └── demo.gif
├── data/
│   └── production_log.csv    # generated at runtime
└── tests/
    └── test_quality_check.py
```

## Setup

```bash
pip install -r requirements.txt
```

## Usage

```bash
python src/main.py
```

This simulates parts moving through the cell, evaluates each against the
thresholds in [config.json](config.json), and writes results to
`data/production_log.csv`.

## RoboDK station

[src/robodk_setup.py](src/robodk_setup.py) builds a visual station in
[RoboDK](https://robodk.com/): an ABB IRB 120, a base cell frame, an infeed
frame with a placeholder workpiece, and three pallets (pass / fail /
rework) whose bin IDs match `robot.bins` in [config.json](config.json).
Requires RoboDK to be installed (it will auto-start if not already
running):

```bash
python src/robodk_setup.py
```

The script is idempotent — re-running it rebuilds the cell from scratch
instead of duplicating items. It only builds the layout; it does not yet
drive the robot's motion, so `main.py` still runs as an independent logic
simulation (see below).

## Tests

```bash
pytest
```

## Configuration

All thresholds (part dimensions, surface quality, color deviation) and
robot/bin settings live in [config.json](config.json) and can be tuned
without touching code.
