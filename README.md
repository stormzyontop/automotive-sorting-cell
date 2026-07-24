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
│   ├── robodk_setup.py       # builds the station in RoboDK (table, workpieces, sorting bins)
│   └── robodk_simulate.py    # drives the robot through a pick -> inspect -> sort cycle
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
[RoboDK](https://robodk.com/): an ABB IRB 120, an infeed table holding
three workpieces to pick and inspect, and three open-top sorting bins
(pass / fail / rework) whose bin IDs match `robot.bins` in
[config.json](config.json). Requires RoboDK to be installed (it will
auto-start if not already running):

```bash
python src/robodk_setup.py
```

The script is idempotent — re-running it rebuilds the cell from scratch
instead of duplicating items.

## RoboDK pick-and-sort simulation

[src/robodk_simulate.py](src/robodk_simulate.py) actually drives the
robot: it (re)builds the station, assigns each of the three workpieces a
random "true" category (pass / fail / rework) and tints it to match that
bin's color — standing in for what a real vision/quality inspection would
report — then picks each part off the table, moves it to the matching
bin, and releases it. Every target pose is checked for reachability with
`SolveIK` before any motion starts, so an out-of-reach layout fails fast
with a clear error instead of a mid-run RoboDK fault.

```bash
python src/robodk_simulate.py
```

After the cycle, it verifies the result and prints a report — this is
the "pre-colored parts, check the robot actually sorts them correctly"
self-test: each workpiece's final parent bin AND its actual world
position (must fall inside that bin's footprint) are checked against its
assigned category. Exit code is 0 if every part landed correctly, 1
otherwise, so it can be scripted/re-run repeatedly. `main.py` remains a
separate, independent logic-only simulation (random measurements → CSV),
not wired to the RoboDK motion.

## Tests

```bash
pytest
```

## Configuration

All thresholds (part dimensions, surface quality, color deviation) and
robot/bin settings live in [config.json](config.json) and can be tuned
without touching code.
