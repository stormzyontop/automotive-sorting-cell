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
│   └── production_logger.py  # CSV production logging
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

## Tests

```bash
pytest
```

## Configuration

All thresholds (part dimensions, surface quality, color deviation) and
robot/bin settings live in [config.json](config.json) and can be tuned
without touching code.
