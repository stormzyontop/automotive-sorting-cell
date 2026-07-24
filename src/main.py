"""Entry point for the automotive sorting cell.

Simulates parts arriving on the line, runs each through the quality
check, and has the robot sort it into the appropriate bin while the
result is logged to CSV.
"""

import json
import logging
import random

from quality_check import PartMeasurement, check_quality
from robot_control import RobotController
from production_logger import ProductionLogger

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)


def load_config(path: str = "config.json") -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def simulate_measurement(part_id: str) -> PartMeasurement:
    """Stand-in for a real camera/sensor reading."""
    return PartMeasurement(
        part_id=part_id,
        dimension_mm=random.uniform(43, 57),
        surface_score=random.uniform(0.75, 1.0),
        color_deviation=random.uniform(0.0, 0.15),
    )


def run(num_parts: int = 10) -> None:
    config = load_config()

    robot = RobotController(config)
    robot.connect()

    logger_csv = ProductionLogger(config["logging"]["output_path"])

    for i in range(1, num_parts + 1):
        part_id = f"PART-{i:04d}"
        measurement = simulate_measurement(part_id)
        result = check_quality(measurement, config)

        category = "pass" if result.passed else ("rework" if result.reason == "surface_defect" else "fail")
        bin_id = robot.sort_part(part_id, category)

        logger_csv.log(part_id, "PASS" if result.passed else "FAIL", result.reason, bin_id)
        logger.info("%s -> %s (%s)", part_id, bin_id, result.reason)

    robot.disconnect()


if __name__ == "__main__":
    run()
