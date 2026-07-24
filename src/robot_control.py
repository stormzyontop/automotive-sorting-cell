"""Robot controller for routing inspected parts to the correct output bin.

A real deployment would talk to the robot controller over serial/TCP.
This module simulates that link so the cell logic can be developed and
tested without hardware attached.
"""

import logging

logger = logging.getLogger(__name__)


class RobotController:
    def __init__(self, config: dict):
        self.config = config["robot"]
        self.connected = False

    def connect(self) -> None:
        logger.info("Connecting to robot (%s) on %s", self.config["connection"], self.config["port"])
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def sort_part(self, part_id: str, category: str) -> str:
        """Move a part into the bin matching its quality category.

        category is one of "pass", "fail", "rework".
        Returns the bin identifier the part was placed into.
        """
        if not self.connected:
            raise RuntimeError("Robot is not connected")

        bin_id = self.config["bins"][category]
        logger.info("Moving part %s to %s (speed=%s mm/s)", part_id, bin_id, self.config["move_speed_mm_s"])
        return bin_id
