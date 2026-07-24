"""CSV logging of production/sorting results."""

import csv
import os
from datetime import datetime, timezone


class ProductionLogger:
    FIELDNAMES = ["timestamp", "part_id", "result", "reason", "bin"]

    def __init__(self, output_path: str):
        self.output_path = output_path
        self._ensure_header()

    def _ensure_header(self) -> None:
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        if not os.path.exists(self.output_path):
            with open(self.output_path, "w", newline="", encoding="utf-8") as f:
                csv.DictWriter(f, fieldnames=self.FIELDNAMES).writeheader()

    def log(self, part_id: str, result: str, reason: str, bin_id: str) -> None:
        with open(self.output_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            writer.writerow({
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "part_id": part_id,
                "result": result,
                "reason": reason,
                "bin": bin_id,
            })
