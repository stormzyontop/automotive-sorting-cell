"""Quality inspection logic for parts moving through the sorting cell."""

from dataclasses import dataclass


@dataclass
class PartMeasurement:
    part_id: str
    dimension_mm: float
    surface_score: float
    color_deviation: float


@dataclass
class QualityResult:
    part_id: str
    passed: bool
    reason: str


def check_quality(measurement: PartMeasurement, config: dict) -> QualityResult:
    """Evaluate a part measurement against the configured thresholds."""
    thresholds = config["quality_check"]

    if not (thresholds["min_dimension_mm"] <= measurement.dimension_mm <= thresholds["max_dimension_mm"]):
        return QualityResult(measurement.part_id, False, "dimension_out_of_range")

    if measurement.surface_score < thresholds["min_surface_score"]:
        return QualityResult(measurement.part_id, False, "surface_defect")

    if measurement.color_deviation > thresholds["allowed_color_deviation"]:
        return QualityResult(measurement.part_id, False, "color_mismatch")

    return QualityResult(measurement.part_id, True, "ok")
