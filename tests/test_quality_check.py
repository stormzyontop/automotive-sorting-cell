import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from quality_check import PartMeasurement, check_quality

CONFIG = {
    "quality_check": {
        "min_dimension_mm": 45.0,
        "max_dimension_mm": 55.0,
        "min_surface_score": 0.85,
        "allowed_color_deviation": 0.1,
    }
}


def test_part_within_all_thresholds_passes():
    measurement = PartMeasurement("PART-0001", dimension_mm=50.0, surface_score=0.95, color_deviation=0.02)
    result = check_quality(measurement, CONFIG)
    assert result.passed
    assert result.reason == "ok"


def test_part_too_small_fails_dimension_check():
    measurement = PartMeasurement("PART-0002", dimension_mm=40.0, surface_score=0.95, color_deviation=0.02)
    result = check_quality(measurement, CONFIG)
    assert not result.passed
    assert result.reason == "dimension_out_of_range"


def test_part_too_large_fails_dimension_check():
    measurement = PartMeasurement("PART-0003", dimension_mm=60.0, surface_score=0.95, color_deviation=0.02)
    result = check_quality(measurement, CONFIG)
    assert not result.passed
    assert result.reason == "dimension_out_of_range"


def test_low_surface_score_fails():
    measurement = PartMeasurement("PART-0004", dimension_mm=50.0, surface_score=0.5, color_deviation=0.02)
    result = check_quality(measurement, CONFIG)
    assert not result.passed
    assert result.reason == "surface_defect"


def test_color_deviation_too_high_fails():
    measurement = PartMeasurement("PART-0005", dimension_mm=50.0, surface_score=0.95, color_deviation=0.5)
    result = check_quality(measurement, CONFIG)
    assert not result.passed
    assert result.reason == "color_mismatch"
