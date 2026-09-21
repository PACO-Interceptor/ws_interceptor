"""Unit tests for the bearing calculation module."""

import math

from interceptor.bearing import pixel_to_bearing, subtended_angle
import pytest

FX = 100.0
FY = 100.0
CX = 320.0
CY = 240.0


def test_center_pixel_bearing():
    """Pixel at center should yield unit vector along forward axis (1, 0, 0)."""
    bx, by, bz = pixel_to_bearing(CX, CY, FX, FY, CX, CY)
    assert bx == pytest.approx(1.0)
    assert by == pytest.approx(0.0)
    assert bz == pytest.approx(0.0)


def test_right_pixel_bearing():
    """Pixel to the right of center should yield negative y in FLU frame."""
    _, by, _ = pixel_to_bearing(CX + 100.0, CY, FX, FY, CX, CY)
    assert by < 0.0


def test_above_pixel_bearing():
    """Pixel above center (v < cy) should yield positive z in FLU frame."""
    _, _, bz = pixel_to_bearing(CX, CY - 50.0, FX, FY, CX, CY)
    assert bz > 0.0


def test_bearing_unit_norm():
    """Calculated bearing vector should have magnitude 1.0."""
    bx, by, bz = pixel_to_bearing(CX + 45.0, CY - 30.0, FX, FY, CX, CY)
    norm = math.sqrt(bx * bx + by * by + bz * bz)
    assert norm == pytest.approx(1.0)


def test_zero_focal_length_raises():
    """Zero focal length should raise a ValueError."""
    with pytest.raises(ValueError):
        pixel_to_bearing(CX, CY, 0.0, FY, CX, CY)
    with pytest.raises(ValueError):
        pixel_to_bearing(CX, CY, FX, 0.0, CX, CY)


def test_subtended_angle_matches_small_angle_approximation():
    """For a small centered box, the angle should be close to width / fx."""
    width = 10.0
    theta = subtended_angle(CX, CY, width, 1000.0, CX, CY)
    assert theta == pytest.approx(width / 1000.0, rel=1e-3)


def test_subtended_angle_grows_with_box_width():
    """A wider box at the same place must subtend a larger angle."""
    small = subtended_angle(CX, CY, 10.0, FX, CX, CY)
    big = subtended_angle(CX, CY, 40.0, FX, CX, CY)
    assert big > small


def test_subtended_angle_shrinks_off_axis():
    """The same box seen off-axis subtends less angle than at the center."""
    centered = subtended_angle(CX, CY, 40.0, FX, CX, CY)
    off_axis = subtended_angle(CX + 200.0, CY, 40.0, FX, CX, CY)
    assert off_axis < centered


def test_subtended_angle_rejects_bad_input():
    """Zero focal length or a non-positive width should raise a ValueError."""
    with pytest.raises(ValueError):
        subtended_angle(CX, CY, 10.0, 0.0, CX, CY)
    with pytest.raises(ValueError):
        subtended_angle(CX, CY, 0.0, FX, CX, CY)
