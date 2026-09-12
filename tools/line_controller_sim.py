"""Simulate the five-sensor line position and proportional steering law."""
from __future__ import annotations

import argparse


def position(readings: list[int]) -> float:
    weights = (-2.0, -1.0, 0.0, 1.0, 2.0)
    total = sum(readings)
    return sum(value * weight for value, weight in zip(readings, weights)) / total if total else 0.0


def command(readings: list[int], base_speed: int, gain: float) -> tuple[int, int, float]:
    error = position(readings)
    correction = gain * error
    return round(base_speed + correction), round(base_speed - correction), error


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("readings", nargs=5, type=int, default=[100, 500, 900, 500, 100])
    parser.add_argument("--base-speed", type=int, default=120)
    parser.add_argument("--gain", type=float, default=35.0)
    args = parser.parse_args()
    left, right, error = command(args.readings, args.base_speed, args.gain)
    print(f"line_error={error:.3f} left_pwm={left} right_pwm={right}")
