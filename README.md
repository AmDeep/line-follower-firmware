# Line Follower Robot Firmware

## Engineering evidence

- `tools/line_controller_sim.py` converts five reflectance readings into a weighted line error and left/right PWM commands.
- Run `python tools/line_controller_sim.py 100 500 900 500 100` to inspect steering behavior.
- The control interface is kept separate from sensor scaling and motor actuation so tuning remains approachable.

## Objective

Provide a clean, tunable proportional controller for a differential-drive robot that follows a dark line on a light surface using an array of five reflectance sensors. The code is structured so that integral and derivative terms can be added later without rewriting the sensor or motor layers.

## Strategy

- Calibrate each sensor for a few seconds at startup so the same code works under different lighting conditions.
- Normalize readings to a 0-1000 range, then compute a weighted average that maps the line position to a signed error in the range roughly -2000 to +2000.
- Apply a single proportional gain to the error and add/subtract it from a base speed for the left and right motors.
- Keep the motor driver interface (direction pins + PWM) behind a small helper so the control law stays independent of the H-bridge wiring.

## What worked

- Weighted-average position estimation produced a smooth error signal even when the line was under only one or two sensors.
- A short calibration period that recorded min and max for each channel removed most ambient-light sensitivity.
- Constraining motor commands to a maximum PWM value prevented the robot from accelerating out of control on long straight sections.
- Serial output of error and both motor speeds made on-the-fly gain tuning practical.

## What failed and how it was resolved

- Without calibration the raw ADC values saturated under bright light and the position estimate collapsed. Resolution: always run a 3-second min/max calibration at startup.
- A pure on/off (bang-bang) controller caused oscillation on curves. Resolution: replace with a proportional term that scales correction with error magnitude.
- When the line was completely lost the weighted average divided by a near-zero total and produced garbage. Resolution: detect a low total reflectance sum and fall back to zero error (drive straight) or a previous error memory.

## Engineering principles and frameworks used

- Proportional control as the simplest closed-loop law.
- Sensor normalization and weighted averaging for position estimation.
- Separation of sensing, control law, and actuation.
- Calibration at run time rather than hard-coded thresholds.
- Observability of the error signal for iterative tuning.

## Hardware

- Arduino Uno or Nano
- Two DC motors with an L298N or TB6612 driver
- Five IR reflectance sensors (TCRT5000 or QTR-style array)
- Chassis, wheels and battery pack

## Wiring (example)

| Sensor      | Arduino |
|-------------|---------|
| Left outer  | A0      |
| Left inner  | A1      |
| Center      | A2      |
| Right inner | A3      |
| Right outer | A4      |

| Motor driver   | Arduino |
|----------------|---------|
| ENA (left PWM) | D5      |
| IN1            | D6      |
| IN2            | D7      |
| ENB (right PWM)| D9      |
| IN3            | D10     |
| IN4            | D11     |

## Software

Upload line_follower.ino. The sketch calibrates for three seconds (move the sensors over both black and white), then starts following.

## Possible extensions

- Add integral and derivative terms for full PID.
- Detect intersections and choose a direction.
- Add an ultrasonic sensor for obstacle avoidance.
