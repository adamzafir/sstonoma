# IDE 2026 Robotics Codebase

Pybricks-based control code for a LEGO SPIKE Prime robot focused on gyro turning and line-tracking behaviors.

## Project Structure

- `main.py` - primary run loop for line tracking + gyro turns.
- `testmission1.py` - scripted mission sequence for testing path logic.
- `definitions.py` - hardware mapping, constants, and shared globals.
- `linetracking.py` - PD-style line tracking helpers (distance and junction modes).
- `gyrofunctions.py` - gyro reset/turn and related movement utilities.

## Requirements

- LEGO SPIKE Prime hub
- Pybricks firmware on the hub
- Python code deployed through the Pybricks app/editor

## Hardware Ports (from `definitions.py`)

- Left drive motor: `Port.B`
- Right drive motor: `Port.A`
- Attachment motor: `Port.F`
- Left color sensor: `Port.C`
- Right color sensor: `Port.D`
- Object color sensor: `Port.E`

## Quick Start

1. Open this project in your Pybricks workflow.
2. Confirm motor/sensor wiring matches the configured ports.
3. Tune constants in `definitions.py` if your field lighting or drivetrain differs.
4. Run `main.py` for continuous line-tracking behavior, or `testmission1.py` for a mission script.

## Tuning Notes

- `THRESHOLD`, `Black_Line_Value`, `White_Line_Value` impact line detection reliability.
- `PD_KP` / `PD_KD` and line tracking `kp` / `kd` values affect stability and responsiveness.
- Gyro constants (`Gyro_*`) control turn aggressiveness and heading correction.

## Safety

Test with wheels off the ground after parameter changes, then validate on the field at reduced speed before full runs.
