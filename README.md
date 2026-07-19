# Autonomous Pick-and-Place Robot

Overhead vision-guided 2WD robot with 2-DOF arm.  
2nd-year ECE mini project.

## Hardware
- Custom 3D-printed chassis
- ESP32
- TB6612FNG dual motor driver
- 2x servo arm (base + gripper)
- Overhead 720p USB camera
- Perfboard wiring

## Software
- **Python/OpenCV** — color blob detection, state machine
- **Arduino C++** — UDP command receiver, motor ramping, servo interpolation

## How It Works
1. Overhead camera tracks robot (blue/green markers) and targets (red/yellow)
2. PC computes heading, distance, and arm offset
3. UDP commands stream to ESP32 at 10ms
4. State machine: FIND_OBJECT → ORIENT → APPROACH → GRAB → FIND_DEST → PLACE

## Key Challenge
Arm was offset from robot center due to weight distribution. Added `ARM_OFFSET_PX` 
parameter to navigation frame instead of reprinting chassis.

## Files
- [esp32_robot.ino](ESP32.ino) — Motor control, servo interpolation, UDP parser, watchdog
- [vision_controller.py](Vision.py) — OpenCV pipeline, state machine, navigation logic

## Demo Video
[Watch the robot in action](https://www.linkedin.com/posts/aditya-anil-a48ba7359_robotics-embeddedsystems-opencv-ugcPost-7484576365644795904-moxu/?utm_source=share&utm_medium=member_desktop&rcm=ACoAAFlX8xgBJyX0IBHEsC9QDMeTRqP9eZLxdfw)
