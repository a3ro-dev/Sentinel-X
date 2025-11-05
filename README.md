# Sentinel-X

This project implements a autonomous robot control system using an ESP32 microcontroller. The robot features dual motor control, servo steering, and ultrasonic obstacle detection.

## Hardware Requirements

- ESP32 microcontroller
- 2 DC motors with H-bridge driver
- Servo motor
- HC-SR04 ultrasonic sensor
- Power supply

## Pin Configuration

```
Motors:
- IN1: GPIO18 (Motor 1)
- IN2: GPIO5
- IN3: GPIO15 (Motor 2) 
- IN4: GPIO2
- EN1: GPIO19 (PWM)
- EN2: GPIO21 (PWM)

Servo:
- SERVO: GPIO4 (PWM)

Ultrasonic Sensor:
- TRIG: GPIO12
- ECHO: GPIO14
```

## Features

- Basic movement controls (forward, backward, left, right)
- **Enhanced Algorithm Features:**
  - Exponential Moving Average (EMA) filtering for smoother distance measurements
  - Median filtering for robust outlier rejection
  - Weighted path scoring that prefers forward direction
  - Path width estimation for better navigation decisions
  - Adaptive safe distance based on current speed
  - Proportional steering for smoother, more natural turns
  - Predictive obstacle avoidance using approach rate calculation
  - Adaptive servo sweep density based on obstacle proximity
- Intelligent obstacle detection and avoidance with multi-angle scanning
- Hardware initialization and validation
- Error handling and recovery with error threshold
- Speed control via PWM with differential turning
- Continuous servo scanning for improved obstacle detection

## Operation

The robot:
1. Initializes all hardware components (motors, servo, ultrasonic sensor)
2. Continuously sweeps the servo between 45° and 135° with adaptive step size:
   - Slower, denser scanning (2° steps) when obstacles are close (< 50cm)
   - Normal scanning (3° steps) at medium distances (50-100cm)
   - Faster scanning (5° steps) when path is clear (> 100cm)
3. Takes 3 distance readings per measurement with advanced filtering:
   - Uses median of readings for outlier rejection
   - Applies Exponential Moving Average (EMA) for smoothing
   - Validates readings within HC-SR04 range (2-400cm)
4. Calculates adaptive safe distance based on:
   - Current robot speed (higher speed = larger safe distance)
   - Approach rate (detects rapidly closing obstacles)
   - Increases buffer distance when approaching obstacles quickly
5. If an obstacle is detected within adaptive safe distance:
   - Stops immediately
   - Backs up for 500ms
   - Performs a three-angle scan (45°, 90°, 135°) to find the clearest path
   - Calculates weighted scores for each path (prefers forward direction)
   - Estimates path width from left and right measurements
   - If no clear path is found (< 35cm in all directions), backs up for 1000ms
   - Otherwise, applies proportional steering toward the best direction
6. Proportional steering provides smooth, natural turns instead of sharp binary turns
7. Uses the last valid distance reading if current measurement fails

## Error Handling

- The system performs 3 ultrasonic readings per measurement for accuracy
- Advanced filtering pipeline:
  - Median filtering for robust outlier rejection
  - Exponential Moving Average (EMA) smoothing for noise reduction
  - Validates readings to ensure they're within HC-SR04 range (2-400cm)
- Hardware initialization checks all components before operation
- Main loop includes error recovery with a maximum error threshold (5 errors)
- System will safely stop if too many consecutive errors occur
- Emergency turn performed on error recovery
- Graceful degradation: uses last valid reading if current measurement fails

## Usage

Upload `ota/car.py` to an ESP32 with MicroPython installed. The code will run automatically on boot.

The robot will autonomously navigate while avoiding obstacles. No additional user input is required during operation.