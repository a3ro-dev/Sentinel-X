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
- Obstacle detection and avoidance
- Hardware initialization and validation
- Error handling and recovery
- Speed control via PWM
- Filtered ultrasonic distance measurements

## Operation

The robot:
1. Initializes all hardware components
2. Continuously scans for obstacles using the ultrasonic sensor
3. If an obstacle is detected within 40cm:
   - Stops
   - Backs up briefly
   - Turns left to find a clear path
4. Continues forward if no obstacles are detected

## Error Handling

- The system performs multiple ultrasonic readings for accuracy
- Hardware initialization checks all components
- Main loop includes error recovery with a maximum error threshold
- System will safely stop if too many errors occur

## Usage

Upload 

car.py

 to an ESP32 with MicroPython installed. The code will run automatically on boot.

The robot will autonomously navigate while avoiding obstacles. No additional user input is required during operation.