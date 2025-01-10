# This robot controls a robot with two motors, a servo, and an ultrasonic sensor using an ESP32 microcontroller. 
# It includes functions to move the robot forward, backward, turn left, turn right, and stop. 
# It also measures distance using the ultrasonic sensor and adjusts the robot's movement to avoid obstacles.
# Functions:
#     stop():
#     set_speeds(left_speed, right_speed):
#     measure_distance():
#         Triggers the ultrasonic sensor three times, calculates the distance for each reading, 
#         and returns the average distance. If no valid readings are obtained, it returns 999.
#         Returns:
#     set_servo_angle(angle):
#     move_forward():
#     move_backward():
#     turn_left():
#     turn_right():
# Main loop:
#     Continuously measures the distance in front of the robot. If an obstacle is detected within 25 cm, 
#     it stops the robot, measures distances at different angles (45, 90, 135 degrees), and chooses the 
#     direction with the most space to move. If no clear path is found, it moves backward briefly. 
#     Otherwise, it turns towards the direction with the most space and continues moving forward.

from machine import Pin, PWM, time_pulse_us
import time

# Motor pins
IN1 = Pin(18, Pin.OUT)  # Motor 1
IN2 = Pin(5, Pin.OUT)
IN3 = Pin(15, Pin.OUT)  # Motor 2
IN4 = Pin(2, Pin.OUT)

# Enable pins (PWM)
EN1 = PWM(Pin(19), freq=1000)  # Motor 1 speed control
EN2 = PWM(Pin(21), freq=1000)  # Motor 2 speed control

# Servo Pin
SERVO = PWM(Pin(4), freq=50)
# Initialize servo properly for ESP32

# Ultrasonic sensor pins
TRIG = Pin(12, Pin.OUT)
ECHO = Pin(14, Pin.IN)

SAFE_DISTANCE = 35  # Increased safe distance

def stop():
    """
    Stops both motors.
    """
    IN1.value(0)
    IN2.value(0)
    IN3.value(0)
    IN4.value(0)

def set_speeds(left_speed, right_speed):
    """
    Sets the speed of the left and right motors.
    
    Args:
        left_speed (int): Speed for the left motor (0-1023).
        right_speed (int): Speed for the right motor (0-1023).
    """
    left = min(max(0, left_speed), 1023)
    right = min(max(0, right_speed), 1023)
    EN1.duty(left)
    EN2.duty(right)

def init_hardware():
    """
    Initialize and verify hardware components.
    Returns True if all OK, False if there are issues.
    """
    try:
        # Test motors
        set_speeds(0, 0)
        stop()
        
        # Test servo range
        set_servo_angle(90)
        time.sleep_ms(500)
        
        # Test ultrasonic
        dist = measure_distance()
        if dist == 999:
            raise Exception("Ultrasonic sensor not responding")
            
        return True
    except:
        return False

def measure_distance():
    """
    Improved distance measurement with validation
    """
    readings = []
    for _ in range(3):
        try:
            TRIG.value(0)
            time.sleep_us(2)
            TRIG.value(1)
            time.sleep_us(10)
            TRIG.value(0)
            
            duration = time_pulse_us(ECHO, 1, 30000)  # 30ms timeout
            
            # Validate reading (typical HC-SR04 range: 2cm to 400cm)
            if 0 < duration < 23529:  # Max time for 400cm
                dist_cm = (duration * 0.0343) / 2
                if 2 <= dist_cm <= 400:
                    readings.append(dist_cm)
        except:
            continue
            
    # Require at least 2 valid readings
    if len(readings) >= 2:
        # Remove outliers
        readings.sort()
        return sum(readings[:-1]) / (len(readings)-1)
    return 999

def set_servo_angle(angle):
    """
    Sets the servo to a specific angle.
    
    Args:
        angle (int): The angle to set the servo to (0-180).
    """
    duty = int(51 + (angle / 180) * 51)
    SERVO.duty(duty)

def move_forward():
    """
    Moves the robot forward at a moderate speed.
    """
    IN1.value(1)
    IN2.value(0)
    IN3.value(1)
    IN4.value(0)
    set_speeds(500, 500)  # Lower speed for smoother movement

def move_backward():
    """
    Moves the robot backward at a moderate speed.
    """
    IN1.value(0)
    IN2.value(1)
    IN3.value(0)
    IN4.value(1)
    set_speeds(400, 400)  # Enough to retreat safely

def turn_left():
    """
    Turns the robot left.
    """
    IN1.value(0)
    IN2.value(1)
    IN3.value(1)
    IN4.value(0)
    # Differential speed for smoother turns
    set_speeds(400, 800)

def turn_right():
    """
    Turns the robot right.
    """
    IN1.value(1)
    IN2.value(0)
    IN3.value(0)
    IN4.value(1)
    # Differential speed for smoother turns
    set_speeds(800, 400)

# Initialize hardware
if not init_hardware():
    print("Hardware initialization failed!")
    raise SystemExit()

# Main control variables
angle = 90  # Start centered
direction = 1
last_valid_distance = 999
error_count = 0
MAX_ERRORS = 5

# Main loop with improved error handling
while True:
    try:
        # Reset error count on successful iteration
        error_count = 0
        
        set_servo_angle(angle)
        dist = measure_distance()
        
        # Keep track of last valid reading
        if dist != 999:
            last_valid_distance = dist
        
        # Use last valid distance if current reading failed
        current_dist = dist if dist != 999 else last_valid_distance
        
        if current_dist < SAFE_DISTANCE:
            stop()
            # Back up longer to avoid collisions
            move_backward()
            time.sleep_ms(500)
            stop()

            # Perform a three-angle scan (left, center, right)
            angles_to_check = [45, 90, 135]
            measured_distances = {}
            for a in angles_to_check:
                set_servo_angle(a)
                time.sleep_ms(200)  # Longer wait for servo
                measured_distances[a] = measure_distance()

            # Choose the angle with the greatest distance
            best_angle = max(measured_distances, key=measured_distances.get)
            if measured_distances[best_angle] < SAFE_DISTANCE:
                # If no path is clear, attempt a larger backward move
                move_backward()
                time.sleep_ms(1000)
                stop()
            else:
                # Turn in the best direction
                if best_angle < 90:
                    turn_left()
                elif best_angle > 90:
                    turn_right()
                else:
                    # Straight ahead if center is clear
                    pass
                time.sleep_ms(500)
                stop()
        else:
            # Move forward if clear
            move_forward()
        
        # Smooth servo sweep
        angle += (3 * direction)  # Reduced step size
        if angle >= 135:
            angle = 135
            direction = -1
        elif angle <= 45:
            angle = 45
            direction = 1
            
    except Exception as e:
        error_count += 1
        stop()
        set_speeds(0, 0)
        set_servo_angle(90)
        
        if error_count >= MAX_ERRORS:
            print("Too many errors, stopping!")
            break
            
        # Emergency turn on error
        turn_left()
        time.sleep_ms(200)
        stop()
    
    time.sleep_ms(50)  # Slightly longer delay for smoother operation