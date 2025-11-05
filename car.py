# Sentinel-X Enhanced Obstacle Avoidance Robot
# 
# This robot controls a two-motor robot with servo-mounted ultrasonic sensor using ESP32 microcontroller.
# Features enhanced algorithms for intelligent obstacle avoidance and smooth navigation.
#
# Enhanced Algorithm Features:
#     - Exponential Moving Average (EMA) filtering for smooth distance measurements
#     - Median filtering for robust outlier rejection
#     - Weighted path scoring that prefers forward direction
#     - Path width estimation for better navigation decisions
#     - Adaptive safe distance based on current speed
#     - Proportional steering for smooth, natural turns
#     - Predictive obstacle avoidance using approach rate calculation
#     - Adaptive servo sweep density based on obstacle proximity
#
# Core Functions:
#     stop(): Stops both motors
#     set_speeds(left_speed, right_speed): Sets motor speeds (0-1023)
#     measure_distance(): Takes 3 readings, applies median filter + EMA smoothing
#     set_servo_angle(angle): Sets servo position (0-180°)
#     move_forward(), move_backward(), turn_left(), turn_right(): Movement controls
#     calculate_path_score(angle, distance): Weighted scoring for path selection
#     get_adaptive_safe_distance(speed): Dynamic safe distance calculation
#     proportional_steer(target_angle, base_speed): Smooth steering control
#
# Main loop:
#     1. Adaptive servo sweep (2-5° steps based on proximity)
#     2. Enhanced distance measurement with EMA and median filtering
#     3. Adaptive safe distance calculation (speed + approach rate)
#     4. When obstacle detected: 3-angle scan with weighted scoring
#     5. Path width estimation and proportional steering
#     6. Smooth turns instead of sharp pivots

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

SAFE_DISTANCE = 45  # Base safe distance (increased for better clearance)
BASE_SAFE_DISTANCE = 45  # Base value for adaptive calculations
ROBOT_WIDTH = 20  # Approximate robot width in cm
EMA_ALPHA = 0.3  # EMA smoothing factor (0-1, lower = more smoothing)

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
    Improved distance measurement with median filtering and EMA smoothing
    """
    global distance_ema
    
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
            
    # Use median filtering for better outlier rejection
    if len(readings) >= 2:
        readings.sort()
        # Get median (middle value for odd length, average of middle two for even)
        if len(readings) == 3:
            raw_distance = readings[1]  # Middle value
        else:  # len(readings) == 2
            raw_distance = (readings[0] + readings[1]) / 2
        
        # Apply EMA smoothing
        if distance_ema == 999:
            distance_ema = raw_distance  # Initialize on first reading
        else:
            distance_ema = EMA_ALPHA * raw_distance + (1 - EMA_ALPHA) * distance_ema
        
        return distance_ema
    
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
    set_speeds(450, 450)  # Increased from 400 for faster retreat

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

def calculate_path_score(angle, distance):
    """
    Calculate a weighted score for path selection.
    Prefers forward direction and greater distances.
    
    Args:
        angle (int): Angle of the path (45-135 degrees)
        distance (float): Distance to obstacle at that angle
    
    Returns:
        float: Weighted score for the path
    """
    # Angle factor: 1.0 for forward (90°), slightly lower for sides
    # Formula: 1.0 - (abs(angle - 90) * 0.0005)
    # At 45° or 135°: 1.0 - (45 * 0.0005) = 0.9775
    angle_factor = 1.0 - (abs(angle - 90) * 0.0005)
    
    # Score is distance weighted by angle preference
    return distance * angle_factor


def get_adaptive_safe_distance(current_speed):
    """
    Calculate adaptive safe distance based on current speed.
    Higher speed requires more stopping distance.
    
    Args:
        current_speed (int): Current motor speed (0-1023)
    
    Returns:
        float: Adaptive safe distance in cm
    """
    # Linear scaling: add extra distance based on speed
    # At speed 500, adds about 17cm to base 35cm = ~52cm total
    speed_factor = current_speed / 1023.0
    return BASE_SAFE_DISTANCE + (speed_factor * 35)

def proportional_steer(target_angle, base_speed=500):
    """
    Calculate motor speeds for proportional steering toward target angle.
    
    Args:
        target_angle (int): Target angle to steer toward (45, 90, or 135)
        base_speed (int): Base forward speed
    
    Returns:
        tuple: (left_speed, right_speed)
    """
    # Calculate angle difference from center (90°)
    angle_diff = target_angle - 90
    
    # Steering factor: maps angle difference to speed adjustment
    # At 45° (left): angle_diff = -45, adds to right, reduces left
    # At 135° (right): angle_diff = 45, adds to left, reduces right
    steering_adjustment = abs(angle_diff) * 3  # Scaling factor
    
    if angle_diff < 0:  # Turn left
        left_speed = base_speed - steering_adjustment
        right_speed = base_speed + steering_adjustment
    elif angle_diff > 0:  # Turn right
        left_speed = base_speed + steering_adjustment
        right_speed = base_speed - steering_adjustment
    else:  # Straight
        left_speed = base_speed
        right_speed = base_speed
    
    # Ensure speeds are within valid range
    left_speed = min(max(200, left_speed), 1023)
    right_speed = min(max(200, right_speed), 1023)
    
    return int(left_speed), int(right_speed)

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

# Algorithm enhancement variables
distance_ema = 999  # Initialize EMA for distance filtering
current_speed = 500  # Track current forward speed for adaptive safe distance
previous_distance = 999  # Track previous distance for approach rate calculation
stuck_counter = 0  # Track consecutive failed path attempts for stuck detection

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
        
        # Calculate approach rate ONLY at center position (90°) to avoid servo sweep contamination
        approach_rate = 0
        if angle == 90:  # Only track when looking straight ahead
            if previous_distance != 999 and current_dist != 999:
                raw_approach_rate = previous_distance - current_dist
                # Cap approach rate to realistic values (max 5cm per 50ms iteration)
                approach_rate = max(-5, min(5, raw_approach_rate))
                # Positive approach_rate means obstacle is getting closer
            
            previous_distance = current_dist
        
        # Calculate adaptive safe distance with sanity checks
        adaptive_safe = get_adaptive_safe_distance(current_speed)
        
        # If approaching obstacle quickly, increase safe distance (reduced multiplier and capped)
        if approach_rate > 2:  # Approaching faster than 2cm per loop iteration
            additional_distance = min(approach_rate * 3, 30)  # Max 30cm extra (reduced from 10x)
            adaptive_safe = min(adaptive_safe + additional_distance, 100)  # Cap at 100cm total
        
        if current_dist < adaptive_safe:
            stop()
            # Back up longer to avoid collisions
            move_backward()
            time.sleep_ms(700)  # Increased from 500ms for better clearance
            stop()

            # Perform a three-angle scan (left, center, right)
            angles_to_check = [45, 90, 135]
            measured_distances = {}
            for a in angles_to_check:
                set_servo_angle(a)
                time.sleep_ms(200)  # Longer wait for servo
                measured_distances[a] = measure_distance()

            # Calculate weighted scores for each path
            path_scores = {}
            for a in angles_to_check:
                path_scores[a] = calculate_path_score(a, measured_distances[a])

            # Choose the path with the highest weighted score
            best_angle = max(path_scores, key=path_scores.get)
            best_distance = measured_distances[best_angle]
            
            # Simple corridor check: if going forward, ensure sides aren't too close
            left_clearance = measured_distances[45]
            right_clearance = measured_distances[135]
            min_side_clearance = 30  # Minimum clearance needed on each side
            corridor_too_narrow = (best_angle == 90 and 
                                  (left_clearance < min_side_clearance or 
                                   right_clearance < min_side_clearance))
            
            if best_distance < BASE_SAFE_DISTANCE or corridor_too_narrow:
                # No clear path - increment stuck counter
                stuck_counter += 1
                
                if stuck_counter >= 3:
                    # Stuck detected - perform emergency 180-degree rotation
                    # Rotate in place by spinning wheels in opposite directions
                    IN1.value(1)  # Right motor forward
                    IN2.value(0)
                    IN3.value(0)  # Left motor backward
                    IN4.value(1)
                    set_speeds(600, 600)  # Higher speed for rotation
                    time.sleep_ms(2000)  # Rotate for 2 seconds (~180 degrees)
                    stop()
                    stuck_counter = 0  # Reset counter
                else:
                    # If no path is clear, attempt a larger backward move
                    move_backward()
                    time.sleep_ms(1200)  # Increased from 1000ms for better clearance
                    stop()
            else:
                # Use proportional steering for smoother turns
                left_speed, right_speed = proportional_steer(best_angle, base_speed=500)
                
                # Set motor directions for forward movement
                IN1.value(1)
                IN2.value(0)
                IN3.value(1)
                IN4.value(0)
                
                # Apply calculated speeds for proportional steering
                set_speeds(left_speed, right_speed)
                time.sleep_ms(500)
                stop()
                stuck_counter = 0  # Reset stuck counter on successful turn
        else:
            # Move forward if clear
            move_forward()
            stuck_counter = 0  # Reset stuck counter when moving forward
        
        # Adaptive servo sweep: adjust step size based on proximity
        # When obstacles are close, scan more densely (smaller steps)
        # When path is clear, scan faster (larger steps)
        if current_dist < 50:
            step_size = 2  # Dense scanning when close to obstacles
        elif current_dist < 100:
            step_size = 3  # Normal scanning
        else:
            step_size = 5  # Fast scanning when path is clear
        
        angle += (step_size * direction)
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