import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# ==================== GPIO PIN DEFINITIONS ====================
# Servo Motor Pin (Steering)
SERVO_PIN = 17  # BCM GPIO 17

# DC Motor Pins (L298N Driver - Movement)
DC_MOTOR_IN1 = 27  # Motor direction control 1
DC_MOTOR_IN2 = 22  # Motor direction control 2
DC_MOTOR_EN = 23   # Motor PWM speed control (Enable)

# ==================== GPIO SETUP ====================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup servo pin
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz frequency for servo
servo_pwm.start(0)

# Setup DC motor pins
GPIO.setup(DC_MOTOR_IN1, GPIO.OUT)
GPIO.setup(DC_MOTOR_IN2, GPIO.OUT)
GPIO.setup(DC_MOTOR_EN, GPIO.OUT)
motor_pwm = GPIO.PWM(DC_MOTOR_EN, 1000)  # 1kHz frequency for DC motor
motor_pwm.start(0)

# ==================== MOTOR CONTROL FUNCTIONS ====================
def set_servo_angle(angle):
    """Set servo angle (0-180 degrees) for steering"""
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    duty_cycle = (angle / 18) + 2.5
    servo_pwm.ChangeDutyCycle(duty_cycle)

def set_dc_motor_speed(speed):
    """Set DC motor speed (-100 to 100)"""
    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
    
    if speed > 0:
        # Forward direction
        GPIO.output(DC_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(DC_MOTOR_IN2, GPIO.LOW)
        motor_pwm.ChangeDutyCycle(speed)
    elif speed < 0:
        # Backward direction
        GPIO.output(DC_MOTOR_IN1, GPIO.LOW)
        GPIO.output(DC_MOTOR_IN2, GPIO.HIGH)
        motor_pwm.ChangeDutyCycle(abs(speed))
    else:
        # Stop
        GPIO.output(DC_MOTOR_IN1, GPIO.LOW)
        GPIO.output(DC_MOTOR_IN2, GPIO.LOW)
        motor_pwm.ChangeDutyCycle(0)

# turn on cam
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format":"RGB888", "size": (160, 120)})
picam2.configure(config)
picam2.set_controls({"AwbMode": 1, "AeEnable": 1})  # Auto white balance and exposure
picam2.start()
# time.sleep(1)  # Allow camera to warm up

# Pre-calculate constants
REAL_WIDTH_CM = 5.0
FOCAL_LENGTH_MM = 35.0
SENSOR_WIDTH_MM = 4.0
H_FOV = 60.0  # degrees
AREA_THRESHOLD = 30
KERNAL = np.ones((2, 2), "uint8")  # Minimal kernel for speed
DISPLAY_EVERY_N_FRAMES = 5  # Only display every 5th frame

# Color ranges (optimized for precision)
red_lower = np.array([100, 80, 100], np.uint8)
red_upper = np.array([130, 255, 255], np.uint8)
red_lower2 = np.array([200, 0, 0], np.uint8)
red_upper2 = np.array([210, 0, 0], np.uint8)

green_lower = np.array([40, 40, 45], np.uint8)
green_upper = np.array([80, 255, 255], np.uint8)

# ==================== STATE VARIABLES ====================
STATE_SEARCHING = 0
STATE_TURNING = 1
STATE_DRIVING = 2
STATE_CIRCLING = 3

current_state = STATE_SEARCHING
target_angle = 90  # Servo angle (0-180, 90 is center)
drive_speed = 0
circling_start_time = time.time()  # Initialize with current time
circle_duration = 3.0  # Seconds to circle around object

def get_servo_angle_from_image_angle(angle):
    """Convert image angle (-30 to +30) to servo angle (60 to 120)"""
    # angle range: -30 to +30
    # servo range: 60 to 120
    # 0 image angle = 90 servo angle
    servo_angle = 90 + angle * 1.5  # Scale factor to map image angle to servo angle
    servo_angle = max(60, min(120, servo_angle))  # Clamp between 60-120
    return servo_angle

frame_count = 0
fps_start_time = time.time()

try:
    while True:
        imageFrame = picam2.capture_array()
        frame_count += 1
        
        if imageFrame is None:
            print("Failed to read frame from camera")
            break

        # Convert RGB to HSV colorspace
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2HSV)

        # Set range for red color 
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        
        # Also catch red colors that wrap around the hue spectrum
        red_mask2 = cv2.inRange(hsvFrame, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask, red_mask2)

        # Set range for green color
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

        # to detect only that particular color
        red_mask = cv2.dilate(red_mask, KERNAL, iterations=1)
        green_mask = cv2.dilate(green_mask, KERNAL, iterations=1)

        # Find contours and distances for red and green objects
        objects = []  # list of (color, distance_cm, center_x)
        frame_height, frame_width = imageFrame.shape[:2]
        focal_length_px = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * frame_width

        # Red objects
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > AREA_THRESHOLD:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 0:
                    distance_cm = (REAL_WIDTH_CM * focal_length_px) / w
                    center_x = x + w // 2
                    objects.append(("RED", distance_cm/10, center_x))
                    if frame_count % DISPLAY_EVERY_N_FRAMES == 0:
                        cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 1)

        # Green objects
        contours_green, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > AREA_THRESHOLD:
                x, y, w, h = cv2.boundingRect(contour)
                if w > 0:
                    distance_cm = (REAL_WIDTH_CM * focal_length_px) / w
                    center_x = x + w // 2
                    objects.append(("GREEN", distance_cm/10, center_x))
                    if frame_count % DISPLAY_EVERY_N_FRAMES == 0:
                        cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 1)

        # ==================== STATE MACHINE ====================
        try:
            if not objects:
                raise ValueError("No objects detected")
                
            nearest = min(objects, key=lambda x: x[1])
            color, distance, center_x = nearest
            frame_center_x = frame_width // 2
            
            # Calculate angle from center (-30 to +30 degrees)
            angle = ((center_x - frame_center_x) / (frame_width / 2)) * (H_FOV / 2)
            
            # Deadzone logic
            if abs(angle) <= 10:
                angle = 0
            
            state_names = ['SEARCHING', 'TURNING', 'DRIVING', 'CIRCLING']
            state_str = state_names[current_state] if current_state < len(state_names) else "UNKNOWN"
            print(f"{color}, Distance: {distance:.1f} cm, Angle: {angle:.1f}°, State: {state_str}")
        
        # STATE MACHINE
            if current_state == STATE_SEARCHING:
                # Searching for object - found one, start turning
                current_state = STATE_TURNING
                set_dc_motor_speed(0)  # Stop driving
            
            elif current_state == STATE_TURNING:
                # Calculate servo angle for steering
                target_angle = get_servo_angle_from_image_angle(angle)
                set_servo_angle(target_angle)
                
                # If object is aligned (angle close to 0), start driving
                if abs(angle) < 5:
                    current_state = STATE_DRIVING
                    print("Object aligned! Starting to drive...")
            
            elif current_state == STATE_DRIVING:
                # Drive towards object
                set_servo_angle(90)  # Keep servo centered while driving forward
                
                # Drive until object is very close (< 20cm)
                if distance > 20:
                    set_dc_motor_speed(60)  # 60% forward speed
                else:
                    current_state = STATE_CIRCLING
                    circling_start_time = time.time()
                    print("Close enough! Starting to circle...")
            
            elif current_state == STATE_CIRCLING:
                # Circle around the object
                elapsed = time.time() - circling_start_time
                
                if elapsed < circle_duration:
                    # Circle by steering and moving forward
                    circle_angle = (elapsed / circle_duration) * 360  # 0 to 360 degrees
                    steering_angle = 90 + 30 * np.sin(np.radians(circle_angle))  # Oscillate steering
                    steering_angle = max(60, min(120, steering_angle))
                    
                    set_servo_angle(steering_angle)
                    set_dc_motor_speed(50)  # 50% speed while circling
                else:
                    # Circling complete
                    set_dc_motor_speed(0)
                    set_servo_angle(90)
                    print("Circling complete!")
                    current_state = STATE_SEARCHING
        except (IndexError, ValueError) as e:
            print(f"Error in state machine: {e}")
            set_dc_motor_speed(0)
            set_servo_angle(90)
            current_state = STATE_SEARCHING
        
        if not objects:
            # No object detected
            if current_state != STATE_SEARCHING:
                print("Object lost!")
                set_dc_motor_speed(0)
                set_servo_angle(90)
                current_state = STATE_SEARCHING

        # FPS counter
        if frame_count % 60 == 0:
            elapsed = time.time() - fps_start_time
            fps = 60 / elapsed if elapsed > 0 else 0
            print(f"FPS: {fps:.1f}")
            fps_start_time = time.time()

        # Display only every Nth frame to reduce overhead
        if frame_count % DISPLAY_EVERY_N_FRAMES == 0:
            cv2.imshow("Color Detection", imageFrame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    # Cleanup
    set_dc_motor_speed(0)
    set_servo_angle(90)
    servo_pwm.stop()
    motor_pwm.stop()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()
    print("Cleanup complete")
