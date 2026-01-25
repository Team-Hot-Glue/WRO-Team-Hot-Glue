import numpy as np
import cv2
import time
import serial
import threading
from picamera2 import Picamera2

# Arduino serial communication in background thread
arduino_queue = None
arduino_thread = None

def send_to_arduino(message):
    """Send message to Arduino"""
    try:
        arduino.write(bytes(message, 'utf-8'))
    except:
        pass

# turn on cam
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format":"RGB888", "size": (160, 120)})
picam2.configure(config)
picam2.set_controls({"AwbMode": 1, "AeEnable": 1})  # Auto white balance and exposure
picam2.start()
time.sleep(1)  # Allow camera to warm up


# arduino connection
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1) # CHECK COM PORT!!!!!!!

# Pre-calculate constants
REAL_WIDTH_CM = 5.0
FOCAL_LENGTH_MM = 35.0
SENSOR_WIDTH_MM = 4.0
H_FOV = 60.0  # degrees
AREA_THRESHOLD = 30
KERNAL = np.ones((2, 2), "uint8")  # Minimal kernel for speed
DISPLAY_EVERY_N_FRAMES = 5  # Only display every 5th frame

# Color ranges
red_lower = np.array([0, 80, 100], np.uint8)
red_upper = np.array([10, 255, 255], np.uint8)
red_lower2 = np.array([170, 80, 100], np.uint8)
red_upper2 = np.array([180, 255, 255], np.uint8)

green_lower = np.array([35, 40, 40], np.uint8)
green_upper = np.array([90, 255, 255], np.uint8)

frame_count = 0
fps_start_time = time.time()

while (1):
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

    # Print and send nearest object's color and angle to Arduino
    if objects:
        nearest = min(objects, key=lambda x: x[1])
        center_x = nearest[2]
        frame_center_x = frame_width // 2
        # Calculate angle from center
        angle = ((center_x - frame_center_x) / (frame_width / 2)) * (H_FOV / 2)
        # Deadzone logic
        if abs(angle) <= 10:
            angle_to_send = 0
        else:
            angle_to_send = round(angle, 1)
        print(f"{nearest[0]}, {nearest[1]:.1f} cm, {angle_to_send}")
        # Send to Arduino (non-blocking)
        arduino_message = f"{nearest[0]},{angle_to_send}\n"
        send_to_arduino(arduino_message)

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
        picam2.stop()
        cv2.destroyAllWindows()
        break