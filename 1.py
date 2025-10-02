import numpy as np
import cv2
import time
import serial

# turn on cam
webcam = cv2.VideoCapture(1)
# Set resolution to 720p (1280x720)
webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# arduino connection
#arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1) # CHECK COM PORT!!!!!!!
#def write_read(x): 
#    arduino.write(bytes(x, 'utf-8')) 
#    time.sleep(0.05) 
#    data = arduino.readline() 
#    return data 



while (1):
    _, imageFrame = webcam.read()

    # Convert BGR to HSV colorspace
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for red color 
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Set range for green color
    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)


    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")

    # red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)

    # green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)



    # Find contours and distances for red and green objects
    objects = []  # list of (color, distance_cm, center_x)
    frame_height, frame_width = imageFrame.shape[:2]

    # Red objects
    contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_red:
        area = cv2.contourArea(contour)
        if area > 300:
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(imageFrame, "RED", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
            REAL_WIDTH_CM = 5.0
            FOCAL_LENGTH_MM = 35.0
            SENSOR_WIDTH_MM = 4.0
            focal_length_px = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * frame_width
            if w > 0:
                distance_cm = (REAL_WIDTH_CM * focal_length_px) / w
                cv2.putText(imageFrame, f"{distance_cm/10:.1f}cm", (x, y + h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                center_x = x + w // 2
                objects.append(("RED", distance_cm/10, center_x))

    # Green objects
    contours_green, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours_green:
        area = cv2.contourArea(contour)
        if area > 300:
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(imageFrame, "GREEN", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
            REAL_WIDTH_CM = 5.0
            FOCAL_LENGTH_MM = 35.0
            SENSOR_WIDTH_MM = 4.0
            focal_length_px = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * frame_width
            if w > 0:
                distance_cm = (REAL_WIDTH_CM * focal_length_px) / w
                cv2.putText(imageFrame, f"{distance_cm/10:.1f}cm", (x, y + h + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                center_x = x + w // 2
                objects.append(("GREEN", distance_cm/10, center_x))

    # Print nearest object's angle from center (degrees), with 10 degree deadzone
    if objects:
        nearest = min(objects, key=lambda x: x[1])
        center_x = nearest[2]
        frame_center_x = frame_width // 2
        # Typical webcam horizontal FOV
        H_FOV = 60.0  # degrees
        # Calculate angle from center
        angle = ((center_x - frame_center_x) / (frame_width / 2)) * (H_FOV / 2)
        # Deadzone logic
        if abs(angle) <= 10:
            pos = "CENTER"
        elif angle < -10:
            pos = f"{angle:.1f}°"
        else:
            pos = f"{angle:.1f}°"
        print(f"{nearest[0]}, {nearest[1]:.1f} cm, {pos}")


    # final run
    cv2.imshow("Color Detection", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break