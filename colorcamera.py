import cv2
import numpy as np
import time
import serial
import RPi.GPIO as GPIO
import threading

#UART communication from RPi4 to Piico
ser = serial.Serial(
    '/dev/serial0', 
    baudrate=115200, 
    timeout=1
)
ser.flush()

# ultrasonic sensors
TRIG1 = 23
ECHO1 = 24
TRIG2 = 27
ECHO2 = 17

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False) 
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)

def measure_distance(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.0001)  
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    pulse_start = time.time()
    pulse_end = time.time()

    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    if pulse_duration > 0:
        distance = pulse_duration * 17150
        return round(distance, 2)
    else:
        return None
# camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Set width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Set height

# Global variable and lock for color message
current_color_message = "No color detected"
color_message_lock = threading.Lock()

def process_frame():
    global current_color_message
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            continue
          
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # color ranges in HSV
        lower_red1 = np.array([0, 100, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 70])
        upper_red2 = np.array([179, 255, 255])
        lower_magenta = np.array([150, 150, 70])
        upper_magenta = np.array([160, 255, 255])
        lower_green = np.array([36, 25, 25])
        upper_green = np.array([86, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.add(mask_red1, mask_red2)
        mask_magenta = cv2.inRange(hsv, lower_magenta, upper_magenta)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # detect color obstacles
        message_red = detect_color_obstacle(mask_red, frame, "Red", (0, 0, 255))
        message_green = detect_color_obstacle(mask_green, frame, "Green", (0, 255, 0))
        message_magenta = detect_color_obstacle(mask_magenta, frame, "Magenta", (255, 0, 255))

        with color_message_lock:
            if message_red:
                current_color_message = message_red
                ser.write("TURN_RIGHT\n".encode('utf-8'))
            elif message_green:
                current_color_message = message_green
                ser.write("TURN_LEFT\n".encode('utf-8'))
            elif message_magenta:
                current_color_message = message_magenta
            else:
                current_color_message = "No color detected"

        # Display the frame if needed
        frame = cv2.flip(frame, -1)  # Flip if necessary
        cv2.imshow('Filtered Camera Feed', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def detect_color_obstacle(mask, frame, color_name, color_bgr):
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter out small contours
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
            cv2.putText(frame, f'{color_name} ({w}x{h})', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
            return f"{color_name} obstacle detected"
    return None

try:
    # frame processing
    threading.Thread(target=process_frame, daemon=True).start()

    while True:
        # Measure distances
        distance1 = measure_distance(TRIG1, ECHO1)
        distance2 = measure_distance(TRIG2, ECHO2)

        print(f"Distance 1: {distance1} cm" if distance1 else "Distance 1: Error")
        print(f"Distance 2: {distance2} cm" if distance2 else "Distance 2: Error")
        
        with color_message_lock:
            print(f"Color detected: {current_color_message}")

        ser.write(f"Distance 1: {distance1} cm\n".encode('utf-8'))
        ser.write(f"Distance 2: {distance2} cm\n".encode('utf-8'))
        ser.write(f"Color detected: {current_color_message}\n".encode('utf-8'))

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated by user")

finally:
    cap.release()
    GPIO.cleanup()
    ser.close()
    cv2.destroyAllWindows()
