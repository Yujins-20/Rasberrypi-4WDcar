import cv2
import numpy as np
import math
import time
import torch
import RPi.GPIO as GPIO
from ultralytics import YOLO
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import CircularOutput

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Motor pin definitions
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13



# Steering
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
steering = GPIO.PWM(ENA, 1000)
steering.stop()

# Throttle
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.output(IN3, GPIO.HIGH)
GPIO.output(IN4, GPIO.LOW)
throttle = GPIO.PWM(ENB, 1000)
throttle.stop()

def run(speed=20):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    steering.ChangeDutyCycle(speed)
    throttle.ChangeDutyCycle(speed)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    steering.ChangeDutyCycle(0)
    throttle.ChangeDutyCycle(0)

def left(speed=20):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    steering.ChangeDutyCycle(0)
    throttle.ChangeDutyCycle(speed)

def right(speed=20):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    steering.ChangeDutyCycle(speed)
    throttle.ChangeDutyCycle(0)

def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([90, 120, 0], dtype="uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    edges = cv2.Canny(mask, 50, 100)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10  
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=150)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        print("no line segments detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
                continue
            
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  
    y2 = int(y1 / 2)  
    if slope == 0:
        slope = 0.1
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 0, 0), line_width=6):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 0), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    return heading_image

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90
    return steering_angle

    
model = YOLO("yolov8s-world.pt")
model.set_classes(["person", "bus"])    
    
augment = False
visualize = False
conf_threshold = 0.25
nms_iou_thres = 0.45
max_det = 1000

# Initialize camera
cap = cv2.VideoCapture(0)  # Use the first USB camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Allow the camera to warm up
time.sleep(0.1)


# Video saving setup
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 8, (320, 240))

time.sleep(1)

speed = 8
lastTime = 0
lastError = 0

kp = 0.4
kd = kp * 0.65
move_distance = 20

while True:  # 무한 루프로 변경하여 모든 프레임을 처리하도록 수정
    ret, frame = cap.read()
    if not ret:  # 프레임 읽기 실패 시 루프 종료
        break
    
    frame = cv2.flip(frame, -1)

    try:
        resized_frame = cv2.resize(frame, (640, 640))
    except cv2.error as e:
        print(f"Error resizing frame: {e}")
        break

    im = torch.from_numpy(resized_frame).permute(2, 0, 1).float()
    im /= 255
    im = im.unsqueeze(0)

    results = model.predict(source=im, stream=True, imgsz=512)
    names = model.names

    detected_boxes = []
    for r in results:
        if r.keypoints is not None:
            for c in r.boxes.cls:
                print(names[int(c)])
            for box in r.boxes.xyxy:
                detected_boxes.append(box)

            if detected_boxes:
                sum_x = 0
                sum_y = 0
                for box in detected_boxes:
                    x1, y1, x2, y2 = box[:4]
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    sum_x += center_x
                    sum_y += center_y
                avg_center_x = sum_x / len(detected_boxes)
                avg_center_y = sum_y / len(detected_boxes)
                if avg_center_x < 320:
                    print("Move left")
                    left()
                else:
                    print("Move right")
                    right()
                move_time = int((detected_boxes[0][2] - detected_boxes[0][0]) / 640 * move_distance)
                time.sleep(move_time / 1000.0)
                stop()
                for box in detected_boxes:
                    x1, y1, x2, y2 = box[:4]
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
       
        else:
            speed = 4
            lastTime = 0
            lastError = 0
            
            kp = 0.4
            kd = kp * 0.65
            frame = cv2.flip(frame, -1)
            edges = detect_edges(frame)
            roi = region_of_interest(edges)
            line_segments = detect_line_segments(roi)
            lane_lines = average_slope_intercept(frame, line_segments)
            lane_lines_image = display_lines(frame, lane_lines)
            steering_angle = get_steering_angle(frame, lane_lines)
            heading_image = display_heading_line(lane_lines_image, steering_angle)
            now = time.time()
            dt = now - lastTime
            deviation = steering_angle - 90
            error = abs(deviation)
            if deviation < 5 and deviation > -5:
                deviation = 0
                error = 0
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.LOW)
                steering.stop()
            elif deviation > 5:
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)
                steering.start(100)
            elif deviation < -5:
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
                steering.start(100)
            derivative = kd * (error - lastError) / dt
            proportional = kp * error
            PD = int(speed + derivative + proportional)
            spd = abs(PD)
            if spd > 10:
                spd = 10
            throttle.start(spd)
            lastError = error
            lastTime = time.time()

    out.write(frame)
cap.release()
out.release()