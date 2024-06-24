#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import the necessary packages
import cv2
import numpy as np
from ultralytics import YOLO
from .utils.cv_util import read_color_on_shape
import matplotlib.pyplot as plt

plt.ion()

results = {}

# load models
man_shape_model = YOLO('runs/detect/train6/weights/best.pt')
letter_detection_model = YOLO('runs/detect/train7/weights/best.pt')
number_detection_model = YOLO('runs/detect/train/weights/best.pt')

# load video
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Can't open camera")
    exit()

width = int(640 * 3.6)
height = int(480 * 3.6)
fr = 5
ok = cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
if (not ok):
    print("Could not change image width")
    exit()
ok = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
if (not ok):
    print("Could not change image height")
    exit()
#
ok = cap.set(cv2.CAP_PROP_FPS, fr)
if (not ok):
    print("Could not change camera frame rate")
    exit()

frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frameRate = int(cap.get(cv2.CAP_PROP_FPS))
print(f"w, h, fr: {frameWidth}, {frameHeight}, {frameRate}")

fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('output.avi', fourcc, frameRate,
                      (frameWidth, frameHeight))
num_rows = 2
num_columns = 5
figure, axs = plt.subplots(num_rows, num_columns)
# read frames
while True:
    #Capture frame by frame
    ret, frame = cap.read()
    if not ret:
        print("Cant receive frame. Exiting")
        break
    frame_copy = frame.copy()
    H, W, _ = frame.shape
    # if frame is read correctly ret is True

    # detect shapes and mannequin
    man_shape_detections = man_shape_model(frame)
    num_detects = len(man_shape_detections[0].boxes.data.tolist())
    # plt.imshow(frame_of_interest)
    # plt.pause(0.0001)
    last_index = 0
    for index, detection in enumerate(
            man_shape_detections[0].boxes.data.tolist()):
        last_index = index
        x1, y1, x2, y2, score, class_id = detection
        print(
            f"Detected {man_shape_detections[0].names[class_id]}, at x: {(x1 + x2) / 2}, y: {(y1 + y2) / 2}"
        )
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame_copy,
                    f"{man_shape_detections[0].names[class_id]}, {score}",
                    (int(x1), int(y1)), font, 4, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.rectangle(frame_copy, (max(0, int(x1)), max(0, int(y1))),
                      (min(int(x2), W), min(int(y2), H)), (0, 255, 0), 2)

        frame_of_interest = frame[int(y1):int(y2), int(x1):int(x2)].copy()
        if (class_id == 3):
            continue
        letter_detections = letter_detection_model(frame_of_interest)
        num_detects_letter = len(letter_detections[0].boxes.data.tolist())

        number_detections = number_detection_model(frame_of_interest)
        num_detects_nums = len(number_detections[0].boxes.data.tolist())

        axs[int(index // num_columns)][int(index % num_columns)].imshow(
            cv2.cvtColor(frame_of_interest, cv2.COLOR_BGR2RGB))

        #for letters
        find_letters_numbers(letter_detections, frame_of_interest, 
            frame_copy, x1, x2, y1, y2)

        #for numbers
        find_letters_numbers(number_detections, frame_of_interest, 
            frame_copy, x1, x2, y1, y2)
    
    for index in range(last_index + 1, num_rows * num_columns):
        axs[int(index // num_columns)][int(index % num_columns)].cla()
    plt.pause(0.0001)

    cv2.imshow("Camera video", frame_copy)
    out.write(frame_copy)

    if cv2.waitKey(1) == ord('c'):  # if press c the video capturing will end
        break
