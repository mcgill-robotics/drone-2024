#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import the necessary packages
import cv2
import numpy as np
from ultralytics import YOLO
from utils.cv_util import find_alphanum, draw_text
import matplotlib.pyplot as plt

plt.ion()

results = {}

# load models
man_shape_model = YOLO('./runs/detect/new_shape_model/weights/best.pt')
letter_detection_model = YOLO('runs/detect/letters_yolo/weights/best.pt')
number_detection_model = YOLO('runs/detect/numbers_yolo/weights/best.pt')

# load video
cap = cv2.VideoCapture(
    "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1 ! nvvidconv ! appsink",
    cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Can't open camera")
    exit()

frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frameRate = int(cap.get(cv2.CAP_PROP_FPS))
print(f"w, h, fr: {frameWidth}, {frameHeight}, {frameRate}")

num_rows = 2
num_columns = 5
# figure, axs = plt.subplots(num_rows, num_columns)
# read frames
while True:
    #Capture frame by frame
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = frame[:, :, 0:3]
    frame_copy = frame.copy()
    if not ret:
        print("Cant receive frame. Exiting")
        break
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
        text = f"{man_shape_detections[0].names[class_id]}, {score}"
        draw_text(frame_copy,
                  text,
                  pos=(x1, y1),
                  font_scale=1,
                  text_color=(255, 255, 255))
        cv2.rectangle(frame_copy, (max(0, int(x1)), max(0, int(y1))),
                      (min(int(x2), W), min(int(y2), H)), (0, 255, 0), 2)

        frame_of_interest = frame[int(y1):int(y2), int(x1):int(x2)]
        if (class_id == 3):
            continue
        letter_detections = letter_detection_model(frame_of_interest)
        num_detects_letter = len(letter_detections[0].boxes.data.tolist())

        number_detections = number_detection_model(frame_of_interest)
        num_detects_nums = len(number_detections[0].boxes.data.tolist())

        # axs[int(index // num_columns)][int(index % num_columns)].imshow(
        #     cv2.cvtColor(frame_of_interest, cv2.COLOR_BGR2RGB))

        # for letters
        for detection in letter_detections[0].boxes.data.tolist():
            find_alphanum(detection, letter_detections, frame_of_interest,
                          frame_copy, x1, x2, y1, y2)

        # for numbers
        for detection in number_detections[0].boxes.data.tolist():
            find_alphanum(detection, number_detections, frame_of_interest,
                          frame_copy, x1, x2, y1, y2)

    # for index in range(last_index + 1, num_rows * num_columns):
    #     axs[int(index // num_columns)][int(index % num_columns)].cla()
    # plt.pause(0.0001)

    frame_copy = cv2.resize(frame_copy, (1280, 720))
    cv2.imshow("Camera video", frame_copy)

    if cv2.waitKey(1) == ord('c'):  # if press c the video capturing will end
        break
