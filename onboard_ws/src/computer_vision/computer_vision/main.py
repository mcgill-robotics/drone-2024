#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import the necessary packages
import cv2
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
from sort.sort import *
from ultralytics import YOLO
from util import get_shape, read_letter_and_color_on_shape, write_csv

results = {}

mot_tracker = Sort()

# load models
man_shape_model = YOLO('runs/detect/train1/weights/best.pt')
letter_detection_model = YOLO('runs/detect/train10/weights/best.pt')

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
    detections = []
    for detection in man_shape_detections[0].boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = detection
        print(
            f"Detected {man_shape_detections[0].names[class_id]}, at x: {(x1 + x2) / 2}, y: {(y1 + y2) / 2}"
        )
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame_copy,
                    f"{man_shape_detections[0].names[class_id]}, {score}",
                    (int(x1), int(y1)), font, 4, (255, 255, 255), 2,
                    cv2.LINE_AA)
        cv2.rectangle(frame_copy, (max(0, int(x1)), max(0, int(y1))),
                      (min(int(x2), W), min(int(y2), H)), (0, 255, 0), 2)
        detections.append([x1, y1, x2, y2, score, class_id])

    # track shapes and mannequin
    # track_ids = mot_tracker.update(np.asarray(detections))

    # detect letter
    # letter_detections = letter_detection_model(frame)
    # for detection in letter_detections[0].boxes.data.tolist():
    #     x1, y1, x2, y2, score, letter_id = detection
    #
    #     # assign letter to shape and assign mannequin
    #     xshape1, yshape1, xshape2, yshape2, shape_id = get_shape(
    #         detection, track_ids)
    #
    #     if shape_id == 0:
    #         results[shape_id] = {
    #             'shape': {
    #                 'bbox': [xshape1, yshape1, xshape2, yshape2],
    #                 'color': None
    #             },
    #             'letter': {
    #                 'bbox': [0, 0, 0, 0],
    #                 'text': None,
    #                 'color': None,
    #                 'bbox_score': 0,
    #                 'text_score': 0
    #             }
    #         }
    #     elif shape_id != -1:
    #         # crop to see the letter
    #         letter_crop = frame[int(y1):int(y2), int(x1):int(x2), :]
    #
    #         # read letters and colors
    #         letter_text, letter_text_score, shape_color, letter_color = read_letter_and_color_on_shape(
    #             letter_crop)
    #
    #         if letter_text is not None:
    #             results[shape_id] = {
    #                 'shape': {
    #                     'bbox': [xshape1, yshape1, xshape2, yshape2],
    #                     'color': shape_color
    #                 },
    #                 'letter': {
    #                     'bbox': [x1, y1, x2, y2],
    #                     'text': letter_text,
    #                     'color': letter_color,
    #                     'bbox_score': score,
    #                     'text_score': letter_text_score
    #                 }
    #             }
    cv2.imshow("Camera video", frame_copy)
    out.write(frame_copy)

    if cv2.waitKey(1) == ord('c'):  # if press c the video capturing will end
        break

# write results including getting the x and y values of position
write_csv(results, './test.csv')  #currently all results go to a csv file

# THIS CURRENTLY NEVER ENDS UNLESS U FORCEFULLY END IT
