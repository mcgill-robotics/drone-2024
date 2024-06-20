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
letter_detection_model = YOLO('runs/detect/train2/weights/best.pt')

# load video
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Can't open camera")
    exit
  
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

cv2.imshow('Video', cap) #this stupid line wouldnt run on my laptop so i hope it works for u


# read frames
while True:
    #Capture frame by frame
    ret, frame = cap.read()
    H, W, _ = frame.shape

    # if frame is read correctly ret is True
    if not ret:
        print("Cant receive frame. Exiting")
        break

    # detect shapes and mannequin
    man_shape_detections = man_shape_model(frame)
    print(man_shape_detections)
    detections = []
    for detection in man_shape_detections.boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = detection
        detections.append([x1, y1, x2, y2, score, class_id])

    # track shapes and mannequin
    track_ids = mot_tracker.update(np.asarray(detections))

    # detect letter
    letter_detections = letter_detection_model(frame)
    for detection in letter_detections.boxes.data.tolist():
        x1, y1, x2, y2, score, letter_id = detection

        # assign letter to shape and assign mannequin
        xshape1, yshape1, xshape2, yshape2, shape_id = get_shape(detection, track_ids)

        if shape_id == 0:
            results[shape_id] = {'shape': {'bbox':[xshape1, yshape1, xshape2, yshape2], 'color': None}, 'letter': {'bbox':[0, 0, 0, 0], 'text':None, 'color':None, 'bbox_score':0, 'text_score':0}}
        elif shape_id != -1:
            # crop to see the letter
            letter_crop = frame[int(y1):int(y2), int(x1): int(x2), :]

            # read letters and colors
            letter_text, letter_text_score, shape_color, letter_color = read_letter_and_color_on_shape(letter_crop)

            if letter_text is not None:
                results[shape_id] = {'shape': {'bbox':[xshape1, yshape1, xshape2, yshape2], 'color':shape_color}, 'letter': {'bbox':[x1, y1, x2, y2], 'text':letter_text, 'color':letter_color, 'bbox_score':score, 'text_score':letter_text_score}}
    
 # write results including getting the x and y values of position
write_csv(results, './test.csv')   #currently all results go to a csv file


# THIS CURRENTLY NEVER ENDS UNLESS U FORCEFULLY END IT
