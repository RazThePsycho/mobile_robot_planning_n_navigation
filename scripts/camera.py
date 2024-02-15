#!/usr/bin/env python3
import rospkg
import rospy

import numpy as np
import cv2

import torch
import json

# Constants.
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
SCORE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.45
CONFIDENCE_THRESHOLD = 0.45

# Text parameters.
FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.25
THICKNESS = 1

# Colors.
BLACK = (0, 0, 0)
BLUE = (255, 178, 50)
YELLOW = (0, 255, 255)
PURPLE = (128, 0, 128)

rospack = rospkg.RosPack()

class Detector:
    def __init__(self) -> None:
        model_name = rospy.get_param('~yolo_model_path', 'best2.pt')
        bachelor_pkg_path = rospack.get_path('bachelor')
        model_path = bachelor_pkg_path + '/configs/YOLOv5/' + model_name
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', model_path)

    def yolo(self, input_image):
        results = self.model(input_image)
        return json.loads(results.pandas().xyxy[0].to_json(orient="records"))

    def draw_label(self, input_image, res: dict):
        text_size = cv2.getTextSize(res['name'], FONT_FACE, FONT_SCALE, THICKNESS)
        dim, baseline = text_size[0], text_size[1]
        cv2.rectangle(input_image, (int(res['xmin']), int(res['ymin'])), 
                                   (int(res['xmax']), int(res['ymax'])), PURPLE, 2*THICKNESS)
        # cv2.rectangle(input_image, (int(res['xmin']), int(res['ymin'])), 
        #                            (int(res['xmin'] + dim[0]), int(res['ymin'] + dim[1] + baseline)),
        #                            (0, 0, 0), cv2.FILLED)
        cv2.putText(input_image, f"{res['name']} {res['confidence']:.3f}", 
                    (int(res['xmin']), int(res['ymin']) + dim[1]), FONT_FACE,
                    FONT_SCALE, YELLOW, THICKNESS, cv2.LINE_AA)

    def draw_labels(self, input_image, results: list):
        for r in results:
            self.draw_label(input_image, r)

if __name__ == '__main__':
    detector = Detector()

    img = cv2.imread('/ros_ws/src/bachelor/yolodata/frame0001.jpg')
    results = detector.yolo(img)
    print(results)
    for res in results:
        detector.draw_label(img, res)

    cv2.imshow('Output', img)
    cv2.waitKey(0)
