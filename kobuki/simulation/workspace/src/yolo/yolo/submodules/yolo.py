import os

import cv2 as cv
import numpy as np
import torch

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO as _YOLO


class Yolo():

    def __init__(self, yolo_model : str, classes : list = None):
        self.gpu = torch.cuda.is_available()
        if self.gpu:
            path = os.path.join(get_package_share_directory('yolo'), 'config', yolo_model + '.engine')
        else:
            path = os.path.join(get_package_share_directory('yolo'), 'config', yolo_model + '.pt')
        print(f'model path: {path}')
        self.model = _YOLO(path)
        self.classes = classes if classes != None else [0]

    def run(self, tensor : torch.Tensor):
        return self.model.predict(source=tensor, classes=self.classes, save=False, conf=0.5)[0]

    def merge_masks(self, masks):
        if not masks:
            return False, None
        main_mask = masks[0].data[0].cpu().numpy() * np.uint8(255)
        for mask in masks:
            cv_mask = mask.data[0].cpu().numpy() * np.uint8(255)
            main_mask = cv.bitwise_or(main_mask, cv_mask)
        return True, main_mask
    
    def get_boxes(self, yolo_boxes):
        return [int(x) for box in yolo_boxes for x in box.xyxy.tolist()[0]]
