import numpy as np
from segment_anything import *
from .gui import ImageViewer
import sys
from PyQt5.QtWidgets import QApplication
from .sam3d_error import SAM3DError
import cv2

class SAMSelector:
    """ Initialze a SAM3D model and generates segmentation mask given an image"""
    def __init__(self):
        self.config_saved = False
        self.ex = None
        self.points = []
        self.points_masks = []
        self.box = []

    def launch_gui(self, filepath):
        ''' Launch GUI and collect mask and bounding box. '''
        app = QApplication(sys.argv)
        self.ex = ImageViewer(filepath)
        app.exec_()
        self.retrieve_data()

    def retrieve_data(self):
        ''' Collect mask and bounding box info after closing GUI. '''
        total_points = np.array(self.ex.points)

        if len(total_points) > 0:
            self.points = total_points[:,0:2]
            self.points_masks = total_points[:,2]
        else:
            self.points = []
            self.points_masks = []

        self.box = np.array(self.ex.edges_list)
        self.config_saved = True

    def clear_config(self):
        ''' Clear existing point/bounding box configuration. '''
        if self.config_saved:
            self.points = []
            self.box = []
            self.points_masks = []
            self.ex = None
            self.config_saved = False

    def gen_seg(self, image_path, manual = True, multimask = True, logit = False, mask_input = None, input_point = None, input_label = None, boxes = None):
        """ Optionally launches GUI to select points as well as bounding boxes. """
        try:
            image = cv2.imread(image_path)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        except:
            raise SAM3DError('Image can\'t be loaded. Check your path')

        if manual:
            self.clear_config()
            self.launch_gui(image_path)
            input_point = self.points
            input_label = self.points_masks
            boxes = self.box

        return
