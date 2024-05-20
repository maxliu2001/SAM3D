from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
import numpy as np
from .gui import ImageViewer

class SAMSelector(QDialog):
    """ Initialize a SAM3D model and generate segmentation mask given an image """
    def __init__(self, cv_image):
        super().__init__()
        self.setWindowTitle("SAM3D Selector")
        self.cv_image = cv_image
        self.points = []
        self.points_masks = []
        self.box = []
        self.init_ui()

    def init_ui(self):
        self.viewer = ImageViewer(self.cv_image)
        layout = QVBoxLayout()
        layout.addWidget(self.viewer)

        # Debug label to show image shape
        self.debug_label = QLabel(f"Image shape: {self.cv_image.shape}")
        layout.addWidget(self.debug_label)

        self.setLayout(layout)

    def exec_(self):
        super().exec_()
        self.retrieve_data()

    def retrieve_data(self):
        ''' Collect mask and bounding box info after closing GUI. '''
        total_points = np.array(self.viewer.points)

        if len(total_points) > 0:
            self.points = total_points[:, 0:2]
            self.points_masks = total_points[:, 2]
        else:
            self.points = []
            self.points_masks = []

        self.box = np.array(self.viewer.edges_list)

    def select_params(self):
        """ Launches GUI to select points as well as bounding boxes. """
        self.exec_()

        print(f'Selected points \n {self.points}')
        print(f'Selected point masks \n {self.points_masks}')
        print(f'Selected boxes \n {self.box}')
        return
