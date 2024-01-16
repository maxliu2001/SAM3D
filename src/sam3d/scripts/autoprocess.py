#!/usr/bin/env python
from sam3d_gui import SAMSelector
from ros_sam import SAMClient, show_mask, show_points
import rospkg
import cv2
import numpy as np
import matplotlib.pyplot as plt

class MaskGen:
    ''' Use the ros_sam node utilizing the GUI selector'''
    def __init__(self):
        print("Loading ros_sam service")
        self.sam_client = SAMClient('ros_sam')
        print("Finished loading ros_sam service")
    
    def select_img(self, img_path):
        # Initalize SAM Selector
        print('Initializing Selector')
        selector = SAMSelector()
        print('Selector Initialized.')

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('sam3d')
        image_path = package_path + img_path
        selector.gen_seg(image_path)

        print(f'Selected points are: \n{selector.points}')
        print(f'Selected points masks are: \n{selector.points_masks}')
        print(f'Selected boxes are: \n{selector.box}')

        img = cv2.imread(image_path)
        masks, scores = self.sam_client.segment(img, selector.points, selector.points_masks)

        # visualizing masks
        show_mask(masks[0], plt.gca())
        show_points(selector.points, np.asarray(selector.points_masks), plt.gca())

if __name__ == "__main__":
    task = MaskGen()
    task.select_img('/media/testimage155.jpg')

