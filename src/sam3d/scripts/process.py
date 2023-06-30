#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry, SamPredictor

import numpy as np
import matplotlib.pyplot as plt
import cv2
from pathlib import Path

class Generator:
    def __init__(self):
        rospy.init_node('generator', anonymous=True)
        rospy.loginfo('registered node')
        sam = sam_model_registry["vit_h"](checkpoint="/home/maxliu/sam_vit_h_4b8939.pth")
        rospy.loginfo('registered model')
        self.predictor = SamPredictor(sam)
        rospy.loginfo('initialized SAM3D')
        self.registry = {}
        self.filepath = '/home/maxliu/catkin_ws/src/sam3d/media/test'
        self.temp_file = open('/home/maxliu/catkin_ws/src/sam3d/media/masks.npz', 'wb')
    
    def generate_bounding_box(dim, width):
        height, width, _ = dim
        b = 0 + (height - width)//2
        d = b + width
        a = 0 + (width - width)//2
        c = a + width
        return a, b, c, d

    def collect_mask(self):
        rospy.loginfo('start collecting masks')
        pathli=Path(self.filepath).rglob('*.jpg')
        rospy.loginfo(pathli)
        for path in pathli:
            rospy.loginfo(path)
            img = cv2.imread('path')
            image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.predictor.set_image(image)
            a, b, c, d = self.generate_bounding_box(image.shape, 500)
            input_box = np.array([a, b, c, d])

            masks, _, _ = self.predictor.predict(
                point_coords=None,
                point_labels=None,
                box = input_box,
                multimask_output=False,
            )

            rospy.on_shutdown(self.shutdown_hook)
            # write it in a file
            rospy.loginfo(masks)
            self.registry[path.stem] = masks
    
    def shutdown_hook(self):
        np.savez_compressed(self.temp_file, **self.registry)
        self.temp_file.close()

if __name__ == '__main__':
    try:
        gen = Generator()
        gen.collect_mask()
        gen.shutdown_hook()
    except rospy.ROSInterruptException:
          pass