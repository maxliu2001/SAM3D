#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry, SamPredictor

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import rospkg

class Generator:
    def __init__(self):
        rospy.init_node('generator', anonymous=True)
        rospy.loginfo('registered node')
        sam = sam_model_registry["vit_h"](checkpoint="~/sam_vit_h_4b8939.pth")
        rospy.loginfo('registered model')
        self.predictor = SamPredictor(sam)
        rospy.loginfo('initialized SAM3D')
        self.registry = {}
        self.filepath = rospkg.RosPack().get_path('sam3d') + '/media/test'
        self.temp_file = rospkg.RosPack().get_path('sam3d') + '/media/masks.npz'
        self.input_box = None
    
    def generate_bounding_box(self, li, width):
        height, width = li[0], li[1]
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
            image = plt.imread(path)
            # change color config from BGR to RGB for matplot
            image = image[..., ::-1]
            self.predictor.set_image(image)
            
            # a, b, c, d = self.generate_bounding_box(list(image.shape), 500)
            # input_box = np.array([a, b, c, d])
            # input_point = np.array([[300,300]])
            # input_label = np.array([1])

            # masks, _, _ = self.predictor.predict(
            #     point_coords = input_point,
            #     point_labels = input_label,
            #     box = None,
            #     multimask_output=False,
            # )

            a, b, c, d = self.generate_bounding_box(list(image.shape), 500)
            if not self.input_box:
                self.input_box = np.array([a, b, c, d])

            masks, _, _ = self.predictor.predict(
                point_coords = None,
                point_labels = None,
                box = self.input_box,
                multimask_output=False,
            )

            rospy.on_shutdown(self.shutdown_hook)
            
            # write it in a file
            rospy.loginfo(masks)
            self.registry[path.stem] = masks
    
    def shutdown_hook(self):
        np.savez_compressed(self.temp_file, **self.registry)

if __name__ == '__main__':
    try:
        gen = Generator()
        gen.collect_mask()
        gen.shutdown_hook()
    except rospy.ROSInterruptException:
        pass