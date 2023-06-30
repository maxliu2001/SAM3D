#!/usr/bin/env python
# write depth array to file
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import subprocess
import os
import signal

class DepthGenerator:
    def __init__(self):
        self.count = 0
        self.topic = "/camera/aligned_depth_to_color/image_raw"
        self.resgistry = {}
        self.temp_file = open('/home/maxliu/catkin_ws/src/sam3d/media/depthmap.npz', 'wb')
    
    def img_callback(self, img_msg):
        rospy.loginfo("received depth image")
        bridge = CvBridge()
        depth_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        depth_arr = np.array(depth_img, dtype=np.float)
        self.resgistry[str(self.count)] = depth_arr
        self.count += 1
        rospy.loginfo(depth_arr)

    def run(self):
        rospy.init_node('dlistener', anonymous=True)
        rospy.loginfo("start recording depth channel")
        rospy.Subscriber(self.topic, Image, self.img_callback)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.spin()
    
    def shutdown_hook(self):
        np.savez_compressed(self.temp_file, **self.resgistry)
        self.temp_file.close()

if __name__ == '__main__':
    depth_rec = DepthGenerator()
    depth_rec.run()
    depth_rec.shutdown_hook()
