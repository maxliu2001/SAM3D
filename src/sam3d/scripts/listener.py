#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import signal

class MaskGenerator:
    def __init__(self):
        self.count = 0
        self.bag_proc = None
        self.bagged_file_path = "/home/maxliu/catkin_ws/src/sam3d/media/test2.bag"
        self.topic = '/device_0/sensor_1/Color_0/image/data'
        self.temp_dir = '/home/maxliu/catkin_ws/src/sam3d/media/'

    def img_callback(self, img_msg):
        rospy.loginfo("reveived image")
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imwrite(self.temp_dir+'image{}.png'.format(self.count), cv_img)
        self.count += 1

    def start_listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True)
        # os.mkdir(self.temp_dir)
        rospy.loginfo("start subscribing")
        rospy.Subscriber(self.topic, Image, self.img_callback)
    

    def play_from_bag(self):
        if not self.bag_proc:
            arg_li = ["rosbag", "play", self.bagged_file_path]
            self.bag_proc = subprocess.Popen(arg_li)
            rospy.loginfo("rosbag playing")
    
    def run(self):
        self.start_listener()
        rospy.sleep(1)
        self.play_from_bag()

        rospy.on_shutdown(self.shutdown_hook)
    
    def shutdown_hook(self):
        if self.bag_proc:
            self.bag_proc.send_signal(signal.SIGINT)


if __name__ == '__main__':
    img_listener = MaskGenerator()
    img_listener.run()

    rospy.spin()
