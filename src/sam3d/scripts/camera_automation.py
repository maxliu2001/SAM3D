#!/usr/bin/env python

import rospy
import subprocess
import signal
import rospkg

class CameraAutomation:
    def __init__(self):
        rospy.init_node('camera_automation')

        # Configure launch settings
        self.launch_file = rospkg.RosPack().get_path('sam3d') + "/launch/camera.launch"
        self.bag_file = rospkg.RosPack().get_path('sam3d') + "/media/test_cam_data.bag" # specify path to rosbag
        self.topics_to_record = ["/camera/color/image_raw", "/camera/depth/color/points", "/camera/aligned_depth_to_color/image_raw"]

        self.launch_proc = None
        self.record_proc = None
    
    def start_camera(self):
        cmd = ["roslaunch", self.launch_file]
        self.launch_proc = subprocess.Popen(cmd)
    
    def start_recording(self):
        cmd = ["rosbag", "record", "-O", self.bag_file] + self.topics_to_record
        self.record_proc = subprocess.Popen(cmd)
    
    def stop_recording(self):
        if self.record_proc:
            self.record_proc.send_signal(signal.SIGINT)
    
    def stop_camera(self):
        if self.launch_proc:
            self.launch_proc.send_signal(signal.SIGINT)

    def run(self):
        self.start_camera()
        rospy.sleep(5)
        self.start_recording()

        rospy.on_shutdown(self.shutdown_hook)
    
    def shutdown_hook(self):
        self.stop_recording()
        rospy.sleep(2)
        self.stop_camera()

if __name__=="__main__":
    cam_automator = CameraAutomation()
    cam_automator.run()

    rospy.spin()    
