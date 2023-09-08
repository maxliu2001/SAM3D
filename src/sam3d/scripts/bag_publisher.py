#!/usr/bin/env python
# publish color and depth topics from existing bag files
import rospy
import subprocess
import signal
import rospkg

class BagPublisher:
    def __init__(self):
        self.bagged_file_path = rospkg.RosPack().get_path('sam3d') + "/media/test_cam_data.bag"
        self.process = None
        self.pub = None

    def start_publisher(self):
        if not self.process:
            arg_li = ["rosbag", "play", self.bagged_file_path]
            self.bag_proc = subprocess.Popen(arg_li)
            rospy.loginfo("rosbag playing")
    
    def run(self):
        rospy.init_node('bagpub', anonymous=True)
        rospy.loginfo("start publishing from bag file")
        self.start_publisher()
        rospy.on_shutdown(self.shutdown_hook)
    
    def shutdown_hook(self):
        if self.process:
            self.process.send_signal(signal.SIGINT)

if __name__ == '__main__':
    pub = BagPublisher()
    pub.run()
    
