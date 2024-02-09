# ros_sam integration with GUI selection

This sam_ros node is based on work done by:
@article{buchanan2023online,
  title={Online Estimation of Articulated Objects with Factor Graphs using Vision and Proprioceptive Sensing},
  author={Buchanan, Russell and R{\"o}fer, Adrian and Moura, Jo{\~a}o and Valada, Abhinav and Vijayakumar, Sethu},
  journal={arXiv preprint arXiv:2309.16343},
  year={2023}
}

The node called `sam3d` serves as the main package where serveral important features are supported:
#### Camera Automation Pipeline
The script `camera_automation.py` automatically triggers recording from the camera and records a ROS bag file called `test_cam_data.bag` in `/media` subdirectory within the sam3d package. \
media subdirectory may need to be created piror to start a new recording session. The script can be launched using `roslaunch camera_automation.launch`. 

#### Bag File Preprocessing Pipeline
The scripts `bag_publisher.py`, `listener.py`, `listener_depth.py` allows us to process the bag file by extracting videos to jpg files and depth maps. `bag_publisher.py` plays the ROS bag file while `listener.py` extracts each frame to `/media/test` subdirectory. `listener_depth.py` records the depth channel of each frame to `media/depthmap.npz`.
These scripts can be launched using `roslaunch bag_subscribe.launch`.

#### ros_sam Integration with Customizable GUI
After processing the bag file. We can now utilize ros_sam wrapper with customizable GUI to generate segmentation masks. We can run `roslaunch ros_sam.launch` to trigger `autoprocess.py` script in sam3d. The `autoprocess.py` script launches the `SAMClient` from `ros_sam` and generates masks. It also allows visualization of the generated mask. The image path may need to manually adjusted. The image path is the local path to the image in `sam3d/media` subdirectory.

#### How to use
- Download model checkpoint into ros_sam models directory
- setup sam3d_gui and download all requirement files (Working on a comprehensive main function)
- Run `rosrun ros_sam sam_node.py` (May need to init roscore beforehand)
- Run test file (GPU driver update needed)