# SAM3D
3D Reconstruction using Segment Anything Segmentation on ROS

## Main Branch
The main branch subscribes to the point cloud channel and the depth channel of Intel Realsense Camera. 
It then maps the 2D colored input to its corresponding depth to construct a colored point cloud. The point cloud is loaded onto Open3D to view. 

## Selector Branch
The selector branch contains a wrapper designed to simplify the process of selecting points / point masks as well as bounding boxes for 
Meta's Segment Anything to generate segmentation masks. In manual mode, the PyQt GUI is launched to allow users to manually select desired 
points/bounding boxes before loading the data into SAM's predictor. If manual mode is not selected, it would allow users to input SAM predictor 
parameters through parameters passed to the wrapper method `gen_seg`.</br>
An example of the GUI is as follows:</br>
<img width="594" alt="Screenshot 2023-11-15 071832" src="https://github.com/maxliu2001/SAM3D/assets/30473113/a47b40ef-8ef5-4e06-8d8f-36af60899a4c"></br>
The `test.py` script provides more insights into the usage of this wrapper.
