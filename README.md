# Real-time Image Segmentation Client and Server solution for Meta SAM Model in ROS
Segment Anything Segmentation in ROS

## Server Branch
The server branch of this project is designed to alleviate GPU computation load when processing data stream on the client side. Sometimes it might be a good idea to let another local machine or even a cloud instance to host the SAM model and to make predictions. The server's main function is hosting the SAM model and processing segmentation API requests from the client ROS service through a python Flask server. 
More documentations about setting up the server could be found in the server branch. 

## Client Branch
The client branch consists of a ROS service allows direct input of point coordinates, point coordinate masks as well as bounding boxes in order for the model to make predictions. Additionally, it also supports direct input of segmentation masks to further narrow down the area you want to include in the mask. If you want to make a manual selection of points or bounding box, you can invoke the customizable GUI with a boolean flag which will generate the input parameters once you close it. 

An example of the GUI is as follows:</br>
<img width="594" alt="Screenshot 2023-11-15 071832" src="https://github.com/maxliu2001/SAM3D/assets/30473113/a47b40ef-8ef5-4e06-8d8f-36af60899a4c"></br>
More documentations about setting up the service could be found in the client branch. 
